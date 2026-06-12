// mpc_standalone.cpp — ROS-free MPC + WBC 独立进程
// 通过共享内存与 simulate 通信，替代原有的 ROS2 MPC node
//
// 用法: mpc_standalone <robot_name> <task.info> <urdf> <reference.info> <simulation.info>

#include "motion_control/legged_interface/LeggedRobotInterface.h"
#include "motion_control/legged_wbc/WeightedWbc.h"
#include "motion_control/common/Types.h"
#include "motion_control/gait/MotionPhaseDefinition.h"
#include "motion_control/legged_estimation/StateEstimateBase.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>

// ---------- 共享内存布局（与 simulate/mpc_shm.h 保持一致）----------

struct MpcShmLayout {
    std::atomic<uint64_t> sensor_sequence;
    double  simulation_time;
    double  base_quat[4];
    double  base_position[3];
    double  base_angular_vel[3];
    double  base_linear_vel[3];
    double  joint_position[12];
    double  joint_velocity[12];
    uint8_t contact_flags[4];
    uint8_t mpc_active;

    std::atomic<uint64_t> control_sequence;
    std::atomic<bool>     control_ready;
    double  joint_torque[12];
    double  joint_position_des[12];
    double  joint_velocity_des[12];
};

static volatile bool g_running = true;
static void sigHandler(int) { g_running = false; }

// ---------- 四元数 → ZYX 欧拉角 ----------
inline vector3_t quatToZyx(const Eigen::Quaternion<ocs2::scalar_t>& q) {
    vector3_t zyx;
    ocs2::scalar_t as = std::min(-2.0 * (q.x() * q.z() - q.w() * q.y()), 0.99999);
    zyx(0) = std::atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
                        q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                        q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    return zyx;
}

// ---------- 从共享内存构建 36D measuredRbdState ----------
void buildMeasuredRbdState(MpcShmLayout* shm, ocs2::vector_t& rbdState) {
    Eigen::Quaternion<ocs2::scalar_t> quat(shm->base_quat[0], shm->base_quat[1],
                                            shm->base_quat[2], shm->base_quat[3]);
    vector3_t euler = quatToZyx(quat);

    rbdState.setZero(36);
    rbdState.segment<3>(0) = euler;
    rbdState.segment<3>(3) = Eigen::Map<const vector3_t>(shm->base_position);
    for (int i = 0; i < 12; i++) rbdState(6 + i) = shm->joint_position[i];
    rbdState.segment<3>(18) = Eigen::Map<const vector3_t>(shm->base_angular_vel);
    rbdState.segment<3>(21) = Eigen::Map<const vector3_t>(shm->base_linear_vel);
    for (int i = 0; i < 12; i++) rbdState(24 + i) = shm->joint_velocity[i];
}

// ======================================================================
int main(int argc, char** argv) {
    if (argc != 6) {
        std::fprintf(stderr, "用法: %s <robot_name> <task.info> <urdf> <reference.info> <simulation.info>\n", argv[0]);
        return 1;
    }
    std::string robotName   = argv[1];
    std::string taskFile    = argv[2];
    std::string urdfFile    = argv[3];
    std::string referenceFile = argv[4];
    std::string simulatorFile = argv[5];

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // ---- 1. 挂载共享内存 ----
    int fd = shm_open("/mpc_control_shm", O_RDWR, 0666);
    if (fd < 0) {
        std::fprintf(stderr, "[MPC] shm_open failed: %s\n", std::strerror(errno));
        return 1;
    }
    void* ptr = mmap(nullptr, sizeof(MpcShmLayout), PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        std::fprintf(stderr, "[MPC] mmap failed: %s\n", std::strerror(errno));
        return 1;
    }
    auto* shm = static_cast<MpcShmLayout*>(ptr);
    std::printf("[MPC] 共享内存已挂载\n");

    // ---- 2. 等待 simulator 写入首帧传感器数据 ----
    std::printf("[MPC] 等待传感器数据...\n");
    {
        uint64_t lastSeq = shm->sensor_sequence.load(std::memory_order_acquire);
        auto start = std::chrono::steady_clock::now();
        while (g_running) {
            uint64_t seq = shm->sensor_sequence.load(std::memory_order_acquire);
            if (seq != lastSeq && seq > 0) break;
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > std::chrono::seconds(30)) {
                std::fprintf(stderr, "[MPC] 超时：30 秒内未收到传感器数据\n");
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::printf("[MPC] 收到首帧数据 t=%.3f\n", shm->simulation_time);

    // ---- 3. 创建 LeggedRobotInterface（加载 config + URDF + Pinocchio）----
    std::printf("[MPC] 加载模型和配置...\n");
    auto leggedInterface = std::make_shared<LeggedRobotInterface>(taskFile, urdfFile, referenceFile);

    // ---- 4. 创建 CentroidalModelRbdConversions ----
    auto rbdConversions = std::make_shared<ocs2::CentroidalModelRbdConversions>(
        leggedInterface->getPinocchioInterface(),
        leggedInterface->getCentroidalModelInfo());

    // ---- 5. 创建 SqpMpc ----
    auto mpc = std::make_shared<ocs2::SqpMpc>(
        leggedInterface->mpcSettings(),
        leggedInterface->sqpSettings(),
        leggedInterface->getOptimalControlProblem(),
        leggedInterface->getInitializer());

    // 设置 ReferenceManager（用 ROS-free 的 SwitchedModelReferenceManager）
    mpc->getSolverPtr()->setReferenceManager(
        leggedInterface->getSwitchedModelReferenceManagerPtr());

    std::printf("[MPC] SqpMpc 已创建\n");

    // ---- 6. 创建 MPC_MRT_Interface ----
    auto mpcMrtInterface = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc);
    mpcMrtInterface->initRollout(&leggedInterface->getRollout());

    // ---- 7. 创建 WBC ----
    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface->getCentroidalModelInfo());
    auto eeKinematics = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(
        leggedInterface->getPinocchioInterface(),
        pinocchioMapping,
        leggedInterface->modelSettings().contactNames3DoF);
    auto wbc = std::make_shared<WeightedWbc>(
        leggedInterface->getPinocchioInterface(),
        leggedInterface->getCentroidalModelInfo(),
        *eeKinematics);
    wbc->loadTasksSetting(taskFile, false);
    std::printf("[MPC] WBC 已创建\n");

    // ---- 8. 读取控制频率 ----
    int mpcFreq = 40, wbcFreq = 500;
    ocs2::loadData::loadCppDataType(simulatorFile, "controller.mpc_control_frequency", mpcFreq);
    ocs2::loadData::loadCppDataType(simulatorFile, "controller.wbc_control_frequency", wbcFreq);
    int freqRatio = wbcFreq / mpcFreq;  // 12 或 13
    double wbcPeriod = 1.0 / wbcFreq;
    std::printf("[MPC] MPC=%dHz  WBC=%dHz  ratio=%d\n", mpcFreq, wbcFreq, freqRatio);

    // ---- 9. 构建初始策略 ----
    std::printf("[MPC] 构建初始策略...\n");
    {
        ocs2::SystemObservation obs;
        obs.time = shm->simulation_time;
        obs.state.setZero(leggedInterface->getCentroidalModelInfo().stateDim);
        obs.input.setZero(leggedInterface->getCentroidalModelInfo().inputDim);
        obs.mode = ModeNumber::STANCE;

        ocs2::vector_t measuredRbd(36);
        buildMeasuredRbdState(shm, measuredRbd);
        obs.state = rbdConversions->computeCentroidalStateFromRbdModel(measuredRbd);

        mpcMrtInterface->setCurrentObservation(obs);
        ocs2::TargetTrajectories targets({obs.time}, {obs.state}, {obs.input});
        mpcMrtInterface->getReferenceManager().setTargetTrajectories(targets);

        ocs2::benchmark::RepeatedTimer initTimer;
        while (!mpcMrtInterface->initialPolicyReceived() && g_running) {
            initTimer.startTimer();
            mpcMrtInterface->advanceMpc();
            initTimer.endTimer();
            double ms = initTimer.getLastIntervalInMilliseconds();
            std::printf("  advanceMpc: %.1f ms\n", ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(1000.0 / leggedInterface->mpcSettings().mrtDesiredFrequency_)));
        }
    }
    std::printf("[MPC] 初始策略就绪\n");

    // 首次 evaluatePolicy + WBC
    ocs2::SystemObservation obs;
    obs.time = shm->simulation_time;
    ocs2::vector_t measuredRbd(36);
    buildMeasuredRbdState(shm, measuredRbd);
    obs.state = rbdConversions->computeCentroidalStateFromRbdModel(measuredRbd);
    obs.input.setZero(leggedInterface->getCentroidalModelInfo().inputDim);

    contact_flag_t contactFlag{};
    for (int i = 0; i < 4; i++) contactFlag[i] = shm->contact_flags[i];
    obs.mode = stanceLeg2ModeNumber(contactFlag);

    // ---- 10. 写首帧控制指令，设置 control_ready = true ----
    {
        mpcMrtInterface->setCurrentObservation(obs);
        mpcMrtInterface->updatePolicy();

        ocs2::vector_t optState, optInput;
        size_t plannedMode = 0;
        mpcMrtInterface->evaluatePolicy(obs.time, obs.state, optState, optInput, plannedMode);

        ocs2::vector_t x = wbc->update(optState, optInput, measuredRbd, plannedMode, wbcPeriod);
        ocs2::vector_t torque = x.tail(12);
        ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optState, leggedInterface->getCentroidalModelInfo());
        ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optInput, leggedInterface->getCentroidalModelInfo());

        // 首帧也做符号修正（RF_HAA=3, LH_HAA=6）
        for (int i = 0; i < 12; i++) {
            double sign = (i == 3 || i == 6) ? -1.0 : 1.0;
            shm->joint_torque[i]        = sign * torque(i);
            shm->joint_position_des[i]  = posDes(i);
            shm->joint_velocity_des[i]  = velDes(i);
        }
        shm->control_sequence.fetch_add(1, std::memory_order_release);
        shm->control_ready.store(true, std::memory_order_release);
        shm->mpc_active = 1;
        std::printf("[MPC] control_ready = true\n");
    }

    // ---- 11. 主循环：500Hz WBC + 40Hz MPC ----
    std::printf("[MPC] 进入主控制循环\n");
    ocs2::benchmark::RepeatedTimer mpcTimer, wbcTimer;
    int mpcCount = 0;
    ocs2::scalar_t yawLast = 0;
    uint64_t lastSensorSeq = 0;
    int diagCount = 0;  // 前 5 帧打印诊断信息

    auto loopStart = std::chrono::steady_clock::now();
    long long step = 0;

    while (g_running) {
        step++;
        auto targetTime = loopStart + std::chrono::microseconds(static_cast<long long>(step * wbcPeriod * 1e6));

        // 等待新传感器数据
        while (g_running) {
            uint64_t seq = shm->sensor_sequence.load(std::memory_order_acquire);
            if (seq != lastSensorSeq) { lastSensorSeq = seq; break; }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            if (std::chrono::steady_clock::now() > targetTime + std::chrono::milliseconds(2)) break;
        }

        // 构建状态
        buildMeasuredRbdState(shm, measuredRbd);

        if (diagCount < 5) {
            std::printf("\n=== 诊断帧 %d (t=%.3f) ===\n", diagCount, shm->simulation_time);
            std::printf("base: quat=(%.4f,%.4f,%.4f,%.4f) pos=(%.4f,%.4f,%.4f)\n",
                        shm->base_quat[0], shm->base_quat[1], shm->base_quat[2], shm->base_quat[3],
                        shm->base_position[0], shm->base_position[1], shm->base_position[2]);
            std::printf("contact: LF=%d RF=%d LH=%d RH=%d\n",
                        shm->contact_flags[0], shm->contact_flags[1], shm->contact_flags[2], shm->contact_flags[3]);
            std::printf("           HAA       HFE       KFE       vel(HFE,KFE)   torque\n");
            const char* leg[4] = {"LF", "RF", "LH", "RH"};
            for (int legi = 0; legi < 4; legi++) {
                int i0 = legi * 3;
                std::printf("  %s: %+7.4f  %+7.4f  %+7.4f   %+6.2f %+6.2f    %+6.2f %+6.2f %+6.2f\n",
                    leg[legi],
                    shm->joint_position[i0], shm->joint_position[i0+1], shm->joint_position[i0+2],
                    shm->joint_velocity[i0+1], shm->joint_velocity[i0+2],
                    shm->joint_torque[i0], shm->joint_torque[i0+1], shm->joint_torque[i0+2]);
            }
            diagCount++;
        }
        obs.time = shm->simulation_time;
        obs.state = rbdConversions->computeCentroidalStateFromRbdModel(measuredRbd);
        yawLast = obs.state(9);  // 简化：不处理 yaw 解绕
        obs.input.setZero(leggedInterface->getCentroidalModelInfo().inputDim);
        for (int i = 0; i < 4; i++) contactFlag[i] = shm->contact_flags[i];
        obs.mode = stanceLeg2ModeNumber(contactFlag);

        mpcMrtInterface->setCurrentObservation(obs);

        // MPC advance（每 13 次 WBC 跑一次）
        mpcCount++;
        if (mpcCount >= freqRatio) {
            mpcCount = 0;
            mpcTimer.startTimer();
            mpcMrtInterface->advanceMpc();
            mpcTimer.endTimer();
        }

        // evaluatePolicy + WBC
        mpcMrtInterface->updatePolicy();
        ocs2::vector_t optState, optInput;
        size_t plannedMode = 0;
        mpcMrtInterface->evaluatePolicy(obs.time, obs.state, optState, optInput, plannedMode);

        wbcTimer.startTimer();
        ocs2::vector_t x = wbc->update(optState, optInput, measuredRbd, plannedMode, wbcPeriod);
        wbcTimer.endTimer();

        // 诊断：打印 WBC 足端力（前 5 帧）
        if (diagCount < 5 && diagCount >= 0) {
            ocs2::vector_t footForces = x.segment<12>(18);  // Fx,Fy,Fz × 4
            double totalFz = footForces(2) + footForces(5) + footForces(8) + footForces(11);
            std::printf("--- WBC 足端力 (N) ---\n");
            std::printf("  LF: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(0), footForces(1), footForces(2));
            std::printf("  RF: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(3), footForces(4), footForces(5));
            std::printf("  LH: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(6), footForces(7), footForces(8));
            std::printf("  RH: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(9), footForces(10), footForces(11));
            std::printf("  总Fz=%.2f N (体重=%.2f N)\n", totalFz, 11.67 * 9.81);
            ocs2::vector_t posDesCheck = ocs2::centroidal_model::getJointAngles(optState, leggedInterface->getCentroidalModelInfo());
            std::printf("  MPC期望关节角 vs 实际:\n");
            const char* jn[12] = {"LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                                  "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"};
            for (int j = 0; j < 12; j++) {
                std::printf("    %s: des=%+7.4f  cur=%+7.4f  diff=%+7.4f\n",
                    jn[j], posDesCheck(j), shm->joint_position[j],
                    posDesCheck(j) - shm->joint_position[j]);
            }
        }

        ocs2::vector_t torque = x.tail(12);
        ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optState, leggedInterface->getCentroidalModelInfo());
        ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optInput, leggedInterface->getCentroidalModelInfo());

        // 写共享内存（RF_HAA=3, LH_HAA=6 力矩取反：URDF/MJCF 轴正向不一致）
        for (int i = 0; i < 12; i++) {
            double sign = (i == 3 || i == 6) ? -1.0 : 1.0;
            shm->joint_torque[i]        = sign * torque(i);
            shm->joint_position_des[i]  = posDes(i);
            shm->joint_velocity_des[i]  = velDes(i);
        }
        shm->control_sequence.fetch_add(1, std::memory_order_release);

        // 休眠以维持 WBC 频率
        auto now = std::chrono::steady_clock::now();
        if (now < targetTime) {
            std::this_thread::sleep_until(targetTime);
        }
    }

    std::printf("[MPC] 退出\n");
    return 0;
}
