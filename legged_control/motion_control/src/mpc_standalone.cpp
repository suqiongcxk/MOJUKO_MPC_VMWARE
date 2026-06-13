// mpc_standalone.cpp — ROS-free MPC + WBC 独立进程
// 通过共享内存与 simulate 通信，替代原有的 ROS2 MPC node
//
// 用法: mpc_standalone <robot_name> <task.info> <urdf> <reference.info> <simulation.info>

#include "motion_control/legged_interface/LeggedRobotInterface.h"
#include "motion_control/legged_wbc/WeightedWbc.h"
#include "motion_control/common/Types.h"
#include "motion_control/gait/MotionPhaseDefinition.h"
#include "motion_control/legged_estimation/StateEstimateBase.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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
#include <cstdarg>

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
    double  target_com_height;
    uint8_t height_updated;

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

    // shm 关节顺序: LF→RF→LH→RH, Pinocchio 模型顺序: LF→LH→RF→RH
    // 需要把 RF(3-5) 和 LH(6-8) 交换
    static const int shm2model[12] = {0,1,2,  6,7,8,  3,4,5,  9,10,11};  // shm idx → model idx
    rbdState.setZero(36);
    rbdState.segment<3>(0) = euler;
    rbdState.segment<3>(3) = Eigen::Map<const vector3_t>(shm->base_position);
    for (int i = 0; i < 12; i++) rbdState(6 + shm2model[i]) = shm->joint_position[i];
    rbdState.segment<3>(18) = Eigen::Map<const vector3_t>(shm->base_angular_vel);
    rbdState.segment<3>(21) = Eigen::Map<const vector3_t>(shm->base_linear_vel);
    for (int i = 0; i < 12; i++) rbdState(24 + shm2model[i]) = shm->joint_velocity[i];
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

    // 安静模式：OCS2 时间警告刷屏，重定向到文件
    std::freopen("/home/cxk/creeper_ws/log/mpc_stderr.log", "w", stderr);

    // 初始策略将在主循环首次迭代中构建

    // ---- 11. 主循环：500Hz WBC + 40Hz MPC ----
    std::printf("[MPC] 进入主控制循环\n");
    ocs2::benchmark::RepeatedTimer mpcTimer, wbcTimer;
    int mpcCount = 0;
    ocs2::scalar_t yawLast = 0;
    uint64_t lastSensorSeq = 0;
    int diagCount = 0;  // 前 20 帧打印诊断信息

    // CSV 日志
    FILE* csv = std::fopen("/home/cxk/creeper_ws/log/mpc_diag.csv", "w");
    if (csv) {
      std::fprintf(csv, "time,nWsr,"
        "LF_HAA_posDes,LF_HFE_posDes,LF_KFE_posDes,"
        "RF_HAA_posDes,RF_HFE_posDes,RF_KFE_posDes,"
        "LH_HAA_posDes,LH_HFE_posDes,LH_KFE_posDes,"
        "RH_HAA_posDes,RH_HFE_posDes,RH_KFE_posDes,"
        "LF_HAA_tau,LF_HFE_tau,LF_KFE_tau,"
        "RF_HAA_tau,RF_HFE_tau,RF_KFE_tau,"
        "LH_HAA_tau,LH_HFE_tau,LH_KFE_tau,"
        "RH_HAA_tau,RH_HFE_tau,RH_KFE_tau,"
        "LF_Fx,LF_Fy,LF_Fz,RF_Fx,RF_Fy,RF_Fz,"
        "LH_Fx,LH_Fy,LH_Fz,RH_Fx,RH_Fy,RH_Fz,"
        // MPC 期望足端力 (optInput, 模型顺序 LF→RF→LH→RH)
        "mpc_F_LFx,mpc_F_LFy,mpc_F_LFz,mpc_F_RFx,mpc_F_RFy,mpc_F_RFz,"
        "mpc_F_LHx,mpc_F_LHy,mpc_F_LHz,mpc_F_RHx,mpc_F_RHy,mpc_F_RHz,"
        // MPC 内部状态
        "meas_com_x,opt_com_x,"
        "ref_p_base_z,meas_p_base_z,opt_p_base_z,"
        "ref_joint_LF_HFE,ref_joint_LF_KFE,ref_joint_LH_HFE,ref_joint_LH_KFE,"
        "meas_joint_LF_HFE,meas_joint_LF_KFE,meas_joint_LH_HFE,meas_joint_LH_KFE\n");
    }

    bool firstIteration = true;
    ocs2::vector_t measuredRbd(36);
    ocs2::SystemObservation obs;
    contact_flag_t contactFlag{};

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

        // 检查高度指令更新
        if (shm->height_updated) {
            double newHeight = shm->target_com_height;
            std::printf("[MPC] 目标高度更新: %.3f m\n", newHeight);
            ocs2::TargetTrajectories targets = mpcMrtInterface->getReferenceManager().getTargetTrajectories();
            for (auto& state : targets.stateTrajectory)
                state(8) = newHeight;
            mpcMrtInterface->getReferenceManager().setTargetTrajectories(targets);
            shm->height_updated = 0;
        }

        // 构建状态
        buildMeasuredRbdState(shm, measuredRbd);

        if (diagCount < 20) {
            std::printf("\n=== 诊断帧 %d (t=%.3f) nWsr=%d ===\n", diagCount, shm->simulation_time, wbc->getLastNwsr());
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

        // 首次迭代：构建初始策略 + 设置 control_ready
        if (firstIteration) {
            ocs2::TargetTrajectories targets({obs.time}, {obs.state}, {obs.input});
            mpcMrtInterface->getReferenceManager().setTargetTrajectories(targets);

            mpcMrtInterface->advanceMpc();
            mpcMrtInterface->updatePolicy();

            ocs2::vector_t optState, optInput;
            size_t plannedMode = 0;
            mpcMrtInterface->evaluatePolicy(obs.time, obs.state, optState, optInput, plannedMode);

            ocs2::vector_t x = wbc->update(optState, optInput, measuredRbd, plannedMode, wbcPeriod);
            ocs2::vector_t torque = x.tail(12);
            ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optState, leggedInterface->getCentroidalModelInfo());
            ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optInput, leggedInterface->getCentroidalModelInfo());

            static const int model2shm_first[12] = {0,1,2, 6,7,8, 3,4,5, 9,10,11};
            for (int i = 0; i < 12; i++) {
                shm->joint_torque[model2shm_first[i]]        = torque(i);
                shm->joint_position_des[model2shm_first[i]]  = posDes(i);
                shm->joint_velocity_des[model2shm_first[i]]  = velDes(i);
            }
            shm->control_sequence.fetch_add(1, std::memory_order_release);
            shm->control_ready.store(true, std::memory_order_release);
            shm->mpc_active = 1;

            std::printf("[MPC] 初始策略就绪 (t=%.3f), control_ready = true\n", obs.time);

            // 打印初始参考轨迹
            {
                const auto& targets_ref = mpcMrtInterface->getReferenceManager().getTargetTrajectories();
                std::printf("[MPC] === 初始参考轨迹 (%zu 个点) ===\n", targets_ref.stateTrajectory.size());
                std::printf("      time  p_base_z  LF_HFE  LF_KFE  LH_HFE  LH_KFE\n");
                for (size_t k = 0; k < targets_ref.stateTrajectory.size(); k++) {
                    const auto& s = targets_ref.stateTrajectory[k];
                    std::printf("  %7.3f  %8.4f  %6.4f  %7.4f  %6.4f  %7.4f\n",
                        targets_ref.timeTrajectory[k], s(8),
                        s(12+1), s(12+2), s(12+4), s(12+5));
                }
                std::printf("[MPC] ==============================\n");
            }

            firstIteration = false;
            mpcCount = 0;
            lastSensorSeq = shm->sensor_sequence.load(std::memory_order_acquire);

            // 保持 WBC 频率
            auto now = std::chrono::steady_clock::now();
            if (now < targetTime) {
                std::this_thread::sleep_until(targetTime);
            }
            continue;
        }

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

        ocs2::vector_t footForces = x.segment<12>(18);

        // 对称性诊断：用完美对称的实测状态再跑一次 WBC，对比力矩
        if (diagCount == 1) {
          ocs2::vector_t symRbd = measuredRbd;
          // 位置：清零所有不对称分量
          symRbd.segment<3>(0).setZero();            // euler = 0
          symRbd(3) = 0.0; symRbd(4) = 0.0;         // base x,y = 0
          for (int leg = 0; leg < 4; leg++) symRbd(6 + leg*3) = 0.0;  // HAA = 0
          for (int j : {1, 2}) {
            double avg = 0.0;
            for (int leg = 0; leg < 4; leg++) avg += measuredRbd(6 + leg*3 + j);
            avg /= 4.0;
            for (int leg = 0; leg < 4; leg++) symRbd(6 + leg*3 + j) = avg;
          }
          // 速度：全部清零
          symRbd.segment<3>(18).setZero();            // base angular vel = 0
          symRbd.segment<3>(21).setZero();            // base linear vel = 0
          for (int i = 0; i < 12; i++) symRbd(24 + i) = 0.0;  // joint vel = 0

          ocs2::vector_t xSym = wbc->update(optState, optInput, symRbd, plannedMode, wbcPeriod);
          ocs2::vector_t tauSym = xSym.tail(12);
          ocs2::vector_t torque = x.tail(12);
          ocs2::vector_t symForce = xSym.segment<12>(18);

          FILE* df = std::fopen("/home/cxk/creeper_ws/log/symmetry_diag.txt", "w");
          auto out = [&](const char* fmt, ...) {
            va_list ap; va_start(ap, fmt);
            std::vfprintf(stdout, fmt, ap); va_end(ap);
            if (df) { va_start(ap, fmt); std::vfprintf(df, fmt, ap); va_end(ap); }
          };
          out("\n=== 对称性诊断 (位置+速度全对称) ===\n");
          // 模型顺序 LF(0-2), LH(3-5), RF(6-8), RH(9-11)
          out("  实测 HAA=[%.4f,%.4f,%.4f,%.4f] euler=[%.4f,%.4f,%.4f]\n",
            measuredRbd(6), measuredRbd(12), measuredRbd(9), measuredRbd(15),
            measuredRbd(0), measuredRbd(1), measuredRbd(2));
          out("  实测基座速度: ang=[%.4f,%.4f,%.4f] lin=[%.4f,%.4f,%.4f]\n",
            measuredRbd(18), measuredRbd(19), measuredRbd(20),
            measuredRbd(21), measuredRbd(22), measuredRbd(23));
          out("  实测 HFE(LF,RF,LH,RH)=[%.4f,%.4f,%.4f,%.4f] KFE=[%.4f,%.4f,%.4f,%.4f]\n",
            measuredRbd(7), measuredRbd(13), measuredRbd(10), measuredRbd(16),
            measuredRbd(8), measuredRbd(14), measuredRbd(11), measuredRbd(17));
          out("  对称 HFE=%.4f KFE=%.4f\n", symRbd(7), symRbd(8));
          out("%8s  %10s  %10s  %10s\n", "关节", "真实力矩", "对称力矩", "差值");
          const char* jn[12] = {"LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                                "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"};
          for (int j = 0; j < 12; j++) {
            out("%8s  %+10.4f  %+10.4f  %+10.4f\n", jn[j], torque(j), tauSym(j), torque(j)-tauSym(j));
          }
          out("--- 对称状态足端力 Fz ---\n");
          out("  LF=%.2f  RF=%.2f  LH=%.2f  RH=%.2f\n", symForce(2), symForce(5), symForce(8), symForce(11));
          // Pinocchio FK: 对称状态下的足端位置
          {
            auto& model = leggedInterface->getPinocchioInterface().getModel();
            auto& data = leggedInterface->getPinocchioInterface().getData();
            const auto& info = leggedInterface->getCentroidalModelInfo();
            // Build Pinocchio q from symRbd
            ocs2::vector_t q(info.generalizedCoordinatesNum);
            q.head<3>() = symRbd.segment<3>(3);  // base pos
            q.segment<3>(3) = symRbd.head<3>();   // base orientation
            q.tail(info.actuatedDofNum) = symRbd.segment(6, info.actuatedDofNum);
            pinocchio::forwardKinematics(model, data, q);
            pinocchio::updateFramePlacements(model, data);
            out("--- Pinocchio FK 足端位置 (对称状态) ---\n");
            const char* footNames[4] = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
            for (size_t i = 0; i < info.endEffectorFrameIndices.size() && i < 4; i++) {
              auto frameIdx = info.endEffectorFrameIndices[i];
              const auto& pos = data.oMf[frameIdx].translation();
              out("  %s: [%.4f, %.4f, %.4f]\n",
                footNames[i], pos.x(), pos.y(), pos.z());
            }
            // KFE Jacobian: 先打印 Pinocchio 模型关节顺序，确认索引
            out("--- Pinocchio 模型关节名及索引 ---\n");
            for (int j = 0; j < model.njoints && j < 25; j++) {
              out("  joint[%d]: %s\n", j, model.names[j].c_str());
            }
            // Jacobian 分析
            pinocchio::computeJointJacobians(model, data, q);
            out("--- 各足端 KFE Jacobian (Jz) - 检查所有关节列 ---\n");
            for (int leg = 0; leg < 4; leg++) {
              auto frameIdx = info.endEffectorFrameIndices[leg];
              Eigen::Matrix<ocs2::scalar_t, 6, Eigen::Dynamic> jac(6, model.nv);
              jac.setZero();
              pinocchio::getFrameJacobian(model, data, frameIdx, pinocchio::LOCAL_WORLD_ALIGNED, jac);
              out("  %s Jz(2,col): ", footNames[leg]);
              for (int col = 6; col < model.nv; col++) {
                if (std::abs(jac(2, col)) > 0.001)
                  out("%d=%.4f ", col, jac(2, col));
              }
              out("\n");
              // Also check the row for z component
            }
            // Mass matrix diagonal for KFE joints
            out("--- 质量矩阵 M 对角元 (KFE 关节, 模型顺序) ---\n");
            out("  LF_KFE: M(8,8)=%.6f\n", data.M(8,8));
            out("  LH_KFE: M(11,11)=%.6f\n", data.M(11,11));
            out("  RF_KFE: M(14,14)=%.6f\n", data.M(14,14));
            out("  RH_KFE: M(17,17)=%.6f\n", data.M(17,17));
          }
          if (df) std::fclose(df);
          out("(已保存到 ~/creeper_ws/log/symmetry_diag.txt)\n\n");
          std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // 诊断：打印 WBC 足端力（前 20 帧）
        if (diagCount < 20 && diagCount >= 0) {
            double totalFz = footForces(2) + footForces(5) + footForces(8) + footForces(11);
            std::printf("--- WBC 足端力 (N) ---\n");
            std::printf("  LF: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(0), footForces(1), footForces(2));
            std::printf("  RF: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(3), footForces(4), footForces(5));
            std::printf("  LH: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(6), footForces(7), footForces(8));
            std::printf("  RH: Fx=%7.2f Fy=%7.2f Fz=%7.2f\n", footForces(9), footForces(10), footForces(11));
            std::printf("  总Fz=%.2f N (体重=%.2f N)\n", totalFz, 11.67 * 9.81);
            std::printf("  MPC期望关节角 vs 实际:\n");
            const char* jn[12] = {"LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                                  "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"};
            for (int j = 0; j < 12; j++) {
                std::printf("    %s: des=%+7.4f  cur=%+7.4f  diff=%+7.4f\n",
                    jn[j], shm->joint_position_des[j], shm->joint_position[j],
                    shm->joint_position_des[j] - shm->joint_position[j]);
            }
        }

        ocs2::vector_t torque = x.tail(12);
        ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optState, leggedInterface->getCentroidalModelInfo());
        ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optInput, leggedInterface->getCentroidalModelInfo());

        // CSV 日志
        if (csv) {
          const auto& targets = mpcMrtInterface->getReferenceManager().getTargetTrajectories();
          double ref_z = targets.stateTrajectory[0](8);
          // 模型顺序: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
          // 质心状态关节从12开始: LF_HFE=13, LF_KFE=14, LH_HFE=16, LH_KFE=17
          double ref_lf_hfe = targets.stateTrajectory[0](12+1);
          double ref_lf_kfe = targets.stateTrajectory[0](12+2);
          double ref_lh_hfe = targets.stateTrajectory[0](12+4);
          double ref_lh_kfe = targets.stateTrajectory[0](12+5);

          std::fprintf(csv, "%.4f,%d,", shm->simulation_time, wbc->getLastNwsr());
          for (int i = 0; i < 12; i++) std::fprintf(csv, "%.4f,", shm->joint_position_des[i]);
          for (int i = 0; i < 12; i++) std::fprintf(csv, "%.4f,", shm->joint_torque[i]);
          for (int i = 0; i < 11; i++) std::fprintf(csv, "%.4f,", footForces(i));
          std::fprintf(csv, "%.4f,", footForces(11));
          // MPC 期望足端力
          for (int i = 0; i < 11; i++) std::fprintf(csv, "%.4f,", optInput(i));
          std::fprintf(csv, "%.4f,", optInput(11));
          // MPC 内部状态
          std::fprintf(csv, "%.4f,%.4f,", obs.state(6), optState(6));
          std::fprintf(csv, "%.4f,%.4f,%.4f,", ref_z, obs.state(8), optState(8));
          std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,", ref_lf_hfe, ref_lf_kfe, ref_lh_hfe, ref_lh_kfe);
          // measuredRbd 在模型顺序: LF_HFE=7, LF_KFE=8, LH_HFE=10, LH_KFE=11
          std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f\n",
            measuredRbd(6+1), measuredRbd(6+2), measuredRbd(6+4), measuredRbd(6+5));
        }

        // 模型顺序(LF→LH→RF→RH)→shm顺序(LF→RF→LH→RH)
        static const int model2shm[12] = {0,1,2, 6,7,8, 3,4,5, 9,10,11};
        for (int i = 0; i < 12; i++) {
            shm->joint_torque[model2shm[i]]        = torque(i);
            shm->joint_position_des[model2shm[i]]  = posDes(i);
            shm->joint_velocity_des[model2shm[i]]  = velDes(i);
        }
        shm->control_sequence.fetch_add(1, std::memory_order_release);

        // 每 250 帧 (0.5s) 打印 MPC 内部状态对比
        if (step % 250 == 0) {
          const auto& targets = mpcMrtInterface->getReferenceManager().getTargetTrajectories();
          double ref_z = targets.stateTrajectory[0](8);
          std::printf("\n[MPC t=%.3f step=%lld] ref_z=%.4f meas_z=%.4f opt_z=%.4f\n",
            shm->simulation_time, step, ref_z, obs.state(8), optState(8));
          std::printf("  Joint   ref     meas    opt(MPC) opt-ref\n");
          const char* jn_sel[4] = {"LF_HFE","LF_KFE","LH_HFE","LH_KFE"};
          int ji[4] = {13, 14, 16, 17};  // centroidal (模型顺序: LF→LH→RF→RH)
          int mji[4] = {7, 8, 10, 11};   // measuredRbd (model order, 6+offset)
          for (int k = 0; k < 4; k++) {
            std::printf("  %-7s %+7.4f %+7.4f %+7.4f %+8.4f\n",
              jn_sel[k],
              targets.stateTrajectory[0](ji[k]),  // ref
              measuredRbd(mji[k]),                 // meas (MuJoCo)
              optState(ji[k]),                     // MPC 优化结果
              optState(ji[k]) - targets.stateTrajectory[0](ji[k]));  // MPC偏离ref的量
          }
        }

        // 休眠以维持 WBC 频率
        auto now = std::chrono::steady_clock::now();
        if (now < targetTime) {
            std::this_thread::sleep_until(targetTime);
        }
    }

    if (csv) std::fclose(csv);
    std::printf("[MPC] 退出\n");
    return 0;
}
