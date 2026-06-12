#pragma once
// mpc_shm.h — 共享内存布局，simulate 和 mpc_standalone 两边共用的头文件
//
// 通信协议（无锁，单写单读）：
//   写入方写完数据后 fetch_add(sequence, release)
//   读取方读到新 sequence 后 acquire 读取数据

#include <atomic>
#include <cstdint>

struct MpcShmLayout {
    // ========== 传感器区：simulate → MPC ==========
    std::atomic<uint64_t> sensor_sequence;   // 写入方每帧 +1
    double  simulation_time;                 // 仿真时间 (d->time)
    double  base_quat[4];                    // 躯干四元数 w,x,y,z
    double  base_position[3];                // 躯干位置 x,y,z
    double  base_angular_vel[3];             // 躯干角速度 wx,wy,wz
    double  base_linear_vel[3];              // 躯干线速度 vx,vy,vz
    double  joint_position[12];              // 12 关节角 (LF→RF→LH→RH)
    double  joint_velocity[12];              // 12 关节角速度
    uint8_t contact_flags[4];                // LF,RF,LH,RH 足端触地标志 (0/1)
    uint8_t mpc_active;                      // 1=sim 请求 MPC 接管

    // ========== 控制区：MPC → simulate ==========
    std::atomic<uint64_t> control_sequence;  // MPC 每写一次 +1
    std::atomic<bool>     control_ready;     // true=首帧有效控制已就绪
    double  joint_torque[12];                // WBC 前馈力矩 (Nm)
    double  joint_position_des[12];          // MPC 期望关节角
    double  joint_velocity_des[12];          // MPC 期望关节角速度
};
