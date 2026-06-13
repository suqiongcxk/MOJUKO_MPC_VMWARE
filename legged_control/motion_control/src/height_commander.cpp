// height_commander.cpp — 终端输入目标 COM 高度，通过共享内存发给 MPC
//
// 用法: height_commander
//   启动后输入高度值 (m)，回车确认

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

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

int main() {
    int fd = shm_open("/mpc_control_shm", O_RDWR, 0666);
    if (fd < 0) {
        std::fprintf(stderr, "无法打开共享内存: %s\n请先启动仿真器和 MPC\n", std::strerror(errno));
        return 1;
    }
    void* ptr = mmap(nullptr, sizeof(MpcShmLayout), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        std::fprintf(stderr, "mmap 失败\n");
        return 1;
    }
    auto* shm = static_cast<MpcShmLayout*>(ptr);

    std::printf("=== 高度控制终端 ===\n");
    std::printf("输入目标 COM 高度 (m)，当前: %.3f\n", shm->target_com_height);
    std::printf("Ctrl+D 退出\n\n");

    char line[256];
    while (std::fgets(line, sizeof(line), stdin)) {
        double h = std::strtod(line, nullptr);
        if (h <= 0 || h > 1.0) {
            std::printf("无效高度: %.3f，请输入 0~1.0 之间的值\n", h);
            continue;
        }
        shm->target_com_height = h;
        shm->height_updated = 1;
        std::printf("→ 目标高度已设为 %.3f m\n", h);
    }
    std::printf("\n退出\n");
    return 0;
}
