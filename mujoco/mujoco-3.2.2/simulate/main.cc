// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// 常量
const double syncMisalign = 0.1;        // 最大允许失步时间（仿真秒）
const double simRefreshFraction = 0.7;  // 渲染刷新周期中用于仿真的比例
const int kErrorLength = 1024;          // 加载错误字符串长度

// 模型和数据
mjModel* m = nullptr;
mjData* d = nullptr;

using Seconds = std::chrono::duration<double>;


//---------------------------------------- 插件处理 -----------------------------------------

// 返回当前可执行文件所在目录的路径
// 用于确定自动加载的插件库的位置
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath 太小，扩大后重试
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path 不是符号链接，直接使用
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath 太小，扩大后重试
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // 不要扫描整个文件系统根目录
  return "";
}



// 扫描插件目录中的库，加载额外插件
void scanPluginLibraries() {
  // 检查并打印直接链接到可执行文件的插件
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // 定义平台相关字符串
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // 尝试打开 ${EXECDIR}/MUJOCO_PLUGIN_DIR 目录
  // ${EXECDIR} 是 simulate 二进制文件所在的目录
  // MUJOCO_PLUGIN_DIR 是预处理器宏
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- 仿真 -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

//---------------------------------------- 控制回调 -----------------------------------------

// mjcb_control 回调：MuJoCo 在 mj_step1 之后、mj_step2 之前自动调用
	// 你只需填好 d->ctrl[0..11]，MuJoCo 自动把力矩施加到对应关节

	//ID分配是顺时针从上到下分配的，类似轮式的民名规则
	void myController(const mjModel* m, mjData* d) {
	  // 站立目标关节角（12 个，顺序 LF→RF→LH→RH）
	  static const double stand_qpos[12] = {
	     0.0, 1.3398, -2.1068,   // LF: HAA=0, HFE=77°, KFE=-121°
	     0.0, 1.3398, -2.1068,   // RF
	     0.0, 1.3398, -2.1068,   // LH
	     0.0, 1.3398, -2.1068    // RH
	  };

	  if(d->time >3.0)
	  {
	  // 蹲姿初始关节角
	  static const double crouch_qpos[12] = {
	    d->qpos[7], d->qpos[8], d->qpos[9],   // LF
	    d->qpos[10], d->qpos[11], d->qpos[12],   // RF
	    d->qpos[13], d->qpos[14], d->qpos[15],   // LH
	    d->qpos[16], d->qpos[17], d->qpos[18]    // RH
	  };

	  // 3 秒 smoothstep 插值：alpha = clamp(time/3, 0, 1)
	  double alpha = (d->time - 3.0) / 3.0;
	  if (alpha > 1.0) alpha = 1.0;
	  double s = alpha * alpha * (3.0 - 2.0 * alpha);  // smoothstep 缓入缓出

	  // PD 增益
	  double kp = 100.0;  // 比例增益（位置误差 → 力矩）
	  double kd = 3.0;    // 阻尼增益（抑制速度振荡）

	  for (int i = 0; i < 12; i++) {
	    double target = crouch_qpos[i] + s * (stand_qpos[i] - crouch_qpos[i]);
	    double pos = d->qpos[7 + i];   // 当前关节角度
	    double vel = d->qvel[6 + i];   // 当前关节角速度
	    d->ctrl[i] = kp * (target - pos) - kd * vel;
	  }
	  }

	}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // 需要此副本，以便下面的 mju::strlen 调用能编译通过
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // 确保文件名不为空
  if (!filename[0]) {
    return nullptr;
  }

  // 加载并编译
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // 移除 loadError 末尾的换行符
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  // 没有错误但加载时间超过 1/4 秒时，报告加载耗时
  if (!loadError[0] && load_seconds > 1.0) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // 编译器警告：打印并暂停
  if (loadError[0]) {
    // 下面的 mj_forward() 会打印警告信息
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

// 后台线程中运行仿真（主线程同时负责渲染）
void PhysicsLoop(mj::Simulate& sim) {
  // CPU-仿真同步点
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // 一直运行直到请求退出
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // 锁定仿真互斥锁
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // 锁定仿真互斥锁
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // 休眠 1ms 或 yield，让主线程运行
    // yield 导致忙等待 - 时间精度更好但更耗电
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // 锁定仿真互斥锁
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // 只有在模型存在时才运行
      if (m) {
        // 运行中
        if (sim.run) {
          bool stepped = false;

          // 记录迭代开始时的 CPU 时间
          const auto startCPU = mj::Simulate::Clock::now();

          // 上次同步以来经过的 CPU 和仿真时间
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // 请求的减速因子
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // 失步条件：距离目标仿真时间的差距大于 syncMisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // 失步（任何原因）：重置同步时间，步进
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // 重新同步
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // 运行单步，让下一次迭代处理时序
            mj_step(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // 同步：步进直到超前 CPU
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // 当仿真落后于 CPU 且在 refreshTime 内时持续步进
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // 第一步之前测量减速比
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // 注入噪声
              sim.InjectNoise();

              // 调用 mj_step
              mj_step(m, d);
              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // 如果重置则退出循环
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // 将当前状态保存到历史缓冲区
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // 暂停
        else {
          // 运行 mj_forward，更新渲染和关节滑块
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // 释放 std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- 物理线程 --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // 如果提供了文件则请求加载模型（否则通过拖放加载）
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // 锁定仿真互斥锁
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // 锁定仿真互斥锁
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // 删除我们分配的所有资源
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ 主函数 --------------------------------------------------

// 在 Rosetta 下运行时，用 macOS 对话框替换命令行错误的机制
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// 运行事件循环
int main(int argc, char** argv) {

  // 如果在 macOS Rosetta 2 下运行则显示错误
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // 打印版本号，检查兼容性
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // 扫描插件目录中的库，加载额外插件
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate 对象封装 UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = nullptr;
  if (argc >  1) {
    filename = argv[1];
  }

  // 注册控制回调（必须在 PhysicsThread 之前注册）
  mjcb_control = myController;

  // 启动物理线程
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // 启动仿真 UI 循环（阻塞调用）
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}