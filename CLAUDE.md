# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace

- **Active workspace**: `~/creeper_ws/src/Quadruped-Control-OCS2-ROS2/` (Creeper development)
- **Reference template**: `~/quadruped_mpc_ws/` (B1 working copy — do NOT modify)

## Build & Run

```bash
# ⚠️ 每次启动仿真前必须：
#   1. 关闭上一次的 MuJoCo 仿真渲染窗口
#   2. 清理所有相关进程
pkill -9 -f "mujoco_simulator\|legged_robot_sqp_mpc\|user_command_node\|ros2 launch\|gnome-terminal"

# Convenience script (handles all sourcing + env)
~/creeper_ws/run.sh creeper
~/creeper_ws/run.sh b1

# Manual launch
cd ~/creeper_ws/src/Quadruped-Control-OCS2-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$(pwd)/mujoco/mujoco-3.2.2/lib:$LD_LIBRARY_PATH
ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=creeper
```

```bash
# Build from workspace
colcon build --symlink-install --continue-on-error

# CppAD OOM fix — ocs2_centroidal_model and motion_control may both OOM on parallel compile:
rm -rf build/ocs2_centroidal_model build/motion_control
MAKEFLAGS="-j1" colcon build --symlink-install --packages-select ocs2_centroidal_model motion_control
```

## Architecture

Colcon workspace at `~/creeper_ws/src/Quadruped-Control-OCS2-ROS2/`. Three ROS2 nodes:

1. **mujoco_simulator** — MuJoCo physics engine at 1000Hz. Reads MJCF/URDF models from `models/<robot>/urdf/`. During the first 2s (`Start_simulate_=true`), applies PD control to hold initial pose. After MPC starts, switches to torque commands from WBC.

2. **legged_robot_sqp_mpc** (motion_control) — OCS2 MPC solver at 40Hz. Loads Pinocchio model from URDF, runs SQP optimization (24D state, 24D control), outputs optimized foot forces and swing trajectories. WBC (at 500Hz within the same node) converts MPC foot forces to joint torques via QP (qpOASES).

3. **user_command_node** — Keyboard teleop, publishes gait commands and target velocities.

### Key source files

| File | Role |
|------|------|
| `MPC_WBC_ROS_Interface.cpp` | Main control loop — 3 WBC call sites at lines ~210, ~283, ~464 |
| `MujocoSimulation.cpp` | MuJoCo sim — `simulateStep()` at line 88 reads `qpos[7+i]` for PD targets; `control_callback()` at line 317 sets `Start_simulate_=false` |
| `WbcBase.cpp` / `WeightedWbc.cpp` | QP formulation with 42 decision vars, 30 equality, 44 inequality constraints; nWSR=20 hardcoded |
| `ModelSettings.h` | Hardcoded joint names (LF→RF→LH→RH: `LF_HAA, LF_HFE, LF_KFE, RF_HAA, ...`) and contact names |
| `LeggedRobotInterface.cpp` | MPC setup — loads task.info, reference.info, URDF |

### Config files (per-robot in `user_command/config/<robot>/`)

- `task.info` — MPC weights (Q/R matrices), initial state, constraints, WBC weights
- `reference.info` — `comHeight`, default joint angles, gait schedule
- `simulation.info` — timestep, control frequencies, PID gains, render flags

## Critical Gotchas

### Build system
- **CppAD OOM**: `ocs2_centroidal_model` compiles CppAD JIT templates that consume massive memory. Must use `MAKEFLAGS="-j1"` for this package only. The CppAD AD templates are split into `ModelHelperFunctionsAD.cpp` (separate from `ModelHelperFunctions.cpp`) to avoid OOM in a single translation unit.
- **`ocs2_self_collision`**: Has COLCON_IGNORE — incompatible with Pinocchio 3.9.0 (`pinocchio::computeDistances` removed). Not needed by legged_control packages.
- **qpOASES**: Pre-built at `qpOASES-master/build/libs/libqpOASES.a`. CMakeLists in motion_control links via relative path.
- **`--continue-on-error` is required**: Several ocs2 packages cannot build on this system (ocs2_raisim needs Raisim SDK, perceptive_anymal needs robot data, etc.). Without this flag, one failure aborts the entire build.
- **COLCON_IGNORE markers** (manually added, not in git): `mujoco/` (contains ROS1 cheetah_fourth), `ocs2_ros2/ocs2_pinocchio/ocs2_self_collision/` (Pinocchio 3.9.0 API mismatch).
- **ocs2_ros2 is a git submodule** in the same colcon workspace. After `git checkout` of the main repo, verify submodule state with `git submodule status`.

### Config parsing
- **boost::property_tree `read_info`**: ONLY `;` and `#` are valid comments. `//` causes parse failure → MPC crashes with "Could not load matrix Q".

### Robot models
- **B1 is the working reference. Never modify B1 files.**
- Joint order in ModelSettings.h: LF→RF→LH→RH (matches URDF document order, not body order)
- MJCF `actuator` joint name must match ModelSettings.h joint names
- MJCF `sensor` jointpos/jointvel order must match ModelSettings.h (LF→RF→LH→RH)

### B1 vs Creeper key differences
| Parameter | B1 | Creeper |
|-----------|-----|---------|
| Mass | ~55 kg | ~11.67 kg |
| Leg length (HFE→Foot) | 0.70m | 0.4285m (61%) |
| Standing comHeight | ~0.54m | ~0.35m |
| Joint torque limit | 91-140 Nm | 23.7 Nm |

### Render visibility groups
- `MujocoSimulation.cpp` only enables `geomgroup[0,1,2]`. Geoms with `group="3"` are invisible. Use `group="0"` for always-visible collision geoms.

### Creeper free-fall phase
- During `Start_simulate_=true` (before MPC activates), NO PD or WBC control is applied — pure MuJoCo physics.
- Trunk initial height `z=0.455` gives ~1.5mm foot clearance for a natural settling.
- Belly box at `pos="0 0 0.03" size=...0.03` (top at z=0.06m) catches the body before it lies completely flat.
- Soft contact params (`solref="0.05 0.9" solimp="0.7 0.9 0.005"`) absorb impact energy to prevent bounce-induced leg clumping.
- **DO NOT add `ref` (qpos0) to Creeper joints** — it breaks the control pipeline (previous attempts failed).

## 开发铁律：模型先行，控制后行

**这是整个项目最重要的原则。忽视这条原则已经多次导致工程被毁。**

控制算法（MPC/WBC）的数学基础是精确的系统模型——质心动量模型、运动学模型、动力学参数。如果模型是错的，所有控制调试都是白费功夫，只会引入更多 hack，最终让工程无法维护。

B1 和 Creeper 的模型差距巨大（质量 ~11.7kg vs ~55kg，腿长 0.43m vs 0.70m，comHeight ~0.35m vs ~0.54m），Creeper 的 MPC 配置（task.info, reference.info）中大量参数是从 B1 复制过来的，不能直接信任。

### 模型验证清单（调试 MPC/WBC 之前必须逐项完成）

1. **URDF 物理参数验证**：质量、惯量、质心位置是否与实际机器人一致
2. **运动学验证（FK）**：站立姿态下足端位置是否正确，comHeight 是否与 reference.info 一致
3. **质心动量模型验证**：Pinocchio 从 URDF 计算的 centroidal dynamics（总质量、CoM、inertia）是否与 URDF 一致
4. **MPC 配置一致性**：task.info 中的 `comHeight`、`mass`、`inertia` 等参数是否从 URDF/真实模型导出，而非随意设置
5. **WBC 配置一致性**：contact points 位置、摩擦锥参数、权重矩阵是否合理

**没有通过模型验证之前，禁止修改 MPC 或 WBC 代码。**

### WBC failure mode
- Creeper 的 qpOASES QP solver 在上一次尝试中失败（nWSR=20）。根因极可能是模型参数不匹配（comHeight=0.38 vs 实际~0.35，mass/inertia 从 B1 复制），导致 QP 约束不可行。**必须先完成模型验证清单，再考虑碰 WBC 代码。**
