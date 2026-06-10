# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# One-time: source ROS2 Humble
source /opt/ros/humble/setup.bash

# Build (from this directory)
cd /home/cxk/quadruped_mpc_ws/src/Quadruped-Control-OCS2-ROS2
colcon build --symlink-install --continue-on-error

# ocs2_centroidal_model uses CppAD JIT — OOMs on parallel compile. If it fails with "signal terminated cc1plus":
rm -rf build/ocs2_centroidal_model
MAKEFLAGS="-j1" colcon build --symlink-install --packages-select ocs2_centroidal_model

# Source and launch (B1 as default)
source install/setup.bash
export LD_LIBRARY_PATH=$(pwd)/mujoco/mujoco-3.2.2/lib:$LD_LIBRARY_PATH
ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=b1

# Creeper
ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=creeper
```

## Architecture

This directory is a colcon workspace located at `~/quadruped_mpc_ws/src/Quadruped-Control-OCS2-ROS2/`. The workspace root is `~/quadruped_mpc_ws/`; all build/install artifacts live here (NOT in the workspace root).

Three ROS2 nodes launched together:

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

### WBC failure mode
- Creeper's qpOASES QP solver fails with "Maximum number of working set recalculations performed" (nWSR=20). Root cause: model parameter mismatch (mass, inertia, comHeight) between MPC config and URDF causes infeasible QP constraints. Fix the config parameters before touching the WBC code.
