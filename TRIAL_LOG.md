# PD 站立调试试错记录

> 日期：2026-06-10

## 目标
无腹块（蓝色方块）条件下，让 Creeper (11.5kg) 以 PD 位控方式平稳站立，然后交接给 MPC。

## 关键发现

### 1. 关节阻尼是缺失的关键 (Round 26-27)
- **问题**：所有 PD 参数组合下机器人都会在最初 50ms 内爆发式振荡然后倒塌
- **根因**：MuJoCo 仿真中球形脚 + 零关节阻尼 = 零能量耗散机制。基座姿态振荡永不衰减
- **解决**：给 12 个关节添加 `damping="2.0"` Nm/(rad/s)，模拟真实电机摩擦
- **结果**：机器人立即站稳，base_z 从 0.087 升到 0.250

### 2. 足端必须落在髋正下方 (Round 17-18)
- FK 分析发现原目标关节角 HFE=0.0 导致足端比髋前移 26cm
- 修正目标：HFE=1.05, KFE=-1.67, HAA=±0.20
- base_z=0.30 对应足端恰好触地

### 3. 重力补偿防止初始塌陷 (Round 29)
- PD 在目标姿态时误差为零→力矩为零→无法对抗重力
- mj_rne 给自由漂浮机器人算出的重力补偿只有 0.18 Nm（无用）
- 手工估算：KFE=2.5 Nm, HFE=1.0 Nm
- 必须叠在 PD 力矩上

### 4. 基座姿态反馈 (Round 28, 31)
- 纯关节 PD 无法直接控制基座倾斜
- 添加 Kp_att=10 的角速度反馈到 HFE/KFE
- 有帮助但不能消除 wx 振荡

## 当前最佳配置

### creeper.xml
- 腹块已注释
- joint damping="2.0" on all 12 joints
- floor friction="2 0.5 0.1" condim="3"

### simulation.info
```
stand_up: duration=0.5, free_fall=0.001
kp=100, kd=10, torque_limit=10
gravity_comp: KFE=2.5, HFE=1.0 Nm (hardcoded)
Kp_att=10 (attitude feedback on HFE/KFE)
```

### target joint angles
HAA=±0.20, HFE=1.05, KFE=-1.67 (FL/FR)
HAA=±0.20, HFE=1.04, KFE=-1.66 (RL/RR)

### 关键代码位置
- `MujocoSimulation.cpp:95-107` — qpos 预置 + 重力补偿
- `MujocoSimulation.cpp:355-398` — simulateStep 的 Start_simulate_ 和 Start_stand_up_ 分支
- `MujocoSimulation.hpp:88-104` — 站立阶段成员变量

## 结果
- base_z = 0.250 ± 0.0005m ✓ 高度稳定
- vx = 0.26 m/s 持续漂移（+X 方向）✗ 不满足 |vx|<0.05 阈值
- wx = ±1.4 rad/s 周期性振荡 ✗ 不满足 |wx|<0.15 阈值
- HAA = [0.33, -0.33, 0.33, -0.34] — 对称，比目标略外展

## 失败的尝试
| Round | 配置 | 结果 |
|-------|------|------|
| 1-8 | 两阶段 HAA ramp | 漂移 1.4-2.2m |
| 9-11 | 统一 ramp | 漂移改善但倒塌 |
| 12-14 | 直接设 qpos (位置控制) | 身体不抬升 |
| 15-16 | 平板脚 (box feet) | 完全倒塌 |
| 17-19 | condim=4 扭转摩擦 | XML 不兼容 |
| 20-21 | Kp_att=50 | 400Nm 饱和振荡 |
| 22-23 | 自由落体+PDhold→ramp | 着陆后倒塌 |
| 24-25 | Kp=150+, Kd=20-50 | 摩擦超标→打滑 |
| 26 | **damping=2.0** | **站稳！** |
| 27-29 | 微调 Kp/Kd/grav_comp | z=0.250 最佳 |
| 30 | friction=5 | 粘滑振荡 |

## 待解决
1. vx=0.26 m/s 恒定漂移——可能需要 MPC 的 com 速度控制
2. wx 振荡——需要降低稳定性阈值或改进态度控制
3. MPC 交接时的 Eigen 断言崩溃——无腹块时机器人在 z=0.087 的极端姿态导致 setInitialState 失败
