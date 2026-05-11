# simple_decision 集成测试

`test_simple_decision.cpp` — 起真实 DecisionSimple 节点，通过 ROS2 中间件发布输入消息、监听输出消息，黑盒验证端到端行为。

## 测试结构

每个测试遵循：发布输入 `SendXxx()` → 等待处理 `SpinFor` / `WaitUntil` → 断言输出

## Gate（门控）

| 测试 | 意图 |
|------|------|
| `Tick_NoRobotStatus_NoOutput` | 无 robot_status 时节点不发 goal_pose，避免基于过期数据做决策 |
| `Tick_WithRobotStatus_PublishesGoal` | 有 RS 且 require_game_running=false 时立刻发 goal |
| `GateBehavior/0` | 有 GS 有 RUNNING 但无 RS → 不发 |
| `GateBehavior/1` | 有 RS 无 GS 且 require_game → 不发 |
| `GateBehavior/2` | 有 RS 有 GS 有 COUNT_DOWN → 不发（比赛未开始） |
| `GateBehavior/3` | 有 RS 有 GS 有 RUNNING 但在 delay 内（300ms < 5s）→ 不发 |
| `GateBehavior/4` | delay=0.1s，wait=1.4s 后 → 发 |
| `LeaveRunning_ResetsGate_NoFurtherOutput` | 离开 RUNNING 后门控复位，不再输出 |
| `HandleGateLog_AllStatuses_DoNotCrash` | 四种门控状态对应的 warn 日志不崩溃 |

## Decision（决策）

| 测试 | 意图 |
|------|------|
| `LowHp_EntersSupply_PublishesSupplyGoal` | hp 低于补给阈值 → SUPPLY 态，target 指向补给坐标 |
| `SupplyRecovered_ExitsToDefaultGoal` | hp/ammo 恢复 → 退出 SUPPLY，target 切回默认坐标 |
| `AttackedRecent_UsesLittleTesMode` | 受击后 chassis 切为 LITTLE_TES（闪避） |
| `AttackedHoldExpired_BackToFollowed` | attacked_recent hold 到期 → chassis 回到 CHASSIS_FOLLOWED |

## Rate-limiting（限速）

| 测试 | 意图 |
|------|------|
| `DefaultGoalPublishing_ThrottledByHz` | 默认态 goal 按 default_goal_hz=1.0 节流 |
| `SupplyGoalPublishing_ThrottledByHz` | SUPPLY 态 goal 按 supply_goal_hz=1.0 节流 |

## Helpers

| 测试 | 意图 |
|------|------|
| `GetRobotPoseMap_TfExists_ReturnsPose` | TF 存在时返回 Pose2D，x/y/yaw 与静态 TF 一致 |
| `GetRobotPoseMap_NoTf_ReturnsNullopt` | 无 TF 时返回 nullopt |

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/guganav/install/setup.bash
cd ~/guganav/build/simple_decision
./test_simple_decision
```
