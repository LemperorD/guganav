# simple_decision 测试

- [单元测试](#单元测试)
  - [test_transform.cpp](#test_transformcpp--消息转换)
  - [test_environment_context.cpp](#test_environment_contextcpp--环境状态聚合)
  - [test_decision.cpp](#test_decisioncpp--决策算法)
- [集成测试](#集成测试)
  - [test_simple_decision.cpp](#test_simple_decisioncpp)
    - [Gate](#gate门控)
    - [Decision](#decision决策)
    - [Rate-limiting](#rate-limiting限速)
    - [Helpers](#helpers)
- [运行](#运行)

## 单元测试

不启动 ROS2 节点，直接测试函数级逻辑。

### test_transform.cpp — 消息转换

| 测试 | 意图 |
|------|------|
| `ConvertQuaternionTest` | ROS Quaternion → 领域 Quaternion 字段拷贝 |
| `ConvertPoseTest` | ROS Pose → 领域 Pose3D，验证 position + orientation |
| `ConvertPointTest` | ROS Point → 领域 Position 字段拷贝 |
| `ConvertRobotStatusTest.CopiesScalarFields` | RobotStatus 标量字段全量转换 |
| `ConvertRobotStatusTest.ThrowsOnNullptr` | nullptr 抛异常 |
| `ConvertGameStatusTest` | GameStatus 字段拷贝 |
| `ConvertArmorsTest.CopiesAllArmorFields` | Armors 列表转换 + pose 字段验证 |
| `ConvertArmorsTest.ThrowsOnNullptr` | nullptr 抛异常 |
| `ConvertArmorsTest.HandlesEmptyArmorList` | 空列表不崩溃 |
| `ConvertTargetTest` | Target 全字段转换 |

### test_environment_context.cpp — 环境状态聚合

| 测试 | 意图 |
|------|------|
| `DefaultConstructor` | 默认构造后 isGameStarted/isGameOver 为 false |
| `OnRobotStatus` | 存储 RS 并设置 has_rs |
| `OnGameStatus_NotStartedToRunning` | COUNT_DOWN → RUNNING 触发 isGameStarted |
| `OnGameStatus_RunningToGameOver` | RUNNING → GAME_OVER 触发 isGameOver |
| `OnGameStatus_ResetGameOver` | resetGameOver 清除标记 |
| `OnGameStatus_AlreadyRunning` | 重复 RUNNING 不再次触发 |
| `OnArmors` / `OnTarget` | 存储 armors / target 数据 |
| `DetectEnemy_TargetTrackingAndInRange` | 追踪目标在距离内 → true |
| `DetectEnemy_TargetTrackingButOutOfRange` | 追踪目标在距离外 → false |
| `DetectEnemy_NoTargetArmorInRange` | 无目标但有装甲在范围内 → true |
| `DetectEnemy_NoTargetArmorOutOfRange` | 装甲在范围外 → false |
| `DetectEnemy_NoTargetNoArmors` | 无目标无装甲 → false |
| `DetectEnemy_NonTrackingTarget` | 非追踪目标被忽略，仍用装甲判断 |
| `IsStatusBad_HpBelowEnter` | HP < 阈值 → true |
| `IsStatusBad_AmmoAtMin` | 弹药 = 0 → true |
| `IsStatusBad_HpAndAmmoOk` | HP 和弹药正常 → false |
| `SetState_DifferentState` | 设不同状态 → 返回 true |
| `SetState_SameStateTwice` | 设相同状态 → 第二次返回 false |
| `CheckReadiness_NoRS` | 无 RS → NO_RS |
| `CheckReadiness_NoGS` | require_game + 无 GS → NO_GS |
| `CheckReadiness_GameNotStarted` | 比赛未开始 → NOT_STARTED |
| `CheckReadiness_StartedButInDelay` | 比赛开始但在延迟内 → IN_DELAY |
| `CheckReadiness_StartedAfterDelay` | 延迟过后 → READY |
| `CheckReadiness_RequireGameFalse` | require_game=false + 有 RS → READY |
| `IsNearRobotPose_NoPose` | 无姿态数据 → false |
| `IsNearRobotPose_WithinTolerance` | 在容差内 → true |
| `IsNearRobotPose_BeyondTolerance` | 超出容差 → false |
| `IsNearRobotPose_ExactlyAtTolerance` | 边界值 → true |
| `GetSnapshot_NoData` | 无数据时返回默认值 |
| `GetSnapshot_WithArmorInRange` | 有装甲 → enemy=true, enemy_recent=true |
| `GetSnapshot_EnemyRecent` | 持续有敌人 → enemy_recent 保持 |
| `GetSnapshot_AttackedRecent` | 受击 → attacked_recent=true |
| `GetSnapshot_AtCenter` | 在默认点 → at_center=true |
| `GetSnapshot_FarFromCenter` | 远离默认点 → at_center=false |
| `GetSnapshot_CopiesMatchStartTime` | match_start_time 正确转换为 Stamp |

### test_decision.cpp — 决策算法

| 测试 | 意图 |
|------|------|
| `ComputeAction_SupplyNotRecovered` | SUPPLY 态未恢复 → 保持 SUPPLY |
| `ComputeAction_SupplyRecovered` | SUPPLY 态已恢复 → 切回 DEFAULT |
| `ComputeAction_StatusBad` | HP 低 → 进入 SUPPLY |
| `ComputeAction_LowAmmo` | 弹药不足 → 进入 SUPPLY |
| `ComputeAction_EnemyRecentNotAttacked` | 有敌人未受击 → FOLLOWED |
| `ComputeAction_EnemyRecentAndAttacked` | 有敌人且受击 → LITTLE_TES |
| `ComputeAction_EnemyRecentWithAttackGoal` | 有敌人 → target 指向最近装甲 |
| `ComputeAction_EnemyRecentWithTrackingTarget` | 有追踪目标 → target 指向目标位置 |
| `ComputeAction_Default_AtCenterAndKeepSpin` | 在默认点 + 陀螺区 → 锁存 spin |
| `ComputeAction_Default_NotAtCenterInKeepSpin` | 不在默认点 → 不锁存 |
| `ComputeAction_Default_OutOfKeepSpin` | 离开陀螺区 → 清除锁存 |
| `ComputeAction_Default_AttackedRecent` | 默认态受击 → LITTLE_TES |
| `ComputeAction_Default_UsesDefaultCoordinates` | 默认态 target 指向默认坐标 |
| `IsStatusRecovered_HpAboveExit` | HP > 退出阈值 → true |
| `IsStatusRecovered_HpBelowExit` | HP < 退出阈值 → false |
| `IsStatusRecovered_AmmoAtMin` | 弹药为 0 → false |
| `IsStatusBad_Healthy` | 健康 → false |
| `IsStatusBad_HpBelowEnter` | HP 低 → true |
| `IsStatusBad_AmmoAtMin` | 弹药不足 → true |
| `FindAttackPosition_TrackingTarget` | 追踪目标 → 用目标 pose + yaw |
| `FindAttackPosition_NoTargetWithArmors` | 无目标 → 选最近装甲 |
| `FindAttackPosition_NoTargetNoArmors` | 无目标无装甲 → nullopt |
| `FindAttackPosition_NonTrackingTarget` | 非追踪目标 → 走装甲逻辑 |

---

## 集成测试

### test_simple_decision.cpp

起真实 DecisionSimple 节点，通过 ROS2 消息发布/订阅验证端到端行为。

每个测试：`SendXxx()` → `SpinFor` / `WaitUntil` → 断言输出

#### Gate（门控）

| 测试 | 意图 |
|------|------|
| `Tick_NoRobotStatus_NoOutput` | 无 RS 不发 goal，避免基于过期数据决策 |
| `Tick_WithRobotStatus_PublishesGoal` | 有 RS 且 require_game=false → 立刻发 goal |
| `GateBehavior/0` | 有 GS+RUNNING 但无 RS → 不发 |
| `GateBehavior/1` | 有 RS 无 GS 且 require_game → 不发 |
| `GateBehavior/2` | 有 RS+GS 但 COUNT_DOWN → 不发（未开始） |
| `GateBehavior/3` | 有 RS+GS+RUNNING 但在 delay 内 → 不发 |
| `GateBehavior/4` | delay=0.1s, wait=1.4s → 发 |
| `LeaveRunning_ResetsGate` | 离开 RUNNING 后门控复位，不再输出 |
| `HandleGateLog_AllStatuses` | 四种门控状态 warn 日志不崩溃 |

#### Decision（决策）

| 测试 | 意图 |
|------|------|
| `LowHp_EntersSupply` | HP < 阈值 → SUPPLY，target=补给坐标 |
| `SupplyRecovered_ExitsToDefault` | HP 恢复 → 退出 SUPPLY，target=默认坐标 |
| `AttackedRecent_UsesLittleTes` | 受击 → chassis=LITTLE_TES（闪避） |
| `AttackedHoldExpired_BackToFollowed` | hold 到期 → chassis=CHASSIS_FOLLOWED |

#### Rate-limiting（限速）

| 测试 | 意图 |
|------|------|
| `DefaultGoalPublishing_ThrottledByHz` | 默认态 goal 按 default_goal_hz=1.0 节流 |
| `SupplyGoalPublishing_ThrottledByHz` | SUPPLY 态 goal 按 supply_goal_hz=1.0 节流 |

#### Helpers

| 测试 | 意图 |
|------|------|
| `GetRobotPoseMap_TfExists` | TF 存在 → 返回 Pose2D，x/y/yaw 正确 |
| `GetRobotPoseMap_NoTf` | 无 TF → 返回 nullopt |

---

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/guganav/install/setup.bash
cd ~/guganav/build/simple_decision

# 全部
for t in test_transform test_environment_context test_decision test_simple_decision; do
  ./$t
done

# 单个
./test_simple_decision --gtest_filter="*LowHp*"
```
