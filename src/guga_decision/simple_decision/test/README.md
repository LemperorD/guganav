# simple_decision 测试

- [simple\_decision 测试](#simple_decision-测试)
  - [概述](#概述)
  - [前置依赖](#前置依赖)
  - [测试输入约定](#测试输入约定)
  - [阈值常量](#阈值常量)
  - [覆盖矩阵](#覆盖矩阵)
  - [已知缺口](#已知缺口)
  - [单元测试](#单元测试)
    - [test\_transform.cpp — 消息转换](#test_transformcpp--消息转换)
    - [test\_environment\_context.cpp — 环境状态聚合](#test_environment_contextcpp--环境状态聚合)
    - [test\_decision.cpp — 决策算法](#test_decisioncpp--决策算法)
  - [集成测试](#集成测试)
    - [test\_simple\_decision.cpp](#test_simple_decisioncpp)
      - [Gate（门控）](#gate门控)
      - [Decision（决策）](#decision决策)
      - [Rate-limiting（限速）](#rate-limiting限速)
      - [Helpers](#helpers)
  - [运行](#运行)

## 概述

**策略**：纯逻辑走单元测试（快、精确、无 ROS2 依赖），ROS2 接线和编排走集成测试（黑盒、真实消息）。不 mock 内部组件。

**被测范围**：消息转换 (`transform`) → 环境聚合 (`environment_context`) → 决策算法 (`decision`) → 节点编排 (`simple_decision`)。

**不测**：ROS2 框架行为（`rclcpp::Node::declare_parameter`、`create_publisher` 等）、tf2 底层。

## 前置依赖

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash    # pb_rm_interfaces, auto_aim_interfaces
source ~/guganav/install/setup.bash    # guga_interfaces, simple_decision
```

需要的 ROS2 包：`rclcpp`, `rclcpp_components`, `tf2_ros`, `geometry_msgs`, `std_msgs`, `guga_interfaces`。

## 测试输入约定

集成测试夹具 `DecisionSimpleTest` 创建节点时 `require_game_running=false`（默认），即发 RS 后不等待比赛状态就进入决策。门控测试通过 `CreateSystem({require, delay, hz})` 覆盖不同配置。

| Helper                                      | 默认行为                                    |
| ------------------------------------------- | ------------------------------------------- |
| `SendRobotStatus(hp, ammo)`                 | is_hp_deduced=false，无 armor/target 数据   |
| `SendGameStatus(progress)`                  | 仅设置 game_progress 字段                   |
| `PublishStaticTF(parent, child, x, y, yaw)` | 发布静态 TF 后 spin 200ms 等待生效          |
| `SpinFor(ms)`                               | `spin_some()` + 10ms sleep 循环，不精确计时 |
| `WaitForGoalAtLeast(n)`                     | 轮询 `spin_some()`，超时 2s 返回 false      |

## 阈值常量

| 常量                                  | 值            | 含义                       |
| ------------------------------------- | ------------- | -------------------------- |
| `kSupplyX / kSupplyY`                 | 0.0, 0.0      | 补给点坐标                 |
| `kDefaultX / kDefaultY / kDefaultYaw` | 2.0, 0.5, 0.0 | 默认目标点                 |
| `hp_survival_enter`                   | 120           | HP 低于此值进入补给        |
| `hp_survival_exit`                    | 300           | HP 高于此值退出补给        |
| `ammo_min`                            | 0             | 弹药等于此值进入补给       |
| `start_delay_sec`                     | 5.0           | 比赛开始后延迟决策秒数     |
| `attack_hold_sec_`                    | 1.5           | 受击后 LITTLE_TES 保持秒数 |
| `combat_max_distance`                 | 8.0           | 侦测敌人的最大距离 (m)     |

## 覆盖矩阵
根据要求,分割出下列覆盖矩阵.

| 需求                    | 单元测试                                             | 集成测试                              |
| ----------------------- | ---------------------------------------------------- | ------------------------------------- |
| 无 RS 不发              | `CheckReadiness_NoRS`                                | `Tick_NoRobotStatus_NoOutput`         |
| 无 GS 不发              | `CheckReadiness_NoGS`                                | `GateBehavior/1`                      |
| 比赛未开始不发          | `CheckReadiness_GameNotStarted`                      | `GateBehavior/2`                      |
| delay 内不发            | `CheckReadiness_StartedButInDelay`                   | `GateBehavior/3`                      |
| delay 后发              | `CheckReadiness_StartedAfterDelay`                   | `GateBehavior/4`                      |
| 离开 RUNNING 复位       | —                                                    | `LeaveRunning_ResetsGate`             |
| gatelog 日志            | —                                                    | `HandleGateLog_AllStatuses`           |
| HP 低→SUPPLY            | `ComputeAction_StatusBad`, `IsStatusBad`             | `LowHp_EntersSupply`                  |
| 弹药空→SUPPLY           | `ComputeAction_LowAmmo`                              | —                                     |
| SUPPLY 恢复→DEFAULT     | `ComputeAction_SupplyRecovered`, `IsStatusRecovered` | `SupplyRecovered_ExitsToDefault`      |
| 受击→LITTLE_TES         | `ComputeAction_EnemyRecentAndAttacked`               | `AttackedRecent_UsesLittleTes`        |
| 受击 hold 到期→FOLLOWED | —                                                    | `AttackedHoldExpired_BackToFollowed`  |
| 有敌人→ATTACK           | `ComputeAction_EnemyRecentWithAttackGoal`            | —                                     |
| 有追踪目标→target 坐标  | `FindAttackPosition_*`                               | —                                     |
| 默认点驻留→spin 锁存    | `ComputeAction_Default_AtCenterAndKeepSpin`          | —                                     |
| 默认态 goal 坐标        | `ComputeAction_Default_UsesDefaultCoordinates`       | —                                     |
| goal 节流               | —                                                    | `DefaultGoalPublishing_ThrottledByHz` |
| SUPPLY goal 节流        | —                                                    | `SupplyGoalPublishing_ThrottledByHz`  |
| TF 查询                 | —                                                    | `GetRobotPoseMap_*`                   |

`—` = 缺覆盖

## 已知缺口

- **「有敌人→ATTACK」集成测缺失**：单元测了算法，节点层没发 armor 验证端到端链路
- **「弹药空→SUPPLY」集成测缺失**：单元测了 `IsStatusBad`，节点层未发低弹药验证完整流程
- **`start_delay_sec` 边界未精确验证**：最小 delay=0.1s，未测 0s 即刻到期
- **`handleGateLog` 只测不崩溃**：未断言各状态输出了对应日志内容
- **受击 hold 到期无单元测**：时间逻辑只在集成层间接验证

---


## 单元测试

不启动 ROS2 节点，直接测试函数级逻辑。

### test_transform.cpp — 消息转换

| 测试                                        | 意图                                                |
| ------------------------------------------- | --------------------------------------------------- |
| `ConvertQuaternionTest`                     | ROS Quaternion → 领域 Quaternion 字段拷贝           |
| `ConvertPoseTest`                           | ROS Pose → 领域 Pose3D，验证 position + orientation |
| `ConvertPointTest`                          | ROS Point → 领域 Position 字段拷贝                  |
| `ConvertRobotStatusTest.CopiesScalarFields` | RobotStatus 标量字段全量转换                        |
| `ConvertRobotStatusTest.ThrowsOnNullptr`    | nullptr 抛异常                                      |
| `ConvertGameStatusTest`                     | GameStatus 字段拷贝                                 |
| `ConvertArmorsTest.CopiesAllArmorFields`    | Armors 列表转换 + pose 字段验证                     |
| `ConvertArmorsTest.ThrowsOnNullptr`         | nullptr 抛异常                                      |
| `ConvertArmorsTest.HandlesEmptyArmorList`   | 空列表不崩溃                                        |
| `ConvertTargetTest`                         | Target 全字段转换                                   |

### test_environment_context.cpp — 环境状态聚合

| 测试                                      | 意图                                         |
| ----------------------------------------- | -------------------------------------------- |
| `DefaultConstructor`                      | 默认构造后 isGameStarted/isGameOver 为 false |
| `OnRobotStatus`                           | 存储 RS 后 buildSnapshot 可读到 current_hp   |
| `OnGameStatus_NotStartedToRunning`        | COUNT_DOWN → RUNNING 触发 isGameStarted      |
| `OnGameStatus_RunningToGameOver`          | RUNNING → GAME_OVER 触发 isGameOver          |
| `OnGameStatus_ResetGameOver`              | resetGameOver 清除标记                       |
| `OnGameStatus_AlreadyRunning`             | 重复 RUNNING 不再次触发                      |
| `OnArmors` / `OnTarget`                   | 存储 armors / target 数据                    |
| `DetectEnemy_TargetTrackingAndInRange`    | 追踪目标在距离内 → true                      |
| `DetectEnemy_TargetTrackingButOutOfRange` | 追踪目标在距离外 → false                     |
| `DetectEnemy_NoTargetArmorInRange`        | 无目标但有装甲在范围内 → true                |
| `DetectEnemy_NoTargetArmorOutOfRange`     | 装甲在范围外 → false                         |
| `DetectEnemy_NoTargetNoArmors`            | 无目标无装甲 → false                         |
| `DetectEnemy_NonTrackingTarget`           | 非追踪目标被忽略，仍用装甲判断               |
| `IsStatusBad_HpBelowEnter`                | HP < 阈值 → true                             |
| `IsStatusBad_AmmoAtMin`                   | 弹药 = 0 → true                              |
| `IsStatusBad_HpAndAmmoOk`                 | HP 和弹药正常 → false                        |
| `changeState_DifferentState`              | 设不同状态 → 返回 true                       |
| `changeState_SameStateTwice`              | 设相同状态 → 第二次返回 false                |
| `CheckReadiness_NoRS`                     | 无 RS → NO_RS                                |
| `CheckReadiness_NoGS`                     | require_game + 无 GS → NO_GS                 |
| `CheckReadiness_GameNotStarted`           | 比赛未开始 → NOT_STARTED                     |
| `CheckReadiness_StartedButInDelay`        | 比赛开始但在延迟内 → IN_DELAY                |
| `CheckReadiness_StartedAfterDelay`        | 延迟过后 → READY                             |
| `CheckReadiness_RequireGameFalse`         | require_game=false + 有 RS → READY           |
| `IsNearRobotPose_NoPose`                  | 无姿态数据 → false                           |
| `IsNearRobotPose_WithinTolerance`         | 在容差内 → true                              |
| `IsNearRobotPose_BeyondTolerance`         | 超出容差 → false                             |
| `IsNearRobotPose_ExactlyAtTolerance`      | 边界值 → true                                |
| `BuildSnapshot_NoData`                    | 无数据时返回默认值                           |
| `BuildSnapshot_WithArmorInRange`          | 有装甲 → enemy=true                          |
| `UpdateTracking_AfterEnemy`               | tick1 敌现→tick2 enemy_recent=true           |
| `UpdateTracking_AfterAttacked`            | tick1 受击→tick2 attacked_recent=true        |
| `BuildSnapshot_AtCenter`                  | 在默认点 → at_center=true                    |
| `BuildSnapshot_FarFromCenter`             | 远离默认点 → at_center=false                 |
| `BuildSnapshot_CopiesMatchStartTime`      | match_start_time 正确转换为 Stamp            |

### test_decision.cpp — 决策算法

| 测试                                           | 意图                             |
| ---------------------------------------------- | -------------------------------- |
| `ComputeAction_SupplyNotRecovered`             | SUPPLY 态未恢复 → 保持 SUPPLY    |
| `ComputeAction_SupplyRecovered`                | SUPPLY 态已恢复 → 切回 DEFAULT   |
| `ComputeAction_StatusBad`                      | HP 低 → 进入 SUPPLY              |
| `ComputeAction_LowAmmo`                        | 弹药不足 → 进入 SUPPLY           |
| `ComputeAction_EnemyRecentNotAttacked`         | 有敌人未受击 → FOLLOWED          |
| `ComputeAction_EnemyRecentAndAttacked`         | 有敌人且受击 → LITTLE_TES        |
| `ComputeAction_EnemyRecentWithAttackGoal`      | 有敌人 → target 指向最近装甲     |
| `ComputeAction_EnemyRecentWithTrackingTarget`  | 有追踪目标 → target 指向目标位置 |
| `ComputeAction_Default_AtCenterAndKeepSpin`    | 在默认点 + 陀螺区 → 锁存 spin    |
| `ComputeAction_Default_NotAtCenterInKeepSpin`  | 不在默认点 → 不锁存              |
| `ComputeAction_Default_OutOfKeepSpin`          | 离开陀螺区 → 清除锁存            |
| `ComputeAction_Default_AttackedRecent`         | 默认态受击 → LITTLE_TES          |
| `ComputeAction_Default_UsesDefaultCoordinates` | 默认态 target 指向默认坐标       |
| `IsStatusRecovered_HpAboveExit`                | HP > 退出阈值 → true             |
| `IsStatusRecovered_HpBelowExit`                | HP < 退出阈值 → false            |
| `IsStatusRecovered_AmmoAtMin`                  | 弹药为 0 → false                 |
| `IsStatusBad_Healthy`                          | 健康 → false                     |
| `IsStatusBad_HpBelowEnter`                     | HP 低 → true                     |
| `IsStatusBad_AmmoAtMin`                        | 弹药不足 → true                  |
| `FindAttackPosition_TrackingTarget`            | 追踪目标 → 用目标 pose + yaw     |
| `FindAttackPosition_NoTargetWithArmors`        | 无目标 → 选最近装甲              |
| `FindAttackPosition_NoTargetNoArmors`          | 无目标无装甲 → nullopt           |
| `FindAttackPosition_NonTrackingTarget`         | 非追踪目标 → 走装甲逻辑          |

---

## 集成测试

### test_simple_decision.cpp

起真实 DecisionSimple 节点，通过 ROS2 消息发布/订阅验证端到端行为。

每个测试：`SendXxx()` → `SpinFor` / `WaitUntil` → 断言输出

#### Gate（门控）

| 测试                                 | 意图                                      |
| ------------------------------------ | ----------------------------------------- |
| `Tick_NoRobotStatus_NoOutput`        | 无 RS 不发 goal，避免基于过期数据决策     |
| `Tick_WithRobotStatus_PublishesGoal` | 有 RS 且 require_game=false → 立刻发 goal |
| `GateBehavior/0`                     | 有 GS+RUNNING 但无 RS → 不发              |
| `GateBehavior/1`                     | 有 RS 无 GS 且 require_game → 不发        |
| `GateBehavior/2`                     | 有 RS+GS 但 COUNT_DOWN → 不发（未开始）   |
| `GateBehavior/3`                     | 有 RS+GS+RUNNING 但在 delay 内 → 不发     |
| `GateBehavior/4`                     | delay=0.1s, wait=1.4s → 发                |
| `LeaveRunning_ResetsGate`            | 离开 RUNNING 后门控复位，不再输出         |
| `HandleGateLog_AllStatuses`          | 四种门控状态 warn 日志不崩溃              |

#### Decision（决策）

| 测试                                 | 意图                                   |
| ------------------------------------ | -------------------------------------- |
| `LowHp_EntersSupply`                 | HP < 阈值 → SUPPLY，target=补给坐标    |
| `SupplyRecovered_ExitsToDefault`     | HP 恢复 → 退出 SUPPLY，target=默认坐标 |
| `AttackedRecent_UsesLittleTes`       | 受击 → chassis=LITTLE_TES（闪避）      |
| `AttackedHoldExpired_BackToFollowed` | hold 到期 → chassis=CHASSIS_FOLLOWED   |

#### Rate-limiting（限速）

| 测试                                  | 意图                                      |
| ------------------------------------- | ----------------------------------------- |
| `DefaultGoalPublishing_ThrottledByHz` | 默认态 goal 按 default_goal_hz=1.0 节流   |
| `SupplyGoalPublishing_ThrottledByHz`  | SUPPLY 态 goal 按 supply_goal_hz=1.0 节流 |

#### Helpers

| 测试                       | 意图                                |
| -------------------------- | ----------------------------------- |
| `GetRobotPoseMap_TfExists` | TF 存在 → 返回 Pose2D，x/y/yaw 正确 |
| `GetRobotPoseMap_NoTf`     | 无 TF → 返回 nullopt                |

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
