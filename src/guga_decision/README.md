# guga_decision

决策模块，实现哨兵在ul/uc战场上的行为决策

## 待办

### 接入导航系统

决策树目前只在假数据源上跑通（`scripts/btview.sh`），还没和真实 Nav2 一起联调。

- **编排**：`scripts/reality.sh`／`simulation.sh` 现在起的是 `simple_decision`，要把
  `guga_rmul_strategy` 接进去。注意命名空间：裁判两路（`/referee/*`）与
  `/chassis_stop` 是绝对名，`goal_pose`／`odometry` 是相对名，带命名空间启动时
  两者的行为不一样。
- **链路验证**：`goal_pose` 确实被 `bt_navigator` 收下（QoS best effort 已核对）；
  controller 的 `xy_goal_tolerance`（0.15）与 `AchieveGoalPose` 的判定一致；
  `velocity_smoother` 的 `/chassis_stop` 确实把速度压零；手柄那一路
  （`usb_joystick_node` 直接发 `/cmd_vel`）是否也要受开关约束。
- **导航结果**：现在只发目标点、不持有 action 句柄，"导航失败""被取消"都不知情。
  要判这类情况得改用 `navigate_to_pose` action client。
- **实车前置数据**：RFID 卡的实际感应距离与"压上到消息到达"的延迟要实测，
  它们决定巡逻点间距（现取 ±0.25）与每个点的停留时间（现在为 0，即不停留）。

### v1.0.0 1v1 模块

与敌方哨兵单挑那一段的决策：何时交战、怎么走位、什么时候撤。感知侧的东西已经
齐了，缺的是"打"这一半。

- **输入已经有了**：`detector/armors`（装甲板）与 `tracker/target`（已跟踪目标的
  位置、速度、yaw、半径）都在 `guga_interfaces` 里，`simple_decision` 已经在用。
  决策层用 `Target` 就够，`tracking` 字段可以直接当"有没有可打的目标"。
- **执行器是缺的**：本仓库里没有任何云台角、开火、弹道解算的话题/服务/action，
  串口下行帧只有 `VX`/`VY`/`WZ_NEG` 三个速度。所以 1v1 要真的"开火"，得先定这个
  接口：谁负责解算与发射、走哪个话题、对应串口哪一帧（还是另有节点）。定下来之前
  能做的只有"该不该打"以及走位、躲避、停火这些不涉及发射的部分。
- **已有的可参考**：`simple_decision` 的 `attackAction` 是"把跟踪目标的位置当成
  目标点贴上去"，另有 `combat_max_distance` 作门限、受击后切小陀螺。BT 这边
  `Attack` 现在是空动作（只打日志、保持原目标前进），1v1 就是把它填上。
- **要和既有分支对齐**：现在 `ShouldEngage` 一成立就跳过占点分支，交战与"去占点"
  是二选一。1v1 要考虑是"边打边走位到点"还是继续二选一。
- **约束已经有了**：热量 `shooter_17mm_1_barrel_heat` 与热量上限、允许发弹量都
  在黑板上，开火判定直接用，不必再读一遍 ROS 消息。
