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
