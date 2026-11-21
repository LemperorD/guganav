# rmul_decision

## 本仓库适配

本包是 `guganav` 中 RMUL 哨兵的行为树决策节点，基于 BehaviorTree.CPP 4 实现。
节点读取裁判系统、比赛状态、RFID 和 TF 位姿，在
`Suppress`、`Occupation`、`PushForward` 三种策略间切换，并通过 Nav2
`NavigateToPose` action 顺序发送航点。

当前实现的是固定点位巡逻与站点转移，不包含瞄准、开火或弹道解算。决策节点只发送
导航目标；底盘速度由 Nav2／控制器下发，`pitch_mode` 则通过串口驱动随运动帧发给
MCU。

本 README 描述当前工作区实现。通用输入层见
[guga_decision_common](../guga_decision_common/README.md)，包在系统中的位置和后续
接入工作见 [guga_decision](../README.md)。

### 构建

从工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/guganav
colcon build --symlink-install --packages-select rmul_decision
source install/setup.bash
```

不使用假数据源、只接入已有 ROS 图时：

```bash
ros2 launch rmul_decision rmul_decision.launch.py
```

包内 launch 只启动决策节点，不会启动 Nav2、定位、裁判串口或假数据。缺少
`navigate_to_pose` action、机器人状态或 TF 时，决策树无法完整推进。

### 无实车调试

仓库提供假裁判、假位姿、假 `NavigateToPose` action 和 Web 行为树界面：

```bash
colcon build --symlink-install \
  --packages-select rmul_decision fake_msg_source
source install/setup.bash
scripts/btview.sh --no-browser
```

浏览器打开 `http://localhost:8080`。另一个终端可实时修改血量、比赛阶段和 RFID：

```bash
scripts/tune_fake_source.sh
scripts/tune_fake_source.sh hp 350
scripts/tune_fake_source.sh game 0
scripts/tune_fake_source.sh game 4
scripts/tune_fake_source.sh rfid center true
```

接入已经运行的 Gazebo/Nav2 时：

```bash
scripts/simulation.sh nav rmul_2025
scripts/btview.sh --simulation
```

`--simulation` 默认使用命名空间 `/red_standard_robot1`，只让
`fake_msg_source` 提供 `/fake_ref` 下的假裁判数据，不发布假 TF。仿真命名空间不同
时传 `--namespace <name>`。

标准 `scripts/reality.sh` 和 `scripts/simulation.sh` 当前仍启动
`simple_decision`。使用本包时需要单独启动决策节点，或把
`guga_bringup` 的决策 launch 切换为 `rmul_decision`。

## 输入与输出

### 输入

所有 ROS 输入由一个 `DecisionInputHub` 统一订阅和缓存，节点实际名称为
`rmul_decision_input`。

| 话题 / TF / action | 类型 | 说明 |
|---|---|---|
| `/referee/robot_status` | `guga_interfaces/RobotStatus` | 当前血量、允许发弹量、枪管热量等 |
| `/referee/game_status` | `guga_interfaces/GameStatus` | 比赛阶段；`4` 表示比赛中 |
| `/referee/rfid_status` | `guga_interfaces/RfidStatus` | 己方补给区与中心增益点状态 |
| `vision/info` | `guga_interfaces/VisionInfo` | 视野内敌方数量 |
| TF `map -> base_footprint` | `tf2` | 机器人位姿；当前主要用于黑板展示 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` action | `GotoPoints` 发送并推进航点 |

裁判三路话题默认使用绝对名称；`vision/info` 和 action 使用相对名称，因此带命名空间
启动时会解析到当前命名空间。`map` 与 `base_footprint` 也可由 `frame_id`、
`base_frame_id` 参数覆盖。

机器人状态是必需输入。未收到 `/referee/robot_status` 时，`Ros2Input` 返回
`RUNNING`，比赛状态和策略节点都不会执行。当前 `rmul_decision` 不检查输入时间戳是否
过期。

### 输出

| 话题 / action | 类型 | 说明 |
|---|---|---|
| `NavigateToPose` goal | `nav2_msgs/NavigateToPose` | 当前航点目标；action result 决定是否推进下一个点 |
| `goal_pose_marker` | `visualization_msgs/Marker` | 当前决策目标箭头，Transient Local QoS |
| `/decision/pitch_mode` | `std_msgs/UInt8` | 过洞姿态模式，`0=关闭`、`1=开启` |
| `/btview/blackboard` | `diagnostic_msgs/DiagnosticArray` | 黑板快照，每个行为树 tick 发布 |
| `/btview/node_status` | `diagnostic_msgs/DiagnosticArray` | 行为树节点状态心跳 |

`/decision/pitch_mode` 由 `serial_driver` 订阅，并写入 BR 运动帧
`0xCD payload[12]`，随后与下一帧 `/cmd_vel` 一起下发 MCU。协议定义见
[通信协议.md](../../../docs/通信协议.md)。

`goal_pose_marker` 不是控制输入，只用于 RViz 观察；“Decision Goal”显示已在默认 RViz
配置中启用。带命名空间时完整话题如
`/red_standard_robot1/goal_pose_marker`。

## 行为树结构

主树定义在 [bt_trees/BT.xml](bt_trees/BT.xml)：

```text
Main
└── KeepRunningUntilFailure
    └── ForceSuccess
        └── ReactiveSequence
            ├── Ros2Input
            ├── IsGameStart
            ├── ForceSuccess(SwitchState)
            └── Switch3(global_state_)
                ├── Suppress
                ├── Occupation
                └── PushForward
```

每个 tick 的执行顺序固定为：

1. `Ros2Input` 刷新输入黑板。
2. `IsGameStart` 要求 `game_progress == 4`。
3. `SwitchState` 尝试进行状态迁移。
4. `Switch3` 根据迁移后的 `global_state_` 执行对应策略子树。

默认 tick 频率是 1 Hz，可通过 `tick_rate_hz` 调整。`FileLogger2` 默认把执行记录写入
`/tmp/bt_trace.btlog`，传空路径可关闭。

### 策略状态机

初始状态为 `Suppress`。状态迁移由两个条件节点触发：

| 触发节点 | 触发条件 | 当前效果 |
|---|---|---|
| `IsRevivedAndDead` | 先观察到 `HP == 0`，随后观察到 `HP > 0` | 调用 `ChangeLowerState` |
| `IsOccupationsafe` | `destination == 4` 连续保持 20 秒且 `HP > 350` | 调用 `ChangeHigherState` |

迁移关系：

```text
ChangeLowerState:
  Suppress     -> Occupation
  Occupation   -> PushForward
  PushForward  -> PushForward

ChangeHigherState:
  Occupation   -> Suppress
  PushForward  -> Occupation
  Suppress     -> Suppress
```

每次状态真正变化时，`route_epoch` 加一。`GotoPoints` 将该批次纳入完成缓存，确保
从另一策略返回时重新计算路线，而不是复用上一次相同 `(destination, strategy)` 的
完成状态。

### 策略与目标点

`CheckHP` 每次 tick 根据当前 `HP` 和策略重新选择 `destination`。

| 策略 | HP 条件 | destination |
|---|---:|---:|
| `Suppress` | `HP > 300` | 5 |
| `Suppress` | `90 < HP <= 300` | 4 |
| `Suppress` | `HP <= 90` | 1 |
| `Occupation` | `HP > 150` | 4 |
| `Occupation` | `90 < HP <= 150` | 3 |
| `Occupation` | `HP <= 90` | 1 |

`PushForward` 的规则同时依赖当前 destination：

| 当前 destination | 条件 | 新 destination |
|---|---:|---:|
| 2 | `HP < 90` | 1 |
| 3 | `HP < 150` | 1 |
| 4 | `HP < 250` | 1 |
| 非 2 / 3 / 4 | 无 | 2 |

另有全局门限：只要当前 `destination == 1` 且 `HP < 400`，`CheckHP` 会保持
destination 为 1。因此低血量回到点 1 后，即使策略阈值已经满足，也要等血量恢复满
才会离开点 1。

`PushForward` 子树还包含 `CheckTime`：

- 到达 destination 2 后停留 15 秒，切换到 3。
- 到达 destination 3 后停留 15 秒，切换到 4。
- `stay_time` 从 `GotoPoints` 返回成功后的下一轮 tick 开始累计。

## 点位与路线

内置坐标写在
[GotoPoints.hpp](include/rmul_decision/action_node/GotoPoints.hpp)：

| 点位 | 坐标 `(x, y)` | 用途 |
|---|---:|---|
| 1 | `(0.0, 0.0)` | 己方补给／复活后低血量点 |
| 2 | `(0.7, -4.0)` | PushForward 第一站 |
| 3 | `(2.7, -4.7)` | PushForward 第二站 |
| 4 | `(5.6, -2.8)` | Occupation 占点位置 |
| 5 | `(8.0, -1.6)` | Suppress 高血量位置 |

部分点位有压制状态专用隧道路线：

| 路线 | 航点序列 |
|---|---|
| `tunnel_1` | `(3.3, 0.2) -> (1.0, 0.2) -> (0.0, 0.0)` |
| `tunnel_4` | `(1.0, 0.2) -> (3.3, 0.2) -> (5.6, -2.8)` |
| `tunnel_5` | `(1.0, 0.2) -> (3.3, 0.2) -> (8.0, -1.6)` |

路线选择规则：

- 仅 `Suppress` 策略可能选择 `tunnel_1`、`tunnel_4`、`tunnel_5`。
- destination 2、3 没有隧道路线，直接前往普通点位。
- 如果上一个 destination 是 4，而当前目标不是 4，本节路线跳过隧道路线，直接前往
  当前普通点位。因此 `Occupation(4) -> Suppress(5)` 会直接去 5。
- 使用隧道路线时，`GotoPoints` 将 `pitch_mode` 写为 1；普通路线写为 0。
- `route_epoch` 变化会清空旧完成状态并重新选择路线，防止跨策略切换后沿用旧路线。

树中 `route_x`、`route_y`、`route_yaw` 默认为空，所以使用上述内置点位。显式映射这三
个端口可以覆盖内置路线；自定义路线不会自动选择隧道，`pitch_mode` 保持 0。

## GotoPoints

`GotoPoints` 是 stateful action，封装了两个协作对象：

- `WaypointNavigator`：保存当前路线、当前航点和剩余航点。
- `GoalController`：发送 `NavigateToPose` action、处理 result 并发布目标 Marker。

工作流程：

1. 根据 destination 和 strategy 构造航点数组。
2. 发送第一个 `NavigateToPose` goal。
3. 每次 action 返回成功，调用 `WaypointNavigator::update(true)` 并发送下一个点。
4. 最后一个点成功后，节点返回 `SUCCESS`。
5. action 失败或取消时，节点返回 `FAILURE` 并停止剩余路线。
6. destination 或 strategy 变化时，取消当前目标并立即开始新路线。

完成缓存以 `(destination, strategy, route_epoch)` 为键。树在目标完成后重新 tick 时不会
重复发送首个航点；状态切换导致 `route_epoch` 变化时则必须重新计算。

## 黑板变量

| 变量 | 类型 | 初始值 | 写入者 | 说明 |
|---|---|---:|---|---|
| `game_progress` | `int` | -1 | `Ros2Input` | 比赛阶段，4 为比赛中 |
| `HP` | `double` | 400 | `Ros2Input` | 当前血量 |
| `ammo` | `int` | 未显式初始化 | `Ros2Input` | 允许发弹量 |
| `robot_x` / `robot_y` | `double` | 未显式初始化 | `Ros2Input` | `map` 系位置；TF 失败时为 NaN |
| `pose_valid` | `bool` | 未显式初始化 | `Ros2Input` | 本轮 TF 查询是否成功 |
| `destination` | `int` | 1 | `CheckHP`、`CheckTime` | 当前目标点编号 |
| `previous_destination` | `int` | -1 | `GotoPoints` | 上一个已启动导航的点位 |
| `route_epoch` | `int` | 0 | `ChangeHigherState`、`ChangeLowerState` | 策略切换批次 |
| `global_state_` | `string` | `"Suppress"` | 状态迁移节点 | 当前策略 |
| `occupation_time` | `double` | 0.0 | `IsOccupationsafe` | 目标 4 的计时 |
| `stay_time` | `double` | 0.0 | `CheckTime` | PushForward 当前点停留时间 |
| `HP_sign` | `int` | 0 | `IsRevivedAndDead` | 用于识别“到 0 后恢复”的边沿 |
| `pitch_mode` | `int` | 0 | `GotoPoints` | 是否正在执行过洞姿态路线 |

`/btview/blackboard` 每个 tick 发布一次上述主要变量，Web 页面和
`ros2 topic echo` 都可查看。策略字符串区分大小写，必须是 `Suppress`、
`Occupation` 或 `PushForward`。

## 参数

### 决策参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tick_rate_hz` | `1.0` | 行为树 tick 频率；非正数回退到 1 Hz |
| `btlog_path` | `/tmp/bt_trace.btlog` | BehaviorTree.CPP 执行记录；空字符串关闭 |
| `navigate_to_pose_action` | `navigate_to_pose` | Nav2 action 名称 |
| `goal_pose_marker_topic` | `goal_pose_marker` | 当前目标箭头话题 |
| `bt_path` | 包内 `bt_trees/BT.xml` | 自定义行为树文件；需通过 `--ros-args -p` 注入 |
| `use_sim_time` | `false` | launch 参数，仿真时应为 true |

### 输入层参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_status_topic` | `/referee/robot_status` | 机器人状态 |
| `game_status_topic` | `/referee/game_status` | 比赛状态 |
| `rfid_status_topic` | `/referee/rfid_status` | RFID 状态 |
| `vision_topic` | `vision/info` | 视觉汇总 |
| `frame_id` | `map` | 位姿查询父帧 |
| `base_frame_id` | `base_footprint` | 位姿查询子帧 |

输入参数没有单独的 YAML 文件，使用 launch 参数或 ROS 参数覆盖：

```bash
ros2 run rmul_decision rmul_decision --ros-args \
  -p tick_rate_hz:=2.0 \
  -p navigate_to_pose_action:=navigate_to_pose \
  -p vision_topic:=/robot1/vision/info
```

## 调试与观测

```bash
# 当前策略、目标点、计时和过洞模式
ros2 topic echo /btview/blackboard

# 串口姿态模式链路
ros2 topic echo /decision/pitch_mode

# 当前导航目标箭头
ros2 topic echo /goal_pose_marker

# action 是否可用
ros2 action list | grep navigate_to_pose
```

行为树 Web 界面同时显示黑板快照、本轮执行路径、节点状态和最近 10 次 tick。离线查看
`.btlog`：

```bash
python3 scripts/btview/btlog_view.py /tmp/bt_trace.btlog
python3 scripts/btview/btlog_view.py /tmp/bt_trace.btlog --tick 7
```

## 当前限制

1. 点位、血量阈值、20 秒占点时间和 15 秒停留时间均写死，尚未参数化。
2. `IsOccupationsafe` 在 destination 变为 4 时即开始计时，而不是等 Nav2 确认到达点 4；
   路径较长时可能把行驶时间计入占点时间。
3. `IsRevivedAndDead` 依赖采样到 `HP == 0`。如果 1 Hz 串口数据恰好漏掉归零帧，本次
   复活边沿可能无法识别。
4. `Ros2Input` 已保存各输入时间戳，但决策树尚未用陈旧超时屏蔽过期裁判或视觉数据。
5. `ammo` 和 `enemy_count` 已进入黑板，但当前策略选择没有使用它们。
6. `pitch_mode` 是模式位，不是目标俯仰角；具体角度和动作由 MCU 实现。
7. 内置路线和隧道选择依赖点位 4 的历史目标判断。新增点位或改变位姿后，需要同步复核
   `point_*`、`tunnel_*` 和 `previous_destination` 规则。
8. 本包目前没有独立单元测试；修改后至少执行构建、XML 加载和 `scripts/btview.sh`
   全链路冒烟测试。
