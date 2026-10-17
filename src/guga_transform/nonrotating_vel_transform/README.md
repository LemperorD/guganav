# nonrotating_vel_transform

本功能包用于适配 NAV2 局部路径规划器在 **TES（小陀螺）自旋** 场景下的速度坐标系。

当底盘高速自旋（TES 模式）时，`base_footprint` 坐标系随底盘一起旋转，Nav2 局部路径
规划器会误以为机器人朝向不断变化，导致路径跟踪失效。本功能包发布一个与
`base_footprint` 共原点、但 yaw 保持与 odom 对齐的伪坐标系
`base_footprint_nonrotating`（`base_footprint → base_footprint_nonrotating` 纯旋转 tf），
并把 Nav2 输出的、基于 `base_footprint_nonrotating` 的速度指令旋转补偿回
`base_footprint` 系后发布给底盘，实现自旋与路径跟随解耦。

## 工作模式（由 `chassis_mode` 话题决定，与 simple_decision 枚举对齐）

| chassis_mode | 含义 | yaw 补偿来源 | 角速度 |
|--------------|------|--------------|--------|
| `0` = chassisFollowed | 云台跟随底盘 | `chassis → base_footprint` 的 yaw（`updateGimbalYaw`） | 直接透传，不叠加自旋 |
| `1` = littleTES | 小陀螺自旋 | `base_footprint` 在 odom 下的 yaw（`odometryCallback`/`syncCallback`） | 叠加 `cmd_spin` 自旋速度 |
| `2` = goHome | 回补给点 | 同 littleTES | 叠加 `cmd_spin` 自旋速度 |

> **默认 littleTES（导航启动即小陀螺）**：节点启动时 `chassis_mode` 取
> `initial_chassis_mode`，自旋速度取 `init_spin_speed`，因此导航栈一启动底盘即可
> 自旋，不依赖决策节点。这两个值按控制器差异写在
> `guga_bringup/config/<mode>/controller/<profile>.yaml`：mppi/mpc 为 `1` /
> `1.0 rad/s`；pid 为 `0` / `0.0`（其 costmap 仍用旋转的 `base_footprint`，不能自旋）。
> 之后 `chassis_mode` / `cmd_spin` 话题（如 `simple_decision`）可覆盖这两个值。

tf 发布规则：`base_footprint → base_footprint_nonrotating` 的旋转角在
chassisFollowed 模式下取 `-chassis_yaw`，其余模式取 `-robot_base_angle`。

> **高频重发补的是"角度"，不是速度值**：控制器在 `base_footprint_nonrotating` 系
> 下发的速度（大小/方向）原样保留，`publishCommand()` 每个 tick 调用
> `estimateRobotBaseAngle()` 取当前 yaw，再用 `rotateVelocity()` 把同一条速度转到
> `base_footprint`/`chassis` 系。littleTES/goHome 下 yaw 在 odom（约 10 Hz）之间按
> `spin_speed * dt * correct_ratio` 线性外推，因此 60 Hz 的每一帧在底盘系里都是新
> 角度；nonrotating 系内的控制指令与控制器预测都不受影响。

由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），
humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳
对齐。因此，本功能包使用 message_filters 的 ApproximateTime 策略同步
`odom` 与 `local_plan`，将 local_plan 的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳
对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - `base_footprint → base_footprint_nonrotating` 纯旋转变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 旋转补偿并叠加自旋后的速度指令（底盘系）
* `vis_cmd_vel_topic` (`visualization_msgs/msg/Marker`) - cmd_vel 可视化箭头（默认 `cmd_vel_marker`）

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - Nav2 输出的速度指令（nonrotating 系）
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据（提供底盘 yaw）
* `local_plan_topic` (`nav_msgs/msg/Path`) - 局部路径规划器的路径（用于时间戳对齐）
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 底盘固定自旋速度，littleTES/goHome 模式下叠加到输出
* `chassis_mode_topic` (`std_msgs/msg/UInt8`) - 底盘模式（0=chassisFollowed, 1=littleTES, 2=goHome）

## Parameters

实车参数在 [`guga_bringup/config/reality/base.yaml`](../../guga_bringup/config/reality/base.yaml)
的 `nonrotating_vel_transform:` 段，仿真参数在
[`guga_bringup/config/simulation/base.yaml`](../../guga_bringup/config/simulation/base.yaml)；
`initial_chassis_mode` / `init_spin_speed` 与控制器相关，写在
`config/<mode>/controller/<profile>.yaml`。下表 default 指节点代码里的默认值。

* `robot_base_frame` (`string`, default: `base_footprint`) - 底盘速度参考坐标系
* `nonrotating_robot_base_frame` (`string`, default: `base_footprint_nonrotating`) - 伪速度参考坐标系（yaw 与 odom 对齐）
* `chassis_frame` (`string`, default: `chassis`) - 云台/底盘关节坐标系，用于 chassisFollowed 模式下的 yaw 补偿
* `odom_topic` (`string`, default: `odom`) - 里程计话题
* `local_plan_topic` (`string`, default: `local_plan`) - 局部路径规划器的路径话题
* `cmd_spin_topic` (`string`, default: `cmd_spin`) - 控制底盘固定旋转速度的话题
* `chassis_mode_topic` (`string`, default: `chassis_mode`) - 底盘模式话题
* `initial_chassis_mode` (`int`, default: `1`) - **启动时的底盘模式，默认 littleTES（导航启动即小陀螺）**；之后由 `chassis_mode_topic`（如 `simple_decision`）覆盖。按控制器差异配置：mppi/mpc 为 1（costmap 走 `base_footprint_nonrotating`），pid 为 0（costmap 走旋转的 `base_footprint`，不能自旋）
* `input_cmd_vel_topic` (`string`, default: `""`) - 输入速度指令的话题（launch 中通常为 `cmd_vel_smoothed`）
* `output_cmd_vel_topic` (`string`, default: `""`) - 输出速度指令的话题（launch 中通常为 `cmd_vel`）
* `init_spin_speed` (`float`, default: `3.14`) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度（rad/s，约每秒一圈）；实车 mppi/mpc 配置为 `1.0`，pid 为 `0.0`
* `publish_frequency` (`double`, default: `50.0`) - 速度指令重发频率（Hz）；只按该频率重发最新指令并重新做旋转补偿，不再限速/滤波。实车/仿真配置为 `60.0`；运行时 `ros2 param set` 修改会重建定时器、立即生效
* `cmd_vel_timeout` (`double`, default: `0.5`) - 超过该时长没有新指令就把线性速度归零；角速度按 `chassis_mode` 保留（littleTES 下继续自旋）
* `enable_spin` (`bool`, default: `true`) - 是否允许本节点改写输出角速度（叠加 `cmd_spin` 的自旋）；false 时 `angular.z` 原样透传，可运行时 `ros2 param set` 切换
* `correct_ratio` (`double`, default: `1.2`) - `odom` 低频（约 10 Hz）时用 `spin_speed * dt` 外推底盘 yaw 的修正系数，实车按旋转补偿滞后/超前微调
* `output_in_chassis_frame` (`bool`, default: `false`) - 输出是否直接变换到 `chassis_frame`（默认 false，输出到 `robot_base_frame`）
* `vis_cmd_vel_topic` (`string`, default: `cmd_vel_marker`) - 速度可视化 Marker 话题
* `vis_frame_id` (`string`, default: `base_footprint`) - 可视化参考坐标系
* `vis_scale` (`double`, default: `1.0`) - 可视化箭头长度缩放
