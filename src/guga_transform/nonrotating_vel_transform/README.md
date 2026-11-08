# nonrotating_vel_transform

速度坐标系变换节点，把 Nav2 输出到伪坐标系 `base_footprint_nonrotating` 的速度指令旋转补偿回
`base_footprint`（或 `chassis`）系，并叠加底盘自旋角速度后发布给串口驱动。

## 架构

主线只有一条：参数配置与回调注册 → `publishCommand()` 定时重发速度 → `publishTransform()` 定时发布 nonratate 坐标系 tf。
详细步骤如下：

1. `src/nonrotating_vel_transform.cpp` — 订阅、模式判断、变换与发布都在这里。
   - 初始化：`onConfigure()` 声明并读取全部参数；`enable_spin`、`enable_vis`、`publish_frequency` 注册运行时
     参数回调，可在 `ros2 param set` 后立即生效。
   - 循环：`cmd_vel_pub_timer_` 每 `1/publish_frequency` 秒执行一次 `publishCommand()`；`tf_pub_timer_` 每 20 ms
     发布伪坐标系 tf；`tf_sub_timer_` 每 50 ms 查询一次 `chassis → base_footprint` 的 yaw。

2. `include/nonrotating_vel_transform/nonrotating_vel_transform.hpp` — 节点类与底盘模式枚举。

文件列表：

- `src/nonrotating_vel_transform.cpp`：节点实现。
- `include/nonrotating_vel_transform/nonrotating_vel_transform.hpp`：节点类声明，以及与 `simple_decision` 对齐的
  `ChassisMode` 枚举（0=chassisFollowed，1=littleTES，2=goHome）。
- `test/nonrotating_vel_transform_test.cpp`：`selectVelocityYawDiff()` 与 `rotateVelocity()` 的单元测试。

## 数据流

```
上游输入：
  odometry（point_lio）、local_plan（controller_server）
  cmd_spin / chassis_mode（simple_decision，可选）
        │
controller_server / behavior_server
  │ cmd_vel_controller
velocity_smoother
  │ cmd_vel_smoothed
nonrotating_vel_transform
  ├─(cmd_vel)────────── robot_base（串口驱动 → MCU）
  ├─(tf) ────────────── base_footprint → base_footprint_nonrotating
  └─(cmd_vel_marker)─── RViz
```

## 输入

| 节点                        | Topic              | 类型                         | 来源包/节点                                              |
| --------------------------- | ------------------ | ---------------------------- | -------------------------------------------------------- |
| `nonrotating_vel_transform` | `cmd_vel_smoothed` | `geometry_msgs/Twist`        | `velocity_smoother`                                      |
| `nonrotating_vel_transform` | `odometry`         | `nav_msgs/Odometry`          | `point_lio`                                              |
| `nonrotating_vel_transform` | `local_plan`       | `nav_msgs/Path`              | `controller_server`                                      |
| `nonrotating_vel_transform` | `cmd_spin`         | `example_interfaces/Float32` | `simple_decision`（可选，缺省用 `init_spin_speed`）      |
| `nonrotating_vel_transform` | `chassis_mode`     | `std_msgs/UInt8`             | `simple_decision`（可选，缺省用 `initial_chassis_mode`） |

## 输出

| 节点                        | Topic            | 类型                        | 坐标系                                                             | 下游订阅者            | 用途                                 |
| --------------------------- | ---------------- | --------------------------- | ------------------------------------------------------------------ | --------------------- | ------------------------------------ |
| `nonrotating_vel_transform` | `cmd_vel`        | `geometry_msgs/Twist`       | `base_footprint`（`output_in_chassis_frame` 为 true 时 `chassis`） | `robot_base` 串口驱动 | 旋转补偿并叠加自旋后的速度指令       |
| `nonrotating_vel_transform` | `tf`             | `tf2_msgs/TFMessage`        | `base_footprint → base_footprint_nonrotating`                      | 导航栈、RViz          | 伪坐标系（纯旋转）                   |
| `nonrotating_vel_transform` | `cmd_vel_marker` | `visualization_msgs/Marker` | 同 `cmd_vel` 的坐标系                                              | RViz                  | 速度箭头可视化，由 `enable_vis` 控制 |

`cmd_vel` 是唯一有控制作用的话题，`tf` 供导航栈在 nonrotating 系内做规划，`cmd_vel_marker` 只用于显示。

## 工作模式

底盘模式由 `chassis_mode` 话题决定，与 `simple_decision` 的枚举对齐：

| chassis_mode          | 含义         | yaw 补偿来源                                                           | 角速度                   |
| --------------------- | ------------ | ---------------------------------------------------------------------- | ------------------------ |
| `0` = chassisFollowed | 云台跟随底盘 | `chassis → base_footprint` 的 yaw（`updateGimbalYaw`）                 | 直接透传，不叠加自旋     |
| `1` = littleTES       | 小陀螺自旋   | `base_footprint` 在 odom 下的 yaw（`odometryCallback`/`syncCallback`） | 叠加 `cmd_spin` 自旋速度 |
| `2` = goHome          | 回补给点     | 同 littleTES                                                           | 叠加 `cmd_spin` 自旋速度 |

节点启动时 `chassis_mode` 取 `initial_chassis_mode`、自旋速度取 `init_spin_speed`，因此导航栈一启动底盘即可自旋，
不依赖决策节点。这两个值按控制器差异写在 `src/guga_bringup/config/<mode>/controller/<profile>.yaml`：mppi/mpc 为
`1` / `1.0 rad/s`；pid 为 `0` / `0.0`（其 costmap 仍用旋转的 `base_footprint`，不能自旋）。之后 `chassis_mode` /
`cmd_spin` 话题（如 `simple_decision`）可覆盖这两个值。

tf 发布规则：`base_footprint → base_footprint_nonrotating` 的旋转角在 chassisFollowed 模式下取
`-chassis_yaw`，其余模式取 `-robot_base_angle`。

高频重发补的是“角度”，不是速度值：控制器在 `base_footprint_nonrotating` 系下发的速度（大小/方向）原样保留，
`publishCommand()` 每个 tick 调用 `estimateRobotBaseAngle()` 取当前 yaw，再用 `rotateVelocity()` 把同一条速度
转到 `base_footprint`/`chassis` 系。littleTES/goHome 下 yaw 在 odom 两次更新之间（实车约 50 Hz、仿真约 10 Hz）按
`spin_speed * dt * correct_ratio` 线性外推，因此 60 Hz 的每一帧在底盘系里都是新角度；nonrotating 系内的控制
指令与控制器预测都不受影响。

Nav2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用
TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包使用 message_filters 的
ApproximateTime 策略同步 `odom` 与 `local_plan`，将 local_plan 的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## 管线

本节讲解各函数作用。

| 位置                          | 函数                     | 职责                                                                             |
| ----------------------------- | ------------------------ | -------------------------------------------------------------------------------- |
| nonrotating_vel_transform.cpp | `onConfigure`            | 声明并读取参数，注册 `enable_spin`/`enable_vis`/`publish_frequency` 的运行时回调 |
| nonrotating_vel_transform.cpp | `cmdVelCallback`         | 缓存上游最新一条速度指令与到达时刻，不直接发布                                   |
| nonrotating_vel_transform.cpp | `cmdSpinCallback`        | 更新 `spin_speed_`（littleTES/goHome 下叠加到输出的自旋角速度）                  |
| nonrotating_vel_transform.cpp | `chassisModeCallback`    | 更新 `chassis_mode_`，决定 yaw 来源与是否叠加自旋                                |
| nonrotating_vel_transform.cpp | `localPlanCallback`      | 记录 `local_plan` 到达时刻，作为控制器已激活的标志                               |
| nonrotating_vel_transform.cpp | `odometryCallback`       | 控制器未激活（距 `local_plan` 超 0.5 s）时，用 odom 更新底盘 yaw                 |
| nonrotating_vel_transform.cpp | `syncCallback`           | `odom`+`local_plan` 同步回调，统一以同步到的 odom 更新 yaw 与时间戳              |
| nonrotating_vel_transform.cpp | `updateGimbalYaw`        | 每 50 ms 查询 `chassis → base_footprint` 的 yaw，供 chassisFollowed 补偿         |
| nonrotating_vel_transform.cpp | `estimateRobotBaseAngle` | 返回当前底盘 yaw；littleTES/goHome 下按 `spin_speed * dt * correct_ratio` 外推   |
| nonrotating_vel_transform.cpp | `selectVelocityYawDiff`  | 按模式选用 `chassis_followed_yaw_` 或底盘 yaw 作为补偿角                         |
| nonrotating_vel_transform.cpp | `rotateVelocity`         | 用二维旋转矩阵作用于 `linear.x`/`linear.y`                                       |
| nonrotating_vel_transform.cpp | `transformVelocity`      | 组合旋转与角速度改写（叠加自旋或原样透传）                                       |
| nonrotating_vel_transform.cpp | `publishCommand`         | 每个 tick 取最新指令、算补偿角并发布；超时后线速度归零、角速度按模式保留         |
| nonrotating_vel_transform.cpp | `publishTransform`       | 发布 `base_footprint → base_footprint_nonrotating` 的纯旋转 tf                   |
| nonrotating_vel_transform.cpp | `visualizeVelocity`      | 发布线速度与角速度两条 RViz Marker 箭头                                          |

`transformVelocity()` 是速度指令的最终出口：`output_in_chassis_frame` 为 true 时按
`chassis_followed_yaw_ - yaw_diff` 旋转到 `chassis` 系，否则只做 yaw 补偿、输出仍在 `base_footprint` 系；
`enable_spin` 为 false 时 `angular.z` 原样透传，不叠加 `spin_speed_`。

## 参数

参数来自 `src/guga_bringup/config/<profile>/base.yaml` 的 `nonrotating_vel_transform:` 段（实车与仿真各一份）；
`initial_chassis_mode` 与 `init_spin_speed` 与控制器相关，写在 `config/<mode>/controller/<profile>.yaml`。
下表默认值指节点代码里的默认值，共 19 个：

| 参数                           | 作用                                                                                                   |
| ------------------------------ | ------------------------------------------------------------------------------------------------------ |
| `robot_base_frame`             | 底盘速度参考坐标系（默认 `base_footprint`）                                                            |
| `nonrotating_robot_base_frame` | 伪速度参考坐标系，yaw 与 odom 对齐（默认 `base_footprint_nonrotating`）                                |
| `chassis_frame`                | 云台/底盘关节坐标系，chassisFollowed 模式下用它做 yaw 补偿（默认 `chassis`）                           |
| `odom_topic`                   | 里程计话题（默认 `odom`）                                                                              |
| `local_plan_topic`             | 局部路径话题，用于时间戳对齐与控制器激活判定（默认 `local_plan`）                                      |
| `cmd_spin_topic`               | 底盘固定自旋速度话题（默认 `cmd_spin`）                                                                |
| `chassis_mode_topic`           | 底盘模式话题（默认 `chassis_mode`）                                                                    |
| `input_cmd_vel_topic`          | 输入速度指令话题，launch 中通常为 `cmd_vel_smoothed`（代码默认空字符串）                               |
| `output_cmd_vel_topic`         | 输出速度指令话题，launch 中通常为 `cmd_vel`（代码默认空字符串）                                        |
| `initial_chassis_mode`         | 启动时的底盘模式（默认 `1`，即 littleTES，导航启动即小陀螺）；之后由 `chassis_mode_topic` 覆盖         |
| `init_spin_speed`              | 未收到 `cmd_spin_topic` 时的固定自旋角速度，rad/s（默认 `3.14`；实车 mppi/mpc 为 `1.0`，pid 为 `0.0`） |
| `publish_frequency`            | 速度指令重发频率，Hz（默认 `50.0`；实车/仿真配置为 `60.0`）；运行时修改会重建定时器、立即生效          |
| `cmd_vel_timeout`              | 超过该时长没有新指令就把线速度归零，角速度按 `chassis_mode` 保留（默认 `0.5` s）                       |
| `enable_spin`                  | 是否允许叠加 `cmd_spin` 的自旋；false 时 `angular.z` 原样透传（默认 `true`，可运行时切换）             |
| `correct_ratio`                | 用 `spin_speed * dt` 外推底盘 yaw 的修正系数，按旋转补偿滞后/超前微调（默认 `1.2`）                    |
| `output_in_chassis_frame`      | 输出是否直接变换到 `chassis_frame`（默认 `false`，输出到 `robot_base_frame`）                          |
| `enable_vis`                   | 是否发布 `cmd_vel_marker` 可视化箭头；false 时不构造 Marker（默认 `true`，可运行时切换）               |
| `vis_cmd_vel_topic`            | 速度可视化 Marker 话题（默认 `cmd_vel_marker`）；Marker 坐标系由 `output_in_chassis_frame` 决定        |
| `vis_scale`                    | 可视化箭头长度缩放（默认 `1.0`）                                                                       |

注意：`init_spin_speed` 只在节点启动后、收到第一条 `cmd_spin` 之前生效；决策节点未启动（`reality_launch.py` 默认
`use_decision:=false`）时，自旋速度一直取该值。

## 测试

```bash
# 构建并运行本包测试（pre-commit 钩子与 CI 都调用它）
scripts/pre-commit/run_nonrotating_tests.sh
```

当前 `nonrotating_vel_transform` 测试套件：

- `nonrotating_vel_transform_test`：覆盖 `selectVelocityYawDiff()` 的模式选择与 `rotateVelocity()` 的二维旋转，
  不涉及话题订阅、定时器与发布行为。

本包已纳入 `.pre-commit-config.yaml`
