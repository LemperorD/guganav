# gimbal_cmd_vel_adapter

本功能包是旧 `fake_vel_transform` 的改名版本，用于在独立云台和旧版 Nav2 `Twist` 接口之间做速度坐标适配。当前主导航 bringup 默认不启动该包，`velocity_smoother` 会直接发布最终 `/cmd_vel`。

该包启动时会创建一个 `nav2_robot_base_frame` 坐标系，其 x、y、z 与 `robot_base_frame` 一致，但 yaw 固定到稳定参考方向，并发布到 tf2。同时它订阅 `input_cmd_vel_topic`，把速度指令转换到 `robot_base_frame` 后发布到 `output_cmd_vel_topic`。

主要目的是适配历史方案：当 Nav2 使用的速度参考坐标系随云台快速变化时，局部控制器会把机器人朝向误认为云台朝向，导致轨迹跟踪异常。使用稳定的 `nav2_robot_base_frame` 可以隔离云台 yaw 对 Nav2 控制的影响。

由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的速度指令
* `local_plan_topic` (`nav_msgs/msg/Path`) - 局部路径规划器的路径
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `nav2_robot_base_frame` (`string`, default: "gimbal_link_fake") - 提供给 Nav2 使用的稳定速度参考坐标系
* `local_plan_topic` (`string`, default: "local_plan") - 局部路径规划器的路径话题
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `nav2_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `init_spin_speed` (`double`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度
* `invert_angular_velocity` (`bool`, default: false) - 调试旋转方向时反转最终输出的 `angular.z`
