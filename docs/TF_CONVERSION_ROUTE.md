# TF 转换线路

本文记录当前项目中 TF 的发布者、转换位置和 Nav2 使用的坐标链。路径和行号以源码为准；仿真命名空间示例为 `red_standard_robot1`。

## 1. Nav2 期望的主链

标准导航链应为：

```text
map
`-- odom                         定位结果
    `-- base_footprint           机器人位姿（底盘）
        `-- chassis              固定安装关系
            `-- gimbal_yaw_odom  云台里程计关节
                `-- gimbal_pitch_odom
                    `-- gimbal_yaw
```

为支持底盘自旋（TES），`nonrotating_vel_transform` 额外发布一个与
`base_footprint` 共原点、朝向不旋转（odom 对齐）的参考系：

```text
base_footprint
`-- base_footprint_nonrotating   nonrotating_vel_transform 动态发布
```

`base_footprint_nonrotating` 被配置为 costmap 的 `robot_base_frame`，使 Nav2
在"扣除底盘自旋"的参考系中导航，控制器输出的速度再由
`nonrotating_vel_transform` 旋转补偿回真实底盘系。

## 2. 各段 TF 的来源

### `map -> odom`

- 正常定位：`small_gicp_relocalization` 发布，见 [small_gicp_relocalization.cpp](../src/guga_localization/small_gicp_relocalization/src/small_gicp_relocalization.cpp)。`publishTransform()` 设置 `header.frame_id = map`、`child_frame_id = odom`。
- SLAM 模式：`slam_launch.py` 启动 `tf2_ros/static_transform_publisher` 发布零变换 `map -> odom`，见 [slam_launch.py](../src/guga_bringup/launch/core/slam_launch.py)。

### `odom -> base_footprint`

`sensor_scan_generation` 订阅：

```text
lidar_odometry       loam_interface 输出
registered_scan      loam_interface 输出的点云
```

在 [sensor_scan_generation.cpp](../src/guga_perception/sensor_scan_generation/src/sensor_scan_generation.cpp) 中：

1. 将 `lidar_odometry.pose` 作为 `odom -> front_mid360`。
2. 调用 `lookupTransform(front_mid360, base_footprint)`，取得
   `T_lidar_base`。当前 `base_frame` 和 `robot_base_frame` 均配置为
   `base_footprint`。
3. 计算：

   ```text
   odom -> base_footprint = (odom -> lidar) * (lidar -> base_footprint)
   ```

4. 通过 `TransformBroadcaster` 发布 `odom -> base_footprint`。
5. 同时发布 `odometry` 话题，其 pose 位于 `odom`，`child_frame_id` 和
   twist 均为 `base_footprint`。

### 机器人内部链

仿真中由 `robot_state_publisher` 根据生成的机器人描述发布；模型定义见 [rm25_example_robot.def.xmacro](../src/guga_sim/rmoss_gz_resources/resource/models/rm25_example_robot/rm25_example_robot.def.xmacro)：

```text
base_footprint -> chassis              fixed
chassis -> gimbal_yaw_odom             revolute
gimbal_yaw_odom -> gimbal_pitch_odom   fixed
gimbal_pitch_odom -> gimbal_yaw        revolute
```

仿真机器人通过 [spawn_robots.launch.py](../src/guga_sim/rmu_gazebo_simulator/rmu_gazebo_simulator/launch/spawn_robots.launch.py) 启动 `robot_state_publisher`，并将 `/tf`、`/tf_static` 重映射到机器人命名空间。

现实导航未使用 robot state publisher 时，由 [static_tf_publisher_launch.py](../src/guga_bringup/launch/support/static_tf_publisher_launch.py) 发布固定的机体、云台和雷达安装关系。

## 3. 输入里程计的转换位置

```text
Point-LIO: aft_mapped_to_init
    ↓ loam_interface
lidar_odometry: pose 转到 odom，child = front_mid360
    ↓ sensor_scan_generation
odometry: pose 转到 base_footprint，并发布 odom -> base_footprint
```

Point-LIO 默认只发布 `aft_mapped_to_init` 话题；其 TF 发布开关 `tf_send_en` 在当前配置中关闭，因此 `loam_interface` 负责把激光里程计接入项目的 `odom` 系。

## 4. `nonrotating_vel_transform` 的转换

实现见 [nonrotating_vel_transform.cpp](../src/guga_controller/nonrotating_vel_transform/src/nonrotating_vel_transform.cpp)：

```text
输入 TF: odom -> base_footprint（由 sensor_scan_generation 发布）
输出 TF: base_footprint -> base_footprint_nonrotating
```

`base_footprint_nonrotating` 与 `base_footprint` 共原点，但朝向扣除了机器人在
odom 系的 yaw，使 Nav2 看到的机器人朝向恒为 odom 对齐（`yaw = 0`）。这为
底盘自旋（TES）提供了"非旋转"的导航参考系。

`chassis_mode` 决定扣减的 yaw 来源：

- `chassisFollowed`：扣 `yaw(chassis -> robot_base_frame)`（云台转角）。
- `littleTES`（TES 自旋）：扣 `odometry` 中机器人（base_footprint）的 yaw。

该节点还订阅 `cmd_vel_smoothed`（控制器经 velocity_smoother 输出的速度），
旋转补偿到真实底盘系后发布 `cmd_vel`，并处理 `cmd_spin`（自旋角速度叠加）与
`chassis_mode`（模式切换）。

## 5. 速度坐标系现状

速度链为：

```text
controller_server
    -> cmd_vel_controller
velocity_smoother
    -> cmd_vel_smoothed
nonrotating_vel_transform（旋转补偿 + 叠加自旋）
    -> cmd_vel
串口驱动 -> MCU
```

`simple_decision` 发布 `chassis_mode` 与 `cmd_spin`：决策层进入 `LITTLE_TES`
模式时，`nonrotating_vel_transform` 将 `spin_speed_`（来自 `cmd_spin` 话题）
叠加到输出角速度，实现底盘自旋。

`sensor_scan_generation::publishOdometry()` 先对相邻 `odom -> base_footprint`
位姿做差，得到 `odom` 轴的线速度和角速度，再乘当前姿态的逆旋转，将其
转换到 `child_frame_id` (`base_footprint`) 后写入 twist。这符合
`nav_msgs/Odometry` 对 twist 坐标系的定义。

`controller_server.odom_topic` 已显式配置为相对话题 `odometry`。
在机器人命名空间下，它订阅 `/red_standard_robot1/odometry`，不会再回退到
Nav2 默认的 `odom` 话题。

此外，仿真 `rmua19_robot_base` 还发布 `robot_base/odom`，其 TF 和 twist 来自 Gazebo 真值，见 [odometry_publisher.cpp](../src/guga_sim/rmoss_gazebo/rmoss_gz_base/src/odometry_publisher.cpp)。Nav2 配置订阅的是相对话题 `odometry`，不是 `robot_base/odom`。

## 6. 运行时检查命令

仿真启动后，在对应命名空间执行：

```bash
ros2 topic info -v /red_standard_robot1/odometry
ros2 topic echo --once /red_standard_robot1/odometry
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_footprint_nonrotating
ros2 param get /red_standard_robot1/controller_server odom_topic
```

注意：话题带机器人命名空间，例如 `/red_standard_robot1/odometry`；TF 的
`frame_id` 在当前模型中仍是 `odom`、`base_footprint` 等未加命名空间的名称。

重点确认：

- `/odometry` 的发布者是否只有 `sensor_scan_generation`。
- `Odometry.child_frame_id` 与 `twist` 实际坐标系是否一致。
- 是否同时存在两个节点发布 `odom -> base_footprint`。
- `controller_server` 的 `odom_topic` 是否为 `odometry`。
- `base_footprint -> base_footprint_nonrotating` 是否由 `nonrotating_vel_transform` 发布。

## 7. 简要结论

当前配置（MPPI 与 MPC profile 统一）使用 `base_footprint` 作为
`robot_base_frame`，`nonrotating_vel_transform` 发布
`base_footprint -> base_footprint_nonrotating` 作为 Nav2 的导航参考系。
costmap / bt_navigator / behavior_server 均使用 `base_footprint_nonrotating`，
控制器在"扣除底盘旋转"的参考系中导航，输出由 `nonrotating_vel_transform`
旋转补偿回真实底盘系。
