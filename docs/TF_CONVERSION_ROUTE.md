# TF 转换线路

本文记录当前项目中 TF 的发布者、转换位置和 Nav2 使用的坐标链。路径和行号以源码为准；仿真命名空间示例为 `red_standard_robot1`。

## 1. Nav2 期望的主链

标准导航链应为：

```text
map
`-- odom                         定位结果
    `-- base_footprint           机器人位姿
        `-- chassis              固定安装关系
            `-- gimbal_yaw_odom  云台里程计关节
                `-- gimbal_pitch_odom
                    `-- gimbal_yaw
```

当前 MPPI 配置还插入了：

```text
gimbal_yaw
`-- gimbal_yaw_fake               fake_vel_transform 动态发布
```

Nav2 当前把 `gimbal_yaw_fake` 配置为 `robot_base_frame`，见 [nav2_params_mppi.yaml](../src/guga_bringup/config/simulation/nav2_params_mppi.yaml) 中的 `bt_navigator`、两个 costmap 和 `behavior_server` 配置。

## 2. 各段 TF 的来源

### `map -> odom`

- 正常定位：`small_gicp_relocalization` 发布，见 [small_gicp_relocalization.cpp](../src/guga_localization/small_gicp_relocalization/src/small_gicp_relocalization.cpp)。`publishTransform()` 设置 `header.frame_id = map`、`child_frame_id = odom`。
- SLAM 模式：`slam_launch.py` 启动 `tf2_ros/static_transform_publisher` 发布零变换 `map -> odom`，见 [slam_launch.py](../src/guga_bringup/launch/core/slam_launch.py)。

### `odom -> base_footprint` 和 `odom -> gimbal_yaw`

`sensor_scan_generation` 订阅：

```text
lidar_odometry       loam_interface 输出
registered_scan      loam_interface 输出的点云
```

在 [sensor_scan_generation.cpp](../src/guga_perception/sensor_scan_generation/src/sensor_scan_generation.cpp) 中：

1. 将 `lidar_odometry.pose` 作为 `odom -> front_mid360`。
2. 调用 `lookupTransform(front_mid360, base_footprint)` 和
   `lookupTransform(front_mid360, gimbal_yaw)`，取得 `T_lidar_base` 和
   `T_lidar_gimbal`。
3. 计算：

   ```text
   odom -> base_footprint = (odom -> lidar) * (lidar -> base_footprint)
   odom -> gimbal_yaw     = (odom -> lidar) * (lidar -> gimbal_yaw)
   ```

4. 通过 `TransformBroadcaster` 发布上述两段 TF。
5. 同时发布 `odometry` 话题，消息的 `child_frame_id` 当前为 `gimbal_yaw`。

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
odometry: pose 转到 gimbal_yaw，并发布 odom -> base_footprint/gimbal_yaw
```

Point-LIO 默认只发布 `aft_mapped_to_init` 话题；其 TF 发布开关 `tf_send_en` 在当前配置中关闭，因此 `loam_interface` 负责把激光里程计接入项目的 `odom` 系。

## 4. `gimbal_yaw_fake` 的转换

实现见 [fake_vel_transform.cpp](../src/guga_controller/fake_vel_transform/src/fake_vel_transform.cpp)：

```text
输入 TF: chassis -> gimbal_yaw
输出 TF: gimbal_yaw -> gimbal_yaw_fake
```

默认 `chassisFollowed` 模式下，输出 yaw 为 `-yaw(chassis -> gimbal_yaw)`，所以：

```text
yaw(chassis -> gimbal_yaw_fake) = 0
```

这使 fake frame 的方向接近底盘方向，但它仍位于云台链上。非 `chassisFollowed` 模式下，代码使用 odometry 中的 `odom -> gimbal_yaw` yaw，行为会切换为接近世界/odom 对齐。

fake 节点还将 `cmd_vel_nav2_result` 转换后发布到 `cmd_vel`，并处理 `cmd_spin`、`chassis_mode`、超时直通和角速度反向。

## 5. 速度坐标系现状

当前速度链为：

```text
MPPI/controller_server
    -> cmd_vel_controller
velocity_smoother
    -> cmd_vel_nav2_result
fake_vel_transform
    -> cmd_vel
底盘控制器
```

`sensor_scan_generation::publishOdometry()` 用相邻 `odom -> gimbal_yaw` 位姿差分计算 twist，但没有将线速度旋转到 `child_frame_id` (`gimbal_yaw`)；当前写入的 `linear.x/y` 实际仍是 `odom` 轴。这是一个坐标语义错误，不能仅靠 fake TF 修复。

此外，仿真 `rmua19_robot_base` 还发布 `robot_base/odom`，其 TF 和 twist 来自 Gazebo 真值，见 [odometry_publisher.cpp](../src/guga_sim/rmoss_gazebo/rmoss_gz_base/src/odometry_publisher.cpp)。Nav2 配置订阅的是相对话题 `odometry`，不是 `robot_base/odom`。

## 6. 运行时检查命令

仿真启动后，在对应命名空间执行：

```bash
ros2 topic info -v /red_standard_robot1/odometry
ros2 topic echo --once /red_standard_robot1/odometry
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo odom gimbal_yaw
ros2 run tf2_ros tf2_echo gimbal_yaw gimbal_yaw_fake
```

注意：话题带机器人命名空间，例如 `/red_standard_robot1/odometry`；TF 的
`frame_id` 在当前模型中仍是 `odom`、`base_footprint` 等未加命名空间的名称。

重点确认：

- `/odometry` 的发布者是否只有 `sensor_scan_generation`。
- `Odometry.child_frame_id` 与 `twist` 实际坐标系是否一致。
- 是否同时存在两个节点发布 `odom -> base_footprint`。
- fake 开启时，`gimbal_yaw_fake` 的 yaw 是否随底盘模式发生跳变。

## 7. 简要结论

对于 MPPI Omni，推荐最终使用 `base_footprint` 作为 `robot_base_frame`，修正 `odometry.twist` 到同一底盘坐标系，再把底盘模式和 `cmd_spin` 逻辑放到底盘适配层。`gimbal_yaw_fake` 不是 Nav2 必需 TF；它是为独立云台和旧版 `Twist` 接口设计的兼容层。
