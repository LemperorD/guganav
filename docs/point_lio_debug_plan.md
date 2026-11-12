# Point-LIO 漂移排查计划

## 目标

原地匀速旋转 1 分钟，`lidar_odometry` 的 yaw 跟随真实角速度约 1.9 rad/s，位置漂移小于 0.1 米。

## 当前状态

- 仓库在 `main` 分支，是原版 Point-LIO。
- 新版本在 `point_lio` 分支，`stash@{0}` 保存了改动。
- `spin_debug` 是新版本录的包，点云缺 `timestamp` 字段，原版回放会发散。
- 无头 Gazebo 启动崩溃，仿真需要图形显示。

## 步骤

### 第一步：确认原版在仿真里的行为

在有显示器的终端启动：

```bash
cd ~/guganav
scripts/simulation.sh nav rmul_2025 use_rviz:=False
```

确认点云有发布者：

```bash
ros2 topic info /red_standard_robot1/livox/lidar
```

`Publisher count` 大于 0 后，录 10 秒：

```bash
ros2 bag record -o ~/bags/orig_live /red_standard_robot1/lidar_odometry
```

判断标准：读包看 yaw 和 pos。yaw 跟随、pos 稳定说明原版正常；yaw 不跟随或 pos 漂移说明原版也漂，问题在配置或数据。

### 第二步：修 `spin_debug` 的点云时间字段

`spin_debug` 的点云字段是 `x, y, z, intensity, ring`，缺 `timestamp`。`preprocess.cpp` 读 `pl_orig.points[i].timestamp` 算时间偏移，字段缺失后时间全是 0。

确认仿真里点云的真实类型：

```bash
ros2 topic info /red_standard_robot1/livox/lidar -v
```

如果原始类型是 `livox_ros_driver2/msg/CustomMsg`，录包时直接录 `CustomMsg`，原版 `lidar_type: 1` 能用。如果已经是 `PointCloud2`，需要在录包前把 `timestamp` 补进点云。

### 第三步：用同一份包分别跑两个版本

用带 `timestamp` 的包，只放传感器数据：

```bash
ros2 bag play ~/bags/spin_fixed --clock \
  --topics /red_standard_robot1/livox/lidar /red_standard_robot1/livox/imu
```

分别启动原版和新版本节点，各录一份 `lidar_odometry`。对比 yaw、pos，新版本额外录 `/point_lio/debug_state` 看 `ba`。

### 第四步：修 `ba` 漂移

新版本的 `ba` 在 80 秒后漂到 `(0.39, -0.08, -1.38)`，`acc - ba` 变大，Z 速度增加，位置掉到 -7.34 米。

把 `ba` 固定为 0：

```cpp
// FrameProcessor::initializeFilter
P.middleRows<3>(12).setZero();
P.middleCols<3>(12).setZero();

// ManifoldEskf::F_x 里删掉
d.block<3, 3>(6, 12) = -dt * x_.rot;
```

### 第五步：验证

重新编译、启动仿真、原地旋转 1 分钟，录 `lidar_odometry` 和 `debug_state`。

判断标准：

- yaw 从初值线性增长，斜率约 1.9 rad/s。
- `pos` 的 Z 变化小于 0.1 米。
- `ba` 保持 0，`vel` 的 Z 分量在零附近。
