# point_LIO

LIO, 雷达-惯性里程计, 接受雷达与IMU的数据, 通过构造流形卡尔曼滤波, 逐点更新小车的位姿, 同时将点云更新至 odometry 坐标系下并下采样, 以供后续节点使用, 是整个导航系统的开端.

## 架构
![point_LIO 主架构：数据流](../../../../docs/img/pointLIO_data.svg)
![point_LIO 主架构：逻辑流](../../../../docs/img/pointLIO_logic.svg)

启动节点为`src/guga_localization/LIO/point_lio/src/ros2/laserMapping.cpp`,
1. 构造节点类时, 注册 yaml 参数, 将其通过 datahandler 类分发至各类中, 之后完成 ros2 publisher 与 subscription 的注册.
2. 完成全局初始化后,进行标定: 先通过 imu 与重力来初始化标定z轴的方向, 然后累积点云进行初始化点云的发布.
3. 准备工作就绪,开始循环: 获取imu/雷达数据 -> 卡尔曼滤波更新传播 -> 发布所需数据


## 数据流

```
point_lio (雷达里程计系)
  │
  └─ 内部左乘安装变换 (base <- lidar), 换到 odom 系
     │
     └─(lidar_odometry)──── terrain_analysis
        (registered_scan)    │
                             ├─(terrain_map)─────────── local_costmap / global_costmap
                             │                          (obstacle_layer, pb_nav2_plugins)
                             │                          └ SLAM 模式：pointcloud_to_laserscan
                             │                            → slam_toolbox (map)
                             └─(terrain_returns_current) local_costmap / global_costmap
                                                        (obstacle_layer, 只做清除)
```

## 输入

| 来源包/节点               | Topic                    | 消息类型                  | QoS                     |
| ------------------------- | ------------------------ | ------------------------- | ----------------------- |
| `serial_driver`           | `/livox/imu`             | `nav_msgs/Odometry`       | rclcpp::SensorDataQoS() |
| `serial_driver`           | `/livox/lidar`(实车)     | `sensor_msgs/PointCloud2` | rclcpp::SensorDataQoS() |
| `ign_sim_pointcloud_tool` | `/velodyne_points`(实车) | `sensor_msgs/PointCloud2` | rclcpp::SensorDataQoS() |

## 输出

| 节点               | Topic                     | 类型                      | 坐标系 | 下游订阅者                                              | 用途                                           |
| ------------------ | ------------------------- | ------------------------- | ------ | ------------------------------------------------------- | ---------------------------------------------- |
| `terrain_analysis` | `terrain_map`             | `sensor_msgs/PointCloud2` | `odom` | `local_costmap` 的 `obstacle_layer`                     | 标记源（`intensity` 为距局部地面的高度）       |
|                    |                           |                           |        | `global_costmap` 的 `obstacle_layer`                    | 标记与清除源                                   |
|                    |                           |                           |        | `pointcloud_to_laserscan`（仅 SLAM 模式）               | 按 `intensity` 0.1~2.0 过滤后交给 slam_toolbox |
| `terrain_analysis` | `terrain_returns_current` | `sensor_msgs/PointCloud2` | `odom` | `local_costmap` 与 `global_costmap` 的 `obstacle_layer` | 清除源（`intensity` 恒 0，无高度语义）         |

`terrain_map` 与 `terrain_returns_current` 都由 `terrain_analysis` 发布，坐标系都是 `odom`，下游都是 `pb_nav2_costmap_2d::ObstacleLayerLocal`：前者供标记，后者只供清除。曾在 R2 设计里新增的 `terrain_obstacles_current`（当帧障碍云）因为没有任何消费者，已删除。

## 网格参数

| 网格                | 分辨率 | 尺寸  | 说明                     |
| ------------------- | ------ | ----- | ------------------------ |
| PersistentVoxelGrid | 1.0 m  | 21×21 | 滑动窗口，累积多帧点云   |
| PerFrameHeightGrid  | 0.2 m  | 51×51 | 随车逐帧重建，估地面高度 |

## 管线
本节讲解各函数作用。


| 位置                  | 函数                     | 职责                                                                                                                         |
| --------------------- | ------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| terrain_analysis_node | `processOnce`            | 一帧驱动：有帧时 `update` → `collectCloud` → 后半段两条入口 → 发布；10 ms 定时器调用一次                                     |
| persistent_voxel_map  | `receiveFrame`           | 收帧：按接收带与接收半径裁剪本帧点云，记下雷达位置与帧时刻，置待处理标记                                                     |
| persistent_voxel_map  | `update`                 | 本帧三步编排：`rollover` → `addFrame` → `rebuildGrids`，三步共用 `receiveFrame` 记下的同一份帧输入                           |
| persistent_voxel_map  | `rollover`               | 车体移动时滚动 terrain voxel 网格，维持以**雷达**为中心的滑动窗口                                                            |
| persistent_voxel_map  | `addFrame`               | 本帧点云按空间位置分配到 terrain voxel 格子                                                                                  |
| persistent_voxel_map  | `rebuildGrids`           | 逐格每帧重建：按水平 `scanVoxelSize` 0.1 / 垂直 `scanVoxelSizeZ` 0.05 下采样，每个叶只留**最新观测**点，再按接收带与年龄过滤 |
| persistent_voxel_map  | `collectCloud`           | 取出以雷达所在格为中心 11×11 格子的累积地形点                                                                                |
| per_frame_height_map  | `compute`                | 后半段主入口，三步串联：`estimateTerrainGround` → `computePlanarElevation` → `computeHeightMap`                              |
| per_frame_height_map  | `estimateTerrainGround`  | 将点膨胀到 planar voxel（3×3），收集地面高度候选值                                                                           |
| per_frame_height_map  | `computePlanarElevation` | 对每个 planar voxel 估地面高度（分位数 `quantileZ`，或最小值）                                                               |
| per_frame_height_map  | `computeHeightMap`       | 计算每点离地高度并写入 `intensity`，生成累计障碍输出 `terrain_map`                                                           |
| per_frame_height_map  | `computeFrameReturns`    | 由本帧点云生成当帧返回的雷达射线云 `terrain_returns_current`                                                                 |

两条输出链：`terrain_map` 是累计障碍云（`intensity` = 距局部地面的高度），`terrain_returns_current` 是当帧返回的雷达射线云（`intensity` 恒 0，供代价地图射线清除）。






## 参数

参数来自 `src/guga_bringup/config/<profile>/base.yaml` 的 `terrain_analysis:` 段（实车与
仿真各一份），由节点声明后按两半的读取范围分发给两个配置结构体。共 15 个：

| 参数                | 作用                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| `scanVoxelSize`     | 融合叶的水平尺寸（0.1 m）                                            |
| `scanVoxelSizeZ`    | 融合叶的垂直尺寸（0.05 m）；必须比水平细，否则地面点会把矮物体点顶掉 |
| `decayTime`         | 观测的衰减时间：超过它、又在近处之外、且未被重新观测的点会被清掉     |
| `maxRelZ`           | 接收带的上沿（相对雷达高度）                                         |
| `disRatioZ`         | 接收带随水平距离放宽的比例                                           |
| `minRelZ`           | 接收带的下沿（相对雷达高度）；同一参数也是后半段障碍输出的地板       |
| `useSorting`        | 地面高度取分位数（true）还是最低点（false）                          |
| `quantileZ`         | `useSorting` 打开时用的分位点                                        |
| `considerDrop`      | 打开时离地高度取绝对值，凹坑也算障碍                                 |
| `limitGroundLift`   | 是否限制地面估计相对最低点的抬升量                                   |
| `maxGroundLift`     | `limitGroundLift` 打开时的最大抬升量                                 |
| `minBlockPointNum`  | 每格参与判定的最少点数                                               |
| `minObstacleHeight` | 障碍输出下界（地面带死区，距局部地面）                               |
| `ceilingClearance`  | 障碍输出上界（净空，距局部地面；也是唯一上界）                       |
| `groundFloorZ`      | 地面候选的绝对高度地板（odom z），低于它的点不参与地面估计           |

另外注意：`config.hpp` 里的结构体默认值只有部分是实车值——`decay_time`、`quantile_z`、
`max_ground_lift`、`max_relative_z` 与 `base.yaml` 不同，单元测试若直接构造
结构体，跑的是默认值而不是实车工况。


## 测试

```bash
# 单元测试（构建 terrain_analysis 包并运行其测试）
scripts/pre-commit/run_terrain_analysis_tests.sh

# 覆盖率（编译 + 运行 + gcovr 报告）
scripts/test/test_terrain_analysis_coverage.sh
```

| 输出            | 路径                                         |
| --------------- | -------------------------------------------- |
| Html 覆盖率报告 | `build/terrain_analysis/coverage.html`       |
| lcov 信息       | `lcov.info`                                  |
| 测试日志        | `build/terrain_analysis/coverage_result.ans` |

当前 `terrain_analysis` 测试套件：

- `test_terrain_analysis`：完整管线行为
- `test_frame_receive`：前半段的帧输入（雷达位置与帧时刻、首帧时刻、观测时刻写进
  intensity、越界裁剪、`update` 清待处理标记）
- `test_algorithm`：体素、地面高程估计与边界处理
- `test_integration`：只通过 ROS 话题驱动节点（不直接调 receiveFrame / update / compute），覆盖订阅与消息转换、逐帧数据分发、发布消息的 frame_id/stamp，以及跨帧的幽灵点清除与体素窗口滚动

`estimateTerrainGround` 与 `computeHeightMap` 都经 `gridIndex(...)` 统一做越界判定；
超出 `51×51` planar 网格的点会被忽略，避免数组越界。