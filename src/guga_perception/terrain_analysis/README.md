# terrain_analysis

局部地形分析节点，从 LiDAR 点云构建高度地图并检测动态障碍物。

## 架构

```
ROS2 消息 → TerrainProcessor::ingest* → TerrainProcessor::run() → publish → ROS2 消息
 (订阅)          (写入内部 state_)          (9 阶段私有管线)          (发布)    (terrain_map)
```

- **TerrainProcessor**（`core/terrain_processor.hpp`）— 自持 `TerrainConfig` + `TerrainState`，
  对外只暴露 `ingestOdometry` / `ingestLaserCloud` / `run` / `terrainCloudElev` 等入口，
  管线各阶段为私有成员，可自由重构而不影响调用方
- **节点层**（`terrain_analysis_node.*`）— 仅做 ROS 接线：声明参数、订阅、定时驱动与发布
- 白盒测试经 `friend` 访问 `TerrainProcessor` 的内部阶段（见 `test_algorithm.cpp` 的 `runStage`）

## 管线

```
rolloverTerrainVoxels → voxelizeTerrain → updateTerrainVoxels → collectTerrainCloud
                                                                      ↓
                                                        estimateTerrainGround
                                                                      ↓
                                                  detectDynamicObstacles
                                                  filterDynamicObstaclePoints
                                                                      ↓
                                                     computePlanarElevation
                                                          computeHeightMap
```

| 阶段                          | 职责                                               |
| ----------------------------- | -------------------------------------------------- |
| `rolloverTerrainVoxels`       | 车辆移动时滚动 terrain voxel 网格，维持以车辆为中心的滑动窗口 |
| `voxelizeTerrain`             | 当前帧点云按空间位置分配到 terrain voxel 格子      |
| `updateTerrainVoxels`         | 逐个格子降采样 + 时间衰减 + 空间高度过滤           |
| `collectTerrainCloud`         | 收集车辆周边 11×11 格子的累积地形点                |
| `estimateTerrainGround`       | 点膨胀到 planar voxel（3×3），收集地面高度候选值   |
| `detectDynamicObstacles`      | 用仰角 + 传感器 FOV 统计潜在动态障碍               |
| `filterDynamicObstaclePoints` | 当前帧高角度点反向印证，清除头顶固定结构的误报     |
| `computePlanarElevation`      | 对每个 planar voxel 估地面高度（分位数 `quantileZ`，或最小值） |
| `computeHeightMap`            | 计算每点离地高度，写入 intensity 生成输出点云      |

## 测试

```bash
# 单元测试（构建两个 terrain 包并运行 terrain_analysis 测试）
scripts/pre-commit/run_terrain_analysis_tests.sh

# 覆盖率（编译 + 运行 + gcovr 报告）
scripts/test/test_terrain_analysis_coverage.sh
```

| 输出            | 路径                                   |
| --------------- | -------------------------------------- |
| Html 覆盖率报告 | `build/terrain_analysis/coverage.html` |
| lcov 信息       | `lcov.info`                            |
| 测试日志        | `build/terrain_analysis/coverage_result.ans` |

当前 `terrain_analysis` 测试套件：

- `test_terrain_analysis`：完整管线行为
- `test_state_ingest`：里程计、点云和清除状态接收
- `test_algorithm`：体素、地面估计、动态障碍和边界处理

`estimateTerrainGround` 经 `voxelIndexOf(VoxelGrid::PLANAR, …)` 统一做越界判定；
超出 `51×51` planar grid 的点会被忽略，避免数组越界。

## 网格参数

| 网格          | 分辨率 | 尺寸  | 说明                     |
| ------------- | ------ | ----- | ------------------------ |
| Terrain voxel | 1.0m   | 21×21 | 滑动窗口，累积多帧地形点 |
| Planar voxel  | 0.2m   | 51×51 | 固定窗口，估算地面高度   |

## 已知风险：坐标系不统一

管线内部同时使用**三个参考系**，且没有在类型或命名上区分——这是当前最容易被误改的地方。

| 参考系 | 定义 | 使用位置 |
| ------ | ---- | -------- |
| odom 世界系 | `point.z` 绝对值 | `terrain_voxel_cloud` 的存量、`planar_voxel_elev` 的数值 |
| 车辆系 | `relative_z = point.z − state_.vehicle_z` | `ingestLaserCloud` 裁剪带、`estimateTerrainGround` 的两个过滤 |
| 地面系 | `point.z − planar_voxel_elev[cell]` | `computeHeightMap` 的 `height_above_ground`、输出 intensity |

### 风险 1：前置筛选（车辆系）与判据（地面系）错位 —— 上坡时会失效

`ingestLaserCloud` 的裁剪带带坡度补偿 `z_margin = disRatioZ × distance`，
但 `estimateTerrainGround` 的 `ceiling_clearance` 过滤是**常数、不随距离放宽**：

```cpp
if (relative_z >= config_.ceiling_clearance) continue;   // 车辆系，0.2 固定
```

后果：上坡时车前 5 m 处地面相对车可达 +1 m（`disRatioZ` 注释本身预期了这一点），
这些地面点**在候选筛选阶段就被剔除**，该处 `planar_voxel_elev` 无候选或偏差，
后续地面系的高度判据也就失去正确基准。即"点云收进来了，又被 ceiling 扔掉"。

### 风险 2：候选筛选与地面估计互为前提（循环依赖）

筛候选想用"高出局部地面多少"，但局部地面 `elev` 正需要候选才能算。
当前用车辆系绕开了这个循环，代价就是风险 1。

### 风险 3：一个参数兼两种语义

`ceilingClearance` 同时表示：

- 隧道能否从下方通过的**车顶净空**（车体属性）；
- 地面候选的**高度上限**（地形属性）。

因此为修坡面而调它，会同时改变隧道通过性。YAML 注释也承认了双重用途
（"不算障碍**且不参与地面估计**"）。

### 风险 4：`maxRelZ` 在本阶段是死配置

当 `ceilingClearance < maxRelZ`（当前 `0.2 < 0.5`）时，两个条件都在约束
`relative_z` 的**上界**，前者严格更紧，故 `relative_z >= max_relative_z` 永不生效。
实测：把 `maxRelZ` 由 0.5 放到 5.0，输出逐点不变。

⇒ 调 `maxRelZ` 对地面估计无任何影响，容易被误认为在生效。
（`maxRelZ` 在 `ingestLaserCloud` 里仍然有效，不能删。）

### 风险 5：索引相对、数值绝对

`planar_point_elev[cell]` 的行列下标来自 `point − vehicle`（相对），
压入的却是 `point.z`（odom 绝对）。自洽（后续两个绝对值相减），
但极易被改成 `point.z − vehicle_z` 而全错。

### 建议的统一方向（尚未实施）

1. **判据统一到地面系**：障碍高度与地面候选筛选都以"距局部地面"为准；
2. **两遍法破循环**：先用最低点估粗地面（最低点对障碍不敏感、坡面稳定），
   再以粗地面为基准筛候选，最后用分位数正式估 `elev`；
3. **拆开 `ceilingClearance`**：车顶净空（隧道）与地面候选高度上限分离，
   使二者可独立调参；
4. 车辆系只保留给**车辆/传感器自身的量**：点云裁剪窗口（车体属性），
   以及以局部地面为基准表达的车顶净空。

### 无需担心的前提

"地面系"依赖 odom z 的**跨帧相对精度**（`elev` 跨帧累积）。当前场景
**不存在长距离下坡**，故长期漂移不构成风险；`point.z` 与 `elev` 同为 odom 量，
其差值天然抵消平移分量。

## 输入

| 节点                 | Topic             | 类型                      | 来源包/节点                  |
| -------------------- | ----------------- | ------------------------- | ---------------------------- |
| `terrainAnalysis`    | `lidar_odometry`  | `nav_msgs/Odometry`       | `point_lio → loam_interface` |
| `terrainAnalysis`    | `registered_scan` | `sensor_msgs/PointCloud2` | `point_lio → loam_interface` |
| `terrainAnalysisExt` | `lidar_odometry`  | `nav_msgs/Odometry`       | `point_lio → loam_interface` |
| `terrainAnalysisExt` | `terrain_map`     | `sensor_msgs/PointCloud2` | `terrainAnalysis` (本包)     |

## 输出

| 节点                 | Topic             | 类型                      | 坐标系 | 下游订阅者                                                  |
| -------------------- | ----------------- | ------------------------- | ------ | ----------------------------------------------------------- |
| `terrainAnalysis`    | `terrain_map`     | `sensor_msgs/PointCloud2` | `odom` | `local_costmap` (intensity_voxel_layer, `pb_nav2_plugins`)  |
|                      |                   |                           |        | `terrainAnalysisExt` (本包)                                 |
| `terrainAnalysisExt` | `terrain_map_ext` | `sensor_msgs/PointCloud2` | `odom` | `global_costmap` (intensity_voxel_layer, `pb_nav2_plugins`) |
|                      |                   |                           |        | `pointcloud_to_laserscan` (`guga_perception`, 仅 SLAM 模式) |

## 数据流

```
point_lio (cloud_registered, aft_mapped_to_init)
  │
loam_interface 
  │
  ├─(lidar_odometry)──── terrainAnalysisExt(terrain_map_ext)
  │                                 │  
  │                                 ├─── global_costmap
  │                                 │    (intensity_voxel_layer, pb_nav2_plugins)
  │                                 │
  │                                 └─── pointcloud_to_laserscan
  │                                      (obstacle_scan)
  │                                             └─ slam_toolbox (map)
  │                                                         (仅 SLAM 模式)
  │
  └─(lidar_odometry)──── terrainAnalysis(terrain_map)
     (registered_scan)    │
                          ├─── local_costmap
                          │    (intensity_voxel_layer, pb_nav2_plugins)
                          │
                          └─── terrainAnalysisExt
                               (terrain_map_ext)── (见上)
```

## 两个节点

|          | `terrainAnalysis` (`terrain_analysis_node.cpp`) | `terrainAnalysisExt` (`terrain_analysis_ext_node.cpp`) |
| -------- | ---------------------------------------------- | ----------------------------------------------------- |
| 输入     | 原始激光点云 + 里程计                          | `lidar_odometry` + `terrain_map`                      |
| 参数默认 | Nav2 参数文件统一配置                          | 仅 `localTerrainMapRadius`                            |
| 用途     | 主地形分析，局部 costmap                       | 按半径裁剪近场地形，全局 costmap + SLAM 建图          |

> 注意：`terrainAnalysisExt` 目前**只做半径过滤**（`mergeLocalTerrain`），
> 原版 CMU 的远场累积与 `checkTerrainConn` 连通性判定整体缺失，且
> `localTerrainMapRadius` 的语义与上游相反（原版用于**排除**近场并输出远场）。
> 详见仓库根 `docs/TODOLIST.md`。
