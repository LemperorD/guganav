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
| odom 世界系 | `point.z` 绝对值 | `terrain_voxel_cloud` 的存量、`planar_voxel_elev` 的数值、`estimateTerrainGround` 的地板 `ground_floor_z` |
| 车辆系 | `relative_z = point.z − state_.vehicle_z` | `ingestLaserCloud` 裁剪带、`computeHeightMap` 的地板 `min_relative_z` |
| 地面系 | `point.z − planar_voxel_elev[cell]` | `computeHeightMap` 的净空判据与 `height_above_ground`、输出 intensity |

### 风险 1（已部分修复）：前置筛选带宽随距离放宽、净空曾是常数

`ingestLaserCloud` 的裁剪带是 `min/max_relative_z ± disRatioZ × distance`，**带宽随
距离放宽**（近处 ≈ 0，远处 5 m 处 ±1.0 m）。而 `estimateTerrainGround` 原有的净空
上界是**常数**，会把抬升的地面按车高砍掉（坡面失效）。

**该净空上界已移除**（见下节风险 3），地面候选现只受绝对地板 `ground_floor_z` 约束。
但裁剪带仍是车辆系、且近处带宽极窄（约 0.2 m），故 `vehicle_z` 的标定误差在**近处**
仍会直接决定"地面点能否进入管线"——远处因带宽放宽而被掩盖。这与仓库根
`docs/TODOLIST.md` 记录的"近处低地面点被忽略"直接相关。

### 风险 2：候选筛选与地面估计互为前提（循环依赖）

筛候选想用"高出局部地面多少"，但局部地面 `elev` 正需要候选才能算。
当前用车辆系绕开了这个循环，代价就是风险 1。

### 风险 3（已修复）：一个参数兼两种语义 → 现仅用于障碍输出

`CEILING_CLEARANCE` 原先同时表示"隧道净空"（车体属性）与"地面候选高度上限"
（地形属性），调其一必动另一。现已从 `estimateTerrainGround` 移除，**只由
`computeHeightMap` 使用**，语义唯一：距**局部地面**达到该值的点不作为障碍输出。

它也不再与 `max_relative_z` 竞争"上界"角色（后者现仅用于 `ingestLaserCloud`
与 `keepTerrainVoxelPoint`）。

### 风险 4（已修复）：`maxRelZ` 曾是死配置

原在 `estimateTerrainGround` 与 `computeHeightMap` 中，`relative_z >= max_relative_z`
永不生效（更紧的净空上界先行）。两处条件均已移除；`maxRelZ` 现仅在
`ingestLaserCloud` 与 `keepTerrainVoxelPoint` 生效，不再是死参数。

### 功能性风险：`CEILING_CLEARANCE` 同时是"可输出障碍的高度上限"

它现在是障碍输出的硬上界，即**距地面高于 0.1 m 的点一律不输出为障碍**。这与
`vehicle_height`（默认 1.5，yaml 0.5，"低于此值才算障碍"）的意图不一致——实际
生效的是更严的 0.1。对地形分析用途这个值偏小，会导致**除脚踝以下全部漏检**。
隧道场景下 0.1 有裕量（实测顶隙约 260 mm），但开阔场地应重新评估。

### 风险 5：索引相对、数值绝对

`planar_point_elev[cell]` 的行列下标来自 `point − vehicle`（相对），
压入的却是 `point.z`（odom 绝对）。自洽（后续两个绝对值相减），
但极易被改成 `point.z − vehicle_z` 而全错。

### 建议的统一方向（尚未实施）

1. **判据统一到地面系**：障碍高度与地面候选筛选都以"距局部地面"为准；
2. **两遍法破循环**：先用最低点估粗地面（最低点对障碍不敏感、坡面稳定），
   再以粗地面为基准筛候选，最后用分位数正式估 `elev`；
3. ~~**拆开 `ceilingClearance`**~~ **已完成**：净空判据已从地面估计阶段移除，
   现只用于障碍输出（见风险 3）；
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
