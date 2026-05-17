# terrain_analysis

局部地形分析节点，从 LiDAR 点云构建高度地图并检测动态障碍物。

## 架构

```
ROS2 消息 → TerrainAnalysisContext → TerrainAlgorithm.run() → publish → ROS2 消息
 (订阅)      (config/state 汇总)       (10 阶段管线)         (发布)     (terrain_map)
```

- **Context 层** (`core/context`) — Config + State 公开 struct，回调接收 odometry/laser/joystick/clearing 消息
- **算法层** (`core/algorithm`) — 纯静态方法，10 阶段管线，无副作用
- **节点层** (`terrain_analysis`) — ROS2 接线，参数声明，组装并驱动上述两层

## 管线

```
rolloverVoxels → voxelize → updateVoxels → extractTerrainCloud
                                                    ↓
                                              estimateGround
                                                    ↓
                                        detectDynamicObstacles
                                        filterDynamicObstaclePoints
                                                    ↓
                                            computeElevation
                                            computeHeightMap
                                            addNoDataObstacles
```

| 阶段 | 职责 |
|------|------|
| `rolloverVoxels` | 车辆移动时滚动体素网格，维持以车辆为中心的滑动窗口 |
| `voxelize` | 当前帧点云按空间位置分配到地形体素格子 |
| `updateVoxels` | 逐个格子降采样 + 时间衰减 + 空间过滤 |
| `extractTerrainCloud` | 提取车辆周边 11×11 格子的累积地形点 |
| `estimateGround` | 点云膨胀到 planar voxel，为后续高度估算准备 |
| `detectDynamicObstacles` | 用仰角 + 传感器 FOV 检测潜在动态障碍 |
| `filterDynamicObstaclePoints` | 当前帧高角度点反向印证，清除头顶固定结构的误报 |
| `computeElevation` | 对每个 planar voxel 估算地面高度（分位数或最小值） |
| `computeHeightMap` | 计算每个点离地高度，生成输出点云 |
| `addNoDataObstacles` | 为数据稀疏区域生成虚拟障碍物 |

## 测试

```bash
# 覆盖率（编译 + 运行 + gcovr 报告）
scripts/test/test_terrain_analysis_coverage.sh
```

| 输出 | 路径 |
|------|------|
| Html 覆盖率报告 | `build/terrain_analysis/coverage.html` |
| lcov 信息 | `lcov.info` |
| 测试日志 | `test_result.ans` |

## 网格参数

| 网格 | 分辨率 | 尺寸 | 说明 |
|------|--------|------|------|
| Terrain voxel | 1.0m | 21×21 | 滑动窗口，累积多帧地形点 |
| Planar voxel | 0.2m | 51×51 | 固定窗口，估算地面高度 |

## 输出

- **Topic**: `/terrain_map` (`sensor_msgs/PointCloud2`)
- **坐标系**: odom
- **Intensity 字段**: 点离地高度（米）
