# rog_map_layer — ESDF Nav2 Costmap Layer 插件

> 基于 ROG-Map 思想实现的欧几里得有符号距离场（ESDF）Nav2 costmap 层插件

## 概述

`rog_map_layer` 是 guganav 导航系统中的一个 **Nav2 costmap layer 插件**，负责从 master costmap 的障碍物信息计算 **欧几里得有符号距离场（Euclidean Signed Distance Field, ESDF）**，并将距离代价写入 master grid。

与 Nav2 自带的 `inflation_layer`（阶梯式代价膨胀）相比，ESDF 提供：
- **精确的欧几里得距离**（而非曼哈顿/切比雪夫近似）
- **连续的距离梯度** `∇d`，供梯度优化型规划器/控制器使用
- **灵活的查询 API**：`getDistance()` / `getGradient()`

**灵感来源**：[ROG-Map](https://github.com/hku-mars/ROG-Map)（HKU MaRS Lab, IROS 2024）— 基于 robocentric 滑动窗口的增量 ESDF。

---

## 架构

```
┌─────────────────────────────────────────────────┐
│              Nav2 Costmap Pipeline               │
│                                                  │
│  static_layer → intensity_voxel_layer             │
│                     ↓                            │
│              esdf_layer  ← (本插件)               │
│                     ↓                            │
│              inflation_layer                      │
└─────────────────────────────────────────────────┘
```

### 编码模式：模式 B（对象式封装）

```
EsdfConfig（不可变参数） → EsdfMap（内部状态） → EsdfLayer（Nav2 插件）
     +                                                  +
EsdfParallelExecutor（TBB 线程池）            IncrementalUpdate（静态工具）
```

### 核心类

| 类 | 职责 |
|----|------|
| `EsdfLayer` | Nav2 Layer 插件接口（`onInitialize` / `updateBounds` / `updateCosts` / `matchSize` / `reset`） |
| `EsdfMap` | 距离场 + 梯度场存储，提供 `computeFull()` / `computeIncremental()` / 查询 API |
| `EsdfConfig` | 不可变参数 struct（ESDF 距离 / TBB 并行 / 输出控制） |
| `EsdfParallelExecutor` | TBB `task_arena` + `parallel_for` / `parallelFor2D` 封装 |
| `IncrementalUpdate` | 增量更新静态工具集（差分检测 / 脏区域膨胀 / 重置） |

---

## 算法

### 距离变换：8SED

使用 **8-point Signed Euclidean Distance Transform**（Felzenszwalb & Huttenlocher）：

1. **初始化**：障碍物格 (cost ≥ `LETHAL_OBSTACLE` = 254) → offset `(0,0)`，其他 → `(INT16_MAX, INT16_MAX)`
2. **Forward pass**（左上→右下）：检查 4 个邻居 `(-1,-1), (0,-1), (1,-1), (-1,0)`，传播最近障碍物的 `(dx, dy)` 偏移量
3. **Backward pass**（右下→左上）：检查 4 个邻居 `(1,0), (-1,1), (0,1), (1,1)`
4. **转换**：`distance = sqrt(dx² + dy²) × resolution`，截断至 `max_distance`
5. **梯度**：中心差分 `∇d = [∂d/∂x, ∂d/∂y]`

复杂度 O(N)，N 为网格单元总数。对 100×100 local costmap (5m, 0.05m) 约 0.05ms。

### TBB 分块并行 8SED

```
┌──────────────────────────────────┐
│  Tile 0,0   │  Tile 0,1  │ ...  │  ← TBB parallel_for 并行
│  独立 8SED   │  独立 8SED  │      │
├─────────────┼────────────┼──────┤
│  Tile 1,0   │  Tile 1,1  │ ...  │
│  独立 8SED   │  独立 8SED  │      │
└──────────────────────────────────┘
     ↓ (接缝修正，可配置迭代次数)
  距离值截断 → 梯度计算
```

### 增量更新

避免每帧全量重算：

1. **差分检测**：`current_occupancy vs previous_occupancy` → 变化 cell 列表（TBB 并行比较）
2. **脏区域膨胀**：对每个变化格，标记 `max_distance / resolution` 半径内的邻居
3. **局部重算**：重置脏区域的偏移量，全图转发+反向传播
4. **全量兜底**：变化率 > `full_update_ratio`（默认 30%）时回退全量计算

### 距离→代价映射

```
dist ≤ 0      → LETHAL_OBSTACLE (254)
dist ≥ max_d  → FREE_SPACE (0)
0 < dist < max_d → INSCRIBED (253) × (1 - dist/max_d)  线性衰减
```

---

## 文件结构

```
rog_map_layer/
├── CMakeLists.txt
├── package.xml
├── costmap_plugins.xml          # pluginlib 注册到 nav2_costmap_2d::Layer
├── include/rog_map_layer/
│   ├── esdf_config.hpp          # EsdfConfig 参数结构体
│   ├── esdf_layer.hpp           # EsdfLayer 插件声明
│   ├── esdf_map.hpp             # EsdfMap 距离场声明
│   ├── esdf_incremental.hpp     # IncrementalUpdate 增量工具
│   └── esdf_parallel.hpp        # EsdfParallelExecutor TBB 封装
└── src/
    ├── esdf_layer.cpp           # 插件接口实现 + pluginlib 注册
    ├── esdf_map.cpp             # 8SED 距离变换 + 梯度计算 + 并行化
    ├── esdf_incremental.cpp     # 增量更新实现
    └── esdf_parallel.cpp        # TBB task_arena 实现
```

---

## 依赖

| 依赖 | 用途 |
|------|------|
| `nav2_costmap_2d` | Layer 基类、Costmap2D、cost_values |
| `nav2_util` | LifecycleNode |
| `rclcpp` / `rclcpp_lifecycle` | ROS2 节点/生命周期 |
| `nav_msgs` | OccupancyGrid 消息（可视化发布） |
| `pluginlib` | 插件注册 |
| `tf2_ros` | 坐标变换 |
| `libtbb-dev` (oneTBB) | 多线程并行（`parallel_for`, `task_arena`） |

---

## 参数配置

在 `nav2_params.yaml` 中：

```yaml
esdf_layer:
  plugin: rog_map_layer::EsdfLayer
  enabled: true

  # ESDF 距离参数
  max_distance: 2.0              # 最大计算距离 (m)
  obstacle_threshold: 254        # 障碍物判定 (LETHAL_OBSTACLE)

  # 更新策略
  incremental_update: true       # 启用增量更新
  full_update_ratio: 0.3         # 变化率阈值（超此比例全量重算）

  # TBB 并行
  enable_parallel: true          # 启用多线程
  num_threads: 3                 # 线程数（默认 = 核心数 - 1）
  tile_size: 32                  # 分块大小 (cells)
  seam_iterations: 2             # 接缝修正迭代次数

  # 输出
  publish_esdf_grid: true        # 发布 ESDF 可视化 OccupancyGrid
  write_to_master: true          # 将 ESDF 代价写入 master grid
```

### Local vs Global Costmap 参考配置差异

| 参数 | Local (5m×5m, 10Hz) | Global (可变, 5Hz) |
|------|---------------------|---------------------|
| `max_distance` | 2.0 | 3.0 |
| `publish_esdf_grid` | true (调试) | false |
| `tile_size` | 32 | 64 |

---

## Nav2 集成

### layer 顺序

```yaml
plugins: ["static_layer", "intensity_voxel_layer", "esdf_layer", "inflation_layer"]
```

ESDF 层放在 obstacle 层之后、inflation 层之前。可选移除 `inflation_layer`，由 ESDF 直接提供距离代价。

### 外部查询 API

规划器（MINCO）和控制器（MPC）可通过 `LayeredCostmap` 获取 ESDF 数据：

```cpp
auto* layered = costmap_ros_->getLayeredCostmap();
for (auto& plugin : *layered->getPlugins()) {
  auto esdf = std::dynamic_pointer_cast<rog_map_layer::EsdfLayer>(plugin);
  if (esdf) {
    float dist = esdf->getDistance(world_x, world_y);
    auto [gx, gy] = esdf->getGradient(world_x, world_y);
  }
}
```

### 可视化 Topic

当 `publish_esdf_grid: true` 时，发布：
- Topic: `<layer_name>/esdf_grid`（默认 `esdf_layer/esdf_grid`）
- 类型: `nav_msgs/OccupancyGrid`
- 含义: 0 = 超出范围（白），100 = 障碍物（黑），中间灰度 = 距离衰减

在 RViz 中添加 `Map` 显示，订阅对应 topic 即可。

---

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rog_map_layer --symlink-install
```

---

## TODO / 后续计划

- [ ] SIMD 向量化（SSE/NEON）加速浮点密集操作
- [ ] GPU 加速（CUDA/OpenCL）支持更大规模场景
- [ ] ESDF 层直接替代 inflation_layer（移除并存模式）
- [ ] 规划器（MINCO smoother）接入 ESDF 梯度
- [ ] MPC 控制器接入 ESDF 避障约束
- [ ] 单元测试（Google Test）
- [ ] TBB 分块并行正确性量化验证（串行 vs 并行结果对比）

---

## 参考

- ROG-Map: <https://github.com/hku-mars/ROG-Map>
- Fast-Planner ESDF: <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>
- Felzenszwalb & Huttenlocher (2012): Distance Transforms of Sampled Functions
- Nav2 Costmap: <https://docs.nav2.org/configuration/packages/costmap-plugins/index.html>
