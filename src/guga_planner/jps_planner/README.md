# jps_planner

`jps_planner` 是当前默认使用的 Nav2 全局规划器插件，插件名为
`jps_planner/JPSPlanner`。它在 costmap 上执行 Jump Point Search（JPS）搜索，
再对 JPS 跳点做 B-spline 平滑，并可接入 ESDF 距离场进行避障优化。

## 架构

`jps_planner` 通过 `nav2_core::GlobalPlanner` 接口接入 Nav2，由 `planner_server`
在每次规划请求时调用。搜索算法是无状态的纯静态实现，节点/插件层负责接线、
参数注入、B-spline 调用与结果回退。

```
include/jps_planner/
├── jps_planner.hpp        # Nav2 GlobalPlanner 插件接口与 createPlan 编排
├── jps_algorithm.hpp      # JPS 搜索公开 API 与后处理
└── jps_node.hpp           # SearchNode / JPSConfig / JPSState 数据结构

src/
├── jps_planner_node.cpp   # configure / cleanup / activate / deactivate / linearInterpolation
├── jps_algorithm.cpp      # generatePath 主循环 / detourCornerHuggingDiagonals
└── detail/
    ├── jps_obstacle.cpp   # 可通行判定、代价缩放、启发式与欧氏代价
    ├── jps_neighbor.cpp   # 强制邻居检测与邻居裁剪
    ├── jps_jump.cpp       # 递归跳跃与后继节点识别
    └── jps_path.cpp       # createPlan / bsplineSmooth / 坐标转换 / 碰撞校验
```

主线只有一条：Nav2 调用 `createPlan()` → JPS 搜索 → 贴障碍对角修正 → 短路径补密 →
B-spline 平滑 → 碰撞校验 → 返回 `nav_msgs::Path`。

## 数据流

```
start / goal (world)
  │
  ├─ costmap_->worldToMap()
  │
  ├─ JPSAlgorithm::generatePath()        # 稀疏跳点，格元中心坐标
  │
  ├─ detourCornerHuggingDiagonals()       # 贴障碍对角段改写为水平/垂直移动
  │
  ├─ densifyMapPath()                     # ≤2 格补密，至少凑足 8 个 B-spline 航点
  │
  ├─ bsplineSmooth()
  │    ├─ 查找 LayeredCostmap 中的 EsdfLayer（enable_esdf 时）
  │    ├─ BSplineOptimizer::fit()          # chord-length 参数化 + 精确插值
  │    ├─ 注入 costmap / ESDF 数据
  │    └─ BSplineOptimizer::optimize()     # 可选梯度下降 + 障碍物投射 + 采样
  │
  ├─ mapContinuousToWorld()               # 连续地图坐标 → 世界坐标
  │
  └─ isPathCollisionFree()                # 碰撞校验
       ├─ 平滑路径碰撞 → 回退线性 JPS 路径
       └─ 最终路径仍碰撞 → 返回空路径
            │
            └─ nav_msgs::Path
```

## 输入 / 输出

`jps_planner` 不直接订阅 Topic，作为 Nav2 规划插件由 `planner_server` 调用。

### 输入

| 接口 | 类型 | 来源 |
| ---- | ---- | ---- |
| `createPlan(start, goal)` | `geometry_msgs/PoseStamped` | Nav2 `planner_server` |
| `costmap_ros` | `nav2_costmap_2d::Costmap2DROS` | Nav2 代价地图 |
| `LayeredCostmap` 中的 `EsdfLayer` | `rog_map_layer::EsdfLayer` | 可选，供 B-spline 软避障 |

### 输出

| 输出 | 类型 | 坐标系 | 下游 |
| ---- | ---- | ------ | ---- |
| `createPlan()` 返回值 | `nav_msgs/Path` | `global_frame` | Nav2 controller / behavior server |
| 共享内存 PATH 槽位 | `guga_ui::UiPath` | 世界坐标 | Pangolin UI 渲染 |

## 代价地图约定

| 值 | 含义 |
| ---- | ---- |
| 0 | 空闲空间 |
| 1–252 | 有代价空间，越高越不倾向通行 |
| 253 | 内切膨胀障碍物，搜索视为不可通行 |
| 254 | 致命障碍物 |
| 255 | 未知空间，`allow_unknown=false` 时视为不可通行 |

- 搜索障碍阈值：`cost >= 253`。
- 对角移动会检查两个相邻边格，禁止从两个阻断格之间斜穿。
- B-spline 平滑路径的碰撞阈值默认 `collision_cost_threshold=253`；调成 254 后只把
  `LETHAL` 当障碍，允许平滑曲线擦过 `INSCRIBED` 格，回退更少。

## 管线

| 位置 | 函数 | 职责 |
| ---- | ---- | ---- |
| `jps_planner_node` | `configure` | 声明并读取 `GridBased.*` ROS 参数，初始化 costmap 与共享内存写入端 |
| `jps_planner_node` | `linearInterpolation` | 把地图坐标路径按分辨率线性插值成 `nav_msgs::Path` |
| `jps_algorithm` | `generatePath` | JPS 主循环：验证起终点 → 优先队列展开跳点 → 回溯路径 |
| `jps_algorithm` | `detourCornerHuggingDiagonals` | 把贴障碍的对角段改写为正交移动，避免 B-spline 切角产生锯齿 |
| `jps_jump` | `jump` | 沿方向递归跳跃，只在终点 / 强制邻居 / 对角分量跳点处停止 |
| `jps_jump` | `identifySuccessors` | 对当前节点找出所有跳点后继；起点探索 8 方向，其余按父方向裁剪 |
| `jps_jump` | `backtracePath` | 从终点沿父指针回溯，输出格元中心坐标路径 |
| `jps_neighbor` | `pruneNeighbors` | 根据父方向裁剪自然邻居，并追加强制邻居方向 |
| `jps_neighbor` | `hasForcedNeighbor` | 检测水平/垂直/对角线方向的强制邻居 |
| `jps_obstacle` | `isTraversable` / `canStep` | 单格元可通行判定与对角切角合法性 |
| `jps_obstacle` | `scaledCost` / `traversalCost` | 把 raw cost 按 Theta* 公式缩放，并乘 `w_traversal_cost` |
| `jps_path` | `densifyMapPath` | 短路径按弧长补密，目标至少 8 个 B-spline 航点 |
| `jps_path` | `bsplineSmooth` | 查找 ESDF、拟合 B-spline、注入数据、运行优化并转世界坐标 |
| `jps_path` | `isPathCollisionFree` | 对路径点与相邻线段做采样级碰撞校验 |
| `jps_path` | `mapContinuousToWorld` / `mapPathToWorld` | 连续地图坐标到世界坐标，不平滑路径再量化 |
| `jps_planner` | `writePathToShm` | 把降采样后的规划结果写入共享内存供 UI 渲染 |

## 参数

### JPS 搜索参数

| 参数 | 代码默认 | 说明 |
| ---- | ---- | ---- |
| `w_euc_cost` | 1.0 | 跳转点间欧几里得距离代价权重 |
| `w_traversal_cost` | 10.0 | 单格元通行代价权重（Theta* 缩放后） |
| `w_heuristic_cost` | 1.0 | A* 启发式权重，控制贪心程度 |
| `allow_unknown` | false | 是否允许穿越未知空间（`cost=255`） |

### B-spline / ESDF 参数

| 参数 | 代码默认 | 说明 |
| ---- | ---- | ---- |
| `enable_bspline` | true | 是否启用 B-spline 平滑 |
| `enable_esdf` | false（声明值） | 是否启用 ESDF 梯度优化；开启且找到 EsdfLayer 时自动打开梯度下降 |
| `esdf_weight` | 100.0 | ESDF 距离代价权重 |
| `esdf_safe_distance` | 0.6 | 路径距障碍物小于该距离（米）时开始产生惩罚 |
| `corridor_halfwidth` | 8.0 | 内部控制点可移动走廊半宽（格元） |
| `smoothness_weight` | 0.1 | B-spline 曲率能量权重 |
| `distance_weight` | 1.0 | B-spline 偏离原始 JPS 航点权重 |
| `max_control_points` | 200 | B-spline 最大控制点数 |
| `max_iterations` | 200 | 梯度下降最大迭代次数 |
| `collision_cost_threshold` | 253 | 平滑路径碰撞阈值，254 更宽松 |

### 当前 profile 参数

`guga_bringup` 的 `planner/jps.yaml`（仿真普通 / SLAM 与实车各有差异）会覆盖声明默认值：

| 参数 | simulation `jps.yaml` | simulation `jps_slam.yaml` | reality `jps.yaml` |
| ---- | ---- | ---- | ---- |
| `allow_unknown` | false | true | false |
| `enable_bspline` | true | true | true |
| `enable_esdf` | true | true | true |
| `esdf_weight` | 200.0 | 200.0 | 200.0 |
| `esdf_safe_distance` | 1.5 | 1.5 | 1.5 |
| `corridor_halfwidth` | 15.0 | 15.0 | 15.0 |
| `smoothness_weight` | 40.0 | 20.0 | 20.0 |
| `distance_weight` | 0.3 | 0.3 | 0.3 |
| `max_control_points` | 24 | 24 | 24 |
| `max_iterations` | 300 | 300 | 300 |

`corridor_halfwidth` 单位是 costmap 格元；分辨率 0.05 m 时 `15.0` 约等于允许控制点
相对初始路径移动 0.75 m。

### ESDF layer 参数

由 `jps.yaml` 一并覆盖，供 `rog_map_layer::EsdfLayer` 使用：

| 作用域 | `max_distance` | `obstacle_threshold` |
| ---- | ---- | ---- |
| `local_costmap` | 2.0 | 254 |
| `global_costmap` | 3.0 | 254 |

## 测试

```bash
# 构建 jps_planner 并运行测试
colcon build --packages-select jps_planner --event-handlers console_direct+
./build/jps_planner/test_jps

# 生成可视化数据（独立可执行，不需要 ROS）
./build/jps_planner/jps_viz_exporter
```

当前 `jps_planner` 测试套件：

- `test_jps`（gtest，31 项，5 个 suite）：搜索行为、JPS 跳点/强制邻居/对角切角、
  边界与异常、代价地图/未知空间、`isTraversable` 单元行为。
- `jps_viz_exporter`：多场景生成 JPS 搜索与可视化数据。

## 常见现象

### 路径贴墙

优先检查 costmap 膨胀半径、`robot_radius`、`esdf_safe_distance` 和
`corridor_halfwidth`。`esdf_safe_distance` 让优化器提前感知障碍物，
`corridor_halfwidth` 限制优化器把路径推离原始 JPS 路径多远。

### 路径像 8 向移动

先看 planner 日志。当前实现会先补密短路径再进 B-spline，正常会打印：

```text
JPSPlanner: densified path from 3 to N waypoints for B-spline
JPSPlanner: B-spline smooth applied, 100 poses
```

如果仍看到 `too few waypoints ... linear fallback`，通常表示路径极短、
B-spline 拟合失败或平滑路径碰撞后回退。
