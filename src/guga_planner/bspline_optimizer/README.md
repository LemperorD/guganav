# bspline_optimizer

把 JPS 搜索得到的稀疏折线航点平滑为 7 阶 B-spline 曲线，并通过几何优化
（平滑 + 距离保持 + ESDF 避障）与障碍物投射，输出一条光滑、无碰撞、曲率有界的全局路径。

## 架构

`bspline_optimizer` 不是 ROS 节点，而是一个纯 C++ 库。公开 API 在
`bspline_optimizer.hpp` / `bspline_optimizer.cpp`，代价函数、梯度下降、栅格查询等
内部实现拆到 `src/detail/` 与 `include/bspline_optimizer/detail/`。

```
include/bspline_optimizer/
├── bspline_optimizer.hpp        # 公开 API：BSplineConfig / State / Result / Optimizer
├── basis_cache.hpp              # 基函数行缓存与代价项预计算入口
└── detail/
    ├── cost_cache.hpp           # 带状基函数行缓存（BandRow / CostCache）
    ├── cost_function.hpp        # 三项代价、归一化标度与解析梯度
    ├── gradient_descent.hpp     # 共轭梯度下降
    ├── grid_utils.hpp           # 代价地图障碍判定与螺旋搜索投射
    └── esdf_utils.hpp           # ESDF 距离/梯度双线性查询

src/
├── bspline_optimizer.cpp        # fit / optimize / sample / curvatureAt / rebuildSpline
└── detail/
    ├── basis.cpp                # Cox-de Boor 基函数求值
    ├── cost_cache.cpp           # 预计算平滑/距离/ESDF 三条基函数行缓存
    ├── cost_function.cpp        # evalTerms / evalCost / computeGradient
    ├── gradient_descent.cpp     # Polak-Ribière 共轭梯度 + 回溯线搜索 + 走廊约束
    ├── grid_utils.cpp           # cellCost / inObstacle / projectPointToFree
    └── esdf_utils.cpp           # esdfDistanceAt / esdfGradientAt
```

主线只有一条：`fit()` 构造样条 → `optimize()` 做几何优化与硬避障 → `sample()` 输出结果。

## 数据流

```
JPS 航点 (map 坐标)
  │
  ├─ fit(path)
  │    ├─ chord-length 参数化 τ_i = arc_i / L
  │    ├─ Eigen::SplineFitting::Interpolate(pts, 7) → 精确插值样条
  │    ├─ 按弧长比例映射到最近原始航点，重采样 M 个控制点
  │    └─ Eigen::KnotAveraging → 节点向量，重建样条
  │
  ├─ 注入 costmap_data / ESDF distance+gradient 指针
  │
  └─ optimize(num_samples)
       ├─ 可选：共轭梯度下降优化内部控制点（平滑 + 距离 + ESDF）
       ├─ 控制点障碍物投射（螺旋搜索）
       ├─ rebuildSpline() + sample()
       └─ 采样点投射 + ≤0.5 格线段加密修正
            │
            └─ BSplineResult
```

## 对外输入 / 输出

`bspline_optimizer` 不订阅 ROS Topic，通过函数调用传参。

### 输入

| 输入                   | 类型                                    | 说明                                          |
| ---------------------- | --------------------------------------- | --------------------------------------------- |
| `fit(path)`            | `std::vector<std::pair<double,double>>` | 地图坐标下的 JPS 航点，≥8 个才走完整 B-spline |
| `state().costmap_data` | `const unsigned char*`                  | 可选，cost ≥ 253 视为障碍物，用于投射         |
| `state().esdf_*`       | `const float*` / 尺寸 / 原点 / 分辨率   | 可选，ESDF 距离场与梯度场，用于软避障梯度     |

### 输出

`optimize(num_samples)` 返回 `BSplineResult`：

| 字段                          | 说明                                       |
| ----------------------------- | ------------------------------------------ |
| `smoothed_path`               | 地图坐标下的采样路径，数量 = `num_samples` |
| `curvature_profile`           | 每个采样点的曲率                           |
| `control_points_xy`           | 优化后的控制点                             |
| `total_curvature_energy`      | 积分曲率能量                               |
| `cost_initial` / `cost_final` | 优化前后代价（当前实现为曲率能量）         |
| `iterations` / `converged`    | 梯度下降迭代次数与收敛标志                 |

## 管线

| 位置                | 函数                                | 职责                                                                               |
| ------------------- | ----------------------------------- | ---------------------------------------------------------------------------------- |
| `bspline_optimizer` | `fit`                               | 构造样条：chord-length 参数化 → 精确插值 → 重采样控制点 → 节点平均；短路径线性回退 |
| `bspline_optimizer` | `optimize`                          | 后半段主入口：可选梯度下降 → 控制点投射 → 重建 → 采样点投射与加密修正              |
| `bspline_optimizer` | `rebuildSpline`                     | 用当前控制点与节点向量重建 Eigen `Spline2D`                                        |
| `bspline_optimizer` | `sample`                            | 在均匀参数 `u∈[0,1]` 处采样当前样条；未拟合或线性回退时插值端点                    |
| `bspline_optimizer` | `curvatureAt`                       | 计算参数 `u` 处曲率 κ                                                              |
| `bspline_optimizer` | `computeCurvatureEnergy`            | 数值积分 `∫₀¹‖C''(u)‖² du`                                                         |
| `basis_cache`       | `buildCostCache`                    | 预计算平滑（二阶导）、距离、ESDF 三条基函数行缓存                                  |
| `cost_function`     | `evalTerms`                         | 计算平滑/距离/ESDF 三项原始代价                                                    |
| `cost_function`     | `evalCost`                          | 三项代价按初始值归一化后加权求和                                                   |
| `cost_function`     | `computeGradient`                   | 计算三项代价对内部控制点的解析梯度                                                 |
| `gradient_descent`  | `gradientDescent`                   | Polak-Ribière 共轭梯度 + 回溯线搜索 + 走廊约束                                     |
| `grid_utils`        | `projectPointToFree`                | 落在障碍格元内的点用螺旋搜索投射到最近空闲格元                                     |
| `esdf_utils`        | `esdfDistanceAt` / `esdfGradientAt` | 在连续世界坐标处双线性插值 ESDF 距离/梯度                                          |

## 代价函数

启用梯度下降时，总代价为三项加权和，且**每项除以自身初始值**做归一化：

```
J = w_s · (J_smooth / J_s0) + w_d · (J_dist / J_d0) + w_e · (J_esdf / J_e0)
```

- `J_smooth = (1/K) Σ‖C''(u_k)‖²`：曲率能量，让拐弯更舒缓，`K=50`。
- `J_dist = Σ‖C(τ_i) − q_i‖²`：偏离原始 JPS 航点的距离，`τ_i` 为 chord-length 参数。
- `J_esdf = Σ max(0, d_safe − d_esdf(p))²`：ESDF 软避障，仅在注入 ESDF 距离场时参与。

## 参数

`BSplineConfig` 各字段（库默认值，`jps_planner` 可用 ROS 参数覆盖）：

| 参数                      | 默认  | 说明                                                           |
| ------------------------- | ----- | -------------------------------------------------------------- |
| `degree`                  | 7     | 样条阶数；当前实现固定为 7（与 `Eigen::Spline` 类型绑定）      |
| `smoothness_weight`       | 0.1   | 平滑代价权重，越大拐弯越圆滑                                   |
| `distance_weight`         | 1.0   | 距离保持权重，越大越贴原路径                                   |
| `esdf_weight`             | 100.0 | ESDF 避障权重                                                  |
| `esdf_safe_distance`      | 0.3   | 距障碍物安全距离（米）                                         |
| `corridor_halfwidth`      | 2.5   | 内部控制点相对初始位置的可移动走廊半宽（格元）                 |
| `max_control_points`      | 200   | 最大控制点数，越小近似越强、拐角越圆                           |
| `max_iterations`          | 200   | 梯度下降最大迭代次数                                           |
| `enable_gradient_descent` | true  | 是否做几何优化                                                 |
| `enable_esdf`             | true  | 是否启用 ESDF 代价（库内实际参与与否取决于是否注入 ESDF 数据） |

> 走廊约束对紧邻端点的控制点额外收紧到 ±0.5 格元，防止终点附近过度弯曲。

## 与 JPSPlanner 集成

`jps_planner` 直接编译本模块源码，规划链路为：

```
createPlan()
  ├─ JPSAlgorithm::generatePath → 稀疏跳点
  ├─ detourCornerHuggingDiagonals（贴障碍对角段改正交，防切角）
  ├─ densifyMapPath（≤2 格补密，凑足 B-spline 航点数）
  ├─ bsplineSmooth()
  │    ├─ 从 LayeredCostmap 查找 EsdfLayer（enable_esdf 时）
  │    ├─ fit() + 注入 costmap / ESDF 数据
  │    └─ optimize() → 地图坐标平滑路径
  └─ 碰撞校验（失败则回退线性 JPS 路径）
```

`jps_planner` 暴露的 ROS 参数：`enable_bspline`、`enable_esdf`、`esdf_weight`、
`esdf_safe_distance`、`corridor_halfwidth`、`smoothness_weight`、`distance_weight`、
`max_control_points`、`max_iterations`、`collision_cost_threshold`。

## 测试

```bash
# 构建并运行 bspline_optimizer 测试
colcon build --packages-select bspline_optimizer
colcon test --packages-select bspline_optimizer --event-handlers console_direct+

# 直接运行联合测试（JPS 前端数据 → B-spline 后端）
./build/bspline_optimizer/joint_test
```

当前 `bspline_optimizer` 测试套件：

- `test_bspline`（gtest，13 项）：拟合精确性、直线不变形、端点保持、曲率、采样数量、未拟合边界行为等。
- `joint_test`：读取 JPS 场景数据，跑 `fit → optimize` 联合链路，输出对比指标与可视化数据。

## 调参建议与已知限制

- **想让平滑更明显**：增大 `smoothness_weight`，或减小 `distance_weight` / `max_control_points`。
- **想让路径更贴 JPS 原路径**：增大 `distance_weight`，减小 `corridor_halfwidth`。
- **靠近障碍物易回退**：增大 `esdf_weight`，适当增大 `esdf_safe_distance`。
- **短路径**：加密后仍不足 8 个航点时 B-spline 不生效，走线性兜底，这是设计行为。
- **计算开销**：主要来自控制点数 × 迭代数；长路径可降低 `max_control_points`。
