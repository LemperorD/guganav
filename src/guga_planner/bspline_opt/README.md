# B-spline 轨迹优化模块 — 设计文档

> 本文档描述当前实现的架构与算法，对应代码已按功能拆分（`src/` + `src/detail/`）。
> 旧版 `docs/DESIGN.md` 记录的是拆分前的设计与历史问题，仅供参考。

## 1. 概述

`bspline_opt` 把 JPS 搜索得到的**稀疏折线航点**平滑为 7 阶 B-spline 曲线，并通过
几何优化（平滑 + 距离保持 + ESDF 避障）和障碍物投射，输出一条光滑、无碰撞、
曲率有界的全局路径。

```
JPS 航点(折线) ──fit──▶ B-spline ──optimize──▶ 平滑路径
                             │                    │
                      chord-length 参数化      共轭梯度 + ESDF + 避障投射
```

## 2. 架构与模块

公开 API 在 `bspline_optimizer.hpp` / `bspline_optimizer.cpp`，内部实现按功能拆分到
`detail/` 目录：

```
include/bspline_opt/
├── bspline_optimizer.hpp        # 公开 API：BSplineConfig / State / Result / Optimizer
└── detail/                      # 内部模块（头文件）
    ├── common.hpp               # 共享常量（kSplineDegree=7）
    ├── grid_utils.hpp           # 代价地图障碍工具
    ├── esdf_utils.hpp           # ESDF 距离/梯度场双线性查询
    ├── basis.hpp                # B-spline 基函数（findSpan / basisValues / basisDerivs）
    ├── cost_cache.hpp           # BandRow / CostCache 带状缓存
    ├── cost_function.hpp        # 三项代价与解析梯度
    └── gradient_descent.hpp     # 共轭梯度优化器

src/
├── bspline_optimizer.cpp        # fit / optimize / sample / curvatureAt ...
└── detail/                      # 与上头文件对应的实现
```

### 2.1 模块职责

| 模块 | 职责 |
| --- | --- |
| `basis` | NURBS Book 基函数算法，计算任意参数处的基函数值/一阶导/二阶导 |
| `cost_cache` | 把基函数行预计算成带状结构，避免优化时重复求值 |
| `cost_function` | 平滑/距离/ESDF 三项代价及其解析梯度（按初始值归一化） |
| `gradient_descent` | 共轭梯度下降 + 回溯线搜索 + 走廊约束 |
| `grid_utils` | 障碍判定与螺旋搜索投射 |
| `esdf_utils` | 距离场/梯度场双线性插值 |

### 2.2 数据流

```
fit(path)
  ├─ chord-length 参数化 τ_i = arc_i / L
  ├─ SplineFitting::Interpolate(7阶) → 精确插值样条
  └─ 按弧长均匀重采样 M 个控制点（M = min(max_control_points, N)）

optimize(num_samples)
  ├─ 构建 CostCache（基函数行缓存）
  ├─ 计算三项代价初始值 → 归一化标度
  ├─ 共轭梯度优化内部控制点（平滑/距离/ESDF）
  ├─ 控制点障碍物投射（螺旋搜索）
  ├─ 重建样条 → 采样
  └─ 采样点投射 + 线段级加密修正（≤0.5 格）
```

## 3. 拟合 fit()

1. **chord-length 参数化**：按航点间欧氏距离累计弧长，参数 τ 归一化到 [0,1]。
2. **精确插值**：用 Eigen `SplineFitting::Interpolate` 生成穿过所有航点的 7 阶样条。
3. **控制点重采样**：沿弧长均匀选取 M 个控制点（默认全部，`max_control_points`
   可限制），控制点越少 → 曲线近似越强、拐角越圆滑。
4. **短路径回退**：航点 < 8 时退化为线性（两端点直线）。

## 4. 代价函数

总代价为三项加权和，**每项除以其初始值**归一化，使权重无量纲、与路径长度/网格
分辨率无关：

```
J = w_s · (J_smooth/J_s0) + w_d · (J_dist/J_d0) + w_e · (J_esdf/J_e0)
```

### 4.1 平滑性 J_smooth

曲率能量（二阶导平方）在参数域上的均值：

```
J_smooth = (1/K) Σ ||C''(u_k)||² ,  u_k = k/K, K=50
```

最小化它让曲线更平直、拐弯更舒缓。

### 4.2 距离保持 J_dist

惩罚曲线在原始航点参数处偏离 JPS 航点：

```
J_dist = Σ ||C(τ_i) − q_i||²
```

权重越大，曲线越贴原折线；越小，曲线越敢“切角”。

### 4.3 ESDF 避障 J_esdf

当曲线点进入障碍物安全距离内时惩罚：

```
J_esdf = Σ max(0, d_safe − d_esdf(p))²
```

ESDF 梯度场提供连续、平滑的推离方向，避免离散栅格带来的“看不见障碍物”问题。

## 5. 优化器：共轭梯度

内部控制点（2·(M−2) 维）作为优化变量，采用：

- **Polak-Ribière 共轭梯度**，卡住时重启为最速下降；
- **方向归一化 + 以格元为单位的步长**（初始 2 格，回溯减半），避免平滑项量级
  过大导致步长塌缩；
- **走廊约束**：每个内部控制点限制在初始位置 ±corridor_halfwidth 内，
  紧邻端点的控制点收紧到 ±0.5 格，防止终点附近过度弯曲；
- **解析梯度**（基于带状基函数缓存），比数值梯度快且无步长敏感问题。

> 历史教训：早期最速下降 + 固定步长在平滑项（量级 ~1e4~1e6）与距离项失衡时
> 会数值停滞，权重怎么调都不生效；因此改用归一化步长 + 共轭梯度。

## 6. 避障与碰撞安全

优化后做两级硬避障（与代价函数的软避障互补）：

1. **控制点投射**：落在障碍格元（cost ≥ 253）内的控制点，用螺旋搜索投射到最近
   空闲格元（半径 ≤8 格）。
2. **采样点投射 + 线段加密**：把落在障碍内的采样点投射到空闲格元；再对相邻点
   之间的线段按 ≤0.5 格加密检查，把仍穿障碍的加密点一并投射，多轮收敛，
   保证输出路径能通过下游碰撞检查、不轻易触发线性回退。

## 7. 输出

`BSplineResult` 包含：

- `smoothed_path`：地图坐标下的采样路径（数量 = num_samples）；
- `curvature_profile`：各采样点曲率；
- `control_points_xy`：优化后的控制点；
- `total_curvature_energy` / `cost_initial` / `cost_final` / `iterations` / `converged`。

## 8. 参数说明

`BSplineConfig` 各字段（代码默认值，可由 JPS 规划器的 ROS 参数覆盖）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `degree` | 7 | 样条阶数 |
| `smoothness_weight` | 0.1 | 平滑代价权重，越大拐弯越圆滑 |
| `distance_weight` | 1.0 | 距离保持权重，越大越贴原路径 |
| `esdf_weight` | 100.0 | ESDF 避障权重 |
| `esdf_safe_distance` | 0.3 | 距障碍物安全距离（米） |
| `corridor_halfwidth` | 2.5 | 控制点可移动走廊半宽（格元） |
| `max_control_points` | 200 | 最大控制点数，越小近似越强 |
| `max_iterations` | 200 | 梯度下降最大迭代数 |
| `enable_gradient_descent` | true | 是否做几何优化 |
| `enable_esdf` | true | 是否启用 ESDF 代价 |

## 9. 与 JPSPlanner 集成

`jps_planner` 直接编译本模块源码（见其 `CMakeLists.txt`），规划链路：

```
createPlan()
  ├─ JPSAlgorithm::generatePath → 稀疏跳点
  ├─ detourCornerHuggingDiagonals（贴障碍对角段改正交，防切角）
  ├─ densifyMapPath（≤2 格补密，凑足 B-spline 航点数）
  ├─ bsplineSmooth()
  │    ├─ 从代价地图插件查找 EsdfLayer（enable_esdf 时）
  │    ├─ fit() + 注入 costmap/esdf 数据
  │    └─ optimize() → 地图坐标平滑路径
  ├─ isPathCollisionFree 校验（失败则回退线性 JPS 路径）
  └─ 世界坐标 + 朝向 → nav_msgs::Path
```

JPS 规划器暴露的可调 ROS 参数（YAML 中位于 `planner_server.GridBased`）：
`enable_bspline`、`enable_esdf`、`esdf_weight`、`esdf_safe_distance`、
`corridor_halfwidth`、`smoothness_weight`、`distance_weight`、
`max_control_points`、`max_iterations`、`collision_cost_threshold`。

## 10. 测试

- `test_bspline.cpp`（gtest，13 项）：拟合精确性、曲率、端点保持、直线不变形、
  优化收敛、采样数量等；
- `joint_test.cpp`：JPS 前端 → B-spline 后端联合测试，输出可视化数据。

运行：

```bash
colcon test --packages-select bspline_opt
```

## 11. 调参建议与已知限制

- **想让平滑更明显**：增大 `smoothness_weight`、减小 `distance_weight` 或
  `max_control_points`（例如 24→16）。
- **想让路径更贴 JPS 原路径**：反之，增大 `distance_weight`。
- **靠近障碍物易回退**：增大 `esdf_weight`、适当减小 `esdf_safe_distance`。
- **短路径**（加密后仍不足 8 个航点，常见于原地小陀螺/近距目标）：B-spline
  不生效，走线性兜底——这是设计行为，不是 bug。
- **计算开销**：主要来自控制点数 × 迭代数；长路径可降低 `max_control_points`
  以换取速度。
