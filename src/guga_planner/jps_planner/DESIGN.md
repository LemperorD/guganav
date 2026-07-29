# JPS Planner — 数学原理与设计文档

> Jump Point Search (JPS) 全局规划器 + B-spline 平滑 + ESDF 梯度优化

## 1. JPS 搜索算法

### 1.1 网格模型

代价地图为二维均匀网格 $\mathcal{G} = \{0, \dots, W-1\} \times \{0, \dots, H-1\}$，分辨率 $r = 0.05\text{ m/cell}$。

每个格元 $(x,y)$ 有一个通行代价值 $c(x,y) \in [0, 255]$（来自 Nav2 costmap）：

| 值 | 含义 |
|----|------|
| $0$ | 空闲空间 (FREE_SPACE) |
| $1 \sim 252$ | 有代价空间 (cost 随值增大) |
| $253$ | 内切膨胀障碍物 (INSCRIBED_INFLATED_OBSTACLE) |
| $254$ | 致命障碍物 (LETHAL_OBSTACLE) |
| $255$ | 未知空间 (NO_INFORMATION) |

**障碍物判定**：$c(x,y) \ge 253$ 视为不可通行（`allow_unknown=false` 时 $255$ 也视为障碍物）。

### 1.2 A* 基础

JPS 是 A* 的扩展。A\* 的代价函数：

$$f(n) = g(n) + h(n)$$

- $g(n)$：从起点到节点 $n$ 的实际代价
- $h(n)$：从节点 $n$ 到终点的启发式估计
- $f(n)$：节点 $n$ 的总估计代价

A\* 优先队列按 $f$ 值升序展开节点，保证最优性当 $h$ 为 admissible（不高估真实代价）。

### 1.3 代价函数

#### 启发函数 $h(n)$

使用**加权欧几里得距离**：

$$h(n) = w_h \cdot \sqrt{(g_x - n_x)^2 + (g_y - n_y)^2}$$

参数：`w_heuristic_cost`（默认 $1.0$），用于调校启发式搜索的贪心程度。

#### 通行代价

原始 costmap 值通过 Theta\* 公式缩放：

$$s(c) = \frac{(26 + 0.9 \cdot c)^2}{252^2}$$

加权通行代价（单格元）：

$$t(c) = w_t \cdot s(c)$$

参数：`w_traversal_cost`（默认 $10.0$），控制对高代价格元的惩罚程度。

#### 单步欧几里得代价

$$d(a, b) = w_e \cdot \sqrt{(a_x - b_x)^2 + (a_y - b_y)^2}$$

参数：`w_euc_cost`（默认 $1.0$），在 $g$ 值更新时作为跳转点间的距离代价。

#### 路径总代价

$$g(\text{succ}) = g(\text{current}) + (\text{跳跃路径上的 }\Sigma t(c)) + d(\text{current}, \text{succ})$$

### 1.4 跳转点与邻居裁剪

JPS 的核心思想是**跳过大量中间节点**，只在"跳转点"处展开 A\* 搜索。

#### 自然邻居裁剪

给定父节点方向 $(dx, dy)$，当前节点 $(x, y)$ 的**自然邻居**被裁剪为：

| 父方向 | 自然邻居 |
|--------|----------|
| 水平直行 $(dx=\pm 1, dy=0)$ | $\{(dx, 0)\}$ |
| 垂直直行 $(dx=0, dy=\pm 1)$ | $\{(0, dy)\}$ |
| 对角线 $(dx=\pm 1, dy=\pm 1)$ | $\{(dx, dy), (dx, 0), (0, dy)\}$ |

裁剪原理：其余方向要么被父节点的更优路径覆盖，要么不存在更优路径。

#### 强制邻居

当裁剪方向的邻格被阻塞、但其对角线远处格元空闲时，该方向成为**强制邻居**，必须额外检查：

**水平方向 $(dx, \pm 1)$ 强制邻居**：
$$\text{obs}(x, y+1) \land \neg\text{obs}(x+dx, y+1) \quad\lor\quad \text{obs}(x, y-1) \land \neg\text{obs}(x+dx, y-1)$$

**垂直方向 $(\pm 1, dy)$ 强制邻居**：
$$\text{obs}(x+1, y) \land \neg\text{obs}(x+1, y+dy) \quad\lor\quad \text{obs}(x-1, y) \land \neg\text{obs}(x-1, y+dy)$$

**对角线方向 $(dx, dy)$ 强制邻居**：
$$\text{obs}(x-dx, y) \land \neg\text{obs}(x-dx, y+dy) \quad\lor\quad \text{obs}(x, y-dy) \land \neg\text{obs}(x+dx, y-dy)$$

### 1.5 递归跳跃

```
function JUMP(x, y, dx, dy, gx, gy):
    nx, ny = x + dx, y + dy

    if 越界 or 障碍物(nx, ny): return null
    acc += traversalCost(nx, ny)

    if (nx, ny) == (gx, gy): return (nx, ny)  // 到达终点
    if hasForcedNeighbor(nx, ny, dx, dy): return (nx, ny)  // 强制邻居

    if dx != 0 and dy != 0:
        // 对角跳跃：需检查两个分量方向是否存在跳转点
        if JUMP(nx, ny, dx, 0, gx, gy) != null: return (nx, ny)
        if JUMP(nx, ny, 0, dy, gx, gy) != null: return (nx, ny)

    return JUMP(nx, ny, dx, dy, gx, gy)  // 继续跳跃
```

跳跃过程中累计通行代价，在找到跳转点或遇到障碍物/边界时停止。

### 1.6 复杂度

- 最坏情况：$O(WH)$（全图遍历）
- 典型情况：$O(\sqrt{WH})$（仅展开跳转点）
- JPS 在开阔地形中比 A\* 快 10-100×（大幅减少展开节点数）

---

## 2. B-spline 平滑

### 2.1 数学定义

给定控制点 $\mathbf{P} = \{\mathbf{p}_0, \dots, \mathbf{p}_{M-1}\}$，$\mathbf{p}_i \in \mathbb{R}^2$，节点向量 $U = \{u_0, \dots, u_{M+k}\}$，§7§ 阶 B-spline 曲线：

$$\mathbf{C}(u) = \sum_{i=0}^{M-1} N_{i,7}(u) \cdot \mathbf{p}_i, \quad u \in [0, 1]$$

其中 $N_{i,7}(u)$ 是 Cox-de Boor 递推定义的 7 阶 B-spline 基函数。$C^6$ 连续，曲率有界。

### 2.2 拟合 (fit)

**步骤 1 — Chord-length 参数化**：

给定 JPS 路径点 $\{\mathbf{q}_0, \dots, \mathbf{q}_{N-1}\}$，弧长：

$$s_i = \sum_{j=1}^{i} \|\mathbf{q}_j - \mathbf{q}_{j-1}\|, \quad s_0 = 0$$

参数化：$\tau_i = s_i / s_{N-1} \in [0, 1]$

**步骤 2 — 精确插值**：使用 `Eigen::SplineFitting::Interpolate` 将 N 个航点精确插值为 7 阶 B-spline。

**步骤 3 — 降采样控制点**：在弧长等分处重新采样 M 个控制点（$M = \min(\text{max\_ctrl\_pts}, N)$，最少 $8$ 个）。

**步骤 4 — Knot Averaging**：从 M 个控制点子集的 chord-length 参数计算新节点向量。

### 2.3 梯度下降优化 (optimize)

#### 优化变量

只优化**内部控制点**（首尾点固定，$M-2$ 个内部控制点，共 $2(M-2)$ 维）：

$$\mathbf{x} = [p_{1,x}, p_{1,y}, p_{2,x}, p_{2,y}, \dots, p_{M-2,x}, p_{M-2,y}]$$

#### 代价函数

$$J(\mathbf{x}) = w_s \cdot J_{\text{smooth}} + w_d \cdot J_{\text{dist}} + w_o \cdot J_{\text{obs}} + w_e \cdot J_{\text{esdf}}$$

| 项 | 公式 | 含义 |
|----|------|------|
| 平滑性 $J_{\text{smooth}}$ | $\frac{1}{K_s} \sum_{i=0}^{K_s} \|\mathbf{C}''(u_i)\|^2$ | 积分曲率能量（$K_s=50$） |
| 距离 $J_{\text{dist}}$ | $\sum_{i} \|\mathbf{C}(\tau_i) - \mathbf{q}_i\|^2$ | 新路径对原始 JPS 航点的偏离 |
| 障碍物 $J_{\text{obs}}$ | $\sum_{i=0}^{K_o} \delta_i \cdot w_o \cdot (1 + \|\mathbf{p}_i - \mathbf{c}_i\|)$ | 二元障碍物惩罚（$K_o=200$ 采样点, $\delta_i = 1$ 当 $cost \ge 253$） |
| ESDF $J_{\text{esdf}}$ | $w_e \cdot \sum_{i=0}^{K_e} [\max(0, d_{safe} - d_{\text{esdf}}(\mathbf{p}_i))]^2$ | 连续距离场避障（$K_e=200$ 采样点） |

#### 梯度计算

使用中心差分（步长 $h = 0.5$ 格元）：

$$\frac{\partial J}{\partial x_j} \approx \frac{J(\mathbf{x} + h \cdot \mathbf{e}_j) - J(\mathbf{x} - h \cdot \mathbf{e}_j)}{2h}$$

$h = 0.5$ 的选择使数值梯度能跨格元"感知"到障碍物边界。

#### 线搜索

回溯线搜索（Armijo-like）：

1. 尝试步长 $\alpha$（初始 $10^{-3}$）
2. 计算候选点 $\mathbf{x}_{try} = \mathbf{x} - \alpha \nabla J$
3. **走廊约束**：$\text{clamp}(x_{try}^j, x_{init}^j - d_{corr}, x_{init}^j + d_{corr})$，限制控制点不偏离初始位置太远
4. 若 $J(\mathbf{x}_{try}) < J(\mathbf{x})$，接受并 $\alpha \leftarrow \min(2\alpha, 0.1)$
5. 否则 $\alpha \leftarrow 0.5\alpha$，最多 15 次尝试

#### 收敛条件

- $\|\nabla J\| < 10^{-8}$（梯度容差）
- 或连续 20 次迭代无改进（早停耐心值）

---

## 3. ESDF 辅助避障

### 3.1 ESDF (Euclidean Signed Distance Field)

ESDF 为代价地图的每个格元提供一个 **到最近障碍物的欧几里得距离**：

$$d_{\text{esdf}}(\mathbf{p}) = \min_{\mathbf{o} \in \mathcal{O}} \|\mathbf{p} - \mathbf{o}\|$$

其中 $\mathcal{O}$ 为所有障碍物格元的集合。距离值用 **8SED 两遍扫描法**（8-point Signed Euclidean Distance transform）计算。

### 3.2 ESDF 在函数中的角色

#### 3.2.1 替代二元障碍物惩罚

传统障碍物代价 $J_{\text{obs}}$ 是**非连续的二元函数**：进入障碍物格元立即产生巨大惩罚，离开则惩罚消失。这导致：

- 梯度在格元边界处跳变（从 $0$ 跳到 $\sim 50000$）
- 数值梯度不稳定，线搜索频繁失败
- 无法区分"刚好在障碍物边上"和"深入障碍物内部"

**ESDF 距离代价**是连续可微的：

$$J_{\text{esdf}} = w_e \cdot \sum_{i=0}^{K_e} [\max(0, d_{\text{safe}} - d_{\text{esdf}}(\mathbf{p}_i))]^2$$

其中 $d_{\text{safe}}$ 为安全距离（默认 $0.3$ m）。

#### 3.2.2 关键性质

| 性质 | 说明 |
|------|------|
| **连续可导** | 距离场 $d_{\text{esdf}}(\mathbf{p})$ 在自由空间中是 Lipschitz 连续且几乎处处可微 |
| **梯度指向安全方向** | $\nabla d_{\text{esdf}}$ 指向远离最近障碍物的方向 |
| **远距离无惩罚** | $d_{\text{esdf}} \ge d_{\text{safe}}$ 时代价为零，只在接近障碍物时生效 |
| **二次增长** | 随距离减小二次增长，产生平滑的"排斥力" |
| **与 $J_{\text{obs}}$ 独立工作** | ESDF 代价与二元障碍物代价相加，互补工作 |

#### 3.2.3 双线性插值查询

ESDF 查询使用双线性插值，在连续坐标上获得平滑的距离值：

$$d(\mathbf{p}) = (1 - \Delta x)(1 - \Delta y) \cdot d_{00} + \Delta x(1 - \Delta y) \cdot d_{10} + (1 - \Delta x)\Delta y \cdot d_{01} + \Delta x \Delta y \cdot d_{11}$$

其中 $(\Delta x, \Delta y)$ 是 $\mathbf{p}$ 相对于最近四个格元的分数坐标。

### 3.3 梯度下降中的 ESDF 代价

ESDF 代价通过以下方式增强了数值梯度下降：

1. **初始化**：B-spline 曲线已通过 SplineFitting 精确穿过 JPS 航点
2. **ESDF 代价引导**：梯度下降将内部控制点沿 $\nabla d_{\text{esdf}}$ 方向推开，增加路径与障碍物的间距
3. **与平滑性代价平衡**：$w_s \cdot J_{\text{smooth}}$ 保持曲率平滑，$w_e \cdot J_{\text{esdf}}$ 推开障碍物
4. **走廊约束**：限制控制点在初始位置 $\pm d_{\text{corr}}$ 范围内，防止过度偏离

### 3.4 数据流

```
┌──────────────────┐    ┌───────────────────┐    ┌──────────────────────┐
│  rog_map_layer   │    │     jps_planner   │    │     bspline_opt      │
│  (ESDF costmap   │───▶│  (JPSPlanner)      │───▶│  (BSplineOptimizer)  │
│   layer plugin)  │    │  configure()       │    │  evalCost()          │
│                  │    │  bsplineSmooth()   │    │  gradientDescent()   │
│  EsdfMap         │    │                   │    │                      │
│  ├ distance[]    │    │  enable_esdf_      │    │  esdfDistanceAt()    │
│  ├ gradient_x[]  │    │  esdf_weight_      │    │  (bilinear interp)   │
│  └ gradient_y[]  │    │  esdf_safe_distance│    │                      │
└──────────────────┘    └───────────────────┘    └──────────────────────┘
```

---

## 4. 参数配置

### 4.1 JPS 搜索参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `w_euc_cost` | 1.0 | 欧几里得距离代价权重 |
| `w_traversal_cost` | 10.0 | 通行代价权重（Theta\* 缩放后） |
| `w_heuristic_cost` | 1.0 | 启发式代价权重 |
| `allow_unknown` | false | 是否允许穿越未知空间 |

### 4.2 B-spline 优化参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enable_bspline` | true | 是否启用 B-spline 平滑 |
| `smoothness_weight` | 0.1 | 曲率平滑权重 |
| `distance_weight` | 10.0 | 到原始路径的距离权重 |
| `obstacle_weight` | 50000.0 | 二元障碍物惩罚权重 |
| `corridor_halfwidth` | 2.5 | 走廊约束半宽度（格元） |
| `max_iterations` | 200 | 梯度下降最大迭代次数 |
| `max_control_points` | 200 | 控制点数量上限 |

### 4.3 ESDF 参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enable_esdf` | false | 是否启用 ESDF 梯度优化 |
| `esdf_weight` | 100.0 | ESDF 距离惩罚权重 |
| `esdf_safe_distance` | 0.3 | ESDF 安全距离（米） |

启用 ESDF 时会自动开启 `enable_gradient_descent`。

### 4.4 权重调校指南

| 场景 | 建议调整 |
|------|----------|
| 路径过于靠近障碍物 | ↑ `esdf_weight`, ↑ `esdf_safe_distance` |
| 路径过于扭曲不平滑 | ↑ `smoothness_weight` |
| 路径偏离原始 JPS 太远 | ↑ `distance_weight`, ↓ `corridor_halfwidth` |
| 梯度下降收敛太慢 | ↓ `max_iterations`, ↑ 初始 `alpha` |
| 路径深入狭窄通道被推开 | ↓ `esdf_weight`, ↓ `esdf_safe_distance` |

---

## 5. 坐标系统

| 坐标系 | 单位 | 用途 |
|--------|------|------|
| 代价地图坐标 (cells) | 格元索引 | JPS 搜索、ESDF 场索引 |
| 地图坐标 (map coords) | 格元（地图原点偏移） | B-spline 拟合、路径输出 |
| 世界坐标 (world coords) | 米 | ROS tf、costmap API |
| 参数坐标 $u$ | [0, 1] | B-spline 求值 |

转换关系：
$$\text{world} = \text{origin} + \text{map\_coord} \times \text{resolution}$$
$$\text{map\_coord} = \text{cell\_center} = (\text{ix} + 0.5, \text{iy} + 0.5)$$

---

## 6. 参考文献

- Harabor, D., & Grastien, A. (2011). *Online Graph Pruning for Pathfinding on Grid Maps*. AAAI.
- Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). *Distance Transforms of Sampled Functions*. Theory of Computing.
- Ren, Y., Cai, Y., Zhu, F., Liang, S., & Zhang, F. (2024). *ROG-Map: An Efficient Robocentric Occupancy Grid Map for Large-scene and High-resolution LiDAR-based Motion Planning*. IROS 2024.
- Piegl, L., & Tiller, W. (1997). *The NURBS Book*. Springer.
- Eigen Splines: `unsupported/Eigen/Splines`
- Nav2 Costmap: https://docs.nav2.org/
