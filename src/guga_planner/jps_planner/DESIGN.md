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

### 1.2 A\* 基础

JPS 是 A\* 的扩展。A\* 的代价函数：

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

## 2. B-spline 轨迹参数化

### 2.1 数学定义

给定控制点 $\mathbf{P} = \{\mathbf{p}_0, \dots, \mathbf{p}_{M-1}\}$，$\mathbf{p}_i \in \mathbb{R}^2$，节点向量 $U = \{u_0, \dots, u_{M+k}\}$，§7§ 阶 B-spline 曲线：

$$\mathbf{C}(u) = \sum_{i=0}^{M-1} N_{i,7}(u) \cdot \mathbf{p}_i, \quad u \in [0, 1]$$

其中 $N_{i,7}(u)$ 是 Cox-de Boor 递推定义的 7 阶 B-spline 基函数。$C^6$ 连续，曲率有界。

**选择 7 阶 B-spline 的原因**：$C^6$ 连续性保证轨迹的加速度、加加速度（jerk）和更高阶导数连续，这对机器人的平滑运动至关重要；高连续性也使得曲率能量 $\int \|\mathbf{C}''\|^2 du$ 成为控制点位置的良态凸函数。

### 2.2 拟合 (fit) — 从 JPS 跳转点到 B-spline 曲线

拟合过程将 JPS 搜索得到的离散跳转点 $\{\mathbf{q}_0, \dots, \mathbf{q}_{N-1}\}$ 转化为连续可导的 B-spline 曲线。整个过程分为四个步骤，在 `BSplineOptimizer::fit()` 中实现。

**步骤 1 — Chord-length 参数化**：

给定 JPS 路径点 $\{\mathbf{q}_0, \dots, \mathbf{q}_{N-1}\}$，弧长：

$$s_i = \sum_{j=1}^{i} \|\mathbf{q}_j - \mathbf{q}_{j-1}\|_2, \quad s_0 = 0$$

参数化：$\tau_i = s_i / s_{N-1} \in [0, 1]$

Chord-length 参数化比均匀参数化更优：它保证参数间距与弧长成正比，避免在航点密集/稀疏处出现参数"拥挤"/"稀疏"现象。

**步骤 2 — 精确插值**：使用 `Eigen::SplineFitting::Interpolate` 将 $N$ 个航点精确插值为 7 阶 B-spline。此步骤求解线性系统：

$$\mathbf{C}(\tau_i) = \mathbf{q}_i, \quad \forall i = 0, \dots, N-1$$

得到插值 B-spline 的控制点和节点向量。该样条精确穿过所有 JPS 航点。

**步骤 3 — 降采样控制点**：在弧长等分处重新采样 $M$ 个控制点（$M = \min(\text{max\_ctrl\_pts}, N)$，最少 $8$ 个）：

$$\mathbf{p}_i = \mathbf{C}_{\text{dense}}\left(\frac{i}{M-1}\right), \quad i = 0, \dots, M-1$$

其中 $\mathbf{C}_{\text{dense}}$ 是步骤 2 中产生的密集插值样条。

**步骤 4 — Knot Averaging**：从 $M$ 个控制点子集的 chord-length 参数计算新节点向量，使用 `Eigen::KnotAveraging`：

$$u_{j+k} = \frac{1}{k} \sum_{i=j}^{j+k-1} \tau_i, \quad j = 1, \dots, M-k$$

此节点向量保证 B-spline 的端点插值性质：$\mathbf{C}(0) = \mathbf{p}_0$（起点），$\mathbf{C}(1) = \mathbf{p}_{M-1}$（终点）。

**路径点不足时的回退**：当 $N < 8$ 时，降级为 1 阶（线性）B-spline，仅保留首尾两个控制点。

### 2.3 曲率计算

曲线 $\mathbf{C}(u) = (x(u), y(u))$ 在参数 $u$ 处的曲率：

$$\kappa(u) = \frac{|x'(u) y''(u) - y'(u) x''(u)|}{(x'(u)^2 + y'(u)^2)^{3/2}}$$

利用 `Eigen::Spline::derivatives<2>(u)` 一次求出一阶和二阶导数。曲率能量用于评估路径平滑度的积分度量：

$$E_{\kappa} = \int_0^1 \|\mathbf{C}''(u)\|^2 du \approx \frac{1}{K+1}\sum_{i=0}^{K} \|\mathbf{C}''(u_i)\|^2$$

其中 $K=200$，$u_i = i/K$。

---

## 3. ESDF (Euclidean Signed Distance Field) 地图

### 3.1 ESDF 定义

ESDF 为代价地图的每个格元提供一个 **到最近障碍物的精确欧几里得距离**：

$$d_{\text{esdf}}(\mathbf{p}) = \min_{\mathbf{o} \in \mathcal{O}} \|\mathbf{p} - \mathbf{o}\|_2$$

其中 $\mathcal{O}$ 为所有障碍物格元的集合。ESDF 由 `rog_map_layer::EsdfLayer` 提供，内部使用 **8SED 两遍扫描法**（8-point Signed Euclidean Distance transform，参见 Felzenszwalb & Huttenlocher, 2012）计算。

### 3.2 ESDF 在系统架构中的位置

ESDF 作为 Nav2 costmap 的一个独立插件层 (`EsdfLayer`) 运行：

```
┌──────────────────────────────────────────────────────────┐
│                   LayeredCostmap                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  StaticLayer  │  │  ObstacleLayer │  │  EsdfLayer   │  │
│  │  (costmap)    │  │  (costmap)    │  │  (ESDF field) │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└──────────────────────────────────────────────────────────┘
         │                                       │
         ▼                                       ▼
   JPSPlanner ─── JPS搜索 (读 costmap)           │
         │                                       │
         ▼                                       ▼
   bsplineSmooth() ─── 梯度下降优化 (读 ESDF distance + gradient)
```

JPS 搜索阶段只使用原始 costmap（障碍物/非障碍物二元判定 + 通行代价值）。ESDF 距离场在 B-spline 梯度下降优化阶段才被使用。

### 3.3 ESDF 数据获取

在 `JPSPlanner::bsplineSmooth()` 中，通过遍历 costmap 插件列表查找 `EsdfLayer` 并提取其内部数据：

```cpp
auto esdf_layer = std::dynamic_pointer_cast<rog_map_layer::EsdfLayer>(plugin);
const auto * esdf_map = esdf_layer->getEsdfMapRaw();
// 注入 BSplineState:
state.esdf_distance   = esdf_map->distanceField().data();
state.esdf_gradient_x = esdf_map->gradientX().data();
state.esdf_gradient_y = esdf_map->gradientY().data();
```

传递到 `BSplineState` 的数据字段包括：

| 字段 | 类型 | 含义 |
|------|------|------|
| `esdf_distance` | `float[]` | 每个格元到最近障碍物的距离 (米) |
| `esdf_gradient_x` | `float[]` | 距离场在 x 方向的梯度 ∂d/∂x (变化/格元) |
| `esdf_gradient_y` | `float[]` | 距离场在 y 方向的梯度 ∂d/∂y (变化/格元) |
| `esdf_w`, `esdf_h` | `int` | ESDF 网格尺寸 (格元) |
| `esdf_resolution` | `double` | ESDF 分辨率 (米/格元, 默认 0.05) |
| `esdf_origin_x/y` | `double` | ESDF 网格原点在世界坐标系的位置 |
| `esdf_max_distance` | `double` | 距离场的截断距离 (超过此值的格元距离被截断) |

### 3.4 双线性插值查询

在梯度下降的 `evalCost()` 中，通过 `esdfDistanceAt()` 对 ESDF 距离场做双线性插值，在连续坐标上获得平滑的距离值。给定世界坐标 $(w_x, w_y)$：

1. **坐标转换**：将世界坐标映射到 ESDF 网格的分数坐标：

   $$f_x = \frac{w_x - o_x}{r}, \quad f_y = \frac{w_y - o_y}{r}$$

   其中 $(o_x, o_y)$ 是 ESDF 原点，$r$ 是分辨率。

2. **整数格元索引和分数偏移**：

   $$i_x = \lfloor f_x \rfloor, \quad i_y = \lfloor f_y \rfloor$$
   $$\Delta x = f_x - i_x, \quad \Delta y = f_y - i_y$$

3. **双线性插值**：

   $$d(\mathbf{w}) = (1-\Delta x)(1-\Delta y) \cdot d_{00} + \Delta x(1-\Delta y) \cdot d_{10} + (1-\Delta x)\Delta y \cdot d_{01} + \Delta x \Delta y \cdot d_{11}$$

   其中 $d_{00}, d_{10}, d_{01}, d_{11}$ 为四个最近格元的距离值。

双线性插值的关键性质：$d(\mathbf{w})$ 是 $C^0$ 连续（值连续），且在格元内部是光滑的（双线性）。虽然梯度在格元边界处可能不连续，但在数值梯度下降的尺度上（$h=0.5$ 格元），这种不连续被充分平滑，不会造成优化困难。

---

## 4. 轨迹优化 — 梯度下降

### 4.1 优化变量

优化变量 $\mathbf{x} \in \mathbb{R}^{2(M-2)}$ 由 B-spline 曲线的**内部控制点**组成，首尾两个控制点固定不变（起点和终点位置锁定）：

$$\mathbf{x} = [\underbrace{p_{1,x}, p_{1,y}}_{\text{控制点 }1}, \underbrace{p_{2,x}, p_{2,y}}_{\text{控制点 }2}, \dots, \underbrace{p_{M-2,x}, p_{M-2,y}}_{\text{控制点 }M-2}]^{\top}$$

其中 $M$ 为控制点总数。首点 $\mathbf{p}_0$（起点）和尾点 $\mathbf{p}_{M-1}$（终点）在优化全程保持不变。

**维度**：$2(M-2)$ 维。对于典型路径 $M \approx 30\sim 100$，优化空间为 $56\sim 196$ 维。

**为什么只优化内部控制点？** 起点和终点由规划请求给定，不可更改。内部控制点决定了曲线的形状和平滑性，且 B-spline 曲线在端点处由边界控制点主导，中间控制点的影响随距离衰减，因此在约束区间内移动内部控制点是安全的。

**从 $\mathbf{x}$ 重建控制点矩阵**：每次评估代价函数时，将优化变量 $\mathbf{x}$ 与固定的首尾控制点拼接为 $2 \times M$ 控制点矩阵：

$$\mathbf{P} = \begin{bmatrix}
p_{0,x} & x_0 & x_2 & \dots & x_{2(M-3)} & p_{M-1,x} \\
p_{0,y} & x_1 & x_3 & \dots & x_{2(M-3)+1} & p_{M-1,y}
\end{bmatrix}_{2 \times M}$$

然后用控制点 $\mathbf{P}$ 和节点向量 $U$ 构造 B-spline：$\mathbf{C}(u) = \sum_{i=0}^{M-1} N_{i,7}(u) \cdot \mathbf{p}_i$。

### 4.2 目标函数（代价函数）

总目标函数为四项加权和：

$$J(\mathbf{x}) = w_s \cdot J_{\text{smooth}}(\mathbf{x}) + w_d \cdot J_{\text{dist}}(\mathbf{x}) + w_o \cdot J_{\text{obs}}(\mathbf{x}) + w_e \cdot J_{\text{esdf}}(\mathbf{x})$$

以下逐一详解每一项的数学定义、物理含义和梯度特性。

---

#### 4.2.1 平滑性代价 $J_{\text{smooth}}$

$$J_{\text{smooth}}(\mathbf{x}) = \frac{1}{K_s+1} \sum_{i=0}^{K_s} \|\mathbf{C}''(u_i)\|^2, \quad u_i = \frac{i}{K_s}, \quad K_s = 50$$

或等价地写为连续形式：

$$J_{\text{smooth}}(\mathbf{x}) \approx \int_0^1 \|\mathbf{C}''(u)\|^2 du$$

**物理含义**：$\|\mathbf{C}''(u)\|^2 = (d^2x/du^2)^2 + (d^2y/du^2)^2$ 是曲线的**曲率能量**——法向加速度的平方积分。最小化此代价项鼓励曲线尽可能平直（二阶导数为零，即直线），避免不必要的拐弯和蛇形路径。

**梯度特性**：$\nabla_{\mathbf{x}} J_{\text{smooth}}$ 指向减少控制点间曲率变化的方向。由于 B-spline 的二阶导数是控制点的线性函数，$J_{\text{smooth}}$ 是 $\mathbf{x}$ 的**凸二次函数**，梯度下降此部分收敛极快。

**权重**：$w_s = 0.1$（默认），较低的值使平滑性不主导其他代价项。

---

#### 4.2.2 距离保持代价 $J_{\text{dist}}$

$$J_{\text{dist}}(\mathbf{x}) = \sum_{i=0}^{N-1} \|\mathbf{C}(\tau_i) - \mathbf{q}_i\|^2$$

其中 $\tau_i$ 是第 $i$ 个原始 JPS 航点 $\mathbf{q}_i$ 的 chord-length 参数。

**物理含义**：惩罚优化后的 B-spline 曲线在原始航点参数处偏离 JPS 搜索得到的航点。初始曲线精确穿过所有航点（来自 `SplineFitting::Interpolate`），因此初始时此项为零；优化过程中，若平滑性或 ESDF 代价推动控制点移动，此项产生约束力将曲线拉回原始航点附近。

**为什么在 chord-length 参数处评估而非均匀参数处**：chord-length 参数与原始航点的弧长对应。如果在均匀参数处评估，样条在航点密集区域可能尚未到达该航点（参数间距太小），导致错误的"偏离"惩罚。

**权重**：$w_d = 10.0$（默认）。

---

#### 4.2.3 二元障碍物代价 $J_{\text{obs}}$

$$J_{\text{obs}}(\mathbf{x}) = \sum_{i=0}^{K_o} \delta_i \cdot w_o \cdot \big(1 + |\Delta x_i| + |\Delta y_i|\big), \quad K_o = 200$$

其中：

$$\delta_i = \begin{cases} 1, & c(\lfloor C_x(u_i) \rfloor, \lfloor C_y(u_i) \rfloor) \ge 253 \\ 0, & \text{otherwise} \end{cases}$$

$$\Delta x_i = C_x(u_i) - \lfloor C_x(u_i) \rfloor - 0.5$$
$$\Delta y_i = C_y(u_i) - \lfloor C_y(u_i) \rfloor - 0.5$$

即当采样点落入障碍物格元（cost $\ge 253$）时，施加与采样点到格元中心的偏移量成正比的惩罚。

**物理含义**：这是一个**硬障碍物避障代价**。它是非连续的二元函数——在障碍物格元边界处从零跳变到 $\sim 50000$。在梯度下降中，这一项依靠数值步长 $h=0.5$ 格元来"感知"障碍物边界：当控制点被移动到障碍物格元内时，代价急剧上升，梯度指向避开障碍物的方向。

**局限性**：由于二元性质，梯度在大部分自由空间中为零，只有在恰好跨越障碍物边界时才非零。这使得 $J_{\text{obs}}$ 单独使用时优化不稳定——这就是引入 ESDF 代价的动机。

**权重**：$w_o = 50000.0$（默认），极高的值确保了障碍物的严格不可穿越性。

---

#### 4.2.4 ESDF 距离场避障代价 $J_{\text{esdf}}$（核心创新）

$$J_{\text{esdf}}(\mathbf{x}) = w_e \cdot \sum_{i=0}^{K_e} \big[\max(0, d_{\text{safe}} - d_{\text{esdf}}(\mathbf{C}(u_i)))\big]^2, \quad K_e = 200$$

其中：

- $d_{\text{safe}}$ 是安全距离（默认 $0.3$ m）
- $d_{\text{esdf}}(\mathbf{p})$ 是通过双线性插值从 ESDF 距离场查询的连续距离值
- $\mathbf{C}(u_i)$ 是 B-spline 在世界坐标下的采样点
- $K_e = 200$ 个采样点均匀分布在 $u \in [0, 1]$ 上

**坐标转换**：在计算 $J_{\text{esdf}}$ 前，需要将 B-spline 的采样点从地图坐标转换为世界坐标。`evalCost()` 中通过：

$$w_x = p_x \cdot r + o_x, \quad w_y = p_y \cdot r + o_y$$

将地图坐标 $(p_x, p_y)$ 转换为世界坐标 $(w_x, w_y)$，然后调用 `esdfDistanceAt()` 查询 ESDF 距离。

**物理含义**：当路径上的任一点与最近障碍物的距离小于 $d_{\text{safe}}$ 时，产生与距离违反量的平方成正比的惩罚。该代价的梯度为：

$$\frac{\partial J_{\text{esdf}}}{\partial \mathbf{x}} = 2w_e \sum_{i} \max(0, d_{\text{safe}} - d_i) \cdot \left(-\frac{\partial d_i}{\partial \mathbf{x}}\right)$$

其中 $-\nabla d_{\text{esdf}}$ 指向**远离最近障碍物**的方向。因此梯度下降将路径点沿距离场增大的方向推开，增加路径与障碍物的间隙。

**关键数学性质**：

| 性质 | 说明 | 优势 |
|------|------|------|
| **连续可微** | $d_{\text{esdf}}(\mathbf{p})$ 在自由空间中是 Lipschitz 连续且几乎处处可微（除距离场的脊线/shock 曲线外） | 梯度下降每次迭代都能获得有效的下降方向 |
| **梯度指向安全方向** | $\nabla d_{\text{esdf}}$ 精确指向远离最近障碍物的方向 | 优化自然将路径推向开阔空间 |
| **二次增长** | 代价随距离减小以二次速率增长 | 产生平滑递增的"排斥力"，不会像二元代价那样突变 |
| **远距离零惩罚** | $d \ge d_{\text{safe}}$ 时代价和梯度均为零 | 不影响已经安全的路径段 |
| **与 $J_{\text{obs}}$ 互补** | ESDF 代价提供平滑梯度，$J_{\text{obs}}$ 提供硬约束兜底 | 两者联合使用时 ESDF 引导路径逐步远离障碍物，$J_{\text{obs}}$ 确保不会穿越 |

**与二元障碍物代价的对比**：

| 维度 | $J_{\text{obs}}$ | $J_{\text{esdf}}$ |
|------|-----------------|-------------------|
| 连续性 | 非连续（格元边界跳变） | 连续且几乎处处可微 |
| 梯度 | 只在障碍物边界非零 | 在 $d < d_{\text{safe}}$ 区域内持续非零 |
| 优化效率 | 线搜索频繁失败，需要较大 $h$ | 梯度平滑，线搜索稳定 |
| 敏感范围 | 仅障碍物格元内部 | 障碍物周围 $d_{\text{safe}}$ 半径内 |
| 区分度 | 无法区分远近 | 越近代价越大，提供方向性信息 |

**数值示例**：假设 B-spline 上一点距离障碍物 $0.1$ m，$d_{\text{safe}} = 0.3$ m，$w_e = 100$：

$$J_{\text{esdf}} \text{ 单点贡献} = 100 \times (0.3 - 0.1)^2 = 100 \times 0.04 = 4.0$$

对比二元代价（假设该点在障碍物格元内，离中心 $0.3$ 格元）：

$$J_{\text{obs}} \text{ 单点贡献} \approx 50000 \times (1 + 0.3) = 65000$$

ESDF 的 $4.0$ 比二元的 $65000$ 小四个数量级——但 ESDF 的连续梯度让优化器**提前感知**到障碍物并逐步调整路径，而非在碰墙时才"惊恐"地做出大跳跃。

---

### 4.3 全局代价函数汇总

| 项 | 符号 | $K$ 采样点数 | 默认权重 | 连续/非连续 | 梯度域 |
|----|------|-------------|----------|------------|--------|
| 平滑性 | $J_{\text{smooth}}$ | 50 | $w_s=0.1$ | $C^\infty$ | 全空间连续 |
| 距离保持 | $J_{\text{dist}}$ | $N$ 个航点 | $w_d=10.0$ | $C^\infty$ | 全空间连续 |
| 二元障碍物 | $J_{\text{obs}}$ | 200 | $w_o=50000$ | 非连续 | 只在障碍物边界非零 |
| ESDF 距离场 | $J_{\text{esdf}}$ | 200 | $w_e=100.0$ | 几乎处处 $C^1$ | $d < d_{\text{safe}}$ 区域持续非零 |

### 4.4 优化算法：数值梯度下降 + 回溯线搜索

优化在 `gradientDescent()` 函数中实现。整体算法流程如下：

```
算法 1: 数值梯度下降 + 回溯线搜索

输入: 初始内部控制点位置 x_init ∈ ℝ^{2(M-2)}
      B-spline 节点向量 U
      原始 JPS 航点 {q_i} 及其 chord-length 参数 {τ_i}
      固定首尾控制点 (p_0, p_{M-1})
      代价函数权重 w_s, w_d, w_o, w_e
      ESDF 数据 (距离场, 梯度场, 分辨率, 原点)
      max_iterations = 200

参数: h = 0.5          (数值差分步长, 格元)
      α = 10^{-3}      (初始线搜索步长)
      g_tol = 10^{-8}  (梯度范数收敛容差)
      patience = 20    (早停: 连续无改进迭代数上限)

输出: x_opt (优化后的内部控制点位置)

1: x ← x_init
2: f_best ← evalCost(x);  no_improve ← 0

3: for iter = 0 to max_iterations - 1 do
4:     // ── 步骤 A: 数值梯度计算 ──
5:     for j = 0 to 2(M-2)-1 do
6:         x_j ← x_j + h
7:         f_plus ← evalCost(x)
8:         x_j ← x_j - 2h
9:         f_minus ← evalCost(x)
10:        x_j ← x_j + h          // 恢复
11:        g_j ← (f_plus - f_minus) / (2h)
12:    end for
13:    // ── 步骤 B: 收敛检查 ──
14:    ||g|| ← sqrt(Σ g_j²)
15:    if ||g|| < g_tol then converged ← true; break
16:
17:    // ── 步骤 C: 回溯线搜索 ──
18:    α ← min(2α, 0.1)          // 增长试探步长
19:    found ← false
20:    for ls = 0 to 14 do
21:        for j = 0 to 2(M-2)-1 do
22:            x_try_j ← x_j - α · g_j
23:            x_try_j ← clamp(x_try_j,
24:                x_init_j - d_corr, x_init_j + d_corr)  // 走廊约束
25:        end for
26:        f_try ← evalCost(x_try)
27:        if f_try < f_best then
28:            x ← x_try; f_best ← f_try
29:            found ← true; no_improve ← 0
30:            break
31:        end if
32:        α ← α / 2
33:    end for
34:
35:    // ── 步骤 D: 早停检查 ──
36:    if not found then
37:        no_improve ← no_improve + 1
38:        if no_improve >= patience then converged ← true; break
39:    end if
40: end for
```

#### 4.4.1 数值梯度计算（步骤 A，第 5-12 行）

采用**中心差分**公式：

$$\frac{\partial J}{\partial x_j} \approx \frac{J(\mathbf{x} + h \cdot \mathbf{e}_j) - J(\mathbf{x} - h \cdot \mathbf{e}_j)}{2h}$$

其中 $\mathbf{e}_j$ 是第 $j$ 个标准基向量，$h = 0.5$ 格元。

**为什么数值梯度而非解析梯度？**
1. B-spline 求值（`Eigen::Spline`）和 ESDF 双线性插值（`esdfDistanceAt`）的解析梯度链过长且难以维护
2. 代价函数的许多部分（格元索引 `int(cx)`、$\max(0, \cdot)$ 操作符）导致解析梯度在关键点上非连续
3. 数值梯度对实现细节不敏感，代码修改后无需更新梯度表达式
4. 每次迭代需要 $2 \times 2(M-2)$ 次代价函数评估；对 $M \approx 30$，每次迭代约 $112$ 次评估，即使是 200 次迭代也在实时规划的可承受范围内

**为什么 $h = 0.5$ 格元（而非更小的值）？**

| $h$ 值 | 效果 |
|--------|------|
| $10^{-3}$ | 障碍物代价梯度为零——控制点移动不足以跨越格元边界，二维代价不产生变化 |
| $0.5$ | 差分恰好跨越半个格元，能"感知"到相邻格元的障碍物，产生有意义的梯度 |
| $1.0$ | 梯度噪声增大，可能跨过薄障碍物，丢失局部信息 |

$h = 0.5$ 的选择使数值梯度能跨格元"感知"到障碍物边界和 ESDF 距离变化，同时保持梯度的局部性。

**计算复杂度分析**：
- 每次 `evalCost()` 包含：50 个平滑性采样点 + $N$ 个距离代价点 + 200 个障碍物采样点 + 200 个 ESDF 采样点
- 总计 $\approx 450 + N$ 次 B-spline 求值，每次求值是 $O(M)$ 的 Cox-de Boor 递推
- 每次迭代的梯度计算为 $2 \times 2(M-2)$ 次 `evalCost()`
- 对 $M=30$（56 维变量），每次迭代约 112 次 `evalCost()`，每次 `evalCost()` 约 480 次 B-spline 求值

#### 4.4.2 线搜索策略（步骤 C，第 18-33 行）

采用**回溯线搜索**（Armijo-like，但不要求充分下降条件，只要求新代价严格小于当前最优代价）：

1. **步长增长试探**：$\alpha \leftarrow \min(2\alpha, 0.1)$。每次成功迭代后增大步长，加速收敛；上限 $0.1$ 防止步长过大跳过局部极小。

2. **候选点计算**：$\mathbf{x}_{\text{try}} = \mathbf{x} - \alpha \nabla J$，即沿负梯度方向移动。

3. **走廊约束**（见 §4.5）：将候选控制点限制在初始位置 $\pm d_{\text{corr}}$ 范围内。

4. **接受条件**：$J(\mathbf{x}_{\text{try}}) < J(\mathbf{x}_{\text{best}})$。若接受，标记 `found = true` 并继续下一轮迭代；若拒绝，$\alpha \leftarrow \alpha / 2$（步长减半），最多重试 15 次。

5. **步长衰减**：每次拒绝后 $\alpha \leftarrow \alpha/2$，相当于指数衰减 $\alpha_k = \alpha_0 \cdot 2^{-k}$。

#### 4.4.3 收敛条件（步骤 B + D）

优化在满足以下任一条件时终止：

- **梯度收敛**：$\|\nabla J\|_2 < 10^{-8}$。梯度范数极小意味着已达到驻点（极小值）。
- **早停**：连续 20 次迭代没有找到可接受的新解。线搜索在代价函数凹凸不平（常见于包含二元障碍物代价时）时可能频繁拒绝候选点，早停机制防止无限循环。
- **达到最大迭代次数**：200 次。

### 4.5 走廊约束

在每个线搜索步中，候选控制点被限制在以其初始位置为中心的矩形"走廊"内：

$$x_{\text{try}}^j \in \big[x_{\text{init}}^j - d_{\text{corr}}, \; x_{\text{init}}^j + d_{\text{corr}}\big], \quad \forall j$$

其中 $d_{\text{corr}} = 8.0$ 格元（JPSPlanner 默认 `corridor_halfwidth`）。

**走廊约束的目的**：

1. **防止过度偏离**：限制优化后曲线与 JPS 原始路径的最大偏离，保证路径的全局拓扑一致性。没有走廊约束时，ESDF 排斥力可能将路径推出狭窄通道、绕过障碍物而非穿越它们——这可能破坏 JPS 搜索找到的可行通道。

2. **保留 JPS 路径的可行性**：JPS 在离散网格上已验证该走廊内存在可行通道。限制控制点在该走廊内移动，保证优化后的路径仍然大致沿着已验证的通道。

3. **数值稳定性**：限制搜索域为紧致集，保证线搜索候选点始终在合理区间内，避免控制点飞入无效区域导致 spline 求值异常。

**走廊宽度选择**：$d_{\text{corr}} = 2.5$ 格元（$0.125$ m 在 $0.05$ m 分辨率下）是一个经验平衡：足够宽以允许 ESDF 梯度将路径推开障碍物（一般需要 $1\sim 2$ 格元的位移），但不够宽到改变路径的拓扑连通性。

### 4.6 障碍物投射后处理

梯度下降完成后，`BSplineOptimizer::optimize()` 执行两轮障碍物投射：

#### 第一轮：控制点投射

对每个内部控制点 $\mathbf{p}_i (i = 1, \dots, M-2)$：

```cpp
projectPointToFree(p_i.x, p_i.y)
    if costmap[floor(p_i.x)][floor(p_i.y)] < 253: return  // 已在安全区域
    for r = 1 to 8:           // 螺旋搜索
        for dx = -r to r:
            for dy = -r to r:
                if |dx| < r and |dy| < r: skip  // 跳过内层已搜索区域
                if costmap[ix+dx][iy+dy] < 253:
                    p_i = (ix+dx+0.5, iy+dy+0.5)  // 移动到空闲格元中心
                    return
```

搜索半径为 $8$ 格元，优先找到最近（曼哈顿距离）的空闲格元。

#### 第二轮：采样路径点投射

对输出路径的每个采样点执行相同的螺旋搜索投射，确保最终输出的全部航点都不在障碍物内。

**投射后处理的意义**：梯度下降的走廊约束和 ESDF 代价可能仍无法将所有控制点完全移出障碍物（特别是初始控制点本来就落在障碍物内的情况），投射步骤提供了**确定性**的无障碍保证。

### 4.7 完整的优化流程（从 JPS 到平滑路径）

```
输入: start + goal (世界坐标)
  │
  ▼
1. costmap_->worldToMap() → 格元坐标 (sx, sy), (gx, gy)
  │
  ▼
2. JPSAlgorithm::generatePath()
  │  ┌─ A* 优先队列展开跳转点
  │  ├─ 累计通行代价 Σ t(c) + d(jump_pts)
  │  └─ backtracePath 回溯
  ▼
  输出: map_path (跳转点, 格元中心, N 个航点)
  │
  ▼
3. BSplineOptimizer::fit()
  │  ├─ chord-length 参数化 τ_i = s_i / s_{N-1}
  │  ├─ Eigen::SplineFitting::Interpolate(pts, 7)
  │  ├─ 弧长等分处重采样 M 个控制点
  │  └─ Eigen::KnotAveraging → 节点向量 U
  ▼
  输出: fitted B-spline (C^6 连续, 精确穿过 JPS 航点)
  │
  ▼
4. ESDF 数据注入 (JPSPlanner::bsplineSmooth)
  │  ┌─ 查找 LayeredCostmap 中的 EsdfLayer
  │  └─ 提取 EsdfMap 的 distance[], gradient_x[], gradient_y[]
  ▼
  opt.state().esdf_distance = ...  (指针注入)
  │
  ▼
5. BSplineOptimizer::optimize(num_samples)
  │
  ├─ [若 enable_gradient_descent = true] ──────────────────────┐
  │  │                                                         │
  │  ├─ 构造优化变量 x ∈ ℝ^{2(M-2)} (内部控制点)              │
  │  │                                                         │
  │  ├─ gradientDescent() (算法 1)                             │
  │  │  │                                                      │
  │  │  ├─ 数值中心差分 ∇J (维度: 2(M-2), h=0.5)              │
  │  │  │  │                                                   │
  │  │  │  └─ evalCost(x) ─────────────────────────────┐      │
  │  │  │     │  ├─ J_smooth: 50 采样点 ‖C''‖²        │      │
  │  │  │     │  ├─ J_dist:   N 航点 ‖C(τ_i)-q_i‖²   │      │
  │  │  │     │  ├─ J_obs:    200 采样点 二元障碍物   │      │
  │  │  │     │  └─ J_esdf:   200 采样点 ESDF 距离场  │      │
  │  │  │     │        └─ esdfDistanceAt() 双线性插值 │      │
  │  │  │     └───────────────────────────────────────┘      │
  │  │  │                                                      │
  │  │  ├─ 回溯线搜索 (α 自适应, clamp 走廊约束)               │
  │  │  └─ 收敛: ‖∇J‖ < 1e-8 或 patience=20                   │
  │  │                                                         │
  │  └─ 更新 state_.control_points                             │
  │                                                             │
  ├─ 控制点障碍物投射 (螺旋搜索 r=1..8)                       │
  ├─ rebuildSpline()                                          │
  ├─ 采样路径点障碍物投射                                      │
  └─ 计算曲率 profile                                          │
  │
  ▼
6. 地图坐标 → 世界坐标 (costmap_->mapToWorld)
  │
  ▼
输出: nav_msgs::Path (C^6 连续平滑路径, 障碍物安全保证)
  │
  ▼
7. writePathToShm() → 推送到共享内存 (Pangolin UI 渲染)
```

### 4.8 启用 ESDF 时的参数覆盖

在 `JPSPlanner::bsplineSmooth()` 中，当检测到 EsdfLayer 时自动覆盖优化器配置：

```cpp
bspline_config_.enable_esdf = true;           // 激活 ESDF 代价项 J_esdf
bspline_config_.enable_gradient_descent = true; // 自动开启梯度下降
bspline_config_.esdf_weight = esdf_weight_;     // w_e (默认 100.0)
bspline_config_.esdf_safe_distance = esdf_safe_distance_; // d_safe (默认 0.3 m)
```

这意味着启用 ESDF 时，整体优化流程变为 $J(\mathbf{x})$ 的全四项评估。如果 ESDF 层未找到，回退到不含 $J_{\text{esdf}}$ 的三项优化（仅 $J_{\text{smooth}} + J_{\text{dist}} + J_{\text{obs}}$）。

**不启用 ESDF 时的行为**（默认模式）：
- B-spline 精确插值已完成，梯度下降被跳过（`enable_gradient_descent = false`）
- `optimize()` 只执行障碍物投射后处理（控制点 + 采样点）
- 依赖 JPS 搜索的路径安全性 + B-spline 的插值平滑性

---

## 5. 参数配置

### 5.1 JPS 搜索参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `w_euc_cost` | 1.0 | 欧几里得距离代价权重 |
| `w_traversal_cost` | 10.0 | 通行代价权重（Theta\* 缩放后） |
| `w_heuristic_cost` | 1.0 | 启发式代价权重 |
| `allow_unknown` | false | 是否允许穿越未知空间 |

### 5.2 B-spline 优化参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enable_bspline` | true | 是否启用 B-spline 平滑 |
| `smoothness_weight` | 0.1 | 曲率平滑权重 |
| `distance_weight` | 10.0 | 到原始路径的距离权重 |
| `obstacle_weight` | 50000.0 | 二元障碍物惩罚权重 |
| `corridor_halfwidth` | 8.0 | 走廊约束半宽度（格元） |
| `max_iterations` | 200 | 梯度下降最大迭代次数 |
| `max_control_points` | 200 | 控制点数量上限 |

### 5.3 ESDF 参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enable_esdf` | false | 是否启用 ESDF 梯度优化 |
| `esdf_weight` | 100.0 | ESDF 距离惩罚权重 |
| `esdf_safe_distance` | 0.6 | ESDF 安全距离（米） |

启用 ESDF 时会自动开启 `enable_gradient_descent`。

### 5.4 权重调校指南

| 场景 | 建议调整 |
|------|----------|
| 路径过于靠近障碍物 | ↑ `esdf_weight`, ↑ `esdf_safe_distance` |
| 路径过于扭曲不平滑 | ↑ `smoothness_weight` |
| 路径偏离原始 JPS 太远 | ↑ `distance_weight`, ↓ `corridor_halfwidth` |
| 梯度下降收敛太慢 | ↓ `max_iterations`, ↑ 初始 `alpha` |
| 路径深入狭窄通道被推开 | ↓ `esdf_weight`, ↓ `esdf_safe_distance` |

---

## 6. 坐标系统

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

## 7. ESDF 梯度下降的数学分析

### 7.1 代价函数对控制点位置的梯度

对于单个 ESDF 采样点 $\mathbf{p}_{\text{world}} = \mathbf{C}(u_k) \cdot r + \mathbf{o}$（$r$ 为分辨率，$\mathbf{o}$ 为原点），其代价为：

$$j_k(\mathbf{x}) = w_e \cdot \big[\max(0, d_{\text{safe}} - d_{\text{esdf}}(\mathbf{p}_{\text{world}}))\big]^2$$

应用链式法则：

$$\frac{\partial j_k}{\partial x_j} = -2w_e \cdot \max(0, d_{\text{safe}} - d_{\text{esdf}}) \cdot \frac{\partial d_{\text{esdf}}}{\partial \mathbf{p}_{\text{world}}} \cdot r \cdot \frac{\partial \mathbf{C}}{\partial x_j}$$

其中：
- $\frac{\partial d_{\text{esdf}}}{\partial \mathbf{p}_{\text{world}}}$ 是 ESDF 距离场的梯度（通过 `esdf_gradient_x/y` 预存或双线性插值的差分近似）
- $\frac{\partial \mathbf{C}}{\partial x_j} = N_{j,7}(u_k) \cdot \mathbf{I}_{2 \times 2}$（B-spline 基函数，控制点对曲线求值的线性影响）
- $r$ 是地图坐标到世界坐标的缩放因子

**数值梯度的等价性**：在中心差分中，$\pm h$ 扰动自动通过 B-spline 传播到曲线采样点，再经坐标转换到世界坐标，最后通过 ESDF 双线性插值产生代价差异——完整地近似了上述链式梯度。

### 7.2 ESDF 排斥力的有效范围

$J_{\text{esdf}}$ 只在 $d_{\text{esdf}} < d_{\text{safe}}$ 时非零，这意味着 ESDF 的排斥力有一个**有效半径**：

$$r_{\text{eff}} = d_{\text{safe}} = 0.3\text{ m}$$

在 $r_{\text{eff}}$ 内，排斥力随距离减小线性增长（代价为二次 → 梯度为线性）：

$$\|\nabla J_{\text{esdf}}\| \propto 2w_e \cdot |d_{\text{safe}} - d_{\text{esdf}}|$$

在 $r_{\text{eff}}$ 外，排斥力恒为零。这保证了 ESDF 只影响接近障碍物的路径段，不影响开阔区域已经安全的路径段。

### 7.3 与其他代价项的交互

在梯度下降中，四项代价的梯度同时作用于优化变量 $\mathbf{x}$：

$$\nabla J(\mathbf{x}) = w_s \nabla J_{\text{smooth}} + w_d \nabla J_{\text{dist}} + w_o \nabla J_{\text{obs}} + w_e \nabla J_{\text{esdf}}$$

**平衡分析**（在典型路径点的量级估计）：

| 分量 | 典型梯度范数 | 方向 |
|------|-------------|------|
| $w_s \nabla J_{\text{smooth}}$ | $\sim 10^{-2}$ | 指向减小曲率方向（拉直路径） |
| $w_d \nabla J_{\text{dist}}$ | $\sim 0$（初始插值为 0） | 指向原始航点 |
| $w_o \nabla J_{\text{obs}}$ | $\sim 0$（安全区域）或 $\gg 10^4$（障碍物内） | 指向障碍物外 |
| $w_e \nabla J_{\text{esdf}}$ | $\sim 1\sim 10$（$d < d_{\text{safe}}$ 时） | 指向远离最近障碍物 |

在安全区域内（$d \ge d_{\text{safe}}$）：
- $\nabla J_{\text{esdf}} = 0$，$\nabla J_{\text{obs}} = 0$
- 只有 $\nabla J_{\text{smooth}}$ 在起作用，缓慢拉直路径
- $\nabla J_{\text{dist}}$ 仅在控制点偏离原始插值位置时非零

在接近障碍物区域（$d < d_{\text{safe}}$）：
- $\nabla J_{\text{esdf}}$ 成为主导项，方向明确：远离障碍物
- 走廊约束防止过度偏离

在障碍物内部（d 很小或 $c \ge 253$）：
- $\nabla J_{\text{obs}}$ 暴增至 $\sim 10^5$ 量级，压倒所有其他项
- 数值步长 $h=0.5$ 保证了梯度的正确方向

---

## 8. 参考文献

- Harabor, D., & Grastien, A. (2011). *Online Graph Pruning for Pathfinding on Grid Maps*. AAAI.
- Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). *Distance Transforms of Sampled Functions*. Theory of Computing.
- Ren, Y., Cai, Y., Zhu, F., Liang, S., & Zhang, F. (2024). *ROG-Map: An Efficient Robocentric Occupancy Grid Map for Large-scene and High-resolution LiDAR-based Motion Planning*. IROS 2024.
- Piegl, L., & Tiller, W. (1997). *The NURBS Book*. Springer.
- Nocedal, J., & Wright, S. J. (2006). *Numerical Optimization*. Springer. (线搜索与梯度下降的理论基础)
- Eigen Splines: `unsupported/Eigen/Splines`
- Nav2 Costmap: https://docs.nav2.org/
