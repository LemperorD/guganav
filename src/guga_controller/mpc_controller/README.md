# MPC 全向移动机器人路径跟踪控制器

## 概述

本控制器基于**线性时变模型预测控制**（LTV-MPC），针对**全向移动机器人**（如 RoboMaster 哨兵机器人）设计，作为 Nav2 Controller 插件运行，在 20 Hz 频率下输出车体坐标系速度指令 $(v_x, v_y, \omega)$。

**数学模型双方案：**
- **Model A**（§3）：世界坐标系跟踪 MPC，优化变量为控制偏差 $\Delta\mathbf{U}$
- **Model B**（§4）：增广误差状态 MPC，添加终端代价保证收敛到终点

---

## 1. 机器人运动学模型

### 1.1 坐标系定义

- **世界坐标系** $\{W\}$：固定惯性系，通常以地图原点为基准
- **车体坐标系** $\{B\}$：固连于机器人质心，$x_b$ 向前、$y_b$ 向左、$z_b$ 向上（右手系）

### 1.2 全向轮运动学（连续时间）

全向移动机器人的运动学模型描述了车体速度到世界坐标速度的变换关系。

**状态变量：**

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} \in \mathbb{R}^3$$

- $x, y$：机器人在世界坐标系中的位置
- $\theta$：机器人的偏航角（yaw），从 $x$ 轴正方向逆时针为正

**控制输入：**

$$\mathbf{u} = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix} \in \mathbb{R}^3$$

- $v_x$：车体坐标系下的前向线速度
- $v_y$：车体坐标系下的横向线速度
- $\omega$：绕 $z$ 轴的角速度

**运动学微分方程：**

$$\dot{\mathbf{x}} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \underbrace{\begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{\mathbf{R}_z(\theta)} \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}$$

其中 $\mathbf{R}_z(\theta)$ 是绕 $z$ 轴的旋转矩阵（仅取 $x,y$ 平面分量）。全向轮的特点在于 $v_y \neq 0$，允许横向平动——这是与传统差速/Ackermann 模型的核心区别。

### 1.3 离散时间模型

采用**四阶 Runge-Kutta 法**（RK4）进行离散化，在精度和计算效率之间取得平衡。给定控制步长 $\Delta t$，从状态 $\mathbf{x}_k$ 和控制量 $\mathbf{u}_k$ 递推：

$$
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_k, \mathbf{u}_k) \\
\mathbf{k}_2 &= f\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1, \mathbf{u}_k\right) \\
\mathbf{k}_3 &= f\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_2, \mathbf{u}_k\right) \\
\mathbf{k}_4 &= f(\mathbf{x}_k + \Delta t \cdot \mathbf{k}_3, \mathbf{u}_k\right) \\
\mathbf{x}_{k+1} &= \mathbf{x}_k + \frac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
\end{aligned}
$$

其中 $f(\mathbf{x}, \mathbf{u}) = \mathbf{R}_z(\theta) \cdot \mathbf{u}$ 为运动学方程的右端。

当计算资源受限时，可退化为**前向欧拉法**：

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k)$$

### 1.4 线性化

将非线性离散动力学在**当前状态和参考控制附近**线性化，得到线性时变（LTV）模型：

$$\tilde{\mathbf{x}}_{k+1} = \mathbf{A}_k \tilde{\mathbf{x}}_k + \mathbf{B}_k \tilde{\mathbf{u}}_k$$

其中 $\tilde{\mathbf{x}}_k = \mathbf{x}_k - \mathbf{x}_k^{ref}$、$\tilde{\mathbf{u}}_k = \mathbf{u}_k - \mathbf{u}_k^{ref}$。

对前向欧拉离散化，Jacobian 矩阵为：

$$\mathbf{A}_k = \mathbf{I}_3 + \Delta t \cdot \left.\frac{\partial f}{\partial \mathbf{x}}\right|_{(\mathbf{x}_k^{ref}, \mathbf{u}_k^{ref})} = \begin{bmatrix} 1 & 0 & \Delta t(-v_{x,k}^{ref}\sin\theta_k^{ref} - v_{y,k}^{ref}\cos\theta_k^{ref}) \\ 0 & 1 & \Delta t(v_{x,k}^{ref}\cos\theta_k^{ref} - v_{y,k}^{ref}\sin\theta_k^{ref}) \\ 0 & 0 & 1 \end{bmatrix}$$

$$\mathbf{B}_k = \Delta t \cdot \left.\frac{\partial f}{\partial \mathbf{u}}\right|_{\theta_k^{ref}} = \Delta t \cdot \begin{bmatrix} \cos\theta_k^{ref} & -\sin\theta_k^{ref} & 0 \\ \sin\theta_k^{ref} & \cos\theta_k^{ref} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**线性化点选择**：$\mathbf{x}_k^{ref}$ 为参考轨迹在第 $k$ 步的状态，$\mathbf{u}_k^{ref}$ 为对应的参考控制量。在初始求解时，可用上一周期的解作为线性化点。

---

## 2. 路径跟踪误差定义

### 2.1 参考路径表示

Nav2 规划器输出的全局路径为离散航点序列：

$$\mathcal{P} = \{\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_M\}, \quad \mathbf{p}_i = [x_i, y_i]^T$$

参考航向由相邻航点确定：

$$\theta_i^{ref} = \text{atan2}(y_{i+1} - y_i, x_{i+1} - x_i)$$

### 2.2 跟踪误差

**位置误差**（世界坐标系）：

$$\mathbf{e}_{pos,k} = \begin{bmatrix} x_k - x_k^{ref} \\ y_k - y_k^{ref} \end{bmatrix}$$

**航向误差**（归一化到 $[-\pi, \pi]$）：

$$e_{\theta,k} = \text{wrapToPi}(\theta_k - \theta_k^{ref})$$

将位置误差投影到车体坐标系，得到**横向误差**和**纵向误差**：

$$\begin{bmatrix} e_{lon,k} \\ e_{lat,k} \end{bmatrix} = \begin{bmatrix} \cos\theta_k^{ref} & \sin\theta_k^{ref} \\ -\sin\theta_k^{ref} & \cos\theta_k^{ref} \end{bmatrix} \begin{bmatrix} x_k - x_k^{ref} \\ y_k - y_k^{ref} \end{bmatrix}$$

- $e_{lon,k}$：沿路径方向的纵向误差（落后/超前）
- $e_{lat,k}$：垂直于路径的横向误差（cross-track error）

---

## 3. MPC 最优控制问题

### 3.1 优化变量

在时刻 $t$，MPC 求解以下变量：

$$\mathbf{U}_t = [\mathbf{u}_{0|t}, \mathbf{u}_{1|t}, \ldots, \mathbf{u}_{N-1|t}]^T$$

共 $N$ 个控制量，每个为 $\mathbb{R}^3$ 向量。对应的预测状态序列为：

$$\mathbf{X}_t = [\mathbf{x}_{1|t}, \mathbf{x}_{2|t}, \ldots, \mathbf{x}_{N|t}]^T$$

其中 $\mathbf{x}_{0|t} = \mathbf{x}(t)$ 为当前机器人状态。

### 3.2 代价函数

采用二次型代价函数，包含三项：

$$J = J_{track} + J_{ctrl} + J_{smooth}$$

#### 3.2.1 状态跟踪代价

$$\boxed{J_{track} = \sum_{k=1}^{N} \|\mathbf{x}_{k|t} - \mathbf{x}_{k|t}^{ref}\|_{\mathbf{Q}}^2}$$

其中 $\mathbf{Q} = \text{diag}(q_x, q_y, q_\theta)$，权重越大对跟踪精度的要求越高。

物理意义：惩罚预测状态与参考状态的偏差。$q_x, q_y$ 控制位置跟踪精度，$q_\theta$ 控制航向跟踪精度。在全向移动场景中，$q_x$ 和 $q_y$ 通常取相同值（各向同性）。

#### 3.2.2 控制代价

$$\boxed{J_{ctrl} = \sum_{k=0}^{N-1} \|\mathbf{u}_{k|t} - \mathbf{u}_{k|t}^{ref}\|_{\mathbf{R}}^2}$$

其中 $\mathbf{R} = \text{diag}(r_{vx}, r_{vy}, r_\omega)$。

物理意义：惩罚偏离参考速度的控制量。在无参考速度（仅路径跟踪）时，$\mathbf{u}^{ref} = \mathbf{0}$，此项即控制能量的正则化。

#### 3.2.3 控制平滑代价

$$\boxed{J_{smooth} = \sum_{k=0}^{N-2} \|\mathbf{u}_{k+1|t} - \mathbf{u}_{k|t}\|_{\mathbf{R}_d}^2}$$

其中 $\mathbf{R}_d = \text{diag}(r_{dvx}, r_{dvy}, r_{d\omega})$。

物理意义：抑制相邻控制步之间的大幅跳变，保证速度指令平滑。这相当于对控制变化率施加软约束，避免机械冲击。

### 3.3 约束

#### 硬约束

$$\mathbf{u}_{min} \leq \mathbf{u}_{k|t} \leq \mathbf{u}_{max}, \quad \forall k = 0, 1, \ldots, N-1$$

即：

$$\begin{bmatrix} v_{x,min} \\ v_{y,min} \\ \omega_{min} \end{bmatrix} \leq \begin{bmatrix} v_{x,k} \\ v_{y,k} \\ \omega_{k} \end{bmatrix} \leq \begin{bmatrix} v_{x,max} \\ v_{y,max} \\ \omega_{max} \end{bmatrix}$$

对于 RoboMaster 哨兵机器人，典型范围为：
- $v_x \in [-3.0, 3.0]$ m/s
- $v_y \in [-3.0, 3.0]$ m/s
- $\omega \in [-6.0, 6.0]$ rad/s

#### 软约束（可选，通过松弛变量实现）

避免优化问题不可行：

- **走廊约束**：将状态约束在参考路径的 $\pm d_{max}$ 范围内
- **终点约束**：$\|\mathbf{x}_{N|t} - \mathbf{x}_{N|t}^{ref}\| \leq \epsilon_{term}$

### 3.4 优化问题紧凑形式

将 LTV 预测模型、代价函数和约束合并，得到标准**二次规划**（QP）形式：

$$\boxed{\begin{aligned} \min_{\Delta\mathbf{U}} \quad & \frac{1}{2} \Delta\mathbf{U}^T \mathbf{H} \Delta\mathbf{U} + \mathbf{g}^T \Delta\mathbf{U} \\ \text{s.t.} \quad & \mathbf{u}_{min} \leq \Delta\mathbf{u}_k + \mathbf{u}_k^{ref} \leq \mathbf{u}_{max}, \quad \forall k \end{aligned}}$$

其中 $\Delta\mathbf{U} = [\tilde{\mathbf{u}}_{0|t}, \ldots, \tilde{\mathbf{u}}_{N-1|t}]^T$ 是相对于参考控制的偏差。

Hessian 矩阵 $\mathbf{H}$ 和梯度向量 $\mathbf{g}$ 由运动学模型的线性化 Jacobian 矩阵 $(\mathbf{A}_k, \mathbf{B}_k)$ 和权重矩阵 $(\mathbf{Q}, \mathbf{R}, \mathbf{R}_d)$ 的递推组装而成（详见 [H 和 g 矩阵组装流程](#8-h-和-g-矩阵组装流程)）。

### 3.5 滚动时域（Receding Horizon）

1. 在当前时刻 $t$，测量机器人状态 $\mathbf{x}(t)$
2. 求解 QP 得到最优控制序列 $\mathbf{U}_t^*$
3. **仅执行第一个控制量** $\mathbf{u}_{0|t}^*$，发布为 `TwistStamped`
4. 前进到 $t + \Delta t$，重复步骤 1

---

## 4. Model B — 增广误差状态预测控制（Augmented Error-State MPC）

### 4.1 核心思想

Model A 在**世界坐标系**中逐点跟踪参考轨迹，代价函数鼓励状态 $\mathbf{x}_k$ 逼近参考状态 $\mathbf{x}_k^{ref}$。然而：
- 参考轨迹的终点 $\mathbf{x}_N^{ref}$ 可能并非机器人目标位置
- 缺乏对**终点到达精度**的显式保证
- 稳态误差不可由代价参数直接调控

Model B 将 MPC 问题重构为**误差动力学空间**中的调节问题（regulator），增广弧长参数 $s$ 使系统能显式地沿路径推进并收敛到终点。

**关键改进：**
- $\checkmark$ 增广状态向量 $\mathbf{z} = [e_x, e_y, e_\theta, v_x, v_y, \omega, s]^T \in \mathbb{R}^7$
- $\checkmark$ 终端代价 $\|\mathbf{e}_N\|_{P_N}^2$ 显式惩罚预测终点误差
- $\checkmark$ $\mathbf{y}_{ref} = \mathbf{0}$ — 目标是将所有误差驱至零
- $\checkmark$ 弧长 $s$ 动态由切向速度驱动，自然地沿路径向前滚动

### 4.2 增广状态变量

$$\mathbf{z} = \begin{bmatrix} e_x \\ e_y \\ e_\theta \\ v_x \\ v_y \\ \omega \\ s \end{bmatrix} \in \mathbb{R}^7$$

| 符号 | 物理意义 | 坐标系 |
|------|---------|--------|
| $e_x$ | 沿参考路径切向的位置误差（纵向误差） | Frenet 框架 |
| $e_y$ | 垂直参考路径的位置误差（横向误差 / cross-track error） | Frenet 框架 |
| $e_\theta$ | 航向角误差（$\theta - \theta^{ref}$，包装至 $[-\pi, \pi]$） | 世界 |
| $v_x, v_y, \omega$ | 当前/上一时刻的控制量（用于平滑代价） | 车体 |
| $s$ | 沿参考路径的弧长参数 | — |

### 4.3 控制变量

控制量不直接作为优化变量，而是优化**相对于参考控制的偏差**：

$$\tilde{\mathbf{u}}_k = \begin{bmatrix} \tilde{v}_{x,k} \\ \tilde{v}_{y,k} \\ \tilde{\omega}_k \end{bmatrix} = \begin{bmatrix} v_{x,k} - v_{x,k}^{ref} \\ v_{y,k} - v_{y,k}^{ref} \\ \omega_k - \omega_k^{ref} \end{bmatrix}$$

加速度耦合关系：
$$u_k = u_k^{ref} + \tilde{u}_k$$

### 4.4 误差动力学（连续时间）

误差动态在 Frenet 框架中推导。令参考航向为 $\theta^{ref}(s)$，路径曲率为 $\kappa(s)$。

**纵向误差动态**（沿路径切线方向）：
$$\dot{e}_x = v_x\cos e_\theta - v_y\sin e_\theta - \dot{s}(1 - \kappa e_y)$$

**横向误差动态**（沿路径法线方向）：
$$\dot{e}_y = v_x\sin e_\theta + v_y\cos e_\theta - \dot{s}\kappa e_x$$

**航向误差动态**：
$$\dot{e}_\theta = \omega - \dot{s}\kappa(s)$$

**弧长动态**（由切向速度驱动）：
$$\dot{s} = \frac{v_x\cos e_\theta - v_y\sin e_\theta}{1 - \kappa(s) e_y}$$

（在 $\kappa e_y \ll 1$ 且 $e_\theta$ 较小时，$\dot{s} \approx v_x$）

### 4.5 离散化与代价函数

**离散动力学**（前向欧拉，步长 $dt$）：
$$\mathbf{z}_{k+1} = \mathbf{z}_k + dt \cdot f_{err}(\mathbf{z}_k, \tilde{\mathbf{u}}_k, s_k)$$

其中 $f_{err}$ 为 §4.4 的右端函数在离散时刻 $t_k$ 的求值。

**非线性最小二乘代价**：

**阶段代价**（$k = 0, 1, \ldots, N-1$）：
$$h_{ls}(\mathbf{z}_k, \tilde{\mathbf{u}}_k) = \begin{bmatrix} e_x \\ e_y \\ e_\theta \\ \tilde{v}_x \\ \tilde{v}_y \\ \tilde{\omega} \\ \tilde{v}_x - v_x^{prev} \\ \tilde{v}_y - v_y^{prev} \\ \tilde{\omega} - \omega^{prev} \end{bmatrix} \in \mathbb{R}^9$$

$$\boxed{J_{stage,k} = \frac{1}{2} \|h_{ls}(\mathbf{z}_k, \tilde{\mathbf{u}}_k)\|_{\mathbf{W}}^2}$$

$\mathbf{W} = \text{diag}(w_{ex}, w_{ey}, w_{e\theta}, w_{vx}, w_{vy}, w_{\omega}, w_{dvx}, w_{dvy}, w_{d\omega})$

**终端代价**（$k = N$）：
$$h_{ls}^N(\mathbf{z}_N) = \begin{bmatrix} e_x \\ e_y \\ e_\theta \end{bmatrix} \in \mathbb{R}^3$$

$$\boxed{J_{term} = \frac{1}{2} \|h_{ls}^N(\mathbf{z}_N)\|_{\mathbf{W}_N}^2}$$

**终端权重矩阵放大**以强化终点收敛：
$$\mathbf{W}_N = w_{term} \cdot \text{diag}(w_{ex}, w_{ey}, w_{e\theta})$$

典型取值 $w_{term} = 10 \sim 100$，物理意义是将终端误差惩罚放大 10–100 倍。

### 4.6 优化问题

$$\boxed{\begin{aligned} \min_{\tilde{\mathbf{U}} = [\tilde{\mathbf{u}}_0, \ldots, \tilde{\mathbf{u}}_{N-1}]} \quad & \sum_{k=0}^{N-1} \frac{1}{2} \|h_{ls}(\mathbf{z}_k, \tilde{\mathbf{u}}_k)\|_{\mathbf{W}}^2 + \frac{1}{2} \|h_{ls}^N(\mathbf{z}_N)\|_{\mathbf{W}_N}^2 \\ \text{s.t.} \quad & \mathbf{z}_{k+1} = \mathbf{z}_k + dt \cdot f_{err}(\mathbf{z}_k, \tilde{\mathbf{u}}_k), \quad k = 0,\ldots,N-1 \\ & \mathbf{z}_0 = \mathbf{z}(t) \quad \text{(当前增广状态)} \\ & \mathbf{u}_{min} \leq \mathbf{u}_k^{ref} + \tilde{\mathbf{u}}_k \leq \mathbf{u}_{max}, \quad \forall k \end{aligned}}$$

### 4.7 Model A vs Model B 对比

| 维度 | Model A | Model B |
|------|---------|---------|
| 状态空间 | $\mathbb{R}^3$ 世界系 | $\mathbb{R}^7$ 增广误差系 |
| 代价参考 | $\mathbf{y}_k^{ref} = [x_k^{ref}, y_k^{ref}, \theta_k^{ref}]$ 逐点变化 | $\mathbf{y}_{ref} = \mathbf{0}$（常数，所有误差趋向于零） |
| 终端代价 | 增大 $Q$ 照抄阶段代价 | 独立的终端权重 $W_N = w_{term} \cdot Q_e$ |
| 终点收敛 | 软约束，依赖路径准确 | 强约束，终端权重保证 |
| 弧长推进 | 从路径上采样 $N$ 点 | $s$ 动力学自主推进 |
| 计算量 | $O(N \cdot 3^2)$ 3x3 分块矩阵 | $O(N \cdot 7^2)$ 7x7 块 — 但 acados SQP 自动处理 |
| 适用场景 | 跟踪预定义平滑路径 | 精确到达端点，容错更大 |

**Model B 的优势**：
1. 终端代价相当于隐式的终端不变集约束（terminal invariant set），理论上可保证渐近稳定
2. $s$ 的自主推进避免了"回头"参考点选取的模糊性
3. $\mathbf{y}_{ref} = 0$ 简化了参考设置，模型匹配参数调整更直观

---

## 5. 参考轨迹生成

### 5.1 前瞻点选取

在全局路径上以**速度自适应前瞻距离**选取参考起点，再向前取 $N$ 个等距采样点作为预测时域内的参考轨迹。

$$d_{lookahead} = d_{min} + k_v \cdot \|\mathbf{v}\|$$

其中 $d_{min}$ 为最小前瞻距离，$k_v$ 为速度比例系数，$\|\mathbf{v}\|$ 为当前车速范数。

### 5.2 参考速度生成

参考速度由路径曲率和最大速度约束共同决定：

$$v^{ref} = v_{max} \cdot \frac{1}{1 + \kappa_{scale} \cdot |\kappa|}$$

- $\kappa$：路径曲率，通过三点圆弧法计算
- $\kappa_{scale}$：曲率缩放系数，曲率越大速度越低
- 参考角速度：$\omega^{ref} = v^{ref} \cdot \kappa$

### 5.3 参考航向

由参考路径的切线方向确定：

$$\theta_k^{ref} = \text{atan2}(y_{k+1}^{ref} - y_k^{ref}, \quad x_{k+1}^{ref} - x_k^{ref})$$

---

## 6. 参数总表

### 6.1 MPC 核心参数

| 参数 | 符号 | 单位 | 典型值 | 说明 |
|------|------|------|--------|------|
| 预测时域 | $N$ | - | 15 | 预测步数，平衡精度与计算量 |
| 控制步长 | $\Delta t$ | s | 0.05 | 与 Nav2 20 Hz 控制器频率对齐 |
| 预测时长 | $T = N\Delta t$ | s | 0.75 | 预测窗口总时长 |

### 6.2 状态跟踪权重

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| 纵向位置权重 | $q_x$ | 10.0 | $x$ 方向跟踪偏差惩罚 |
| 横向位置权重 | $q_y$ | 10.0 | $y$ 方向跟踪偏差惩罚 |
| 航向角权重 | $q_\theta$ | 2.0 | $\theta$ 角度跟踪偏差惩罚 |

### 6.3 控制权重

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| 前向速度权重 | $r_{vx}$ | 0.1 | 抑制过大的前向速度指令 |
| 横向速度权重 | $r_{vy}$ | 0.1 | 抑制过大的横向速度指令 |
| 角速度权重 | $r_\omega$ | 0.05 | 抑制过大的角速度指令 |
| 速度变化权重 | $r_{dvx}, r_{dvy}, r_{d\omega}$ | 0.5 | 控制平滑性惩罚 |
| 终端误差放大 | $w_{term}$ | 20.0 | Model B 专属，强化终态收敛 |

### 6.4 物理约束

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| 最大前向速度 | $v_{x,max}$ | 3.0 m/s | 正向最大线速度 |
| 最大后退速度 | $v_{x,min}$ | -3.0 m/s | 反向最大线速度 |
| 最大横向速度 | $v_{y,max}$ | 3.0 m/s | 横向最大速度 |
| 最大角速度 | $\omega_{max}$ | 6.0 rad/s | 绝对值 |

### 6.5 路径跟踪参数

| 参数 | 符号 | 单位 | 典型值 | 说明 |
|------|------|------|--------|------|
| 最小前瞻距离 | $d_{min}$ | m | 0.2 | 低速时最小前瞻量 |
| 前瞻速度系数 | $k_v$ | s | 1.0 | 前瞻量与速度的比例系数 |
| 曲率缩放系数 | $\kappa_{scale}$ | m | 2.0 | 曲率对参考速度的衰减强度 |
| 参考路径最大长度 | $d_{path,max}$ | m | 5.0 | 路径截断长度（costmap范围） |
| 离散化方式 | - | - | RK4 | 运动学模型离散化方法 |

---

## 7. 轨迹接口扩展设计

### 7.1 接口抽象

MPC 控制器的参考轨迹通过抽象接口 `TrajectoryProvider` 接入，支持多种表示形式：

```
                    ┌─────────────────┐
                    │  Nav2 Planner    │
                    └────────┬────────┘
                             │ nav_msgs::Path (离散航点)
                             ▼
              ┌──────────────────────────┐
              │    TrajectoryProvider    │  ← 抽象接口
              │  + eval(s) → ReferencePt │
              │  + curvature(s) → double │
              │  + arcLength() → double  │
              └──────┬─────────┬─────────┘
                     │         │
            ┌────────┘         └────────────┐
            ▼                               ▼
   ┌─────────────────┐            ┌──────────────────┐
   │ DiscreteProvider│            │SplineProvider    │
   │ (线性插值)      │            │ (B样条/MINCO)    │
   └─────────────────┘            └──────────────────┘
```

### 7.2 轨迹提供者模式

| 模式 | 来源 | eval 方式 | 适用场景 |
|------|------|-----------|---------|
| `DISCRETE` | Nav2 Path 航点 | 线性/三次插值 | 标准 Nav2 规划输出 |
| `B_SPLINE` | B 样条控制点 | de Boor 算法 | 离线平滑轨迹 |
| `MINCO` | MINCO 参数 | MINCO 解析求值 | 在线轨迹优化 |

通过 ROS2 parameter `reference_mode`（值 0/1/2）在运行时切换。

### 7.3 统一内部表示

无论外部输入格式如何，`TrajectoryProvider` 在预测时域内产生统一结构：

```
struct ReferencePoint {
  double x, y, theta;    // 参考位姿
  double vx, vy, omega;  // 参考速度（可选）
  double curvature;      // 路径曲率
  double s;              // 路径弧长参数
};
```

MPC 求解器仅依赖这个统一接口，与轨迹表示形式解耦。

---

## 8. 数值求解

### 8.1 求解器选择

| 求解器 | 语言 | 适用阶段 | 说明 |
|--------|------|---------|------|
| CasADi | Python | 仿真/原型 | 自动微分 + IPOPT，验证模型 |
| acados | C/Python | 仿真/部署 | 实时 QP 求解器（HPIPM），性能最优 |
| osqp-eigen | C++ | 部署 | ROS2 常用，apt 安装，集成简单 |

### 8.2 实时性保证

- **最大求解时间**：$< 5$ ms（包括线性化 + QP 组装 + 求解）
- **降级策略**：若求解超时，回退到零阶保持（使用上一周期的控制量）
- **QP 预热**：利用上一周期的解作为当前 QP 的初始猜测（warm start），减少迭代次数

---

## 9. 参考文献

1. Künzle, P., et al. "acados — a modular open-source framework for fast embedded optimal control." *Mathematical Programming Computation*, 2022.
2. Frasch, J. V., et al. "An auto-generated nonlinear MPC algorithm for real-time obstacle avoidance of ground vehicles." *European Control Conference (ECC)*, 2013.
3. Macenski, S., et al. "The Marathon 2: A Navigation System." *IROS*, 2023.
4. Rawlings, J. B., Mayne, D. Q., & Diehl, M. M. *Model Predictive Control: Theory, Computation, and Design*. Nob Hill Publishing, 2017.
