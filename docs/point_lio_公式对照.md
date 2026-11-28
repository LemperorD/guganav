# Point-LIO 原版与当前实现的公式对照

## 一、对照基准与结论

两版的状态转移、IMU 量测方程、点面残差与雅可比的代数形式一致。对照分析中识别出的五处核心差异为：过程噪声的时间尺度与取值、初始协方差、重力是否在线估计、点面匹配阈值、六面标定与零偏初值；另有残差截断、启动窗口与地图参数三处工程性差异。其中前三项（过程噪声与初始协方差解耦对齐、重力切平面常值在线微调、实车 15° 重力角同步）现已在当前代码与实车配置中完成对齐，下文同步标注修改前后的状态。

原版取本仓库历史提交 fc0361c 中的 `src/guga_thirdparty/point_lio`。这份代码保留了 IKFoM 的 30 维输出模型，公式与 Point-LIO 论文一致，下文称为原版。当前实现取 `src/guga_localization/LIO/point_lio` 的手写 24 维 ESKF，下文称为当前版。两版都运行在输出模型下（原版的 `use_imu_as_input = false`）。

有一点先说明：两版的雷达更新在 Livox 数据上都是逐点更新，单点噪声都取 0.01，这一项没有差异。原版代码里另有一个信息形式分支，只有当一次更新包含 30 个以上有效点时才启用，按 Livox 的点组划分不会进入这个分支。第五节给出依据。

## 二、状态与误差定义

原版的状态是 30 维，顺序为位置、旋转、雷达外参旋转、雷达外参平移、速度、角速度、加速度、重力、陀螺零偏、加速度零偏。当前版的状态是 24 维，顺序为旋转、位置、速度、陀螺零偏、加速度零偏、重力、角速度、加速度。

| 项目 | 原版 | 当前版 |
| --- | --- | --- |
| 维数 | 30 | 24 |
| 位置 | 列 0 到 2 | 列 3 到 5 |
| 旋转 | 列 3 到 5 | 列 0 到 2 |
| 外参旋转、外参平移 | 列 6 到 8、列 9 到 11，在线估计 | 不作为状态，固定 |
| 速度 | 列 12 到 14 | 列 6 到 8 |
| 角速度、加速度 | 列 15 到 17、列 18 到 20 | 列 18 到 20、列 21 到 23 |
| 重力 | 列 21 到 23，在线估计（$\mathbb{R}^3$） | 列 15 到 17，常值在线微调（二维切平面，$Q_g=0$） |
| 陀螺零偏、加速度零偏 | 列 24 到 26、列 27 到 29 | 列 9 到 11、列 12 到 14 |

两版的误差更新都用右扰动。对旋转，误差为零时写成 $\mathbf{R} \leftarrow \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\theta})$；对向量状态，误差为零时写成 $\mathbf{x} \leftarrow \mathbf{x} + \delta\mathbf{x}$。当前版的 `State::boxplus` 按这个定义实现，原版的 `SOn` 流形也按这个定义实现。

原版给重力块初始方差 $10^{-4}\mathbf{I}$，并在状态转移里保留 $\partial\dot{\mathbf{v}}/\partial\mathbf{g} = \mathbf{I}$、过程噪声 $\mathbf{Q}_g = \mathbf{0}$。当前版曾将重力块清零冻结，现已改为**二维切平面常值在线微调**（`init_gravity_cov = 1e-4` 启用，置 `0` 退回冻结），同时在 `predictCovariance` 中恢复 $\partial\dot{\mathbf{v}}/\partial\mathbf{g} = \Delta t\mathbf{I}$、保持 $\mathbf{Q}_g = \mathbf{0}$：

$$
\mathbf{P}_{15:17,\,15:17}^{(0)} = \sigma_{g,0}^2\left(\mathbf{I} - \hat{\mathbf{g}}_0\hat{\mathbf{g}}_0^{\mathsf{T}}\right)
$$

由于 $\mathbf{Q}_g = \mathbf{0}$，径向零空间 $\hat{\mathbf{g}}_0^{\mathsf{T}}\mathbf{P}_{15:17,*} = \mathbf{0}$ 在传播与更新中严格保持，滤波器只在线修正重力方向、锁定模长（更新后同时重归一到 $g_0$ 消去二阶伸长），避免平地运动时 $g_z$ 与 $b_{a,z}$ 耦合发散。

## 三、状态传播

### 3.1 连续动力学

两版的连续方程相同，均满足 $\dot{\mathbf{g}} = \mathbf{0}$ 并保留 $\partial\dot{\mathbf{v}}/\partial\mathbf{g} = \mathbf{I}$。

$$
\dot{\mathbf{R}} = \mathbf{R}\,[\boldsymbol{\omega}]_\times,\qquad
\dot{\mathbf{p}} = \mathbf{v},\qquad
\dot{\mathbf{v}} = \mathbf{R}\,\mathbf{a} + \mathbf{g}
$$

$$
\dot{\mathbf{b}}_g = \mathbf{n}_{bg},\qquad
\dot{\mathbf{b}}_a = \mathbf{n}_{ba},\qquad
\dot{\boldsymbol{\omega}} = \mathbf{n}_{g},\qquad
\dot{\mathbf{a}} = \mathbf{n}_{a},\qquad
\dot{\mathbf{g}} = \mathbf{0}
$$

这里的 $\mathbf{R}$ 是机体到世界系的旋转，无量纲；$\mathbf{p}$ 是位置，单位米；$\mathbf{v}$ 是速度，单位米每秒；$\mathbf{a}$ 是机体坐标系下的比力，单位米每二次方秒；$\mathbf{g}$ 是世界系重力，单位米每二次方秒；$\boldsymbol{\omega}$ 是机体角速度，单位弧度每秒；$\mathbf{b}_g$ 与 $\mathbf{b}_a$ 分别是陀螺零偏与加速度零偏；$\mathbf{n}_{bg}$、$\mathbf{n}_{ba}$、$\mathbf{n}_{g}$、$\mathbf{n}_{a}$ 是四项过程噪声。

### 3.2 状态传播公式

两版都用一阶积分，把连续导数乘以时间间隔后做流形加法。

原版在 `get_f_output` 里给出

$$
\mathbf{f} = \begin{bmatrix}\mathbf{v}\\ \boldsymbol{\omega}\\ \mathbf{R}\mathbf{a} + \mathbf{g}\\ \mathbf{0}\\ \vdots\end{bmatrix},
\qquad
\mathbf{x}_{k+1} = \mathbf{x}_k \boxplus \left(\Delta t\,\mathbf{f}(\mathbf{x}_k)\right)
$$

当前版在 `ManifoldEskf::f_` 里写同一组分量的另一个排列。

$$
\mathbf{f} = \begin{bmatrix}\boldsymbol{\omega}\\ \mathbf{v}\\ \mathbf{R}\mathbf{a} + \mathbf{g}\\ \mathbf{0}_{15}\end{bmatrix}
$$

### 3.3 协方差传播公式

原版在 IKFoM 的 `predict` 里先构造状态转移矩阵 $\mathbf{F}_x$，再算

$$
\mathbf{P}_{k+1} = \mathbf{F}_x\,\mathbf{P}_k\,\mathbf{F}_x^{\mathsf{T}} + \mathbf{Q}\,\Delta t^2
$$

旋转块取 $\mathbf{F}_x(3{:}5,3{:}5) = \mathrm{Exp}(-\boldsymbol{\omega}\Delta t)$，旋转对其余状态的耦合经过 $\mathbf{A}(-\boldsymbol{\omega}\Delta t)$ 投影。其余块由 `df_dx_output` 给出：位置对速度 $\Delta t\mathbf{I}$，速度对旋转 $-\Delta t\mathbf{R}[\mathbf{a}]_\times$，速度对重力 $\Delta t\mathbf{I}$，速度对加速度 $\Delta t\mathbf{R}$，旋转对角速度 $\Delta t\mathbf{I}$。

论文把旋转对角速度这一块写成一阶形式 $\Delta t\mathbf{I}$（即论文里的 $\mathbf{F}_{1,7}$），当前版与论文一致。原版的 $\mathbf{A}$ 投影来自 IKFoM 通用流形实现对 $\mathrm{Exp}$ 求导时多保留的一项，属于二阶修正，不是论文的要求。

当前版在 `ManifoldEskf::predictCovariance` 里按块乘两次实现同一个 $\mathbf{F}_x\mathbf{P}\mathbf{F}_x^{\mathsf{T}}$。块结构与原版一致，旋转对角速度的耦合取 $\Delta t\mathbf{I}$，噪声项写成

$$
\mathbf{P}_{k+1} = \mathbf{F}_x\,\mathbf{P}_k\,\mathbf{F}_x^{\mathsf{T}} + \mathrm{diag}(\mathbf{Q})\,\Delta t
$$

$\mathrm{diag}(\mathbf{Q})$ 只把四个 $3\times3$ 块加到零偏、角速度、加速度的对角块上。原版多出的 $\mathbf{A}$ 投影与 $\Delta t\mathbf{I}$ 相差 $\tfrac12\|\boldsymbol{\omega}\|\Delta t^2$，在 $\Delta t = 5$ 毫秒、$\|\boldsymbol{\omega}\| = 1$ 弧度每秒时约为 $1.3\times10^{-5}$，可以忽略。噪声项的时间尺度差别不能忽略：原版按离散零阶保持写成 $\mathbf{Q}_{\text{disc}}\Delta t^2$（且默认开启 `prop_at_freq_of_imu: True` 将协方差步长锁定在 5 毫秒），当前版按连续功率谱密度写成 $\mathbf{Q}_c\Delta t$，在变步长与逐雷达点传播下满足时间切分不变性；两者在固定 200 赫兹下的换算关系为 $\mathbf{Q}_c = \mathbf{Q}_{\text{disc}}\cdot 0.005$。

### 3.4 过程噪声的数值对照

取 IMU 频率 200 赫兹，$\Delta t = 5$ 毫秒，按两版的参数文件计算每 5 毫秒注入的方差（当前版实车已按 $\mathbf{Q}_c = \mathbf{Q}_{\text{disc}}\cdot 0.005$ 对齐原版，仿真保留独立参数）：

| 状态块 | 原版 | 当前版（实车，已对齐） | 当前版（仿真） |
| --- | --- | --- | --- |
| 角速度 $\boldsymbol{\omega}$ | $1000 \times \Delta t^2 = 0.025$ | $5.0 \times \Delta t = 0.025$ | $100 \times \Delta t = 0.5$ |
| 加速度 $\mathbf{a}$ | $500 \times \Delta t^2 = 0.0125$ | $2.5 \times \Delta t = 0.0125$ | $50 \times \Delta t = 0.25$ |
| 陀螺零偏 $\mathbf{b}_g$ | $10^{-4} \times \Delta t^2 = 2.5\times10^{-9}$ | $5\times10^{-7} \times \Delta t = 2.5\times10^{-9}$ | $10^{-4} \times \Delta t = 5\times10^{-7}$ |
| 加速度零偏 $\mathbf{b}_a$ | 同上 | 同上 | 同上 |
| 速度 $\mathbf{v}$ | $20 \times \Delta t^2 = 5\times10^{-4}$ | 无独立噪声项 | 无独立噪声项 |

实车配置同步后，四项过程噪声的每 5 毫秒注入方差与原版一致；仅差原版的速度独立过程噪声项，当前版速度方差通过加速度块的 $\Delta t\mathbf{R}$ 耦合获得。

### 3.5 初始协方差

原版 `reset_cov_output` 给位置、旋转、速度、角速度、加速度都取 $0.01$，重力取 $10^{-4}$，两个零偏取 $10^{-3}$。当前版 `ManifoldEskf::initialize` 曾把角速度、加速度和零偏的初始方差直接复用过程噪声参数，现已拆出独立的 `init_omg_cov`、`init_acc_cov`、`init_b_gyr_cov`、`init_b_acc_cov`、`init_gravity_cov`，与原版对齐：

| 状态块 | 原版 | 当前版（实车，已解耦对齐） | 当前版（仿真，默认） |
| --- | --- | --- | --- |
| 位置、旋转、速度 | 0.01 | 0.01 | 0.01 |
| 角速度 | 0.01 | 0.01 | 0.01 |
| 加速度 | 0.01 | 0.01 | 0.01 |
| 陀螺零偏、加速度零偏 | $10^{-3}$ | $10^{-3}$ | $10^{-3}$ |
| 重力 | $10^{-4}\mathbf{I}$ | $10^{-4}(\mathbf{I}-\hat{\mathbf{g}}_0\hat{\mathbf{g}}_0^{\mathsf{T}})$ | $10^{-4}(\mathbf{I}-\hat{\mathbf{g}}_0\hat{\mathbf{g}}_0^{\mathsf{T}})$ |

解耦后，调过程噪声功率谱密度 $\mathbf{Q}_c$ 不再连带改动初始协方差 $\mathbf{P}_0$。

## 四、IMU 量测更新

### 4.1 量测方程

两版的残差与雅可比相同。

$$
\mathbf{z} = \begin{bmatrix}\boldsymbol{\omega}_m\\ \mathbf{a}_m\end{bmatrix}
- \begin{bmatrix}\boldsymbol{\omega} + \mathbf{b}_g\\ \mathbf{a} + \mathbf{b}_a\end{bmatrix},
\qquad
\mathbf{H} = \begin{bmatrix}\mathbf{I} & \mathbf{0} & \mathbf{I} & \mathbf{0}\\ \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{I}\end{bmatrix}
$$

$\mathbf{H}$ 的行块依次对应角速度残差对零偏与角速度、加速度残差对零偏与加速度。原版在 IKFoM 里用 $\mathbf{P}$ 的第 15 到 17 列和第 24 到 26 列实现同一个作用，当前版用 `H_I` 的四个单位块实现，两者代数等价。

### 4.2 协方差与增益

原版：

$$
\mathbf{K} = \mathbf{P}\mathbf{H}^{\mathsf{T}}\left(\mathbf{H}\mathbf{P}\mathbf{H}^{\mathsf{T}} + \mathbf{R}\right)^{-1},\qquad
\mathbf{P}^{+} = \mathbf{P} - \mathbf{K}\mathbf{H}\mathbf{P}
$$

当前版：

$$
\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^{\mathsf{T}} + \mathbf{R},\qquad
\mathbf{K} = \mathbf{P}\mathbf{H}^{\mathsf{T}}\mathbf{S}^{-1},\qquad
\mathbf{P}^{+} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^{\mathsf{T}} + \mathbf{K}\mathbf{R}\mathbf{K}^{\mathsf{T}}
$$

当前版用 Joseph 形式，与原版的一阶形式在精确算术下等价。当前版在更新后还把前三维乘以右雅可比 $\mathbf{J}_r = \mathbf{I} - \tfrac12[\delta\boldsymbol{\theta}]_\times + \cdots$ 做误差重置，原版不做这一步。旋转修正量在 0.01 弧度时 $\mathbf{J}_r$ 与单位阵相差约 0.5%，影响很小。

### 4.3 量测噪声取值

| 参数 | 原版 | 当前版（实车，已对齐） | 当前版（仿真） |
| --- | --- | --- | --- |
| `imu_meas_omg_cov` | 0.01 | 0.01 | 1.0 |
| `imu_meas_acc_cov` | 0.01 | 0.01 | 20.0 |

实车量测方差现已与原版对齐为 0.01；仿真因 Gazebo 加速度坏点保留较大值。

## 五、雷达量测更新

### 5.1 残差与雅可比

两版的残差都取平面方程的负值：

$$
r = -\left(\mathbf{n}^{\mathsf{T}}\mathbf{p}_w + d\right),
\qquad
\mathbf{p}_w = \mathbf{R}\left(\mathbf{R}_{LI}\mathbf{p}_L + \mathbf{t}_{LI}\right) + \mathbf{p}
$$

$\mathbf{n}$ 是单位法向量，无量纲；$d$ 是平面截距，单位米；$\mathbf{p}_L$ 是雷达系点坐标，单位米；$\mathbf{R}_{LI}$ 与 $\mathbf{t}_{LI}$ 是雷达到 IMU 的固定外参；$\mathbf{p}$ 是 IMU 位置，单位米。对右扰动 $\mathbf{R}\leftarrow\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\theta})$，有

$$
\frac{\partial}{\partial\delta\boldsymbol{\theta}}\left(\mathbf{n}^{\mathsf{T}}\mathbf{p}_w\right)
= \left([\mathbf{p}_{imu}]_\times\,\mathbf{R}^{\mathsf{T}}\mathbf{n}\right)^{\mathsf{T}}
= \mathbf{A}^{\mathsf{T}}
$$

其中 $\mathbf{p}_{imu} = \mathbf{R}_{LI}\mathbf{p}_L + \mathbf{t}_{LI}$，$\mathbf{A} = [\mathbf{p}_{imu}]_\times\mathbf{R}^{\mathsf{T}}\mathbf{n}$。所以残差对状态的一行雅可比是 $\left[\mathbf{A}^{\mathsf{T}},\ \mathbf{n}^{\mathsf{T}}\right]$，对应旋转与位置。原版状态顺序是位置在前，写成 $\left[\mathbf{n}^{\mathsf{T}},\ \mathbf{A}^{\mathsf{T}}\right]$；当前版旋转在前，写成 $\left[\mathbf{A}^{\mathsf{T}},\ \mathbf{n}^{\mathsf{T}}\right]$。两版的符号约定一致。

点的筛选条件也相同：近邻点不少于 5 个，`esti_plane` 判定平面有效，且满足

$$
\|\mathbf{p}_{imu}\| > \texttt{match\_s}\times d_{\perp}^2
$$

$d_{\perp}$ 是点到平面的距离，单位米；`match_s` 是无量纲阈值。原版取 81，当前版取 10。取 10 米远的点、$\|\mathbf{p}_{imu}\| = 10$ 米计算，原版接受的点面距离上界是 $0.35$ 米，当前版是 $1.0$ 米。

### 5.2 点组划分与更新次数

两版都先把降采样后的点云按时间偏移升序排序，再用 `time_compressing` 切分点组。这个函数在「后一点的时间偏移大于前一点」时切一刀。排序之后时间偏移单调不减，切分条件在每一步都成立，所以点组退化为单个点，或者退化为时间偏移完全相同的若干点。Livox 的点时间偏移以纳秒给出、按毫秒存成单精度浮点，相邻点通常相差几微秒，因此点组就是一个点。

原版的更新函数按点组内有效点数 $N$ 分成两个分支：

1. $N < 30$ 时走协方差分支，$\mathbf{K} = \mathbf{P}_{0:12}\mathbf{H}^{\mathsf{T}}\left(\mathbf{H}\mathbf{P}_{0:12}\mathbf{H}^{\mathsf{T}} + M\mathbf{I}\right)^{-1}$，$M$ 取 `lidar_meas_cov` 并按方差使用。
2. $N \ge 30$ 时走信息分支，$\mathbf{P}_{\text{inv}}^{+} = \mathbf{P}^{-1} + M\mathbf{H}^{\mathsf{T}}\mathbf{H}$，$\mathbf{K} = \mathbf{P}^{+}\mathbf{H}^{\mathsf{T}}M$，$M$ 被当作信息量，等效单点方差是 $1/M$。

当前版没有分支，逐点做标量更新：

$$
s = \mathbf{h}^{\mathsf{T}}\mathbf{P}\mathbf{h} + \sigma^2,\qquad
\mathbf{g} = \mathbf{P}\mathbf{h},\qquad
\mathbf{k} = \mathbf{g}/s
$$

$$
\delta\mathbf{x} \mathrel{+}= \mathbf{k}\left(r - \mathbf{h}^{\mathsf{T}}\delta\mathbf{x}\right),\qquad
\mathbf{P} \leftarrow \mathbf{P} - \mathbf{k}\mathbf{g}^{\mathsf{T}} - \mathbf{g}\mathbf{k}^{\mathsf{T}} + s\,\mathbf{k}\mathbf{k}^{\mathsf{T}}
$$

$\sigma^2$ 直接取 `lidar_meas_cov`。单点情况下 $\delta\mathbf{x}$ 的修正项为零，这个更新与协方差分支的单点形式代数等价。

所以两版在实车数据上的单点雷达更新是同一个公式，同样的 $\sigma^2 = 0.01$。差别只在两处：当前版把单点残差截断在 $\pm0.5$ 米，原版不截断；点组含多个点时，当前版做顺序标量卡尔曼更新（不重新线性化雅可比，在观测噪声对角 $\sigma^2\mathbf{I}$ 下与原版协方差分支的批量更新精确代数等价）。若点组恰好包含 30 个以上有效点（例如点时间字段没有填、整帧同时间戳），原版会进入信息分支，此时它的等效单点方差变成 $1/M = 100$，远大于当前版的 0.01，两版行为会明显分开。这一点在仿真数据上需要单独确认。

### 5.3 两种分支的数值对照

设先验位置方差 $P^{-} = 0.01$，$H = 1$，单点残差 $0.05$ 米。

| 点组有效点数 | 原版分支 | 原版单次修正占残差 | 当前版单次修正占残差 |
| --- | --- | --- | --- |
| 1 | 协方差 | 50% | 50% |
| 30 | 信息 | 0.3% | 96% |
| 100 | 信息 | 1% | 99% |

前两列按同一组数值算出：协方差分支取 $\sigma^2 = 0.01$，信息分支取 $M = 0.01$ 作为信息量。点组只有一个点时两版一致；点组变大之后原版的修正比例迅速下降。这一条值得在仿真数据上核实点组大小。

## 六、初始化

原版把首批 100 个 IMU 样本的均值当作重力方向，不区分样本是否静止，陀螺零偏不从这批样本写入状态，留给滤波器在线估计。当前版只累加角速度模长小于 0.1 弧度每秒的样本，陀螺零偏取静止样本均值，加速度零偏初值取零。

当前版在 `Imu::append` 里先做六面标定修正：

$$
\mathbf{a}_{true} = \mathbf{M}\left(\mathbf{a}_{meas} - \mathbf{b}_{a,calib}/\text{scale}\right),\qquad
\boldsymbol{\omega}_{true} = \boldsymbol{\omega}_{meas} - \mathbf{b}_{g,calib}
$$

$\mathbf{M}$ 是 3 乘 3 修正矩阵，$\mathbf{b}_{a,calib}$ 与 $\mathbf{b}_{g,calib}$ 是标定零偏，`scale` 是原始单位与米每二次方秒的换算系数。实车配置打开了 `imu_calib_enable`，仿真没有。原版没有这一步，零偏完全靠滤波器在线估计。

重力方向取参数 `mapping.gravity`。原版实车与仿真都取 $(0,0,-9.81)$。当前实车输出变换 `body_to_base_r` 对应安装关系 $R_z(-135^\circ)R_x(-15^\circ)$（倾角 15 度），旧版实车配置误保留了旧安装角的 $(0,1.885,-9.627)$（11.08 度），导致静止初始化先吸收 3.92 度、输出再扣 15 度，`odom` 系整体歪斜 3.92 度。现已将实车 `mapping.gravity` 与 `gravity_init` 同步修正为 $(0, 2.5381, -9.4725)$：倾角严格为 15 度，模长取 $9.80665\text{ m/s}^2$ 与 `scripts/imu_calib` 六面标定标尺 `G_STANDARD` 一致。

当前版从进入跟踪开始的前 20 个点组只传播、不做雷达更新（`FrameProcessor.cpp` 里的 `startup_frames_`）。按点组为单点计算，这对应约 20 个点，时间长度在毫秒量级，可以忽略。

## 七、参数对照表

| 参数 | 原版 mid360 | 当前实车（已同步） | 当前仿真 | 说明 |
| --- | --- | --- | --- | --- |
| `lidar_meas_cov` | 0.01 | 0.01 | 0.1 | 单点更新时两版都按方差使用，一致 |
| `match_s` | 81 | 10 | 10 | 当前版接受更差的点面匹配（待进一步讨论/验证） |
| `acc_cov_output` | 500 | 2.5 | 50 | 原版乘 $\Delta t^2$，当前版乘 $\Delta t$；实车已按 200 Hz 对齐 |
| `gyr_cov_output` | 1000 | 5.0 | 100 | 同上 |
| `imu_meas_acc_cov` | 0.01 | 0.01 | 20 | 实车已对齐原版；仿真保留防坏点值 |
| `imu_meas_omg_cov` | 0.01 | 0.01 | 1.0 | 同上 |
| `b_gyr_cov`、`b_acc_cov` | $10^{-4}$ | $5\times10^{-7}$ | $10^{-4}$ | 实车已按 $\times 0.005$ 对齐原版每步方差 |
| `init_omg_cov`、`init_acc_cov` | 0.01 | 0.01 | 0.01（默认） | 已与过程噪声解耦 |
| `init_b_gyr_cov`、`init_b_acc_cov` | $10^{-3}$ | $10^{-3}$ | $10^{-3}$（默认） | 已与过程噪声解耦 |
| `init_gravity_cov` | $10^{-4}$ | $10^{-4}$ | $10^{-4}$（默认） | 二维切平面常值在线微调（$Q_g=0$） |
| `vel_cov` | 20 | 无此参数 | 无此参数 | 当前版少了速度过程噪声 |
| `plane_thr` | 0.1 | 0.1 | 0.1 | 相同 |
| `ivox_grid_resolution` | 2.0 | 0.5 | 0.5 | 当前版地图更细 |
| `ivox_nearby_type` | 6 | 6 | 18 | 邻域不同 |
| `filter_size_surf`、`filter_size_map` | 0.5、0.5 | 0.5、0.5 | 0.2、0.2 | 仿真降采样更细 |
| `point_filter_num` | 4 | 4 | 8 | 仿真抽稀更多 |
| `init_map_size` | 10 | 10 | 10 | 相同 |
| `gravity` | $(0,0,-9.81)$ | $(0,2.5381,-9.4725)$ | $(0,0,-9.81)$ | 实车对齐 15° 安装角与 $9.80665$ 标尺 |
| `lidar_time_inte` | 0.1 | 0.02 | 0.1 | 只影响切帧回退值 |

## 八、与漂移有关的差异排序

### 8.1 过程噪声时间尺度与初始协方差（实车已解耦对齐）

修改前，当前版实车角速度与加速度过程噪声分别为原版的 10 倍和 40 倍，零偏过程噪声为 200 倍，且把 `gyr_cov_output`/`acc_cov_output` 复用为初始方差（比原版大 5000–10000 倍）。现已在 `ManifoldEskf::initialize` 中拆出独立的 `init_*_cov` 参数，并在实车配置中将 `gyr_cov_output = 5.0`、`acc_cov_output = 2.5`、`b_gyr_cov = b_acc_cov = 5e-7`、`imu_meas_*_cov = 0.01` 按 200 Hz 对齐到原版每步噪声量级。

验证方法：用 `scripts/test/run_point_lio_replay_test.sh` 与 `scripts/test/run_point_lio_replay_z_test.sh` 校验解耦对齐后的零偏收敛与终值。

### 8.2 重力估计与 15 度安装角（已启用二维切平面在线微调并修正倾角）

修改前存在两处独立问题：
1. **外部坐标系失配**：实车 `body_to_base_r` 对应 15 度倾角，而旧 `mapping.gravity` 对应 11.08 度，导致输出 `odom` 系相对真实水平面歪斜 3.92 度（5 米处地面高度偏差约 34 厘米）。现已将实车 `mapping.gravity` 与 `gravity_init` 修正为 $(0, +g\sin 15^\circ, -g\cos 15^\circ) = (0, 2.5381, -9.4725)$（$g = 9.80665\text{ m/s}^2$）。
2. **内部重力冻结**：此前 $\mathbf{P}_g = \mathbf{0}$ 冻结重力，只要静止初始化因残余零偏带入微小方向误差 $\delta\mathbf{g}$，车体转向后机体系零偏 $\mathbf{b}_a$ 无法抵消世界系常值误差 $\delta\mathbf{g}$。现已恢复 $\partial\dot{\mathbf{v}}/\partial\mathbf{g} = \Delta t\mathbf{I}$ 并启用二维切平面常值在线微调（$\mathbf{P}_{g,0} = 10^{-4}(\mathbf{I}-\hat{\mathbf{g}}_0\hat{\mathbf{g}}_0^{\mathsf{T}})$，$\mathbf{Q}_g = \mathbf{0}$），既允许转弯后修正初始重力方向误差，又不给 $\dot{\mathbf{g}}$ 加过程噪声，且锁定径向零空间避免与 $b_{a,z}$ 耦合发散。

验证方法：实车上电静止初始化后在水平场地旋转与往返行驶，检查 `odom` 点云地面是否水平以及 `/point_lio/debug_state` 中 `g` 与 `ba` 的收敛情况。

### 8.3 点面匹配阈值放宽三倍

`match_s` 从 81 降到 10 后，10 米远处的点允许 1.0 米的点面距离进入更新，原版只允许 0.35 米。单点更新下 $\sigma^2 = 0.01$、先验位置方差 0.01 时，单点增益约 0.5，一个错误匹配点可以把位置按它残差的一半拉动。当前版虽然把残差截断在 0.5 米，截断后的错误点仍然有 0.25 米的拉动能力。这一条是我最担心的，因为它同时出现在实车与仿真配置里。

验证方法：把 `match_s` 改回 81，其余不动，用同一条回放包比较漂移与位姿抖动。

### 8.4 六面标定与零偏初值

当前版把零偏从在线估计改成离线标定加初值。如果标定值准确，它比原版的在线估计更快、更稳。如果标定值有随温度变化的漂移，当前版的零偏状态增益小，补不回来。这一条只在长时间运行或温度变化明显的场景里才会显现。

验证方法：连续跑 20 分钟以上，看 `/point_lio/debug_state` 里 `bg`、`ba` 是否随温度变化，以及漂移是否随时间加速。

### 8.5 残差截断、启动窗口与地图参数

残差截断只在初始误差或地图错位超过 0.5 米时起作用。启动窗口按点组为单点计算只有约 20 个点，影响很小。地图参数（iVox 0.5 米对 2.0 米、仿真体素 0.2 米）改变地图密度与匹配质量，对漂移的影响小于前三项，但仿真与实车不能用同一套结论。

## 九、验证方案

1. 先用 `scripts/test/run_point_lio_replay_test.sh` 与 `scripts/test/run_point_lio_replay_z_test.sh` 记录基线，重点记 `bg`、`ba` 的收敛时间与终值。
2. 单独改 `match_s` 回 81，用同一条真实回放包比较漂移指标（累计位置误差除以路程，见 `docs/评价基线.md`）。
3. 单独改 `gyr_cov_output`、`acc_cov_output` 与初始协方差的写法，比较零偏收敛时间。
4. 核算实车安装俯仰，必要时修正 `mapping.gravity` 与 `body_to_base_r`。
5. 在仿真数据上确认点组大小，判断原版是否进入信息分支。

## 十、检查题

1. 若把点云按时间升序排序后再调用 `time_compressing`，为什么点组会退化为单点，这与「按时间戳分组」的注释是否矛盾？
2. 当前版把角速度初始方差写成 `gyr_cov_output`，为什么单看角速度与零偏这一对两状态块会得出「零偏增益变大」的结论，而整条协方差链的结论可能相反？
3. `match_s` 取 10 时，10 米远处允许的最大点面距离是多少，它和单点增益 0.5 一起会把一个错误匹配点放大成多大的位置拉动？
