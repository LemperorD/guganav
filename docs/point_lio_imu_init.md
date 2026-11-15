# Point-LIO IMU 初始化设计记录

本文记录 IMU 静止初始化的讨论结论、设计依据和待定问题。代码由项目作者编写，本文只记录方案与决策。

建立于 2026 年 11 月 15 日。

## 1. 范围与前提

- 本次只重写 IMU 初始化。地图引导、先验地图初始位姿、滤波器量测方程不在本次范围。
- 起跑条件按保证静止启动设计。等不到静止段时超时回退，节点继续运行。
- 初始化期间到达的雷达帧直接丢弃并计数，不缓存。
- 初始化结果由 `FrameProcessor` 应用到 `ManifoldEskf`，初始化类本身不依赖滤波器和 ROS 消息类型。

## 2. 初始化在解什么

### 2.1 对齐方程

初始化解一个对齐问题：给定地图系重力向量 $g$，求初始姿态 $R_0$，使

$$R_0 \bar f + g = 0$$

在初始时刻成立。其中 $\bar f$ 是静止样本的比力均值，单位 m/s²。这个等式等价于让初始时刻滤波器预测的加速度与加速度计测到的比力一致。

### 2.2 方程怎么解

静止时第 $k$ 个样本的比力满足

$$f_k = -R_0^{\mathsf T} g + b_a + n_k,$$

其中 $f_k$ 是比力，即加速度计的输出，单位 m/s²，静止时比力等于负的体坐标系重力加速度；$R_0$ 是体坐标系到地图系的旋转；$g$ 是地图系重力向量，单位 m/s²；$b_a$ 是加速度零偏，单位 m/s²；$n_k$ 是测量噪声，单位 m/s²。

取静止样本均值，忽略噪声和零偏，得到 $\bar f \approx -R_0^{\mathsf T} g$。转成方向：

$$u_b = -\frac{\bar f}{\lVert \bar f \rVert}, \qquad u_w = \frac{g}{\lVert g \rVert}, \qquad R_0 u_b = u_w.$$

$u_b$ 是体坐标系里的重力方向，$u_w$ 是地图系里的重力方向，两个都是单位向量。比力方向与重力方向相反，负号来自这里。

$R_0 u_b = u_w$ 只有两个独立约束，剩下一个自由度是绕 $u_w$ 的偏航。最小旋转把旋转轴取成 $u_b \times u_w$，偏航因此为零。

实车配置的数值：$g = (0, 1.885, -9.627)$，$\lVert g \rVert = 9.8098$，$u_w = (0, 0.1922, -0.9814)$。机器人处在标称姿态时 $\bar f = (0, -1.885, 9.627)$，$u_b$ 与 $u_w$ 相同，$R_0$ 取单位阵。

### 2.3 初始化输出

| 输出量    | 来源                      | 说明                                             |
| --------- | ------------------------- | ------------------------------------------------ |
| $R_0$     | 由 $\bar f$ 与 $g$ 对齐求出 | 两个自由度来自重力方向，偏航按最小旋转取零       |
| $g$       | `mapping.gravity`         | 固定常量，不估计                                 |
| $acc_0$   | $\bar f$                   | 让初始 IMU 残差为零                              |
| $v_0$     | 零                        | 静止判据无法区分静止与匀速平移                   |
| $p_0$     | 零                        | 定义地图原点                                     |
| $\omega_0$ | 零                       | 第一次 IMU 更新拉到测量值                        |
| $bg$      | $\bar\omega$               | 当前滤波器中不参与更新，只影响调试显示           |
| $ba$      | 零                        | 协方差置零，冻结                                 |
| $P$       | 静止段统计量              | 姿态、位置、速度、零偏四块                       |

### 2.4 重力大小不作为估计量

加速度计测比力，静止时 $\lVert \bar f \rVert = s \lVert g \rVert$，$s$ 是加速度计刻度误差，无量纲。用同一路加速度计定重力大小，得到的是 $s \lVert g \rVert$，刻度误差被并入重力。

数值：刻度误差 1% 时测得 $\lVert \bar f \rVert = 9.908$ m/s²，与配置的 9.8098 差 0.098，小于静止判据的 0.3，一致性检查会通过。滤波器随后按 1.01 倍积分加速度，真实的 0.5 m/s² 加速度在 10 秒里多积出 0.25 米位置。雷达会把位置拉回来一部分，速度估计会带上偏差。

另一个后果是每次启动估出的 $g$ 会随零偏、温度和噪声变化，世界系的重力定义每次不同，跨会话一致性没有保证。

测得的大小只用于两件事：检查 `acc_norm` 的单位换算，检查静止判据里的模长偏差。

### 2.5 地图 z 轴与竖直方向的关系

地图系是 `camera_init`，地图系里的重力向量由 `mapping.gravity` 给出，地图 z 轴不保证竖直。

- 实车配置 `gravity = (0, 1.885, -9.627)`，与竖直方向差 11.08 度。
- 仿真配置 `gravity = (0, 1.703, -9.66)`，与竖直方向差 10.0 度。
- 把重力配成 $(0, 0, -9.81)$，地图 z 轴才会竖直，此时静态初始化把姿态对齐到水平。

## 3. 现状与问题

### 3.1 现在的流程

1. `FrameProcessor::initializeIteration` 调用 `Synchronizer::syncPackages` 取一帧，随后 `initScan` 设置重力并丢弃雷达开始之前的 IMU 样本。
2. `prepareFrame` 调用 `Imu::process`。`Imu` 在 `Stage::Initializing` 下执行 `IMU_init`，把所有样本计入 `init_iter_num_`，只把角速度模长小于 0.1 rad/s 的样本计入均值。
3. 样本数到 100 后 `initState` 写入姿态、加速度、零偏，阶段切到 `INITIALIZINGMAP`。
4. `initMapState` 累积世界系点云，达到 `init_map_size = 10` 后插入 iVox，阶段切到 `TRACKING`。

### 3.2 问题清单

- 完成判据是 100 个样本，与静止时长无关。200 Hz 下是 0.5 秒，100 Hz 下是 1 秒，启动时在运动也能满足。
- `static_count_` 为 0 时，`initState` 静默取单位姿态，零偏取零，之后不再重试，没有告警。
- 初始化只在雷达帧到达时推进。`syncPackages` 已经把帧从 `frame_buffer_` 弹出，`prepareFrame` 返回 false 后这一帧丢失，丢失数量没有记录。
- 第一帧的 `imu_measure` 会带上雷达开始之前的样本，因为 `discardBefore` 在 `collectUntil` 之后才调用。
- `ManifoldEskf::initialize` 的初始协方差是写死的 0.01 对角块，与静止段统计量无关。
- `mapping.vel_cov`、`prior_pcd.init_pose`、`mapping.gravity_init` 从 YAML 读入后没有使用。`mapping.imu_en` 在 YAML 里存在，C++ 没有读取。
- `bg` 和 `ba` 不进入量测方程。`applyImuUpdate` 的残差只对 omega 与 acc 求更新，`H_I` 没有零偏列，`ba` 的协方差还被置零。初始化估出的 `bg` 只影响调试显示。
- IMU 话题没有数据时 `syncPackages` 一直返回 false，500 Hz 定时器空转，没有超时也没有警告。
- `init_map_size = 10` 在仿真和实车配置里都小于一帧点数，等于第一帧就建图。建图用当时还在变化的状态把点转到世界系。
- `startup_frames_ < 20` 按时间组计数，与时间不对应，注释里的“等 acc 收敛”没有可核对的判据。

最后两条属于地图引导，记录在案，不在本次改动范围。

## 4. 方案

### 4.1 静止判据

逐样本判据，同时满足才算静止样本：

| 参数           | 默认值 | 单位  | 含义                         |
| -------------- | ------ | ----- | ---------------------------- |
| $r_\omega$     | 0.05   | rad/s | 角速度模长上限               |
| $r_a$          | 0.3    | m/s²  | 加速度模长与 $g$ 之差的上限  |

窗口判据，同时满足才算完成：

| 参数       | 默认值    | 单位 | 含义                   |
| ---------- | --------- | ---- | ---------------------- |
| $T$        | 1.0       | s    | 窗口时长               |
| 跨度下限   | 0.95 $T$  | s    | 排除窗口未填满         |
| 静止比例   | 0.9       | 无   | 容忍偶发振动与丢包     |
| 最少样本   | 30        | 个   | 排除低频 IMU           |

新增一条窗口层面的方向判据，待定：窗口内每个静止样本相对 $\bar f$ 的夹角 $\theta_k$ 的均方根不大于 $r_\theta$，默认 1 度。加速度模长判据对水平加速度几乎无效：水平加速度 0.3 m/s² 只让模长增加 0.0046 m/s²，方向却偏 1.75 度。方向判据同时是姿态协方差的输入。

### 4.2 估计

用静止样本均值 $\bar f$ 和 $\bar\omega$，按 2.2 节求 $R_0$，并令 $acc_0 = \bar f$、$\omega_0 = 0$、$bg = \bar\omega$、$ba = 0$、$v_0 = p_0 = 0$。

### 4.3 初始协方差

| 块         | 取值                                         | 依据                                       |
| ---------- | -------------------------------------------- | ------------------------------------------ |
| 姿态       | $\max(\mathrm{RMS}(\theta_k)/\sqrt{\lvert S\rvert},\ 0.5^\circ)$ | 主要误差来自加速度零偏，0.05 m/s² 对应 0.29 度 |
| 位置       | 0.05 m                                       | 初始时刻位置按定义是原点，给小量避免奇异   |
| 速度       | 0.1 m/s                                      | 静止判据无法排除匀速平移                   |
| 陀螺零偏   | $\max(\mathrm{std}(\omega)/\sqrt{\lvert S\rvert},\ 0.005\ \mathrm{rad/s})$ | 下限对应零偏的温度漂移与随机游走 |
| 加速度零偏 | 0                                            | 与滤波器一致，冻结                         |
| 重力       | 0                                            | 与滤波器一致，固定                         |
| omega、acc | `gyr_cov_output`、`acc_cov_output`           | 保留现有大值，让第一次 IMU 更新拉到测量值  |

### 4.4 完成、超时与回退

- 判据满足时状态置 `Ready`，记录完成时刻和统计量，之后不再累积。
- 从第一个样本的单调时钟时刻算起超过 `timeout`（默认 10 秒）仍未满足，状态置 `Fallback`。用单调时钟而不是样本时间，IMU 断流时也能触发。
- 回退结果：姿态取单位阵，`bg` 取窗口角速度均值，`ba` 取零，姿态标准差放大到 5 度，速度标准差放大到 0.3 m/s。重力取哪个量待定，见问题 Q2。
- 回退时写 ERROR 日志，内容包括窗口样本数、静止比例、角速度均值、加速度模长均值。

### 4.5 接口

```cpp
struct ImuInitParams {
  double static_duration{1.0};
  double static_ratio{0.9};
  int min_static_samples{30};
  double gyro_threshold{0.05};
  double acc_threshold{0.3};
  double angle_threshold{1.0 * M_PI / 180.0};
  double timeout{10.0};
  double velocity_sigma{0.1};
  double position_sigma{0.05};
  double attitude_sigma_floor{0.5 * M_PI / 180.0};
  double bias_sigma_floor{0.005};
};

struct ImuInitResult {
  Eigen::Matrix3d rot{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gravity{0.0, 0.0, -9.81};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  double attitude_sigma{0.0};
  double bg_sigma{0.0};
  double velocity_sigma{0.1};
  double position_sigma{0.05};
  double completion_time{0.0};
  int static_samples{0};
  int window_samples{0};
  double static_span{0.0};
  bool used_fallback{false};
};

class ImuInitializer {
public:
  enum class Status { Collecting, Ready, Fallback };
  void configure(const ImuInitParams& params,
                 const Eigen::Vector3d& gravity_world,
                 const Eigen::Vector3d& gravity_init);
  void observe(const Eigen::Vector3d& omega,
               const Eigen::Vector3d& acc_scaled,
               double timestamp);
  Status poll();
  Status status() const;
  const ImuInitResult& result() const;
  void reset();
};
```

加速度由调用方先乘 `gravity_magnitude / acc_norm` 换算到 m/s² 再送入。

### 4.6 接入点

1. `Imu` 只保留缓冲与时间有序，删除 `Stage`、`IMU_init`、`initState`、`Set_init_z` 和几个统计成员。`append` 改为返回 `bool`，拒绝乱序样本时告诉调用方。
2. `FrameProcessor::onImu` 先把时差加到时间戳上，再分别送入 `Imu` 与 `ImuInitializer`。初始化节奏与雷达帧无关。
3. `FrameProcessor::onLidarFrame` 与 `onLidarFrames` 在初始化未应用时只累加丢弃计数，不进入 `Lidar` 的帧缓冲。
4. `FrameProcessor::initializeIteration` 在初始化未完成时不调用 `syncPackages`，完成后调用 `ManifoldEskf::applyInitialization` 写入 `x_` 与 `P_`，再切到建图阶段。
5. `Synchronizer::syncPackages` 在取到帧之后、`collectUntil` 之前调用 `imu.discardBefore(frame.start_time)`，修掉第一帧带上雷达开始之前样本的问题。
6. `ManifoldEskf` 新增 `applyInitialization(const ImuInitResult&)`，重置 `x_` 与 `P_`。

## 5. 已定事项

- [x] 初始化期间到达的雷达帧直接丢弃并计数，不缓存。
- [x] 起跑条件按保证静止启动设计。
- [x] 等不到静止段时超时回退，节点继续运行。
- [x] 代码由项目作者编写，本文只记录方案。

## 6. 待讨论问题

- [ ] Q1 静止阈值用默认值还是实测值。建议先录 60 秒静止 IMU，统计 $\lVert \omega_k \rVert$ 与 $\lvert \lVert a_k \rVert - g \rvert$ 的 99.99 分位，取 2 到 3 倍，并用同一段数据回放初始化器。
- [ ] Q2 超时回退时重力用哪个量。三种取法的初始加速度残差如下：取标称重力配单位阵为 $2g\sin(2.5^\circ) = 0.855$ m/s²（对应 5 度倾斜）；取窗口均值反推，初始残差约为 $\lvert \lVert \bar a \rVert - g \rvert$，机器人运动时窗口均值会被运动加速度污染；取实际姿态需要先知道姿态，回退场景下没有这个信息。
- [ ] Q3 是否加入方向残差判据 $r_\theta$。
- [ ] Q4 参数命名用 `init.*` 还是 `imu_init.*`，以及写进哪几份 YAML。
- [ ] Q5 是否需要单元测试或离线回放工具。
- [ ] Q6 `bg` 与 `ba` 是否保留在初始化输出里。当前滤波器不使用它们。
- [ ] Q7 初始化完成后的第一帧，允许它的起始时刻略早于完成时刻，还是严格丢弃。丢弃的代价是多等 0.1 秒。

## 7. 验证计划

- 合成数据：静止 2 秒加噪声，初始化器应在 1 秒内判 `Ready`，姿态误差小于 0.2 度。
- 合成数据：以 0.2 rad/s 匀速旋转，初始化器应拒绝并触发 `Fallback`。
- 实车或仿真静止数据回放：检查 1 秒判完成，日志里的窗口样本数与静止比例合理。
- 仿真原地旋转 1 分钟：yaw 跟随 1.9 rad/s，位置漂移小于 0.1 米，与 `docs/point_lio_debug_plan.md` 的验收标准一致。

## 8. 变更记录

- 2026-11-15：建立文件，记录初始化定义、现状问题、方案与待定事项。
