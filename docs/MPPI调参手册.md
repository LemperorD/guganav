# MPPI 调参手册（ROS 2 Humble 适用）

本文面向本项目（ROS 2 Humble + `nav2_mppi_controller` 1.1.20，插件名 `guga_source_mppi_controller::MPPIController`）的 MPPI 调参工作，
先整理官方与社区资料中可复用的调参方法，再对照本项目 `config/simulation/controller/mppi.yaml` 与 `config/reality/controller/mppi.yaml` 的现状给出待处理清单。

阅读顺序建议：第 1 节（机制）→ 第 3 节（调参顺序）→ 第 4 节（定量关系）→ 第 6 节（本项目体检）。

更细的参数含义、逐症状的调整数值起点、以及按来源分级的出处，见 [MPPI调参_资料汇编.md](MPPI调参_资料汇编.md)（外部资料整理，区分官方/源码/社区/未验证四类来源，并附未核实清单）；本文只保留与本项目直接相关的结论与判断。

---

## 1. MPPI 在本项目中的运行机制

### 1.1 每个控制周期的五个步骤

1. 以上一周期的最优控制序列为先验，叠加高斯噪声生成 `batch_size` 条候选控制序列；
2. 用运动模型（本项目为 `Omni`）对每条控制序列前向积分，得到 `time_steps` 个轨迹点；
3. 每个已加载的 critic 对每条候选轨迹累加代价，得到长度 `batch_size` 的代价向量；
4. 代价向量减去最小值后做 softmax 加权（见 1.2），得到新的最优控制序列；
5. 取该序列的第一个控制量执行，序列整体前移一位，进入下一周期。

对应源码：`src/guga_controller/nav2_mppi_controller/src/optimizer.cpp`（`updateControlSequence()`）。

### 1.2 代价如何变成控制量

关键代码（`optimizer.cpp`，Humble 1.1.20）：

```cpp
// 控制努力项：gamma / std^2 * Σ u · (u_noised - u)
costs_ += gamma / powf(sampling_std.vx, 2) * xt::sum(control_sequence_.vx * bounded_noises_vx, 1);
costs_ += gamma / powf(sampling_std.wz, 2) * xt::sum(control_sequence_.wz * bounded_noises_wz, 1);

// 归一化后做 softmax：权重 = exp(-Δcost / temperature) / Σ exp(...)
auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
```

由此得到三条对调参直接有用的结论：

- `temperature` 是 softmax 的温度，作用对象是**归一化之后的代价差**。`temperature → 0` 时结果趋近代价最低的单条轨迹，`temperature` 很大时结果趋近所有候选的均值。它的实际效果取决于代价分布的宽度，因此**修改任何 critic 的权重量级之后，`temperature` 的等效选择性会跟着改变**。
- `gamma` 项被除以 `std²`。把 `vx_std` 从默认 0.2 加到 0.5（本项目当前取值），同一 `gamma` 的控制努力惩罚被削弱到 1/6.25。两者不能各自独立地调。
- 代价先减去最小值再取指数，所以 softmax 只看候选之间的相对差异；把某个 critic 的权重整体放大并不会提高它相对其他 critic 的话语权，只有改变**代价分布的相对形状**才会。

### 1.3 本项目特有的两处约定

- **底盘常转（小陀螺）**：MPPI 输出的速度表达在 `base_footprint_nonrotating` 系，yaw 不参与控制。`controller:=mppi` 启动时 `init_spin_speed=6.28`（`launch/core/navigation_launch.py`）。这解释了两份配置里 `GoalAngleCritic` 关闭、`reality` 中 `wz_max: 0.0` 的原因。
- **速度最终由 velocity_smoother 限幅**：`velocity_smoother` 的 `max_velocity: [2.5, 2.5, 3.0]`（`config/simulation/base.yaml`）、`max_accel: [2.0, 2.0, 2.5]`、`max_decel: [-3.0, -3.0, -4.0]`。MPPI 输出的速度指令会再经过这一层加速度整形，因此"起步慢""加速慢"一类现象未必是 MPPI 的问题。

---

## 2. 参数按杠杆大小分级

调参时先分清一个参数属于哪一级，可以避免在三级参数上反复试探而真正受限的一级参数没动。

| 级别 | 作用 | 参数 |
| --- | --- | --- |
| 一级：决定可行域 | 机器人物理上能做什么 | `motion_model`、`vx_max`/`vx_min`/`vy_max`/`wz_max`、`controller_frequency` 与 `model_dt`、`batch_size`、`time_steps` |
| 二级：决定行为偏好 | 在所有可行轨迹里更偏好哪些 | `critics` 列表、各 critic 的 `cost_weight`、`threshold_to_consider`、`offset_from_furthest`、`critical_cost`/`collision_cost` |
| 三级：决定探索与选择性 | 采样范围与对代价差异的敏感度 | `vx_std`/`vy_std`/`wz_std`、`temperature`、`gamma` |
| 四级：工程与调试 | 路径处理、可视化、容错 | `prune_distance`、`transform_tolerance`、`max_robot_pose_search_dist`、`enforce_path_inversion` 及其容差、`visualize`、`regenerate_noises`、`retry_attempt_limit`、`reset_period`（Humble 专有） |

官方对 `iteration_count` 的说明是"保持 1，宁可增加 `batch_size`"，本项目两份配置均为 1，与建议一致。

### 2.1 版本边界：Humble 与新版（Rolling/Jazzy）的差异

照抄新版教程到 Humble 会得到"参数写了但不生效"的结果，这一节列出已核实的差异。判定依据是本地 1.1.20 源码（`grep` 结果）与新版官方文档页。

| 参数 | Humble 1.1.20 | 新版（Rolling 文档中） |
| --- | --- | --- |
| `ax_min` / `ax_max` / `ay_min` / `ay_max` / `az_max` | **不存在**。本地源码 `src/`、`include/` 中均无这些标识符 | 存在，默认 3.0 / 3.0 / −3.0 / 3.0 / 3.5，用于在原始控制量上施加加速度限幅 |
| `clamp_raw_controls` | 不存在 | 存在，默认 false，配合上面的加速度限幅使用 |
| `open_loop` | 不存在 | 存在，默认 false |
| `sgf_order` | 不存在（Savitzky-Golay 滤波在 `optimizer.cpp` 中已存在，但阶数不可配） | 存在，默认 2 |
| `publish_optimal_trajectory` / `publish_critics_stats` / `critic_index_to_visualize` | 不存在 | 存在 |
| `TrajectoryValidator.plugin` | 不存在 | 存在 |
| `allow_parameter_qos_overrides` | 不存在 | 存在 |
| `reset_period` | **Humble 专有**（README 注明 only in Humble） | 已移除 |
| `ObstaclesCritic.inflation_radius` / `cost_scaling_factor` | **Humble 专有**，必须与局部代价地图膨胀层一致 | 已换成 `inflation_layer_name`，直接从指定膨胀层读取参数 |
| `PathAlignLegacyCritic` | 存在（2023 年 10 月前的旧公式） | 现行文档中不再列出 |
| `PathAngleCritic` 的行为选择参数 | `forward_preference`（bool）；`vx_min >= 0`（不允许倒车）时被强制为 true | 改为 `mode`（int，0/1/2）。本项目 `reality` 配置里写的 `mode: 0` 在本版本不存在 |
| `PathAngleCritic` 默认值 | `cost_weight` 2.0、`offset_from_furthest` 4、`max_angle_to_furthest` 1.2 | `cost_weight` 2.2、`offset_from_furthest` 20、`max_angle_to_furthest` 0.785398 |
| `ObstaclesCritic.collision_cost` 默认值 | 10000.0 | 100000.0 |
| 可视化话题 | `/trajectories`（`visualization_msgs/MarkerArray`）与 `transformed_global_plan` | 另有按代价着色的可视化与 `~/critics_stats` |

由于 Humble 缺少原生的加速度限幅参数，本项目实车的加速度约束全部落在这两处：`velocity_smoother` 的 `max_accel`/`max_decel`/`max_velocity`，以及 `ConstraintCritic`（速度上限约束）与采样标准差（间接限制单步速度变化）。

---

## 3. 官方给出的调参顺序

以下顺序来自官方 README 的 "Notes to Users" 一节（[Humble 分支 README](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)），维护者 Steve Macenski 在 issue 中反复指向这一节作为调参依据（见 [#5375](https://github.com/ros-navigation/navigation2/issues/5375)、[#4849](https://github.com/ros-navigation/navigation2/issues/4849)）。

1. **先定速度 profile 与运动模型**。`vx_max`/`vx_min`/`vy_max`/`wz_max` 与 `motion_model` 必须与真实底盘一致。这一步没定，后面所有 critic 权重都没有意义。
2. **让预测视界、代价地图尺寸、offset 三者自洽**。预测视界（`time_steps × model_dt`）乘以最大速度得到的距离，必须小于代价地图半径；否则最大速度被人为限制。`PathFollowCritic` 与 `PathAlignCritic` 的 `offset_from_furthest` 也受同一约束（详见第 4 节）。
3. **先调障碍 critic，再调路径跟随，最后调贴线**。原文的顺序是：
   - 先让障碍 critic 与膨胀层参数配合出期望的避障行为；`repulsion_weight` 与膨胀半径成反比（惩罚形式为 `inflation_radius - min_dist_to_obstacle`），膨胀半径越大，权重应越小；权重过大会表现为"从自由空间进入代价空间时显著减速"或"窄走廊里抖动"。
   - 然后调 `PathFollowCritic` 的权重，让机器人愿意顶着避障代价继续前进。权重不足时机器人会拒绝进入任何有代价的区域（宁可在 0 代价区停留）。
   - 最后调 `PathAlignCritic`。过度加权会剥夺绕开动态障碍的能力（表现为减速甚至停下），官方建议保持"能大致走在中线、必要时可偏离"的平衡。
   - 官方补充：如果路径由代价感知的规划器生成（本项目的 Smac/JPS 都是），可以适当降低障碍 critic、让贴线 critic 起作用。
4. **最后微调采样标准差**。新版文档里有一句 Humble README 没有的补充："角速度抖动明显就减小对应 std；机器人达不到最大速度就增大 std 以扩大搜索范围"。同时要注意里程计发布频率不应低于控制频率，否则加速度限幅用不满。
5. **`temperature` 与 `gamma` 一般不动**。官方对 `gamma` 的说明是"这个参数很复杂，默认值在很宽的范围内都工作良好"。二者只有在完成 1–4 步之后仍不满意时才动。

---

## 4. 调参时必须先算清楚的定量关系

这些关系是官方文档中给得最明确、也最容易被忽略的部分。

### 4.1 `model_dt` 与 `controller_frequency`

官方原话："`model_dt` 一般应设成控制周期的长度，例如控制频率 20 Hz 时取 0.05；也可以设得更小，但不能更大。"

本项目 1.1.20 源码对失配的处理（`controller.cpp` 的 configure 阶段）：控制周期小于 `model_dt` 时只告警并关闭控制序列移位；控制周期大于 `model_dt` 时直接抛异常，`controller_server` 起不来；两者相差小于 1e-6 时正常工作并开启序列移位。两份 yaml 里的注释与源码行为一致。

### 4.2 预测视界 × 最大速度 vs 代价地图半径

预测视界 `T = time_steps × model_dt`。官方给的下界判据是：

```
T × v_max ≤ 代价地图半径
```

不满足时，官方说明是"机器人的最大速度与行为会被代价地图尺寸人为限制"。

### 4.3 `offset_from_furthest` 的基线公式

官方给的经验公式：

```
offset_from_furthest ≈ T × v_max / path_resolution / 3
```

说明同时指出：设得过小（如 5）会在开始跟踪路径时误触发，产生局部极小；设得过大（如 50）则相对路径分辨率与代价地图尺寸而言可能永远不触发，或只在满速时触发。

该参数在 `PathAlignCritic`、`PathFollowCritic`、`PathAngleCritic` 中的含义都是"从本批次轨迹达到的最远路径点再往前取若干个路径点"，但取到点之后各自的计算不同（维护者在 [#4376](https://github.com/ros-navigation/navigation2/issues/4376) 中明确回答过这一点）。

### 4.4 `threshold_to_consider` 与预测视界的交接

官方建议把 `PathFollowCritic` 与 `GoalCritic` 的 `threshold_to_consider` 设成与预测视界对应的距离，让两者在交接处平滑切换。若不这样做，路径跟随 critic 在接近目标时会先减速（因为它的参考点在目标处）。

这里有一个容易搞反的细节：两类 critic 的阈值方向**相反**（差别来自 `utils::withinPositionGoalTolerance` 在各自 `score()` 里的用法）：

- 目标类（`GoalCritic`、`GoalAngleCritic`）：距离目标**小于**阈值时才计分（`if (!within...Tolerance) return;`）。
- 路径类（`PathAlignCritic`、`PathFollowCritic`、`PathAngleCritic`、`PreferForwardCritic`）：距离目标**小于**阈值时**停止**计分（`if (within...Tolerance) return;`）。

所以"把两者设成同一个值"的实际含义是让交接点重合，而不是让两者同时生效。本项目 simulation 设为 `GoalCritic 2.0` 与 `PathFollowCritic 1.4`，结果是 1.4~2.0 m 区间两者同时计分，1.4 m 以内只剩目标引力（贴线 critic 的阈值 0.2 也已关闭）。

### 4.5 膨胀半径与 `repulsion_weight` 的反比关系

`ObstaclesCritic` 把代价地图的代价换算成"到障碍的距离"，再按 `inflation_radius - min_dist_to_obstacle` 的形式给分。
因此修改膨胀层参数之后，`repulsion_weight` 需要重新标定：膨胀半径变大而权重不降，就会在进入代价空间时突然大幅减速。

Humble 的 `ObstaclesCritic` 自身还带有 `inflation_radius` 与 `cost_scaling_factor` 两个参数，官方要求它们**与局部代价地图膨胀层的取值一致**，否则距离换算本身就错。

### 4.6 `batch_size` 与频率的配对

ROSCon 2023 官方演讲（[On Use of Nav2 MPPI Controller](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf)）给出的已调优组合是：30 Hz 配 2000 条采样，50 Hz 配 1000 条采样。同一演讲强调"代价地图的平滑膨胀非常关键"。

### 4.7 采样标准差的两种相反指引

官方给出的方向是：机器人达不到最大速度就增大 `vx_std`，以扩大速度空间的搜索范围（这条在新版文档中由维护者补入）。
但 Open Navigation 在高惯性工业车辆上的实测给出相反方向：低加速平台应当**减小** std（例如从 0.2 降到 0.1），判据是 σ ≈ `a_max × model_dt`，理由是过大的采样散布会让优化器反复选中超出底盘真实加速度能力的候选轨迹，反而压低实际速度（[原文](https://opennav.org/news/mppi-low-acceleration/)）。

两种说法并不矛盾，区别在平台：采样散布应当覆盖底盘真实能实现的动态范围。判断方法是把 `visualize` 打开，用 `/trajectories` 看候选轨迹的**上沿速度**——上沿明显低于 `vx_max` 说明探索不足（应增大），远高于底盘实际加速能力说明散布过大（应减小）。本项目的 `vx_std`/`vy_std` 取 0.5、`wz_std` 取 1.0，都是官方默认值的 2.5 倍，属于"大散布"一侧，是否合适要按这个判据实测确认。

---

## 5. 症状与参数的对应

| 症状 | 首先怀疑 | 调整方向 |
| --- | --- | --- |
| 速度上不去，达不到 `vx_max` | 预测视界超出代价地图半径；`PathFollowCritic` 权重不足；velocity_smoother 限幅 | 按 4.2 检查；提高 `PathFollowCritic.cost_weight`；检查 smoother 的 `max_velocity`/`max_accel` |
| 角速度或横向速度抖动（chatter） | 对应维度的 `*_std` 偏大；`temperature` 过小；障碍 critic 权重大于膨胀半径所能支撑 | 减小对应 `*_std`；检查 `repulsion_weight` 与膨胀半径是否匹配 |
| 画龙 / 蛇行 | `PathAlignCritic` 权重不足而障碍 critic 权重过大；障碍 critic 在自由与代价边界反复取舍 | 按第 3 节第 3 步重新平衡两者 |
| 拒绝进入任何低代价区域（宁可在 0 代价区停留） | 障碍 critic 权重相对 `PathFollowCritic` 过高 | 提高 `PathFollowCritic.cost_weight`，或降低障碍 critic 权重 |
| 窄走廊里减速或抖动 | `repulsion_weight` 与膨胀半径不匹配 | 见 4.5 |
| 到目标附近转圈 / 不收敛 | `GoalCritic` 与 `PathFollowCritic` 的 `threshold_to_consider` 交接不良；`near_goal_distance` 设置 | 按 4.4 对齐阈值；检查 `GoalCritic.threshold_to_consider` |
| 开始跟踪路径时出现怪动作或局部极小 | `PathAlignCritic.offset_from_furthest` 过小 | 按 4.3 提高 |
| 贴不住路径 | `PathAlignCritic.cost_weight` 偏低；`trajectory_point_step` 过大 | 提高权重，或减小步长（1–10 为合理区间） |
| 绕不动动态障碍 | `PathAlignCritic` 权重过高；`max_path_occupancy_ratio` 未生效 | 降低贴线权重 |
| 全向底盘只肯走直线、不肯横移 | 任一节点的 `min_y_velocity_threshold` 过大；velocity_smoother 的 `max_velocity[1]=0` | 见 issue [#5376](https://github.com/ros-navigation/navigation2/issues/5376) 的结论：该提问者最终是 velocity_smoother 的默认横向速度上限导致，改掉后才正常 |
| 不想让机器人转向但仍出现转向噪声 | `wz_std` 未随 `wz_max = 0` 调小 | `wz_std` 保持非零即可（Humble 把它放在 `gamma / std²` 的分母上，设为 0 会产生 `NaN`），要调小取 0.01 这类非零值。社区常见的"想让机器人不转就把 `wz_std` 设 0"在 Humble 上不成立 |

---

## 6. 本项目两份 MPPI 配置的体检

对照物：官方 Humble README 的示例配置、第 4 节的定量关系、以及本项目 1.1.20 源码的实际行为。

### 6.0 本轮已完成的改动（6.1、6.2 记录的是改动前的排查结论）

6.3 与 6.4 已按改动后的配置更新；6.1、6.2 中凡被本节覆盖的项，以本节为准。

已完成：

1. `robot_radius` 四处统一为 0.369（`simulation/controller/mppi.yaml`、`simulation/base.yaml`、`reality/base.yaml` 的 local 与 global 各一处）。仿真是外扩（0.35 → 0.369），实车是收窄（0.374 → 0.369）。
2. 实车 MPPI 的采样与频率参数与仿真对齐：`controller_frequency` 60 → 30、`model_dt` 0.1 → 0.033333333、`time_steps` 48 → 56、`batch_size` 1200 → 1800、`vx_std`/`vy_std` 0.25 → 0.5。改动后实车预测视界 1.867 s、视界内前向距离 4.67 m，与仿真一致，并满足代价地图半径 6 m 的判据。
3. 实车 `CostCritic` 照搬仿真值：`cost_power` 10 → 1、`cost_weight` 14 → 4。
4. 实车删除 `ax_max` / `ay_max`（Humble 不声明这两个参数）。
5. 实车 `ObstaclesCritic` 取值与仿真对齐：`repulsion_weight` 1.0 → 2.0、`inflation_radius` 0.55 → 0.40。该 critic 仍未列入 `critics` 列表，不参与计算。

有意保留的差异（不是遗漏）：

- `wz_max`：仿真 3.0，实车 0.0。实车的 yaw 由小陀螺与决策控制，MPPI 不输出转向速度。
- 仿真的 `VelocityDeadbandCritic` 配置块未迁移：它不在仿真的 `critics` 列表中，本身不参与计算，迁移只会增加一处无效配置。
- `velocity_smoother` 未对齐：仿真 `scale_velocities: True`、`max_accel: [2.0, 2.0, 2.5]`，实车 `False`、`[4.5, 4.5, 5.0]`。这一层决定实际加减速能力，若要求两个平台表现一致，需要与 MPPI 参数一起决定。

仍未决定：

- 仿真 local 的 `inflation_radius`（0.40，与 inscribed 0.369 之间只剩 0.031 m 梯度带）。
- 实车 local 的 `inflation_radius`（0.364，小于 inscribed 0.369）。

### 6.1 确定不生效或自相矛盾的项

| # | 位置 | 现状 | 问题 |
| --- | --- | --- | --- |
| 1 | `reality/controller/mppi.yaml` | `controller_frequency: 60.0` 而 `model_dt: 0.1` | 控制周期 0.0167 小于 `model_dt`，按源码只告警并**关闭控制序列移位**；官方要求 `model_dt` 等于控制周期且不得更大。`log/base_mppi.json` 记录过运行期 `model_dt = 0.166666667`，同属这类失配 |
| 2 | `reality/controller/mppi.yaml` | `wz_max: 0.0` 而 `wz_std: 1.0` | `wz_max = 0` 使 `wz` 被钳到 0，`wz_std` 的采样噪声不产生任何效果，属于多余开销但无害。**不要因此把 `wz_std` 设为 0**：Humble 的 `updateControlSequence()` 做 `gamma / wz_std²` 除法且没有零值守卫，除出 `inf` 后再乘上恒为 0 的控制量会得到 `NaN`，整批代价与最终输出一起失效。要调小就取 0.01 这样的非零值；零值守卫是主分支 PR #5110 之后才加入的 |
| 3 | `reality/controller/mppi.yaml` | `ax_max: 5.0`、`ay_max: 5.0` | 本地 1.1.20 源码的 `parameters_handler.cpp` 未声明这两个参数（加速度限幅是 Rolling 之后才加入的），静态 yaml 中被静默忽略，运行期用 `ros2 param set` 会得到 "Parameter not found" 告警。加速度限制实际由 velocity_smoother 承担 |
| 4 | 两份配置 | `ObstaclesCritic: enabled: true`、`VelocityDeadbandCritic: enabled: true`，但都不在 `critics` 列表中 | `CriticManager::loadCritics()` 只按 `critics` 列表实例化 critic，未列入的配置块完全不生效。当前两个 profile 的避障完全由 `CostCritic` 承担；`simulation` 中 `VelocityDeadbandCritic.deadband_velocities: [0.5, 0.0, 0.0]` 同样未生效 |
| 5 | `reality/controller/mppi.yaml`（local costmap） | `robot_radius: 0.369` 而 `inflation_radius: 0.364` | 膨胀半径小于内切半径（本轮 robot_radius 从 0.374 改到 0.369 后两者相差 5 mm），`inflation_layer` 会报警，且内切半径之外没有代价梯度。global costmap 已按同一理由改为 0.9，local 侧未同步，而 MPPI 的 `CostCritic` 读的是 local 代价地图 |
| 6 | 两份配置 | `ObstaclesCritic.inflation_radius` / `cost_scaling_factor` 与 local costmap 膨胀层不一致（simulation：0.40/10.0 对 0.40/8.0；reality：0.55/10.0 对 0.364/4.0） | 该 critic 目前未启用，一旦启用，其距离换算会直接错。Humble 要求两者一致 |

### 6.2 与官方建议有偏差、需要实测判断的项

| # | 位置 | 现状 | 说明 |
| --- | --- | --- | --- |
| 7 | `reality` | 预测视界 `48 × 0.1 = 4.8 s`，`vx_max = 2.5 m/s` → 前向覆盖 12 m，而 local costmap 半径 6 m | 违反 4.2 的判据，官方明确指出此时最大速度会被人为限制 |
| 8 | `simulation` | 预测视界 `56 × 0.033333 = 1.867 s` → 前向覆盖 4.67 m，local costmap 半径 6 m | 满足 4.2 判据 |
| 9 | 两份配置 | `PathAlignCritic.offset_from_furthest: 10` | 按 4.3 的基线公式，simulation 应约 31、reality 应约 80（按路径分辨率 0.05 m 估算，实际分辨率需按 smoother 输出确认）。当前取值偏小，可能在起步阶段误触发 |
| 10 | 两份配置 | `vx_std: 0.5`、`vy_std: 0.5`、`wz_std: 1.0`（官方默认 0.2/0.2/0.4） | 高速机器人确实需要更大的采样标准差，但按 1.2 的耦合关系，这会把 `gamma` 的控制努力项削弱约 6 倍 |
| 11 | `reality` | `CostCritic.cost_power: 10`（simulation 为 1） | `CostCritic` 的幂次作用在 `weight × 累计代价 / 轨迹点数` 之上，10 次幂会把代价分布拉得极开，等效于把该 critic 变成硬约束。若这是实车的保守策略，建议在注释中写明意图 |
| 12 | 两份配置 | `prune_distance` 未设置（默认 1.5 m） | 官方要求它与最大速度和预测视界成比例；当前 1.5 m 只相当于 2.5 m/s 下 0.6 s 的路径长度 |
| 13 | 两份配置 | `GoalCritic.threshold_to_consider: 2.0`、`PathFollowCritic.threshold_to_consider: 1.4` | 按 4.4，这两个阈值宜与预测视界对应的距离对齐，以保证 2 m 以内的交接行为连续 |
| 14 | `reality` | local costmap `update_frequency: 10.0` 而 `controller_frequency: 60.0` | 不是错误，但意味着同一张代价图会被 6 个控制周期复用，避障反应延迟以 100 ms 为下限 |

### 6.3 计算核对表

| 量 | simulation | reality | 判据 |
| --- | --- | --- | --- |
| `controller_frequency` | 30 Hz | 30 Hz | 官方建议不低于 20 Hz，推荐 30 Hz |
| `model_dt` | 0.033333 | 0.033333 | 必须等于 1/30（与 1/30 的误差 3e-10，满足 1e-6 容差） |
| `time_steps` | 56 | 56 | — |
| 预测视界 `T` | 1.867 s | 1.867 s | — |
| 视界内前向距离 `T × vx_max` | 4.67 m | 4.67 m | ≤ local costmap 半径 6 m，两者均满足 |
| `batch_size` | 1800 | 1800 | 官方 30 Hz↔2000、50 Hz↔1000 |
| 单周期轨迹点数 `batch × steps` | 100 800 | 100 800 | 与频率相乘估实时性 |
| `PathAlign.offset_from_furthest` | 10 | 10 | 基线约 31 |

两份配置的控制器参数现已一致，差异只剩 `wz_max`（见 6.0）。`vx_std`/`vy_std` 由 0.25 提到 0.5 之后，按 1.2 节的耦合关系，`gamma` 的控制努力项被削弱到原来的 1/4。

### 6.4 专项：仿真 profile 下"偶发撞障碍"的成因分析

先给出判定判据的源码依据（`src/critics/cost_critic.cpp`，Humble 1.1.20）：

```cpp
switch (static_cast<unsigned char>(cost)) {
  case (LETHAL_OBSTACLE):                     // 254
    return true;
  case (INSCRIBED_INFLATED_OBSTACLE):         // 253
    return consider_footprint_ ? false : true;   // 本项目 consider_footprint=false → 直接判碰撞
  case (NO_INFORMATION):                      // 255
    return is_tracking_unknown ? false : true;   // 本项目未开 track_unknown_space → 直接判碰撞
}
```

另外 `costAtPose()` 对代价地图**范围之外**的坐标直接返回 `NO_INFORMATION`。
这两条合起来说明：在当前配置下，"中心点进入内切半径"与"轨迹点跑出代价地图"都会被当作**真碰撞**处理（代价 1e6），而不是软惩罚。

按此逐项排查，仿真侧可疑点如下（按可能性排序）：

1. **局部代价地图的膨胀梯度带过窄**。simulation 的 local costmap 为 `robot_radius 0.369`（本轮由 0.35 改上来）+ `inflation_radius 0.40`，即内切半径之外只剩 0.031 m 的代价衰减区。
   `CostCritic` 的行为因此退化成两档：位姿落在 0.369 m 内 → 每点 `critical_cost` 300 或直接判碰撞；落在 0.369 m 外 → 代价迅速降到接近 0。
   中间的"渐进斥力"几乎没有，机器人因此倾向于贴着 0.369 m 的硬边界走，边缘余量不足时表现为偶发擦碰。
   官方对这一点有过说明（"代价地图的平滑膨胀非常关键"，以及 `repulsion_weight` 与膨胀半径成反比），global costmap 已按同一理由改到 0.9，local 侧未同步，而 `CostCritic` 读的是 local 代价地图。
   调整方向：把 local costmap 的 `inflation_radius` 提到 0.55~0.70（不是 0.9——局部图更小，梯度过宽会让机器人不肯进窄口），同时观察 `cost_scaling_factor` 是否要一并调整；两者都要与 `CostCritic` 的权重一起看，不是单独改一个参数就能定。

   量化依据：Nav2 膨胀层的代价约为 `253 × exp(-cost_scaling_factor × (d - inscribed_radius))`。

   | 配置 | 梯度带宽 | 带内位姿代价范围 |
   | --- | --- | --- |
   | 当前 local（仿真）：inscribed 0.369，inflation 0.40，scaling 8.0 | 0.031 m | 253 → 197 |
   | 当前 global（实车）：inscribed 0.369，inflation 0.9，scaling 4.0 | 0.531 m | 253 → 30 |
   | 可选：inscribed 0.369，inflation 0.70，scaling 4.0 | 0.331 m | 253 → 67 |

   第一行说明当前 local 的这个带几乎不衰减：0.369 m 之外代价直接掉到 0，带内则一直维持在 197 以上。`CostCritic` 的"一般避障偏好"项按位姿代价累加，在这种分布下没有可用的中间档位。
2. **近目标 1 m 内关闭了一般避障偏好**。`CostCritic.near_goal_distance: 1.0` 使机器人在距目标 1 m 以内时不再累加一般的位姿代价，只剩碰撞与内切硬惩罚；`GoalCritic.threshold_to_consider: 2.0` 使目标代价从 2 m 起算。
   若目标点本身靠近障碍（赛场上的目标点常贴着墙或障碍），最后 1 m 就是最容易蹭的一段。官方默认该参数为 0.5。
3. **控制链路时间不同步**。local costmap `update_frequency: 10.0`、控制器 `controller_frequency: 30.0`、`velocity_smoother` 的 `smoothing_frequency: 20.0`。
   在 2.5 m/s 下，一张代价图的时效是 0.1 s ≈ 0.25 m 位移，且 MPPI 的 `model_dt = 1/30` 假设"自己输出的速度会被立刻执行"，而中间还夹着 20 Hz 的平滑器与加速度限幅。动态障碍场景下这是偶发事件的合理来源。

   另外，仿真与实车的平滑器策略目前并不一致：仿真为 `scale_velocities: True`、`max_accel: [2.0, 2.0, 2.5]`、`max_decel: [-3.0, -3.0, -4.0]`，实车为 `scale_velocities: False`、`max_accel: [4.5, 4.5, 5.0]`、`max_decel: [-4.5, -4.5, -5.0]`。仿真的加减速能力约为实车的一半，且限幅方式（按比例缩放还是逐轴削顶）不同。这一条不影响撞障碍的成因判断，但会影响"仿真调好的 critic 权重能否迁移到实车"，迁移前建议先把这一层对齐。
4. **感知链漏检**。local costmap 的障碍来自 `terrain_map`（terrain_analysis 的平面网格输出）与 `terrain_returns_current`，源级高度下限为 0（`min_obstacle_height: 0.0`）。被判为地面或高度带以外的障碍不会进入代价地图，控制器无从避让。
   排查方法是把"撞的那一刻"的 `terrain_map` 点云与该时刻的 local costmap 对照，看障碍是否出现过。
5. **`ObstaclesCritic` 未启用**。它的 `repulsion_weight` 提供的正是第 1 条缺失的"渐进斥力"（按到障碍的距离给分，作用范围 `inflation_radius`）。
   当前它写在 yaml 里但不在 `critics` 列表，等于这套配置完全没参与计算。若启用，`inflation_radius`/`cost_scaling_factor` 必须同步为 local costmap 的实际值（当前写成 0.40/10.0，而 local 是 0.40/8.0）。

上述 1、2、5 三条互相耦合：它们都在描述"缺少中等距离上的避障引导"。合理的一次只改一项的顺序是先做第 1 条（把梯度带做出来），再看第 2 条的近目标行为，最后决定是否启用第 5 条。

### 6.5 专项：如何把"偶发撞障碍"定位到具体层

"偶发"意味着不是每次都发生，因此需要先确定坏现象出现在哪一层，而不是直接改参数。

1. 复现时录制 rosbag，至少包含：`local_costmap/costmap_raw`、`/trajectories`（需要把 `FollowPath.visualize` 临时设为 true）、`transformed_global_plan`、`odom`、最终下发的速度指令、以及 `terrain_map`。
2. 回放撞击前 0.5 s：若该障碍**已经出现在 local costmap 中**，问题在控制器或执行链（第 1、2、3 条）；若**没有出现**，问题在感知或代价地图（第 4 条），改 MPPI 参数不会有帮助。
3. 若障碍已在图中，再看 `/trajectories`：最优轨迹是否本来就贴着障碍边缘（第 1、2 条），还是指令正确而实际运动偏离（第 3 条，查 velocity_smoother 与底盘响应）。
4. 记录发生的场景特征（直线高速、窄口、目标点附近、动态障碍），不同特征的成因不同，混在一起看会互相掩盖。

---

## 7. 调试与操作手段


- **在线改参数**：`scripts/tune_controller.sh` 按 `scripts/params_list/mppi_para.txt` 生成菜单，通过 `ros2 param set` 即时生效（参数由 `ParametersHandler::dynamicParamsCallback` 接收）。注意该回调对未声明的参数会打印 "Parameter not found" 告警，可借此快速判定某个参数在当前版本是否存在（例如 `ax_max`）。
  每次参数变更后 `ParametersHandler` 会执行 post 回调，`Optimizer` 随之 `reset()`（`optimizer.cpp` 中注册的 `addPostCallback`），控制序列与代价状态清零，因此在线改参后会有一次短暂的行为跳变，比较两次试验时应把这段排除在外。`controller_frequency` 是静态参数，改动需要重启节点。
- **候选轨迹可视化**：把 `FollowPath.visualize` 临时设为 `true`，配合 `TrajectoryVisualizer.trajectory_step`/`time_step` 降采样。官方明确说明它显著拖慢控制器，只用于调试，`batch_size` 与 `time_steps` 越大越明显。
- **只看优化后的轨迹**：本版本（1.1.20）发布 `trajectories` 与 `transformed_global_plan` 两个话题；新版才有的 `publish_optimal_trajectory`、`publish_critics_stats` 在本版本不可用。
- **改完源码后的验证**：`scripts/pre-commit/run_mppi_tests.sh` 会编译 `nav2_mppi_controller` 并跑 12 个 gtest 套件。
- **单变量对比**：一次只改一个二级或三级参数，改前用 `tune_controller.sh dump` 记录当前值，改后跑同一条路径对比。

### 7.1 仿真上跑一轮调参的具体步骤

```bash
# 1. 启动（本手册涉及的是 MPPI，因此 controller 固定为 mppi）
scripts/simulation.sh nav rmuc_2025 planner:=smac2d controller:=mppi use_rviz:=True

# 2. 记录基线参数（另存一份，便于对比与回退）
scripts/tune_controller.sh save /tmp/mppi_baseline.yaml
scripts/tune_controller.sh show

# 3. 在线试参（改完立即生效，不需要重启；重启后回到 yaml 取值）
scripts/tune_controller.sh set CostCritic.cost_weight 6.0
#    未声明的参数会打印 "Parameter not found"，可用来确认版本是否支持

# 4. 需要看候选轨迹时（会显著拖慢控制器，看完记得关掉）
scripts/tune_controller.sh set visualize true

# 5. 复现失败场景时录制数据
ros2 bag record /local_costmap/costmap_raw /trajectories \
  /transformed_global_plan /odom /cmd_vel /terrain_map
```

对应本项目的参数名对照见 `scripts/params_list/mppi_para.txt`（同目录下还有 MPC 与 PID 的对照表）。

### 7.2 仿真侧已确认需要修的项（与手感无关，属配置错误）

1. `critics` 列表补上实际想用的 critic。当前 `ObstaclesCritic` 与 `VelocityDeadbandCritic` 的配置块不会被加载；若不打算用，应删掉配置块或注释，避免以后误以为它在生效。
2. 若启用 `ObstaclesCritic`，把它的 `inflation_radius`/`cost_scaling_factor` 改成与 local costmap 膨胀层完全一致。
3. `PathAlignCritic.offset_from_furthest: 10` 相对官方基线（约 31）偏小，容易在起步阶段误触发；改这个值之前先确认路径点实际间距。

### 7.3 系统化调参（可选，适合多轮反复试参）

手工改一项、跑一遍的做法在参数多于两三个时会失去可比性。可借鉴的工程化做法是：固定场景、固定初始位姿，录制一段 rosbag，然后用参数覆盖批量回放，对每个参数组合记录客观指标。

项目内已有一个 bag 回放对比的先例：`scripts/pl_cmp/`（Point-LIO 版本对比）。MPPI 侧可用的指标包括：与障碍的最小距离、任务耗时、指令速度的抖动（角速度或横向速度的方差）、是否触发恢复行为。

需要注意的技术前提：`ros2 param set` 是运行期修改，回放时要保证每次回放的初始状态一致，否则指标不可比。

---

## 8. 来源

官方：

- [nav2_mppi_controller README（humble 分支）](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)：参数表与 "Notes to Users" 调参方法论，本文第 3、4 节的主要依据。
- [Nav2 文档 MPPI 配置页（Rolling 版）](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)：新版参数（加速度限幅等），用于区分版本差异。
- [ROSCon 2023：On Use of Nav2 MPPI Controller](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf)：`batch_size` 与频率的配对、膨胀平滑的重要性。
- [issue #4376](https://github.com/ros-navigation/navigation2/issues/4376)：`PathAlign` 与 `PathAngle` 的分工、`offset_from_furthest` 的含义。
- [issue #5375](https://github.com/ros-navigation/navigation2/issues/5375)：维护者对差速底盘近距离机动的答复（指向 README 调参节）。
- [issue #5376](https://github.com/ros-navigation/navigation2/issues/5376)：`wz_std = 0` 的作用，以及 velocity_smoother 默认横向速度上限导致全向底盘不横移的实例。
- [issue #4849](https://github.com/ros-navigation/navigation2/issues/4849)：维护者说明调参问题应在 Robotics Stack Exchange 讨论，并指向 README 的 Notes to Users。
- [issue #5386](https://github.com/ros-navigation/navigation2/issues/5386)、[#5694](https://github.com/ros-navigation/navigation2/issues/5694)：同一份 YAML 在 Humble 能跑到 0.8 m/s、在 Jazzy 只有 0.25 m/s 一类现象，根因是 `ax_max` 等加速度参数在 Humble 不被读取、在新版被真正施加。今后若升级 Nav2 版本，需要重新确认这几个参数。
- [issue #5714](https://github.com/ros-navigation/navigation2/issues/5714)、[#4049](https://github.com/ros-navigation/navigation2/issues/4049)：维护者给出的吞吐配比（"至少 30 Hz 配 2000 条采样，或 50 Hz 配 1000 条"）与 20 Hz 的下限说明。

社区（仅作案情参考，这些帖子本身没有给出经过验证的参数结论）：

- [Nav2 MPPI Controller Will Not Reach Max Speed](https://robotics.stackexchange.com/questions/117814/nav2-mppi-controller-will-not-reach-max-speed)：6 m/s 目标只跑到 0.7 m/s 的公开案例，其参数中预测视界为 2 s，对应前向距离 12 m，远超其代价地图尺寸，与第 4.2 节的判据一致。
- [Nav2 MPPI Controller Tuning help](https://robotics.stackexchange.com/questions/113238/nav2-mppi-controller-tuning-help)：Ackermann 车辆贴线不准，只保留 PathFollow/PathAngle 两个 critic 的做法未解决问题；该帖没有有效回答。
- [In-place rotation with MPPI controller using turtlebot3](https://robotics.stackexchange.com/questions/114198/in-place-rotation-with-mppi-controller-using-turtlebot3)：原地转向相关，唯一回答是建议改用 Rotation Shim Controller。
- [MPPI Critics Debugging & Tuning（Artefacts）](https://docs.artefacts.com/example-projects/mppi/)：商业遥测工具的做法示例，思路是按参数组合回放并对比 critic 代价，可作 7.3 节的参考，工具本身需要注册，本项目无法直接使用。
- 中文技术站上大量"Nav2 MPPI 调参"文章为自动生成内容，出现过把新版才有的 `ax_max` 一类参数当作通用配置介绍的写法，不宜作为依据（本项目 Humble 版本不支持这些参数，见 2.1 节）。
- [Open Navigation：Improving MPPI for High-Inertia Industrial Vehicles](https://opennav.org/news/mppi-low-acceleration/)：厂商实测，4.7 节中"低加速平台应减小采样标准差"与 `model_dt` 失配影响加速度上限的分析来源。
- [NAV2: MPPI Parameters Tuning（robotcopper）](https://robotcopper.github.io/ROS/MPPI.html)：社区长文，用于交叉验证，其中关于 `prune_distance` 的定量建议不是官方结论。

本仓库内：

- [MPPI调参_资料汇编.md](MPPI调参_资料汇编.md)：本轮调研的外部资料汇编（598 行），含逐症状的数值起点、核心参数的数学含义、按官方/源码/社区/未验证分级的出处，以及 12 条未核实事项清单。手册中凡标注"待实测"的结论，其取舍依据都可以在该汇编的未验证清单里找到对应条目。

项目内：

- `config/simulation/controller/mppi.yaml`、`config/reality/controller/mppi.yaml`：两份被体检的配置。
- `src/guga_controller/nav2_mppi_controller/src/optimizer.cpp`、`src/critics/*.cpp`、`src/parameters_handler.cpp`：机制与默认值的直接依据。
- `log/base_mppi.json`、`log/base_sim_mppi.json`、`log/after_mppi.json`：运行期参数快照。
