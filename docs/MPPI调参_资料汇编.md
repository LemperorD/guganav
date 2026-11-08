# Nav2 MPPI 控制器（nav2_mppi_controller）症状→参数调参技术要点汇编

> 本文是本轮 MPPI 调参调研整理出的**外部资料汇编**，用于查参数含义与出处，不含本项目的配置结论。项目侧的问题清单与改动建议见 [MPPI调参手册.md](MPPI调参手册.md)。
> 文中标注了四类来源（官方/源码/社区/未验证），第 5 节集中列出 12 条未核实事项，引用时请连带看该节。

面向 ROS 2 Humble 的 nav2_mppi_controller，以 1.1.17～1.1.20 为主要参考版本。文中凡涉及「Humble 与 Jazzy/Rolling 行为不同」的地方都单独标出，因为 Humble 的 MPPI 与后续版本在**参数集合、critic 集合、命名、数值行为**上都有实质差异。

**来源分级约定**
- 【官方】= Nav2 官方文档 / README / 维护者 Steve Macenski 本人发言
- 【源码】= 我在对应分支源码里直接核实
- 【社区】= 其他用户报告的个人经验
- 【未验证】= 有说法但我无法核实

---

## 0. 先看这一节：Humble 特有的六个前提

在调任何参数之前，这六条会解释掉相当一部分「调了没反应」的情况。

### 0.1 Humble 的 MPPI 没有加速度参数

Humble 的 `ControlConstraints` 结构体只有 `vx_max / vx_min / vy / wz` 四个成员（[constraints.hpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/models/constraints.hpp)）；`optimizer.cpp` 里 `ax_max`、`ax_min`、`ay_max`、`az_max` 出现次数为 0（[optimizer.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp)）。Humble 版 README 的参数表里也没有这几个参数。

**结论**：在 Humble 配置里写 `ax_max: 3.0`、`az_max: 3.5`、`ay_max: 0.0` 完全不生效，不报错也不警告。加速度在 Humble 只能间接影响：靠 `vx_std`/`wz_std`（采样散布范围）和轨迹平滑后的控制序列变化率。

同一现象在 Jazzy/主分支会发生实质变化：`ax_max` 默认 `3.0`、`az_max` 默认 `3.5` 会被真正读取并施加硬约束，`opts.ax_max: 3.0` 这类偏小的默认值会把速度压住。社区在 [issue #5386](https://github.com/ros-navigation/navigation2/issues/5386) 里定位到「同一份 YAML 在 Humble 能到 0.8 m/s、在 Jazzy 只有 0.25 m/s」的根因正是「这些加速度参数在 Humble 不生效、在 Jazzy 生效」，维护者回复「Great! I'm glad you found your issue :-)」。另一用户在 [#5694](https://github.com/ros-navigation/navigation2/issues/5694) 报告迁移后速度被恒定压在 0.17 m/s。

### 0.2 `wz_std` 不能设为 0

Humble 的优化器在控制代价项里做 `gamma / wz_std²` 的除法（[optimizer.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L362-L373)），`wz_std = 0` 会得到 `nan`，进而污染整条控制序列。主分支已加 `if (s.sampling_std.wz > 0.0f)` 守卫（[optimizer.cpp @main](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L666)），修复来自 [PR #5110](https://github.com/ros-navigation/navigation2/pull/5110)（对应 [issue #5021](https://github.com/ros-navigation/navigation2/issues/5021)，维护者确认并合并）。

**实践**：Humble 上想让机器人不转，最小可用值是 `wz_std: 0.01` 而不是 0。维护者在上面的 issue 里给的原话是「If you set the `wz_std` to zero, then no WZ will be computed and thus no rotation will occur」——这句话对 Humble 需要打折扣，因为 Humble 会先出 NaN。

### 0.3 `PathAngleCritic` 的「方向偏好」参数名是 `forward_preference`，不是 `mode`

Humble 1.1.20 源码读取的是 `forward_preference`（bool，默认 true）（[path_angle_critic.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/path_angle_critic.cpp#L44-L49)）。大量社区示例里的 `mode: 0/1/2` 属于 Jazzy 及以后。

另外 Humble 里这个 critic 是否允许倒车**不由自身参数决定**，而是读父级 `controller_server.ros__parameters.vx_min`：`vx_min` 为 0 时 `reversing_allowed_ = false`，并强制 `forward_preference_ = true`。所以 `vx_min: 0.0` 的机器人在 Humble 上等于「永远不倒车」。

### 0.4 Humble 默认控制器是 DWB，不是 MPPI

Humble 的 `nav2_bringup/params/nav2_params.yaml` 里 `controller_server.FollowPath.plugin` 是 `dwb_core::DWBLocalPlanner`，全文件不出现 MPPI（【源码】核实）。所以「Humble 的 MPPI 默认参数」只能以 `nav2_mppi_controller/README.md` 和 `optimizer.cpp` 里的 `getParam(..., default)` 为准。

### 0.5 `model_dt` 必须**恰好等于** 1/controller_frequency

```cpp
const double controller_period = 1.0 / controller_frequency;
if ((controller_period + eps) < settings_.model_dt)      -> 仅 WARN
else if (abs(controller_period - settings_.model_dt) < eps) -> INFO，开启控制序列平移
else                                                       -> throw std::runtime_error("Controller period more then model dt, set it equal to model dt")
```

见 [optimizer.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L100-L122)。参数名是 `controller_server` 下的 `controller_frequency`，MPPI 通过 `getParentParam` 读取；写在 `FollowPath:` 下面读不到（维护者在 [issue #4049](https://github.com/ros-navigation/navigation2/issues/4049) 里对一份把它写在插件块内的配置评论：「That's also not set in the right place, so I don't think that even is working」）。

**实践**：20 Hz ↔ `model_dt: 0.05`；30 Hz ↔ `0.0333`；50 Hz ↔ `0.02`。`time_steps * model_dt` 是预测时域。

**不一致会发生什么（Humble 上因为不读加速度参数，危害相对小；Jazzy 及以后是真问题）**：Open Navigation 的高惯性平台分析指出两条机制（[原文](https://opennav.org/news/mppi-low-acceleration/)）：
1. 控制周期 0.05 s 而 `model_dt = 0.1 s` 时，加速度约束是在优化器里按 `model_dt` 施加的，**等于把 `cmd_vel` 的等效加速度上限放大了 1 倍**（比值 = `model_dt / control_period`）；开环模式下看不到里程计反馈，这个放大不会被纠正。
2. 即使两者相等，控制序列平移（`shift_control_sequence`）会跳过 `vx(0)` 而执行 `vx(1)`，**又白白多用了一个时间步，让加速度上限再加倍**。他们的修法是把 `t = 0` 索引硬设为当前速度，使 `t = 1` 必须落在「当前速度 ± 1 个时间步的动态限制」内。

所以「`model_dt` 严格等于控制周期」不只是为了避免异常，它是加速度行为可预测的前提。

### 0.6 控制频率下限

维护者在 [#4049](https://github.com/ros-navigation/navigation2/issues/4049) 的原话：20 Hz 是底线，「30hz is the minimum I recommend」；在 [#5714](https://github.com/ros-navigation/navigation2/issues/5714) 给出吞吐配比：「at least 30hz with 2000 samples or 50hz with 1000 samples」。

这个配比在维护者的 ROSCon 2023 演讲《On Use of Nav2 MPPI Controller》里作为官方数字出现（我在幻灯片文本中核实到原文「**Tuned: 30Hz @ 2000, 50Hz @ 1000**」，[幻灯片直链](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf)、[录像](https://vimeo.com/879001391)）。同一页还有一条容易被忽略但很关键的提示：「**Costmap Smooth Inflation Critical! (like Smac)**」——膨胀层不够平滑会直接损害 MPPI 的避障决策质量，这属于「先修地基再调参」的范畴。

---

## 1. 常见症状 → 具体参数调整表

每条给出：**参数名 / 调整方向 / 典型数值起点 / 根因**。凡未特别标注来源的，方向性结论出自官方 README 的 *Notes to Users*（[Humble 版](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md#notes-to-users)，[当前版](https://docs.nav2.org/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/#notes-to-users)）。

### 1.1 机器人抖动、角速度 chatter、头尾摇摆

| 项 | 内容 |
|---|---|
| 参数 | `wz_std` ↓ ；`vx_std` ↓（仅当线性也抖） |
| 起点 | `wz_std` 0.4 → 0.25 → 0.15；下限 0.05。Humble 禁止到 0 |
| 参数 | 障碍类 critic 权重 ↓ 或膨胀层参数对齐 |
| 起点 | Humble `ObstaclesCritic.repulsion_weight` 1.5 → 0.8～1.0（大机器人/大膨胀半径时）；或改用 `CostCritic` 并把 `cost_weight` 从 3.81 降到 ~2.5 |
| 参数 | `PathAlignCritic.cost_weight` ↓，`PathAngleCritic.cost_weight` ↑，`offset_from_furthest` ↑ |
| 起点 | PathAlign 14 → 8～10；PathAngle 2.0 → 3.5～5.0；两者 `offset_from_furthest` 按 §3.2 公式算出的基线值 |

**官方根因**：`wz_std` 过大 → 采样出来的角速度散布过宽 → 加权平均结果在相邻周期之间来回跳（README：「If you're seeing a lot of chatter in the angular velocity, reduce its std」）。

**官方根因（障碍侧）**：「If the Obstacle critic is not well tuned with the costmap parameters (inflation radius, scale) it can cause the robot to **wobble** significantly as it attempts to take finitely lower-cost trajectories with a slightly lower cost in exchange for jerky motion.」同一个 critic 在自由空间里还会「try to maximize time in a small pocket of 0-cost over a more natural motion」，表现为原地画圈式微动。

**社区根因与配方**：[issue #5531](https://github.com/ros-navigation/navigation2/issues/5531)（仿真稳定、实机摇摆）。用户 Manouselis 的修复是「**decreasing the `PathAlignCritic` `cost_weight`** and **increasing the `cost_weight` of the `PathAngleCritic`** as well as **increasing the `offset_from_furthest`** based on the value that the documentation suggests (1/3 of the projected horizon). Lastly, **decreasing the acceleration/deceleration limits also helped**」。同线程维护者先怀疑丢拍，建议试 `model_dt: 0.05 → 0.0333`。注意这是 Jazzy，Humble 上「降低加速度限制」那条不适用（§0.1）。

**实机才抖、仿真不抖**：维护者的判断是硬件侧差异而非参数问题——「odometry rates, quality, control execution latency」（[#5531](https://github.com/ros-navigation/navigation2/issues/5531)）。官方 README 也要求「your odometry publishes at least as fast as your control frequency (ideally much faster)」。

### 1.2 转向过度 / 画龙 / 路径跟踪偏移（不贴路径）

| 项 | 内容 |
|---|---|
| 参数 | `PathAlignCritic.cost_weight` ↑（贴路径） |
| 起点 | 默认 14.0；贴不住就到 18～20；再高会让机器人「不肯离开路径躲障碍」 |
| 参数 | `PathAlignCritic.offset_from_furthest` |
| 起点 | 官方基线 `time_steps * model_dt * vx_max / path_resolution / 3.0`。例：56×0.05×0.5 / 0.05 / 3 ≈ 9～10（2 cm 路径分辨率则约 4） |
| 参数 | `PathAlignCritic.trajectory_point_step` 4 → 2（提高对齐精度，代价是算力） |
| 参数 | `PathAngleCritic.max_angle_to_furthest` 1.2 → 0.6～0.8（更早介入修正航向） |
| 参数 | `PathAngleCritic.cost_weight` 2.0 → 3.0～5.0 |
| 参数 | `use_path_orientations: true`（配合 Smac Hybrid-A*/State Lattice，让方向变化只发生在规划器要求的位置） |
| 参数 | `prune_distance` ↑（见 §1.6，被截断的路径会让跟踪整体变差） |

**官方根因（两个 critic 的分工）**：维护者在 [issue #4376](https://github.com/ros-navigation/navigation2/issues/4376) 解释：
- `PathAlign` 是**做全部主要工作**的那个，「aligns the robot to the path by looking at the integrated distance between the reference path and the trajectory and scoring negatively for the sum total discrepancy」；
- `PathAngle` 是**补边角场景**的，「looks at the angle wrt the robot and a point on the path some N points ahead and scores negatively if the angle is 'large'」，价值在于「get out of some sticky situations where you're doing a sharp turn or otherwise are misaligned」；删掉它会让「failure rates / total task times increase」。

**注意**：把 PathAlign 权重推得过高会换来另一种失败——官方 README：「highly tuning the system to follow the path will give it less ability to deviate to avoid obstacles (though it'll slow and stop)」。

### 1.3 不避障 / 贴障碍走

| 项 | 内容 |
|---|---|
| 参数 | `CostCritic.cost_weight` ↑（Humble 默认 3.81） |
| 起点 | 4.5～6.0；同时把 `critical_cost` 300 → 400～600 |
| 参数 | `CostCritic.consider_footprint: true` |
| 参数 | `PathAlignCritic.cost_weight` ↓（给避障让出空间） |
| 参数 | 局部代价地图 `inflation_layer.inflation_radius` ↑、`cost_scaling_factor` ↓（让代价地形更宽更缓） |
| 参数 | Humble 的 `ObstaclesCritic.inflation_radius` / `cost_scaling_factor` 必须与局部代价地图**一致**（README 标注 "Humble only"） |

**官方根因**：`CostCritic` 直接用膨胀代价，`cost_weight: 3.81` 是官方默认（维护者声明这个值「closely pre-tuned」，对小型 AMR）。机器人尺寸远大于 TurtleBot 时需要重新配。

**Humble 的 `ObstaclesCritic` 有一个已知的精度缺陷**（这点决定了「想贴障碍走但贴不住」很可能是工具问题而非参数问题）：它把代价反算成距离时用的是 `inflation_scale_factor * min_radius - log(cost) + log(253)` 这一类公式，且当 footprint 有任一点落在内切圆区域时 `footprintCostAtPose` 返回的最高代价恒为 `INSCRIBED_INFLATED_OBSTACLE`，反算出的距离恒定（[obstacles_critic.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/obstacles_critic.cpp#L112-L125)）。原始讨论在 [issue #4057](https://github.com/ros-navigation/navigation2/issues/4057)：报告者的原话是「If the footprint has some points in the inscribed area, the score is the same regardless of the distance to the obstacle... the score will be the same in a situation where it is important to evade obstacles」。这也是 `CostCritic`（当时叫 `InflationCostCritic`）被引入的直接动因。

[issue #5590](https://github.com/ros-navigation/navigation2/issues/5590) 进一步指出，同样的反算公式对**非圆形**机器人本身就不准确（它假定代价来自内切半径处的点，而 footprint 检查的高代价可能来自任一角点），会「May prefer trajectories that are actually closer to obstacles」。该 issue 是报告性质，未确认已修。

**可执行的规避**：非圆形机器人优先用 `CostCritic`（不做距离反算、直接用代价），或把 Humble `ObstaclesCritic` 的 `cost_scaling_factor` 调小（代价沿膨胀半径衰减更慢，代价阶梯更细，量化误差相对变小）。

### 1.4 绕障太保守 / 在自由空间里不肯动 / 起步犹豫

| 项 | 内容 |
|---|---|
| 参数 | `CostCritic.cost_weight` ↓，或 Humble `ObstaclesCritic.repulsion_weight` ↓ |
| 起点 | repulsion_weight 1.5 → 0.8～1.0；CostCritic 3.81 → 2.5～3.0 |
| 参数 | `PathFollowCritic.cost_weight` ↑（提高「往前走」的意愿） |
| 起点 | 5.0 → 8～15（官方：「To overcome them, increase the FollowPath critic cost」） |
| 参数 | `PruneDistance` 与局部代价地图尺寸 ↑（见 §1.6） |
| 参数 | `PathAngleCritic.offset_from_furthest` ↓ 或 `max_angle_to_furthest` ↑（降低转向抑制力度） |
| 参数 | `PreferForwardCritic.cost_weight` ↓（当前进意愿被过度惩罚时） |

**官方根因**（原文，值得逐字理解）：「it may generally **refuse to go into costed space at all** when starting in a free 0-cost space if the gain is set disproportionately higher than the Path Follow scoring... This is due to the critic cost of **staying in free space becoming more attractive than entering even lightly costed space** in exchange for progression along the task.」

即：在自由空间里「待在原地」的总代价，低于「进入一段低代价区域以换取路径推进」的总代价，优化器理性地选择了不动。解法就是拉高 PathFollow 或压低障碍项，**两者必须成对调**。

**社区配方（Humble 实机验证）**：[issue #5928](https://github.com/ros-navigation/navigation2/issues/5928) 里用户给出的可用组合是「setting the `offset_from_furthest` in `PathAlignCritic` low and increase `PathFollowCritic.offset_from_furthest` to 20 or 30 then it works」。维护者在该线程的补充是：「Is your local costmap large enough to move at 0.8 m/s for a trajectory time of `time_steps * model_dt` seconds? If not, adjust that. If so, review the `vx_std` / `wz_std` params.」

### 1.5 到点不收敛 / 在目标附近转圈

先弄清楚机制，再调，否则容易调错方向。

**`threshold_to_consider` 对两类 critic 的含义是相反的**，这一点在源码里定义得很清楚（[utils.hpp `withinPositionGoalTolerance`](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp#L237-L258)）：

- **目标类**（`GoalCritic`、`GoalAngleCritic`）：`if (!withinPositionGoalTolerance(...)) return;` → **只在机器人进入该半径时才生效**。默认 `GoalCritic` 1.4 m、`GoalAngleCritic` 0.5 m。
- **路径类**（`PathAlign`、`PathFollow`、`PathAngle`、`PreferForward`）：`if (withinPositionGoalTolerance(...)) return;` → **进入该半径后主动关闭**，把终点控制权交给目标类 critic。默认 `PathAlign`/`PathAngle` 0.5 m、`PathFollow` 1.4 m。

所以「离终点几厘米处转圈/停住」本质上是**交接带里的权重失衡**：

| 症状 | 调整 |
|---|---|
| 终点附近航向对不上、原地不转或反复转 | `GoalAngleCritic.cost_weight` 3.0 → 5～8；或把 `GoalAngleCritic.threshold_to_consider` 0.5 → 0.8～1.0（更早开始摆正） |
| 位置精度差（差 0.1～0.3 m 就不再靠近） | `GoalCritic.cost_weight` 5.0 → 8～10；`threshold_to_consider` 设为预测时域×速度（官方：与 path follower 的 threshold 设成同一个值可以「make sure you have clean hand-offs」） |
| 想让机器人一直贴路径到终点 | 把路径类的 `threshold_to_consider` 压到 0.0（维护者在 [Robotics SE #105182](https://robotics.stackexchange.com/questions/105182/appropriate-tuning-of-the-mppi-controller-for-close-goals) 给的方案），代价是需要重新平衡目标类权重，「you'll likely have to retune the goal/goal angle critics weights in their entirety」 |
| 终点在障碍附近，机器人不敢靠近 | `CostCritic.near_goal_distance` 0.5 → 0.8～1.2（官方默认注释：靠近终点时停止施加偏好避障项，允许它收敛到紧贴障碍的目标位姿） |
| 到点但一直在做微小修正（cmd_vel 长期 ~0.001 m/s） | goal_checker 的 `xy_goal_tolerance` 与 `yaw_goal_tolerance` 太紧；或 `min_x_velocity_threshold` / 速度平滑器 `deadband_velocity` 把微小指令吃掉，命令没到轮子。见 [issue #6185](https://github.com/ros-navigation/navigation2/issues/6185)（Humble，cmd_vel x≈0.0007、ω≈-0.06 但机器人不动） |

**维护者的官方立场（重要）**：他对目标的默认交接逻辑是刻意调的。「when you're within a meter or two of the goal, you really don't care much about what the global path says... my default tuning has that behavior... those throw the on-approach-to-final-goal tuning that I spent weeks on on your behalf」（[Robotics SE #105182](https://robotics.stackexchange.com/questions/105182/appropriate-tuning-of-the-mppi-controller-for-close-goals)）。也就是说，把路径 critic 的 gate 改小属于「改变官方设计意图」，要预期终点精度下降。

**近似距离机动（<0.5 m 的贴边目标）**：[issue #5375](https://github.com/ros-navigation/navigation2/issues/5375) 是这类问题的完整案例（差动机器人无法在终点附近修正横向误差，反复超时触发 backup）。维护者在该 issue 里没有给参数，而是指向 README 的调参章节，并说明「we don't provide tuning help in the issue tracker」。后续有用户在该 issue 留言报告同一现象（差动模型「cannot correct its lateral error around the goal position」）——**该问题在 issue 内没有形成公认解**，实际可行路线是：(a) 放宽 goal 容差并用 RotationShim 接管最终朝向；(b) 用 Omni 运动模型（若有横移能力）；(c) 把 `vx_min` 设为负值并提高 `PreferForwardCritic` 之外的自由度，让控制器能自己决定倒车修正。

### 1.6 速度上不去 / 达不到 vx_max

按概率从高到低排查，**前两条是环境问题不是 MPPI 问题**：

**① 速度平滑器静默限幅（最容易被忽略）**
Humble 的 `nav2_bringup/params/nav2_params.yaml` 里：

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.26, 0.0, 1.0]
    min_velocity: [-0.26, 0.0, -1.0]
```

（【源码】核实）。`max_velocity` 的 **y 分量默认是 0**，`max_velocity[0]` 默认 0.26——这两个默认值会静默把 MPPI 算出来的速度裁掉。[issue #5376](https://github.com/ros-navigation/navigation2/issues/5376) 的社区结论正是这个：「we eventually noticed that the velocity smoother plugin was using the defaults of nav2 which set max_velocity of y to 0. Changing that plus disabling `PathAngleCritic` got the expected behavior」。
**动作**：把 `max_velocity` / `min_velocity` 改成和 MPPI 的 `vx_max` / `vx_min` / `wz_max` 一致（或干脆把 velocity_smoother 从管线里去掉——参 [issue #6357](https://github.com/ros-navigation/navigation2/issues/6357) 里维护者的说明：smoother 是硬限制的施加者，MPPI 的加速度参数是「行为期望」）。

**② 底盘/驱动层限速**
同样在 #5694 里出现：`diff_drive_controller` 的 `linear.x.max_velocity`、`has_velocity_limits` 会先于 MPPI 生效。排查顺序：`ros2 topic echo /cmd_vel` → `/cmd_vel_nav` → `/cmd_vel_smoothed`，看速度在哪一级掉的。

**③ 采样标准差不足（MPPI 内部第一大原因）**
| 项 | 内容 |
|---|---|
| 参数 | `vx_std` ↑（主）、必要时 `wz_std` ↑ |
| 起点 | 目标 0.5 m/s → 0.2～0.3；目标 1.0 m/s → 0.3～0.5；目标 2.0 m/s 以上 → 0.6～1.0 |

这个参数是 [issue #4970](https://github.com/ros-navigation/navigation2/issues/4970) 的结论：用户发现「To achieve the desired result, we also need to increase `vx_std`」，维护者回应「Ah this is a really good insight... Would you mind opening a PR against the README with that note in the tuning section?」，随后**自己更新了官方文档**。现在官方文档里的原句是：

> If you're not seeing your robot get to full speed, **increase your std to explore more of the space**. ... The faster the robot is set to go, the **higher the velocity sampling standard deviations** should be in order to effectively explore the velocity space.

**数学根因**：控制序列在每一步都是 `vx = Σ_i w_i · (vx_nominal + ε_i)`，其中 `ε_i ~ N(0, vx_std²)`。若 `vx_std = 0.2` 而 `vx_max = 1.0`，绝大部分样本落在 0～0.6 之间，加权平均的结果自然被限制在低速区，**与 critic 权重无关**。这解释了「怎么调 cost 都没用」。

**副作用提醒**：提高 std 会同时削弱控制代价项（控制代价系数是 `gamma / std²`），并且社区反复报告「提高 std 后振荡加剧」（[#5694](https://github.com/ros-navigation/navigation2/issues/5694)：「I tried increasing vx_std and wz_std as you suggested; however, this caused significant oscillation」）。因此应按 §3 的顺序，先定速度 std，再回头压角速度 chatter。

**④ `prune_distance` 截断路径**
| 项 | 内容 |
|---|---|
| 起点 | `prune_distance ≥ vx_max × time_steps × model_dt × 0.3`（**社区经验值**，来自 [robotcopper 的 MPPI 调参长文](https://robotcopper.github.io/ROS/MPPI.html)） |
| 官方约束 | `prune_distance` 与「最大速度、预测时域」成比例 |

`prune_distance`（默认 1.5 m）定义为「Distance ahead of nearest point on path to robot to prune path to」——局部路径只保留机器人最近点前方这么长的一段（[path_handler.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/path_handler.cpp#L37-L75)）。官方文档对此有一条明确断言：

> The Path Follow critic **cannot drive velocities greater than the projectable distance of that velocity on the available path on the rolling costmap**.

即路径被剪短之后，「够得着的前方点」变近，PathFollow 的 carrot 变近，速度上限就被压住了。robotcopper 的表述更直接：「a prune_distance too low compared to the desired speed will result in a truncated speed」。

**⑤ 预测时域 × 最大速度 > 代价地图半径**
官方原文（含算例）：「if you predict forward 3 seconds (60 steps @ 0.05s per step) at 0.5m/s maximum speed, the **minimum** required costmap radius is 1.5m - or 3m total width.」超过这个范围，速度会被 costmap 人为限制。Humble 局部代价地图默认 3×3 m，跑 0.5 m/s 以上就必须放大。同理 `PathAlign` / `PathFollow` 的 `offset_from_furthest` 也要能在代价地图半径内放得下，否则被 threshold 掉。

**⑥ 从 Humble 迁到 Jazzy 后新增的加速度限制**
见 §0.1。特征是「同一份参数在 Humble 快、在 Jazzy 慢」。修复方向是把 Jazzy 的 `ax_max`/`ax_min` 放大（社区报告「had to increase `ax_max` to 15.0 in order to make my robot move as fast as I wanted」），或把 velocity_smoother 的 `max_accel`/`max_decel` 同步放大。

### 1.7 倒车 / 掉头行为异常

| 症状 | 参数 | 方向与起点 | 根因 |
|---|---|---|---|
| 该倒车时不倒车 | `PreferForwardCritic.cost_weight` | 5.0 → 0 或 1.0；`threshold_to_consider` 0.5 → 0.2 | 该 critic 直接惩罚反向前进。社区在 [issue #4425](https://github.com/ros-navigation/navigation2/issues/4425) 的修复是把 `PreferForwardCritic` 设为 0 |
| 该掉头时绕大圈 | `PathAngleCritic` 相关 | Humble：确认 `vx_min < 0`（否则 `reversing_allowed_=false`）；`max_angle_to_furthest` 0.8～1.0；`offset_from_furthest` 4 → 2～4 | 见 §0.3 |
| 原地不动也不转 | `wz_std` | 0 → 0.05～0.2（**Humble 严禁 0**） | §0.2 |
| 必须在指定点换向 | `enforce_path_inversion: true`、`inversion_xy_tolerance` 0.2、`inversion_yaw_tolerance` 0.4 | 三者一起用 | 官方定义：把路径在换向点处剪断，强制机器人「at or very near the planner's requested inversion point」换向 |
| 差动机器人无法原地转向起步 | RotationShimController | `angular_dist_threshold: 0.785`、`rotate_to_heading_angular_vel: 1.8`、`max_angular_accel: 3.2`、`forward_sampling_distance: 0.5` | 见下 |
| Ackermann 转弯跟不上/逆向后乱打方向 | `AckermannConstraints.min_turning_r` | 按真实最小转弯半径设（不能设 0）；并确认参数真的被读到 | 见下 |

**RotationShim 是维护者推荐的「简单出路」**。在 [issue #4049](https://github.com/ros-navigation/navigation2/issues/4049)（窄通道内目标在身后、机器人无法倒车）里，维护者说「The rotation shim controller certainly would be the easy way out」，同时表示 MPPI 自己配合 `PathAngleCritic` 也能做到原地旋转（「I've definitely been able to tune the robot with no backward motion and rotate in place to the heading of a path. Though, not in that confined of space.」）。RotationShim 的机制是：接收新路径时先原地转到路径朝向，再把手交给 `primary_controller`（[README @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/README.md)）。它还有个 `rotate_to_goal_heading` 参数：进入 XY 容差后由它接管转向目标航向——这正是 §1.5「终点朝向对不上」的工程化解法。[Robotics SE #114198](https://robotics.stackexchange.com/questions/114198/in-place-rotation-with-mppi-controller-using-turtlebot3)（原地旋转做不出来，提问者试过改 `PathAngleCritic` 全部参数、关 `PreferForward`、开 `use_path_orientations`，均无效）唯一的回答也是「I'd recommend looking into the Rotation Shim Controller」。

**Ackermann 的两个坑**：
1. Humble 的 `min_turning_r` 参数通过 `getParam(min_turning_r_, "min_turning_r", 0.2)` 读取，命名空间是 `<controller>.AckermannConstraints`（[motion_models.hpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp#L92-L96)）。写错层级会静默回落到 0.2。[issue #4509](https://github.com/ros-navigation/navigation2/issues/4509) 就是这个现象（默认 launch 下 `min_turning_r` 回落 0.2）。
2. 约束形式是 `wz[i] = copysign(|vx[i]| / min_turning_r, wz[i])`——**转向速率与纵向速度成正比**，`vx = 0` 时 `wz = 0`，即该模型物理上无法原地转向。这也是 Ackermann 车做 3 点掉头/倒车时行为怪异的结构性原因，不要指望用 critic 权重解决。

### 1.8 路径跟踪偏移的另一种成因：`offset_from_furthest` 用错

`offset_from_furthest` 不是距离，是**路径点索引**。官方定义（Humble README）：
- `PathFollow`：`offset_from_furthest`「Number of path points **after** furthest one any trajectory achieves to drive path tracking relative to.」默认 6。
- `PathAlign`：默认 20，且带一个**前置门**——路径总点数小于该值时 critic 直接 `return`（[path_align_critic.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/path_align_critic.cpp#L47-L58)）。
- `PathAngle`：默认 4。

**官方给的取值判据**：
- 太小（如 5）：官方原文「it can trigger when a robot is simply trying to start path tracking causing some suboptimal behaviors and **local minima** while starting a task」。
- 太大（如 50）：「the critic may never trigger or only do so when at full-speed」。
- 基线公式：「**`prediction_horizon_s * max_speed / path_resolution / 3.0`** is a good baseline.」

**关键耦合（官方强调）**：这个参数的值取决于路径分辨率与代价地图尺寸，「its important while selecting these parameters to make sure that the theoretical offsets can exist on the costmap settings selected with the maximum prediction horizon and velocities desired」。[Robotics SE #113238](https://robotics.stackexchange.com/questions/113238/nav2-mppi-controller-tuning-help) 的提问者（Ackermann，最小转弯半径 4.5 m）把 `offset_from_furthest` 统一设为 20/5/4、`PathFollow`/`PathAngle` 权重提到 10，仍然「vehicle not following the path accurately」——**该问题在 SE 上至今没有回答**（我在 2026-10 通过 Stack Exchange API 核实答案数为 0），不要把它当作已验证配方。

### 1.9 控制频率不足导致的延迟 / 算力不够

| 参数 | 起点 | 影响 |
|---|---|---|
| `controller_frequency` | 20 → 30 Hz（官方推荐下限 30） | 提高控制带宽；必须同步改 `model_dt`（§0.5） |
| `time_steps` | 56 | 预测时域 = `time_steps × model_dt`。降 `time_steps` 是最省算力的手段（线性下降） |
| `batch_size` | 1000～2000 | 线性影响算力。官方维护者建议「at least 30hz with 2000 samples or 50hz with 1000 samples」 |
| `iteration_count` | **保持 1** | 官方：「Recommend to keep as 1 and prefer more batches」 |
| `visualize` | 调参时 true，平时 **false** | 官方：「Visualizing 2000 batches @ 56 points at 30 hz is a lot」 |
| `regenerate_noises` | false | 官方：true 会引入「compute jittering at run-time due to thread wake-ups to resample normal distribution」 |
| `TrajectoryVisualizer.trajectory_step` / `time_step` | 5 / 3 → 调参时 **100 / 100** | 社区经验：只显示最优轨迹 |
| `CostCritic.consider_footprint` | true（安全）→ false（省算力） | 官方：footprint 检查「comes at increased compute cost」。维护者在 [issue #4057](https://github.com/ros-navigation/navigation2/issues/4057) 里说这个逐点代价查询是「the single slowest operation in the controller」 |
| `trajectory_point_step`（各 critic） | 2～4 | 单纯降算力，官方：1～10 都算合理 |
| `controller_server.failure_tolerance` | 0.3 s | 控制周期抖动超过这个值才报警；丢拍频繁时先看它有没有报 |
| `use_realtime_priority` | true | 维护者在 [#5714](https://github.com/ros-navigation/navigation2/issues/5714) 建议（Jetson 场景）。**Humble 不支持** |

Humble 上还有两个版本特性值得知道：
- **Eigen 重写不存在于 Humble**（Humble 用 xtensor）。维护者在 #5714 提到切到基于 Eigen 的发行版能拿到 30%+（Jetson）/约 50%（x86）的性能提升。
- `reset_period`（默认 1.0 s）**只在 Humble 存在**（官方 README 标注 "only in Humble due to backport ABI policies"），语义是「required time of inactivity to reset optimizer」。控制器被闲置超过这个时间会重置优化器，重启后第一帧的轨迹是重新采样的——低速精细接近场景里，这会造成「停顿一下再重新规划」的观感。想减轻可以把 `reset_period` 调大（如 5.0），但会延长从异常状态恢复的时间。

---

## 2. 核心参数的物理与数学含义

### 2.1 MPPI 的实际计算流程（Humble 版，逐步对应源码）

Humble 每个控制周期的 `evalControl()` 做这些事（[optimizer.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp)）：

1. `setNoisedControls`：对上一周期的控制序列加高斯噪声 → `cvx = vx + N(0, vx_std²)`，`cwz = wz + N(0, wz_std²)`（`cvy` 仅 Omni 模型）
2. `predict`：每个时间步的 `vx`/`wz` 直接取自对应的带噪控制（Humble 的 `MotionModel::predict` 只是移位赋值，**没有任何加速度限幅**）
3. `integrateStateVelocities`：按运动模型积分成 `batch_size` 条候选轨迹
4. 各 critic 把自己的代价累加到 `costs_`（长度 = `batch_size` 的向量）
5. `updateControlSequence()`：加控制代价 → 归一化 → softmax 加权 → 得到新的控制序列
6. `fallback` 循环：若 `critics_data_.fail_flag`（所有候选轨迹都碰撞）为真，则 `reset()` 并重试，超过 `retry_attempt_limit` 抛异常
7. `savitskyGolayFilter`：对控制序列做 9 点二次 Savitzky-Golay 滤波（[utils.hpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp#L451-L458)）；`time_steps < 20` 时直接跳过
8. `shiftControlSequence`：若 `model_dt == 1/controller_frequency`，把控制序列左移一格（滚动时域复用）

### 2.2 `temperature`（softmax 的温度，即论文里的 λ）

**代码事实**（[optimizer.cpp#L377-L382](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L377-L382)）：

```cpp
auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
control_sequence_.vx = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
```

即权重 `w_i = exp(−ΔC_i / T) / Σ_j exp(−ΔC_j / T)`，`T = temperature`，`ΔC_i = costs_i − min(costs)`（成本下移，对应论文中的 ρ 平移，防止指数下溢）。

**与原始论文的术语对应（容易搞混，必须说清）**：Williams 等人的 *Information Theoretic MPC* 论文里这个量叫 **λ，论文称其为「inverse temperature」（逆温度）**，权重形式是 `w ∝ exp(−S(V)/λ)` ——注意 λ 同样在**分母**上（原文式 (14)(22)，[arXiv:1707.02342](https://arxiv.org/pdf/1707.02342.pdf) §III-A、§III-D-2）。因此：

$$\texttt{temperature}_{\text{Nav2}} \;\equiv\; \lambda_{\text{论文}}$$

**名称上叫「逆温度」，位置上却是分母**，这就是社区里两套说法互相矛盾的原因。用一句可操作的话记住：**Nav2 的 `temperature` 就是论文里的 λ，改它就是改 λ。**

论文图 3 的说明是「Low values of λ result in many trajectories being rejected, high values of λ take close to an un-weighted average」，与官方文档对 `temperature` 的描述完全一致：「Selectiveness of trajectories by their costs (The closer this value to 0, the 'more' we take in consideration controls with less cost), **0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration**」。

**λ（temperature）的绝对数值必须有代价尺度配套**：λ 只有相对于代价的量级才有意义。一份同行评审论文（Appl. Sci. 2025, 15, 9114，四旋翼 MPPI）用 `λ = 10³`，而代价 J 的量级是几百——比值与 Nav2 的 `0.3 / (几米量级代价)` 属于同一区间（[MDPI 论文](https://www.mdpi.com/2079-8956/14/3/228/pdf)）。所以**看到别人写 `temperature: 1000` 不要照抄**：那是代价函数被归一化到完全不同量级的结果。反过来，把 Nav2 的 `temperature` 设到社区示例里的 `0.01`（如 [issue #4849](https://github.com/ros-navigation/navigation2/issues/4849)）意味着权重几乎完全压到单条最小代价轨迹上，等价于退化成 argmin，会失去 MPPI 的平滑性——论文明确指出 λ 过小时「importance sampling **oscillates between solutions instead of converging**」。

**因果链**：

| 调整 | 机制 | 可观察后果 |
|---|---|---|
| `temperature` ↓ (0.3 → 0.05～0.1) | 权重分布更尖，接近 argmin | 更激进、更贴最优轨迹；但候选池随机性被放大成「硬选择」，相邻周期选到不同轨迹族 → **抖动**；论文指出 λ 过小会「oscillates between solutions instead of converging」。极端值（如 #4849 里的 0.01）实际等于退化为「每周期取最小代价样本」，失去平滑性 |
| `temperature` ↑ (0.3 → 1.0～2.0) | 权重趋平，加权平均 | 运动变平滑、鲁棒，但**对代价不敏感**：避障反应迟钝、终点精度差、速度爬升慢。极端值等于「所有轨迹的平均」，控制器近乎失去优化能力 |

**第三方实测数据**：Open Navigation（Nav2 维护者所在公司）在一篇高惯性平台调参长文里，把 `temperature` 增大当作抑制噪声的候选手段，结论是「**Increasing temperature does successfully reduce the magnitude of the noise, but may be slower to react to dynamic obstacles or changes. Additionally, the noise spikes are quite significant and change sign frequently in large swings**」，并明确评价这个方法「we don't think this is what anyone would want」——它本质上是靠更多样本平均来压噪声（[Improving MPPI for High-Inertia Industrial Vehicles](https://opennav.org/news/mppi-low-acceleration/)）。同一篇里他们试过**直接删掉重要性采样项**（把 `gamma` 项注释掉）和**只调 `gamma`**，结论是「Manipulating gamma did not seem to have a great impact either, so we'll skip that analysis.」——这条是社区/厂商层面关于「`gamma` 对噪声治理基本无效」的实证，与 §2.3 的数学分析一致（`gamma` 约束的是「偏离上一周期控制序列」，不是「每周期内的随机散布」）。

**默认值 0.3 是官方调好的**；社区长文（robotcopper）的建议也是「Can be left at 0.3」。只有在「明确知道要更激进/更保守」时才动它，且应把 `temperature` 与 critic 权重的**量级**联系起来看：critic 代价是距离的线性量级（几米以内），`cost_weight` 改动 2～3 倍与 `temperature` 改动 2～3 倍在很多场景下效果相似，所以**不要同时动两者**。

### 2.3 `gamma`（控制代价系数，不是折扣因子）

**先纠一个常见误解**：`gamma` **不是**强化学习里的时序折扣因子 γ，也**不是**按时间步衰减未来代价的系数——Nav2 的代价累加里没有任何 `γ^t`。它的真实身份是**控制代价项的乘子**，来自论文式 (22) 的 `γ = λ(1 − α)`。

**代码事实**（[optimizer.cpp#L360-L374](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L360-L374)）：

```cpp
auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
costs_ += gamma / powf(sampling_std.vx, 2) *
          xt::sum(control_sequence_.vx * bounded_noises_vx, 1);
// wz、vy 同理
```

**这是二次型控制代价**：`γ · Σ_t u_t · (u_t^noisy − u_t) / σ²`。对照论文公式 `w(V) = exp(−(1/λ)S(V) + γ Σ_t û_tᵀ Σ⁻¹ v_t)`（式 22），Nav2 的 `gamma` 就是那个 **γ**，定义是

$$\gamma = \lambda(1-\alpha)$$

其中 α 刻画「新基准分布在多大概率上沿用上一周期的控制序列」。论文原文：「**A value in-between zero and one balances the two requirements of low energy and smoothness**」，且「with α = 1... which keeps U near the distribution corresponding to Û... **as placing a cost on how much the new open loop control law is allowed to deviate from the previous one, which is useful for creating smooth motions**」（§III-D-2）。

所以：
- **`gamma` ↑** → 偏离上一周期控制序列的代价更大 → 运动更**平滑**、更保守 → 但也更「迟钝」，对突发障碍反应慢，且起步/转向更「黏」。
- **`gamma` ↓** → 允许控制量大幅跳变 → 更灵活、响应快 → 抖动风险上升。
- `1/σ²` 的加权意味着：**`gamma` 与 `vx_std`/`wz_std` 强耦合**。把 `vx_std` 从 0.2 提到 0.5，控制代价系数会掉到原来的 1/6.25，等效于把 `gamma` 从 0.015 降到 0.0024——这就是为什么「提高 std 让速度上去了，但抖动也来了」。想保持平滑度不变，提高 std 时应同步放大 `gamma`。

**默认值 0.015**。官方 README 有一处文本自相矛盾（「likely won't need to be changed from the default of `0.1`」但表格里 Default 写 0.015），**以源码为准：0.015**（[optimizer.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L74)）。当前文档已删掉「0.1」这个残留。

### 2.4 `batch_size`、`time_steps`、`iteration_count`、`model_dt`

| 参数 | 含义 | 调大 | 调小 |
|---|---|---|---|
| `batch_size`（默认 1000） | 每周期采样的候选轨迹条数 N | 蒙特卡洛估计方差 ∝ 1/N 下降，速度/避障决策更稳；算力线性上升 | 算力下降；N 太小时**最优解可能根本没被采到**，表现为「行为时好时坏、偶发卡死」 |
| `time_steps`（默认 56） | 每条轨迹的预测点数 T | 预测时域 *= 线性增加 → 提前减速、提前转弯；算力线性上升；官方提示「成本在整个时域上累加」 | 能更早发现靠近的障碍；但转弯/制动预判变短，高速下容易冲过头 |
| `iteration_count`（默认 1） | 同一批样本被重复加权的次数 | 官方明确「Recommend to keep as 1 and prefer more batches」 | — |
| `model_dt`（默认 0.05） | 单步时长 | **必须等于 1/controller_frequency**（§0.5）；官方：「you may also set it **lower but not larger**」 | 预测时域 = T×dt 缩短；同时 `time_steps × model_dt` 应 ≈ `prune_distance / vx_max` |

**注意**：`iteration_count > 1` 时，由于 `regenerate_noises: false`（默认），同一批噪声被重复使用；`optimize()` 的循环体是 `generateNoisedTrajectories(); evalTrajectoriesScores(); updateControlSequence();`，多次迭代只是用同一批样本反复更新控制序列，收益远低于增加 `batch_size`，所以官方的建议是有道理的。

**成本累积的时域特性**：各 critic 默认 `cost_power: 1`，且大多对整条轨迹取 `xt::mean(...)` 或积分（如 `PathFollow` 用轨迹**末端点**到目标路径点的距离，`PathAlign` 用整条轨迹与路径的积分偏差）。这意味着**提高 `time_steps` 会同时放大基于「全程」的 critic（PathAlign、CostCritic、ObstaclesCritic）的影响权重**（因为累加点数变多），而基于末端点的 critic（PathFollow、GoalCritic）不受影响。这是「改了 `time_steps` 之后行为全变」的原因。

### 2.5 `vx_std` / `vy_std` / `wz_std`

三个作用同时存在：

1. **采样散布**：`u_noisy = u_nominal + N(0, σ²)`（[noise_generator.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/noise_generator.cpp#L111-L120)）
2. **控制代价系数**：`gamma / σ²`（§2.3）
3. **有效探索半径**：`u_noisy` 会被 clip 到 `[vx_min, vx_max]` / `[−wz_max, wz_max]`，所以 σ 决定「从当前速度出发能跳到多宽的速度区间」

**因果链**：

| | 调大 | 调小 |
|---|---|---|
| 探索 | 能采到更高速度、更大角速度 → **能跑到 `vx_max`**、能做大转向 | 速度被锁在标称值附近 → 达不到 `vx_max`、转不动 |
| 平滑度 | 控制代价项被削弱 → 抖动、chatter | 过度平滑 → 反应迟钝、陷入局部最优 |
| 特殊约束 | `vy_std` 必须为 0（差动） | `wz_std` 在 Humble **不能为 0**（NaN）；`vy_std: 0` 在 Omni 上会失横移能力 |

**官方两条判据**（现在文档与 README 都有）：
> Tune std's carefully for low acceleration. If you're seeing a lot of **chatter in the angular velocity, reduce its std**. If you're **not seeing your robot get to full speed, increase your std** to explore more of the space.

**可执行的起点**（我的建议，非官方）：以 `vx_std ≈ 0.2 × vx_max`~`0.5 × vx_max` 为初值区间，`wz_std ≈ 0.4`（不要超过 `0.5 × wz_max`）。即 0.5 m/s → 0.2～0.3；1.0 m/s → 0.3～0.5。

**重要反例：低加速度平台要把 std 调小，不是调大**。Open Navigation 给出的定量推导很值得抄一遍（[原文](https://opennav.org/news/mppi-low-acceleration/)）：

> 我们的 `STD` 是 0.2（线速度与角速度）。`1σ` 就是每个时间步 0.2 m/s 和 0.2 rad/s。如果时间步是 0.1 s，而加速度限制是 0.25 m/s²，那么每步实际上只允许变化 0.025 m/s —— 比我们采出来的噪声值小得多。要让 `STD = 0.2` 说得通，需要机器人能在单个时间步内加速约 0.2 m/s，也就是 2 m/s²。这对普通 AMR 完全合理，**这正是 Nav2 默认值如此设定的原因**。

所以他们把 `vx_std` / `wz_std` **从 0.2 降到 0.1**，结果是「a pretty big reduction in noise while still exploring far more than we can realistically achieve」，稳态角速度噪声进入 ±0.01 rad/s 量级，并给出一条结论：「**Reducing the STD is a complete and legitimate solution to the noise increase from using low acceleration with unclamped controls.**」

**判据（我的归纳）**：让 `1σ` 在单个时间步内产生的速度变化，与底盘真实加速度能力同量级，即

$$\sigma \;\approx\; a_{\max} \times \texttt{model\_dt}$$

低加速平台（`a_max ≈ 0.25 m/s²`、`model_dt = 0.1`）→ `σ ≈ 0.025～0.1`；普通 AMR（`a_max ≈ 2 m/s²`、`model_dt = 0.05`）→ `σ ≈ 0.1～0.2`，与 Nav2 默认 0.2 吻合；高速平台（`vx_max ≥ 1 m/s`）才需要把 σ 推到 0.3 以上。**这条判据非官方，但它同时解释了「默认值为什么是 0.2」和「为什么低加速机器人该往小调」，实用性较高。**

⚠️ 冲突提醒：官方 README 说「速度上不去就增大 std」，OpenNav 说「噪声大就减小 std」。两者不矛盾——前者针对**探索空间不足**，后者针对**探索空间远大于执行能力**。判断方法：看 `/trajectories` 上候选轨迹的上沿速度。上沿明显低于 `vx_max` → 增大；上沿远高于底盘实际能做到的加速度 → 减小。

⚠️ 补充：OpenNav 给出的**物理解释**（「the robot is exploring mostly physically unreachable states, adding noise without information」）只是降 σ 有效的**其中一个原因**。另一个原因是 §2.3 的数学耦合——控制代价系数是 `gamma / σ²`，σ 减半等于把控制代价项放大 4 倍。两个效应叠加，所以降 σ 的降噪效果比单纯看「探索范围」所预期的更明显。**同时也意味着：降 σ 等价于隐式放大了 `gamma`**，如果原本 `gamma` 就偏大（机器人迟钝），降 σ 会让迟钝更明显。

### 2.6 `consider_footprint`

- `false`（默认）：只用机器人中心点的代价，机器人被近似为质点/圆。
- `true`：用 `footprintCostAtPose` 做 SE2 多边形检查。

**影响**：
- 精度/安全：`true` 能发现「中心在自由空间但角点已经撞了」的情况；对非圆形（矩形、叉车、移动机械臂）**必须开**。
- 算力：明显上升。维护者在 [issue #4057](https://github.com/ros-navigation/navigation2/issues/4057) 说逐点代价查询是整个控制器里最慢的单项操作，是「why you can't run this at 50,60,80 hz on older CPUs」的主因。
- 与 `ObstaclesCritic` 的交互（Humble）：开 footprint 后 `INSCRIBED_INFLATED_OBSTACLE` 不再算碰撞（[obstacles_critic.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/obstacles_critic.cpp#L204-L212)），但反算距离的公式仍然是按内切半径写的 → 距离估计失准（§1.3）。

### 2.7 `prune_distance` 与 `max_robot_pose_search_dist`

| 参数 | 默认 | 含义 |
|---|---|---|
| `prune_distance` | 1.5 | 以机器人最近路径点为起点，**只保留前方这么长的局部路径**（官方：「Distance ahead of nearest point on path to robot to prune path to」） |
| `max_robot_pose_search_dist` | 代价地图半尺寸 | 在整条全局路径上，最多往前搜索多远去找「离机器人最近的路径点」。默认取代价地图半宽，用于防止路径成环时匹配到错误分支 |

**因果链**：`prune_distance` ↓ → 局部路径短 → `PathFollow` 的 carrot 近 → **速度上不去**、到终点前的减速点变近、换向点判断变早；`prune_distance` ↑ → 速度上限放开、路径跟踪更稳，但每条轨迹要匹配更长的路径（算力线性上升），且局部路径超出代价地图的部分无法评估（被 threshold 掉）。

---

## 3. 调参顺序与耦合关系

### 3.1 顺序（严格按阶段，每阶段只调一类）

**阶段 0：把不可调的东西固定下来**
1. `motion_model` 必须与底盘一致（`DiffDrive` / `Omni` / `Ackermann`）
2. `controller_frequency` 定在 20～30 Hz；`model_dt = 1/controller_frequency`（§0.5）
3. `vx_max` / `vx_min` / `vy_max` / `wz_max` = 底盘真实极限 × 0.8
4. 检查管线限幅：`velocity_smoother.max_velocity` / `min_velocity`、`diff_drive_controller` 的 `max_velocity`（§1.6 ①②）
5. 局部代价地图：`width/height ≥ 2 × vx_max × time_steps × model_dt`，`resolution ≤ 0.05`，`inflation_radius` 略大于内切半径，`cost_scaling_factor` 与（Humble 的）`ObstaclesCritic` 保持一致
6. 目标容差：`goal_checker.xy_goal_tolerance`（0.1～0.25）、`yaw_goal_tolerance`（0.1～0.25）——**先定容差再调 critic**（社区长文：「you must first be satisfied with the GoalChecker's yaw_goal_tolerance params」）

**阶段 1：预测时域与采样（决定「能力上限」）**
7. `time_steps`：`time_steps × model_dt ≈ 2～3 s`。先不动
8. `batch_size`：算力允许就 2000（官方维护者建议「30hz with 2000 samples」）
9. `prune_distance`：`≥ vx_max × time_steps × model_dt × 0.3`，且 `≤` 代价地图半宽
10. **`vx_std` / `wz_std`：按目标速度设定**（§2.5）。这是决定「能不能跑起来」的一步，做完再谈别的

**阶段 2：避障行为**
11. 先只保留一个障碍 critic（Humble 建议 `CostCritic`），把其余 critic 的权重设 0

> **⚠️ 这里有一个必须避开的陷阱**：维护者明确说过 MPPI **没有内建行为**，「There is no objective functions 'built-in', you have to specify each to get each element of the total behavior.」他还直接回复过「只留一个 critic」的做法：「Its not possible to run the controller with just that single critic. **Nothing is driving the robot forward (PathFollow)**, among other things. ... In effect, no critics are currently being evaluated.」（[Robotics SE #105468](https://robotics.stackexchange.com/questions/105468/)）
>
> 所以「把其余 critic 权重设 0」只能用于**诊断**（确认某个 critic 是否在捣乱），而且要**保留能驱动前进的最小集合**。可用的最小集合（按运动模型，社区长文整理，非官方）：
> - DiffDrive / Ackermann：`GoalCritic`、`GoalAngleCritic`、`ObstaclesCritic`(或 `CostCritic`)、`PathAngleCritic`、`PathFollowCritic`、`PreferForwardCritic`
> - Omni：`GoalCritic`、`GoalAngleCritic`、`ObstaclesCritic`、`TwirlingCritic`、`PathFollowCritic`、`PreferForwardCritic`
>
> 注意**两个集合里都没有 `PathAlignCritic`**——按维护者 #4376 的说法它是「做全部主要工作」的那个，所以这个「最小集合」更应理解为「缺了会立即失效的那几个」，而不是「推荐配置」。**`ConstraintCritic` 必须保留**，它负责把速度/加速度压回约束内。

12. 调 `CostCritic.cost_weight`（3.0～6.0）与 `critical_cost`（300～600），配合代价地图膨胀参数，直到「在通道里居中、贴近障碍时会减速绕开」；官方原话的验收标准是「smooth motion roughly in the center of spaces without significant close interactions with obstacles」
13. 若「不敢进入代价空间」，按 §1.4 成对调整（障碍权重 ↓ / PathFollow ↑）

**阶段 3：路径跟踪**
14. 调 `PathAlignCritic.cost_weight`（10～20）与 `offset_from_furthest`（官方公式）
15. 再调 `PathAngleCritic.cost_weight`（2～5）与 `max_angle_to_furthest`（0.6～1.2）
16. 按需 `PathFollowCritic.cost_weight`（5～15）与 `offset_from_furthest`（跟随「离机器人多远的 carrot」）

**阶段 4：终点收敛**
17. 用路径类 critic 的 `threshold_to_consider` 定义交接带宽度
18. 调 `GoalCritic` / `GoalAngleCritic` 的权重（5→8～10，3→5～8）
19. 终点朝向做不好就上 `RotationShimController.rotate_to_goal_heading`

**阶段 5：最后才碰的两个参数**
20. `temperature`（0.2～0.5）
21. `gamma`（0.01～0.03）

### 3.2 不能独立调的耦合对

| 耦合对 | 关系 |
|---|---|
| `vx_std` ↔ `gamma` | 控制代价系数 = `gamma / std²`。改 std 必须按平方率同步考虑 gamma |
| `vx_std` ↔ `wz_std` ↔ 抖动 | 提 std 换速度，代价是 chatter。两者要分别定：先按速度定 `vx_std`，再按「角速度 chatter 是否可接受」压 `wz_std` |
| `model_dt` ↔ `controller_frequency` | 必须严格相等，否则报运行时异常（§0.5） |
| `model_dt` ↔ `time_steps` ↔ `prune_distance` ↔ `vx_max` | `time_steps × model_dt × vx_max` 决定预测距离；`prune_distance` 与代价地图半径必须容纳它 |
| `CostCritic.cost_weight` ↔ `PathAlignCritic.cost_weight` | 一个要「离障碍远」，一个要「贴路径」。同时调高 = 互相抵消，表现为机器人抖动或卡住 |
| 障碍权重 ↔ `inflation_radius` / `cost_scaling_factor` | Humble 的 `ObstaclesCritic` 惩罚形如 `(inflation_radius − dist)`，膨胀半径变大必须把 `repulsion_weight` 调小（官方） |
| 路径类 `threshold_to_consider` ↔ 目标类 `threshold_to_consider` | 两者定义交接带的两个边界，应设成相同值以获得「clean hand-offs」（官方） |
| `offset_from_furthest` ↔ 路径分辨率 ↔ 代价地图半径 | 同一个数值在不同路径分辨率下含义完全不同（官方给了公式） |
| `vx_min` ↔ `PathAngleCritic` 倒车 | `vx_min = 0` 会让 Humble 的 PathAngleCritic 强制 `forward_preference = true`（§0.3） |
| `wz_std` ↔ `prefer_forward` / `PreferForwardCritic` | `wz_std` 太小则采不到原地转向的轨迹，这时把 PreferForward 调低也没用（[issue #4049](https://github.com/ros-navigation/navigation2/issues/4049) 提问者的猜测是「MPPI is not sampling those pure spin turn trajectories in the first place」，方向是对的） |

---

## 4. 调试手段

### 4.1 Humble 可用的可视化话题（已核实）

| 话题 | 类型 | 触发条件 |
|---|---|---|
| `/trajectories` | `visualization_msgs/MarkerArray` | `visualize: true`。包含全部候选轨迹 + 最优轨迹 |
| `transformed_global_plan` | `nav_msgs/Path` | 同上。这是 MPPI 实际跟踪的那段局部路径（含 `prune_distance` 之后的结果） |

【源码】来自 [trajectory_visualizer.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/trajectory_visualizer.cpp#L29-L30)。注意 Humble **没有** `/critics_stats`、`publish_critics_stats`、`critic_index_to_visualize`、`publish_optimal_trajectory` 这些后续版本才加的调试输出——这些是 Jazzy/主分支的能力。

### 4.2 在 RViz 里看候选轨迹

1. 先降可视化密度，避免刷屏：`TrajectoryVisualizer.trajectory_step: 100`、`time_step: 100`（社区经验，只保留最优轨迹）；确认行为后再降到 10/5 看整个样本云。
2. 加 `MarkerArray` 显示 `/trajectories`，注意 Marker 的 namespace 是 `"Candidate Trajectories"` 与 `"Optimal Trajectory"`，可以按 namespace 分开勾选。
3. 加 `Path` 显示 `transformed_global_plan`——**这一步非常关键**：很多「不贴路径」问题其实是 MPPI 拿到的局部路径本身就短/被剪断/被 `max_robot_pose_search_dist` 匹配到了错误分支，看这条话题能立刻区分「critic 调不好」和「输入路径就有问题」。
4. 注意 `visualize: true` 会显著增加算力，官方明确不建议在部署时开启。

**看什么**：
- 样本云是**全绿**（低代价）还是**大部分红/品红**？全红说明在约束边界上，需要看是不是 `min_x_velocity_threshold` 或 costmap 有问题。
- 最优轨迹与路径的夹角——如果最优轨迹明显偏离 `transformed_global_plan`，是 PathAlign 不够或障碍项过大。
- 候选轨迹在**目标速度附近有没有样本**？如果样本云的上沿远低于 `vx_max`，立即按 §1.6③ 提 `vx_std`，不要再调 critic。
- 相邻两帧的最优轨迹是否在跳变（锯齿）→ 抖动，按 §1.1 处理。

### 4.3 用 `ros2 param set` 在线试参（确认可用）

MPPI 在 Humble 实现了动态参数回调（[parameters_handler.hpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/tools/parameters_handler.hpp#L60-L71)），且注册了 post 回调 `addPostCallback([this]() {reset();})`（[optimizer.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp#L86)），所以：

```bash
# 动态改 critic 权重：立即生效
ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 18.0
ros2 param set /controller_server FollowPath.vx_std 0.35
ros2 param set /controller_server FollowPath.wz_std 0.25
ros2 param set /controller_server FollowPath.ObstaclesCritic.repulsion_weight 0.9

# 查看当前值 / 全部 MPPI 参数
ros2 param get /controller_server FollowPath.vx_std
ros2 param dump /controller_server | sed -n '/FollowPath/,/^[^ ]/p'

# 持久化：把试出来的值写回 yaml，重启后生效
ros2 param dump /controller_server > /tmp/controller_server.yaml
```

**注意事项**：
- 每次 `ros2 param set` 都会触发 `reset()`（优化器状态清零、噪声重采样），所以**会在设置瞬间产生一次轨迹重置**，看到一帧异常指令是正常的。
- 参数层级必须完整：`FollowPath.<CriticName>.<param>`。critic 实例名就是 `critics:` 列表里的字符串（如 `"PathAlignCritic"`）。
- **不是所有参数都能动态改**：`controller_frequency` 是通过 `getParentParam(..., ParameterType::Static)` 读取的静态参数，`model_dt` 变了会重新走 `setOffset()` 并可能抛异常。这两类参数请改 yaml 后重启。
- 维护者对一份参数「看起来写了但可能没生效」的配置有过明确警告：如果 critic 名前缀或层级不对（例如用 `mppi_critics/ObstaclesCritic` 这类自定义名），参数根本不会被读取，表现就是「怎么调都没变化」（[issue #4049](https://github.com/ros-navigation/navigation2/issues/4049)）。**调参前先确认参数真的被读到**——最可靠的办法是看 controller_server 启动日志里的 `RCLCPP_INFO`，例如 `"GoalCritic instantiated with %d power and %f weight."`、`"PathAngleCritic instantiated with ... Reversing %s"`（各 critic 的 `initialize()` 都会打印自己的权重与 power）。这是**最快确认配置生效的手段**。

### 4.4 其他日志/度量手段

- **控制周期耗时**：Humble 的 `controller.cpp` 里有被注释掉的 `BENCHMARK_TESTING` 宏，打开后每周期打印 `"Control loop execution time: %ld [ms]"`（[controller.cpp @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/controller.cpp#L84-L110)）。这是判断「控制器算得过来吗」的最直接方式：耗时必须显著小于 `1/controller_frequency`。
- **`/cmd_vel` 链路对比**：同时 `ros2 topic echo` 三个话题——`/cmd_vel_nav`（controller_server 输出）、`/cmd_vel_smoothed`（平滑器输出）、底盘实际收到的话题。§1.6 的两个环境原因都靠这招定位。
- **`/odom` 与 `cmd_vel` 的对比**：官方 README 提到「your odometry publishes at least as fast as your control frequency (ideally much faster) when using low accelerations」——里程计频率低于控制频率会直接引入相位滞后，表现为画龙。
- **失败路径**：`optimizer.cpp` 的 `fallback()` 在反复失败后抛 `"Optimizer fail to compute path"`，会打印 `/trajectories` 上品红色的碰撞轨迹；如果看到「机器人在终点附近反复 backup」（如 [issue #5375](https://github.com/ros-navigation/navigation2/issues/5375) 的录像），先去查 `progress_checker`（`required_movement_radius` / `movement_time_allowance`）和 `failure_tolerance`——超时触发的 backup 是行为树恢复行为，不是 MPPI 在倒车。
- **参数扫描/自动调参**（可选）：有研究者用贝叶斯优化在 Isaac Sim 里做 MPPI 的 sim-to-real 超参数搜索（[OmniMPPI](https://proximityrobotics.github.io/OmniMPPI/)）。该项目页只说明优化了 **batch size / control noise / temperature** 三类量，**未公开搜索范围、最优值与敏感性排序**，因此无法从中提取可复制的数值，仅作为方法参考。

### 4.5 一个可量化的「样本是否够用」指标：有效样本数 ESS

MPPI 的权重是 softmax，`temperature` 过小或代价尺度失衡时，权重会集中到极少数样本上，此时「batch_size = 2000」是虚假的充分性。有效样本数定义为

$$\mathrm{ESS} = \frac{1}{\sum_i w_i^2}$$

其中 `w_i` 就是 §2.2 的 softmax 权重。经验判据是 **ESS 应落在 `[0.1K, 0.5K]`**（K = `batch_size`）：低于 0.1K 说明优化器实际上只在少数几条轨迹间做硬选择，抖动与「偶发卡死」都会增加，此时应**增大 `temperature` 或减小 critic 权重**；高于 0.5K 说明权重过于平坦、优化器失去了选择性，应**减小 `temperature`**。（该指标与区间出自中文 MPPI 理论教程，属**社区经验，非 Nav2 官方**；Nav2 本体不发布 ESS。第三方实现里有 `target_ess_ratio: 0.5` 这类配置，与该上界独立吻合。）

**怎么在 Nav2 里近似观察**：Humble 没有 `/critics_stats`，但可以打开 `BENCHMARK_TESTING` 之外的办法——在 RViz 里看 `/trajectories` 的**候选轨迹分布**：如果最优轨迹与样本云中最靠近它的几条几乎重合、其余样本全部远离，说明权重高度集中（低 ESS）的不适感；如果样本云大面积接近最优轨迹，则 ESS 偏高。要精确算只能改代码或自行订阅 `/trajectories` 复算。

---

## 5. 未验证 / 存疑清单（使用时请注意）

1. **「提高 `vx_std` 就能达到 `vx_max`」的具体倍数关系没有官方定量指引**。官方只给了定性方向。我给出的 `vx_std ≈ 0.2~0.5 × vx_max` 属于经验建议，需要按机型验证。
2. **`prune_distance ≥ vx_max × time_steps × model_dt × 0.3`** 出自社区长文（robotcopper），非官方；官方只说「in proportion to your maximum velocity and prediction horizon」。
3. **[issue #5375](https://github.com/ros-navigation/navigation2/issues/5375)（差动机器人近距离机动）在 issue 内没有得到参数级解决方案**，维护者明确表示不在 issue tracker 提供调参支持。后续有用户报告同样问题但无解。把它当成「已知难点」而不是「有配方」。
4. **[Robotics SE #113238](https://robotics.stackexchange.com/questions/113238/nav2-mppi-controller-tuning-help) 与 [#117814](https://robotics.stackexchange.com/questions/117814) 至今 0 个回答**（我通过 Stack Exchange API 核实）。其中 #117814 的配置（`vx_max: 6.0`、`vx_std: 0.8`、`time_steps: 40`、Jazzy）恰好把「提 std 换速度」做到了位却仍卡在 0.7 m/s，说明还存在别的限制因素（本地代价地图 50×50 m 已排除尺寸问题，但**未查到最终原因**）。
5. **`PathAngleCritic` 在 Humble 的 `forward_preference` 语义**：源码里 `reversing_allowed_` 由父级 `vx_min` 决定，但 `forward_preference = false` 且允许倒车时的具体打分逻辑（对轨迹反向时的角度修正）我只读了代码，未找到文档化的行为描述。
6. **PR #5110 是否已 backport 到 Humble 发行版二进制**：我在 humble 分支源码里确认了**没有** `wz_std > 0` 守卫，因此从源码判断 Humble 仍然有 NaN 风险；但我没有核实 apt 二进制包的具体补丁级别，**建议以「不要设 `wz_std: 0`」为准，无论哪个版本**。
7. **ROSCon 2023 "On Use of Nav2 MPPI Controller" 演讲**（[slides 直链](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf)，[录像](https://vimeo.com/879001391)）：本汇编引用了其中「Tuned: 30Hz @ 2000, 50Hz @ 1000」与「Costmap Smooth Inflation Critical!」两条，均在幻灯片文本中直接核实。**该 PDF 直链在常规抓取下会返回异常响应，需要分段获取**；如果读者拿不到完整 PDF，以录像为准。
8. **`ObstaclesCritic` 距离反算误差（[#5590](https://github.com/ros-navigation/navigation2/issues/5590)）** 是报告性质，我没有核实是否已修复或计划修复。
9. **ESS 指标与 `[0.1K, 0.5K]` 区间** 出自中文 MPPI 理论教程与第三方实现，**不是 Nav2 官方指标**，Nav2 本体不发布它。
10. **OmniMPPI（贝叶斯优化 MPPI 超参）** 论文在 IEEE 付费墙后（document 11163854），arXiv 无预印本；项目页仅说明优化了 batch size / control noise / temperature，**搜索范围、最优值、敏感性排序均未验证**。
11. **「其他团队的开源参数文件」这一项收获有限**：我尝试的若干候选路径全部 404（`ROBOTIS-GIT/turtlebot3` 的 `turtlebot3_navigation2/param/humble/*.yaml`、`ROBOTIS-GIT/turtlebot4` 的 `turtlebot4_navigation/config|params/nav2*.yaml`、`clearpathrobotics/clearpath_navigation` 的 nav2 配置），GitHub 的代码搜索需要登录、搜索 API 在本环境受限额约束。因此本汇编**没有收录任何「真实项目 MPPI 参数文件」的完整对照**，取而代之的是：Nav2 官方默认值（Humble bringup 与 MPPI README 源码默认）、issue/PR 里用户公开贴出的完整配置（#5375、#4849、#4970、#4049、#5531、#6160、#6185 等），以及 OpenNav 的实测参数。**如需真实项目参数对照，建议直接用 GitHub 网页代码搜索 `MPPIController` + `vx_std` 手工收集，这部分我未能完成。**
12. 本文所有「典型数值起点」中，凡标注【社区】或我明确写成「我的建议」的，都不来自官方文档，需要按机型实测确认。

---

## 6. 一页速查

```
先验证环境（否则白调）：
  velocity_smoother.max_velocity / min_velocity 是否 ≥ MPPI 的 vx_max / wz_max
  diff_drive_controller 的 max_velocity 是否够
  controller_frequency 在 20~30 Hz，model_dt == 1/controller_frequency
  局部代价地图半径 ≥ vx_max * time_steps * model_dt
  启动日志里每个 critic 的 "instantiated with ... weight" 是否与配置一致

症状 → 首选参数（Humble）
  抖动/chatter          → wz_std ↓ (0.4→0.25→0.15)；障碍权重 ↓
  画龙/不贴路径         → PathAlign.cost_weight ↑ (14→18)；offset_from_furthest 按公式
  不避障               → CostCritic.cost_weight ↑ (3.81→5)；critical_cost ↑
  不敢动/绕障保守       → 障碍权重 ↓ / PathFollow.cost_weight ↑ (5→10)，成对调
  终点转圈/不收敛        → 调 threshold_to_consider 交接带；Goal/GoalAngle 权重 ↑
  速度上不去            → vx_std ↑（主）；prune_distance ↑；查 smoother 限幅
  倒车异常             → PreferForward.cost_weight → 0；确认 vx_min < 0
  原地转不了            → wz_std ≥ 0.05（严禁 0）；或上 RotationShim
  算力不足             → time_steps ↓、batch_size ↓、visualize false、consider_footprint false

永远最后才动：temperature (0.2~0.5)、gamma (0.01~0.03)
```

---

### 主要来源

- Nav2 官方 MPPI 文档（含 Notes to Users）：[docs.nav2.org … configuring_mppic](https://docs.nav2.org/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- MPPI README（Humble 分支，本汇编的「官方」基准）：[nav2_mppi_controller/README.md @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)
- Humble 源码：`optimizer.cpp`、`motion_models.hpp`、`constraints.hpp`、`path_angle_critic.cpp`、`path_align_critic.cpp`、`path_follow_critic.cpp`、`obstacles_critic.cpp`、`trajectory_visualizer.cpp`、`utils.hpp`、`parameters_handler.hpp`
- Humble 默认参数：[nav2_params.yaml @humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
- 关键 issue：[#5375](https://github.com/ros-navigation/navigation2/issues/5375)、[#5376](https://github.com/ros-navigation/navigation2/issues/5376)、[#4376](https://github.com/ros-navigation/navigation2/issues/4376)、[#4849](https://github.com/ros-navigation/navigation2/issues/4849)、[#4970](https://github.com/ros-navigation/navigation2/issues/4970)、[#4049](https://github.com/ros-navigation/navigation2/issues/4049)、[#4057](https://github.com/ros-navigation/navigation2/issues/4057)、[#4425](https://github.com/ros-navigation/navigation2/issues/4425)、[#4509](https://github.com/ros-navigation/navigation2/issues/4509)、[#5021](https://github.com/ros-navigation/navigation2/issues/5021)、[#5386](https://github.com/ros-navigation/navigation2/issues/5386)、[#5464](https://github.com/ros-navigation/navigation2/issues/5464)、[#5531](https://github.com/ros-navigation/navigation2/issues/5531)、[#5590](https://github.com/ros-navigation/navigation2/issues/5590)、[#5661](https://github.com/ros-navigation/navigation2/issues/5661)、[#5694](https://github.com/ros-navigation/navigation2/issues/5694)、[#5714](https://github.com/ros-navigation/navigation2/issues/5714)、[#5928](https://github.com/ros-navigation/navigation2/issues/5928)、[#6160](https://github.com/ros-navigation/navigation2/issues/6160)、[#6185](https://github.com/ros-navigation/navigation2/issues/6185)、[#6357](https://github.com/ros-navigation/navigation2/issues/6357)
- Robotics Stack Exchange：[#113238](https://robotics.stackexchange.com/questions/113238/nav2-mppi-controller-tuning-help)（0 回答）、[#117814](https://robotics.stackexchange.com/questions/117814)（0 回答）、[#114198](https://robotics.stackexchange.com/questions/114198/in-place-rotation-with-mppi-controller-using-turtlebot3)、[#105182](https://robotics.stackexchange.com/questions/105182/appropriate-tuning-of-the-mppi-controller-for-close-goals)、[#113975](https://robotics.stackexchange.com/questions/113975/how-to-achieve-stable-lane-following-for-a-large-non-holonomic-vehicle-in-nav2-w)、[#115479](https://robotics.stackexchange.com/questions/115479/nav2-mppi-standalone-usage-without-path-defined)
- ROS Discourse：[Nav2 MPPIController Tuning Help](https://discourse.openrobotics.org/t/nav2-mppicontroller-tuning-help/39922)（提问被引导至 Robotics SE，无技术回答）
- 论文：Williams et al., *Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving*，[arXiv:1707.02342](https://arxiv.org/pdf/1707.02342.pdf)
- 维护者 ROSCon 2023 演讲《On Use of Nav2 MPPI Controller》：[幻灯片](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf) / [录像](https://vimeo.com/879001391)
- 厂商（Open Navigation）实测长文：[Improving MPPI for High-Inertia Industrial Vehicles](https://opennav.org/news/mppi-low-acceleration/)（低加速平台的 std / model_dt 分析来源；ROS Discourse 讨论：[54403](https://discourse.openrobotics.org/t/nav2-blog-improving-mppi-for-high-interia-industrial-vehicles/54403)）
- 社区长文（我用于交叉验证，非官方）：[NAV2: MPPI Parameters Tuning（Floris Jousselin）](https://robotcopper.github.io/ROS/MPPI.html)
