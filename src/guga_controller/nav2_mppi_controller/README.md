# nav2_mppi_controller

> **调参入口**：`scripts/paramview.sh` 打开 Web 界面，可动态改参数、与合并后的配置基线对比、可直接保存。`scripts/paramview/params_list/mppi_para.txt` 处可添加动态修改参数。

基于采样的局部轨迹控制器：从高斯分布采样控制序列、按插件化评价器（critic）打分并 softmax 加权，
输出最优速度指令。本包是 Nav2 官方 MPPI 控制器的本地 fork，插件名改为
`guga_source_mppi_controller::MPPIController`，由 `controller_server` 的 `FollowPath` 插件加载。

## 架构

主线只有一条：`controller_server` 每个控制周期调用一次 `computeVelocityCommands()` →
`transformPath()` 裁剪路径 → `Optimizer::evalControl()` 跑一次 MPPI → 返回 `TwistStamped`。
详细步骤如下：

1. `src/controller.cpp` — `MPPIController`，实现 `nav2_core::Controller` 接口。
   - 初始化：`configure()` 读 `visualize` / `reset_period`，组装 `Optimizer`、`PathHandler`、
     `ParametersHandler` 与 `TrajectoryVisualizer`；`setPlan()` 把全局路径交给 `PathHandler`。
   - 循环：`computeVelocityCommands()` 是每周期入口，距上次调用超过 `reset_period` 时先复位，再调用优化器。

2. `src/optimizer.cpp` — MPPI 主循环。
   - `prepare()` 更新状态与实测速度、移动控制序列，并生成带噪控制；
   - `optimize()` 迭代 `iteration_count` 次：采样轨迹 → 评价器打分 → softmax 加权更新控制序列；
   - `fallback()` 在无可行轨迹时软复位重试；
   - 结束后做 Savitsky-Golay 平滑，再去除不符合加速度的选项，最后 `getControlFromSequenceAsTwist()`
     取第一个预测序列的第一位（yaml里频率不等于 1/dt）/第零位（二者相等时）作为输出。

3. `src/critics/`、`src/critic_manager.cpp` — 12 个插件式评价器与加载、调度逻辑。

文件列表：

- `src/controller.cpp`：控制器插件入口与生命周期。
- `src/optimizer.cpp`：MPPI 优化主循环、约束与运动模型积分。
- `src/critic_manager.cpp`：按 `critics` 列表插件化加载评价器并依序计分。
- `src/path_handler.cpp`：全局路径裁剪与坐标变换。
- `src/parameters_handler.cpp`：参数声明与动态更新（支持 `ros2 param set`）。
- `src/noise_generator.cpp`：采样噪声生成。
- `src/trajectory_visualizer.cpp`：候选轨迹与最优轨迹可视化。
- `include/nav2_mppi_controller/motion_models.hpp`：DiffDrive / Omni / Ackermann 运动模型。
- `include/nav2_mppi_controller/critics/*.hpp`：各评价器接口。
- `test/*.cpp`：单元测试（见“测试”一节）。

本地 fork 相对上游的行为差异：插件名改为 `guga_source_mppi_controller::MPPIController`（上游名
`nav2_mppi_controller::MPPIController` 在本地不可用）；移植了上游加速度约束（PR #4352 / #5266 / #6072）；
平滑（SGF）之后、下发之前再按加速度包络钳制一次；新增 `debug_timing` 参数输出单周期耗时拆解；新增
`last_command_vel_` 记录上一周期指令，供 odom 低频时的滞后补偿使用。

TES（小陀螺）场景的坐标系适配不在本包：局部代价地图与控制器位姿使用 `base_footprint_nonrotating`，
自旋速度叠加（`cmd_spin` + `chassis_mode`）由 `nonrotating_vel_transform` 在输出 `cmd_vel` 前完成。

## 数据流

```
bt_navigator
  │ 全局路径（setPlan 接口，非话题）
  ▼
controller_server（controller_frequency: 30 Hz）
  │ 位姿（话题） / 速度（话题） / goal_checker
  ▼
MPPIController（FollowPath 插件）
  │ cmd_vel_controller（launch 中 remap cmd_vel → cmd_vel_controller）
  ▼
velocity_smoother（30 Hz）→ cmd_vel_smoothed
  ▼
nonrotating_vel_transform（60 Hz 重发 + 自旋叠加）→ cmd_vel → robot_base

  MPPI 内部：transformPath → evalControl（prepare → optimize × iteration_count
  → fallback 重试 → SGF 平滑 → 约束钳制 → TwistStamped）
  可选输出：trajectories / transformed_global_plan → RViz
```

## 输入

| 节点                | 接口/Topic   | 类型                 | 来源包/节点                                         |
| ------------------- | ------------ | -------------------- | --------------------------------------------------- |
| `MPPIController`    | `setPlan()`  | `nav_msgs/Path`      | `controller_server`（由 bt_navigator 触发）         |
| `controller_server` | `odometry`   | `nav_msgs/Odometry`  | `point_lio`（`odom_topic` 配置项）                  |
| `MPPIController`    | 代价地图接口 | `nav2_costmap_2d`    | `local_costmap`（`Costmap2DROS`）                   |
| `MPPIController`    | `tf`         | `tf2_msgs/TFMessage` | 导航栈（`map → odom → base_footprint_nonrotating`） |

## 输出

| 节点                | Topic                     | 类型                             | 坐标系                       | 下游订阅者          | 用途                                 |
| ------------------- | ------------------------- | -------------------------------- | ---------------------------- | ------------------- | ------------------------------------ |
| `controller_server` | `cmd_vel_controller`      | `geometry_msgs/Twist`            | 机器人本体（nonrotating 系） | `velocity_smoother` | 最优速度指令                         |
| `MPPIController`    | `trajectories`            | `visualization_msgs/MarkerArray` | 代价地图全局系（map）        | RViz                | 候选与最优轨迹（`visualize` 打开时） |
| `MPPIController`    | `transformed_global_plan` | `nav_msgs/Path`                  | 代价地图全局系（map）        | RViz                | 裁剪后的局部路径                     |

`cmd_vel_controller` 是唯一的控制输出，经 `velocity_smoother`（加速度限幅）与
`nonrotating_vel_transform`（旋转补偿 + 自旋叠加）之后才送到串口驱动；后两个可视化话题只在
`visualize: true` 时发布，且会明显增加计算量。

## 评价器

`critics` 列表决定加载哪些插件；列表里的每个 critic 在 `score()` 开头检查自己的 `enabled` 参数，
`enabled: false` 时直接返回、不计分。本地实车配置加载 9 个，实际计分 5 个：

| 插件                     | 作用                                                      | 实车配置（列表 / enabled） |
| ------------------------ | --------------------------------------------------------- | -------------------------- |
| `ConstraintCritic`       | 约束速度与加速度越界                                      | 列表内 / true              |
| `CostCritic`             | 用代价地图代价避障，`consider_footprint` 控制是否按足迹算 | 列表内 / true              |
| `ObstaclesCritic`        | 用“到障碍的距离”反算避障代价，与膨胀层参数耦合            | 不在列表                   |
| `GoalCritic`             | 到目标点的距离代价                                        | 列表内 / true              |
| `GoalAngleCritic`        | 到点后车头朝向代价                                        | 列表内 / false             |
| `PathAlignCritic`        | 不贴合轨迹的代价                                          | 列表内 / true              |
| `PathAlignLegacyCritic`  | 2023-10 之前的旧对齐公式                                  | 不在列表                   |
| `PathFollowCritic`       | 沿路径推进（惩罚偏离路径前端的滞后，鼓励向路线方向前进）  | 列表内 / true              |
| `PathAngleCritic`        | 路径方向角偏好                                            | 列表内 / false             |
| `PreferForwardCritic`    | 偏好前进（抑制倒车）                                      | 列表内 / false             |
| `TwirlingCritic`         | 抑制不必要的旋转：对整条轨迹的平均 \|wz\| 无条件计代价    | 列表内 / false             |
| `VelocityDeadbandCritic` | 低速死区，抑制低速抖动                                    | 不在列表（注释中）         |

实车计分的 5 个是 Constraint、Cost、Goal、PathAlign、PathFollow。`ObstaclesCritic` 的取值与仿真对齐但
未启用：启用前需要把 `inflation_radius` / `cost_scaling_factor` 改成与局部代价地图膨胀层完全一致（详见
`src/guga_bringup/config/reality/controller/mppi.yaml` 的注释）。`TwirlingCritic` 的“抑制转向”是全局旋钮，不是只在原地
打转时生效；调大后车头不易摆动，但也更容易转不动。

## 管线

本节讲解主要函数作用。

| 函数                              | 位置                      | 职责                                                                            |
| --------------------------------- | ------------------------- | ------------------------------------------------------------------------------- |
| `configure`                       | controller.cpp            | 读 `visualize`/`reset_period`，初始化优化器、路径处理与可视化                   |
| `setPlan`                         | controller.cpp            | 把全局路径交给 `PathHandler`                                                    |
| `computeVelocityCommands`         | controller.cpp            | 每周期入口：超时复位 → `transformPath` → `evalControl` → 可选 `visualize`       |
| `transformPath`                   | path_handler.cpp          | 把路径裁剪到代价地图可及范围，并变换到机器人局部系                              |
| `evalControl`                     | optimizer.cpp             | 一次优化的总编排：prepare → optimize 迭代 → fallback 重试 → SGF 平滑 → 约束钳制 |
| `prepare`                         | optimizer.cpp             | 更新位姿与速度、移动控制序列、生成带噪控制                                      |
| `optimize`                        | optimizer.cpp             | `iteration_count` 次采样 → 评分 → softmax 加权，更新控制序列                    |
| `fallback`                        | optimizer.cpp             | 无可行轨迹时软复位并重试                                                        |
| `applyControlSequenceConstraints` | optimizer.cpp             | 按 `ax/ay/az` 与速度上限逐时间步钳制控制序列                                    |
| `getControlFromSequenceAsTwist`   | optimizer.cpp             | 取第一个时间步的控制量，转成 `TwistStamped`                                     |
| `evalTrajectoriesScores`          | critic_manager.cpp        | 依序调用各 critic 的 `score()`；`debug_timing` 时记录每个 critic 的耗时         |
| `add` / `visualize`               | trajectory_visualizer.cpp | `visualize: true` 时发布候选轨迹、最优轨迹与局部路径                            |
| `generateNextNoises`              | noise_generator.cpp       | 生成高斯采样噪声（`regenerate_noises` 决定是否每步重采样）                      |


## 参数

参数来自 `src/guga_bringup/config/<profile>/controller/mppi.yaml` 的 `controller_server` 段与
`FollowPath` 子段。下表是实车（reality）的取值；仿真（simulation）的采样与速度上限不同，见对应文件。
未在本地配置中显式写出的参数取上游默认值。

### 控制器与优化器

| 参数                   | 作用                                                    |
| ---------------------- | ------------------------------------------------------- |
| `controller_frequency` | 控制周期（实车 30 Hz，`model_dt` 必须与之对齐）         |
| `plugin`               | 实车为 `guga_source_mppi_controller::MPPIController`    |
| `motion_model`         | 运动模型，实车 `Omni`（麦克纳姆底盘）                   |
| `time_steps`           | 每条采样轨迹的时间步数（实车 48）                       |
| `model_dt`             | 采样点时间间隔（实车 1/30 s，须等于 1/控制频率）        |
| `batch_size`           | 每轮采样的候选轨迹数（实车 1400）                       |
| `iteration_count`      | MPPI 迭代次数（实车 2；上游建议保持 1、优先加大 batch） |
| `temperature`          | softmax 选择强度，越小越偏向最优轨迹（实车 0.2）        |
| `gamma`                | 控制序列平滑与能耗的权衡（实车 0.012）                  |
| `visualize`            | 是否发布候选轨迹（实车 false，调试时打开）              |
| `debug_timing`         | 本 fork 新增：每 2 s 打印单周期耗时与各 critic 耗时     |
| `reset_period`         | 无调用超过该时长就复位优化器（默认 1.0 s）              |

### 采样与速度上限

| 参数     | 作用                                                                                    |
| -------- | --------------------------------------------------------------------------------------- |
| `vx_std` | 线速度 x 的采样标准差（实车 0.3）                                                       |
| `vy_std` | 线速度 y 的采样标准差（实车 0.3）                                                       |
| `wz_std` | 角速度采样标准差（实车 0.01，几乎不采样转向）                                           |
| `vx_max` | 最大前进速度（实车 3.0 m/s）                                                            |
| `vx_min` | 最大倒车速度（实车 -3.0 m/s）                                                           |
| `vy_max` | 最大横向速度（实车 3.0 m/s）                                                            |
| `vy_min` | 最小横向速度（实车 -4.0 m/s）                                                           |
| `wz_max` | 最大角速度（实车 1.0 rad/s；自旋由 `nonrotating_vel_transform` 叠加，控制器不输出转向） |

### 加速度约束

| 参数     | 作用                                                                  |
| -------- | --------------------------------------------------------------------- |
| `ax_max` | 加速上限（实车 2.0 m/s²，不宽于下游执行能力）                         |
| `ax_min` | 减速下限（实车 -4.5 m/s²，必须与 `velocity_smoother.max_decel` 一致） |
| `ay_max` | 横向加速上限（实车 3.0 m/s²）                                         |
| `ay_min` | 横向减速下限（实车 -4.5 m/s²）                                        |
| `az_max` | 角加速度上限（实车 0.0，自旋角速度不归控制器管）                      |

### 评价器参数（实车值）

| 评价器                | 参数与取值                                                                                                                                          |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ConstraintCritic`    | `cost_weight: 5.0`、`cost_power: 1`                                                                                                                 |
| `CostCritic`          | `cost_weight: 4.0`、`critical_cost: 300`、`trajectory_point_step: 2`、`consider_footprint: false`、`collision_cost: 1e6`、`near_goal_distance: 1.0` |
| `GoalCritic`          | `cost_weight: 5.0`、`threshold_to_consider: 1.4`                                                                                                    |
| `GoalAngleCritic`     | 已配置但不启用：`cost_weight: 5.0`、`threshold_to_consider: 0.5`                                                                                    |
| `PathAlignCritic`     | `cost_weight: 5.0`、`offset_from_furthest: 10`、`trajectory_point_step: 4`、`max_path_occupancy_ratio: 0.05`                                        |
| `PathFollowCritic`    | `cost_weight: 7.0`、`offset_from_furthest: 6`、`threshold_to_consider: 0.8`                                                                         |
| `PathAngleCritic`     | 已配置但不启用：`cost_weight: 2.0`、`offset_from_furthest: 4`、`max_angle_to_furthest: 1.0`                                                         |
| `PreferForwardCritic` | 已配置但不启用：`cost_weight: 5.0`、`threshold_to_consider: 0.5`                                                                                    |
| `TwirlingCritic`      | 已配置但不启用：`cost_weight: 3.0`                                                                                                                  |
| `ObstaclesCritic`     | 不在 `critics` 列表：`repulsion_weight: 2.0`、`critical_weight: 25.0`、`inflation_radius: 0.367`、`cost_scaling_factor: 10.0`                       |

### 轨迹可视化

| 参数              | 作用                           |
| ----------------- | ------------------------------ |
| `trajectory_step` | 候选轨迹抽样的步长（实车 5）   |
| `time_step`       | 单条轨迹上取点的步长（实车 3） |

注意：`model_dt` 与 `controller_frequency` 必须对齐，误差超过 1e-6 时控制序列移位会关闭或
`configure` 直接抛异常；`ax_min` / `ay_min` 与 `velocity_smoother.max_decel` 成对调整，只改一边会出现
过冲或减速偏慢。

## 测试

```bash
# 构建并运行本包测试（pre-commit 钩子与 CI 都调用它）
scripts/pre-commit/run_mppi_tests.sh
```

当前 `nav2_mppi_controller` 测试套件：

- `optimizer_smoke_test`：优化器冒烟测试
- `optimizer_unit_tests`：优化器单元测试
- `controller_state_transition_test`：控制器生命周期状态转换
- `models_test`：状态、轨迹、控制序列等模型
- `noise_generator_test`：噪声生成
- `parameter_handler_test`：参数声明与动态更新
- `motion_model_tests`：DiffDrive / Omni / Ackermann 运动模型
- `trajectory_visualizer_tests`：轨迹可视化
- `utils_test`：路径与代价工具函数
- `path_handler_test`：路径裁剪与变换
- `critic_manager_test`：评价器加载与评分调度
- `critics_tests`：各评价器行为（单独链接 `mppi_critics`）

本包已纳入 `.pre-commit-config.yaml`（clang-format 与 `test-mppi-controller`）与
`.github/workflows/ci.yml`。

包内 `doxygen/Doxyfile` 可在包根生成方法级 HTML 文档（类页面里列出每个函数与其中文 `@brief` 说明）：

```bash
# 1. 在包根生成（输出 doxygen/html/index.html，生成物由根 .gitignore 忽略）
cd src/guga_controller/nav2_mppi_controller && doxygen doxygen/Doxyfile

# 2. 回工作区根构建，安装到 share 后用浏览器打开（在 source 过 setup.bash 的终端里）
colcon build --packages-select nav2_mppi_controller
google-chrome "$(ros2 pkg prefix nav2_mppi_controller)/share/nav2_mppi_controller/doxygen/html/index.html"
```

需要 doxygen 1.13 及以上；没有 root 权限时可使用官方便携二进制，把其 `bin/` 目录加入 `PATH`。
