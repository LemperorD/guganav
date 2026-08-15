# guga_planner

规划器模块，负责 Nav2 全局规划、路径平滑和轨迹优化。

## 当前使用的 planner

默认 profile（`jps_pid`）使用的是 `jps_planner/JPSPlanner`，挂在 Nav2 的 `planner_server` 上。
它先在 costmap 上做 JPS 搜索，再根据参数选择 B-spline 平滑和 ESDF 梯度优化。

### 配置位置

`scripts/simulation.sh` 默认读取的是 `src/guga_bringup/config/simulation/nav2_params.yaml`。
如果是实机或别的 launch，再看 `src/guga_bringup/config/reality/nav2_params.yaml` 和
`src/guga_bringup/config/simulation/nav2_params_mpc.yaml`。

不同 `navigation_profile` 对应不同参数文件，planner/controller 各不相同：

| profile | 参数文件 | planner | controller |
|---------|---------|---------|-----------|
| `jps_pid`（默认） | `simulation/nav2_params.yaml` | `jps_planner/JPSPlanner` | `pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode` |
| `2d_mppi` | `simulation/nav2_params_mppi.yaml` | `nav2_smac_planner/SmacPlanner2D` | `guga_source_mppi_controller::MPPIController` |
| `jps_mpc` | `simulation/nav2_params_mpc.yaml` | `nav2_smac_planner/SmacPlannerHybrid`（JPS 已注释） | `mpc_controller::MpcControllerNode` |

> 实车 `reality/nav2_params.yaml` 与默认仿真 profile 一样挂 `jps_planner/JPSPlanner`。

当前仿真里常调的三个值是：

- `esdf_weight: 200.0`
- `esdf_safe_distance: 1.5`
- `corridor_halfwidth: 15.0`

为了让障碍周围的不可通行区域更保守，仿真参数中还调大了机器人半径：

- local costmap: `robot_radius: 0.30`
- global costmap: `robot_radius: 0.40`

代码默认值分别是：

- `esdf_weight: 100.0`
- `esdf_safe_distance: 0.6`
- `corridor_halfwidth: 8.0`

核心配置片段如下：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "jps_planner/JPSPlanner"
```

### 实现位置

- `src/guga_planner/jps_planner/src/jps_planner.cpp`
- `src/guga_planner/jps_planner/include/jps_planner/jps_planner.hpp`
- `src/guga_planner/jps_planner/jps_planner_plugin.xml`

### 常改参数

- `w_euc_cost`
- `w_traversal_cost`
- `w_heuristic_cost`
- `allow_unknown`
- `enable_bspline`
- `enable_esdf`
- `esdf_weight`
- `esdf_safe_distance`
- `corridor_halfwidth`

路径贴墙时优先调大 `esdf_safe_distance` 和 `corridor_halfwidth`。
`corridor_halfwidth` 的单位是 costmap 格元；仿真分辨率 0.05m 时，`15.0` 约等于允许控制点横向移动 0.75m。
如果路径呈现明显 45 度/水平/竖直分段，优先看 JPS 日志是否进入了短路径补密后的 B-spline。

### 如果要换 planner

只改 `planner_server` 下面的 `planner_plugins` 和 `GridBased.plugin` 即可。
例如把 `jps_planner/JPSPlanner` 换成别的 Nav2 planner 插件。

## 相关能力

- `jps_planner`：当前主用的全局规划器
- `bspline_opt`：B-spline 平滑与优化工具
- `pb_nav2_plugins`：Nav2 额外行为和 costmap 插件

## TODO

- [X] 前端轨迹生成（JPS / JPS 搜索）
- [X] 后端轨迹优化（minco / b-spline）
- [ ] JPS 引入动力学约束
- [ ] JPS 使用 ESDF 信息规划路径
- [ ] 动态障碍物避障（点云滤波识别 + 轨迹预测）
