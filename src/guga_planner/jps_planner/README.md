# jps_planner

`jps_planner` 是当前默认使用的 Nav2 全局规划器插件，插件名为
`jps_planner/JPSPlanner`。它在 costmap 上执行 JPS 搜索，再对 JPS 跳点做
B-spline 平滑，并可选接入 ESDF 距离场进行避障优化。

更完整的算法说明见 [DESIGN.md](./DESIGN.md)。

## 数据流

```text
start / goal (world)
  -> costmap worldToMap
  -> JPSAlgorithm::generatePath
  -> map_path, 格元中心坐标
  -> 短路径补密, 至少满足 7 阶 B-spline 的 8 个输入点
  -> BSplineOptimizer::fit
  -> 可选 ESDF 梯度下降
  -> 障碍物投射后处理
  -> 连续地图坐标转 world
  -> nav_msgs::Path
```

JPS 输出的是稀疏跳点。在开阔区域中，起点、拐点、终点可能只有 2 到 3 个点。
如果直接线性插值，路径会保留 8 邻域搜索的 45 度/水平/竖直折线特征。当前实现会先
在地图坐标中补密短路径，再进入 B-spline，避免日志里出现
`too few waypoints ... linear fallback` 后路径退化成 8 向折线。

## 安全处理

- JPS 障碍物阈值为 `cost >= 253`，即 Nav2 的内切膨胀障碍物及以上不可通行。
- 对角移动会检查两个相邻边格，禁止从两个阻断格之间斜穿。
- B-spline 输出保持连续地图坐标，不再取整到整数格，避免平滑路径被重新量化。
- 平滑路径会做最终 costmap 碰撞检查；如果碰撞，回退到线性 JPS 路径；仍碰撞则返回空路径。

## 当前仿真参数

`scripts/simulation.sh` 默认读取
`src/guga_bringup/config/simulation/nav2_params.yaml`。当前仿真中和 JPS 避障相关的主要值是：

```yaml
local_costmap:
  robot_radius: 0.25

global_costmap:
  robot_radius: 0.35

planner_server:
  ros__parameters:
    GridBased:
      enable_bspline: true
      enable_esdf: true
      esdf_weight: 200.0
      esdf_safe_distance: 1.5
      corridor_halfwidth: 15.0
```

`corridor_halfwidth` 的单位是 costmap 格元。仿真分辨率为 0.05 m 时，`15.0`
约等于允许控制点相对初始路径移动 0.75 m。

## 常见现象

### 路径贴墙

优先检查 costmap 膨胀半径、`robot_radius`、`esdf_safe_distance` 和
`corridor_halfwidth`。`esdf_safe_distance` 负责让优化器提前感知障碍物，
`corridor_halfwidth` 限制优化器能把路径推离原始 JPS 路径多远。

### 路径像 8 向移动

先看 planner 日志。如果只输出少量 JPS 跳点，旧逻辑会因为点数不足跳过 B-spline。
当前实现会打印类似：

```text
JPSPlanner: densified path from 3 to N waypoints for B-spline
JPSPlanner: B-spline smooth applied, 100 poses
```

如果仍然看到线性 fallback，通常表示路径极短、B-spline 拟合失败或平滑路径碰撞后回退。

## 测试

```bash
colcon build --packages-select jps_planner --event-handlers console_direct+
./build/jps_planner/test_jps
```
