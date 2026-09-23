# ObstacleLayerLocal

`pb_nav2_costmap_2d::ObstacleLayerLocal` 是本仓库的 Nav2 障碍层。

## 架构

Nav2 框架插件,代价地图按 `update_frequency` 每周期调用`updateBounds`与`updateCosts`。

## 数据流

```
terrain_analysis ─┬─(terrain_map)────────────┐
                  └─(terrain_returns_current)┤
                                             ▼
                                ObservationBuffer（源级高度与范围过滤）
                                             │  每周期 updateBounds
                      ───────────────────────┴────────────────────────
                      清除：raytraceFreespace → 本层栅格写 FREE_SPACE   
                      标记：逐点判高度与距离 → 写 LETHAL_OBSTACLE        
                      ───────────────────────┬────────────────────────
                                             ▼
                      updateCosts：足印刷 FREE_SPACE + 按 combination_method 合并
                                             ▼
                                        主栅格 → inflation_layer
```

## 输入

| 点云话题名                | Topic                                 | 类型                      | 提供者             |
| ------------------------- | ------------------------------------- | ------------------------- | ------------------ |
| `terrain_map`             | `<namespace>/terrain_map`             | `sensor_msgs/PointCloud2` | `terrain_analysis` |
| `terrain_returns_current` | `<namespace>/terrain_returns_current` | `sensor_msgs/PointCloud2` | `terrain_analysis` |

## 输出

这一层不发布话题，输出是它对代价地图的贡献：

| 输出       | 说明                                                                                                           |
| ---------- | -------------------------------------------------------------------------------------------------------------- |
| 本层栅格   | 标记写 `LETHAL_OBSTACLE`、清除写 `FREE_SPACE`、未触及的格保持 `NO_INFORMATION`（`track_unknown_space` 为真时） |
| 主栅格贡献 | 由 `combination_method` 决定；取较大值时静态层的致命格不会被本层抹掉                                           |
| `current_` | 各源 `isCurrent()` 的与结果，作为本层新鲜度上报                                                                |
| 清图服务   | `isClearable()` 返回真，`reset` 会清空本层栅格与缓冲时间戳                                                     |

## 管线
本节讲解各函数作用。

### 官方 API

[costmap_layer 插件写法链接](https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_costmap2d_plugin/writing_new_costmap2d_plugin/)

| 函数                                                | 职责                                                                                                                                                             |
| --------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `onInitialize`                                      | 声明层级与源级参数，为每个源建 `ObservationBuffer`、TF `MessageFilter` 与订阅，登记 `dynamicParametersCallback`                                                  |
| `updateBounds`                                      | 确定本轮更新障碍地图的范围。内部对本轮新产生的局部栅格地图进行以下步骤:先调用激光清障函数标记空地,再重新标记合法的障碍,并据此更新更新地图的范围。                |
| `updateCosts`                                       | `footprint_clearing_enabled` 为真时，标记`base_footprint`为 `FREE_SPACE`，再根据 `combination_method_` 将局部栅格地图合并进主栅格地图 (目前使用覆盖的合并方式)。 |
| `activate` / `deactivate` / `reset` / `isClearable` | 重新订阅与退订话题；`isClearable` 返回真使清图服务能清理layer产生的局部地图；`reset` 是重置函数                                                                  |

### 本类自有

| 函数                        | 职责                                                                                 |
| --------------------------- | ------------------------------------------------------------------------------------ |
| `dynamicParametersCallback` | 允许运行时动态接收层级参数并更新成员                                                 |
| `laserScanCallback`         | LaserScan 投影成点云后写入该源缓冲                                                   |
| `laserScanValidInfCallback` | 与 `laserScanCallback` 相同，另外先把正无穷替换为 `range_max`                        |
| `pointCloud2Callback`       | 点云写入该源缓冲                                                                     |
| `getMarkingObservations`    | 从标记源的缓冲取出观测（取出前清掉过期观测），并累计 `isCurrent()`                   |
| `getClearingObservations`   | 从清除源的缓冲取出观测（取出前清掉过期观测），并累计 `isCurrent()`                   |
| `raytraceFreespace`         | 对一条观测做射线清除：原点转栅格、端点裁剪到地图矩形、`raytraceLine` 写 `FREE_SPACE` |
| `updateRaytraceBounds`      | 把按 `raytrace_max_range` 截断后的端点并入窗口                                       |
| `updateFootprint`           | 把足印顶点并入窗口；足印内的 `FREE_SPACE` 由 `updateCosts` 写                        |
| `resetBuffersLastUpdated`   | 复位各源缓冲的最后更新时刻                                                           |
| `addStaticObservation`      | 把一条观测追加进静态标记或清除列表；本仓库未使用                                     |
| `clearStaticObservations`   | 清空这两份静态列表；本仓库未使用                                                     |

其中 `raytraceFreespace` 的名字与语义对标官方 `ObstacleLayer` 的虚函数（`VoxelLayer` 就是重写它），但本类不继承那层，所以在本类里只是同名成员，不能标 `override`；本层仍保留 `virtual`，以留住同一个扩展点。

## 参数

### Nav2参数

| 参数                                          | 默认值        | 实车值                                   | 生效时机   | 作用                                                                                                     |
| --------------------------------------------- | ------------- | ---------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------- |
| `plugin`                                      | —             | `pb_nav2_costmap_2d::ObstacleLayerLocal` | 启动时     | 插件类型                                                                                                 |
| `enabled`                                     | `true`        | `true`                                   | 运行期可改 | 是否启用 obstacle_layer；为假时 `updateBounds` 与 `updateCosts` 都直接返回，层内栅格保留旧值但不参与合并 |
| `footprint_clearing_enabled`                  | `true`        | `true`                                   | 运行期可改 | 是否每周期把车体足印内的本层格写成 `FREE_SPACE`                                                          |
| `combination_method`                          | `1`           | `1`                                      | 运行期可改 | 合并进主栅格的方式：`0` 覆盖（跳过本层的 `NO_INFORMATION`）、`1` 取较大值、其它值不写                    |
| `min_obstacle_height` / `max_obstacle_height` | `0.0` / `2.0` | 未设置（跑默认值）                       | 运行期可改 | 标记循环的高度范围，**odom 绝对高度**；与源级同名参数是两处独立过滤                                      |
| `observation_sources`                         | `""`          | `terrain_map terrain_returns_current`    | 启动时     | 源名列表，空格分隔；只在启动时读一次，决定建哪些缓冲与订阅                                               |

### 针对具体点云的参数

标记源指 `terrain_map`，清除源指 `terrain_returns_current`；数值取自 `reality/controller/mppi.yaml` 的 local 与 global 两段，两段不同的地方分别标注。

| 参数                                          | 默认值        | marking实车值                                | clearing实车值                 | 生效时机 | 作用                                                                                                                                                                 |
| --------------------------------------------- | ------------- | -------------------------------------------- | ------------------------------ | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `topic`                                       | 源名本身      | `<ns>/terrain_map`                           | `<ns>/terrain_returns_current` | 启动时   | 订阅的话题                                                                                                                                                           |
| `data_type`                                   | `LaserScan`   | `PointCloud2`                                | `PointCloud2`                  | 启动时   | `PointCloud2` 或 `LaserScan`，决定走哪个回调                                                                                                                         |
| `sensor_frame`                                | `""`          | `front_mid360`                               | `front_mid360`                 | 启动时   | 射线清除的原点为该坐标系的原点。留空则回退成点云自己的 `frame_id`，而本层点云在 odom 系，射线原点就成了 odom 原点                                                    |
| `marking`                                     | `true`        | `true`                                       | `false`                        | 启动时   | 该点云参与标记障碍                                                                                                                                                   |
| `clearing`                                    | `false`       | `false`（global 的 `terrain_map` 为 `true`） | `true`                         | 启动时   | 该点云参与射线清除。默认是 false，漏写会让射线清除静默失效                                                                                                           |
| `obstacle_max_range` / `obstacle_min_range`   | `2.5` / `0.0` | local `5.0` / `0.2`；global `10.0` / `0.2`   | 同左                           | 启动时   | 障碍标记范围                                                                                                                                                         |
| `raytrace_max_range` / `raytrace_min_range`   | `3.0` / `0.0` | local `5.5` / `0.2`；global `5.0` / `0.2`    | 同左                           | 启动时   | 射线清除的范围                                                                                                                                                       |
| `min_obstacle_height` / `max_obstacle_height` | `0.0` / `0.0` | `0.0` / `2.0`                                | `-3.0` / `3.0`                 | 启动时   | 允许点云高度范围，odom 绝对高度，在 `ObservationBuffer` 里先于标记与清除过滤；与层级同名参数是两处独立过滤。默认两边都是 0.0，等于只留 z 恰好为 0 的点，必须显式配置 |
| `observation_persistence`                     | `0.0`         | 未设置                                       | `0.0`                          | 启动时   | 观测在缓冲区里保留多久（秒）；`0.0` 表示只留最新一帧                                                                                                                 |
| `expected_update_rate`                        | `0.0`         | 未设置                                       | 未设置                         | 启动时   | 期望接收间隔（秒），超时会报告该帧点云不新鲜并影响 `current_`；默认 `0.0` 等于不检查                                                                                 |
| `inf_is_valid`                                | `false`       | 未设置                                       | 未设置                         | 启动时   | 仅 `LaserScan` 使用，是否把 `inf` 当有效测量参与清除                                                                                                                 |


清除源的源级高度取 ±3.0，是因为地面返回的雷达射线的绝对 z 略低于 0，留 0 会把它们全部滤掉，清除就失去端点。



## 测试

```bash
colcon test --packages-select pb_nav2_plugins
```

- `test/obstacle_tests.cpp`：取自上游的集成测试，启用的 5 个用例覆盖栅格标记结果、反复 `reset`、以及经 `Costmap2DROS` 与 pluginlib 的动态参数设置。上游自己用 `#if (0)` 关掉的那一段不参与编译。
- `test/testing_helper.hpp`：上游同名文件的改写版，把其中的 `ObstacleLayer` 换成本层，使这些用例驱动本实现。
- 尚未覆盖：射线清除的边界情形（端点越界裁剪、射线被近处障碍挡住）、多源缓冲的时序。
