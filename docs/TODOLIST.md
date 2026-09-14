# TODOLIST

| 分类 | 任务 | 所属包 |
| --- | --- | --- |
| 感知 | 坡面识别：让哨兵上坡时对准坡面法线，可参考川大开源 | `terrain_analysis` |
| 感知 | Point-LIO 重构：代码太乱，不急，可先当黑盒用 | `point_lio` |
| 感知（实车） | 低矮障碍物无法识别 | `terrain_analysis` / `rog_map_layer` |
| 感知（实车） | 眼前（近距）障碍物识别异常 | `terrain_analysis` / `rog_map_layer` |
| 感知（实车，进行中） | 近处低地面点被错误忽略；怀疑与实车 `vehicle_z` 基准未标定有关 | `terrain_analysis` |
| 建图 | 建图模式无法清除伪静态障碍物（动态目标轨迹被当静态地图保留） | `slam_toolbox`（外部依赖）/ `terrain_analysis_ext` |
| 感知 | `terrain_analysis_ext` 已退化为近场半径过滤器：全局代价地图实际只拿到 4 m 地形，而配置按 10 m 工作 | `terrain_analysis_ext` / `terrain_analysis` |
| 感知（待定） | Terrain voxel 网格 21×21 是否有必要：它同时承担"前瞻预存"与"每帧空转 72% 格子"两重角色，缩小有行为代价 | `terrain_analysis` |
| 控制 | MPPI 的 GPU 方案（MPPI 本体已接入） | `nav2_mppi_controller` |
| 控制（实车） | 避障后退方向错误：朝 chassis 后方运动，而非背离障碍物 | `pb_omni_pid_pursuit_controller` |
| 重构 | `ui_types.hpp` 里全是魔法数字，待修复 | `guga_ui_common` |

## 进行中：近处低地面点被忽略

**现象**：实车近处的低地面点被错误忽略（远处正常）。

**已排除**：3×3 邻域膨胀（`addToPlanarNeighborhood3x3`）。实验：把膨胀改成只写
中心格，下沉区输出**完全不变**（96 点）；且实测 `planar_voxel_elev` 精确落在实际
地面高度（环带内 −0.6000、环带外 −0.5000）。故膨胀既未抬高 `elev`、也未丢弃点。

**当前怀疑（主）**：`vehicle_z` 基准未标定。

`lidar_odometry` 的 z 由 `loam_interface` 产生：

```
tf_odom_to_lidar = tf(base_footprint→lidar) * tf(Point-LIO 位姿)
```

即它是 **`base_footprint` 相对 odom 原点**的高度，而 **odom 原点是 Point-LIO 启动
时定下的**——没有任何保证"`vehicle_z ≈ 0` 时车正好在地面上"。若启动时雷达离地
0.3 m，则全程带 0.3 m 常数偏置。而有两处把 `vehicle_z` 当基准：

| 判据 | 形态 | 偏置后果 |
| --- | --- | --- |
| 净空上界（两阶段） | `point.z − vehicle_z >= CEILING_CLEARANCE(0.1)` | 允许进入的 10 cm 带随车整体平移 |
| 下界 `min_relative_z` | `point.z − vehicle_z <= −1.5` | 同样平移 |

**为何"近处"最明显**：`ingestLaserCloud` 的裁剪带是
`min/max_relative_z ± disRatioZ × distance`，**带宽随距离放宽**；而净空是
**常数 0.1 m、不放宽**。于是

- 近处 `z_margin ≈ 0`，实际生效的只有 `[−1.5, +0.1]` 这条 0.2 m 窄带，对偏置极敏感；
- 远处 `z_margin` 可达 1 m 以上，把带撑宽，偏置被掩盖。

**注意**：用合成点云未能复现（改变 `vehicle_z` 于 −0.2~+0.3，输出仅 2592 vs 2601
点）——合成地面点恰好都排在 `[−1.5, +0.1]` 内。实车有噪声、起伏与漂移才会越界，
故**尚不能断言"就是高度没对"**，只能说该因果链成立且与"近处"特征吻合。

**下次先取这三个实测值**（`ros2 topic echo /lidar_odometry --field pose.pose.position`）：

1. 开机静止时 `z` 是否为 0（非 0 即常数偏置）；
2. 静止 30 s 的 `z` 漂移量（常数偏置可标定；漂移则必须改为相对地面）；
3. 同一时刻激光测到的地面 z（可用 `terrain_map` 大片点的 z）。二者之差即真实偏置。

**对症处理**（取决于上面结果）：

- 仅常数偏置 → 标定；并把净空判据的基准从 `vehicle_z` 换成
  `ground_floor_z + 离地高度`，与 `vehicle_z` 解耦；
- 有漂移 → 判据必须用**相对局部地面**（`computeHeightMap` 已改成这样），
  `estimateTerrainGround` 需走两遍法（先最低点估粗地面）。

两条路都会顺带解决 README 风险 1（坡面）——那是同一处基准问题。

## 排查线索

- **低矮障碍物**：疑似被 `terrain_analysis` 的高度阈值链过滤（`minRelZ` / `maxRelZ` /
  `minBlockPointNum` / `CEILING_CLEARANCE`），或下游代价地图的 `min_obstacle_intensity`
  将其丢弃；需先定位是哪一环。注意 `CEILING_CLEARANCE` 现已固定为 0.1 m，
  即**高出车顶 10 cm 以上的点一律不输出为障碍**。
- **眼前障碍物**：近距点云占比高且分布集中，需排查是否被地面估计抬高、动态障碍过滤误清，
  或近距裁剪范围（`obstacle_min_range`）影响。与上面"近处低地面点"很可能同源。
- **伪静态障碍物**：SLAM 模式点云来自 `terrain_map_ext`，地图由 `slam_toolbox` 维护且无
  消退机制；该包不在工作区内，改动需走配置或上游输入。
- **避障后退方向**：需检查后退避障的期望速度是否在正确坐标系下生成
  （`prefer_forward_critic.cpp:42` 的后退惩罚 / 底盘正方向约定）。
- **MPPI GPU 方案**：接入入口见 `nav2_mppi_controller`；导航组合用 `controller:=mppi`。
- **ext 退化为近场过滤器**：ext 现在只做 `mergeLocalTerrain` 半径过滤，无远场累积、
  无连通性检查；原版 CMU 的远场 + `checkTerrainConn` 判定整体缺失（参数表是从原版
  抄的，实现只剩近场合并段）。注意 `localTerrainMapRadius` 语义被反转：原版用它
  **排除**近场、输出远场，我们用它**只保留**近场。半径链：主版裁剪 ±11 m →
  提取窗口 ±5 m（`collectTerrainCloud` 的局部常量 `EXTRACT_HALF_WINDOW=5` × 1.0 m）
  → ext 再滤 ≤4 m；
  而 `global_costmap` 的 `obstacle_max_range` 配的是 10 m
  （`reality/nav2_params.yaml`），即全局代价地图拿到的地形比 `local_costmap`（5 m）还少。
- **Terrain voxel 网格 21×21 是否必要（待定，已测量）**：不能只看"外圈 320 格从未被
  `collectTerrainCloud` 读取"就断定是空转。外圈实为**车辆前方的地形预存区**：
  `voxelizeTerrain` 按车辆当前位置归格、`rolloverTerrainVoxels` 随车滚动，车前进时
  外圈点会被滚进 ±5 m 窗口参与地面估计。且 `ingestLaserCloud` 的接收半径绑定网格宽度
  （`terrain_voxel_size * (HALF_WIDTH + 1)`），缩网格会连带把接收范围从 ±11 m 降到 ±6 m。
  已测量（21×21 → 11×11，静止/移动差分）：
  - `updateTerrainVoxels` 单次 0.376 ms → 0.220 ms（**省 41.5%，即 0.156 ms/帧**）
  - **静止时地面估计零差异**（`planar_voxel_elev` 逐格相同，候选点 67392 对等）
  - 移动时地面候选点 **−12%**，并新增 **126~220 个完全无地面估计的空格**
    （旧版移动时空格为 0），因 `minBlockPointNum=10` 整格丢弃而出现时检时漏
  - 结论：0.156 ms/帧 换上述行为变化不划算，**已回退**。
  若仍要拿这部分 CPU，应改走"保留 21 宽网格 + 只遍历/维护内圈"的索引方案
  （行为可保不变），而不是缩网格。注意 `computePlanarElevation` 是**分位数**
  （`quantileZ`）不是中位数，候选越少越向 `elevateByMinimum` 退化。
- **坐标系不统一（完整版见 `src/guga_perception/terrain_analysis/README.md`
  的「已知风险」节）**：管线内同时使用 odom 绝对 / 车辆相对 / 局部地面三个参考系。
  其中 `CEILING_CLEARANCE` 与 `min_relative_z` 仍是相对车高，
  `estimateTerrainGround` 的上界因此在上坡时会把抬升的地面点剔除。
