# TODOLIST

| 分类 | 任务 | 所属包 |
| --- | --- | --- |
| 感知 | 坡面识别：让哨兵上坡时对准坡面法线，可参考川大开源 | `terrain_analysis` |
| 感知 | Point-LIO 重构：代码太乱，不急，可先当黑盒用 | `point_lio` |
| 感知（实车） | 低矮障碍物无法识别 | `terrain_analysis` / `rog_map_layer` |
| 感知（实车） | 眼前（近距）障碍物识别异常 | `terrain_analysis` / `rog_map_layer` |
| 感知（实车） | **待实测验证**：`vehicle_z` 基准（已按 O 离地 255 mm 推算为 −0.230，需实车读数确认） | `terrain_analysis` |
| 建图 | 建图模式无法清除伪静态障碍物（动态目标轨迹被当静态地图保留） | `slam_toolbox`（外部依赖）/ `terrain_analysis` |
| 感知 | ~~`terrain_analysis_ext` 退化为近场半径过滤器~~ **已删除**（2026-09-17）：消费者（`global_costmap`、`pointcloud_to_laserscan`）改指 `terrain_map`，属严格放宽（4 m → ≤±5.1 m） | `terrain_analysis` |
| 感知（待定） | Terrain voxel 网格 21×21 是否有必要：它同时承担"前瞻预存"与"每帧空转 72% 格子"两重角色，缩小有行为代价 | `terrain_analysis` |
| 控制 | MPPI 的 GPU 方案（MPPI 本体已接入） | `nav2_mppi_controller` |
| 控制（实车） | 避障后退方向错误：朝 chassis 后方运动，而非背离障碍物 | `pb_omni_pid_pursuit_controller` |
| 重构 | `ui_types.hpp` 里全是魔法数字，待修复 | `guga_ui_common` |

## 已修复待验证：近处低地面点被忽略

**现象**：实车近处的低地面点被错误忽略（远处正常）。

### 已完成的修复（2026-09 提交 34be0f4）

**根因（已定位）**：`estimateTerrainGround` 里有一条 `CEILING_CLEARANCE` 上界，
按**车高**筛地面候选。以实测标定值（O 离地 255 mm、`vehicle_z ≈ −0.230`）代入，
其作用面是 `point.z ≥ −0.130`，即**离地仅 0.125 m**——凡高于脚踝的点全部被当
"天花板"丢弃。这是**设计问题**（净空是"障碍能否通过"的判据，与"哪些点属于地面"
无关），不是标定问题。

**修复内容**：

1. 从 `estimateTerrainGround` **移除**该净空上界，该阶段现只保留下界
   `ground_floor_z`（绝对 z）。`CEILING_CLEARANCE` 现仅由 `computeHeightMap`
   使用，语义唯一。
2. `groundFloorZ` 由 `−2.0` 改为 **`−0.45`**（原值离地尚有 1.75 m 余量，形同虚设）。
3. 测试 `EstimateTerrainGround_CeilingPoint_ExcludedFromElevation` 断言的正是被
   移除的行为，改为 `AboveCeilingClearance_StillParticipates`；README 风险节
   的 3、4 标为已修复，风险 1 记为已部分修复。

**已验证**（合成点云）：10% 上坡时，修复前 `d ≥ 1.25 m` 的地面点全被丢弃
（该范围输出为 0），修复后 x 每 0.5 m 分箱均约 40 点、**连续无截断**。平地与
0/10/20% 坡的"输出点与真实地面最大偏差"均为 0.0000 m。

### 待实测确认（下次）

推算用的 `O 离地 255 mm` 来自机械队员口述；`vehicle_z ≈ −0.230` 由
`base_footprint→O = 0.230` 推得，**尚未在实车验证**。请读：

```bash
ros2 topic echo /lidar_odometry --field pose.pose.position --once
```

- 若静止时 `z ≈ −0.230` → 推算成立，本次修复即为正解；
- 若明显不同（尤其接近 0）→ TF 链与装机不符，需按实测重设 `groundFloorZ`
  与净空基准（并把该值反馈回本文件）。

**已排除**：3×3 邻域膨胀（`addToPlanarNeighborhood3x3`）。实验：把膨胀改成只写
中心格，下沉区输出**完全不变**（96 点）；且实测 `planar_voxel_elev` 精确落在实际
地面高度（环带内 −0.6000、环带外 −0.5000）。故膨胀既未抬高 `elev`、也未丢弃点。

**当前怀疑（主）**：`vehicle_z` 基准未标定。

`lidar_odometry` 的 z 由 `loam_interface` 产生：

```
tf_odom_to_lidar = tf(base_footprint→lidar) * tf(Point-LIO 位姿)
```

即它是 **`base_footprint` 相对 odom 原点**的高度，而 **odom 原点是 Point-LIO 启动
时定下的**——没有任何保证"`vehicle_z ≈ 0` 时车正好在地面上"。而有两处把 `vehicle_z`
当基准：

| 判据 | 形态 | 偏置后果 |
| --- | --- | --- |
| 净空上界（两阶段） | `point.z − vehicle_z >= CEILING_CLEARANCE(0.1)` | 允许进入的 10 cm 带随车整体平移 |
| 下界 `min_relative_z` | `point.z − vehicle_z <= −1.5` | 同样平移 |

### terrain 用的 vehicle 从哪来（已核实）

`terrain_analysis_node.cpp` 直接把消息里的位置塞进状态，**无 TF 查询、无 frame 换算、
无偏置补偿**：

```cpp
processor_.ingestOdometry(msg->pose.pose.position.x,   // → state_.vehicle_x
                          msg->pose.pose.position.y,   // → state_.vehicle_y
                          msg->pose.pose.position.z,   // → state_.vehicle_z
                          roll, pitch, yaw);           // 由四元数 getRPY 解出
```

三点结论：

1. **参考系是 `odom` 而非 `map`**：订阅的 `lidar_odometry` 与发布的 `terrain_map`
   都标 `frame_id = odom`，terrain 全程不知道 `map` 存在。实车默认 `slam:=False`
   （走 GICP 重定位）时，terrain 的 `vehicle_*` 与"车在地图里的位置"差一个
   `map→odom` 变换。
2. **`vehicle_z` 是 `base_footprint` 的高度**，因 `loam_interface` 做了
   `tf(odom→base_footprint) = tf(base_footprint→lidar) * tf(Point-LIO 位姿)`。
3. **姿态来自消息四元数、不是 TF**：`chassis→front_mid360` 有 −10° 安装 roll，
   `loam_interface` 已把它折进 `odom→base_footprint`，故 terrain 拿到的 roll 是
   车体姿态、不含该倾角。

### 高度链（实车，`static_tf_publisher_launch.py`）

| 变换 | z |
| --- | --- |
| `base_footprint → chassis` | +0.123 |
| `chassis → front_mid360` | +0.107（另 x=+0.225、roll=−π/18、yaw=−π/2） |
| **合计 `base_footprint → front_mid360`** | **+0.230 m** |

### 雷达标定点是 O（探测中心），不是底面

据 [Livox Mid-360 User Manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf)
第 11 页 Coordinates 节："Point O is the origin, and O-XYZ is the point cloud
coordinates"，配图尺寸 60.0±0.5（俯视直径）、39.5、14.3、7° 光学轴倾角。安装要求
另注明"Use the bottom surface for mounting"。

⇒ **点云坐标以 O 为原点**，故 ROS 里 `front_mid360` frame 对应 **O**；O 在本体内部，
距底面约 **39.5 mm**。因此上表 0.230 是 **base_footprint 到 O 的高度差**，
到底面约为 0.230 − 0.0395 ≈ **0.190 m**。

**由此**：

```
vehicle_z = (地面到 O 的高度) − 0.230
```

而 `CEILING_CLEARANCE = 0.1`（作用面在 O 下方 0.1 m）与 `minRelZ = −1.5` 都锚在
**探测中心**上，其物理含义取决于"O 离地多高"这个**装机尺寸，不在本仓库内**。
另外 `groundFloorZ = −2.0` 是**绝对 z**，与车高无关——若实际地面 z ≈ −0.23，
该地板离地尚有约 1.77 m 余量，形同虚设。

**须向机械队员确认（待问）**：

1. 实车 **O（雷达探测中心）离地高度** 是多少 → 直接定出 `vehicle_z = 该值 − 0.230`；
2. URDF/网格里 **`front_mid360` 这一 frame 的原点是否放在 O 处**（若当初按底面建模，
   会整体差 39.5 mm；那三个数 x=0.225 / z=0.107 也应是在 chassis 系下量到 O 的）；
3. 底盘基准面到雷达**底面**的实测距离 → 自检：应约等于 `0.230 − 0.0395 ≈ 0.190 m`。

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
- **伪静态障碍物**：SLAM 模式点云来自 `terrain_map`（原 `terrain_map_ext` 已
  于 2026-09-17 删除），地图由 `slam_toolbox` 维护且无
  消退机制；该包不在工作区内，改动需走配置或上游输入。
- **避障后退方向**：需检查后退避障的期望速度是否在正确坐标系下生成
  （`prefer_forward_critic.cpp:42` 的后退惩罚 / 底盘正方向约定）。
- **MPPI GPU 方案**：接入入口见 `nav2_mppi_controller`；导航组合用 `controller:=mppi`。
- **ext 已删除（2026-09-17）**：它当时只剩 `mergeLocalTerrain` 一个半径过滤器——
  把 `terrain_map` 里距车 > `localTerrainMapRadius`(4.0 m) 的点丢掉再转发，无信息
  增量，而 4 m 比 `local_costmap` 需要的 5 m 还窄，**在做负功**。原版 CMU 的远场累积
  与 `checkTerrainConn` 连通性判定在这套移植里本来就不存在（参数表是从原版抄的），
  不是这次删除造成的；若将来需要远场/连通性判定，需另行设计。
  删除后消费者改指 `terrain_map`（严格放宽：≤4 m → ≤±5.1 m，受 planar 网格限制）：
  `global_costmap` 的 `intensity_voxel_layer`、以及 SLAM 模式的 `pointcloud_to_laserscan`。
  随之移除：`terrain_map_ext` 话题、`localTerrainMapRadius` 参数、`navigation_launch.py`
  里的 ext 节点（独立与 composable 两种声明）、`guga_bringup` 的 exec_depend、
  两个脚本的包列表与 rviz 话题项。
  现半径链：主版裁剪 ±11 m → 提取窗口 ±5.5 m（`collectTerrainCloud` 的
  `EXTRACT_HALF_WINDOW=5` × 1.0 m 格）→ 输出受 planar 网格限制为 ±5.1 m。
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
