# Terrain 衰减、高度过滤与 Nav2 射线清障方案（设计稿）

> 2026-09-17 审查后修订：**只修改文档，未实施代码或 YAML 改动**。
> 当前方案以 R1-R8 为准：**terrain 衰减和高度过滤 + 纯官方 ObstacleLayer 射线清障，不新增 costmap 插件**。文末折叠保存的旧稿含已否定的推导，不作为实施依据。
> **2026-09-17 追加（已实施）**：删除 `terrain_analysis_ext`，其消费者（`global_costmap`、`pointcloud_to_laserscan`）改指 `terrain_map`——影响见 R2。**R1–R8 其余内容仍未实施。**
>
> **本文结构**：正文 R1–R8 是当前方案——R1 职责与"不动 costmap"的边界 · R2 两个当帧话题与单一标记入口 · R3 terrain 视野内时间衰减 · R4 高度过滤与清除输入分开 · R5 官方 `ObstacleLayer` 配置 · R6 成功判据的边界 · R7 实施边界与步骤 · R8 验收条件与旧稿更正。文末 `<details>` 内为审查前旧稿（含 §3 事实基础、D1–D20 推导、A1–A9 附加审查、§11 复核命令），**其条目哪些撤回、哪些保留一律以 R8 的更正表为准**。

## R1. 职责与“不动 costmap”的边界

**采用 terrain 做视野内时间衰减及高度过滤，local costmap 通过配置改用官方 `nav2_costmap_2d::ObstacleLayer`。** 不修改 Nav2 核心，不修改现有 `IntensityVoxelLayer`，不新增适配插件。本阶段只改本文，以下代码和配置内容均为后续实施设计。

两种删除分别生效：terrain 过期的是感知历史点，ObstacleLayer 射线删除的是自身保存的栅格标记。**有有效射线经过的旧标记可以清除；没有射线经过时，不承诺 terrain 衰减能同步清掉 costmap。** 本方案接受该限制，不再以“所有残留都有时间上界”作为成功判据。

| 功能 | 负责位置 | 数据约束 |
| --- | --- | --- |
| 视野内时间衰减 | terrain | 保存观测时间和有效视野内的未重观测时长 |
| 地面估计、高度过滤 | terrain | 按局部地面计算高度，标记输出提前过滤地面及可通行净空 |
| 障碍标记与射线清障 | 同一个 Nav2 官方 `ObstacleLayer` 实例 | 当帧障碍做 marking，当帧回波及采集位姿做 clearing |
| 无射线覆盖的栅格残留 | 本阶段无时间兜底 | 可由后续有效射线、窗口移出、footprint 或显式清图消除；均不是固定时间保证 |

本机 Humble 的 `ObstacleLayer`、`VoxelLayer` 都是可配置加载的官方插件，内部包含射线清障；`costmap_plugins.xml` 中没有独立通用 `RaytraceLayer`。`ObstacleLayer` 是 2D 清除，`VoxelLayer` 是 3D 体素清除，两者不能等同。

“不动 costmap”需区分三个边界：

- **不改 Nav2 核心源码：可以。** 直接加载已安装的官方插件。
- **不改现有 `IntensityVoxelLayer`：可以。** local 替换为官方层，global 继续使用原插件和原输入；不会因修改共用类而意外改变 global。
- **不新增或修改 costmap 插件代码：本方案可以。** terrain 内实现衰减，官方层按射线清除；代价是两者的删除不做强同步。若将来要求“即使无射线，costmap 也必须随 terrain 到期删除”，才需要重新评审同步方案，不能靠现有官方参数实现。

另挂一个 clearing-only `ObstacleLayer` 不能替已有层清除障碍记忆：官方 raytrace 写的是自己的层网格；`combination_method: 1` 合并时 `max(FREE, LETHAL)` 仍为 lethal。改为覆盖合并可能擦掉静态层障碍，也没有删除原障碍层记忆，不采用此方案。

## R2. 两个当帧话题与单一标记入口

保留现有 `terrain_map` 给既有消费者。**`terrain_analysis_ext` 已删除（2026-09-17）**：它当时只剩 `mergeLocalTerrain` 一个半径过滤器（把距车 > `localTerrainMapRadius`(4.0 m) 的点丢掉再转发，无信息增量），两个原消费者改指 `terrain_map`，属**严格放宽**：

| 原消费者 | 原输入 | 现输入 |
| --- | --- | --- |
| `global_costmap` 的 `intensity_voxel_layer` | `terrain_map_ext`（≤4 m） | `terrain_map`（≤±5.1 m，受 planar 网格限制） |
| `pointcloud_to_laserscan`（仅 SLAM 模式） | `terrain_map_ext` | `terrain_map` |

`terrain_map_ext` 话题与 `localTerrainMapRadius` 参数随之不存在；`navigation_launch.py` 不再启动 ext 节点。为新的 local 链路另新增两个标准 `sensor_msgs/msg/PointCloud2` 话题，名称为设计约定，尚未实现：

| 数据 | 内容 | 用途 |
| --- | --- | --- |
| `terrain_obstacles_current` | 仅本帧实际观测、且通过高度过滤的点 | 唯一 marking 来源 |
| `terrain_returns_current` | 本帧有效回波，保留可用于清除的地面回波 | 官方 raytrace 的 clearing 来源 |

地面估计可以利用衰减后的历史，但 `terrain_obstacles_current` 中每个 XYZ 必须来自当前扫描。不能把累计云改一个新时间戳当作当帧观测；也不能由历史地图体素质心合成清除射线端点。感知历史保留在 terrain 内，**不接入 local 的 marking/clearing 来源**。因此 terrain 衰减主要控制地面估计等感知历史，不直接控制官方层的障碍寿命。

两个输出使用同一扫描的原始时间戳和实际坐标系，在一次处理完成后连续发布。官方层为两个源维护独立观测缓冲，**相同时间戳不代表原子配对**；接收、TF 等待及 costmap 调度可能导致新 clearing 搭配旧 marking，短暂重标残影。以回放测试量化这一延迟，本阶段不新增消息封装或同步插件。

当有效扫描中没有障碍时，仍发布带完整 XYZ 字段定义、正确 header 的零点障碍云，替换上一帧 marking 观测；**空云自身不清除栅格**。地面估计失败不能等同于“没有障碍”：新分支需输出诊断并使对应 clearing 失效或受限，不能在分类失败时继续用宽泛回波擦除已有障碍。具体失效行为在 terrain 实现和回放验收中验证。

新输出从当前扫描单独生成，不能直接复用受旧 dy_obs 抑制的累计输出。旧 dy_obs 保留于兼容链路；新链路不采用其“动态即抑制”逻辑，必须把这项行为差异纳入回归。

## R3. 视野内时间衰减仍在 terrain

建议首先采用以下明确语义：**历史点在有效传感器视野内、持续未重新观测达到阈值后过期；离开视野时暂停累计，重新进入后继续累计。** 这与“墙钟时间已很老，刚进视野立即删除”不同。

视野按该扫描时刻的传感器 TF、有效距离、水平/垂直角范围以及已知自遮挡区域定义。不能仅凭“在 local costmap 窗口里”判断可观测；也不能用当前机器人位姿替代扫描时刻位姿。

对每个历史空间单元保存独立的 `last_seen` 和 `unseen_visible_time`，不把时间放进用于高度的 intensity 并参与均值降采样。每个有效新扫描执行：

1. 有匹配新观测：更新 `last_seen`，`unseen_visible_time = 0`。
2. 未重观测且处于有效视野：累计有效扫描间隔。
3. 视野外：冻结该计时器。在有效视野内累计达到阈值的单元从 terrain 历史集合删除。

只累计连续有效扫描覆盖的间隔；扫描中断、TF 失败、时间回跳、重复时间戳不能推进计时或伪造刷新。跨中断恢复时不补算整个停机时长。距离近也不应被旧 `noDecayDis` 永久豁免，否则违反“视野内衰减”。

**遮挡策略必须与验收一致：** 上述几何视野包括被其他物体遮挡的区域，因此真实但被遮挡的障碍也可能超时遗忘。若要求遮挡期间保留，则需加入遮挡可见性判断并暂停计时；此时不得承诺所有遮挡残影有墙钟消失上界。首版按几何视野衰减设计，不再沿用旧稿 P2 的“所有暂未观测障碍都不能删除”。

`decayTime = 0.5 s` 可作为与现配置一致的试验起点，不是已验证的实车结论。衰减检查现在随每一帧重建执行——原先由 `voxelTimeUpdateThre`（1.0 s）节流的重建门槛已删除，体素格改为逐帧重建，并按异性叶（水平 0.1 m / 垂直 0.05 m）保留最新观测点，因此 0.5 s 阈值能获得相应响应时间（2026-09-17 实施）。

对连续处于有效视野的单元，terrain 历史点过期延迟约束为 `T_decay + T_scan + T_processing`，前提是这些时延均有界；这不是 costmap 标记的消失上界。costmap 清除取决于后续射线及消息处理。视野外不承诺过期时间上界；terrain 历史仍由固定空间窗口约束存储范围，窗口移出与时间过期是不同淘汰原因。不能使用旧稿 `2.5/v` 作为一般残留上界。

## R4. 高度过滤与清除输入分开

标记点高度定义为 `h = z - ground_z(x, y)`。地面估计有效时，按 `h_min <= h < h_block` 生成当帧障碍云，`h_min` 取 **0.04 m**（2026-09-17 实车确定；原写 0.1 m）。地面点可以用于 clearing，但不用于 marking。官方 ObstacleLayer 不检查 intensity，因此必须在 terrain 中实际剔除不符合高度条件的标记点，不能只修改 intensity 值。

新链路按车高 0.52 m 加 100 mm 裕量设计，使用 `h_block = ceilingClearance = 0.62 m`；不沿用额外的 `h < vehicleHeight(0.52)` 截断。0.52-0.62 m 的悬空结构仍应阻挡。该值为初始设计值，须用实车尺寸和横梁场景验收；新链路的选择不自动改变旧 global/SLAM 输出。

`terrain_returns_current` 只来自当前实际回波，做有限值、传感器有效范围、自体点等必要过滤；“不按障碍 intensity 门限过滤”不等于所有点无条件可清。Nav2 源级 `min/max_obstacle_height` 在 `ObservationBuffer` 中确实生效，必须与层级门限一并设置为覆盖预期 odom 高度的安全范围，不能把源级下界留为 0 后宣称地面参与清除。

首版平面场景复用官方 2D `ObstacleLayer`，不依赖 16 层体素柱做高度判定。2D 射线会忽略高度，跨越低矮障碍的高射线存在误清风险，必须测试并约束清除观测的几何适用范围；不能据此承诺任意三维场景的保真。若该风险在目标场景无法接受，另行评审基于官方 `VoxelLayer` 的 3D 路线，其高度最多 16 层，不能按旧稿配置 32 层。

## R5. 直接配置官方 ObstacleLayer

local 使用**一个**官方层，包含 marking 与 clearing 两个观测源。官方已有滚动窗口搬运、clearing 后 marking、footprint 清除和清图服务，无需自行实现。最终沿既有 static、obstacle、ESDF、inflation 顺序合并。

以下为待实施的配置片段，只展示 local 替换项；不是当前已生效配置。`<robot_namespace>` 沿用仓库 launch 的替换规则，frame 名称必须与实际 TF 树一致。

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "esdf_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        footprint_clearing_enabled: true
        combination_method: 1
        min_obstacle_height: -3.0
        max_obstacle_height: 3.0
        observation_sources: "terrain_marking terrain_clearing"
        terrain_marking:
          topic: <robot_namespace>/terrain_obstacles_current
          data_type: PointCloud2
          sensor_frame: front_mid360
          marking: true
          clearing: false
          min_obstacle_height: -3.0
          max_obstacle_height: 3.0
          obstacle_min_range: 0.2
          obstacle_max_range: 5.0
          observation_persistence: 0.0
          expected_update_rate: 0.2
        terrain_clearing:
          topic: <robot_namespace>/terrain_returns_current
          data_type: PointCloud2
          sensor_frame: front_mid360
          marking: false
          clearing: true
          min_obstacle_height: -3.0
          max_obstacle_height: 3.0
          raytrace_min_range: 0.0
          raytrace_max_range: 4.0
          observation_persistence: 0.0
          expected_update_rate: 0.2
```

`-3...3 m` 是平面试验使用的 odom 绝对高度安全范围，不代表离地高度，不可代替 terrain 高度分类。后续跨高程场景须重新评估。4.0 m 射线范围用于当前 5x5 m local 窗口，但几何范围足够不等于射线实际覆盖每个格。

`expected_update_rate` 单位为秒，这里 0.2 表示最大期望接收间隔，不是 0.2 Hz。两个源都配置健康检测；`observation_persistence: 0` 表示只保留最新一帧，不是自动过期。这符合“只用当帧来源”，但官方层可跨多个更新周期重用最新观测。

官方源独立缓冲带来以下边界：

- 新 clearing 到达而新 marking 未到达时，旧 marking 可能重新标记已清格；等新的障碍云或合法空障碍云替换后，再由有效射线清除。
- 超出 `expected_update_rate` 会报告 source 不新鲜、使 costmap current 状态受影响，**不会自动清空最后一帧，也不保证停止所有旧观测处理**。须验证控制器在地图不新鲜时的停控行为，不能把该参数当作历史点 TTL。
- 扫描中断不推进 terrain 衰减；不能通过持续重发旧云并刷新时间戳掩盖中断。TF 延迟、单话题丢失和恢复行为纳入验收。
- `observation_persistence` 改成正值会让多帧观测并存，仍不等于栅格 TTL；本方案不以调该值实现衰减。

射线删除的是本层贡献，不强行覆盖 master 中的静态障碍。静态地图自身的动态残影属于建图问题；即使本层已清除，static layer 仍可能让 master 保持 lethal，验收需区分二者。

## R6. 成功判据的边界

| 对象与场景 | 本方案保证或目标 |
| --- | --- |
| terrain 历史点持续在有效视野内且未重观测 | 按 R3 定义累计时间并过期 |
| costmap 旧标记被适用的当前射线经过，且没有旧 marking 重放 | 由官方射线清除，目标在正常双源更新下 0.2 s 内完成，须回放实测 |
| costmap 旧标记无射线覆盖 | 可以残留，不承诺随 terrain TTL 消失 |
| 累计 terrain 云中仍有旧障碍 | 不会作为 local marking 输入，因此不会因历史云反复刷新标记 |
| 地面和超净空结构 | terrain 在 marking 输出前过滤；不能仅依赖官方层的绝对 z 门限 |

本阶段取消“新增删除同步适配层”和“发布 retained_obstacles 完整历史快照”的要求。不采用把过期点包装成实测射线端点的方法，也不以高频整图清除服务实现逐点衰减。若无射线残留在实际导航中不可接受，再立项解决栅格时间兜底；不在本方案中隐含承诺已解决。

## R7. 实施边界与步骤（尚未实施）

1. terrain 新链路实现 R3 历史生命周期，独立记录时间，新增有效地面估计检查；保留旧输出接口。首次切换以新分支参数或独立实例隔离，避免直接修改共用历史后声称 global 数值不变。
2. 新增两个标准当帧点云输出，落实相同采集时间戳、正确 frame、合法空云与失败诊断；不新增自定义消息及快照同步协议。
3. 核对实际 launch 使用的 reality/simulation 和 controller 配置，local 插件列表中以官方 `obstacle_layer` 替换旧 `intensity_voxel_layer`，不能并存后仍让旧层消费累计云。
4. global 保持旧插件与旧输入；ESDF、inflation 保持在新障碍层之后。旧层的 `z_voxels`、`mark_threshold`、intensity 等参数不搬进官方 ObstacleLayer。
5. 先运行合成场景和 rosbag 回放，再做实车验证。回滚为 local 配置恢复旧插件及原 `terrain_map` 来源，关闭新 terrain 分支；无 Nav2 或旧插件源码需要回滚。

## R8. 验收条件与旧稿更正

| 场景 | 必须验证 |
| --- | --- |
| 障碍移走，旧点仍在 terrain 历史中，当前有有效穿透射线 | 清除本层残影；记录双源接收时间、旧缓冲重标与最终清除时延，正常更新下验证 0.2 s 目标 |
| 障碍移走，无穿透射线，但连续位于有效视野内 | terrain 历史按时过期；单独记录 costmap 可能残留，不将其描述为已被时间兜底 |
| 障碍离开视野、重新进入视野 | 离开时暂停计时；回来后继续，不因墙钟间隔直接过期 |
| 真实障碍连续有当帧观测 | 当帧 marking 恢复被射线清除的终点，持续保留 |
| 遮挡及高射线越过低障碍 | 结果符合明确的遮挡策略；评估 2D 清除是否适用，不沿用旧 P2 的无条件保留承诺 |
| 同格旧点过期但新障碍存在 | 新障碍持续由当帧 marking 标记，terrain 删除不会发送伪造的自由空间证据 |
| 地面、0.1 m 障碍、0.52-0.62 m 横梁 | 地面不标障碍，矮障碍不被绝对 z 门限误滤，横梁遵守选定净空裕量 |
| 当帧障碍数为零 | 合法空障碍云替换旧 marking 缓冲；已有栅格仍须由射线等机制清除 |
| 扫描中断、重复帧、乱序、TF 失败、单源丢包、无效地面区域 | terrain 不推进虚假衰减；验证官方旧观测重用范围、健康告警及下游停控/恢复行为 |
| 车辆移动、地图滚动、清图、生命周期重启 | 标记位置正确；验证官方缓冲在 reset 后是否重标，不能要求未实现的原子清图或仅新帧重建语义 |
| global 与 SLAM 回归 | local 插件替换本身不改旧链路；隔离的新 terrain 分支启用后继续验证旧输出一致 |
| 端到端性能 | 分别测 terrain、双话题传输及 costmap p99，报告错帧与清除延迟 |

### 旧稿更正表

下表把上文结论逐条落位（"旧稿条目"一列按语义对应旧稿编号，不是旧稿原文的引用格式）。

| 旧稿条目 | 判定 | 更正内容 |
| --- | --- | --- |
| A1 `planar_voxel_elev` 以 0 作"无地面估计"哨兵 → 误输出 | **撤回**（例子不成立） | 无候选误输出的例子被默认 `minBlockPointNum = 10` 挡住 |
| A2 下采样把观测时间戳做算术平均 | **保留** | 缺陷成立，按 R3 修复（时间不再放进参与均值降采样的 intensity） |
| A7 `transformToSensorFrame` 的 roll/pitch 符号不一致 | **撤回** | 符号符合逆旋转 |
| D20 删除 dy_obs 是"纯减法、行为不变" | **撤回** | dy_obs 删除不是行为不变 |
| D5 保留体素图会导致"无界增长" | **撤回** | 固定体素网格不会无界增长（仍应确认 `mark_threshold` 语义） |
| 旧稿 §1/D15 关于"时间清除可兜底无射线残留" | **撤回** | terrain 停止输出无法单独清除持久栅格 |
| D14 附注"源级 `min/max_obstacle_height` 对本层无效（死配置）" | **撤回** | 源级高度门限有效（在 `ObservationBuffer` 内先于 marking 生效） |
| 旧稿 §6.1/§7 只改共用插件代码、global 可"先不动" | **撤回** | 修改共用插件会同时影响 global |
| 旧稿其余数值与性能记录 | **待复核** | 仅作为历史材料，不作为实施依据 |

未被上表点名撤回的旧稿条目（§3 事实基础、§10 中除 A1/A7 外的审查项、D9/D11/D18 等）同样按最后一行"待复核的历史材料"对待。

---

<details>
<summary>历史设计稿（审查前，已被 R1-R8 替代，保留供追溯）</summary>

> **本节是审查前的旧稿，已被正文 R1–R8 取代**：只作证据与推导的追溯材料，**不作为实施依据**。
> 其中哪些条目撤回、哪些保留，以上文 R8 的更正表为准；未点名撤回的部分也按"待复核的历史材料"对待。
> 快照：`/home/rog/guganav`，分支 `terrain`，HEAD `a633a25`（2026-09-17）；外部证据 `/home/rog/navigation2`，`git describe` = `1.1.20-7-g3c3db59d`（Humble）。
> 相关回滚基线：`terrain_ray_era_backup` / tag `backup/pre-ray-rollback`（= `80d5f7a`，射线时代末态）。

---

### 1. 结论摘要

**目标**：让 costmap 里"障碍是否存在"由**观测证据**决定，而不是由 `terrain_map` 的内容决定；并让幽灵点（动态物体残影）有**有界**的消失路径。

**方案**：**删除职责分层**，而不是"搬迁"——terrain 继续负责它本来就负责的删除（时间清除、天花板/净空过滤），costmap 侧只**新增**"即时清除"这一条路径（复用 nav2 官方 `ObstacleLayer` 的 raytrace）；代价是第三方层 `pb_nav2_costmap_2d::IntensityVoxelLayer` 必须**获得记忆**。

| 职责 | 归属 | 依据 |
| --- | --- | --- |
| 地面分割 + 相对地面高度语义（intensity） | **terrain 保留**（nav2 官方无逐格地面估计） | §3.1 |
| 天花板 / 净空过滤（`ceilingClearance = 0.62`，基准是局部地面） | **terrain 保留** | D18 |
| 时间清除（`decayTime` / `voxelTimeUpdateThre`） | **terrain 保留** | D15 |
| 即时清除（按本帧观测证据判定"已消失"） | **costmap 侧新增**（raytrace） | D1–D3 |

**本阶段范围**：只解决**平面**情形；坡面相关的论证与验证项显式推迟（见 D19），改动本身与坡度正交。

**四处代码前提（缺一不可）**：

| # | 改动 | 若不做的后果 |
| --- | --- | --- |
| D4 | 删除 `updateBounds` 里每周期的 `resetMaps()` | 无记忆 ⇒ raytrace 无对象可清，参数永久是死配置 |
| D5 | 但保留每周期 `voxel_grid_.reset()` | 体素列计数只增不减 ⇒ `mark_threshold` 语义漂移、体素图无界增长 |
| D6 | 修 `updateOrigin`（窗口滚动时搬运数据） | 标记钉死在网格下标上 ⇒ 障碍相对世界拖影/错位，误差随车速累积 |
| D7 | `isClearable()` 改 `true` | "清代价地图"清不掉障碍记忆 ⇒ 排障时误判为感知故障 |

**两处 yaml 前提（静默失败风险最高）**：

- **`clearing: true`**（per-source，默认 `false`）——不开则清除列表为空，raytrace 空转，**不报错不告警**。
- **`raytrace_max_range: 4.0`**（默认 3.0 不够）——下界由窗口几何推出：`2.5·√2 + 0.05 + 0.225 = 3.811 m`。

**核心判据（本方案唯一的"新想法"）**：**marking 按 intensity 过滤，clearing 不过滤**。

- marking 必须保留 `intensity ∈ [0.1, 2.0]`，因为 `computeHeightMap` 会把 height≈0 的地面/坡面点也输出，而官方 marking 只比绝对 z（`obstacle_layer.cpp:470,476`）——不过滤则地面变 lethal。
- clearing 必须不设门限，因为射线端点是"某个被观测到的表面"，地面点同样能证明**射线路径为空**；若也过滤，近场射线大量缺失，地面附近的记忆将永远清不掉。

**明确不做**：不在坡面上用 `min/max_obstacle_height` 同时追求"坡面不误标"和"矮障碍不漏检"——绝对 z 门限下这对目标数学上矛盾（20° 坡要求下界 ≥ 1.57 m，而 0.3 m 方块的 odom z 只有 0.045 m）。

---

### 2. 成功判据

推导前先固定判据，否则无法判断方案是否成立。

| 命题 | 内容 | 判据 |
| --- | --- | --- |
| **P1** | 幽灵点有界消失 | 消失时间 ≤ 2 个 costmap 周期（10 Hz ⇒ 0.2 s） |
| **P2** | 仍然存在、只是暂时未被观测的障碍不被删 | 与"无清除"对照组一致 |
| **P3** | 地面/坡面不进入 lethal | 平面阶段：地面 lethal 面积 = 0（回归项）；坡面见 D19 |
| **P4** | 无永久残留：每类标记都有可推导的清除路径 | 残留时间上界能写成公式 |
| **P5** | 开销不劣化 | costmap update p99 在预算内 |

---

### 3. 事实基础

以下事实全部可在本仓库或本机 nav2 源码中直接复核（复核命令见 §11）。

#### 3.1 本仓库现状

| 事实 | 出处 |
| --- | --- |
| `terrain_analysis` 只发布 `terrain_map`（`PointCloud2`，frame `odom`），intensity = 离局部地面的高度 | `terrain_analysis/src/terrain_analysis_node.cpp:115-138`；`src/core/terrain_processor.cpp:313-323` |
| 输出条件：`0 ≤ height_above_ground < vehicleHeight(0.52)` ⇒ **地面/坡面点也在输出里** | `terrain_processor.cpp:319-323` |
| `terrain_map` 是跨帧累积体素云，重建由 `voxelTimeUpdateThre(1.0 s)` 或 `voxelPointUpdateThre(100)` 触发 | `config.hpp:65-68`；`terrain_processor.cpp:121-154` |
| `IntensityVoxelLayer` 继承官方 `ObstacleLayer`，但重写 `updateBounds`：每周期 `resetMaps()`、只用 marking、从不 raytrace | `pb_nav2_plugins/include/.../intensity_voxel_layer.hpp:36`；`src/layers/intensity_voxel_layer.cpp:99-201`（`resetMaps` 在 :109，marking 在 :122） |
| marking 判据 = 绝对 `pz ∈ [min,max]_obstacle_height` **且** `intensity ∈ [0.1, 2.0]` | 同上 :140-147 |
| `isClearable()` 返回 `false` | `intensity_voxel_layer.hpp:55` |
| `updateOrigin` 只改 `origin_x_/origin_y_`，**不搬数据** | `intensity_voxel_layer.cpp:203-213` |
| `resetMaps()` 同时清 2D 层网格与 `voxel_grid_` | 同上 :93-97 |
| 三个消费者都按 intensity 门限使用该点云 | `reality/nav2_params.yaml:383-384,444-445`；`:158-159`（`pointcloud_to_laserscan` 的 `min/max_intensity: 0.1/2.0`） |
| 标定：odom 原点在雷达中心 O，O 离地 255 mm ⇒ 地面 z ≈ −0.255、`vehicle_z` ≈ −0.230 | `config.hpp:83-92`；`reality/nav2_params.yaml:127` |
| 雷达相对车体前移 +0.225 m（`--x 0.225`） | `guga_bringup/launch/support/static_tf_publisher_launch.py:160` |
| local costmap：5×5 m、0.05 m、rolling、10 Hz；`obstacle_max_range: 5.0` | `reality/nav2_params.yaml:359-398` |
| 场地含坡：`BR_PROTOCOL.md` 有"飞坡前/后增益点" | `docs/BR_PROTOCOL.md:120-123` |
| `useSorting=True, quantileZ=0.2` 的作用是"让坡面相对各自局部地面接近零" | `reality/nav2_params.yaml:109-110`；`LOCAL_CONTROL_DYNAMIC_AVOIDANCE_DECISION.md:44` |

#### 3.2 nav2 官方（本机 Humble 1.1.20 源码）

| 事实 | 出处 |
| --- | --- |
| 官方顺序：`getClearingObservations` → `raytraceFreespace` → **之后**才 marking | `nav2_costmap_2d/plugins/obstacle_layer.cpp:435-447` |
| 清除半径由 `raytrace_max_range` 截断，**默认 3.0**；`raytrace_min_range` 默认 0.0 | 同上 :149-150；使用处 :701-705 |
| per-source `clearing` 默认 **false** | 同上 :141 |
| marking 只比绝对 z，**没有 intensity 概念**；`pz` 是 global frame 里的绝对高度 | 同上 :470,476；`src/observation_buffer.cpp:116` |
| 官方 `ObstacleLayer::updateBounds` **不**重置自己的网格；`resetMaps` 只在 `reset()` 里 | 同上 :756-758 |
| 层网格是记忆载体：`updateCosts` 按 `combination_method_`（1 = `updateWithMax`，switch 在 :553 / 合并调用在 :558）合并进 master | 同上 :533-566；yaml 配 `combination_method: 1` |
| 每周期 master 只重置本次 bounds 窗口（:216），然后逐层 `updateCosts`（:220） | `src/layered_costmap.cpp:216-222` |
| `raytraceLine` 的 Bresenham **对终点也执行一次动作**，而清除动作是 `MarkCell(costmap_, FREE_SPACE)` | `include/nav2_costmap_2d/costmap_2d.hpp`（`bresenham2D` 尾部 `at(offset)`）；`obstacle_layer.cpp:701-705` |
| 清除只做 2D（只用 `x,y`，忽略 `z`） | `obstacle_layer.cpp:663-666` |
| 官方 `Costmap2D::updateOrigin` 保存重叠区 → `resetMaps()` → 按新原点写回；官方 `VoxelLayer::updateOrigin` 连体素图一起搬 | `src/costmap_2d.cpp`（`updateOrigin`）；`plugins/voxel_layer.cpp`（`VoxelLayer::updateOrigin`） |
| `isClearable()` 只影响清代价地图服务；官方 `ObstacleLayer/VoxelLayer` 均返回 `true` | `src/clear_costmap_service.cpp:116`；`obstacle_layer.hpp:132` |
| 官方 `VoxelLayer` **只有 `max_obstacle_height`，没有 `min_obstacle_height`**；低于 `origin_z` 的点被夹到底层体素仍会标 lethal | `plugins/voxel_layer.cpp:67,192,213-228` |
| 官方有 `DenoiseLayer`（`minimal_group_size: 2`、`group_connectivity_type: 8`），已注册 | `plugins/denoise_layer.cpp`；`costmap_plugins.xml:18` |
| 官方层**没有任何时间衰减（清除）机制**：整个 `nav2_costmap_2d` grep `decay` 零命中 | `grep -rn "decay" plugins/ include/ src/`（无输出） |
| `observation_persistence` **不是**标记衰减，只是"观测在缓冲区里保留多久"（`observation_keep_time`），决定 marking/clearing 用哪些观测 | `plugins/obstacle_layer.cpp:139,155-156`；`src/observation_buffer.cpp:54,63` |
| 官方层**没有天花板/净空语义**：只有绝对 z 的 `max_obstacle_height`，无法表达"距**局部地面** ≥ 某值则可从下方通过" | `plugins/obstacle_layer.cpp:476`；terrain 侧对照 `terrain_processor.cpp:303-308` |
| 时间衰减只有**第三方**层有：STVL 的 decay、社区 `nonpersistent_voxel_layer` | 外部仓库，非官方 |

---

### 4. 逐句推导

#### D1 幽灵点是"设计必然"，不是 bug

**【事实】** `terrain_map` 是跨帧累积（`config.hpp:65-68`），costmap 侧每周期从它全量重画（`intensity_voxel_layer.cpp:109-177`）。

**【推得】** 障碍从 costmap 消失的唯一路径是"terrain 体素被重建"，其触发条件（1 s 或累计 100 点）与观测无关。

**【所以】** 要实现 P1，必须新增一条**按观测判定**的删除路径。这是问题根源，不是参数没调好。

#### D2 删除路径放 costmap 侧——理由是"错误是否可自愈"

**【事实】** 两条候选：感知侧（在 `terrain_analysis` 内按本帧射线穿透清历史点）或 costmap 侧（官方 raytrace）。

**【推得】**

- 感知侧删除**不可自愈**：点被删后要等该体素**再次重建**才回来。这与实测形态一致——0.5 m **静止**障碍、车开走 6 m、输出点 404 → 90（被削掉 3/4）。
- costmap 侧删除**下一周期自愈**：被清的是层网格里的标记，只要下一帧仍观测到该障碍，marking 会立刻重新标记（前提是感知侧不再删点）。

**【所以】** 清除放 costmap 侧。**这是本方案最重要的取舍：把误删代价从"点消失、需等重建"降为"标记缺失一个周期"。**

#### D3 用官方 raytrace，不自研

**【事实】** 官方已实现 `getClearingObservations` + `raytraceFreespace`（`obstacle_layer.cpp:435-442`），2D 推进，长度由 `raytrace_max_range` 截断（默认 3.0）。

**【推得】** ① 复用成熟实现；② 代价是"栅格数"而非"点数 × 高度带"（自研 3D 版实测单独约 2.8 ms/帧 @12000 点）；③ **默认 3 m 截断天然满足 P2 的远场部分**——远场标记根本不会被射线碰到。

**【所以】** 清除动作 = 官方 raytrace；自研射线清障整条路线可以永久放弃。

#### D4 必须有记忆 ⇒ 删掉每周期 `resetMaps()`

**【事实】** 官方 `updateBounds` 不重置自己的网格（`resetMaps` 只在 `reset()`，:756-758）；fork 在 :109 每周期调用。

**【推得】** raytrace 的语义是"清掉**先前标记**"。每周期先清空 ⇒ 不存在先前标记 ⇒ raytrace 无对象可清，属死代码（这也解释了为什么 yaml 里即便写了 `raytrace_max_range` 也毫无作用）。

**【所以】** 删除 `:109` 的 `resetMaps();`。

#### D5 但体素图必须继续每周期清空（只删一半）

**【事实】** fork 的 `resetMaps()` 覆盖 2D 层网格 + `voxel_grid_.reset()`（:93-97）；`voxel_grid_` 参与 `markVoxelInMap(...)` 的返回值（:170）。

**【推得】** 连体素一起保留 ⇒ 列计数只增不减，`mark_threshold` 语义随时间漂移；体素图无界增长 ⇒ 内存增长，`publish_voxel_map` 时话题膨胀。

**【所以】** 把 `:109` 换成 `voxel_grid_.reset()`。记忆只放 2D 层网格，体素图保持"每周期快照"原语义。

#### D6 窗口滚动必须搬运数据 ⇒ 换掉 fork 的 `updateOrigin`（最易漏）

**【事实】** fork 的 `updateOrigin`（:203-213）只改 `origin_x_/origin_y_`；官方 `Costmap2D::updateOrigin` 会保存重叠区 → 重置 → 写回；官方 `VoxelLayer::updateOrigin` 连体素图一起搬。

**【推得】** 现在不搬数据无害（每周期重画）。一旦有记忆，标记钉死在**网格下标**上而世界原点在移动 ⇒ 障碍相对世界拖影/错位，误差随车速线性累积（3 m/s、10 Hz = 每周期 0.3 m）。

**【所以】** 删除或改写该覆盖，使基类搬运逻辑生效。**症状最像"传感器噪声"，最难归因。**

#### D7 清代价地图服务必须能清掉记忆 ⇒ `isClearable()` 改 `true`

**【事实】** fork 返回 `false`（`hpp:55`）；`clear_costmap_service.cpp:116` 只对 `isClearable()` 为真的层调用 `reset()`。

**【推得】** 有记忆后，执行"清代价地图"不会清掉障碍记忆 ⇒ 出现"清了还在"，且会被误判为感知故障。

**【所以】** 改 `true`。

#### D8 marking 保留 intensity 门限 ⇒ 形成"marking 过滤、clearing 不过滤"的非对称

**【事实】**

- `computeHeightMap` 输出 `0 ≤ height < vehicleHeight` 的点（`terrain_processor.cpp:319-323`）⇒ 地面/坡面点（intensity≈0）也在 `terrain_map` 里。
- 官方 marking 只比绝对 z（`obstacle_layer.cpp:470,476`），无 intensity 概念。

**【推得】** 让官方 marking 直接消费 `terrain_map` ⇒ 地面与坡面变 lethal ⇒ 违反 P3。

**【关键推论】** 清除侧**不需要**门限：射线端点是"被观测到的表面"，地面点同样证明路径为空；若也过滤，近场射线大量缺失 ⇒ 地面附近的记忆永远清不掉 ⇒ 违反 P4。

**【所以】** 非对称设计：**marking 按 `intensity ∈ [0.1, 2.0]` 过滤；clearing 使用全部点。**

**【连带结论】** 这条否定了"直接退回纯官方 `ObstacleLayer`"的捷径——除非把 0.1 m 门限上移到感知侧（见 §9 备选 B）。

#### D9 顺序必须 clearing → marking

**【事实】** `raytraceLine` 的 Bresenham 实现对终点**再执行一次动作**（`bresenham2D` 尾部 `at(offset)`），而清除动作是 `MarkCell(costmap_, FREE_SPACE)`；官方顺序是 raytrace 早于 marking。

**【推得】** 清除会把**射线终点所在格**也清成 FREE，而终点正是本帧观测到的障碍表面。若把 clearing 插在 marking 之后，本帧刚标上的障碍会被自己那条射线的终点清除 ⇒ 所有障碍每周期被清一次（表现为闪烁/几乎全消失）。

**【所以】** clearing 必须插在 marking 循环**之前**。这与历史记录 `f5e0771 "修正射线清理的两处失效——阶段顺序与判据维度"` 是同一机理。

#### D10 `clearing` 必须显式打开

**【事实】** per-source `clearing` 默认 `false`（`obstacle_layer.cpp:141`）；`getClearingObservations` 只返回 `clearing=true` 的源。

**【推得】** 不写 ⇒ 清除列表为空 ⇒ raytrace 空转，**不报错不告警** ⇒ 会误判为"方案无效"并回到自研路线。

**【所以】** yaml 必须显式加 `clearing: true`。**本方案静默失败风险最高的一处。**

#### D11 `raytrace_max_range` 下界由窗口几何推出（默认 3.0 不够）

1. 标记只能产生在层网格内，越界点被 `worldToMap3D` 丢弃（`intensity_voxel_layer.cpp:160-167`）；
2. local costmap 5×5 m、rolling、车居中 ⇒ 窗口内任意点距车最远 = 对角 = `2.5·√2 = 3.536 m`；
3. `updateOrigin` 按 0.05 m 取整 ⇒ +0.05 m；
4. 清除半径以**雷达原点**度量（`raytraceFreespace` 用 `obs.origin_`），雷达相对车体前移 0.225 m；
5. ⇒ 最坏 `3.536 + 0.05 + 0.225 = 3.811 m`。

**【推得】** 取 **4.0**。用默认 3.0 时窗口四角标记永远清不掉（违反 P4），症状是"角上有幽灵点不消失"。

**【自洽性】** 4.0 < `obstacle_max_range`(5.0) 不构成缺口，因为标记根本进不到 3.54 m 以外；两条不等式互相印证。

#### D12 `raytrace_min_range` 取 0

**【事实】** 车体内部清理由 `footprint_clearing_enabled: true` → `updateFootprint` → `setConvexPolygonCost(..., FREE_SPACE)` 负责（:63-79）。

**【所以】** 取 0.0，避免与既有机制重叠。

#### D13 `mark_threshold` 保持 0 —— 不要用它做噪声抑制

**【事实】** `pz < origin_z_` 时点被**夹到** `origin_z_` 那一层（:161-164）；`origin_z: 0.0`、`z_resolution: 0.05`、`z_voxels: 16` ⇒ 体素柱只覆盖 odom z ∈ [0, 0.8] m；地面在 odom z ≈ −0.255。

**【推得】** 平地上 0.1~0.3 m 的障碍，其点大多 z < 0.05 ⇒ 夹进第 0 层，**只占 1 个高度层** ⇒ `mark_threshold` 从 0 提到 1 会整类丢掉低矮真障碍。

**【所以】** 噪声抑制留在感知侧（`minBlockPointNum = 10` 已在做），costmap 侧不动。

**【若将来要用】** 前置条件是重设高度柱：`origin_z ≤ −0.7`，且 `z_voxels × z_resolution ≥ 0.7 + 0.62 + 余量 ≈ 1.6 m` ⇒ 0.05 m 分辨率下 `z_voxels ≥ 32`。属另一课题。

#### D14 `max_obstacle_height: 2.0` 的现存缺陷必须一并修

**【事实】** fork 在 `pz > max_obstacle_height_` 时**静默丢点**（:140），且读的是**层级**参数（:41）；地面高度 `z_ground(d) = −0.255 + d·tanθ`。

**【推得】** 5 m 处地面达到 2.0 m 的坡角：`tanθ = (2.0 + 0.255)/5 = 0.451` ⇒ **θ = 24.3°**。即坡角超过 24.3° 时 5 m 外的坡面点（含其上障碍）被丢弃。

**【所以】** 把 z 上下界放宽为"安全网"（如 −3.0 … +3.0），语义上界交给 `intensity ≤ ceilingClearance = 0.62`。否则：无记忆时只是当帧漏检，有记忆后会变成"坡上障碍时隐时现"。

**【顺带发现的死配置】** yaml 里 per-source 的 `min/max_obstacle_height` 对本层**无效**（本层用层级值，`hpp:68` + `cpp:41`）；而 per-source 的 `obstacle_max_range/min_range` **生效**（:129-130,155）。改参数别改错位置。

#### D15 三类标记不会被 raytrace 清 ⇒ 残留上界公式

| 情形 | 是否可清 |
| --- | --- |
| (i) 位于 `raytrace_max_range` 之外 | 由 D11 已排除（标记进不到 3.54 m 以外） |
| (ii) 射线永不穿过（遮挡、盲区、坡顶背面） | **物理上不可清** |
| (iii) 车静止且窗口不滚动 | **不可清** |

**【推导】** 残留时间上界 = 窗口半宽 / 车速 = `2.5 / v`：

| 车速 | 残留上界 |
| --- | --- |
| 1.0 m/s | 2.5 s |
| 2.5 m/s | 1.0 s |
| 3.0 m/s | 0.83 s |
| 0（静止） | **无上界** |

**【所以】** 必须**保留感知侧的时间路径**（`decayTime` / `voxelTimeUpdateThre`）。它管"感知侧不再输出"，raytrace 管"costmap 侧立即清除"，**两者互补且不重叠**。这修正了"射线清障 vs 按时间遗忘"的二选一框架：那从来不是二选一，是分工——把时间路径关掉，等于把 (ii)(iii) 的兜底拆了。

#### D16 可选兜底：`DenoiseLayer`

**【事实】** 官方 `DenoiseLayer` 在本机 Humble 1.1.20 源码树中已存在并注册；官方教程明示它会误删真实的细小障碍。

**【所以】** 只在实测出现"孤立残点"时才加，且必须单独回归 P2（它与 P2 直接对抗）。

#### D17 若 (ii) 类残留在实车不可接受 ⇒ 唯一现成选项是 STVL（第三方）

它做 3D 体素级清除 + 时间衰减，正好覆盖 (ii)。代价：引入第三方层、重建参数面、独立维护。**记为备选，不作为第一步。**

#### D18 天花板/净空过滤必须留在 terrain（nav2 层不自带）

**【事实】**

- terrain 的净空过滤：`height_above_ground >= ceilingClearance(0.62)` 的点**不输出**为障碍（`terrain_processor.cpp:303-308`），基准是**该格局部地面**，注释明确写的是"车辆可从下方通过（隧道场景）"。
- 官方 nav2 层只有绝对 z 的 `max_obstacle_height`（`obstacle_layer.cpp:476`），没有任何"净空"概念，也没有逐格地面。
- fork 也没有：它用层级 `max_obstacle_height`（`intensity_voxel_layer.cpp:41,140`），同样是绝对 z。

**【推得】**

1. 它是一个**净空过滤**，不是"高度过滤"：1 m 高的墙仍会由 0.62 m 以下那段标成 lethal，所以不会出现"高障碍整体消失"。
2. 因此该语义**必须**由 terrain 承担，且**必须保留**——这与"保留时间清除"是同一类结论：terrain 的删除/过滤职责不变。
3. 有记忆之后，**"单帧误 emit 一个天花板点"的代价被放大**：现在每周期 `resetMaps()` 会兜底，误标只活一个周期；有记忆后它会一直留到"有射线穿过该格"。⇒ 该过滤器的回归测试价值上升，不能因为"costmap 会重画"就放松。
4. 与 D14 方向一致：把 costmap 侧 z 上界放宽为安全网（−3.0…+3.0），正是为了**不让两套门限互相掩盖**——天花板语义唯一归属 terrain。你们 README 里"`CEILING_CLEARANCE` 曾同时承担两种语义、调其一必动另一"就是双门限掩盖问题的前例。

**【平面阶段必须说清的一条边界，避免把结论用错】**
平地时 `ground_z ≈ −0.255` 是常数，因此 `max_obstacle_height = −0.255 + 0.62 = +0.365` 在数学上**可以近似等价**于 terrain 的净空过滤——两者只差一个参考系常量。所以"nav2 无法表达天花板语义"这句话**只在坡面/地面高度变化时成立**；平面阶段它可以用参数近似表达。

即便如此，平面阶段仍不建议把该门限搬到 costmap：它依赖 odom z 的绝对标定，而 `vehicle_z` 基准目前尚未实测确认（`docs/TODOLIST.md:9`，登记为"待实测验证"）。标定不确定时，挂在地面上的门限比挂在 odom 原点上稳。

#### D19 本阶段范围：只解决平面

**【推得】** 本方案的四处代码改动（D4–D7）与两处 yaml 前提（D10、D11）都是"记忆 / 清除 / 窗口搬运 / 可清"，**与坡度正交** ⇒ 坡面推迟**不会导致返工**，坡面阶段不需要重做这四处。

| 本阶段**推迟**（登记，不在范围内） | 本阶段**不推迟**（平面上同样必须） |
| --- | --- |
| D14 的坡角阈值推导（24.3°）与"绝对 z 门限在坡面上无解"的论证 | intensity 门限（地面点不被标 lethal，与坡度无关） |
| §8 验证矩阵中 P3 的坡面场景 | `clearing: true` 与 `raytrace_max_range = 4.0`（由窗口几何推出，与地形无关） |
| 全局 costmap（阶段 3） | 时间清除保留（静止/遮挡情形与坡度无关，见 D15） |

**【前置条件提醒】** 若"先解决平面"被理解为"平地上可以不用 terrain、直接用官方层 + 绝对 z 门限"，则该路线**唯一的立足点**是 odom z 的绝对标定，而 `vehicle_z` 基准尚未实测确认（`docs/TODOLIST.md:9`）；另外两个消费者（`pointcloud_to_laserscan` 的 SLAM 输入、`EsdfLayer`）仍依赖 terrain 输出的 intensity 语义。在标定闭合前，平面阶段仍建议保留 terrain 作前端。

#### D20 dy_obs（动态障碍）机制的去留：删掉机制，保留需求

**【事实】** 当前代码（`a633a25`）里 dy_obs 是"启用但惰性"的：

- `detectDynamicObstacles`（`terrain_processor.cpp:199-240`）与 `filterDynamicObstaclePoints`（:242-267）用**同一判据**（`scan_angle > min_dy_obs_angle`）、遍历**同一批几何点**（`terrain_cloud` 由 `laser_cloud_crop` 累积而来，本帧点必然同时在两边）；
- `filter` 把 `planar_voxel_dy_obs[cell]` 直接置 0（:264），`detect` 的计数在同帧被清掉；
- `computeHeightMap` 的抑制分支在 :309-311。

**【项目已实测（`e623154` 提交信息）】**

| 项 | 数据 |
| --- | --- |
| 同帧自我抵消 | detect 标记 **2236 格** → filter 后只剩 **6 格**（**99.7% 抵消**） |
| 残留 6 格的实际危害 | 被抑制的候选点共 **56 个，全部是离地高度 0.000 的地面点**，本就过不了输出判据；**误删真实障碍 = 0 个** |
| 参数语义 | `minDyObsPointNum=1`（一个点抑制整格）；VFOV 检查用的是雷达自身视场（近似恒真）；近距分支 `+= min_dy_obs_point_num` 是**用点数阈值冒充计数增量**（:219-222） |
| 根因 | `planar_voxel_dy_obs` 每帧 `fill(0)`，**没有跨帧证据**，而"动态"判定本质需要跨帧运动证据 |

**【推得】新方案让它更没必要**：现在"抑制输出" ≡ "从 costmap 移除"（因为每周期 `resetMaps()` 全量重画）；**一旦 costmap 有记忆（D4），这个等号失效**——抑制输出只是"不再刷新标记"，清除要等射线。所以 dy_obs 连名义上的那点清除作用，在新架构下也不存在了。

**【所以】必须把"机制"和"需求"分开：**

| 需求 | 谁承接 | 状态 |
| --- | --- | --- |
| **A. 幽灵点/残影**（已消失的障碍要离开地图） | 本方案的 costmap raytrace（观测证据） | 被覆盖，且比启发式硬 |
| **B. 动态障碍要被看见并避开** | marking 输出 | 覆盖。注意 dy_obs 是**反方向**（抑制输出），从来不该承担这件事 |
| **C. 动态物体污染持久地图**（全局 costmap / SLAM 留住伪静态轨迹） | **无人承接** | **未解**。`TODOLIST.md` 已登记；dy_obs 不是它的解（无 ID、无关联、无速度） |

**【结论】** 机制**删除**，需求保留。它与本方案解耦，属"纯减法、行为不变"，可以**独立先做**（节点参数 29 → 22，审查面与验证面的干扰变量同时减少）。

**【两个操作提醒】**

1. **摘不过来**：`git show e623154 | git apply --check` 会在 `terrain_processor.hpp`、`terrain_processor.cpp`、`terrain_analysis_node.cpp`、`test_algorithm.cpp` 四处冲突（射线时代中间重排过这些文件）。需手工做，性质仍是纯减法。
2. `e623154` 里混了**无关清理**：同时删了 `launch/terrain_analysis.launch`（其 exec 名 `terrainAnalysis` 早已不存在、参数值与现状全矛盾、无任何引用）与 `CMakeLists.txt` 中对应 install。摘的时候要分开，别混进同一次评审。

---

### 5. 参数推导汇总

| 参数 | 现值 | 建议值 | 推导依据 |
| --- | --- | --- | --- |
| `terrain_map.clearing` | 未设置（false） | **true** | D10：不开则静默失效 |
| `terrain_map.raytrace_max_range` | 未设置（3.0） | **4.0** | D11：`2.5√2 + 0.05 + 0.225 = 3.811` |
| `terrain_map.raytrace_min_range` | 未设置（0.0） | 0.0 | D12 |
| `min_obstacle_height`（层级） | 0.0 | **−3.0** | D14：安全网；语义上界交给 intensity |
| `max_obstacle_height`（层级） | 2.0 | **+3.0** | D14：放宽为安全网；天花板语义由 terrain 的 `ceilingClearance` 唯一承担（D18） |
| `obstacle_max_range` | 5.0 | 不变 | D11 自洽性检查 |
| `mark_threshold` | 0 | 不变 | D13：0.05 m 体素柱无法可靠分辨，勿用作噪声抑制 |
| `z_voxels` / `origin_z` / `z_resolution` | 16 / 0.0 / 0.05 | 不变 | D13：改动前置条件不足即会误删低矮障碍 |
| `ceilingClearance`（**terrain 侧**，非本层参数） | 0.62 | 不变 | D18：净空语义唯一归属 terrain；勿与 costmap 侧 z 上界形成双门限互相掩盖 |
| `decayTime` / `voxelTimeUpdateThre`（**terrain 侧**） | 0.5 / 1.0（yaml）；2.0 / 2.0（代码默认） | 不变 | D15：时间清除是 (ii)(iii) 类残留的唯一兜底，必须保留 |

---

### 6. 改动清单（设计内容，本文未实施）

#### 6.1 代码：`src/guga_planner/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp`

```cpp
 void IntensityVoxelLayer::updateBounds(...)
 {
   if (rolling_window_) {
     updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
   }

-  resetMaps();                 // 每周期清空 → 无记忆 → raytrace 无对象可清（D4）
+  voxel_grid_.reset();         // 只清体素图，保住 mark_threshold 语义与内存上界（D5）

   if (!enabled_) return;
   useExtraBounds(min_x, min_y, max_x, max_y);

+  // 清除必须在标记之前：raytraceLine 会把射线终点所在格也清成 FREE，
+  // 而终点就是本帧观测到的障碍表面（D9）。顺序反了，障碍每周期被自己清掉一次。
+  std::vector<Observation> clearing_observations;
+  if (!getClearingObservations(clearing_observations)) {
+    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 2000,
+                         "clearing observations unavailable — check <source>.clearing=true");
+  }
+  for (const auto & clearing_obs : clearing_observations) {
+    raytraceFreespace(clearing_obs, min_x, min_y, max_x, max_y);   // protected virtual
+  }
+
+  // 非对称（D8）：clearing 用全部点（地面点也证明路径为空），
+  //              marking 仍按 intensity ∈ [min,max] 过滤（否则地面/坡面变 lethal）。
   bool current = true;
   std::vector<Observation> observations;
   current = getMarkingObservations(observations) && current;
```

另两处：

| 位置 | 改动 | 依据 |
| --- | --- | --- |
| `intensity_voxel_layer.cpp:203-213` | 删除 `updateOrigin` 覆盖（或转调 `Costmap2D::updateOrigin`），使窗口滚动时搬运 2D 网格数据 | D6 |
| `intensity_voxel_layer.hpp:55` | `isClearable()` → `return true;` | D7 |

#### 6.2 yaml：**4 份文件必须同步**

`reality/nav2_params.yaml`（local :371-398、global :432-459）、`reality/controller/mppi.yaml`（:134、:193）、`reality/controller/pid.yaml`（:93、:152），simulation 侧同理。

```yaml
      intensity_voxel_layer:
        plugin: pb_nav2_costmap_2d::IntensityVoxelLayer
        min_obstacle_height: -3.0      # 由 0.0 放宽：安全网，不是语义门限（D14）
        max_obstacle_height: 3.0       # 由 2.0 放宽：否则坡角 >24.3° 时 5 m 外障碍被丢（D14）
        observation_sources: terrain_map
        terrain_map:
          data_type: PointCloud2
          topic: <robot_namespace>/terrain_map
          sensor_frame: front_mid360
          clearing: true               # 新增，默认 false；不开则 raytrace 空转（D10）
          raytrace_max_range: 4.0      # 新增；≥ 3.536 + 0.05 + 0.225（D11）
          raytrace_min_range: 0.0      # 新增；车体内清理由 footprint_clearing 负责（D12）
          obstacle_max_range: 5.0      # 不变
          obstacle_min_range: 0.2      # 不变
```

只改一处会让对照组跑在另一份配置上——与"工作副本被回退导致反向结论"是同类坑。
**全局 costmap 先不动**（链路是 `terrain_map_ext`，而 ext 只做 4 m 半径裁剪，语义需先理顺，见 §7 阶段 3）。

---

### 7. 分阶段落地与回滚

| 阶段 | 内容 | 回滚点 |
| --- | --- | --- |
| **-1（可选，独立）** | **D20：删除 dy_obs 机制**（纯减法、行为不变）。先做可让 P1/P2 的对照组少一个干扰变量 | 单 commit；`e623154` 可作对照 |
| 0 | 只加 yaml（`clearing: true` + 两个 raytrace 值） | 参数级，无害 |
| 1 | 代码 D4/D5/D6/D7 四处 | 单 commit |
| 2 | D10–D14 参数与安全网放宽 | 单 commit |
| 3 | 全局 costmap + `terrain_map_ext`（需另行推导半径语义） | 单独评审 |
| **并行** | §10 审查清单（A1–A9）：其中 **A1、A2 与本方案直接相关**，建议在阶段 1 之前或同时处理 | 各自独立 |

回滚基线：`a633a25`；射线时代全部提交在 `terrain_ray_era_backup` / `backup/pre-ray-rollback`。

---

### 8. 验证矩阵

| 命题 | 场景 | 指标 | 通过判据 | 现有基线 |
| --- | --- | --- | --- | --- |
| P1 | 0.3 m 方块 2 m→5 m 平移，车静止 | 原位残点在 costmap 上的 lethal 格数 | ≤ 0.2 s 归零 | 仅关射线时 25 点一直不降，直到 +0.8 s 由时间路径一次清 0 |
| P2 | 0.5 m **静止**障碍，车开走 6 m | 该处保留点数/格数 | **等于**"无清除"对照（404） | 开自研射线时退化为 90 |
| P3 | **平面**（本阶段）：平地地面点不被标 lethal；坡面场景按 D19 推迟 | 地面 lethal 面积 | 地面 lethal = 0 | intensity 门限下已为 0，做回归 |
| P3b | 天花板/净空（保留项）：隧道/横梁下方可通行 | 距地 ≥0.62 m 的结构是否被标 lethal | 其**下方**可通行；0.62 m 以下部分仍标 lethal | 由 terrain `ceilingClearance` 承担（D18），本方案不改 |
| P4 | 车静止 + 障碍消失 + 遮挡 | 标记残留时间 | 实测记录；> 10 s 即判定需 D16/D17 | 新指标，无基线 |
| P5 | 12000 点合成帧 + 实车 | costmap update p99、CPU、raytrace 单独耗时 | p99 在 100 ms 预算内可接受占比 | terrain 管线 2.51~4.67 ms 作参照 |
| 回归 | 全局 costmap 逐点对比 | 与改动前差异 | 必须 0 差异 | — |

---

### 9. 本方案不解决什么

> **先说清本方案不改动什么**：terrain 的删除/过滤职责**一条都不减**——时间清除（D15）、天花板/净空过滤（D18）、地面分割与相对高度语义（§3.1）全部保留。本方案只在 costmap 侧**增加**"即时清除"这一条路径。

1. **(ii) 遮挡/盲区类残留**：物理上不可清，需 D16 或 D17 兜底。这是本方案唯一的硬缺口。
2. **动态障碍跟踪与预测**：官方与第三方都没有，本方案不涉及。
3. **坡度/可通行性**：本方案不产出地形描述，与 `LOCAL_CONTROL_DYNAMIC_AVOIDANCE_DECISION.md` 要求的 descriptor 是两件事。
4. **坡面**：本阶段明确不覆盖，推迟项与不推迟项见 D19。注意 D18 里那条边界——"nav2 无法表达天花板语义"只在坡面/地面高度变化时成立，平面上它可以被参数近似表达。
5. **全局 costmap**：留在阶段 3。

**备选 B（备记，不在本次范围）**：把 0.1 m 门限上移到 `terrain_analysis`（只输出通过门限的点），即可用**纯官方** `ObstacleLayer`。代价是 `terrain_map` 语义改变，而它还有两个消费者按 intensity 过滤（`IntensityVoxelLayer`、`pointcloud_to_laserscan`），要一起改，改动面反而更大。

**明确不要做（坡面阶段的前置警告，本阶段不触发）**：不在坡面上用 `min/max_obstacle_height` 同时追求"坡面不误标"和"矮障碍不漏检"——20° 坡需要下界 ≥ 1.57 m，而 0.3 m 方块的 odom z 只有 0.045 m，两者不可兼得，与调参无关。

---

### 10. 附加审查：terrain 包其余部分的逻辑问题

> 审查范围：`src/guga_perception/terrain_analysis/` 下两个包（`terrain_analysis`、`terrain_analysis_ext`）的算法与节点逻辑。
> 严重度：**高** = 会导致错误输出且难归因；**中** = 在特定条件下产生错误输出；**低** = 语义/浪费/健壮性。
> 每条给出"是否与本方案相关"——A1、A2 **直接相关**（有记忆后误输出会持久化）。

#### A1（中高）`planar_voxel_elev` 用 0 当"无地面估计"哨兵，而 odom z = 0 是合法高度

**【证据】** `resetPlanarVoxels` 每次 `fill(0)`（:405-411）；`elevateByQuantile` 在无候选时直接 `return`，值保持 0（:438-440）；`computeHeightMap` 用 `ground_z = planar_voxel_elev[cell]` 直接相减，**没有任何有效性判断**（:295-296）。

**【推得】** 无候选格的 `ground_z` 被当成"地面在 odom z = 0"，而实际地面在 odom ≈ **−0.255**（偏了 0.255 m，且方向朝上）。于是 `height_above_ground` 退化为绝对 z，产生两个方向的错误：

| 情形 | 真实值 | 输出 | 后果 |
| --- | --- | --- | --- |
| 真实离地 **0.3 m** 的矮障碍（本应输出） | 0.3 | z ≈ 0.045 → **0.045** | < 下游门限 0.1 ⇒ **漏检** |
| 真实离地 **0.755 m**（> `ceilingClearance` 0.62，本应"可从下方通过"） | 0.755 | z ≈ 0.5 → **0.5** | < 0.62 ⇒ **被当作障碍输出**（绕过净空规则） |

**【何时发生】** 该 planar 格在 3×3 膨胀后仍无候选点 ⇒ 局部稀疏、或被 `ingestLaserCloud` 的裁剪带裁掉（README 风险 1：近处带宽仅约 0.2 m）。不是罕见路径。

**【与本方案关系】直接相关**：有记忆后，这类误输出会一直留到被射线清除（无记忆时只活一个周期）。

**【建议】** 引入显式有效性（`planar_voxel_elev` 用 NaN 或增加 bitset 标记"有估计"），无估计的格不输出点；同时厘清它与 `minBlockPointNum` 的分工。

#### A2（中）下采样把"观测时间戳"做算术平均，而语义需要"最新"

**【证据】** `updateTerrainVoxels` 的顺序是**先降采样、后判过期**（:138-149）；PCL `VoxelGrid` 默认 `downsample_all_data_ = true`（`/usr/include/pcl-1.12/pcl/filters/voxel_grid.h:201`），走 `CentroidPoint<PointT>` 分支（`impl/voxel_grid.hpp:416-424`），而 `CentroidPoint` 对 intensity 字段做**平均**（`pcl/common/centroid.h:981` 的字段表：Intensity → Average）。这里的 intensity 存的正是"该点被观测的时刻"。

**【推得】** 一个叶子里混有新旧观测时，代表点的"观测时间"是**均值**，既非最新也非最旧。对**稀疏**表面（每格每帧只落几个点 → 攒够 100 点要接近 1 s）⇒ 均值被拉老 ⇒ `keepTerrainVoxelPoint` 判过期 ⇒ **整叶代表点被删，包含刚刚观测到的那个点**。这就是"仍然存在的障碍被删"（P2 类失败）的一个独立成因。

**【为什么至今没暴露】** 代码默认 `noDecayDis = 4.0` 会豁免整个 5.5 m 窗口（近场全豁免）→ 掩盖；但 reality yaml 是 `noDecayDis: 0.0`（豁免为零）→ 真实生效。

**【建议】** 把"判过期"移到降采样**之前**（先删旧点，再对存活点降采样 → 均值只混新鲜时间戳）。
**注意**：`setDownsampleAllData(false)` **不是**解法——PCL 在 false 分支只写 XYZ（`impl/voxel_grid.hpp:404-412`），intensity 不会被赋值，全变 0，等价于"所有点都过期"。

#### A3（中低）`minBlockPointNum` 计的是 3×3 膨胀后的样本数

**【证据】** `addToPlanarNeighborhood3x3` 把每个点写进 9 个格（:413-433）；`computeHeightMap` 用 `planar_point_elev[cell].size()` 判 ≥ `minBlockPointNum`（:318-320）。

**【推得】** 实际判据是"**0.6 m × 0.6 m 邻域内** ≥10 点"，比"本格 ≥10 点"宽松约 9 倍；而 README 的功能表把它描述为"planar voxel 的最小有效点数"。同一容器同时承担**地面候选集合**与**障碍密度计数**两种语义。

**【建议】** 要么显式接受并修 README/注释，要么给密度判据独立计数。

#### A4（中低）`intensity` 字段被复用三次，任何中间处理都会改变过期语义

**【证据】** `ingestLaserCloud:55` 写入时间戳 → `updateTerrainVoxels:145-147` 用它判过期 → `computeHeightMap:322` 覆盖为离地高度。原始雷达强度丢失。

**【推得】** 这不是错误，但**A2 就是它的一个实例**——凡是对 intensity 做算术（下采样、滤波、体素化）都会同时改变"观测时间"的语义。这块是复发风险区。

**【建议】** 修 A2 时顺便把"观测时间"与"输出高度"分成两个通道（或在类型上区分），与 D8 里"intensity 语义"的结论合并考虑。

#### A5（低）采集窗与 planar 网格不匹配：5.1–5.5 m 环带是纯浪费

**【证据】** `collectTerrainCloud` 取 ±5.5 m（11×11 个 1 m 格，:156-172）；planar 只有 ±5.1 m（`grid.hpp`：51 × 0.2 m）；`estimateTerrainGround`、`detectDynamicObstacles`、`computeHeightMap` 都先做 planar 越界判定并丢弃。

**【推得】** 约 **14%**（面积比 `(5.5²−5.1²)/5.5²`）的采集点进入 3 个循环后被丢弃 ⇒ 纯 CPU 浪费；同时"输出覆盖 ±5.1 m"与"配置声称 ±5.5 m"口径不一致，容易被误读成"漏检"。

**【建议】** 把两者对齐（改一处即可），或在注释/README 里写明。

#### A6（低）帧丢失路径：`ingest` 会覆盖尚未处理的帧

**【证据】** 节点用 10 ms 定时器（`terrain_analysis_node.cpp:118-119`），`processOnce` 仅在 `hasPendingCloud()` 时运行（:122-130）；`ingestLaserCloud` 每次 `laser_cloud_crop->clear()` 重建（:46-59）。

**【推得】** 若两帧扫描落在同一个定时器周期内（输入 > 100 Hz），前者的裁剪结果被覆盖 ⇒ 该帧的点**永不进入体素**（静默丢数据，地图上留下空洞）。当前 Livox 50 Hz 安全；一旦改配置（点云合并、多雷达、换更高频话题）就会静默劣化。

**【建议】** 把驱动周期与输入频率解耦（事件驱动或队列计数），至少在丢弃时计数并告警。

#### A7（低）`transformToSensorFrame` 的 roll 符号与 pitch 约定不一致（随 D20 一起删除）

**【证据】** pitch 用标准形式 `x' = x cosP − z sinP`、`z' = x sinP + z cosP`（:369-372）；roll 写成 `y' = y cosR + z sinR`、`z' = −y sinR + z cosR`（:374-377），即绕 x 转 **−R**，符号与 pitch 相反。

**【推得】** 目前只被 dy_obs 使用（且 dy_obs 惰性）⇒ 实际无影响；但若有人复用该函数，会得到镜像的横滚。

**【建议】** 随 dy_obs 一起删除（`e623154` 的删除清单已包含它和 `SensorPoint`）。

#### A8（低）输出 frame 硬编码 `"odom"`，不校验输入 frame

**【证据】** `terrain_analysis_node.cpp:137` 写死 `message.header.frame_id = "odom"`；`ingestLaserCloud` 完全不看 `msg->header.frame_id`，也不做 TF。

**【推得】** 若上游 `registered_scan` 换到 `map`（SLAM/上游改动），输出会**静默标错 frame**，而三个消费者都按 `odom` 解释（costmap 的 `global_frame: odom` + `sensor_frame: front_mid360`）。

**【建议】** 启动或首帧时校验 frame 与 costmap `global_frame` 一致，不一致则告警；或按 TF 转换。

#### A9（低，设计特性而非缺陷）索引锚在车辆位置，滚动却按整格补偿 ⇒ 跨帧累积有 ≤1 格的位置涂抹

**【证据】** `voxelIndexOf` 以 `vehicle_x/y` 为锚（:464-490）；`rolloverTerrainVoxels` 仅在 `|vehicle − center| > voxel_size` 时整格滚动（:81-105），`shiftGrid` 每次移一格（:382-403）。我验算过：**整格**位移时两者精确互补（车前进 1 格 → 索引 −1 → 数据 −1，重新对齐）。

**【推得】** 但车辆在一个格内的连续位移没有对应的数据搬移 ⇒ 同一世界位置的观测在不同帧可能落进相邻格（最多差 1 格），在整格边界处自校正。对 **planar 无影响**（每帧重建、不跨帧累积）；对 **terrain 体素**表现为：同一表面可能同时存在于相邻两格 ⇒ 点数轻微膨胀、格内容偏"糊"（这也是 3×3 膨胀能掩盖很多问题的原因）。

**【建议】** 属上游设计，不建议在平面阶段改动。若要收窄，须把索引锚点改成滚动中心 `c`（planar 也要同锚），属独立课题。

#### 已检查、确认无问题的项（避免重复排查）

| 项 | 结论 |
| --- | --- |
| `shiftGrid` 与 `rolloverTerrainVoxels` 的方向配对 | 正确（±1 格精确互补，见 A9 验算） |
| `shouldPruneTerrainVoxel` 的 elapsed 计算 | 正确：存的是"相对 init 的时间"，二次相减抵消，等价于"距上次重建的时长" |
| `keepTerrainVoxelPoint` 的 `near \|\| fresh` 语义 | 与上游 `dis < noDecayDis \|\| age < decayTime` 等价 |
| 节点内是否有数据竞争 | **无**：容器是 `component_container_isolated`（`bringup_launch.py:200`），同节点回调落在同一 MutuallyExclusive 组内串行执行，`state_` 无锁也安全。**边界条件**：若换成 `component_container_mt`，`state_` 与 `terrain_cloud*` 的读写会立刻变成真竞争 |
| `elevateByQuantile` 的分位数取整 | 正确（越界已夹到 `point_count − 1`） |
| 输出时间戳 | 正确：继承输入扫描时间戳，不是发布时刻 |

---

### 11. 附录：证据复核命令

本文所有"事实"均可用下列命令在本机复核（只读）。

```bash
# 仓库侧
cd /home/rog/guganav
git log --oneline -1                                   # 应为 a633a25
sed -n '99,201p' src/guga_planner/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp
sed -n '281,325p' src/guga_perception/terrain_analysis/terrain_analysis/src/core/terrain_processor.cpp
sed -n '104,133p' src/guga_bringup/config/reality/nav2_params.yaml
sed -n '150,180p' src/guga_bringup/launch/support/static_tf_publisher_launch.py

# nav2 官方侧（外部 checkout，版本 1.1.20-7-g3c3db59d）
cd /home/rog/navigation2/nav2_costmap_2d
sed -n '435,447p' plugins/obstacle_layer.cpp            # clearing 早于 marking
sed -n '139,151p' plugins/obstacle_layer.cpp            # clearing 默认 false、raytrace 默认 3.0
sed -n '470,477p' plugins/obstacle_layer.cpp            # marking 只比绝对 z
sed -n '624,712p' plugins/obstacle_layer.cpp            # raytraceFreespace 全貌
sed -n '533,566p' plugins/obstacle_layer.cpp            # combination_method → updateWithMax
sed -n '100,140p' src/observation_buffer.cpp            # 点云被变换到 global frame
grep -n "updateOrigin" -A6 src/costmap_2d.cpp           # 重叠区搬运 + resetMaps
grep -n "min_obstacle_height\|max_obstacle_height" plugins/voxel_layer.cpp   # 体素层无下界
sed -n '213,228p' plugins/voxel_layer.cpp               # 低于 origin_z 被夹住仍标 lethal
```

#### §10 附加审查（A1–A9）的复核命令

```bash
cd /home/rog/guganav
T=src/guga_perception/terrain_analysis/terrain_analysis
sed -n '405,411p' $T/src/core/terrain_processor.cpp   # A1  resetPlanarVoxels fill(0)
sed -n '435,453p' $T/src/core/terrain_processor.cpp   # A1  无候选时 return，值保持 0
sed -n '286,300p' $T/src/core/terrain_processor.cpp   # A1  直接相减，无有效性判断
sed -n '132,154p' $T/src/core/terrain_processor.cpp   # A2  先降采样、后判过期
sed -n '413,433p' $T/src/core/terrain_processor.cpp   # A3  3×3 膨胀写 9 个格
sed -n '313,324p' $T/src/core/terrain_processor.cpp   # A3  用膨胀后的 size() 判密度
sed -n '362,380p' $T/src/core/terrain_processor.cpp   # A7  pitch 与 roll 的符号
sed -n '115,140p' $T/src/terrain_analysis_node.cpp    # A6/A8 定时器、processOnce、frame 写死
grep -n "component_container_isolated" src/guga_bringup/launch/core/bringup_launch.py

# PCL 侧（A2 的关键前提）
grep -n "downsample_all_data_(true)" /usr/include/pcl-1.12/pcl/filters/voxel_grid.h
grep -n "Intensity" -A2 /usr/include/pcl-1.12/pcl/common/centroid.h | head
sed -n '400,425p' /usr/include/pcl-1.12/pcl/filters/impl/voxel_grid.hpp

# D20 的实测依据
git log --format='%s%n%n%b' -1 e623154
git show e623154 | git apply --check -        # 期望报 4 处冲突
```

#### 术语

- **幽灵点 / 残点**：动态物体离开后，其点仍留在 `terrain_map`（跨帧累积）并在 costmap 上标为 lethal 的现象。
- **记忆**：costmap 层网格跨周期保留标记的能力。本方案的核心是"把记忆加回 costmap 侧，同时把删除交给观测证据"。
- **非对称判据**：marking 过滤（intensity 门限）、clearing 不过滤（全部点）。

</details>
