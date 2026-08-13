# RoboMaster 哨兵局部路径跟踪与动态避障控制方案决策报告

> 状态：技术评审稿（已同步当前小陀螺控制实现）
> 仓库快照：`/home/rog/guganav`，2026-08-06（Asia/Shanghai）
> 目标平台：ROS 2 Humble + Nav2，i9-12900H，两全两舵，控制量 `vx/vy/wz`，目标控制频率 ≥10 Hz、峰值速度约 3 m/s
> 资料检索截止：2026-08-06；网页均于该日访问

## 1. 决策摘要

**建议主路线：Nav2 MPPI（`Omni`）+ 条件化里程计 + 独立地形描述层 + 坡度/牵引监督器 + 真实多边形 footprint + 最终速度链路后的 Nav2 Collision Monitor。** 先保持 JPS + B-spline 和现有 costmap/ESDF 基础设施不变，把局部控制从“只跟踪既定路径”升级为“预测多条可行局部轨迹并允许绕开临时障碍”。当前 `/terrain_map` 只是相对局部地面的障碍高度点云，并不含坡度、横坡或可通行速度；当前 `/odometry` 的 twist 是位姿裸差分且存在坐标语义风险。两者均不能直接承担可靠坡道闭环。坡度监督器应使用新增的 terrain descriptor、IMU pitch/roll、轮速与条件化里程计残差，动态收紧 `vx/vy/wz`、加减速度和 jerk；MPPI 仍负责平面局部绕障，不能把其 `Omni` 模型解释为已建模重力、轮胎附着或坡面三维运动。Nav2 Humble 官方分支直接包含 MPPI、DWB 和 Collision Monitor；MPPI 官方实现明确支持全向底盘、CPU-only，并在较老的四代 i5 上测得 50 Hz 以上，因此 i9-12900H 达到 10–20 Hz 的算力风险低，最终仍须在完整感知负载下实测。[R1][R2][R23][R24]

**稳定备选 1：HERO 路线的 MINCO + 二阶全向 acados MPC。** 哈工大（威海）HERO 2026 已公开一套 ROS 2/Nav2 赛场代码：Smac Hybrid 后以 costmap ESDF 做两阶段 MINCO 五次多项式优化，行为树配置 15 Hz 重规划，再由 40 步/2 s 的双积分器 MPC 以配置 100 Hz 跟踪；实车配置为 2.2 m/s、3.0 m/s²。[R21][R22] 它对“高精度、平滑穿越”的证据相关性最高，但公开模型是平面世界系双积分器，没有坡度、重力、轮胎附着、轮速或舵角状态；避障发生在 MINCO 层，MPC 本身也不读 costmap。建议作为独立 A/B 候选吸收和复现，在完成坡道模型/状态验证前不承担坡道主控。

**稳定备选 2：DWB。** 它是 Nav2 Humble 官方控制器，原生支持全向速度采样、加速度动态窗口和 footprint 碰撞判定，代码成熟、故障可解释，适合作为 MPPI 失败时的确定性低速回退；但在 `vx × vy × wz` 三维采样、3 m/s 和狭窄动态交互下，上限明显低于 MPPI。[R3]

**当前 Omni PID Pursuit 仅保留为 A/B 基线和受限模式。** 它不是动态避障回退：当前碰撞检测只抽取局部参考路径上的约 10 个中心点，没有 rollout 最终指令、没有 footprint 扫掠、不会主动产生绕障轨迹。主控失败且障碍状态不明时应停车，不能无条件切 PID 继续走。

**前卫路线：在 HERO 二阶模型上扩展全向 NMPC/MPCC + 动态目标预测 + 拓扑多初值/概率约束。** 其理论上限最高：可统一处理 `vx/vy/wz`、加速度/jerk、舵轮转角与转速、静态 ESDF、动态机器人预测和不同绕行拓扑。T-MPC++/SH-MPC 的公开论文与工程实现证明了动态环境中并行拓扑轨迹和概率碰撞约束的可行性，公开工程报告典型 20–30 Hz；但其示例模型主要是二阶单车模型，不是本项目的全向 Nav2 插件，论文结果还主要使用商业 Forces Pro。HERO 代码提供了有价值的全向 acados 起点，却尚未实现上述障碍/概率/拓扑约束，故只建议作为并行研发，不阻塞 MPPI 落地。[R9][R10][R11][R21]

**最重要的非算法结论：** 3 m/s 下的安全和“高精度穿越”不能由任一局部规划算法单独保证，坡道更不能沿用平地速度上限。必须先闭合 footprint、上下坡实测制动/回退距离、坡中驻车能力、最终小陀螺指令、串口 watchdog/量纲、时间同步和动态障碍观测延迟。当前 PID 已在 controller 内生成小陀螺完整 `wz`，MPPI profile 也通过 `SpinCritic` 在 rollout 中评分目标 `wz`，二者都经过 velocity smoother；但 Collision Monitor、坡道禁转包络以及 MCU watchdog/限幅仍未闭合。串口 payload 也没有可见的时间戳/序号/ACK。

## 2. 当前项目审计

### 2.1 实际数据链

```text
Point-LIO /aft_mapped_to_init -> loam_interface /lidar_odometry
  -> sensor_scan_generation /odometry -> controller / velocity_smoother

terrain_map / terrain_map_ext
  -> IntensityVoxelLayer -> ESDF -> Inflation -> local/global costmap
  -> JPS (global) -> 7 阶 B-spline（显式 ESDF 梯度优化为可选）-> nav_msgs/Path
  -> Omni PID Pursuit @ 20 Hz -> cmd_vel_controller
  -> Nav2 velocity_smoother @ 20 Hz
  -> /cmd_vel -> serial_driver -> BR/0xCD/26B payload -> MCU
```

| 层 | 当前已有能力 | 评审发现 |
|---|---|---|
| 全局规划 | JPS，`cost>=253` 禁行；禁止角点穿越；B-spline 后做 costmap 碰撞检查，失败回退线性 JPS | 适合保留。实车 YAML 只显式设置 `enable_bspline: true`，而 planner 的 `enable_esdf` 默认是 `false`，所以 **B-spline 的显式 ESDF 梯度优化在当前 reality 配置下并未启用**；ESDF costmap layer 本身仍启用。它解决静态全局可达性，不等价于局部动态避障 |
| costmap | 0.05 m；局部 5 m×5 m，10 Hz；IntensityVoxel + ESDF + Inflation；`robot_radius=0.2` | 3 m/s 时有效前视半径仅约 2.5 m；真实外形未知，圆半径不够支撑高精度门/通道穿越；动态物体只有“当前占用”，无速度与未来轨迹 |
| 当前控制器 | 全向 `vx/vy/wz`、20 Hz、速度前视、曲率/接近减速、横向误差修正、模式切换 | 只跟踪 path；碰撞检测是路径中心点采样，不是控制轨迹的 footprint 碰撞；`setSpeedLimit()` 未实现 |
| Nav2 参数 | `v_xy<=2.5 m/s`，平滑器 `a_xy<=4.5 m/s²`；目标容差 0.15 m | `min_y_velocity_threshold=0.5 m/s` 会把低于阈值的实测横向速度当作 0，直接损害全向精细闭环；`yaw_goal_tolerance=6.28` 等于不约束终点朝向 |
| 坡道处理 | Point-LIO 保留 6DoF pose；terrain analysis 能避免连续坡面被整体投成障碍 | `/odometry` 未输出可审计的融合速度/协方差；`/terrain_map` 的 intensity 只是离地高度，不是坡度或可通行性。controller、smoother 和串口上限仍为平地常数，未见坡度/附着/打滑速度包络或驻坡策略 [R23][R24] |
| 小陀螺 | `simple_decision` 发布模式和转速；PID 直接输出目标 `wz`；MPPI 用 `SpinCritic`；`gimbal_cmd_vel_adapter` 不在主链 | 已经过 smoother，但仍需 Collision Monitor、坡道禁转包络和旋转 footprint 扫掠验证 |
| 串口 | `/cmd_vel` 到 26B BR 帧；CRC8；传 `vx/vy` 两套坐标值和 `-wz`；115200 baud | 消息触发式发送；单次写最长等待 100 ms；可见协议无时间戳/序号/ACK；配置注释称 m/s→mm/s 理论比例 1000，但实际 YAML 为 50，必须核对 MCU 单位和标定 |
| 自研 MPC | acados 全向运动学模型、`vx/vy/wz` 输入上界、Nav2 插件外壳 | 未启用；状态被固定为 `{0,0,0}`；只追一个 lookahead 点；无障碍、加速度、舵轮约束；求解失败后仍读取输出；`setSpeedLimit()` 为空。当前只能视为求解器接线原型 |

### 2.2 里程计与 terrain map 专项审计

#### 里程计：pose 链可用，坡道速度反馈尚不合格

当前实车链为：

```text
Point-LIO `aft_mapped_to_init`（6DoF pose）
  -> loam_interface `/lidar_odometry`（转换到 odom，child=`front_mid360`）
  -> sensor_scan_generation `/odometry`（转换到 `gimbal_yaw`）
  -> Nav2 controller / velocity_smoother
```

- Point-LIO 实车配置启用 IMU 测量与逐传播时刻 odometry 发布（`mapping.imu_en=True`、`publish_odometry_without_downsample=True`），但采用 `use_imu_as_input=False` 的 30 维 IMU-as-output 模式，且不发布自身 TF。配置重力向量约为 `[0, 0.187, -0.955]`；它可能是雷达安装姿态补偿，也可能是遗留标定，必须用静止多姿态与已知坡角重新核验，不能直接当真实坡度零点。
- Point-LIO 内部状态含速度，但其公开 `Odometry` 这里只填写 pose；`loam_interface` 再次只转发 pose，不写 twist 或 pose/twist covariance。随后 `sensor_scan_generation` 用连续 pose 做一阶有限差分，只以 `10 m/s`、`20 rad/s` 阈值拒绝尖峰，没有滤波、融合状态或置信度。
- 该差分线速度来自 `odom` 世界系坐标差，却直接写入 `child_frame_id=gimbal_yaw` 的 `twist`。ROS `nav_msgs/Odometry` 规定 pose 在 `header.frame_id`、twist 在 `child_frame_id`，因此这里存在明确的坐标语义风险。[R24] 在横移、旋转、小陀螺和坡面上，这会直接污染 MPPI/DWB 的闭环初速度。
- TF 查询失败时函数返回单位变换并继续发布，而不是丢弃该帧或标记 invalid；所有 covariance 默认为零，消费者无法区分高质量状态、陈旧状态与静默 TF 故障。实车 `controller_server.min_y_velocity_threshold=0.5 m/s` 还会把大量精细横向反馈钳成零；`velocity_smoother` 当前为 `OPEN_LOOP`，其 odom 配置不会弥补上述问题。

**决策**：不要让控制器直接消费当前 `/odometry` 作为最终状态反馈。增加一个 odometry conditioning/fusion 边界：保留 Point-LIO 6DoF pose，优先使用其滤波器速度状态或经时间戳校验的滤波差分，严格转换为底盘/控制 `child_frame` 下的 `vx/vy/wz`，填写 pose/twist covariance，并发布 `valid/age/source/innovation` 质量状态。TF 失败、时间倒退或超龄时不允许 identity 代替真值。Nav2 只消费稳定的水平 `x/y/yaw` 与 body-frame `vx/vy/wz`；完整 `z/roll/pitch`、IMU 和重力方向并行提供给 terrain estimator 与坡度监督器，不建议直接扩进标准 MPPI `Omni` 状态。

轮速、两舵角和电机反馈不是可选的“精度增强”：它们是区分“LIO 认为车在动”和“轮子空转/车体下滑”的必要观测。以轮地运动学预测速度与 LIO/IMU 速度的创新量做 slip 指标，但阈值必须按正常转向、横移、颠簸和各坡度实测分布冻结。

#### terrain map：可保留为障碍高度层，不能当完整地形模型

- `terrain_analysis` 输入 `/lidar_odometry` 与 `/registered_scan`，输出 `/terrain_map`；扩展实例输出 `/terrain_map_ext`。输出点 intensity 的实际定义是 `point.z - estimated_ground_height`，即**相对局部估计地面的离地高度**。local 默认地形/平面体素尺度分别为 `1.0 m`、`0.2 m`；地面高度取局部最低点或分位数。
- 实车 `useSorting=True, quantileZ=0.2` 的作用，是让连续坡面相对各自局部地面的高度接近零，从而不把整面坡当障碍。这可支持“坡面点云不过度抬高 costmap”，但没有显式估计纵坡、横坡、粗糙度、坡向、地形置信度、摩擦或可通行速度。
- `clearDyObs=True` 只是按点的相对高度、距离与雷达垂直视场启发式过滤疑似动态点；没有目标 ID、数据关联、速度估计或未来轨迹，不能算动态障碍跟踪/预测。
- 扩展实例 YAML 的 `checkTerrainConn`、`terrainConnThre`、`ceilingFilteringThre` 与代码声明的 `checkTerrainConnectivity`、`terrainConnectivityThreshold`、`ceilingFilterThreshold` 不一致，因此这些配置项很可能未被节点采用。当前 `noDataObstacle=False` 也意味着坡顶盲区、落差或负障碍造成的无数据区不会默认阻塞。

**决策**：保留现有 `/terrain_map[_ext] -> IntensityVoxelLayer`，但将其正式命名和职责限定为“障碍离地高度层”。并行新增地形描述栅格/查询接口，至少输出 `ground_elevation`、沿候选路径纵坡、横坡、roughness、台阶/落差、no-data risk、confidence、data_age、traversability 与建议速度上限。三类消费者职责不同：

| 消费者 | 使用地形信息做什么 | 不应做什么 |
|---|---|---|
| JPS / terrain-aware planner | 当存在多条路线时，比较坡度、横坡、粗糙度和盲区累计代价；不可通行台阶/坡度作硬禁行 | 不用瞬时 IMU pitch 给整条全局路径定代价 |
| MPPI Terrain Critic | 对短时域候选轨迹评分，偏好低风险、可制动、少横坡路线 | critic 软代价不代替硬速度/驻坡包络 |
| 坡度/牵引监督器 | 结合 IMU 与轮速/LIO 残差，输出上下坡非对称的速度、加减速度、jerk、`vy/wz` 限制或 STOP | 不负责绕障或选择全局拓扑 |

短期若不新增完整 terrain descriptor，至少从局部地面拟合输出坡度/横坡/残差/置信度和 no-data mask；在此之前，JPS 只保持现状穿越已知固定坡道，不应宣称已经 terrain-aware，坡顶未知区必须通过专门限速/盲区规则补偿。

### 2.3 已有哨兵工程证据

- **HERO 2026 是本轮最重要的一手证据。** 哈工大（威海）HERO 于 2026-07-21 一次性公开 2025/2026 赛季导航快照；根 README 将其描述为赛场代码，并附实车/比赛视频，声明曾在约 10 cm 几何裕度下完成隧道测试。[R21] 固定提交 `d96408c` 可直接确认以下实际链路：双雷达融合与稠密点云 → `dog_map/fast_layer` → 0.025 m costmap → Smac Hybrid → 两阶段 MINCO/ESDF → 二阶全向 acados MPC。声明和视频是有价值的场景证据，但公开仓库没有原始测试数据、统计次数或独立真值，**不能把“10 cm”写成已复现验收结果**。
- HERO 的行为树以 15 Hz 重跑 planner+smoother；MINCO 使用五次多项式和 L-BFGS，将 ESDF 距离、速度、加速度作为离散积分的**软惩罚**，实际 YAML 为 `safe_distance=0.3 m`、`max_vel=2.2 m/s`、`max_acc=3.0 m/s²`。MPC 是世界系双积分器：状态 `[x,y,yaw,vx,vy,wz]`，控制 `[ax,ay,alpha]`，40 步/2 s，SQP-RTI+HPIPM，速度和加速度为箱约束，配置控制频率 100 Hz。[R21][R22]
- 必须区分 HERO 的两层能力：**MINCO 负责当前 costmap/ESDF 的反应式重规划，MPC 只负责时间轨迹全状态跟踪。** `hero_mpc_controller` 不查询 costmap、不含障碍/footprint/对手预测约束，`setPlan()` 只缓存而不使用 Nav2 path，实际订阅专用 MINCO 多项式话题。因此它不是“动态障碍 NMPC”，更不是带概率安全保证的局部 MPC。
- 公开快照的工程风险也需计入：MPC 初始速度明确使用上一周期预测指令而非实测里程计速度；TF 变换失败时继续把原 frame 的 pose 当 map pose；`get_next_state()` 实际取预测第 3 阶段；`setSpeedLimit()` 未实现；MINCO 已接收 footprint subscriber 但优化只查询轨迹中心点 ESDF；优化后未见独立连续 footprint 碰撞复核。规划包未见单元/基准测试，仓库未附 acados 本体，且 `pb_minco_smoother/package.xml` 声明 `rmoss_interfaces`、CMake/源码却使用 `interfaces`。这些不否定其赛场价值，但意味着移植前必须先做可复现构建和故障审计。[R21]
- 北极熊战队 `pb2025_sentry_nav` 是与本项目最接近且持续维护的公开工程：ROS 2 Humble、Nav2、Mid360/Point-LIO、terrain analysis、`gimbal_cmd_vel_adapter` 和 `pb_omni_pid_pursuit_controller`；其 README 明确使用 Nav2 Global Planner + Omni PID Pursuit。[R7] 它证明当前框架具备 Sim2Real 工程基础，但没有证明 3 m/s 动态避障或带预测的局部规划上限。
- 西电 IRobot、华科 HUST 等也公开过哨兵 SLAM/导航仓库，更多证明 LIO + ROS 导航是常见路线。[R8] 注意 **HUST 是华中科技大学，不是 HITWH 哈工大（威海）HERO**，不能混作 HERO 证据。

## 3. 能力分层：不要混淆理论、开源实现和本项目代码

| 方案 | 算法理论能力 | 成熟开源实现能力 | 本项目已有能力 |
|---|---|---|---|
| Nav2 MPPI | 随机采样控制序列、前向预测、critic 加权；可在同一周期兼顾跟踪和绕障 | **强**：Nav2 Humble 官方包，Omni/差速/Ackermann，CPU 向量化、测试和失败重试 [R1][R2] | **无实现，低集成门槛**：直接适配 `nav2_core::Controller`；costmap/path/odom 接口已有 |
| Nav2 DWB | 动态窗口/轨迹 rollout；速度与加速度可达集内择优 | **强**：Nav2 Humble 官方包，原生全向轨迹生成器和 footprint critic [R3] | **无实现，低集成门槛** |
| 当前 PID Pursuit | 精确、低延迟的路径误差反馈；没有局部搜索空间 | **中**：本队/北极熊系实现，有单元测试；非 Nav2 官方通用控制器 [R7] | **已运行**，但动态避障能力弱 |
| HERO MINCO + MPC | MINCO 生成时间参数化避障轨迹；二阶全向 MPC 以硬速度/加速度箱约束跟踪 | **中**：有 2026 哨兵完整快照和赛场视频，但单提交、规划包无测试，依赖/manifest 尚需修复 [R21][R22] | **无**；本项目仅有不同且更早期的 acados 原型，不能视为已集成 |
| TEB | 联合优化位姿与时间，支持全向、速度/加速度和动态障碍（取决于输入） | **ROS 1 强、ROS 2 Humble 弱**：权威仓库主分支为 Noetic；检索到的 Humble 移植仓库规模小、README 近空 [R4][R5] | 无 |
| 全向 NMPC/MPCC + T-MPC++/SH-MPC | 可表达动力学硬约束、动态障碍预测、不确定性与多拓扑；理论上限最高 | **部分成熟**：T-MPC 工程支持 ROS/ROS2、acados/Forces Pro并报告 20–30 Hz，但非 Nav2 即插即用、默认模型非全向 [R9] | **求解器原型**，离可用控制器仍远 |
| CBF-QP 安全滤波 | 在模型、状态和障碍集合准确且 QP 可行时可给出条件安全不变性 [R14] | 通用库/论文多，适配两全两舵和移动障碍仍需自研 | 无 |

“硬约束”需严格解释：优化器里的速度上界是硬约束，不代表现实世界碰撞绝不会发生。传感延迟、地图误差、离散采样、求解超时、轮胎打滑和 MCU 执行偏差都会破坏保证。最终安全至少需要规划层、独立碰撞监控、PC/MCU 限幅与 watchdog、物理急停四层。

## 4. 三个稳定方案

### S1（首选）：Nav2 MPPI Omni

MPPI 从上一周期控制序列出发，加入高斯扰动形成批量候选，按全向运动模型前向积分，再用路径、目标、速度、costmap/footprint 等 critic 打分并更新控制序列。[R1][R6]

**优势**

- 官方 Humble 分支直接存在，接口与当前 controller_server 一致；原生 `Omni` 和 `vy` 采样。
- 相比 DWB 的规则网格采样，批量张量计算更适合三维控制空间；允许偏离全局 path 绕开临时障碍，再回归路径。
- 官方 Humble README 给出“较旧四代 i5 上 50+ Hz”和 CPU-only；i9-12900H 有足够余量用于 20 Hz 起步，但不能把该数字直接外推到本项目完整点云、ESDF 和双雷达负载。[R1]
- critic 插件机制适合后续加入地形速度、动态轨迹、模式偏好，而主接口保持稳定。

**边界/故障模式**

- 标准 MPPI 从 costmap 看到动态机器人只是随时间刷新的占用；它是预测“自己”，不是预测“对方”。会反应式绕行，但不能天然解决高速交叉、遮挡后出现或对向博弈。
- `Omni` 运动模型只在平面上积分 `vx/vy/wz`。坡面上世界水平投影速度、沿车体坡面速度和轮速不再等价；重力引起的上坡加速能力下降、下坡制动距离增长、横坡侧滑均不在标准 rollout 中。[R1][R23]
- 障碍通常以高代价处理，不是形式化碰撞硬约束；采样分布没有覆盖可行通道时可能停车、抖动或选择次优侧。
- 狭窄通道中，Obstacle/PathAlign 权重、inflation 半径、footprint 与噪声方差耦合；排斥过强会不敢进，过弱会擦碰。
- `time_steps × model_dt × v_max` 不应超出可用 rolling costmap。官方文档也明确指出预测距离和 costmap 尺寸必须匹配。[R1] 若预测 1.0 s、3 m/s，仅运动距离已 3 m，当前半径 2.5 m 不足；建议从 7–8 m 方形局部图、15–20 Hz 更新开始基准测试。
- Humble 版本功能少于新 Nav2 文档；配置必须以 Humble 分支 README/源码为准，不能直接复制 2026 主线参数。

**建议初始工程边界（不是最终参数）**

- controller 20 Hz，`model_dt=0.05 s`，预测窗先取 0.8–1.2 s；以 p99 计算时间低于 40 ms 为门槛逐步扩大 batch/horizon。
- 坡道先使用同一 MPPI 算法的独立 `Slope` 参数/限速模式，而不是立刻切换另一套规划器：抑制横坡 `vy` 和大 `wz`，上下坡分别使用实测速度/加减速度包络；坡度状态不可信时按更保守包络运行。是否需要单独加载 `MPPISlope` 插件实例，由参数切换原子性和 A/B 结果决定。
- 可增加 terrain critic，对候选轨迹沿途的坡度、横坡、粗糙度和坡顶盲区代价评分；它负责“选哪条路”，独立监督器负责“这条指令是否仍在实测牵引/制动包络内”。Critic 软代价不得代替下游限幅和驻坡。
- 使用实测多边形 footprint，开启 footprint-aware cost critic；狭窄区由地形/语义层降低限速，不用单一全局膨胀半径同时承担安全和舒适距离。
- 普通模式和小陀螺模式都由 controller 输出完整 `vx/vy/wz`；`SpinCritic` 让 MPPI rollout 包含目标旋转。禁止在 smoother/串口/MCU 再叠加第二份 `wz`，最终命令仍必须经过 Collision Monitor 并测试扫掠 footprint。
- 保留失败计数、最后可行解年龄和 deadline miss；超限先零速过渡，再切限速 DWB，仍无合法轨迹则停车，不能复用未确认的旧控制量。

### S2（哨兵专项备选）：HERO MINCO + 二阶全向 MPC

HERO 路线不是单一 controller，而是“频繁全局/局部重规划 + 时间轨迹优化 + 高频跟踪”的组合。Smac Hybrid 产生几何路径；MINCO 从 costmap 生成局部 ESDF，以五次多项式联合优化路径形状和分段时间；MPC 再跟踪位置、世界系速度、加速度前馈和可选航向。[R21][R22]

**为什么值得进入稳定组 A/B**：它是唯一公开且与 RoboMaster 哨兵、全向高速和窄通道直接对应的完整路线；轨迹连续到加速度，适合两全两舵减少突变；二阶 MPC 对速度、加速度做显式箱约束；HERO 配置的 100 Hz 控制/15 Hz 重规划远高于本项目 ≥10 Hz 目标，并有赛场视频和 10 cm 裕度自述。相比直接研发 T-MPC++，其 ROS 2/Nav2 接口和问题规模更接近本项目。

**不能由公开代码推出的能力**：配置频率不是实测 p99；README 的 10 cm 是团队声明，不含样本量和真值；没有 3 m/s、动态横穿成功率或求解时延数据。MINCO 的障碍、速度和加速度是采样软惩罚，MPC 的硬约束也只是简化双积分器箱约束，不包含合速度圆约束、jerk、轮速/舵角、摩擦、坡度或实际执行延迟。

**主要故障模式**：动态障碍只通过下一轮 costmap/ESDF 重规划反应，没有速度估计和未来占用；错误拓扑仍可能卡在局部极小值。MPC 不做碰撞检查，MINCO 无可用 ESDF 时会继续无避障优化，stage 2 失败时还可能返回原始 path；中心点 ESDF 与圆形 `robot_radius` 不能替代真实多边形扫掠。控制器用上一周期预测速度作为 MPC 初态，在打滑、碰撞或切换控制器后可能与实车状态分离。

**采用条件**：先把固定提交做成可复现 Humble 构建；补齐 footprint 连续碰撞复核、实测状态融合、切换时状态重置、速度限幅接口和求解/轨迹年龄 telemetry。建议只移植 MINCO 思路和二阶模型，不复制 HERO 的 Smac/地图/行为树全栈；先以当前 JPS path 为输入和 shadow controller 运行。若 p99 两阶段平滑无法稳定达到所需重规划周期，降为“静态精确穿越专用控制器”，不承担动态避障。

### S3（确定性回退）：Nav2 DWB

DWB 在当前速度及加速度边界内生成候选速度/轨迹，用插件 critic 对 path、goal、footprint、障碍和振荡等评分。官方实现的 trajectory generator 支持全向和差速底盘。[R3]

**适合**：需要最少新依赖、可解释调试、确定性较强的低/中速回退；也适合作为 MPPI 的工程对照组。坡道上仅允许在监督器健康且处于已经实测的低速包络内回退。

**优点**：Humble 官方维护；无随机采样；非法 footprint 轨迹可直接剔除；CPU、内存和部署成本低；`LimitedAccelGenerator` 与动态窗口便于约束短期可达速度。

**上限**：三维速度格点数量随 `vx_samples × vy_samples × vtheta_samples` 增长；采样稀疏时窄门精度差，采样密集时计算量升高。它仍只使用当前 costmap，不预测动态机器人的未来；在对向狭窄通道中常出现停走、左右犹豫或局部极小值。3 m/s 下 rollout 时间、离散步长和 local map 必须同步增大。

**不适用**：把 DWB 当作最终高速动态交互上限；出现打滑、横坡越界、下坡制动余量不足，或需要显式舵轮/坡度动力学、概率碰撞约束、长时域多拓扑决策时。

### 基线/受限模式：当前 Omni PID Pursuit + 独立安全层

**保留理由**：已有实机经验、延迟低、参数少、路径畅通时跟踪直接；可提供稳定 A/B 基线，以及由行为树显式选择的限速返航模式。

**必须承认的能力边界**：当前实现只在 transformed path 上抽样中心点；不会检查最终 `vx/vy/wz` rollout、扫掠 footprint 或小陀螺叠加后的命令，也不会在障碍前生成左右绕行。速度硬限幅和 velocity smoother 不能替代碰撞约束。

**适用**：静态或动态障碍稀少、路径已经安全、速度受限的模式；仅在近场安全传感器仍健康时用于降级撤离。

**不适用**：3 m/s 动态穿行、对向会车、盲角横穿、要求主动绕障、非常小的几何余量。

PID 不能作为“前方不明时继续运动”的自动回退。它只应由行为树在 costmap/路径新鲜、未来制动区无障碍且限速的条件下显式选择；否则主控与 DWB 均失败时输出零速。

三种稳定方案及 PID 基线都应共享独立 Collision Monitor。Humble 官方说明它直接从 LaserScan/PointCloud 形成 stop/slow/approach 区域，绕过 costmap/规划器，典型处理时间约为 360 点激光 4–5 ms、24K 点云 4–20 ms；但它不是安全认证实时系统。[R2]

## 5. 一个前卫方案：全向二阶 NMPC/MPCC + 预测动态约束

### 5.1 推荐问题形式

- 状态：`x=[px, py, yaw, vx, vy, wz]`；控制：`u=[ax, ay, αz]`。坡度、横坡、附着估计作为时变模型参数；若能得到舵轮反馈，再加入两舵轮转角/角速度和轮速状态。只有确需三维车体姿态预测时才把 pitch/roll 加入状态。
- 代价：MPCC contour/lag error、目标进度、控制量与 jerk、离障距离、地形粗糙/坡度、横坡侧滑风险、能耗和模式偏好。
- 硬约束：速度、加速度、jerk、舵角速度/轮速、随坡度变化的牵引/制动包络、摩擦圆或其保守近似、静态安全走廊；动态机器人用时变椭圆或占用管约束。重力项必须区分上坡驱动和下坡制动，参数由实测辨识而不是只用名义摩擦系数。
- 动态预测：双雷达点云去地面/聚类/数据关联，输出位置、速度、协方差和 1–2 s 多模态预测。未确认轨迹采用膨胀占用管，不将“没检测到”当作空闲。
- 多拓扑：像 T-MPC++ 一样并行求解“左绕/右绕/停让”等不同初值，避免单个 NMPC 卡在错误一侧；不确定性高时采用 chance/scenario constraint。[R9][R10][R11]
- 安全滤波：可用 CBF-QP 对 NMPC 输出做最小修改，但只在模型/障碍边界/求解可行假设内称为硬保证；CBF 失败仍需零速和 MCU watchdog。[R14]

acados 支持 OCP-NLP、约束、软约束、控制变化率和 RTI，并能拆分 preparation/feedback 降低控制延迟，适合作为 CPU 求解器。[R15][R16] i9-12900H 可利用多核并行多个拓扑，但实时线程应固定核心并避免与 Point-LIO/ESDF 抢占；该路线也不依赖 GPU。

### 5.2 为什么本轮不直接推荐上线

1. 当前 `mpc_controller` 只做一阶全向运动学和单 lookahead 点；HERO 虽已有二阶全向跟踪器，但障碍仍在上游 MINCO 软代价中，二者都不是目标中的动态避障 NMPC。[R21]
2. 两全两舵不是“任意 `vx/vy/wz` 都可瞬时实现”；没有舵角、轮速和执行延迟反馈，模型上限无法兑现。
3. T-MPC++ 公开实现虽支持 ROS2/acados，但不是 Nav2 Humble controller 插件，示例主要是二阶单车模型；论文结果主要基于 Forces Pro，不能把论文频率直接视为本项目 acados 频率。[R9]
4. 动态预测错误会把强优化器变成强错误决策器；需要比控制器更大的感知、标定、数据关联和失效测试投入。

建议它作为 MPPI 落地后的独立实验分支：优先复用并修正 HERO 的二阶全向模型和 MINCO 消息接口，先完成确定性静态 NMPC，再加舵轮约束和实测状态融合，再加单目标预测，最后才做多拓扑和概率约束。

## 6. 主流候选总表

评分 1–5，5 为更适合本项目；“硬约束”指实现可表达/执行的程度，不代表整机安全认证。

| 方案 | 全向 | 动态避障 | 坡道原生模型 | 硬约束 | 3 m/s/≥10 Hz | GPU | 调参 | Humble | 维护成本 | 可达上限 |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| Nav2 MPPI Omni | 5 | 3（反应式） | 1（平面） | 2 | 5 | CPU-only，GPU 无收益 | 3 | 5 | 4 | 4 |
| Nav2 DWB | 5 | 2（反应式） | 1（平面） | 3（离散轨迹拒绝） | 4 | 不用 | 3 | 5 | 5 | 3 |
| HERO MINCO + 二阶 MPC | 5 | 3（15 Hz 反应式重规划） | 1（平面双积分器） | 3（MPC 箱约束；MINCO 软约束） | 4（公开配置 2.2 m/s/100 Hz，未见 p99） | 不用 | 2 | 3（ROS 2；Humble 构建待复现） | 2 | 4 |
| 当前 PID Pursuit | 5 | 1 | 1（平面） | 1 | 5 | 不用 | 4 | 5 | 4 | 2 |
| 坡度感知 NMPC/MPCC + T-MPC/SH-MPC | 5 | 5 | 5（目标设计） | 5（条件性） | 4（待实测） | 非必需 | 1 | 2 | 1 | 5 |
| TEB | 5 | 3 | 1（平面） | 3 | 3 | 不用 | 2 | 1 | 2 | 4 |
| Nav2 Regulated Pure Pursuit | 1（不输出侧向控制） | 2 | 1（平面） | 2 | 5 | 不用 | 4 | 5 | 5 | 2 |
| ROS1 `mpc_local_planner` | 取决模型 | 3 | 取决自定义模型 | 4 | 3 | 不用 | 2 | 1 | 1 | 4 |
| ORCA/RVO2/VO | 5 | 5（需可靠跟踪） | 1（速度空间平面） | 3 | 5 | 不用 | 3 | 需自研 | 2 | 3 |
| CBF-QP 单独使用 | 5 | 4 | 4（可表达，需自研） | 5（强假设） | 5 | 不用 | 2 | 需自研 | 2 | 4 |
| 学习式局部策略/强化学习 | 5 | 4 | 3（取决训练域） | 1 | 5 | 训练需要，推理未必 | 1 | 需自研 | 1 | 5/不确定 |

未入选四个详述方案的主因：TEB 的权威实现仍以 ROS 1 为主，Humble 移植成熟度不足；RPP 不利用 `vy`；ORCA 擅长多智能体互避但不负责静态几何/底盘动力学，且对方机器人不一定互惠；纯 CBF 是安全滤波而非完整路径跟踪器；学习式策略难给出赛前可审计的故障边界。[R4][R5][R14][R17][R18][R19]

其余候选仅列来源：Nav2 Regulated Pure Pursuit [R18]；ROS 1 `mpc_local_planner` [R19]；ORCA/RVO2 [R17]；CBF-QP [R14]；CADRL/SARL 一类学习式人群导航 [R20]。VFH/APF、Elastic Band、DWA 原型和 Rotation Shim 也被检查，但分别属于反应式启发法、TEB/DWB 的前代或包装器，未形成比上述候选更合适的独立路线。

## 7. 推荐架构

```text
单/双雷达 + IMU + 轮速/电机反馈（需补齐）
  -> 时间同步、外参、去车体点、每雷达健康状态
  -> Point-LIO 6DoF pose
  -> odom conditioning/fusion -> body vx/vy/wz + covariance/valid/age
  -> 障碍高度 terrain map -> IntensityVoxelLayer
  -> terrain descriptor（坡度/横坡/粗糙度/置信度/盲区/速度上限）
  -> 静态/未知 costmap + ESDF          动态聚类/跟踪/预测（第二阶段）
                  \                    /
             JPS + B-spline       predicted tracks
                      \            /
        Nav2 Controller Server（同一时刻仅一个控制器有权输出）
          ├─ MPPI Omni：常规主控 @20 Hz
          ├─ HERO MINCO+MPC：shadow/精确穿越候选
          ├─ DWB：确定性低速回退
          └─ PID：显式选择的受限基线，不作盲回退
                         |
       坡度/牵引监督器（上下坡非对称速度、加减速度、wz 包络）
                         |
       controller 内模式管理（PID/MPPI 均输出完整小陀螺 wz）
                         |
      Collision Monitor（最终命令、原始近场点云）
                         |
    PC 侧 finite/限幅/超时 -> 串口 -> MCU watchdog/限幅/急停
```

### 架构要点

- **多控制器、单安全出口**：在一个 `controller_server` 中加载多个 Nav2 插件，用 Controller Selector/行为树选择 `controller_id`；禁止多节点同时发布同一 `/cmd_vel`。切换请求必须先检查 path/costmap/TF/odom 新鲜度，再经过制动或限加速度过渡，且下游 smoother、模式合成、Collision Monitor 和串口只有一套。
- **仲裁优先级**：正常为 MPPI，进入坡道后使用受监督的 `Slope` 包络；连续无合法轨迹、连续 deadline miss 或资源超限时，先零速过渡，再仅在坡度/牵引状态允许时切 DWB 并降低速度。打滑、横坡越界、驻坡不可确认、DWB 仍失败、costmap/TF stale 或近场传感器失效时直接停车/请求 MCU 驻坡。PID 只允许在行为树明确标记的“已知安全路径 + 低速”模式使用，不能因 MPPI/DWB 失败自动接管。HERO 候选在完成状态重置、footprint 和坡道模型复核前只做 shadow；其当前静态 `last_cmd_*` 记忆在插件切换后可能保留旧状态。[R21]
- **防抖与恢复**：不要因单帧无解切换。初始建议以连续 3 个控制周期失败或 1 个严重 stale/安全事件触发降级；恢复主控需连续 2 s 健康并满足最短驻留时间。数值只是待 A/B 冻结的起点，切换原因、前后指令、控制器状态和驻留时间必须进 bag。
- **坡道监督器不是第四个控制器**：它只收紧所有控制器共享的可执行包络，并可请求 `FLAT/SLOPE/STOP` 模式。输入至少包括局部路径方向坡度、IMU pitch/roll、坡度置信度、轮速与惯导/雷达里程计残差、costmap 年龄；输出包括 `v_xy_max`、上坡驱动加速度、下坡制动减速度、横移/角速度上限和是否允许小陀螺。切换必须有坡度与时间滞回。
- **里程计双输出**：Nav2 的 odom 明确为平面 pose + `child_frame` 下的 body twist；完整 6DoF 姿态、z、协方差、状态龄期和创新量另行保留给地形/牵引模块。TF 或时间状态 invalid 时停止更新并触发限速/停车，禁止用单位 TF 或全零 covariance 冒充正常数据。[R24]
- **地形与障碍分离**：现有 terrain map 只继续表达相对地面的障碍高度；可通行斜坡不能简单标 lethal。新增 descriptor 将坡度、横坡、粗糙度、置信度和 no-data 风险形成连续代价及速度上界。坡顶盲区、负障碍和坡面地面点误标需专门规则，2D costmap 无法自行表达。[R23]
- **坐标与速度语义**：冻结 `map/odom/base_link` 在 pitch/roll 下的定义，分别记录水平投影速度、沿坡车体速度和轮速。任何 `cos(pitch)` 换算只能在标定后放在一个明确层中，禁止 controller、TF 变换和 MCU 各自补偿一次。
- **牵引与驻坡**：用实车试验辨识每个坡度/方向/载荷下的 p05 可用驱动和制动能力、打滑阈值以及停止后回退。轮速与 Point-LIO/IMU 速度残差持续超阈值、横坡过大或 roll 快速增长时直接降速/停车；坡中零速必须由 MCU 制动/闭环保持，不能依赖 PC 连续发送零速防溜车。
- **几何真实**：以实测车体多边形和定位/点云 3σ 误差形成 footprint + uncertainty margin；狭窄区动态减小舒适膨胀，但不得缩小物理 footprint。
- **速度包络**：最大速度必须是 clearance、曲率、坡度/方向、粗糙度、观测龄期和实测制动距离的函数，不能只设一个 3 m/s 上限。平地保守起始式仍为 `d_required = v·latency_p99 + v²/(2·a_brake_p05) + footprint_margin + perception_3σ`；坡道必须将 `a_brake_p05` 换成同坡度、同方向、同载荷实测值，下坡不得沿用平地数值。
- **双雷达不是简单拼接**：分别监控时间戳、掉线、外参和遮挡，再在统一时刻融合；否则点云重影会制造假障碍或清除真障碍。控制器不应直接依赖“点数更多”获得安全性。
- **GPU**：四条入选路线均不需要 GPU。i9 的 CPU 余量优先给 Point-LIO、点云过滤、跟踪和控制；只有后续采用深度网络分割/预测时才讨论独显/Jetson。不要为了 GPU 重写成熟 CPU 控制器。
- **模式统一安全**：FOLLOW、GO_HOME、LITTLE_TES 均经过同一个最终 safety filter；小陀螺不能绕过 smoother/collision monitor。坡道上默认禁用小陀螺，只有完成相应坡度、横坡和附着 A/B 后才按包络逐级开放。

## 8. 主要风险与不适用条件

| 风险 | 后果 | 进入实机高速前的关闭条件 |
|---|---|---|
| footprint/传感误差不明 | 窄门擦碰或不敢通过 | 实测外形、外参、定位/障碍误差分布，定义 3σ margin |
| MCU 量纲/watchdog 未核验 | 速度比例错误、断流继续运动 | 台架核对 `vel_trans_scale`、符号、饱和、超时零速、ACK/反馈 |
| MCU/末端重复叠加小陀螺 | 预测指令与执行指令不一致 | MCU 只执行 payload `wz`；记录最终命令；完成旋转扫掠测试 |
| 下坡制动能力未知 | 动态障碍前停车距离不足 | 按坡度/方向/载荷/地面测 p05 制动曲线，速度包络使用下坡实测值 |
| 坡中驻车或断流保持未知 | 回退、侧滑或二次碰撞 | 验证 MCU hill-hold、断流、重启和通信超时；物理防护下完成坡中停启 |
| 坡顶/坡底点云与定位畸变 | 假障碍、漏检或控制突跳 | 双方向 bags 验证地面分割、外参、pitch/roll TF 和 costmap 连续性；盲区内强制限速 |
| 轮胎打滑/横坡侧滑 | 里程计与执行分离，模型预测失效 | 轮速/电机反馈与 LIO/IMU 残差检测；冻结 slope/roll/slip 停车阈值 |
| odom twist 坐标错误/裸差分 | 横移反馈错误、critic 初态和降级判断失真 | body-frame twist 单元/回放验证；补 covariance、age、valid；TF 失败不发布伪正常状态 |
| terrain map 被误当坡度图 | planner/critic 使用错误物理量，坡顶漏检 | 分离障碍高度与 terrain descriptor；核对 connectivity 参数名；冻结 no-data 策略 |
| 5 m local map / 10 Hz costmap | 3 m/s 前视和制动余量不足 | 用实测延迟/制动式验算；扩大地图并证明 p99 更新周期 |
| 仅占用栅格、无动态速度 | 横穿/对向预测晚 | 稳定阶段限速；前卫阶段引入 tracker/prediction |
| 坡面/颠簸定位瞬变 | 控制振荡、costmap 重影 | 坡/颠簸 bags 的姿态、点云和 odom 误差通过门槛 |
| 求解超时/无可行轨迹 | 复用陈旧控制或突然转向 | 明确 zero/fallback；记录解龄、状态码和 deadline miss |

下列条件下 MPPI 也不适合作为“已解决”：传感器看不到负障碍；动态机器人在制动距离内突然出现；真实底盘不能跟随规划的全向加速度；通道几何余量小于综合 3σ 误差；串口/MCU 不具备超时制动；希望获得 SIL/功能安全认证。

## 9. 量化指标与验收门槛

所有结果至少报告中位数、P95、P99 和最差值；成功率给出试验次数，不只给百分比。以下是首轮建议门槛，实车尺寸和制动标定后冻结。

| 类别 | 指标 | 定义/建议门槛 |
|---|---|---|
| 安全 | 碰撞/人工接管 | 仿真 0；封闭实车阶段 0；任何接管算该次失败 |
| 安全 | 最小净空 `d_min` | 车体多边形到障碍的最小距离；不得小于冻结后的 `perception_3σ + control_margin` |
| 安全 | 最小 TTC | 基于相对速度；正常场景 P1 > 0.8 s，低于阈值必须触发减速/停车；最终值由制动试验修订 |
| 精确穿越 | 通道成功率 | 三档单侧几何余量 0.20/0.10/0.05 m，各 ≥30 次；0.10 m 档 ≥29/30 且无接触；0.05 m 先作为能力摸底，不在未完成误差预算前承诺 |
| 跟踪 | 横向误差 | 开阔高速 RMSE ≤0.10 m、P95 ≤0.20 m；窄道低速 P95 ≤0.08 m（需高精度外部真值） |
| 动态 | 通过成功率/阻塞 | 横穿、对向、追越各 ≥30 次；成功 ≥29/30；无效停滞（速度 <0.1 m/s 且有可行通道）P95 <2 s |
| 坡道 | 上/下坡与坡中启停 | 每个冻结坡度、方向、载荷、路面组合各 ≥30 次；0 溜坡越界、0 失控侧滑、0 接触；最大回退距离单独冻结 |
| 坡道 | 制动与盲区速度 | 上/下坡停止距离 P50/P95/P99；坡顶首次有效障碍观测距离必须大于该方向 `d_required`，否则降低进入速度 |
| 牵引 | 打滑与状态一致性 | 报告轮速-车体速度残差、持续时间、触发降级延迟；阈值由正常转弯/颠簸分布和已知低附着试验共同冻结 |
| 状态估计 | odom 速度/姿态 | 以独立真值评估 body `vx/vy/wz` RMSE/P95、pitch/roll 偏差、延迟与跳变；covariance 一致性用 NIS/覆盖率报告，TF/时间故障 100% 标记 invalid |
| 地形 | descriptor 精度 | 已知坡台上纵坡/横坡误差、roughness 重复性、no-data/落差召回率及 false-positive；按距离分桶报告 P50/P95 |
| 平顺 | `a/jerk/wz` 违约 | 超出标定可实现边界的样本占比 0；颠簸段垂向加速度 RMS 和峰值相对基线不恶化 |
| 机电 | 坡道电流/温度 | 最大坡度连续往返 15 min，无过流保护或热降频；电机/驱动峰值和 P95 电流、温升、母线压降均记录 |
| 实时 | 控制周期 | 实际输出 ≥10 Hz；推荐 20 Hz；p99 compute <0.8×周期，deadline miss <0.1% |
| 时延 | 端到端 | 点云时间戳→costmap→最终 `/cmd_vel`→MCU 执行反馈；报告 P50/P95/P99，P99 必须进入制动距离公式 |
| 资源 | CPU/RAM/温度 | 完整 Point-LIO + ESDF + 控制负载连续 15 min，无降频；单控制器核占用和全机 P95 分别报告 |
| 仲裁 | 切换正确性 | 触发→零速/新控制器首条合法命令延迟 P99 <冻结阈值；0 次双发布、0 次旧非零命令复用、0 次 2 s 内来回切换 |
| 鲁棒 | 故障恢复 | 单雷达遮挡/掉线、TF 抖动、costmap stale、solver failure、串口断连均进入定义的限速/停车状态，且不输出陈旧非零命令 |

不要用 odometry 自身作为窄门横向误差真值；应使用场外全站仪/动作捕捉、AprilTag 独立测量或经标定的固定相机。障碍净空也应由独立几何真值复核。

## 10. rosbag、仿真与实机 A/B 方案

### 10.1 rosbag 回放

**记录**：原始每雷达点云与 IMU（含角速度/线加速度）、`/tf`/`/tf_static`、Point-LIO 原始 pose、`/lidar_odometry`、条件化 odometry 及 covariance/valid/age/innovation、轮速/舵角/电机电流温度（当前若无则列为待补）、原始 terrain map intensity、terrain descriptor 各字段、local/global costmap、全局/局部 path、坡道模式及动态速度包络、三段速度话题（controller/smoothed/final）、当前 `controller_id`、切换原因/驻留时间、控制器状态/计算时间、MINCO 轨迹及年龄、模式与 spin、串口上下行原始或解码反馈、CPU/温度。所有传感器保留原时间戳。

建立 10 类 bag：开阔高速 S 弯、窄门、狭窄直廊、静止堵塞、横穿机器人、对向会车、上/下坡、坡中停启、坡顶/坡底盲区、低附着/横坡/颠簸；小陀螺分别在平地和经批准的坡度采集。每类至少 5 个方向/速度组合，并保留失败样本。

回放分两种，禁止混淆：

1. **固定状态 shadow replay**：A/B 在同一 odom/path/costmap 快照上只比较计算时间、输出合法性、预测 clearance 和故障处理。原 bag 的机器人轨迹由旧控制器产生，因此不能用它证明新控制器闭环成功率。
2. **闭环重仿真**：把 bag 中静态地图、传感噪声模型和动态障碍轨迹重建到仿真，A/B 各自驱动物理模型，才比较通过率、时间和最小净空。

回放应固定代码版本、参数哈希、随机种子；MPPI 至少运行 10 个种子，报告分布而非单次最好结果。回放倍率先 1×；高倍率只用于压力测试，不作为时延成绩。

### 10.2 仿真

- 使用现有 RMUC/RMUL 2024–2026 地图，补充可参数化门宽、走廊宽、坡度、路面摩擦/颠簸和移动机器人 actor。
- 坡道矩阵：坡度先取 0/5/10/15° 作为仿真扫描点，最终以赛场最大坡度和实车允许坡度替换；覆盖直上/直下、斜向、横坡、坡中停启、坡顶急停和载荷变化。物理模型必须含重力、轮地摩擦和执行饱和，纯运动学仿真不计入坡道结论。
- 动态场景：90°横穿、盲角出现、对向、同向慢车、两机器人夹击、障碍突然停止；速度 0.5/1.0/2.0/3.0 m/s，出现时距 0.5–3 s。
- 扰动：点云延迟 0/50/100/200 ms、1–10% 丢帧、外参偏差、odom 漂移/跳变、摩擦下降、左右轮附着不一致、轮速偏差、单雷达掉线。
- 分级速度：0.5 → 1.0 → 2.0 → 3.0 m/s；前一级所有安全门槛通过才进入下一级。
- 控制器 A/B/C/D：当前 PID、DWB、MPPI、HERO MINCO+MPC 复现版；HERO 候选在可复现构建与独立碰撞复核完成前只做 shadow，前卫 NMPC 只在 shadow/仿真达到稳定门槛后加入。
- 仲裁故障注入：强制 MPPI 超时/无解、DWB 无合法轨迹、costmap/TF stale、单雷达掉线和控制器恢复，验证“MPPI→零速过渡→限速 DWB→停车”以及恢复滞回；明确验证不会自动落到 PID。

### 10.3 实机 A/B

采用配对、随机顺序试验；同一电量区间、轮胎/舵轮状态、载荷、场地、雷达安装和地图版本。每个 controller×场景×方向×速度至少 30 次；A/B 交替运行以抵消温度、电池和场地变化。

1. 台架：轮组架空，核对 `vx/vy/wz` 方向、比例、饱和、NaN、断流、串口超时和 MCU watchdog；随后在有机械防回退措施的坡台验证断流、重启和坡中保持。
2. 围栏低速：软障碍和泡沫门，0.5 m/s；人工急停员和无线硬急停。
3. 静态精度：开阔 path、三档窄门、斜进/横移/旋转通过。
4. 坡道静态：按 0.3 → 0.5 → 1.0 m/s 分级完成直上/直下、坡中停启、坡顶障碍急停和斜向进入；先验证监督器，再比较控制器。每级均以实测停止距离决定是否提速，不预设坡道可达 3 m/s。
5. 坡道扰动：逐级增加载荷、降低附着、制造单侧附着差和短时轮速异常，验证 slip 检测、降级和驻坡；现场使用机械限位/安全绳，禁止人员站在下坡方向。
6. 动态：先在平地由遥控标准机器人按可重复轨迹横穿/对向；随后仅在静态坡道全部通过的速度档加入坡顶出现和坡面停止障碍。
7. 小陀螺：先平地原地、开阔平移、窄道；坡道默认关闭，最后只在已批准坡度/速度包络内测试。
8. 耐久：候选方案完成 15 min 连续坡道往返和 30 min 比赛节奏，检查电机/驱动温升、母线压降、热降频、线程饥饿和定位漂移。

**晋级规则**：安全指标是 veto，不允许用更快的圈速抵消碰撞/接管；然后比较动态成功率、P95 通行时间、跟踪误差和资源。MPPI 若未显著优于 PID 的动态成功率，先排查感知/costmap 时延，不应只继续调 critic 权重。

## 11. 建议实施顺序（供下一轮评审）

1. 冻结实车 footprint、速度单位、平地及上下坡制动曲线、坡中保持、MCU watchdog 和端到端时延；修正/验证 `min_y_velocity_threshold`。
2. 先关闭状态语义缺口：建立 conditioning/fusion odom，修正 body twist 坐标、滤波/协方差/age/valid，TF 失败不再产生单位变换；同时核验 Point-LIO 重力/外参与坡道姿态误差。
3. 将现有 terrain map 限定为障碍高度层，核对 connectivity 参数名和 no-data 策略；新增 terrain slope/横坡/roughness/confidence/age/drop descriptor，并接入只记录不干预的 slope/slip telemetry。
4. 建立独立坡度/牵引监督器和 `FLAT/SLOPE/STOP` 包络，再建立 MPPI/DWB/PID controller selector、单发布者规则和“主控→DWB→停车”仲裁；Collision Monitor 处理 controller 输出并经 smoother 后的完整小陀螺指令。
5. 将 HERO 固定提交单独做可复现构建和 shadow：先测 MINCO p99、轨迹连续性与 MPC 状态误差；其平面双积分器未加入坡度项前，不进入坡道实机 A/B。
6. 用 rosbag shadow + 含重力/摩擦的闭环仿真完成平地 0.5–3 m/s、坡道按制动能力分级；MPPI 通过后进入实机 A/B。HERO 候选按其公开 2.2 m/s 平地边界先验收再讨论提速。
7. 第二阶段加入动态目标 tracker，并先作为记录/评估输出；确认预测优于常速度基线后，再接 MPPI 自定义 critic。
8. NMPC 分支优先复用 HERO 二阶 acados 模型思想，但重新加入坡度/重力、舵轮、牵引/制动、障碍约束、实测状态融合和失败策略；不得直接把当前原型或未经审计的 HERO 插件切为生产 `FollowPath`。

## 12. 不确定项（必须单独关闭）

- HERO 的算法类型和主要配置已由固定提交确认；但运行平台、实际控制/重规划 P50/P99、比赛时是否与公开参数完全一致、10 cm 测试的车体尺寸/样本数/速度/接触判据、动态障碍成功率仍未知。其标定脚本显式使用 ROS 2 Humble typestore，源码接口也与本项目接近，但根 README 未声明受支持发行版，Humble 全量构建仍需本地复现。[R21]
- 两全两舵的实际几何、舵角/轮速边界、转向延迟、低摩擦可实现加速度，以及 MCU 是否自行做运动学解算。
- 赛场最大纵坡/横坡、坡长、坡顶曲率、表面材料和雨尘影响；3 m/s 是平地目标还是也要求坡道达到，当前不能假设二者相同。
- 是否可获取四轮轮速、两舵角、电机电流/温度和制动状态；MCU 能否在通信断流、导航进程崩溃和上电重启时独立驻坡。
- 串口 `vel_trans_scale=50` 的真实物理单位、MCU 饱和/符号、断流 watchdog、反馈频率和 ACK 语义。
- 实车 footprint、雷达外参/时间同步误差、Point-LIO 在坡/颠簸/小陀螺下的 3σ 误差。
- “较高精度障碍物穿越”最终指的是狭缝/窄门，还是允许跨越坡、台阶、减速带；两者需要不同感知和验收方法。
- 双雷达型号、安装位置、时钟同步、盲区和算力预算。
- 当前工作树中的仿真参数存在未提交修改，本报告以 reality `nav2_params.yaml` 和已提交源码为主，没有把工作树仿真差异当作稳定配置。

## 参考资料

- **[R1]** Nav2, *Model Predictive Path Integral Controller*, Humble branch README；明确 Omni、CPU-only、50+ Hz（四代 i5）、critic 和预测窗/costmap关系。Humble 分支当前提交 `3c3db59`，2026-06-03。<https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller>
- **[R2]** Nav2, *Collision Monitor*, Humble branch README；stop/slow/approach、传感器输入和处理时间，且明确非硬实时安全认证。<https://github.com/ros-navigation/navigation2/tree/humble/nav2_collision_monitor>
- **[R3]** Nav2, *DWB Controller*, Humble branch README；全向/差速 trajectory generators 和 footprint critics。<https://github.com/ros-navigation/navigation2/tree/humble/nav2_dwb_controller>
- **[R4]** Rösmann et al., *Integrated Online Trajectory Planning and Optimization in Distinctive Topologies*, IROS 2017, DOI: [10.1109/IROS.2017.8202234](https://doi.org/10.1109/IROS.2017.8202234)；TEB 权威工程：<https://github.com/rst-tu-dortmund/teb_local_planner>（默认 `noetic-devel`）。
- **[R5]** 社区 Humble TEB 移植检索样本：<https://github.com/lgx233/teb_local_planner-for-ros2-humble>，创建/最后推送 2025-05-30；截至检索日 4 stars、README 仅标题。此条只用于评估工程成熟度，不评价 TEB 理论。
- **[R6]** Williams et al., *Aggressive Driving with Model Predictive Path Integral Control*, ICRA 2016, DOI: [10.1109/ICRA.2016.7487277](https://doi.org/10.1109/ICRA.2016.7487277)；以及 *Information-Theoretic MPC*, IEEE T-RO 2018, DOI: [10.1109/TRO.2018.2865891](https://doi.org/10.1109/TRO.2018.2865891)。
- **[R7]** SMBU PolarBear, *pb2025_sentry_nav*, ROS 2 Humble 哨兵 Sim2Real 工程，README 明确 Nav2 Global Planner + Omni PID Pursuit；仓库最后推送 2026-08-03。<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>
- **[R8]** 西电 IRobot 哨兵导航：<https://github.com/IRobot-Algorithm/sentry_navigation>；华中科技大学 HUST 2023 哨兵导航：<https://github.com/WWho22/Hust_Sentry_SLAM>。仅作公开工程路线样本。
- **[R9]** TU Delft AMR, *mpc_planner*；ROS/ROS2、acados/Forces Pro、动态/静态障碍模块、典型 20–30 Hz 由项目 README 声明；最后推送 2025-03-30。<https://github.com/tud-amr/mpc_planner>
- **[R10]** de Groot et al., *Topology-Driven Parallel Trajectory Optimization in Dynamic Environments*, IEEE T-RO 2024, DOI: [10.1109/TRO.2024.3475047](https://doi.org/10.1109/TRO.2024.3475047)。
- **[R11]** de Groot et al., *Scenario-Based Trajectory Optimization with Bounded Probability of Collision*, IJRR 2024；预印本：<https://arxiv.org/abs/2307.01070>。
- **[R12]** 哈工大（威海）HERO 竞技机器人实验室官方 Bilibili 账号：<https://space.bilibili.com/208915710>；用于核验队伍身份和公开视频来源。
- **[R13]** RoboMaster 官方，哈工大（威海）HERO 2026 全国赛比赛视频样本：<https://www.bilibili.com/video/BV1vdMf6EEwZ>。只证明赛场表现，不证明算法类型。
- **[R14]** Ames et al., *Control Barrier Function Based Quadratic Programs for Safety Critical Systems*, IEEE TAC 2017, DOI: [10.1109/TAC.2016.2638961](https://doi.org/10.1109/TAC.2016.2638961)。
- **[R15]** acados 官方文档，OCP-NLP、RTI、软约束、控制变化率：<https://docs.acados.org/features/index.html>。
- **[R16]** Verschueren et al., *acados: a modular open-source framework for fast embedded optimal control*, 2020 revision，<https://arxiv.org/abs/1910.13753>。
- **[R17]** Fox, Burgard, Thrun, *The Dynamic Window Approach to Collision Avoidance*, IEEE RAM 1997, DOI: [10.1109/100.580977](https://doi.org/10.1109/100.580977)；van den Berg et al., *Reciprocal n-body collision avoidance*, ISRR 2009 / Springer 2011，RVO2：<https://gamma.cs.unc.edu/RVO2/>。
- **[R18]** Nav2, *Regulated Pure Pursuit Controller*, Humble branch：<https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller>。
- **[R19]** TU Dortmund, ROS 1 *mpc_local_planner*：<https://github.com/rst-tu-dortmund/mpc_local_planner>。
- **[R20]** Chen et al., *Decentralized Non-communicating Multiagent Collision Avoidance with Deep Reinforcement Learning*, ICRA 2017；参考实现 CrowdNav：<https://github.com/vita-epfl/CrowdNav>。仅作学习式路线来源，不代表 ROS 2 Humble 可直接部署。
- **[R21]** 哈工大（威海）HERO 竞技机器人实验室，*HERO 2026 Sentry NAV*，固定提交 [`d96408c`](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV/tree/d96408c3de28311585756b8d55f96549010c7285)，提交日期 2026-07-21，访问 2026-08-06。根 README 声明赛场代码、双雷达、MINCO+MPC 和 10 cm 裕度测试；本报告的频率、模型、约束及代码风险均由该固定提交的源码/YAML 复核。仓库截至访问日只有该初始提交，仓库级未声明 license，各子包声明不一。
- **[R22]** Wang et al., *Geometrically Constrained Trajectory Optimization for Multicopters*, IEEE T-RO 38(5), 2022, DOI: [10.1109/TRO.2022.3160022](https://doi.org/10.1109/TRO.2022.3160022)；MINCO/GCOPTER 官方工程：<https://github.com/ZJU-FAST-Lab/GCOPTER>。HERO 的 2D MINCO 实现借用了其稀疏多项式轨迹表示思想，但地面车 ESDF/软约束改造应以 HERO 源码而非原论文能力评价。
- **[R23]** Nav2 Humble MPPI `motion_models.hpp`，内置 DiffDrive/Omni/Ackermann 均在 `x/y/yaw` 平面状态上施加速度约束：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp>；Nav2 Costmap 2D 配置文档：<https://docs.nav2.org/configuration/packages/configuring-costmaps.html>。访问 2026-08-06。用于界定标准 Nav2 局部控制/costmap 的平面能力，不代表其原生处理坡度动力学。
- **[R24]** ROS 2 Humble `nav_msgs/Odometry` 消息定义：pose 使用 `header.frame_id`，twist 使用 `child_frame_id`。<https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html>，访问 2026-08-06；本项目实际计算与参数结论来自下列仓库内审计入口。

## 仓库内审计入口

- 当前实车 Nav2 参数：[`src/guga_bringup/config/reality/nav2_params.yaml`](../src/guga_bringup/config/reality/nav2_params.yaml)
- 导航启动/速度 remap：[`src/guga_bringup/launch/core/navigation_launch.py`](../src/guga_bringup/launch/core/navigation_launch.py)
- 当前 PID 控制器：[`src/guga_controller/pb_omni_pid_pursuit_controller`](../src/guga_controller/pb_omni_pid_pursuit_controller)
- acados MPC 原型：[`src/guga_controller/mpc_controller`](../src/guga_controller/mpc_controller)
- 小陀螺/速度坐标变换：[`src/guga_controller/gimbal_cmd_vel_adapter`](../src/guga_controller/gimbal_cmd_vel_adapter)
- 串口速度链：[`src/guga_driver/serial_driver`](../src/guga_driver/serial_driver)
- JPS/B-spline/ESDF 设计：[`src/guga_planner/jps_planner/DESIGN.md`](../src/guga_planner/jps_planner/DESIGN.md)
- Point-LIO odometry 发布：[`src/guga_localization/point_lio/src/laserMapping.cpp`](../src/guga_localization/point_lio/src/laserMapping.cpp)
- odom/点云转换：[`src/guga_perception/loam_interface`](../src/guga_perception/loam_interface)、[`src/guga_perception/sensor_scan_generation`](../src/guga_perception/sensor_scan_generation)
- terrain map 生成：[`src/guga_perception/terrain_analysis`](../src/guga_perception/terrain_analysis)
