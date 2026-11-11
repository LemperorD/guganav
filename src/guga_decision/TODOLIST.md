# 决策方案公共能力 TODO


## P0：先建立三套方案都依赖的公共底座

- [x] 明确公共包边界
  - [x] 新建候选方案中立的 `guga_decision_common`，公共核心不得依赖
        BehaviorTree.CPP。
  - [x] 公共核心只放数据契约、ROS 输入输出、门控、定位和通用判断；战术分支、
        状态优先级和树结构继续留在各决策方案内。
  - [x] BT 适配头文件与核心库分文件维护，作为公共核心的薄包装。

- [ ] 定义统一的 `DecisionSnapshot`
  - [x] 包含 RobotStatus：当前血量、最大血量、允许发弹量、热量、受击标志。
  - [x] 包含 GameStatus：比赛阶段、接收时间、比赛开始时间。
  - [ ] 包含 Pose：`map` 系下的 `x/y/yaw`、定位是否有效、数据时间。
  - [x] 包含 RFID：中心控制区、己方补给区、是否收到过 RFID 数据。
  - [ ] 包含 Threat：跟踪目标、装甲板、敌人数量、最近敌情时间、最近受击时间。
  - [x] 每一类输入都提供 `received` 或时间戳，禁止用 `0/false` 同时表示
        “没有数据”和“真实值为零”。
  - [x] 提供不可变快照或线程安全读取接口，供状态机和 BT 两种运行方式调用。
  - [ ] 为快照字段、默认值、坐标系和新鲜度语义补单元测试。

- [ ] 实现 `DecisionInputHub`
  - [x] 统一订阅 `/referee/robot_status`、`/referee/game_status`、
        `/referee/rfid_status` 和视觉汇总；跟踪目标与装甲板待补。
  - [x] 话题名、frame、`base_frame_id` 和 QoS 全部参数化，并提供与现有包一致的
        默认值。
  - [x] 输入回调只更新缓存，不在回调线程里执行决策或访问行为树黑板。
  - [ ] 三套方案通过同一个 Hub 获取数据；接入公共 Hub 后不得再重复订阅同一话题。
        当前 `guga_rmul_strategy` 已接入，另外两套待接入。
  - [ ] 增加集成测试，覆盖无数据、部分数据和全部数据三种状态。

- [ ] 实现 `GameGate`
  - [ ] 统一定义状态：`WAIT_ROBOT_STATUS`、`WAIT_GAME_STATUS`、
        `NOT_RUNNING`、`DEBOUNCING`、`START_DELAY`、`READY`。
  - [ ] 支持“比赛阶段持续满足若干秒才放行”的去抖参数。
  - [ ] 支持比赛开始后的启动延迟参数。
  - [ ] 同时提供“当前是否可决策”和“本局是否已经开始”两种结果，供状态机和
        BT 分别使用。
  - [ ] 比赛离开进行中阶段时明确复位门控状态。
  - [ ] `simple_decision` 的 `checkReadiness`、`rmul_decision` 的
        `IsGameStart`、`guga_rmul_strategy` 的 `IsGameStart` 后续只保留策略包装，
        判断逻辑统一调用 `GameGate`。

- [ ] 实现 `PoseProvider` 与 `AtPose`
  - [x] 统一查询 `map -> base_footprint`，支持自定义 `frame_id` 和
        `base_frame_id`。
  - [x] 使用最近可用 TF，定位失败时返回“未知”，不能被当成到达目标。
  - [ ] 提供候选方案中立的两点位距离判断接口：
        `AtPose(x, y, tolerance)`。
  - [ ] 到达容差由调用方传入或配置，不在公共核心中写死。
  - [ ] 为定位缺失、NaN、容差边界和正常到达补测试。

- [ ] 实现 `GoalController`
  - [ ] 统一构造 `geometry_msgs/PoseStamped`，支持 `x/y/yaw` 和固定 frame。
  - [ ] 统一导航目标 QoS，默认与 Nav2 `bt_navigator` 的订阅行为一致。
  - [ ] 支持目标去重、周期重发和强制重发。
  - [ ] 重发周期由配置决定，允许 `simple_decision` 按状态配置不同频率，也允许
        BT 方案使用固定周期。
  - [ ] 明确“重复目标会让 Nav2 重启导航”，公共接口不能默认每个 tick 都发布。
  - [ ] 为相同目标、目标变化、强制重发和定位失败补测试。

## P1：统一三套方案重复出现的策略输入与执行接口

- [ ] 实现 `ResourcePolicy`
  - [ ] 提供通用双阈值滞回：低于进入阈值触发，高于退出阈值恢复。
  - [ ] 支持 HP 和弹药分别配置进入/退出条件。
  - [ ] 支持“任一资源不足即触发”和“全部资源恢复才退出”两种组合方式。
  - [ ] 滞回状态由调用方持有或按实例命名，避免多个策略实例互相污染。
  - [ ] `simple_decision` 的 `needs_supply`、`rmul_decision` 的 `CheckHP`、
        `guga_rmul_strategy` 的 `CheckHpSafe` 复用同一套算法，阈值仍由各方案配置。

- [ ] 统一 `ThreatSnapshot`
  - [ ] 整理跟踪目标、装甲板、最近敌情、最近受击和目标距离。
  - [ ] 明确目标是否有效、是否 tracking、数据年龄和所在坐标系。
  - [ ] 公共层只负责生成威胁上下文，不决定是否交战。
  - [ ] `simple_decision` 的攻击目标选择、
        `guga_rmul_strategy` 的 `ShouldEngage` 等策略判断继续保留在各自方案内。

- [ ] 实现 `MotionOutput`
  - [ ] 统一停车开关 `/chassis_stop` 的发布接口。
  - [ ] 为 `chassis_mode` 和 `cmd_spin` 提供公共输出接口。
  - [ ] 公共接口只执行动作，不决定何时停车、何时小陀螺或何时恢复跟随。
  - [ ] 明确各输出是电平还是事件，并保证超时语义一致。

- [ ] 提供 BT 适配节点
  - [x] `Ros2InputNode`：共享输入 Hub，写入可配置输出端口；机器人状态未到时
        返回 `RUNNING`。
  - [ ] `GameGateNode`：包装 `GameGate`，返回 `SUCCESS`、`FAILURE` 或
        `RUNNING`。
  - [ ] `LowResourceNode`：包装 `ResourcePolicy`。
  - [ ] `AtPoseNode`：包装 `AtPose`。
  - [ ] `SetGoalNode`、`PublishGoalNode`：包装 `GoalController`。
  - [ ] `SetStopNode`、`ChassisModeNode`、`SpinNode`：包装 `MotionOutput`。
  - [ ] BT 适配节点只做端口读写、状态转换和日志，不复制公共核心逻辑。
  - [ ] `simple_decision` 不依赖 BehaviorTree.CPP，直接调用公共核心。

- [ ] 统一配置契约
  - [ ] 统一话题名、frame、QoS、去抖时间、启动延迟、到达容差和重发周期字段。
  - [ ] 统一 HP、弹药阈值的字段名；各候选方案可以配置不同数值。
  - [ ] 统一补给点、中心区、默认点和巡逻点的配置格式。
  - [ ] 为配置缺失、类型错误和值域错误提供启动期检查。

## P2：确认有两个以上方案需要后再公共化的能力

- [ ] RFID 公共节点
  - [ ] 将 RFID 状态放入 `DecisionSnapshot`。
  - [ ] 提供 `AtRfid(target)`，支持中心区和己方补给区。
  - [ ] 区分“RFID 话题未收到”和“当前未压到卡上”。
  - [ ] 如果只有 `guga_rmul_strategy` 使用 RFID，暂不把整套 RFID 战术子树
        移入公共层。

- [ ] 通用巡逻节点
  - [ ] 支持绝对点位、相对基点偏移和命名点位三种输入形式。
  - [ ] 支持单点容差、单点超时、最大圈数和停止条件。
  - [ ] 只有第二个候选方案也需要巡逻时，再将
        `guga_rmul_strategy` 的 `Patrol` 提取为公共 BT 工具。

- [ ] 通用计时与驻留判断
  - [ ] 提供 `HoldFor` 或 `ElapsedWhileTrue`，用于比赛启动延迟、占点驻留和
        单点等待。
  - [ ] 明确条件失效后是立即清零，还是保留计时状态。
  - [ ] 在需求明确前，不直接复用 `rmul_decision` 的 `CheckTime` 和
        `IsOccupationsafe`。

- [ ] 决策观测与调试
  - [ ] 公共快照支持导出一次决策输入摘要。
  - [ ] 统一记录门控状态、当前目标、资源阈值档位和导航重发事件。
  - [ ] 为三套方案提供可比较的离线日志字段。

## P3：验证与接入检查

- [ ] 为公共包补齐单元测试和集成测试。
- [ ] 建立三套候选方案的兼容性测试，至少覆盖：
  - [ ] 无裁判数据显示时全部保持不动。
  - [ ] 比赛未开始和启动延迟内不产生导航目标。
  - [ ] 低血量或低弹药时能产生与各自策略一致的补给决策。
  - [ ] 相同目标不会因每个 tick 重复发布而反复重启 Nav2。
  - [ ] 定位不可用时不会被误判为已经到位。
  - [ ] 三套方案停止、恢复和底盘模式输出符合各自策略。
- [ ] 在同一个仿真场景中分别运行三套方案，确认公共接口行为一致。
- [ ] 公共层接入后，检查三套方案没有重复订阅、重复发布或重复查询 TF。
- [ ] 更新 `src/guga_decision/README.md`，说明三套方案均为备选方案，以及公共层
      与方案私有逻辑的边界。


