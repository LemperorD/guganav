# scripts

这里存放项目常用脚本。启动类脚本应从脚本所在位置进入仓库根目录，避免依赖调用者当前目录。

## 导航与建图入口

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/simulation.sh`    | 一键启动完整仿真，`nav[n]` 或 `map[m]` 会拉起 Gazebo 与导航/建图 RViz。 |
| `scripts/reality.sh`       | 实车统一启动入口，`nav` 为定位/导航、`map` 为建图（`slam:=True`），可追加 `use_decision:=true` 等 launch 参数。 |

示例：

```bash
scripts/simulation.sh n
scripts/simulation.sh nav
scripts/simulation.sh m rmul_2025
scripts/simulation.sh map rmul_2025
scripts/simulation.sh nav rmuc_2025 use_rviz:=False
```

## 捕获 Nav2 崩溃

正常启动导航后，在另一个终端执行 `scripts/nav2_gdb.sh`，然后在 RViz
复现问题。脚本自动附加唯一的 `nav2_container`，在 SIGSEGV/SIGABRT
发生时保存所有线程的完整调用栈到 `/tmp/nav2_gdb_*.txt`，并保留暂停现场。
多个容器时用 `scripts/nav2_gdb.sh PID` 指定。需要 GDB 和 sudo 权限。
实车调试请架空轮子。输入 `quit` 退出 GDB 并脱离进程。

## 调参（在线改参）

| 文件 | 用途 |
| ---- | ---- |
| `scripts/paramview.sh` | 动态调参 Web 界面的入口，默认展示 `controller_server` 的 `FollowPath`（mppi 参数表）；`--profile pid`、`--mode simulation`、`--read-only` 等选项见 `--help`。 |
| `scripts/paramview/paramview_server.py` | Web 服务端：周期读参数推送给页面，保存时按 base → controller → planner 合并顺序写回三层 yaml。 |
| `scripts/paramview/params_list/*_para.txt` | 各控制器参数对照表（mppi/mpc/pid），行格式 `参数名\|说明`。 |
| `scripts/paramview/param_merge.py` | 三层配置合并，供 paramview 标注“当前值已偏离配置值”，也可单独用来做结构重构前后的等价性比较。 |

## 构建

| 脚本                     | 用途                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `scripts/colconBuild.sh` | 使用 Release 配置执行 `colcon build`，并导出 `compile_commands.json`。 |
| `scripts/ci/env_humble.sh` | 加载 ROS Humble 和 acados 环境变量，供本地构建与 CI 复用。 |
| `scripts/ci/install_deps_humble.sh` | 安装并校验 ROS Humble 公开依赖、固定版本的 small_gicp/acados、acados t_renderer、Pangolin 和 xmacro。 |

## Git 辅助

| 脚本                 | 用途                                                                    |
| -------------------- | ----------------------------------------------------------------------- |
| `scripts/gitPush.sh` | 按项目格式生成 commit message；传入 `--push` 时提交成功后推送当前分支。 |

## 调试与调参

| 脚本                     | 用途                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `scripts/tune_fake_source.sh` | 运行时调整 `fake_msg_source` 的裁判数据与敌人数量，改动下一帧即生效。 |
| `scripts/tune_controller.sh` | 运行时调整 controller 的 critics 权重与速度限制等参数。             |

`tune_fake_source.sh` 常用命令：

```bash
scripts/tune_fake_source.sh              # 交互菜单
scripts/tune_fake_source.sh show         # 显示当前取值
scripts/tune_fake_source.sh hp 150       # 改当前血量
scripts/tune_fake_source.sh hit 30       # 在现有血量上扣 30，模拟受击
scripts/tune_fake_source.sh game 0       # 切到赛前；4 切回比赛中
scripts/tune_fake_source.sh game 4
scripts/tune_fake_source.sh rfid center true   # 模拟压上中心控制区
scripts/tune_fake_source.sh enemy 2      # 让视觉"看见" 2 个敌人（0 表示敌人消失）
scripts/tune_fake_source.sh save /tmp/ref.yaml      # 保存当前取值
scripts/tune_fake_source.sh restore /tmp/ref.yaml   # 恢复
```

节点名默认 `/fake_msg_source`；数据源带命名空间启动时（例如仿真里的
`/red_standard_robot1`），把完整节点名作为最后一个参数传入。

脚本或者行为树可视化面板都能改，两者走的是同一套参数服务。裁判两路（
`/referee/robot_status`、`/referee/rfid_status`）用的是绝对话题名，与实车
`serial_driver_node` 一致；视觉 `vision/info` 仍是相对名。RFID 两个开关
`rfid_center`（中心控制区，占点）、`rfid_supply`（己方补给区，回血）在面板里
直接填 `true`／`false`，用来模拟机器人压到卡上。UL 场地里没有基地增益点，回血靠
占领补给区（规则 3.3.2.1）。

比赛阶段 `game_progress` 也在面板里（0 未开始、1 准备、2 自检、3 倒计时、4 比赛中、
5 结算）。默认就是 4，否则树上"比赛开始满 1 秒才放行"的开关会把整棵树挡住；改成 0
再改回 4 就能看到它重新计时。

假数据源还给出假位姿：发 `odometry` 话题和 `odom -> base_footprint` 变换，
外加一条静态的 `map -> odom`。`sim_speed` 不为 0 时机器人朝决策节点发出的
`goal_pose` 走（默认 0.2 m/s），所以 `btview.sh` 起来之后能看到"未到达 → 每个
周期重发目标 → 进入 20 cm → 已到达 → 进占点分支"这一整条路径，不需要额外的
`static_transform_publisher`。把 `sim_speed` 改成 0，位置就固定由 `robot_x`、
`robot_y` 决定，用来手试两条分支。只在不开仿真、不接实车时用：真机上
`odom -> base_footprint` 由驱动发布、`map -> odom` 由定位发布，同时跑会争同一个
变换（`publish_map_to_odom` 置 false 可以只让假数据源发前者）。

## 行为树可视化

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/btview.sh`        | 一键启动假数据源、`rmul_decision` 与行为树 Web 界面，Ctrl-C 一并停止。  |
| `scripts/btview/btview_server.py` | Web 界面的服务端：tail 执行记录 + 转发数据源参数，一般由 `btview.sh` 拉起。 |
| `scripts/btview/btlog_view.py`    | 离线解析 `.btlog` 执行记录，打印树结构与每次 tick 的执行路径。          |

行为树的节点在几微秒内跑完就回到 `IDLE`，Web 界面不会逐事件等待或动画回放。
收到一次 tick 后直接应用完整事件序列并显示这一轮的最终路径；需要查看微秒级
先后顺序时，使用下面的 `btlog_view.py` 展开完整事件序列。

```bash
scripts/btview.sh                    # rmul_decision，浏览器打开 http://localhost:8080
scripts/btview.sh --simulation       # 接入已运行的 Gazebo/Nav2
scripts/btview.sh --decision guga_rmul_strategy   # 切回旧策略节点
scripts/btview.sh --hz 10 --port 8090

python3 scripts/btview/btlog_view.py ~/Desktop/1.btlog            # 摘要
python3 scripts/btview/btlog_view.py ~/Desktop/1.btlog --tick 7   # 展开第 7 次 tick
```

假裁判联调时，`btview.sh` 终端保持运行，另开一个终端即可动态改场景：

```bash
scripts/tune_fake_source.sh                    # 交互菜单
scripts/tune_fake_source.sh hp 350
scripts/tune_fake_source.sh game 0
scripts/tune_fake_source.sh game 4
scripts/tune_fake_source.sh rfid supply true
```

这些改动直接写 `fake_msg_source` 的 ROS 2 参数，下一帧生效，不需要重启
`rmul_decision`。Web 页面里也能改同一组参数。

接入 Gazebo 仿真时，先按正常流程启动仿真，再开一个终端：

```bash
scripts/simulation.sh nav rmul_2025
scripts/btview.sh --simulation
```

`--simulation` 默认把行为树放入 `/red_standard_robot1`，与 Nav2 使用同一套
`goal_pose`、TF 和速度话题。假裁判源放在独立的 `/fake_ref` 命名空间，只提供
`/referee/*` 数据，不发布会与 Gazebo 冲突的假位姿。该模式会自动启用
`use_sim_time`。仿真命名空间不同时显式传入：

```bash
scripts/btview.sh --simulation --namespace my_robot
```

该模式只接入已经运行的 Gazebo/Nav2，不会自行启动仿真。

`GotoPoints` 每个新航点还会发布 `goal_pose_marker`（`visualization_msgs/Marker`，
Transient Local）。默认 Nav2 RViz 配置已加入“Decision Goal”显示，因此即使 RViz
晚于目标发布启动，也能看到 `GotoPoints` 当前选中的目标箭头。仿真中完整话题为
`/red_standard_robot1/goal_pose_marker`。

右侧“决策树黑板”会显示 `game_progress`、`HP`、`destination`、
`global_state_`、占点/停留计时、复活标志以及导航输入等变量。黑板快照由
`rmul_decision` 发布到 `/btview/blackboard`，页面每 250 ms 检查一次变化，
只读展示；左侧仍是假裁判参数，可以直接修改。

节点状态另有 `/btview/node_status` 每 tick 心跳。`FileLogger2` 只记录状态变化，
因此持续的 `RUNNING` 不会产生新日志；Web 页面用心跳快照刷新稳定状态。

执行记录由决策节点的 `btlog_path` 参数决定（默认 `/tmp/bt_trace.btlog`），
传空字符串可关闭。`btview.sh` 会把三个进程的日志写到 `log/btview/`。

同一个子行为树被多处调用时，日志里会存下每一份实例，节点编号（`#uid`）各不相同，
界面上按调用点分开画，子树节点下面那行小字标的就是调用它的位置。页面晚于节点
打开也能看到最近 10 次 tick：服务端会把这几次补发给新连接。

节点边框按这一轮的状态着色：绿=成功，黄=运行中，红=失败，灰=这一轮没走到。
一次 tick 只有几微秒，所以黄边框一般只在节点确实停在运行中时才看得到，
例如 `ROS2Wrapper` 还没收到数据、返回 `RUNNING` 的那几轮。连线的黄条表示
这一轮走过的路径。

## 测试

| 脚本                                               | 用途                                                   |
| -------------------------------------------------- | ------------------------------------------------------ |
| `scripts/pre-commit/run_terrain_analysis_tests.sh` | pre-commit/CI 使用的 `terrain_analysis` 快速测试入口。 |
| `scripts/pre-commit/run_pid_tests.sh`              | pre-commit/CI 使用的 PID/controller 快速测试入口。     |
| `scripts/pre-commit/run_simple_decision_tests.sh`  | pre-commit/CI 使用的 `simple_decision` 快速测试入口。  |
| `scripts/pre-commit/run_jps_tests.sh`              | pre-commit/CI 使用的 `jps_planner` 快速测试入口。      |
| `scripts/pre-commit/run_mppi_tests.sh`             | pre-commit/CI 使用的 `nav2_mppi_controller` 快速测试入口。 |
| `scripts/test/run_point_lio_smoke_test.sh`         | 构建 PointLIO 并运行 LiDAR 主链路冒烟测试。           |

## 无实车 UI 测试（假串口）

不需要实车即可验证 UI 标签能否更新：用虚拟串口向 `serial_driver` 喂 BR 协议帧。

| 文件 | 用途 |
| ---- | ---- |
| `scripts/fake_serial_driver/fake_mcu.py` | 假下位机，创建虚拟串口并持续发送运动帧/裁判帧。 |
| `scripts/fake_serial_driver/shm_yaw_probe.py` | 读取 YAW 共享内存槽，判断是写端还是 UI 端的问题。 |

完整步骤、预期效果和常见问题见 [`scripts/fake_serial_driver/README.md`](fake_serial_driver/README.md)。

## 手动覆盖率

覆盖率脚本用于本地阶段性检查，不作为默认 pre-commit/CI 流程。

| 脚本                                                  | 用途                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------- |
| `scripts/test/test_terrain_analysis_coverage.sh`      | 构建并运行 `terrain_analysis` 测试，生成 gcovr 覆盖率报告。               |
| `scripts/test/test_simple_decision_coverage.sh`       | 构建并运行 `simple_decision` 测试，生成 gcovr 覆盖率报告。                |
| `scripts/test/test_pb_omni_pid_pursuit_controller.sh` | 构建并运行 `pb_omni_pid_pursuit_controller` 测试，生成 gcovr 覆盖率报告。 |
