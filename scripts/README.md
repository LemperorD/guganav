# scripts

这里存放项目常用脚本。启动类脚本应从脚本所在位置进入仓库根目录，避免依赖调用者当前目录。

## 导航与建图入口

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/simulation.sh`    | 一键启动完整仿真，`nav[n]` 或 `map[m]` 会拉起 Gazebo 与导航/建图 RViz。 |
| `scripts/map.sh`           | 启动实车建图入口，`slam:=True`。                                       |
| `scripts/nav_decision.sh`  | 基于统一实车 launch 启动导航决策测试，开启 RViz 与通信，关闭 robot state publisher。 |
| `scripts/save_map.sh`      | 保存实车 2D 栅格地图到 `src/guga_bringup/map/reality/`。               |

示例：

```bash
scripts/simulation.sh n
scripts/simulation.sh nav
scripts/simulation.sh m rmul_2025
scripts/simulation.sh map rmul_2025
scripts/simulation.sh nav rmuc_2025 use_rviz:=False
```

## 构建

| 脚本                     | 用途                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `scripts/colconBuild.sh` | 使用 Release 配置执行 `colcon build`，自动检测 MVS SDK 以纳入或跳过 `hik_driver`，并导出 `compile_commands.json`。 |
| `scripts/ci/env_humble.sh` | 加载 ROS Humble 和 acados 环境变量，供本地构建与 CI 复用。 |
| `scripts/ci/install_deps_humble.sh` | 安装并校验 ROS Humble 公开依赖、固定版本的 small_gicp/acados、acados t_renderer、Pangolin 和 xmacro；MVS SDK 需要在相机/硬件环境单独接入。 |

## Git 辅助

| 脚本                 | 用途                                                                    |
| -------------------- | ----------------------------------------------------------------------- |
| `scripts/gitPush.sh` | 按项目格式生成 commit message；传入 `--push` 时提交成功后推送当前分支。 |

## 调试与调参

| 脚本                     | 用途                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `scripts/tune_referee.sh` | 运行时调整 `fake_referee` 的裁判数据（血量、发弹量、热量等），改动下一帧即生效。 |
| `scripts/tune_controller.sh` | 运行时调整 controller 的 critics 权重与速度限制等参数。             |

`tune_referee.sh` 常用命令：

```bash
scripts/tune_referee.sh              # 交互菜单
scripts/tune_referee.sh show         # 显示当前取值
scripts/tune_referee.sh hp 150       # 改当前血量
scripts/tune_referee.sh hit 30       # 在现有血量上扣 30，模拟受击
scripts/tune_referee.sh save /tmp/ref.yaml      # 保存当前取值
scripts/tune_referee.sh restore /tmp/ref.yaml   # 恢复
```

节点名默认 `/fake_referee`；假裁判带命名空间启动时（例如仿真里的
`/red_standard_robot1`），把完整节点名作为最后一个参数传入。

## 行为树可视化

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/btview.sh`        | 一键启动假裁判、决策节点与行为树 Web 界面，Ctrl-C 一并停止。            |
| `scripts/btview_server.py` | Web 界面的服务端：tail 执行记录 + 转发假裁判参数，一般由 `btview.sh` 拉起。 |
| `scripts/btlog_view.py`    | 离线解析 `.btlog` 执行记录，打印树结构与每次 tick 的执行路径。          |

行为树的节点在几微秒内跑完就回到 `IDLE`，实时界面看到的永远是静态状态，
看不出执行顺序。所以 Web 界面不做实时高亮，而是每完成一次 tick 就定格显示
这一轮走过的路径；离线工具则直接按时间戳排出完整序列。

```bash
scripts/btview.sh                    # 浏览器打开 http://localhost:8080
scripts/btview.sh --hz 10 --port 8090

python3 scripts/btlog_view.py ~/Desktop/1.btlog            # 摘要
python3 scripts/btlog_view.py ~/Desktop/1.btlog --tick 7   # 展开第 7 次 tick
```

执行记录由决策节点的 `btlog_path` 参数决定（默认 `/tmp/bt_trace.btlog`），
传空字符串可关闭。`btview.sh` 会把三个进程的日志写到 `log/btview/`。

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
| `scripts/fake_mcu.py` | 假下位机，创建虚拟串口并持续发送运动帧/裁判帧。 |
| `scripts/shm_yaw_probe.py` | 读取 YAW 共享内存槽，判断是写端还是 UI 端的问题。 |

完整步骤、预期效果和常见问题见 [`scripts/fake_serial_test.md`](fake_serial_test.md)。

## 手动覆盖率

覆盖率脚本用于本地阶段性检查，不作为默认 pre-commit/CI 流程。

| 脚本                                                  | 用途                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------- |
| `scripts/test/test_terrain_analysis_coverage.sh`      | 构建并运行 `terrain_analysis` 测试，生成 gcovr 覆盖率报告。               |
| `scripts/test/test_simple_decision_coverage.sh`       | 构建并运行 `simple_decision` 测试，生成 gcovr 覆盖率报告。                |
| `scripts/test/test_pb_omni_pid_pursuit_controller.sh` | 构建并运行 `pb_omni_pid_pursuit_controller` 测试，生成 gcovr 覆盖率报告。 |
