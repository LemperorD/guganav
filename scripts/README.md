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
| `scripts/colconBuild.sh` | 使用 Release 配置执行 `colcon build`，自动检测 MVS SDK 以纳入或跳过 `hik_driver`，并导出 `compile_commands.json`。 |
| `scripts/ci/env_humble.sh` | 加载 ROS Humble 和 acados 环境变量，供本地构建与 CI 复用。 |
| `scripts/ci/install_deps_humble.sh` | 安装并校验 ROS Humble 公开依赖、固定版本的 small_gicp/acados、acados t_renderer、Pangolin 和 xmacro；MVS SDK 需要在相机/硬件环境单独接入。 |

## Git 辅助

| 脚本                 | 用途                                                                    |
| -------------------- | ----------------------------------------------------------------------- |
| `scripts/gitPush.sh` | 按项目格式生成 commit message；传入 `--push` 时提交成功后推送当前分支。 |

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
