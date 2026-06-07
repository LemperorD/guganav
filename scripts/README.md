# scripts

这里存放项目常用脚本。启动类脚本应从脚本所在位置进入仓库根目录，避免依赖调用者当前目录。

## 导航与建图入口

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/simulation.sh`    | 一键启动完整仿真，`nav[n]` 或 `map[m]` 会拉起 Gazebo 与导航/建图 RViz。 |
| `scripts/map.sh`           | 启动实车建图入口，`slam:=True`。                                       |
| `scripts/nav_decision.sh`  | 基于统一实车 launch 启动导航决策测试，开启 RViz 与通信，关闭 robot state publisher。 |
| `scripts/save_map.sh`      | 保存实车 2D 栅格地图到 `src/guga_nav_bringup/map/reality/`。           |
| `scripts/baseline.sh`      | 交互式直线基线总入口，可选择录包、测频率、profiling 或往返发 goal。    |

示例：

```bash
scripts/simulation.sh n
scripts/simulation.sh nav
scripts/simulation.sh m rmul_2025
scripts/simulation.sh map rmul_2025
scripts/simulation.sh nav rmuc_2025 use_rviz:=False
```

## 构建

| 脚本                            | 用途                                                                       |
| ------------------------------- | -------------------------------------------------------------------------- |
| `scripts/Build/colconbuild.sh`  | 执行默认 `colcon build`，并导出 `compile_commands.json`。                  |
| `scripts/Build/colconBuild-O2.sh` | 使用 `RelWithDebInfo`、`-O2` 和调试符号构建，供 perf/Hotspot 分析使用。 |

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

## 性能与基线

| 脚本                                      | 用途                                                   |
| ----------------------------------------- | ------------------------------------------------------ |
| `scripts/perf/record_straight_baseline.sh` | 录制直线轻量行为基线 bag。                            |
| `scripts/perf/record_straight_diagnostic.sh` | 录制直线诊断 bag，额外包含点云、terrain 和 costmap。 |
| `scripts/perf/record_straight_profiling.sh` | 对指定进程采集 perf/Hotspot profiling 数据。         |
| `scripts/perf/measure_hot_topic_hz.sh` | 测量导航热点 topic 的接收频率并输出汇总。 |
| `scripts/perf/publish_roundtrip_goal.py` | 按固定间隔发布 `0,0` 与 `9,-5.5` 往返 goal。 |

## 手动覆盖率

覆盖率脚本用于本地阶段性检查，不作为默认 pre-commit/CI 流程。

| 脚本                                                  | 用途                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------- |
| `scripts/test/test_terrain_analysis_coverage.sh`      | 构建并运行 `terrain_analysis` 测试，生成 gcovr 覆盖率报告。               |
| `scripts/test/test_simple_decision_coverage.sh`       | 构建并运行 `simple_decision` 测试，生成 gcovr 覆盖率报告。                |
| `scripts/test/test_pb_omni_pid_pursuit_controller.sh` | 构建并运行 `pb_omni_pid_pursuit_controller` 测试，生成 gcovr 覆盖率报告。 |
