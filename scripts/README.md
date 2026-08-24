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

## 测试

| 脚本                                               | 用途                                                   |
| -------------------------------------------------- | ------------------------------------------------------ |
| `scripts/pre-commit/run_terrain_analysis_tests.sh` | pre-commit/CI 使用的 `terrain_analysis` 快速测试入口。 |
| `scripts/pre-commit/run_pid_tests.sh`              | pre-commit/CI 使用的 PID/controller 快速测试入口。     |
| `scripts/pre-commit/run_simple_decision_tests.sh`  | pre-commit/CI 使用的 `simple_decision` 快速测试入口。  |
| `scripts/pre-commit/run_jps_tests.sh`              | pre-commit/CI 使用的 `jps_planner` 快速测试入口。      |
| `scripts/pre-commit/run_mppi_tests.sh`             | pre-commit/CI 使用的 `nav2_mppi_controller` 快速测试入口。 |
| `scripts/test/run_point_lio_smoke_test.sh`         | 构建 PointLIO 并运行 LiDAR 主链路冒烟测试。           |

## 手动覆盖率

覆盖率脚本用于本地阶段性检查，不作为默认 pre-commit/CI 流程。

| 脚本                                                  | 用途                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------- |
| `scripts/test/test_terrain_analysis_coverage.sh`      | 构建并运行 `terrain_analysis` 测试，生成 gcovr 覆盖率报告。               |
| `scripts/test/test_simple_decision_coverage.sh`       | 构建并运行 `simple_decision` 测试，生成 gcovr 覆盖率报告。                |
| `scripts/test/test_pb_omni_pid_pursuit_controller.sh` | 构建并运行 `pb_omni_pid_pursuit_controller` 测试，生成 gcovr 覆盖率报告。 |
