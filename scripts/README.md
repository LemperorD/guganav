# scripts

这里存放项目常用脚本。启动类脚本应从脚本所在位置进入仓库根目录，避免依赖调用者当前目录。

## 导航与建图入口

| 脚本                       | 用途                                                                   |
| -------------------------- | ---------------------------------------------------------------------- |
| `scripts/simu_nav.sh`      | 启动仿真导航，`world:=rmul_2025`，`slam:=False`。                      |
| `scripts/simu_map.sh`      | 启动仿真建图，`world:=rmul_2025`，`slam:=True`。                       |
| `scripts/simu_gz.sh`       | 启动 Gazebo 仿真环境。                                                 |
| `scripts/map.sh`           | 启动实车建图入口，`slam:=True`。                                       |
| `scripts/nav_decision.sh`  | 启动实车导航决策测试入口，`slam:=True`，`behavior_tree_type:=manual`。 |
| `scripts/real_save_map.sh` | 保存实车 2D 栅格地图到 `src/guga_nav_bringup/map/reality/`。           |

## 构建

| 脚本                     | 用途                                                                   |
| ------------------------ | ---------------------------------------------------------------------- |
| `scripts/colconBuild.sh` | 使用 Release 配置执行 `colcon build`，并导出 `compile_commands.json`。 |

## 测试与覆盖率

| 脚本                                                  | 用途                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------- |
| `scripts/pre-commit/run_pid_tests.sh`                 | pre-commit 使用的 PID/controller 快速构建与测试入口。                     |
| `scripts/test/test_terrain_analysis_coverage.sh`      | 构建并运行 `terrain_analysis` 测试，生成 gcovr 覆盖率报告。               |
| `scripts/test/test_simple_decision_coverage.sh`       | 构建并运行 `simple_decision` 测试，生成 gcovr 覆盖率报告。                |
| `scripts/test/test_pb_omni_pid_pursuit_controller.sh` | 构建并运行 `pb_omni_pid_pursuit_controller` 测试，生成 gcovr 覆盖率报告。 |

## rosbag 诊断

| 脚本                                   | 用途                                                         |
| -------------------------------------- | ------------------------------------------------------------ |
| `scripts/analyze_controller.py`        | 从 rosbag 统计控制器速度、局部路径、全局路径和错误日志指标。 |
| `scripts/visualize_bag_diagnostics.py` | 从 rosbag 生成速度跟踪、碰撞、costmap 等诊断图和 CSV。       |

示例：

```bash
python3 scripts/analyze_controller.py <bag_path> --storage mcap
python3 scripts/visualize_bag_diagnostics.py <bag_path> --storage mcap --output-prefix bag_diag
```
