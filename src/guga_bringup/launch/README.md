# guga_bringup launch

本目录按启动层级组织导航 launch，根目录只保留对外入口。

## 入口层

| Launch | 用途 |
| --- | --- |
| `reality_launch.py` | 实车导航/建图入口；`use_decision:=True` 时同时启动 `simple_decision`。 |
| `simulation_launch.py` | 仿真导航/建图入口，由 `scripts/simulation.sh` 调用。支持 `jps_pid`、`2d_mppi`、`jps_mpc` 三种导航组合。 |

## 仿真导航组合

`simulation_launch.py` 支持 9 种 planner/controller 组合（默认 `planner:=jps controller:=pid`）：

| planner | controller |
|---------|-----------|
| `jps`（JPSPlanner） | `pid`（OmniPidPursuitControllerNode） |
| `smac2d`（SmacPlanner2D） | `mppi`（MPPIController，工作区源码构建） |
| `smachybrid`（SmacPlannerHybrid） | `mpc`（MpcControllerNode） |

示例：

```bash
# 交互式选择 planner/controller（终端弹菜单）
scripts/simulation.sh nav rmul_2025

# 非交互/脚本调用时显式选择
scripts/simulation.sh nav rmul_2025 planner:=jps controller:=pid
scripts/simulation.sh nav rmul_2025 planner:=smac2d controller:=mppi
scripts/simulation.sh nav rmul_2025 planner:=smachybrid controller:=mpc

# legacy profile 仍可用（等价组合）
scripts/simulation.sh nav rmul_2025 navigation_profile:=jps_pid   # = jps+pid
scripts/simulation.sh nav rmul_2025 navigation_profile:=2d_mppi   # = smac2d+mppi
scripts/simulation.sh nav rmul_2025 navigation_profile:=jps_mpc   # = smachybrid+mpc
```

参数采用**三层合并**（launch 侧按 base → controller → planner 顺序覆盖）：

- `config/simulation/base.yaml` — 公共参数（传感器/terrain/BT/costmap 公共键）。
- `config/simulation/controller/<controller>.yaml` — 控制器差异（含 costmap 系与调参）。
- `config/simulation/planner/<planner>.yaml` — 规划器差异（planner_server + costmap plugins/esdf）。

自定义实验时可传入 `params_file:=/absolute/path/to/params.yaml` 退化为单文件模式
（三份都指向该文件），显式指定优先于分层默认值。

## 核心层

`core/` 只放 Nav2 主栈组合与 lifecycle 相关 launch：

| Launch | 用途 |
| --- | --- |
| `core/bringup_launch.py` | 组合定位/SLAM 与导航节点。 |
| `core/localization_launch.py` | 非 SLAM 分支定位。 |
| `core/navigation_launch.py` | Nav2 导航节点。 |
| `core/slam_launch.py` | SLAM 分支。 |

## 辅助层

`support/` 放入口可选启动或被入口包装的辅助节点：

| Launch | 用途 |
| --- | --- |
| `support/communication_launch.py` | 通信节点。 |
| `support/robot_state_publisher_launch.py` | 独立导航时的 robot state publisher。 |
| `support/static_tf_publisher_launch.py` | 不启动 robot state publisher 时的静态 TF。 |
| `support/rviz_launch.py` | RViz。 |

`scripts/nav_decision.sh` 会显式传入 `use_decision:=True`，启动决策节点并发布
`chassis_mode`/`cmd_spin`。普通 `reality_launch.py` 默认不启动决策，便于手动导航。
