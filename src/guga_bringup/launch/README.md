# guga_nav_bringup launch

本目录按启动层级组织导航 launch，根目录只保留对外入口。

## 入口层

| Launch | 用途 |
| --- | --- |
| `reality_launch.py` | 实车导航/建图入口，由 `scripts/map.sh` 和 `scripts/nav_decision.sh` 调用。 |
| `simulation_launch.py` | 仿真导航/建图入口，由 `scripts/simulation.sh` 调用。 |

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
