# guga_bringup

bringup模块，存放launch、参数、rviz等文件

仿真导航支持三种可选组合，通过 `navigation_profile:=jps_pid`、
`navigation_profile:=2d_mppi` 或 `navigation_profile:=jps_mpc` 选择：

| profile | 全局规划器 | 控制器 | 参数文件 |
|---------|-----------|--------|---------|
| `jps_pid` | JPS | 全向 PID | `nav2_params.yaml` |
| `2d_mppi` | SmacPlanner2D | MPPI | `nav2_params_mppi.yaml` |
| `jps_mpc` | SmacPlannerHybrid | MPC | `nav2_params_mpc.yaml` |

`2d_mppi` 与 `jps_mpc` 使用统一的 `base_footprint_nonrotating` 参考系：
`nonrotating_vel_transform` 发布与 `base_footprint` 共原点、朝向不旋转
（odom 对齐）的坐标系，作为 Nav2 的 `robot_base_frame`，支持底盘自旋
（TES）。`jps_pid` 使用 `base_footprint`，不走该参考系。

TES 自旋：`simple_decision` 发布 `chassis_mode` 与 `cmd_spin`，
`nonrotating_vel_transform` 将 `cmd_spin` 的角速度叠加到输出。

速度链：

```text
controller_server -> cmd_vel_controller
velocity_smoother -> cmd_vel_smoothed
nonrotating_vel_transform（旋转补偿 + 叠加自旋） -> cmd_vel
串口驱动 -> MCU
```

## TODOLIST
