# guga_bringup

bringup模块，存放launch、参数、rviz等文件

仿真导航支持 9 种 planner/controller 组合，通过
`planner:=jps|smac2d|smachybrid` 与 `controller:=pid|mppi|mpc` 选择
（legacy `navigation_profile:=jps_pid|2d_mppi|jps_mpc` 仍可用）：

| 组合 | 全局规划器 | 控制器 | 参数文件（三层合并） |
|------|-----------|--------|---------------------|
| `jps_pid`（默认） | JPS | 全向 PID | `base.yaml` + `controller/pid.yaml` + `planner/jps.yaml` |
| `2d_mppi` | SmacPlanner2D | MPPI | `base.yaml` + `controller/mppi.yaml` + `planner/smac2d.yaml` |
| `jps_mpc` | SmacPlannerHybrid | MPC | `base.yaml` + `controller/mpc.yaml` + `planner/smachybrid.yaml` | |

`2d_mppi` 与 `jps_mpc` 使用统一的 `base_footprint_nonrotating` 参考系：
`nonrotating_vel_transform` 发布与 `base_footprint` 共原点、朝向不旋转
（odom 对齐）的坐标系，作为 Nav2 的 `robot_base_frame`，支持底盘自旋
（TES）。`jps_pid` 使用 `base_footprint`，不走该参考系。

**导航启动即小陀螺**：`nonrotating_vel_transform` 的无控制器差异参数在
`config/<mode>/base.yaml` 的 `nonrotating_vel_transform:` 段；启动模式与自旋速度
写在 `config/<mode>/controller/<profile>.yaml`：`mppi`/`mpc` 为
`initial_chassis_mode=1`（littleTES）+ `init_spin_speed=1.0`，所以
`2d_mppi`/`jps_mpc` 导航栈一启动底盘即自旋，不依赖决策节点；`pid` 为
`initial_chassis_mode=0`（跟随模式）+ `init_spin_speed=0.0`，不自旋。

`nonrotating_vel_transform` 的其余参数（话题、坐标系、`publish_frequency`、
`enable_spin`、`correct_ratio` 等）在 `config/<mode>/base.yaml` 的
`nonrotating_vel_transform:` 段；用单文件 `params_file:=` 调试时，该文件需自带
这一段（分层文件不再补齐）。

TES 自旋接管：`simple_decision`（`always_tes` 默认 true）运行后持续发布
`chassis_mode=LITTLE_TES` 与 `cmd_spin`，`nonrotating_vel_transform` 将
`cmd_spin` 的角速度叠加到输出。

速度链：

```text
controller_server -> cmd_vel_controller
velocity_smoother -> cmd_vel_smoothed
nonrotating_vel_transform（旋转补偿 + 叠加自旋） -> cmd_vel
串口驱动 -> MCU
```

## TODOLIST
