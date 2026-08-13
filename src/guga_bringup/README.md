# guga_bringup

bringup模块，存放launch、参数、rviz等文件

仿真导航支持两种可选组合：默认的 `jps_pid` 使用 JPS 全局规划器和全向 PID，
`2d_mppi` 使用 Nav2 `SmacPlanner2D` 和 MPPI。通过
`navigation_profile:=jps_pid` 或 `navigation_profile:=2d_mppi` 选择。

两种 profile 均使用 `base_footprint`，不需要 `gimbal_yaw_fake`。小陀螺由
`chassis_mode=2` 和 `cmd_spin` 控制：PID profile 直接输出目标 `wz`，MPPI
profile 使用 `guga_mppi_critics/SpinCritic` 在 rollout 中优化目标 `wz`；输出都继续经过
`velocity_smoother` 后发布到 `/cmd_vel`。

## TODOLIST
