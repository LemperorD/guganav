# guga_bringup

bringup模块，存放launch、参数、rviz等文件

仿真导航支持两种可选组合：默认的 `jps_pid` 使用 JPS 全局规划器和全向 PID，
`2d_mppi` 使用 Nav2 `SmacPlanner2D` 和 MPPI。通过
`navigation_profile:=jps_pid` 或 `navigation_profile:=2d_mppi` 选择。

## TODOLIST
