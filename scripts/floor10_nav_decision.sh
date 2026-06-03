# 北航机器人队十楼实验室导航测试脚本
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source install/setup.bash
ros2 launch guga_nav_bringup rm_navigation_reality_launch_nm_noRobot.py slam:=True behavior_tree_type:=manual
