# 导航测试脚本
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source install/setup.bash
ros2 launch guga_nav_bringup reality_launch.py \
slam:=True \
use_robot_state_pub:=False \
use_rviz:=True \
use_communication:=True \
behavior_tree_type:=manual
