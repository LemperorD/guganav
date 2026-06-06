cd "$(dirname "${BASH_SOURCE[0]}")/.."
source ./install/setup.bash
ros2 launch guga_nav_bringup reality_launch.py \
slam:=True
