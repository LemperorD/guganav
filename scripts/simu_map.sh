cd "$(dirname "${BASH_SOURCE[0]}")/.."
source ./install/setup.bash
ros2 launch guga_nav_bringup rm_navigation_simulation_launch.py \
world:=rmul_2025 \
slam:=True
