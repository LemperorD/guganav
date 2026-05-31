cd "$(dirname "${BASH_SOURCE[0]}")/.."
source ./install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py