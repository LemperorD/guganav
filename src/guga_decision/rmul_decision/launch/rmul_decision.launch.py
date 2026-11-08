from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmul_decision',
            executable='rmul_decision',
            name='decision_node',
            output='screen'
        )
    ])