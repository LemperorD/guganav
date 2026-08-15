from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_share = get_package_share_directory(
        'dynamic_obstacle'
    )

    config_file = os.path.join(
        package_share,
        'config',
        'obstacles.yaml'
    )

    return LaunchDescription([

        Node(
            package='dynamic_obstacle',
            executable='obstacle_manager',
            name='dynamic_obstacle_node',
            output='screen',
            parameters=[
                config_file
            ]
        )

    ])
