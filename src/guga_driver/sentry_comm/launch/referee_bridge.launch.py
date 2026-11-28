from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shm_name = LaunchConfiguration("shm_name")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "shm_name",
                default_value="/s2c_serial_comm",
                description="Sentry2Communicator 共享内存名字",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="50.0",
                description="裁判系统话题的发布频率",
            ),
            Node(
                package="sentry_comm",
                executable="referee_bridge_node",
                output="screen",
                parameters=[
                    {
                        "shm_name": shm_name,
                        "publish_rate_hz": ParameterValue(
                            publish_rate_hz, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
