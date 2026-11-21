from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shm_name = LaunchConfiguration("shm_name")
    write_rate_hz = LaunchConfiguration("write_rate_hz")
    command_timeout_sec = LaunchConfiguration("command_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "shm_name",
                default_value="/s2c_serial_comm",
                description="Sentry2Communicator 共享内存名字",
            ),
            DeclareLaunchArgument(
                "write_rate_hz",
                default_value="100.0",
                description="写入共享内存的频率",
            ),
            DeclareLaunchArgument(
                "command_timeout_sec",
                default_value="0.5",
                description="超过该时长没有新 cmd_vel 就把速度清零",
            ),
            Node(
                package="sentry_comm",
                executable="nav_command_node",
                output="screen",
                parameters=[
                    {
                        "shm_name": shm_name,
                        "write_rate_hz": ParameterValue(
                            write_rate_hz, value_type=float
                        ),
                        "command_timeout_sec": ParameterValue(
                            command_timeout_sec, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
