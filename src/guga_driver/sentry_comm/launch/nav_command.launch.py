from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shm_name = LaunchConfiguration("shm_name")
    write_rate_hz = LaunchConfiguration("write_rate_hz")
    command_timeout_sec = LaunchConfiguration("command_timeout_sec")
    vel_trans_scale = LaunchConfiguration("vel_trans_scale")
    vel_yaw_scale = LaunchConfiguration("vel_yaw_scale")

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
            DeclareLaunchArgument(
                "vel_trans_scale",
                default_value="40.0",
                description="平移速度下发缩放（与旧 serial_driver 的 vel_trans_scale 一致）",
            ),
            DeclareLaunchArgument(
                "vel_yaw_scale",
                default_value="1.0",
                description="yaw 角速度下发缩放（旧实现不缩放，保持 1.0）",
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
                        "vel_trans_scale": ParameterValue(
                            vel_trans_scale, value_type=float
                        ),
                        "vel_yaw_scale": ParameterValue(
                            vel_yaw_scale, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
