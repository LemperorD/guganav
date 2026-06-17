import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Hikrobot camera driver node."""

    hik_dir = get_package_share_directory("hik_driver")

    # Launch arguments
    declared_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([hik_dir, "config", "hik.yaml"]),
        description="Path to the Hik camera configuration YAML file",
    )

    declared_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS logging level",
    )

    # Environment
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )

    # Node
    start_hik_camera = Node(
        name="hik_camera_node",
        package="hik_driver",
        executable="hik_driver_node_exe",
        parameters=[LaunchConfiguration("params_file")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        colorized_output_envvar,
        declared_params_file,
        declared_log_level,
        start_hik_camera,
    ])
