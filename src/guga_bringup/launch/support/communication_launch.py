import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    communication_cfg_dir = LaunchConfiguration("communication_cfg_dir")
    use_sentry_comm = LaunchConfiguration("use_sentry_comm")
    sentry_comm_shm_name = LaunchConfiguration("sentry_comm_shm_name")
    communication_dir = get_package_share_directory("serial_driver")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_communication_cfg_dir = DeclareLaunchArgument(
        "communication_cfg_dir",
        default_value=PathJoinSubstitution(
            [communication_dir, "config", "serial_driver.yaml"]
        ),
        description="Path to the communication config file",
    )

    declare_use_sentry_comm = DeclareLaunchArgument(
        "use_sentry_comm",
        default_value="True",
        description="True 时启动 sentry_comm 的节点，False 时启动旧 serial_driver",
    )

    declare_sentry_comm_shm_name = DeclareLaunchArgument(
        "sentry_comm_shm_name",
        default_value="/s2c_serial_comm",
        description="Sentry2Communicator 共享内存名字",
    )

    start_communication_node = Node(
        package="serial_driver",
        executable="serial_driver_node_exe",
        namespace=namespace,
        parameters=[communication_cfg_dir],
        output="screen",
        condition=UnlessCondition(use_sentry_comm),
    )

    start_nav_command_node = Node(
        package="sentry_comm",
        executable="nav_command_node",
        namespace=namespace,
        output="screen",
        parameters=[{"shm_name": sentry_comm_shm_name}],
        condition=IfCondition(use_sentry_comm),
    )

    start_referee_bridge_node = Node(
        package="sentry_comm",
        executable="referee_bridge_node",
        output="screen",
        parameters=[{"shm_name": sentry_comm_shm_name}],
        condition=IfCondition(use_sentry_comm),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_communication_cfg_dir)
    ld.add_action(declare_use_sentry_comm)
    ld.add_action(declare_sentry_comm_shm_name)
    ld.add_action(start_communication_node)
    ld.add_action(start_nav_command_node)
    ld.add_action(start_referee_bridge_node)

    return ld
