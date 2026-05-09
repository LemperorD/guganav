from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    default_params = os.path.join(
        get_package_share_directory("simple_decision"),
        "config", 
        "simple_decision.yaml",
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="simple_decision params",
    )

    container = ComposableNodeContainer(
        name="simple_decision_container",
        namespace="", 
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="simple_decision",
                plugin="simple_decision::DecisionSimple",
                name="simple_decision",
                parameters=[params_file],
            ),
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_params,
        GroupAction([
            PushRosNamespace(namespace),
            container,
        ]),
    ])
