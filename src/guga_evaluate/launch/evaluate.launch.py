import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("guga_evaluate")

    namespace = LaunchConfiguration("namespace")
    mode = LaunchConfiguration("mode")
    output_dir = LaunchConfiguration("output_dir")
    params_file = LaunchConfiguration("params_file")
    workspace = LaunchConfiguration("workspace")
    use_ground_truth = LaunchConfiguration("use_ground_truth")
    use_sim_time = LaunchConfiguration("use_sim_time")
    save_data = LaunchConfiguration("save_data")
    show_visualization = LaunchConfiguration("show_visualization")

    declarations = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"),
        DeclareLaunchArgument(
            "mode", default_value="reality",
            description="Evaluation mode label: reality or simulation"),
        DeclareLaunchArgument(
            "output_dir", default_value="/tmp/guga_evaluate",
            description="Directory for CSV and JSON results"),
        DeclareLaunchArgument(
            "workspace", default_value="",
            description="Workspace path recorded in metadata"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_dir, "config", "evaluate.yaml"),
            description="Evaluation parameter file"),
        DeclareLaunchArgument(
            "use_ground_truth", default_value="false",
            description="Subscribe to the Gazebo ground-truth odometry topic"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use the Gazebo clock"),
        DeclareLaunchArgument(
            "save_data", default_value="false",
            description="Write CSV and JSON evaluation files"),
        DeclareLaunchArgument(
            "show_visualization", default_value="true",
            description="Open the live Matplotlib dashboard"),
    ]

    evaluator = Node(
        package="guga_evaluate",
        executable="evaluate_node",
        name="guga_evaluate",
        output="screen",
        parameters=[
            params_file,
            {
                "mode": mode,
                "output_dir": output_dir,
                "workspace": workspace,
                "save_data": ParameterValue(save_data, value_type=bool),
                "use_ground_truth": ParameterValue(
                    use_ground_truth, value_type=bool),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    visualizer = Node(
        package="guga_evaluate",
        executable="visualize_node",
        name="guga_evaluate_visualizer",
        output="screen",
        condition=IfCondition(show_visualization),
        parameters=[params_file, {"use_sim_time": ParameterValue(
            use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(
        declarations
        + [GroupAction([
            PushRosNamespace(namespace), evaluator, visualizer,
        ])]
    )
