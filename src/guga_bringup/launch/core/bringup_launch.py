import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution as Equals,
    IfElseSubstitution as IfElse,
    LaunchConfiguration,
    NotEqualsSubstitution as NotEquals,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("guga_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    # 导航参数三文件（base/controller/planner）与控制器选择，原样透传给 navigation_launch
    base_params_file = LaunchConfiguration("base_params_file")
    controller_params_file = LaunchConfiguration("controller_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")
    controller = LaunchConfiguration("controller")

    # ── <robot_namespace> 文件替换 ──
    # nav2_common 的 ReplaceString 把输入当文件打开：simulation 分层模式下
    # params_file 为空字符串，必须用条件跳过；替换值按 namespace 动态选择。
    # base/controller/planner 三文件同样替换（costmap topic 含 <robot_namespace>）。
    def with_namespace_replace(sub):
        return ReplaceString(
            condition=IfCondition(NotEquals(sub, "")),
            source_file=sub,
            replacements={
                "<robot_namespace>": IfElse(
                    Equals(LaunchConfiguration("namespace"), ""),
                    "",
                    ["/", namespace],
                )
            },
        )

    params_file = with_namespace_replace(params_file)
    base_params_file = with_namespace_replace(base_params_file)
    controller_params_file = with_namespace_replace(controller_params_file)
    planner_params_file = with_namespace_replace(planner_params_file)
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    def rewritten_params(source):
        return ParameterFile(
            RewrittenYaml(
                source_file=source,
                root_key=namespace,
                param_rewrites=param_substitutions,
                convert_types=True,
            ),
            allow_substs=True,
        )

    # 传感器/公共节点（point_lio、terrain_analysis 等）只读 base 层参数。
    configured_params = rewritten_params(base_params_file)

    # Costmap2DROS 是 controller/planner 组件内部创建的子节点，不会继承
    # LoadComposableNodes 发给父组件的参数。组合模式下必须把完整的
    # base -> controller -> planner 参数链加到容器命令行，才能让
    # local_costmap/global_costmap 子节点按名称读到各层配置。
    container_params = [
        configured_params,
        rewritten_params(controller_params_file),
        rewritten_params(planner_params_file),
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_controller_cmd = DeclareLaunchArgument(
        "controller",
        default_value="pid",
        choices=["pid", "mppi", "mpc"],
        description="Controller profile, forwarded to navigation_launch for chassis mode",
    )
    # 三文件默认值：显式 params_file 非空（reality / 调试）时退化为单文件；
    # 否则用 simulation 分层默认（simulation_launch 总会显式传入覆盖）。
    def default_params_file(which):
        return PythonExpression(
            [
                "'", params_file, "' != '' and '", params_file,
                "' or '",
                os.path.join(bringup_dir, "config", "simulation", which),
                "'",
            ]
        )

    declare_base_params_file_cmd = DeclareLaunchArgument(
        "base_params_file",
        default_value=default_params_file("base.yaml"),
        description="Common params file (merge base layer)",
    )
    declare_controller_params_file_cmd = DeclareLaunchArgument(
        "controller_params_file",
        default_value=default_params_file("controller/pid.yaml"),
        description="Controller-diff params file",
    )
    declare_planner_params_file_cmd = DeclareLaunchArgument(
        "planner_params_file",
        default_value=default_params_file("planner/jps.yaml"),
        description="Planner-diff params file",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether to run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file", description="Full path to prior PCD file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[*container_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "core", "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                    "base_params_file": base_params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "core", "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "base_params_file": base_params_file,
                    "prior_pcd_file": prior_pcd_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "core", "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "base_params_file": base_params_file,
                    "controller_params_file": controller_params_file,
                    "planner_params_file": planner_params_file,
                    "controller": controller,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_controller_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_controller_params_file_cmd)
    ld.add_action(declare_planner_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
