import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("guga_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # ── 参数文件注入（base → controller → planner 顺序覆盖合并）──
    # simulation.sh 经 simulation_launch/bringup_launch 传入三个文件路径；
    # 显式 params_file:= 时退化为单文件（reality / 临时调试）。
    params_file = LaunchConfiguration("params_file")
    base_params_file = LaunchConfiguration("base_params_file")
    controller_params_file = LaunchConfiguration("controller_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")
    controller = LaunchConfiguration("controller")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

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

    # base(公共) → controller(控制器差异) → planner(规划器差异)，
    # rclcpp 对多参数文件做叶子级覆盖，后者覆盖前者。
    configured_params = [
        rewritten_params(base_params_file),
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

    # 控制器选择：pid/mppi/mpc。底盘模式（启动即小陀螺）由它推断，
    # 不再需要独立的 navigation_profile 参数。
    declare_controller_cmd = DeclareLaunchArgument(
        "controller",
        default_value="pid",
        choices=["pid", "mppi", "mpc"],
        description="Controller profile: pid (omni PID), mppi, or mpc",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # 三文件默认值：显式 params_file 非空时退化为单文件；否则用
    # simulation 默认组合（base + pid + jps）。
    def default_params_file(which):
        return PythonExpression(
            [
                "'", params_file, "' != '' and '", params_file,
                "' or '",
                os.path.join(bringup_dir, "config", "simulation", which),
                "'",
            ]
        )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Full path to a single params file override (disables 3-file merge)",
    )
    declare_base_params_file_cmd = DeclareLaunchArgument(
        "base_params_file",
        default_value=default_params_file("base.yaml"),
        description="Common params file (base layer of the merge)",
    )
    declare_controller_params_file_cmd = DeclareLaunchArgument(
        "controller_params_file",
        default_value=default_params_file("controller/pid.yaml"),
        description="Controller-diff params file (overrides base)",
    )
    declare_planner_params_file_cmd = DeclareLaunchArgument(
        "planner_params_file",
        default_value=default_params_file("planner/jps.yaml"),
        description="Planner-diff params file (overrides base/controller)",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # 非组合模式：独立进程运行 terrain_analysis（组件化后仍保留独立入口）
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrain_analysis_exe",
        name="terrain_analysis",
        output="screen",
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=configured_params,
    )

    # 非组合模式：独立进程运行 terrain_analysis_ext
    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrain_analysis_ext_exe",
        name="terrain_analysis_ext",
        output="screen",
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=configured_params,
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                ],
            ),
            Node(
                package="nonrotating_vel_transform",
                executable="nonrotating_vel_transform_node",
                name="nonrotating_vel_transform",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_base_frame": "base_footprint",
                        "nonrotating_robot_base_frame": "base_footprint_nonrotating",
                        "chassis_frame": "chassis",
                        "odom_topic": "odometry",
                        "local_plan_topic": "local_plan",
                        "input_cmd_vel_topic": "cmd_vel_smoothed",
                        "output_cmd_vel_topic": "cmd_vel",
                        "cmd_spin_topic": "cmd_spin",
                        "chassis_mode_topic": "chassis_mode",
                        # mppi/mpc 走 base_footprint_nonrotating，启动即小陀螺；
                        # pid 的 costmap 仍用旋转的 base_footprint，不能自旋，保持跟随模式
                        "initial_chassis_mode": PythonExpression(
                            [
                                "1 if '", controller, "' in ('mppi', 'mpc') else 0",
                            ]
                        ),
                        "init_spin_speed": PythonExpression(
                            [
                                "6.28 if '", controller, "' in ('mppi', 'mpc') else 0.0",
                            ]
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="terrain_analysis",
                plugin="terrain_analysis::TerrainAnalysis",
                name="terrain_analysis",
                parameters=configured_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="terrain_analysis_ext",
                plugin="terrain_analysis_ext::TerrainAnalysisExtNode",
                name="terrain_analysis_ext",
                parameters=configured_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=configured_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=configured_params,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=configured_params,
                # extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=configured_params,
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=configured_params,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=configured_params,
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=configured_params,
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=configured_params,
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=configured_params,
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                ],
            ),
            ComposableNode(
                package="nonrotating_vel_transform",
                plugin="nonrotating_vel_transform::NonrotatingVelTransform",
                name="nonrotating_vel_transform",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_base_frame": "base_footprint",
                        "nonrotating_robot_base_frame": "base_footprint_nonrotating",
                        "chassis_frame": "chassis",
                        "odom_topic": "odometry",
                        "local_plan_topic": "local_plan",
                        "input_cmd_vel_topic": "cmd_vel_smoothed",
                        "output_cmd_vel_topic": "cmd_vel",
                        "cmd_spin_topic": "cmd_spin",
                        "chassis_mode_topic": "chassis_mode",
                        # mppi/mpc 走 base_footprint_nonrotating，启动即小陀螺；
                        # pid 的 costmap 仍用旋转的 base_footprint，不能自旋，保持跟随模式
                        "initial_chassis_mode": PythonExpression(
                            [
                                "1 if '", controller, "' in ('mppi', 'mpc') else 0",
                            ]
                        ),
                        "init_spin_speed": PythonExpression(
                            [
                                "6.28 if '", controller, "' in ('mppi', 'mpc') else 0.0",
                            ]
                        ),
                    }
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_controller_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_controller_params_file_cmd)
    ld.add_action(declare_planner_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
