import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("guga_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    show_prior_pcd = LaunchConfiguration("show_prior_pcd")
    pcd_save = LaunchConfiguration("pcd_save")
    pcd_save_path = LaunchConfiguration("pcd_save_path")
    pcd_save_require_lost = LaunchConfiguration("pcd_save_require_lost")
    pcd_save_trigger_topic = LaunchConfiguration("pcd_save_trigger_topic")
    params_file = LaunchConfiguration("params_file")
    base_params_file = LaunchConfiguration("base_params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    map_lifecycle_nodes = ["map_server"]
    relocalization_lifecycle_nodes = ["small_gicp_relocalization"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=base_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_show_prior_pcd_cmd = DeclareLaunchArgument(
        "show_prior_pcd",
        default_value="True",
        description="Publish the transformed prior PCD on /cloud_pcd",
    )

    declare_pcd_save_cmd = DeclareLaunchArgument(
        "pcd_save",
        default_value="True",
        description="Save accumulated scans to point_lio/PCD on shutdown in navigation mode",
    )

    declare_pcd_save_path_cmd = DeclareLaunchArgument(
        "pcd_save_path",
        default_value="",
        description=(
            "Full PCD output path; empty keeps the timestamped "
            "point_lio/PCD default"
        ),
    )

    declare_pcd_save_trigger_topic_cmd = DeclareLaunchArgument(
        "pcd_save_trigger_topic",
        default_value="gicp_lost",
        description=(
            "Bool topic used when pcd_save_require_lost is True; saving starts "
            "after it receives true"
        ),
    )

    declare_pcd_save_require_lost_cmd = DeclareLaunchArgument(
        "pcd_save_require_lost",
        default_value="True",
        description=(
            "Wait for the GICP LOST trigger before saving PCD; False saves "
            "from startup"
        ),
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_base_params_file_cmd = DeclareLaunchArgument(
        "base_params_file",
        default_value=PythonExpression(
            [
                "'", params_file, "' != '' and '", params_file, "' or '",
                os.path.join(bringup_dir, "config", "simulation", "base.yaml"), "'",
            ]
        ),
        description="Common params file (base layer); falls back to params_file if set",
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

    start_point_lio_node = LifecycleNode(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        namespace="",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
            {"pcd_save.pcd_save_en": ParameterValue(pcd_save, value_type=bool)},
            {"pcd_save.path": pcd_save_path},
            {"pcd_save.require_lost": ParameterValue(pcd_save_require_lost, value_type=bool)},
            {"pcd_save.trigger_topic": pcd_save_trigger_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    configure_point_lio = RegisterEventHandler(
        OnProcessStart(
            target_action=start_point_lio_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_point_lio_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_point_lio = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_point_lio_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_point_lio_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        "prior_pcd_file": prior_pcd_file,
                        "publish_prior_pcd": ParameterValue(
                            show_prior_pcd, value_type=bool
                        ),
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": map_lifecycle_nodes},
                ],
            ),
            # The relocalization node has its own lifecycle and deliberately
            # deactivates itself on LOST. It does not use Nav2's bond heartbeat,
            # so manage it separately with bond checking disabled.
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_relocalization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"bond_timeout": 0.0},
                    {"node_names": relocalization_lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[
                    configured_params,
                    {
                        "prior_pcd_file": prior_pcd_file,
                        "publish_prior_pcd": ParameterValue(
                            show_prior_pcd, value_type=bool
                        ),
                    },
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": map_lifecycle_nodes,
                    }
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_relocalization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "bond_timeout": 0.0,
                        "node_names": relocalization_lifecycle_nodes,
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
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_show_prior_pcd_cmd)
    ld.add_action(declare_pcd_save_cmd)
    ld.add_action(declare_pcd_save_path_cmd)
    ld.add_action(declare_pcd_save_trigger_topic_cmd)
    ld.add_action(declare_pcd_save_require_lost_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(configure_point_lio)
    ld.add_action(activate_point_lio)
    ld.add_action(start_point_lio_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
