import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("guga_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_communication = LaunchConfiguration("use_communication")
    use_ui = LaunchConfiguration("use_ui")
    use_decision = LaunchConfiguration("use_decision")
    # ── 参数分层（与 simulation 同机制）：planner/controller 选择 → 三文件合并 ──
    planner = LaunchConfiguration("planner")
    controller = LaunchConfiguration("controller")
    base_params_file = LaunchConfiguration("base_params_file")
    controller_params_file = LaunchConfiguration("controller_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "reality", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "reality", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        # 默认必须指向真实文件,不能为空字符串:
        # reality 会把 params_file 原样传给 bringup_launch,而 bringup 用它作为
        # RewrittenYaml 的参数源;若为空 → open('') → "No such file or directory: ''"。
        default_value=os.path.join(bringup_dir, "config", "reality", "nav2_params.yaml"),
        description=(
            "Single params file override (disables 3-file merge); "
            "default uses reality/nav2_params.yaml"
        ),
    )
    declare_planner_cmd = DeclareLaunchArgument(
        "planner", default_value="jps", choices=["jps", "smac2d", "smachybrid"],
        description="Global planner: jps, smac2d, or smachybrid",
    )
    declare_controller_cmd = DeclareLaunchArgument(
        "controller", default_value="pid", choices=["pid", "mppi", "mpc"],
        description="Controller: pid (omni PID), mppi, or mpc",
    )
    def default_params_file(which):
        return PythonExpression(
            [
                "'", params_file, "' != '' and '", params_file,
                "' or '", os.path.join(bringup_dir, "config", "reality", which), "'",
            ]
        )
    declare_base_params_file_cmd = DeclareLaunchArgument(
        "base_params_file", default_value=default_params_file("base.yaml"),
        description="Common params file (merge base layer)",
    )
    declare_controller_params_file_cmd = DeclareLaunchArgument(
        "controller_params_file",
        default_value=PythonExpression(
            [
                "'", params_file, "' != '' and '", params_file, "' or ('",
                controller, "' == 'mppi' and '",
                os.path.join(bringup_dir, "config", "reality", "controller", "mppi.yaml"),
                "' or '",
                os.path.join(bringup_dir, "config", "reality", "controller", "pid.yaml"),
                "')",
            ]
        ),
        description="Controller-diff params file (pid default, mppi available)",
    )
    declare_planner_params_file_cmd = DeclareLaunchArgument(
        "planner_params_file", default_value=default_params_file("planner/jps.yaml"),
        description="Planner-diff params file",
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

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        # default_value="False", # disable robot_state_publisher when using reality
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
        # "use_rviz", default_value="False", description="Whether to start RVIZ"
    )

    declare_use_communication_cmd = DeclareLaunchArgument(
        "use_communication",
        default_value="True",
        description="Whether to start the communication node",
    )

    declare_use_ui_cmd = DeclareLaunchArgument(
        "use_ui",
        default_value="False",
        description="Whether to start the guga_ui_pangolin process",
    )

    declare_use_decision_cmd = DeclareLaunchArgument(
        "use_decision",
        default_value="False",
        description="Whether to start simple_decision",
    )

    # Create our own temporary YAML files that include substitutions

    # livox 节点参数直接指向 base.yaml 的绝对路径,不经过 PythonExpression
    # 动态求值(base_params_file 在 bringup 层才被 ReplaceString 处理,此处
    # 求值为空会触发 RewrittenYaml 打开 '' → FileNotFoundError)。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(bringup_dir, "config", "reality", "base.yaml"),
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # When not using robot state publisher, start static TF publisher
    start_static_tf_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "static_tf_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(PythonExpression(["not ", use_robot_state_pub])),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "core", "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "base_params_file": base_params_file,
            "controller_params_file": controller_params_file,
            "planner_params_file": planner_params_file,
            "controller": controller,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    communication_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "communication_launch.py")
        ),
        condition=IfCondition(use_communication),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    guga_ui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "guga_ui_launch.py")
        ),
        condition=IfCondition(use_ui),
    )

    decision_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("simple_decision"),
                "launch",
                "simple_decision.launch.py",
            )
        ),
        condition=IfCondition(use_decision),
        launch_arguments={"namespace": namespace}.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_controller_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_controller_params_file_cmd)
    ld.add_action(declare_planner_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_communication_cmd)
    ld.add_action(declare_use_ui_cmd)
    ld.add_action(declare_use_decision_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_static_tf_publisher_cmd)
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(communication_cmd)
    ld.add_action(guga_ui_cmd)
    ld.add_action(decision_cmd)

    return ld
