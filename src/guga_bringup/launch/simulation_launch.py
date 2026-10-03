import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("guga_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

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
    use_rviz = LaunchConfiguration("use_rviz")
    use_ui = LaunchConfiguration("use_ui")
    # ── 导航参数分层：base -> controller -> planner -> smoother ──
    # 显式 params_file:= 时退化为单文件（四份都指向它）
    planner = LaunchConfiguration("planner")
    controller = LaunchConfiguration("controller")
    smoother = LaunchConfiguration("smoother")
    base_params_file = LaunchConfiguration("base_params_file")
    controller_params_file = LaunchConfiguration("controller_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")
    smoother_params_file = LaunchConfiguration("smoother_params_file")

    # 传感器工具节点只读 base 层公共参数
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=base_params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether to run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmuc_2025",
        description="Select world: 'rmul_2024' or 'rmuc_2024' or 'rmul_2025' or 'rmuc_2025' or 'rmuc_2026' or 'rmul_2026'(map file share the same name as this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "simulation", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    # planner/controller 决定前三层参数文件，smoother 选择最后一层；
    # 各层在运行时合并，不需要为每个组合准备独立 yaml。
    declare_planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value="jps",
        choices=["jps", "smac2d", "smachybrid"],
        description="Global planner: jps, smac2d, or smachybrid",
    )
    declare_controller_cmd = DeclareLaunchArgument(
        "controller",
        # 建图模式（slam:=True）默认 mppi：costmap 走 base_footprint_nonrotating，
        # 且 navigation_launch 由 controller 推断底盘模式 → 开局即小陀螺；
        # 比赛模式默认 pid（保守，不自旋）
        default_value=PythonExpression(["'", slam, "' == 'True' and 'mppi' or 'pid'"]),
        choices=["pid", "mppi", "mpc"],
        description="Controller: pid (omni PID), mppi, or mpc",
    )
    declare_smoother_cmd = DeclareLaunchArgument(
        "smoother",
        default_value="planner",
        choices=["planner", "minco"],
        description=(
            "Path smoother: planner keeps planner-native smoothing; minco uses "
            "pb_minco_smoother through Nav2 smoother_server"
        ),
    )

    # 前三层默认值：显式 params_file 非空时都指向它（单文件覆盖）；
    # 否则按 planner/controller 名字取 config/simulation 下的分层文件。
    def sim_params_file(subdir, name):
        return PythonExpression(
            [
                "'",
                params_file,
                "' != '' and '",
                params_file,
                "' or '",
                os.path.join(bringup_dir, "config", "simulation", subdir, name),
                "'",
            ]
        )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Single params file override (disables layered merge)",
    )
    declare_base_params_file_cmd = DeclareLaunchArgument(
        "base_params_file",
        default_value=sim_params_file("", "base.yaml"),
        description="Common params file (merge base layer)",
    )
    # controller 名 → 文件：三个完整路径条件选择（值带引号嵌入表达式，
    # 避免字符串拼接导致裸标识符）
    declare_controller_params_file_cmd = DeclareLaunchArgument(
        "controller_params_file",
        default_value=PythonExpression(
            [
                "'",
                params_file,
                "' != '' and '",
                params_file,
                "' or ('",
                controller,
                "' == 'mppi' and '",
                os.path.join(
                    bringup_dir, "config", "simulation", "controller", "mppi.yaml"
                ),
                "' or '",
                controller,
                "' == 'mpc' and '",
                os.path.join(
                    bringup_dir, "config", "simulation", "controller", "mpc.yaml"
                ),
                "' or '",
                os.path.join(
                    bringup_dir, "config", "simulation", "controller", "pid.yaml"
                ),
                "')",
            ]
        ),
        description="Controller-diff params file (overrides base)",
    )
    declare_planner_params_file_cmd = DeclareLaunchArgument(
        "planner_params_file",
        default_value=PythonExpression(
            [
                "'",
                params_file,
                "' != '' and '",
                params_file,
                "' or ('",
                # 建图模式（slam:=True）且 JPS 时：用 jps_slam.yaml（allow_unknown=true，
                # 敢走未探索区域）；比赛模式保持 jps.yaml 保守配置
                slam,
                "' == 'True' and '",
                planner,
                "' == 'jps' and '",
                os.path.join(
                    bringup_dir, "config", "simulation", "planner", "jps_slam.yaml"
                ),
                "' or '",
                planner,
                "' == 'smac2d' and '",
                os.path.join(
                    bringup_dir, "config", "simulation", "planner", "smac2d.yaml"
                ),
                "' or '",
                planner,
                "' == 'smachybrid' and '",
                os.path.join(
                    bringup_dir, "config", "simulation", "planner", "smachybrid.yaml"
                ),
                "' or '",
                os.path.join(
                    bringup_dir, "config", "simulation", "planner", "jps.yaml"
                ),
                "')",
            ]
        ),
        description="Planner-diff params file (overrides base/controller)",
    )
    declare_smoother_params_file_cmd = DeclareLaunchArgument(
        "smoother_params_file",
        default_value=PythonExpression(
            [
                "'",
                params_file,
                "' != '' and '",
                params_file,
                "' or ('",
                smoother,
                "' == 'minco' and '",
                os.path.join(bringup_dir, "config", "smoother", "minco.yaml"),
                "' or '",
                os.path.join(bringup_dir, "config", "smoother", "planner.yaml"),
                "')",
            ]
        ),
        description="Smoother-diff params file (final merge layer)",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description=(
            "Whether to use composed bringup. Disabled by default in simulation "
            "to avoid ROS 2 Humble LoadComposableNodes shutdown races."
        ),
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view_mpc.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_ui_cmd = DeclareLaunchArgument(
        "use_ui",
        default_value="False",
        description="Whether to start the guga_ui_pangolin process",
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
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
            "smoother_params_file": smoother_params_file,
            "planner": planner,
            "controller": controller,
            "smoother": smoother,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    guga_ui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "support", "guga_ui_launch.py")
        ),
        condition=IfCondition(use_ui),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_controller_cmd)
    ld.add_action(declare_smoother_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_controller_params_file_cmd)
    ld.add_action(declare_planner_params_file_cmd)
    ld.add_action(declare_smoother_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_ui_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(guga_ui_cmd)

    return ld
