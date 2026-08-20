import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("guga_bringup")

    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    base_params_file = LaunchConfiguration("base_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Variables
    lifecycle_nodes = ["map_saver"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=base_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_map_saver_server_cmd = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_sync_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    # small_point_lio 替换 point_lio 作为 slam 模式的里程计源。
    # - 配置按场景区分(用 use_sim_time 判断,不依赖 base_params_file 字符串,
    #   因为它经过 ReplaceString 处理,PythonExpression 无法可靠求值):
    #   仿真(use_sim_time=True)→ config/velodyne.yaml(通用 PointCloud2);
    #   实车(use_sim_time=False)→ config/mid360.yaml(livox CustomMsg)。
    # - 输出 /Odometry、/cloud_registered 均为 odom 系,直接 remap 成下游
    #   terrain_analysis / sensor_scan_generation 期望的 lidar_odometry /
    #   registered_scan,绕开 loam_interface。
    # - publish_tf=False:odom->base_footprint 由 sensor_scan_generation 统一发布。
    small_lio_config = PythonExpression(
        [
            "'",
            use_sim_time,
            "' == 'True' and '",
            os.path.join(get_package_share_directory("small_point_lio"), "config",
                         "velodyne.yaml"),
            "' or '",
            os.path.join(get_package_share_directory("small_point_lio"), "config",
                         "mid360.yaml"),
            "'",
        ]
    )

    start_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            small_lio_config,  # 直接传配置文件路径(文件有自身顶层 key small_point_lio)
            {"publish_tf": False},  # odom->base_footprint 由 sensor_scan_generation 统一发布
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/Odometry", "lidar_odometry"),
            ("/cloud_registered", "registered_scan"),
        ],
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # 仿真 TF 树缺 base_footprint 层(URDF 根是 chassis,static_tf 只在实车跑)。
    # 补 base_footprint->chassis 静态 TF,让 sensor_scan_generation 能查到
    # front_mid360->base_footprint 外参链,否则其 getTransform fallback identity,
    # 合成 odom->base_footprint 错位 → 小车脱离地图。
    # 仿真 chassis 由 RSP 从 URDF 发布,此处 base_footprint 与 chassis 原点重合,
    # 避免引入未知偏移(实车 static_tf 用 z=0.123,仿真 URDF 底盘高 0.15,以 RSP 为准)。
    start_static_base_footprint_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_base_footprint2chassis",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "chassis",
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_base_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)
    ld.add_action(start_static_base_footprint_tf_node)

    return ld
