import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory("hik_camera_ros2_driver")

    params_file = LaunchConfiguration("params_file")
    board_file = LaunchConfiguration("board_file")
    log_level = LaunchConfiguration("log_level")

    # ========== 读取标定板参数 ==========
    board_yaml_path = os.path.join(pkg_dir, "config", "calibration_board.yaml")

    with open(board_yaml_path, "r") as f:
        board_cfg = yaml.safe_load(f)["calibration"]

    board_w = board_cfg["board_width"]
    board_h = board_cfg["board_height"]
    square_m = board_cfg["square_size_mm"] / 1000.0  # 转为米（camera_calibration要求）

    # ========== 环境变量 ==========
    stdout_env = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    color_env = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )

    # ========== 参数声明 ==========
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "camera_params.yaml"),
        description="Camera driver parameter file",
    )

    declare_board_file_cmd = DeclareLaunchArgument(
        "board_file",
        default_value=os.path.join(pkg_dir, "config", "calibration_board.yaml"),
        description="Calibration board config file",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level"
    )

    # ========== 相机驱动节点 ==========
    camera_node = Node(
        name="hik_camera_ros2_driver",
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    # ========== 标定节点 ==========
    calib_node = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="camera_calibrator",
        output="screen",
        arguments=[
            "--size", f"{board_w}x{board_h}",
            "--square", str(square_m),
            "--ros-args",
            "--remap", "image:=/camera/image",
            "--remap", "camera:=/camera"
        ]
    )

    # ========== LaunchDescription ==========
    ld = LaunchDescription()

    ld.add_action(stdout_env)
    ld.add_action(color_env)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_board_file_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(camera_node)
    ld.add_action(calib_node)

    return ld
