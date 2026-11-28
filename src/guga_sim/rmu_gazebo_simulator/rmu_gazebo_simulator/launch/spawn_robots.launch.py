# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def _strip_robot_visuals(robot_xml: str) -> str:
    """移除 Gazebo 机器人 SDF 中的所有 <visual>（及依赖它的 LightBarController），
    彻底消除车体/装甲板对 gpu_lidar 的近距遮挡。

    不影响 physics 所需的 <collision>/<inertial>/<sensor>，
    也不影响传给 robot_state_publisher 的 URDF（RViz 中仍正常显示整车）。
    """
    root = ET.fromstring(robot_xml)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "visual":
                parent.remove(child)
            elif child.tag == "plugin" and "LightBarController" in (child.get("name") or ""):
                parent.remove(child)
    return ET.tostring(root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = os.path.join(
        pkg_pb2025_robot_description,
        "resource",
        "xmacro",
        "simulation_robot.sdf.xmacro",
    )
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # Get spawn robot init pose
    gz_world_path = LaunchConfiguration("gz_world_path").perform(context)
    selected_world = LaunchConfiguration("world").perform(context)
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        robots = config["robots"].get(selected_world)

    if robots is None:
        raise RuntimeError(f"Unknown Gazebo world: {selected_world}")

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    actions = []

    for robot_index, robot in enumerate(robots):
        # Generate SDF from xmacro
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        # 避免 gpu_lidar 渲染本车外壳/装甲板导致近距盲区（RViz 与物理碰撞不受影响）
        robot_xml = _strip_robot_visuals(robot_xml)

        # replace the <robot_name> in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        actions.extend(
            [
                spawn_robot,
                robot_base,
                robot_state_publisher,
                robot_ign_bridge,
                set_performer_service,
            ]
        )

    return actions


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        default_world = config.get("world")

    declare_gz_world_path_cmd = DeclareLaunchArgument(
        "gz_world_path",
        default_value=gz_world_path,
        description="Path to Gazebo world and robot spawn configuration",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="World key used to choose robot spawn poses",
    )

    ld = LaunchDescription()
    ld.add_action(declare_gz_world_path_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
