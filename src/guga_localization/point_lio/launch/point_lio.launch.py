import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_node = LifecycleNode(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
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

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(configure_point_lio)
    ld.add_action(activate_point_lio)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_rviz_node)

    return ld
