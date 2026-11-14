from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tick_rate_hz = LaunchConfiguration('tick_rate_hz')
    btlog_path = LaunchConfiguration('btlog_path')
    navigate_to_pose_action = LaunchConfiguration('navigate_to_pose_action')
    goal_pose_marker_topic = LaunchConfiguration('goal_pose_marker_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tick_rate_hz',
            default_value='1.0',
            description='Behavior tree tick rate in Hz',
        ),
        DeclareLaunchArgument(
            'btlog_path',
            default_value='/tmp/bt_trace.btlog',
            description='BehaviorTree.CPP v4 execution log path; empty disables it',
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_action',
            default_value='navigate_to_pose',
            description='Nav2 NavigateToPose action used by GotoPoints',
        ),
        DeclareLaunchArgument(
            'goal_pose_marker_topic',
            default_value='goal_pose_marker',
            description='RViz marker topic showing the current decision goal',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the Gazebo /clock when running in simulation',
        ),
        Node(
            package='rmul_decision',
            executable='rmul_decision',
            name='decision_node',
            output='screen',
            parameters=[{
                'tick_rate_hz': tick_rate_hz,
                'btlog_path': btlog_path,
                'navigate_to_pose_action': navigate_to_pose_action,
                'goal_pose_marker_topic': goal_pose_marker_topic,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
