from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_actions',
            executable='arm_disarm_server.py',
            name='arm_disarm_server',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions',
            executable='takeoff_action_server.py',
            name='takeoff_action_server',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions',
            executable='land_action_server.py',
            name='land_action_server',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions',
            executable='goto_position_server.py',
            name='goto_position_server',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions',
            executable='set_mode_server.py',
            name='set_mode_server',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions',
            executable='mission_executor_server.py',
            name='mission_executor_server',
            output='screen'
        ),
    ])
