import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_actions_cpp',
            executable='arm_disarm_server',
            name='arm_disarm_server_cpp',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions_cpp',
            executable='takeoff_land_server',
            name='takeoff_land_server_cpp',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions_cpp',
            executable='goto_position_server',
            name='goto_position_server_cpp',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions_cpp',
            executable='set_mode_server',
            name='set_mode_server_cpp',
            output='screen'
        ),
        Node(
            package='px4_ros2_actions_cpp',
            executable='mission_executor_server',
            name='mission_executor_server_cpp',
            output='screen'
        )
    ])
