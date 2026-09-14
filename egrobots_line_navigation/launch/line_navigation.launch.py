"""Simulation plus the line navigator.

    ros2 launch egrobots_line_navigation line_navigation.launch.py

    ros2 action send_goal -f /follow_line \
      egrobots_line_interfaces/action/FollowLine \
      "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 8.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'),
                          'gui': LaunchConfiguration('gui')}.items())

    navigator = Node(
        package='egrobots_line_navigation',
        executable='line_navigator_node',
        name='line_navigator_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'line_params.yaml'),
                    {'use_sim_time': True}],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        simulation,
        navigator,
    ])
