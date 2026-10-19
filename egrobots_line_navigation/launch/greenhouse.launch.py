"""Greenhouse lane navigation: the simulated greenhouse, the rover, and the
navigator set up to drive down a crop lane.

    ros2 launch egrobots_line_navigation greenhouse.launch.py

Then send it down the lane. A and B only say where to start and where to stop;
the crop rows decide where the path runs sideways:

    ros2 action send_goal -f /follow_line \
      egrobots_line_interfaces/action/FollowLine \
      "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 21.0, y: 0.0, z: 0.0}, tolerance: 0.0}"

This is line_navigation.launch.py with the greenhouse world and the greenhouse
parameter profiles; every node is the same.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sensor_suite', default_value='depth'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('row_correction', default_value='true'),
        DeclareLaunchArgument(
            'collision_monitor', default_value='true',
            description='Route commands through nav2_collision_monitor, which '
                        'slows and stops on raw LiDAR regardless of what the '
                        'estimate says'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'safety.launch.py')),
            condition=IfCondition(LaunchConfiguration('collision_monitor'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'line_navigation.launch.py')),
            launch_arguments={'world': 'greenhouse_world.world',
                              'sensor_suite': LaunchConfiguration('sensor_suite'),
                              'nav_params': 'greenhouse_params.yaml',
                              'row_params': 'row_greenhouse.yaml',
                              'ekf_params': 'ekf_greenhouse.yaml',
                              'rviz': LaunchConfiguration('rviz'),
                              'gui': LaunchConfiguration('gui'),
                              'row_correction': LaunchConfiguration('row_correction'),
                              'cmd_vel_topic': PythonExpression(
                                  ["'/cmd_vel_raw' if '",
                                   LaunchConfiguration('collision_monitor'),
                                   "' == 'true' else "
                                   "'/diff_drive_controller/cmd_vel_unstamped'"]),
                              }.items()),
    ])
