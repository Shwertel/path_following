"""Localize against a prebuilt map: map_server + AMCL + lifecycle manager.

    ros2 launch egrobots_line_navigation localization.launch.py \
        map:=/path/to/greenhouse_raw.yaml

Run this ALONGSIDE greenhouse.launch.py, not instead of it. The EKF keeps
publishing odom -> base_link; AMCL adds map -> odom on top. Nothing else in the
stack changes - the navigator still steers by the lane it measures.

Nav2's servers are managed lifecycle nodes: they start up unconfigured and do
nothing until something walks them through configure -> activate. That is what
nav2_lifecycle_manager is for, and forgetting it is the usual reason a Nav2
node appears to run while publishing nothing at all.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')
    params = os.path.join(pkg_share, 'config', 'amcl.yaml')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'greenhouse.yaml'),
            description='Path to the map .yaml'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, {'use_sim_time': True, 'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])
