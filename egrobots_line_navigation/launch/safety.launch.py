"""nav2_collision_monitor: stop and slowdown zones read straight off the LiDAR.

Started by greenhouse.launch.py. It sits between the navigator and the drive
controller - the navigator publishes to /cmd_vel_raw and this republishes to
/diff_drive_controller/cmd_vel_unstamped - so if it is not running, nothing
moves. That is deliberate: a safety layer you can forget to start is not one.

Like the other Nav2 servers it is a managed lifecycle node and does nothing
until the lifecycle manager activates it.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')
    params = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

    return LaunchDescription([
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             parameters=[params, {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_safety', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['collision_monitor']}]),
    ])
