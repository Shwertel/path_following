"""Nav2's planner, controller, recovery behaviours and behaviour tree.

    ros2 launch egrobots_line_navigation navigation.launch.py

Runs alongside greenhouse.launch.py and localization.launch.py. It handles the
parts of a sweep that are not inside a lane - the headland turns and the
approach to the charging bay - through NavigateToPose. Lane legs stay with
FollowLine and the crop-row controller, which holds a 1.6 m lane to about 3 cm.

The controller's commands go to /cmd_vel_raw so they pass through the collision
monitor, like the navigator's.

Everything here is a managed lifecycle node and does nothing until the
lifecycle manager activates it.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')
    params = os.path.join(pkg_share, 'config', 'nav2.yaml')
    # The stock behaviour tree hard-codes which controller to use; ours reads it
    # from a topic, so lane legs and headland legs can be driven differently.
    bt_xml = os.path.join(pkg_share, 'behavior_trees',
                          'navigate_with_controller_choice.xml')
    common = [params, {'use_sim_time': True,
                       'default_nav_to_pose_bt_xml': bt_xml}]

    servers = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common,
             remappings=[('cmd_vel', '/cmd_vel_raw')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': servers}]),
    ])
