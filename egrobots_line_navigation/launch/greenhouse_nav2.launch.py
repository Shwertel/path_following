"""The whole greenhouse stack in one launch: simulation, localization, Nav2.

    ros2 launch egrobots_line_navigation greenhouse_nav2.launch.py
    ros2 run egrobots_line_navigation greenhouse_mission_node --ros-args -p use_sim_time:=true

Equivalent to running greenhouse.launch.py, localization.launch.py and
navigation.launch.py separately, which also works - but starting them together
avoids a class of problem that cost a lot of time: nodes joining the graph in
separate waves sometimes failed to discover each other, and the symptoms were
indirect (costmaps waiting forever for a transform that was being published,
lifecycle transitions timing out).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')

    def include(name, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', name)),
            launch_arguments=args.items() if args else None)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sensor_suite', default_value='depth'),
        # The inner launch's own RViz config predates Nav2; this one shows the
        # map, the costmaps, the planned path and AMCL's particle cloud. The
        # GroupAction is load-bearing: both launch files declare an argument
        # called "rviz", and without a scope the 'false' passed to the inner one
        # overwrites ours in the parent, so our RViz silently never starts.
        GroupAction([
            include('greenhouse.launch.py',
                    gui=LaunchConfiguration('gui'), rviz='false',
                    sensor_suite=LaunchConfiguration('sensor_suite')),
        ]),
        TimerAction(period=35.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            # Our own view, not nav2's stock one. Same map, costmaps, path
            # and particle cloud, plus the camera image, the points that
            # survived the height filter, and the scan synthesised from them,
            # so the whole depth pipeline is visible at once.
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'greenhouse_depth.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')))]),
        # Gazebo needs a moment to have a robot, a clock and a scan before the
        # localizer and the costmaps start asking for transforms.
        TimerAction(period=20.0, actions=[include('localization.launch.py')]),
        TimerAction(period=30.0, actions=[include('navigation.launch.py')]),
    ])
