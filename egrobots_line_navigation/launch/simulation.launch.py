"""Bring up Gazebo, the rover, its controllers, and state estimation.

The state-estimation chain here is deliberately different from earlier tasks:
no wheel odometry anywhere. cmd_prior_node turns the velocity we commanded into
a motion prior, and the EKF fuses that with IMU heading to publish
odom -> base_link. The drive controller's own odometry TF is switched off in
config/ros2_controllers.yaml.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')
    xacro_path = os.path.join(pkg_share, 'urdf', 'egrobots_rover.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'egrobots_world.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'egrobots_rover.rviz')

    use_rviz = LaunchConfiguration('rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo_launch,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description},
                         {'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description',
                        '-entity', 'egrobots_rover', '-z', '0.2'],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller'], output='screen'),
        # Commanded velocity -> motion prior for the EKF (no wheel feedback).
        Node(package='egrobots_line_navigation', executable='cmd_prior_node',
             name='cmd_prior_node', output='screen',
             parameters=[{'use_sim_time': True}]),
        # Gazebo's IMU ships zero covariance, which diverges the filter when no
        # position observation exists to anchor it. This supplies realistic values.
        Node(package='egrobots_line_navigation', executable='imu_relay_node',
             name='imu_relay_node', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'),
                         {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
             condition=IfCondition(use_rviz), output='screen'),
    ])
