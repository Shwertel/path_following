"""Bring up Gazebo, the rover, its controllers, and state estimation.

The state-estimation chain here is deliberately different from earlier tasks:
no wheel odometry anywhere. cmd_prior_node turns the velocity we commanded into
a motion prior, and the EKF fuses that with IMU heading to publish
odom -> base_link. The drive controller's own odometry TF is switched off in
config/ros2_controllers.yaml.

In the road world, row_localizer_node adds a third input: sideways position and
heading measured against the parked-car rows, which bounds the drift that the
other two inputs cannot observe. The same node does the same job for greenhouse
crop rows, with the search windows and thresholds of config/row_greenhouse.yaml.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('egrobots_line_navigation')
    xacro_path = os.path.join(pkg_share, 'urdf', 'egrobots_rover.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'egrobots_rover.rviz')

    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])
    sensor_suite = LaunchConfiguration('sensor_suite')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' sensor_suite:=', sensor_suite]), value_type=str)

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'gui': use_gui}.items())

    return LaunchDescription([
        # FastDDS with shared memory turned off, UDP only.
        #
        # The shared-memory transport leaves robust mutexes in /dev/shm owned by
        # the process that created them. Kill that process hard - which is what
        # tearing this stack down does - and the next process to touch the
        # segment aborts on
        #   pthread_mutex_lock.c: Assertion `e != ESRCH || !robust' failed
        # which is how rviz kept dying. The same stale state wedged nav2's
        # lifecycle_manager part way through bringup, forever, with no timeout.
        #
        # UDP over loopback costs a little throughput on the depth image; the
        # profile enlarges the socket buffers to cover it.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(pkg_share, 'config', 'fastdds_udp.xml')),

        DeclareLaunchArgument('rviz', default_value='true'),
        # Which sensor the rover carries. 'depth' is the front RGB-D camera,
        # 'lidar' the original 360 deg planar scanner, 'both' for comparison.
        # Everything downstream consumes /scan either way: with 'depth' that
        # scan is synthesised by depth_to_scan_node.
        DeclareLaunchArgument('sensor_suite', default_value='depth'),
        # Where the synthesised scan goes. Point it elsewhere when running
        # sensor_suite:=both, so the LiDAR keeps /scan and the two can be
        # compared rather than collide.
        DeclareLaunchArgument('depth_scan_topic', default_value='/scan'),
        # gui:=false runs Gazebo headless - no gzclient, no OpenGL. Useful when
        # the graphics stack is unavailable; the LiDAR is a CPU ray sensor, so
        # physics and sensing are unaffected.
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='egrobots_world.world',
                              description='World file in the package worlds/ folder, '
                                          'e.g. road_world.world'),
        DeclareLaunchArgument('row_correction', default_value='true',
                              description='Fuse the row measurement into the EKF'),
        DeclareLaunchArgument('row_params', default_value='row_road.yaml',
                              description='Row localizer profile in config/, '
                                          'e.g. row_greenhouse.yaml'),
        DeclareLaunchArgument('ekf_params', default_value='ekf.yaml',
                              description='EKF profile in config/, '
                                          'e.g. ekf_greenhouse.yaml'),
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
        # With the depth camera there is no scanner on the robot, so the scan
        # the rest of the stack consumes has to be made. This flattens a
        # height band of the depth cloud into a LaserScan; see the node for
        # why the band, and not the whole cloud.
        Node(package='egrobots_line_navigation', executable='depth_to_scan_node',
             name='depth_to_scan_node', output='screen',
             condition=IfCondition(
                 PythonExpression(["'", sensor_suite, "' != 'lidar'"])),
             parameters=[os.path.join(pkg_share, 'config', 'depth_to_scan.yaml'),
                         {'use_sim_time': True,
                          'scan_topic': LaunchConfiguration('depth_scan_topic')}]),
        # Commanded velocity -> motion prior for the EKF (no wheel feedback).
        Node(package='egrobots_line_navigation', executable='cmd_prior_node',
             name='cmd_prior_node', output='screen',
             parameters=[{'use_sim_time': True}]),
        # Gazebo's IMU ships zero covariance, which diverges the filter when no
        # position observation exists to anchor it. This supplies realistic values.
        Node(package='egrobots_line_navigation', executable='imu_relay_node',
             name='imu_relay_node', output='screen',
             parameters=[{'use_sim_time': True}]),
        # Sideways position and heading from the car rows. In a world without
        # rows it simply never anchors and sends the EKF nothing.
        Node(package='egrobots_line_navigation', executable='row_localizer_node',
             name='row_localizer_node', output='screen',
             parameters=[PathJoinSubstitution([pkg_share, 'config',
                                               LaunchConfiguration('row_params')]),
                         {'use_sim_time': True,
                          'publish_measurement': ParameterValue(
                              LaunchConfiguration('row_correction'), value_type=bool)}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([pkg_share, 'config',
                                               LaunchConfiguration('ekf_params')]),
                         {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
             condition=IfCondition(use_rviz), output='screen'),
    ])
