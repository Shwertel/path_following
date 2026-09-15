import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'egrobots_line_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shwertel',
    maintainer_email='shwertel@todo.todo',
    description='Straight-line A-to-B navigation using only a 2D LiDAR and an IMU.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'line_navigator_node = egrobots_line_navigation.line_navigator_node:main',
            'cmd_prior_node = egrobots_line_navigation.cmd_prior_node:main',
            'pose_logger_node = egrobots_line_navigation.pose_logger_node:main',
            'row_localizer_node = egrobots_line_navigation.row_localizer_node:main',
            'spawn_obstacles = egrobots_line_navigation.spawn_obstacles:main',
            'imu_relay_node = egrobots_line_navigation.imu_relay_node:main',
            'manual_controller = egrobots_line_navigation.manual_controller:main',
        ],
    },
)
