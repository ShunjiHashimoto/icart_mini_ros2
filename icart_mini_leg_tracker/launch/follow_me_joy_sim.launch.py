from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    follow_launch = os.path.join(tracker_share, 'launch', 'follow_me_sim.launch.py')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    path_mode = LaunchConfiguration('path_mode')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('path_mode', default_value='straight'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_launch),
            launch_arguments={
                'gui': gui,
                'use_rviz': use_rviz,
                'path_mode': path_mode,
                'use_joy': 'true',
                'joy_device_id': joy_device_id,
                'joy_device_name': joy_device_name,
                'follow_auto_start': 'false',
                'person_auto_start': 'false',
            }.items(),
        ),
    ])
