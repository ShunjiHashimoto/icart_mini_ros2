import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    follow_launch = os.path.join(tracker_share, 'launch', 'follow_me_biped_sim.launch.py')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    use_joy = LaunchConfiguration('use_joy')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')
    joy_deadzone = LaunchConfiguration('joy_deadzone')

    description_share = get_package_share_directory('icart_mini_description')
    obstacle_world = os.path.join(description_share, 'worlds', 'follow_me_obstacles.world')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value=obstacle_world),
        DeclareLaunchArgument('use_joy', default_value='true'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_launch),
            launch_arguments={
                'world': world,
                'gui': gui,
                'use_rviz': use_rviz,
                'use_joy': use_joy,
                'joy_device_id': joy_device_id,
                'joy_device_name': joy_device_name,
                'joy_deadzone': joy_deadzone,
            }.items(),
        ),
    ])
