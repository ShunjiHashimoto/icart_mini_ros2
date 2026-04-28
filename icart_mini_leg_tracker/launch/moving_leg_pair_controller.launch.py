from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path_mode = LaunchConfiguration('path_mode')
    auto_start = LaunchConfiguration('auto_start')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    straight_speed = LaunchConfiguration('straight_speed')
    straight_min_x = LaunchConfiguration('straight_min_x')
    straight_max_x = LaunchConfiguration('straight_max_x')

    return LaunchDescription([
        DeclareLaunchArgument('path_mode', default_value='straight'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.5'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('straight_speed', default_value='0.12'),
        DeclareLaunchArgument('straight_min_x', default_value='0.45'),
        DeclareLaunchArgument('straight_max_x', default_value='1.5'),

        Node(
            package='icart_mini_leg_tracker',
            executable='moving_leg_pair_controller.py',
            name='moving_leg_pair_controller',
            output='screen',
            parameters=[{
                'path_mode': path_mode,
                'auto_start': auto_start,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'straight_speed': straight_speed,
                'straight_min_x': straight_min_x,
                'straight_max_x': straight_max_x,
            }],
        ),
    ])
