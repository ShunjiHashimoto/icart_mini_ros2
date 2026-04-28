from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    description_share = get_package_share_directory('icart_mini_description')
    gazebo_launch = os.path.join(description_share, 'launch', 'icart_mini_gazebo.launch.py')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    path_mode = LaunchConfiguration('path_mode')
    follow_auto_start = LaunchConfiguration('follow_auto_start')
    person_auto_start = LaunchConfiguration('person_auto_start')
    follow_start_delay = LaunchConfiguration('follow_start_delay')
    person_start_delay = LaunchConfiguration('person_start_delay')
    leg_pair_x = LaunchConfiguration('leg_pair_x')
    leg_pair_y = LaunchConfiguration('leg_pair_y')
    straight_speed = LaunchConfiguration('straight_speed')
    straight_min_x = LaunchConfiguration('straight_min_x')
    straight_max_x = LaunchConfiguration('straight_max_x')
    use_joy = LaunchConfiguration('use_joy')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')
    joy_deadzone = LaunchConfiguration('joy_deadzone')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('path_mode', default_value='straight'),
        DeclareLaunchArgument('follow_auto_start', default_value='true'),
        DeclareLaunchArgument('person_auto_start', default_value='true'),
        DeclareLaunchArgument('follow_start_delay', default_value='4.0'),
        DeclareLaunchArgument('person_start_delay', default_value='6.0'),
        DeclareLaunchArgument('leg_pair_x', default_value='0.5'),
        DeclareLaunchArgument('leg_pair_y', default_value='0.0'),
        DeclareLaunchArgument('straight_speed', default_value='0.12'),
        DeclareLaunchArgument('straight_min_x', default_value='0.45'),
        DeclareLaunchArgument('straight_max_x', default_value='1.5'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gui': gui,
                'use_rviz': use_rviz,
                'spawn_leg_pair': 'true',
                'leg_pair_x': leg_pair_x,
                'leg_pair_y': leg_pair_y,
            }.items(),
        ),

        Node(
            package='icart_mini_leg_tracker',
            executable='leg_cluster_tracking_node',
            name='leg_cluster_tracking_node',
            output='screen',
        ),

        Node(
            package='icart_mini_leg_tracker',
            executable='moving_leg_pair_controller.py',
            name='moving_leg_pair_controller',
            output='screen',
            parameters=[{
                'path_mode': path_mode,
                'auto_start': False,
                'initial_x': leg_pair_x,
                'initial_y': leg_pair_y,
                'straight_speed': straight_speed,
                'straight_min_x': straight_min_x,
                'straight_max_x': straight_max_x,
            }],
        ),

        Node(
            condition=IfCondition(use_joy),
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': joy_device_id,
                'device_name': joy_device_name,
                'deadzone': joy_deadzone,
            }],
        ),

        Node(
            condition=IfCondition(use_joy),
            package='icart_mini_leg_tracker',
            executable='joystick_follow_me_teleop.py',
            name='joystick_follow_me_teleop',
            output='screen',
        ),

        TimerAction(
            period=follow_start_delay,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(follow_auto_start),
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/follow_me/control',
                        'std_msgs/msg/String',
                        '{data: start}',
                    ],
                    output='screen',
                ),
            ],
        ),

        TimerAction(
            period=person_start_delay,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(person_auto_start),
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/person/control',
                        'std_msgs/msg/String',
                        '{data: start}',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
