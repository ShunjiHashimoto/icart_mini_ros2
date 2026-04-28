from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Publish /follow_me/control start after launching the tracker.'
        ),

        Node(
            package='icart_mini_leg_tracker',
            executable='leg_cluster_tracking_node',
            name='leg_cluster_tracking_node',
            output='screen',
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(auto_start),
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
    ])
