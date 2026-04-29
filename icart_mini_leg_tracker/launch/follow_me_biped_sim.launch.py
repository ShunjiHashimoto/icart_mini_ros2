import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('icart_mini_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
    robot_xacro = os.path.join(description_share, 'urdf', 'icart_mini_gazebo.xacro')
    rviz_config = os.path.join(description_share, 'rviz', 'follow_me_sim.rviz')
    world_file = os.path.join(description_share, 'worlds', 'follow_me_empty.world')
    leg_model = os.path.join(description_share, 'models', 'inverted_pendulum_biped', 'leg.sdf')
    direction_marker_model = os.path.join(
        description_share, 'models', 'inverted_pendulum_biped', 'direction_marker.sdf'
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    path_mode = LaunchConfiguration('path_mode')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    straight_speed = LaunchConfiguration('straight_speed')
    straight_max_x = LaunchConfiguration('straight_max_x')
    update_rate = LaunchConfiguration('update_rate')
    step_length = LaunchConfiguration('step_length')
    step_width = LaunchConfiguration('step_width')
    step_frequency = LaunchConfiguration('step_frequency')
    use_joy = LaunchConfiguration('use_joy')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    robot_description = Command(['xacro ', robot_xacro])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_mode', default_value='manual'),
        DeclareLaunchArgument('initial_x', default_value='0.5'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('straight_speed', default_value='0.12'),
        DeclareLaunchArgument('straight_max_x', default_value='1.5'),
        DeclareLaunchArgument('update_rate', default_value='60.0'),
        DeclareLaunchArgument('step_length', default_value='0.24'),
        DeclareLaunchArgument('step_width', default_value='0.22'),
        DeclareLaunchArgument('step_frequency', default_value='1.2'),
        DeclareLaunchArgument('use_joy', default_value='true'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world,
                'gui': gui,
                'verbose': 'true',
            }.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'icart_mini',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
            ],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'biped_left_leg',
                '-file', leg_model,
                '-x', initial_x,
                '-y', '0.11',
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'biped_right_leg',
                '-file', leg_model,
                '-x', initial_x,
                '-y', '-0.11',
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'biped_direction_marker',
                '-file', direction_marker_model,
                '-x', initial_x,
                '-y', initial_y,
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            package='icart_mini_leg_tracker',
            executable='leg_cluster_tracking_node',
            name='leg_cluster_tracking_node',
            output='screen',
        ),

        Node(
            package='icart_mini_leg_tracker',
            executable='inverted_pendulum_biped_controller.py',
            name='inverted_pendulum_biped_controller',
            output='screen',
            parameters=[{
                'auto_start': False,
                'direction_marker_name': 'biped_direction_marker',
                'path_mode': path_mode,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_z': initial_z,
                'straight_speed': straight_speed,
                'straight_max_x': straight_max_x,
                'update_rate': update_rate,
                'step_length': step_length,
                'step_width': step_width,
                'step_frequency': step_frequency,
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

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
