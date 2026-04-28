import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('icart_mini_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'icart_mini_gazebo.xacro')
    leg_pair_xacro_file = os.path.join(pkg_share, 'models', 'leg_pair', 'leg_pair.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'follow_me_empty.world')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'follow_me_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    spawn_leg_pair = LaunchConfiguration('spawn_leg_pair')
    leg_pair_x = LaunchConfiguration('leg_pair_x')
    leg_pair_y = LaunchConfiguration('leg_pair_y')
    leg_pair_z = LaunchConfiguration('leg_pair_z')
    leg_radius = LaunchConfiguration('leg_radius')
    leg_height = LaunchConfiguration('leg_height')
    leg_separation = LaunchConfiguration('leg_separation')

    robot_description = Command(['xacro ', xacro_file])
    leg_pair_description = Command([
        'xacro ', leg_pair_xacro_file,
        ' leg_radius:=', leg_radius,
        ' leg_height:=', leg_height,
        ' leg_separation:=', leg_separation,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the icart_mini configuration.'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo client GUI.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Gazebo world file.'
        ),
        DeclareLaunchArgument(
            'spawn_leg_pair',
            default_value='true',
            description='Spawn the two-cylinder leg-pair target model.'
        ),
        DeclareLaunchArgument(
            'leg_pair_x',
            default_value='0.5',
            description='Initial leg-pair model x position.'
        ),
        DeclareLaunchArgument(
            'leg_pair_y',
            default_value='0.0',
            description='Initial leg-pair model y position.'
        ),
        DeclareLaunchArgument(
            'leg_pair_z',
            default_value='0.0',
            description='Initial leg-pair model z position.'
        ),
        DeclareLaunchArgument(
            'leg_radius',
            default_value='0.045',
            description='Leg cylinder radius.'
        ),
        DeclareLaunchArgument(
            'leg_height',
            default_value='0.7',
            description='Leg cylinder height.'
        ),
        DeclareLaunchArgument(
            'leg_separation',
            default_value='0.22',
            description='Distance between the two leg cylinder centers.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
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
            condition=IfCondition(spawn_leg_pair),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='leg_pair_state_publisher',
            parameters=[{
                'robot_description': leg_pair_description,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('robot_description', 'leg_pair_description'),
            ],
            output='screen',
        ),

        Node(
            condition=IfCondition(spawn_leg_pair),
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'leg_pair',
                '-topic', 'leg_pair_description',
                '-x', leg_pair_x,
                '-y', leg_pair_y,
                '-z', leg_pair_z,
            ],
            output='screen',
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
