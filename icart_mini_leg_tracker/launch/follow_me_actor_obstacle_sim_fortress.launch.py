import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    follow_launch = os.path.join(tracker_share, 'launch', 'follow_me_actor_sim_fortress.launch.py')
    tracker_params_file = os.path.join(
        tracker_share, 'config', 'leg_cluster_tracking_params.yaml'
    )

    description_share = get_package_share_directory('icart_mini_description')
    obstacle_world = os.path.join(description_share, 'worlds', 'follow_me_obstacles_fortress.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=obstacle_world),
        DeclareLaunchArgument('world_name', default_value='follow_me_obstacles'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_mode', default_value='manual'),
        DeclareLaunchArgument('initial_x', default_value='0.9'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('straight_speed', default_value='0.12'),
        DeclareLaunchArgument('straight_max_x', default_value='1.5'),
        DeclareLaunchArgument('update_rate', default_value='60.0'),
        DeclareLaunchArgument('step_length', default_value='0.24'),
        DeclareLaunchArgument('step_width', default_value='0.22'),
        DeclareLaunchArgument('step_frequency', default_value='1.2'),
        DeclareLaunchArgument(
            'actor_pose_publish_rate',
            default_value='30.0',
            description='Actor pose publish rate.',
        ),
        DeclareLaunchArgument(
            'foot_pose_publish_rate',
            default_value='30.0',
            description='DAE-derived actor foot pose publish rate.',
        ),
        DeclareLaunchArgument('foot_z', default_value='0.0'),
        DeclareLaunchArgument('proxy_z', default_value='0.0'),
        DeclareLaunchArgument(
            'proxy_visual',
            default_value='hidden',
            choices=['debug', 'hidden'],
            description='脚プロキシの表示: debug は円柱visualあり、hidden は透明visualとcollisionを残す。',
        ),
        DeclareLaunchArgument(
            'scan_target_mode',
            default_value='actor_proxy',
            choices=['actor_proxy', 'leg_proxy'],
            description='LiDAR検出対象: actor_proxy はActorと同期脚プロキシ、leg_proxy は左右脚プロキシのみ。',
        ),
        DeclareLaunchArgument('use_joy', default_value='true'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),
        DeclareLaunchArgument(
            'tracker_params_file',
            default_value=tracker_params_file,
            description='leg_cluster_tracking_node のしきい値を指定するYAMLファイル。',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'world_name': LaunchConfiguration('world_name'),
                'gui': LaunchConfiguration('gui'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_mode': LaunchConfiguration('path_mode'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'straight_speed': LaunchConfiguration('straight_speed'),
                'straight_max_x': LaunchConfiguration('straight_max_x'),
                'update_rate': LaunchConfiguration('update_rate'),
                'step_length': LaunchConfiguration('step_length'),
                'step_width': LaunchConfiguration('step_width'),
                'step_frequency': LaunchConfiguration('step_frequency'),
                'actor_pose_publish_rate': LaunchConfiguration('actor_pose_publish_rate'),
                'foot_pose_publish_rate': LaunchConfiguration('foot_pose_publish_rate'),
                'foot_z': LaunchConfiguration('foot_z'),
                'proxy_z': LaunchConfiguration('proxy_z'),
                'proxy_visual': LaunchConfiguration('proxy_visual'),
                'scan_target_mode': LaunchConfiguration('scan_target_mode'),
                'use_joy': LaunchConfiguration('use_joy'),
                'joy_device_id': LaunchConfiguration('joy_device_id'),
                'joy_device_name': LaunchConfiguration('joy_device_name'),
                'joy_deadzone': LaunchConfiguration('joy_deadzone'),
                'tracker_params_file': LaunchConfiguration('tracker_params_file'),
            }.items(),
        ),
    ])
