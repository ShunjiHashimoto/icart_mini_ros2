import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SCENARIOS = {
    'baseline_same_motion_no_pillars': {
        'world_file': 'follow_me_empty_fortress.sdf',
        'world_name': 'follow_me_empty',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'straight',
        'motion_linear_speed': '0.25',
        'motion_duration': '10.0',
        'motion_turn_angular_speed': '0.35',
    },
    'leg_like_pillars_corridor': {
        'world_file': 'follow_me_leg_like_pillars_corridor_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_corridor',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'straight',
        'motion_linear_speed': '0.25',
        'motion_duration': '10.0',
        'motion_turn_angular_speed': '0.35',
    },
    'leg_like_pillars_offset_corridor': {
        'world_file': 'follow_me_leg_like_pillars_offset_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_offset',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'straight',
        'motion_linear_speed': '0.25',
        'motion_duration': '10.0',
        'motion_turn_angular_speed': '0.35',
    },
    'obstacle_orbit': {
        'world_file': 'follow_me_orbit_obstacle_fortress.sdf',
        'world_name': 'follow_me_orbit_obstacle',
        'initial_x': '0.9',
        'initial_y': '0.05',
        'motion_scenario': 'orbit_left',
        'motion_linear_speed': '0.25',
        'motion_duration': '18.0',
        'motion_turn_angular_speed': '0.35',
    },
    'wide_corridor_orbit': {
        'world_file': 'follow_me_wide_corridor_fortress.sdf',
        'world_name': 'follow_me_wide_corridor',
        'initial_x': '0.9',
        'initial_y': '0.05',
        'motion_scenario': 'left_orbit_straight_right_orbit_goal_straight',
        'motion_linear_speed': '0.25',
        'motion_duration': '18.0',
        'motion_turn_angular_speed': '0.35',
    },
    'diagonal_walk': {
        'world_file': 'follow_me_empty_fortress.sdf',
        'world_name': 'follow_me_empty',
        'initial_x': '0.9',
        'initial_y': '-0.15',
        'motion_scenario': 'diagonal_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '9.0',
        'motion_turn_angular_speed': '0.18',
    },
    'front_crossing': {
        'world_file': 'follow_me_empty_fortress.sdf',
        'world_name': 'follow_me_empty',
        'initial_x': '0.65',
        'initial_y': '-0.35',
        'motion_scenario': 'front_crossing',
        'motion_linear_speed': '0.25',
        'motion_duration': '6.0',
        'motion_turn_angular_speed': '0.32',
    },
    'turning_forward_walk': {
        'world_file': 'follow_me_empty_fortress.sdf',
        'world_name': 'follow_me_empty',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'turning_forward_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '16.0',
        'motion_turn_angular_speed': '0.25',
    },
    'diagonal_walk_pillars_corridor': {
        'world_file': 'follow_me_leg_like_pillars_corridor_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_corridor',
        'initial_x': '0.9',
        'initial_y': '-0.15',
        'motion_scenario': 'diagonal_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '9.0',
        'motion_turn_angular_speed': '0.18',
    },
    'front_crossing_pillars_corridor': {
        'world_file': 'follow_me_leg_like_pillars_corridor_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_corridor',
        'initial_x': '0.65',
        'initial_y': '-0.35',
        'motion_scenario': 'front_crossing',
        'motion_linear_speed': '0.25',
        'motion_duration': '6.0',
        'motion_turn_angular_speed': '0.32',
    },
    'turning_forward_walk_pillars_corridor': {
        'world_file': 'follow_me_leg_like_pillars_corridor_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_corridor',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'turning_forward_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '16.0',
        'motion_turn_angular_speed': '0.25',
    },
    'diagonal_walk_pillars_offset_corridor': {
        'world_file': 'follow_me_leg_like_pillars_offset_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_offset',
        'initial_x': '0.9',
        'initial_y': '-0.15',
        'motion_scenario': 'diagonal_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '9.0',
        'motion_turn_angular_speed': '0.18',
    },
    'front_crossing_pillars_offset_corridor': {
        'world_file': 'follow_me_leg_like_pillars_offset_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_offset',
        'initial_x': '0.65',
        'initial_y': '-0.35',
        'motion_scenario': 'front_crossing',
        'motion_linear_speed': '0.25',
        'motion_duration': '6.0',
        'motion_turn_angular_speed': '0.32',
    },
    'turning_forward_walk_pillars_offset_corridor': {
        'world_file': 'follow_me_leg_like_pillars_offset_fortress.sdf',
        'world_name': 'follow_me_leg_like_pillars_offset',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'turning_forward_walk',
        'motion_linear_speed': '0.25',
        'motion_duration': '16.0',
        'motion_turn_angular_speed': '0.25',
    },
    'dense_mixed_obstacles_slalom': {
        'world_file': 'follow_me_dense_mixed_obstacles_fortress.sdf',
        'world_name': 'follow_me_dense_mixed_obstacles',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'gentle_slalom',
        'motion_linear_speed': '0.25',
        'motion_duration': '14.0',
        'motion_turn_angular_speed': '0.18',
    },
    'front_crossing_box_gate': {
        'world_file': 'follow_me_front_crossing_box_gate_fortress.sdf',
        'world_name': 'follow_me_front_crossing_box_gate',
        'initial_x': '0.65',
        'initial_y': '-0.35',
        'motion_scenario': 'front_crossing',
        'motion_linear_speed': '0.25',
        'motion_duration': '10.0',
        'motion_turn_angular_speed': '0.28',
    },
    'rectangular_panel_near_pass': {
        'world_file': 'follow_me_rectangular_panels_fortress.sdf',
        'world_name': 'follow_me_rectangular_panels',
        'initial_x': '0.9',
        'initial_y': '0.0',
        'motion_scenario': 'straight',
        'motion_linear_speed': '0.25',
        'motion_duration': '10.0',
        'motion_turn_angular_speed': '0.35',
    },
}


def launch_setup(context, *args, **kwargs):
    scenario_name = LaunchConfiguration('scenario').perform(context)
    scenario = SCENARIOS[scenario_name]

    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    description_share = get_package_share_directory('icart_mini_description')
    follow_launch = os.path.join(
        tracker_share, 'launch', 'follow_me_actor_sim_fortress.launch.py'
    )
    world = os.path.join(description_share, 'worlds', scenario['world_file'])
    # motion_*:=auto の場合は、scenarioごとの既定値を使う。
    motion_scenario = LaunchConfiguration('motion_scenario').perform(context)
    motion_linear_speed = LaunchConfiguration('motion_linear_speed').perform(context)
    motion_duration = LaunchConfiguration('motion_duration').perform(context)
    motion_turn_angular_speed = LaunchConfiguration('motion_turn_angular_speed').perform(context)
    motion_start_delay = float(LaunchConfiguration('motion_start_delay').perform(context))
    motion_publish_rate = float(LaunchConfiguration('motion_publish_rate').perform(context))
    if motion_scenario == 'auto':
        motion_scenario = scenario['motion_scenario']
    if motion_linear_speed == 'auto':
        motion_linear_speed = scenario['motion_linear_speed']
    if motion_duration == 'auto':
        motion_duration = scenario['motion_duration']
    if motion_turn_angular_speed == 'auto':
        motion_turn_angular_speed = scenario['motion_turn_angular_speed']
    motion_linear_speed = float(motion_linear_speed)
    motion_duration = float(motion_duration)
    motion_turn_angular_speed = float(motion_turn_angular_speed)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_launch),
            launch_arguments={
                'world': world,
                'world_name': scenario['world_name'],
                'gui': LaunchConfiguration('gui'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_mode': 'manual',
                'initial_x': scenario['initial_x'],
                'initial_y': scenario['initial_y'],
                'initial_z': '0.0',
                'actor_pose_publish_rate': LaunchConfiguration('actor_pose_publish_rate'),
                'foot_pose_publish_rate': LaunchConfiguration('foot_pose_publish_rate'),
                'foot_z': '0.0',
                'proxy_z': '0.0',
                'proxy_visual': LaunchConfiguration('proxy_visual'),
                'scan_target_mode': LaunchConfiguration('scan_target_mode'),
                'use_joy': LaunchConfiguration('use_joy'),
                'joy_device_id': LaunchConfiguration('joy_device_id'),
                'joy_device_name': LaunchConfiguration('joy_device_name'),
                'joy_deadzone': LaunchConfiguration('joy_deadzone'),
                'tracker_params_file': LaunchConfiguration('tracker_params_file'),
            }.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('run_actor_motion')),
            package='icart_mini_leg_tracker',
            executable='actor_debug_motion_runner.py',
            name='actor_debug_motion_runner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'motion_scenario': motion_scenario,
                'start_delay': motion_start_delay,
                'linear_speed': motion_linear_speed,
                'duration': motion_duration,
                'publish_rate': motion_publish_rate,
                'turn_angular_speed': motion_turn_angular_speed,
                'stop_when_done': True,
            }],
        ),
    ]


def generate_launch_description():
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    tracker_params_file = os.path.join(
        tracker_share, 'config', 'leg_cluster_tracking_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='baseline_same_motion_no_pillars',
            choices=list(SCENARIOS.keys()),
            description='人物追従デバッグ用の再現シナリオ。',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_device_name', default_value=''),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),
        DeclareLaunchArgument(
            'tracker_params_file',
            default_value=tracker_params_file,
            description='leg_cluster_tracking_node のしきい値を指定するYAMLファイル。',
        ),
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
        DeclareLaunchArgument(
            'run_actor_motion',
            default_value='true',
            choices=['true', 'false'],
            description='trueなら通常速度の人物移動を自動publishする。',
        ),
        DeclareLaunchArgument(
            'motion_scenario',
            default_value='auto',
            choices=[
                'auto',
                'straight',
                'stop_restart_turn_left',
                'orbit_left',
                'left_orbit_straight_right_orbit',
                'left_orbit_straight_right_orbit_goal_straight',
                'diagonal_walk',
                'front_crossing',
                'turning_forward_walk',
                'gentle_slalom',
            ],
            description='Actorへpublishする /person/cmd_vel の再現パターン。autoならscenarioごとの既定値を使う。',
        ),
        DeclareLaunchArgument('motion_start_delay', default_value='6.0'),
        DeclareLaunchArgument('motion_linear_speed', default_value='auto'),
        DeclareLaunchArgument('motion_duration', default_value='auto'),
        DeclareLaunchArgument('motion_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('motion_turn_angular_speed', default_value='auto'),
        OpaqueFunction(function=launch_setup),
    ])
