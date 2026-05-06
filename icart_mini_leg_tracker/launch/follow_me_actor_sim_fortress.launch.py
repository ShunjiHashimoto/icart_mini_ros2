import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    description_share = get_package_share_directory('icart_mini_description')
    actor_share = get_package_share_directory('gazebo_ros_actor_plugin')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gazebo_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    robot_xacro = os.path.join(description_share, 'urdf', 'icart_mini_fortress.xacro')
    rviz_config = os.path.join(description_share, 'rviz', 'follow_me_sim.rviz')
    bridge_config = os.path.join(description_share, 'config', 'fortress_bridge.yaml')
    actor_bridge_config = os.path.join(tracker_share, 'config', 'actor_bridge.yaml')
    debug_leg_model = os.path.join(
        description_share, 'models', 'inverted_pendulum_biped', 'leg.sdf'
    )
    hidden_leg_model = os.path.join(
        description_share, 'models', 'inverted_pendulum_biped', 'leg_hidden.sdf'
    )
    direction_marker_model = os.path.join(
        description_share, 'models', 'inverted_pendulum_biped', 'direction_marker.sdf'
    )
    actor_models_path = os.path.join(actor_share, 'config', 'skins')
    actor_xacro = os.path.join(actor_models_path, 'DoctorFemaleWalk', 'model.sdf.xacro')
    actor_plugin_path = os.path.join(actor_share, '..', '..', 'lib')

    world = LaunchConfiguration('world').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    proxy_visual = LaunchConfiguration('proxy_visual').perform(context).strip().lower()
    gz_args = f"-r {world}" if gui in ('1', 'true', 'yes', 'on') else f"-r -s {world}"
    pose_service = f'/world/{world_name}/set_pose'
    # hiddenはGUI上で透明にするだけで、gpu_lidar用のrendering geometryとcollisionは残す。
    leg_model = hidden_leg_model if proxy_visual == 'hidden' else debug_leg_model

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
    actor_pose_publish_rate = LaunchConfiguration('actor_pose_publish_rate')
    foot_pose_publish_rate = LaunchConfiguration('foot_pose_publish_rate')
    foot_z = LaunchConfiguration('foot_z')
    proxy_z = LaunchConfiguration('proxy_z')
    scan_target_mode = LaunchConfiguration('scan_target_mode')
    use_joy = LaunchConfiguration('use_joy')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    tracker_params_file = LaunchConfiguration('tracker_params_file')
    robot_description = Command(['xacro ', robot_xacro])
    actor_description = Command([
        'xacro ', actor_xacro,
        ' actor_name:=person_actor',
        ' follow_mode:=velocity',
        ' linear_velocity:=1.0',
        ' publish_pose:=true',
        ' pose_publish_rate:=', actor_pose_publish_rate,
        # ros_gz_sim create の -x/-y/-z は Actor plugin の TrajectoryPose に入らないため、
        # DAE由来の足poseと /person/actor_pose をworld座標へ戻すoffsetとして渡す。
        ' pose_offset_x:=', initial_x,
        ' pose_offset_y:=', initial_y,
        ' pose_offset_z:=', initial_z,
        ' pose_offset_roll:=0.0',
        ' pose_offset_pitch:=0.0',
        ' pose_offset_yaw:=0.0',
        ' foot_pose_publish_rate:=', foot_pose_publish_rate,
        ' foot_z:=', foot_z,
        ' sync_foot_proxies:=',
        PythonExpression(["'true' if '", scan_target_mode,
                          "' == 'actor_proxy' else 'false'"]),
        ' left_foot_proxy_name:=biped_left_leg',
        ' right_foot_proxy_name:=biped_right_leg',
        ' proxy_z:=', proxy_z,
    ])

    actor_mode_condition = IfCondition(PythonExpression([
        "'", scan_target_mode, "' == 'actor_proxy'",
    ]))
    proxy_mode_condition = IfCondition(PythonExpression([
        "'", scan_target_mode, "' == 'leg_proxy' or '",
        scan_target_mode, "' == 'actor_proxy'",
    ]))

    return [
        # DoctorFemaleWalk の model://config/skins/... 参照を Gazebo が解決できるようにする。
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=actor_share),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=actor_models_path),
        # Gazebo server process が Actor plugin の共有ライブラリを解決できるようにする。
        AppendEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=actor_plugin_path),
        AppendEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value=actor_plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': gz_args,
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='fortress_parameter_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        Node(
            condition=actor_mode_condition,
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='actor_parameter_bridge',
            parameters=[{'config_file': actor_bridge_config}],
            output='screen',
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='fortress_pose_service_bridge',
                    arguments=[
                        f'{pose_service}@ros_gz_interfaces/srv/SetEntityPose',
                    ],
                    output='screen',
                ),
            ],
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
            package='icart_mini_leg_tracker',
            executable='fortress_frame_relay.py',
            name='fortress_frame_relay',
            output='screen',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-name', 'icart_mini',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
            ],
            output='screen',
        ),

        Node(
            condition=actor_mode_condition,
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-string', actor_description,
                '-name', 'person_actor',
                '-x', initial_x,
                '-y', initial_y,
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            condition=proxy_mode_condition,
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-name', 'biped_left_leg',
                '-file', leg_model,
                '-x', initial_x,
                '-y', '0.11',
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            condition=proxy_mode_condition,
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-name', 'biped_right_leg',
                '-file', leg_model,
                '-x', initial_x,
                '-y', '-0.11',
                '-z', initial_z,
            ],
            output='screen',
        ),

        Node(
            condition=LaunchConfigurationEquals('scan_target_mode', 'leg_proxy'),
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-name', 'biped_direction_marker',
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
            # 追従ノード内の経過時間やmarker時刻をGazeboのsim timeに合わせる。
            parameters=[
                tracker_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            condition=LaunchConfigurationEquals('scan_target_mode', 'leg_proxy'),
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
                'pose_service': pose_service,
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
    ]


def generate_launch_description():
    tracker_share = get_package_share_directory('icart_mini_leg_tracker')
    description_share = get_package_share_directory('icart_mini_description')
    world_file = os.path.join(description_share, 'worlds', 'follow_me_empty_fortress.sdf')
    tracker_params_file = os.path.join(
        tracker_share, 'config', 'leg_cluster_tracking_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('world_name', default_value='follow_me_empty'),
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
        OpaqueFunction(function=launch_setup),
    ])
