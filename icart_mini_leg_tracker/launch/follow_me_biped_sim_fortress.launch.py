import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_share = get_package_share_directory('icart_mini_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gazebo_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    robot_xacro = os.path.join(description_share, 'urdf', 'icart_mini_fortress.xacro')
    rviz_config = os.path.join(description_share, 'rviz', 'follow_me_sim.rviz')
    bridge_config = os.path.join(description_share, 'config', 'fortress_bridge.yaml')
    leg_model = os.path.join(description_share, 'models', 'inverted_pendulum_biped', 'leg.sdf')
    direction_marker_model = os.path.join(
        description_share, 'models', 'inverted_pendulum_biped', 'direction_marker.sdf'
    )

    world = LaunchConfiguration('world').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    gz_args = f"-r {world}" if gui in ('1', 'true', 'yes', 'on') else f"-r -s {world}"
    pose_service = f'/world/{world_name}/set_pose'

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
    tracker_params_file = LaunchConfiguration('tracker_params_file')
    robot_description = Command(['xacro ', robot_xacro])

    return [
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
        DeclareLaunchArgument(
            'tracker_params_file',
            default_value=tracker_params_file,
            description='leg_cluster_tracking_node のしきい値を指定するYAMLファイル。',
        ),
        OpaqueFunction(function=launch_setup),
    ])
