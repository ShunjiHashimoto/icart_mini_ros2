from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("urg_node2").find("urg_node2"),
                "launch",
                "urg_node2.launch.py"
            )
        )
    )

    return LaunchDescription([
        # icart_mini_description の launch
        urg_launch,

        # ypspur-coordinator の起動
        ExecuteProcess(
            cmd=["/root/icart_ws/build/ypspur-coordinator", 
                 "-d", "/dev/ttyACM0", 
                 "-p", "/root/icart_ws/src/i-Cart/iCartMini/iCartMini2024.param"],
            output="screen"
        ),

        # joy_node の起動
        ExecuteProcess(
            cmd=["ros2", "run", "joy", "joy_node", 
                 "--ros-args", "--param", "device_id:=0"],
            output="screen",
        ),

        # teleop_twist_joy の起動
        ExecuteProcess(
            cmd=["ros2", "run", "teleop_twist_joy", "teleop_node", 
                 "--ros-args", "--params-file", 
                 "/root/icart_ws/src/icart_mini_ros2/icart_mini_ypspur_bridge/config/teleop_twist_joy_f710_params.yaml"],
            output="screen"
        ),

        # LEDステータスモニタ
        Node(
            package="icart_mini_leg_tracker",
            executable="led_status.py",
            name="led_status_node",
            output="screen"
        ),

        # 3秒待って icart_mini_ypspur_bridge を起動
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "run", "icart_mini_ypspur_bridge", "icart_mini_ypspur_bridge"],
                    output="screen"
                )
            ]
        )
    ])
