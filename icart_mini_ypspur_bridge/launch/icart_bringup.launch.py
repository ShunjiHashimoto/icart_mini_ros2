from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])