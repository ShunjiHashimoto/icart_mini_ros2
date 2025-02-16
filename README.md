# icart_mini_ros2

<img src=docs/imgs/icart_mini.png width=50%>
<img src=docs/imgs/icart_urdf.png width=70%>

## Docker build
```bash
$ cd ~/icart_ws/src/icart_mini_ros2/docker
$ docker build -t icart_mini_ros2:latest .
```

## Docker Run
```bash
$ cd ~/icart_ws/src/icart_mini_ros2/docker
$ ./run.sh 
```

## ypspur Build
ローカルPCであらかじめビルドをしておく。  
コンテナ立ち上げ時に、entrypoint.shにてビルドされたライブラリのパスを通す。  

## joy node
joy_nodeを起動後、teleop_twist_joyを実行し、/joy->/cmd_velに変換する
```bash
$ ros2 run joy joy_node --ros-args --param device_id:=0
$ ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/icart_ws/icart_mini_ros2/icart_mini_ypspur_bridge/config/teleop_twist_joy_f710_params.yaml 
```