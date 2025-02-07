# icart_mini_ros2
## Docker build
```bash
$~/icart_ws/src/icart_mini_ros2/docker$ docker build -t icart_mini_ros2:latest .
```

## Docker Run
```bash
$~/icart_ws/src/icart_mini_ros2/docker# ./run.sh 
```

## ypspur Build
```bash
$ cd icart_ws/build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```
ldconfig は、共有ライブラリのキャッシュを更新し、ライブラリのリンクを管理するコマンド です。

## joy node
joy_nodeを起動後、teleop_twist_joyを実行し、/joy->/cmd_velに変換する
```bash
$ros2 run joy joy_node --ros-args --param device_id:=0
$ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/icart_ws/icart_mini_ros2/icart_mini_ypspur_bridge/teleop_twist_joy_f710_params.yaml 
```