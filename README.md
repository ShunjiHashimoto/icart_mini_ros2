# icart_mini_ros2
屋内外用の小型移動ロボットフレーム「i-Cart mini」のROS 2パッケージ  
<img src=docs/imgs/icart_mini.png width=50%>  
<img src=docs/imgs/icart_urdf.png width=38%> <img src=docs/imgs/icart_rviz.png width=60%>

# Setup
## Docker Build & Run
```bash
$ cd ~/icart_ws/src/icart_mini_ros2/docker
$ docker build -t icart_mini_ros2:latest .
$ ./run.sh 
```

## ypspur Build
ローカルPCであらかじめビルドをしておく。  
コンテナ立ち上げ時に、entrypoint.shにてビルドされたライブラリのパスを通す。  

# Quick Start
## Bringup iCart-Mini
```bash
# terminal 1
$ ros2 launch icart_mini_bringup icart_mini_bringup.launch.py 
# terminal 2
$ ros2 launch icart_mini_description icart_mini_display.launch.py
```