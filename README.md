# icart_mini_ros2

<img src=docs/imgs/icart_mini.png width=50%>
<img src=docs/imgs/icart_urdf.png width=70%>

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
## Bringup iCartMini
```bash
# terminal 1
$ ros2 launch icart_mini_bringup icart_mini_bringup.launch.py 
# terminal 2
$ ros2 launch icart_mini_description icart_mini_display.launch.py
```