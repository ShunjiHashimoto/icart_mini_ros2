# icart_mini_ros2
屋内外用の小型移動ロボットフレーム「[i-Cart mini](https://t-frog.com/products/icart_mini/)」のROS 2パッケージ
<img src=docs/imgs/icart_mini.png width=40%>  
<img src=docs/imgs/icart_urdf.png width=38%> <img src=docs/imgs/icart_rviz.png width=60%>

# System Requirements
- Hardware: Raspberry Pi 5
- OS: Ubuntu23.10
- ROS 2: humble
- Sensor: Hokuyo [UST-10LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16&utm_source=google&utm_medium=cpc&utm_campaign=[P-MAX]&gad_source=1&gclid=Cj0KCQiAwtu9BhC8ARIsAI9JHam6cR3BVtNZ746VwLahng9sImtlVbThGx0BkbivMfSW7eK9brOBjaYaAjHhEALw_wcB#spec)
- Battery: LONG 鉛蓄電池12V x2
# Setup
### Docker Build & Run
```bash
$ cd ~/icart_ws/src/icart_mini_ros2/docker
$ docker build -t icart_mini_ros2:latest .
$ ./run.sh 
```

# Quick Start
### Bringup iCart-Mini
```bash
# terminal 1
$ ros2 launch icart_mini_bringup icart_mini_bringup.launch.py 
# terminal 2
$ ros2 launch icart_mini_description icart_mini_display.launch.py
```

# Required Packages
### YP-Spur(YamabicoProject-Spur) 
[Yp-Spur](https://github.com/openspur/yp-spur)は、移動ロボット用のオープンソースソフトウェアで、ロボットのモーションコントロールをシンプルかつ効率的に行うためのツールです。ビルド方法はYp-SpurのGitHubページを参照ください。      

### urg_node2
[urg_node2](https://github.com/ShunjiHashimoto/urg_node2.git)は、Hokuyo製LiDARセンサーのデータをROS 2トピックとして配信するためのパッケージ

### i-Cart
[i-Cart](https://github.com/BND-tc/i-Cart)は、i-Cartシリーズのモデルデータおよびパラメータファイル
