# セットアップと起動（Setup / Quick Start）

## 動作要件（開発実績）
- ハードウェア: Raspberry Pi 5
- OS: Ubuntu 23.10
- ROS 2: Humble
- Docker: 26.0.0
- センサ: Hokuyo UST-10LX

## Docker でのビルド・起動
```bash
cd ~/icart_ws/src/icart_mini_ros2/docker
docker build -t icart_mini_ros2:latest .
./run.sh
```

## 依存パッケージのビルド
- YP-Spur（ロボット制御）
```bash
cd ~/icart_ws
mkdir build && cd build
cmake ../src/yp-spur
make
sudo make install
```
- urg_node2（Hokuyo LiDAR ドライバ）
```bash
colcon build --symlink-install --packages-select urg_node2
```
- 本レポジトリ内ブリッジ
```bash
colcon build --symlink-install --packages-select icart_mini_ypspur_bridge
```

## Bringup（まとめ起動）
```bash
# 端末1: センサ/走行/ジョイ等をまとめて起動
ros2 launch icart_mini_bringup icart_mini_bringup.launch.py
# 端末2: URDF+RViz 表示
ros2 launch icart_mini_description icart_mini_display.launch.py
```

## デバッグのコマンド例
```bash
# bag 記録（必要なトピックだけ）
ros2 bag record /leg_tracker/cluster_infos /scan /tf /tf_static \
  /leg_tracker/cluster_centers /leg_tracker/cluster_markers /leg_tracker/person_marker
# bag 再生（開始は一時停止）
ros2 bag play my_bag --rate 0.5 --topics /scan --start-pause
# CSV 出力
ros2 topic echo /leg_tracker/cluster_infos --csv > output.csv
```

## 設定ファイル
- teleop_twist_joy: `icart_mini_ypspur_bridge/config/teleop_twist_joy_f710_params.yaml`
- i-Cart Mini パラメータ: `i-Cart` リポジトリの `iCartMini2024.param`
