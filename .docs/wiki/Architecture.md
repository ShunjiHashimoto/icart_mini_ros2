# アーキテクチャ（Packages / Nodes / Topics）

本プロジェクトは ROS 2 Humble を前提に、以下の独立パッケージで構成されています。

## パッケージ一覧
- `icart_mini_bringup`: センサ、走行、ジョイスティック、ブリッジなどをまとめて起動
- `icart_mini_description`: URDF（xacro）と RViz 設定
- `icart_mini_ypspur_bridge`: 走行（YP-Spur）と ROS 2 の橋渡し
- `icart_mini_leg_tracker`: LiDAR からの脚クラスタ検出・追跡・Follow-me 制御

## 代表ノードと主トピック
- 走行ブリッジ（icart_mini_ypspur_bridge）
  - Sub: `/cmd_vel` (geometry_msgs/Twist)
  - Pub: `/odom` (nav_msgs/Odometry), `/joint_states` (sensor_msgs/JointState), TF `odom`→`base_footprint`
- 脚クラスタ追跡（leg_cluster_tracking_node）
  - Sub: `/scan` (sensor_msgs/LaserScan), `/joy` (sensor_msgs/Joy)
  - Pub: `/cmd_vel`, `/leg_tracker/cluster_markers`, `/leg_tracker/cluster_centers`, `/leg_tracker/person_marker`, `/leg_tracker/cluster_infos`, `/leg_tracker/is_lost_target`
- センサ（urg_node2）
  - Pub: `/scan`
- 可視化（rviz2, robot_state_publisher）
  - URDF の TF/可視化、RViz パネル

## Launch 構成（抜粋）
- `icart_mini_bringup/launch/icart_mini_bringup.launch.py`
  - urg_node2 起動
  - YP-Spur `ypspur-coordinator` 起動（デバイス/パラメータ指定）
  - ジョイスティック `joy_node` と `teleop_twist_joy` 起動
  - 走行ブリッジ `icart_mini_ypspur_bridge` を遅延起動（依存立上げ待ち）
- `icart_mini_description/launch/icart_mini_display.launch.py`
  - URDF 読み込みと RViz 表示

## TF 構成
- `odom` → `base_footprint` をブリッジから配信
- LiDAR フレーム（例: `laser`）はセンサ実装側に準拠

## コード参照（代表）
- ブリッジ: `icart_mini_ypspur_bridge/src/icart_mini_ypspur_bridge.cpp`
- 追跡: `icart_mini_leg_tracker/src/leg_cluster_tracking.cpp`
- URDF: `icart_mini_description/urdf/icart_mini.xacro`
- Bringup: `icart_mini_bringup/launch/icart_mini_bringup.launch.py`
