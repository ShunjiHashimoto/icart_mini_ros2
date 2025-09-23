# アーキテクチャ（Packages / Nodes / Topics）

本プロジェクトは ROS 2 Humble を前提に、以下の独立パッケージで構成されています。

## パッケージ一覧
- : センサ、走行、ジョイスティック、ブリッジなどをまとめて起動
- : URDF（xacro）と RViz 設定
- : 走行（YP-Spur）と ROS 2 の橋渡し
- : LiDAR からの脚クラスタ検出・追跡・Follow-me 制御

## 代表ノードと主トピック
- 走行ブリッジ（icart_mini_ypspur_bridge）
  - Sub:  (geometry_msgs/Twist)
  - Pub:  (nav_msgs/Odometry),  (sensor_msgs/JointState), TF →
- 脚クラスタ追跡（leg_cluster_tracking_node）
  - Sub:  (sensor_msgs/LaserScan),  (sensor_msgs/Joy)
  - Pub: , , , , , 
- センサ（urg_node2）
  - Pub: 
- 可視化（rviz2, robot_state_publisher）
  - URDF の TF/可視化、RViz パネル

## Launch 構成（抜粋）
- 
  - urg_node2 起動
  - YP-Spur  起動（デバイス/パラメータ指定）
  - ジョイスティック  と  起動
  - 走行ブリッジ  を遅延起動（依存立上げ待ち）
- 
  - URDF 読み込みと RViz 表示

## TF 構成
-  →  をブリッジから配信
- LiDAR フレーム（例: ）はセンサ実装側に準拠

## コード参照（代表）
- ブリッジ: 
- 追跡: 
- URDF: 
- Bringup: 

