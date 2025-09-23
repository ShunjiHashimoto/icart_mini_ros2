# YP-Spur ブリッジ（走行制御）

 は ROS 2 と YP-Spur を橋渡しし、速度指令  を受けてロボットを駆動、 と 、および TF（→）を配信します。

## 主機能
- YP-Spur 初期化とパラメータ設定（速度・加速度・角速度など）
-  購読 →  へ反映
- , , ,  に基づく配信
- TF ブロードキャスト（→）

## トピック
- Sub:  (geometry_msgs/Twist)
- Pub:  (nav_msgs/Odometry)
- Pub:  (sensor_msgs/JointState)
- TF: →

## 実装参照
- 

## Launch 連携
-  から  を起動
- 同じく  を遅延起動（依存の立ち上がり待ち）

