# YP-Spur ブリッジ（走行制御）

`icart_mini_ypspur_bridge` は ROS 2 と YP-Spur を橋渡しし、速度指令 `/cmd_vel` を受けてロボットを駆動、`/odom` と `/joint_states`、および TF（`odom`→`base_footprint`）を配信します。

## 主機能
- YP-Spur 初期化とパラメータ設定（速度・加速度・角速度など）
- `/cmd_vel` 購読 → `Spur_vel(linear, angular)` へ反映
- `Spur_get_pos_GL`, `Spur_get_vel`, `YP_get_wheel_ang`, `YP_get_wheel_vel` に基づく配信
- TF ブロードキャスト（`odom`→`base_footprint`）

## トピック
- Sub: `/cmd_vel` (geometry_msgs/Twist)
- Pub: `/odom` (nav_msgs/Odometry)
- Pub: `/joint_states` (sensor_msgs/JointState)
- TF: `odom`→`base_footprint`

## 実装参照
- `icart_mini_ypspur_bridge/src/icart_mini_ypspur_bridge.cpp`

## Launch 連携
- `icart_mini_bringup/launch/icart_mini_bringup.launch.py` から `ypspur-coordinator` を起動
- 同じく `icart_mini_ypspur_bridge` を遅延起動（依存の立ち上がり待ち）
