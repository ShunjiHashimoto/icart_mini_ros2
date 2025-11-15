# icart_mini_ros2
屋内外用の小型移動ロボットフレーム「[i-Cart mini](https://t-frog.com/products/icart_mini/)」向けの ROS 2 パッケージ群です。  
<img src=.docs/imgs/icart_mini.png width=40%>  
<img src=.docs/imgs/icart_urdf.png width=38%> <img src=.docs/imgs/icart_rviz.png width=60%>

## System Requirements
- Hardware: Raspberry Pi 5 + i-Cart mini 実機
- OS: Ubuntu 23.10
- ROS 2: Humble Hawksbill
- Docker (任意): 26.0.0 以降
- LiDAR: Hokuyo [UST-10LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16&utm_source=google&utm_medium=cpc&utm_campaign=[P-MAX]&gad_source=1&gclid=Cj0KCQiAwtu9BhC8ARIsAI9JHam6cR3BVtNZ746VwLahng9sImtlVbThGx0BkbivMfSW7eK9brOBjaYaAjHhEALw_wcB#spec)
- 電源: LONG 12V 鉛蓄電池 ×2（24V→5V 変換に DROK 090011_JPN を使用）
- 駆動系: i-Cart mini 付属モータ & モータドライバ

## パッケージ構成
- `icart_mini_bringup`: LiDAR、YP-Spur、ジョイスティックをまとめて起動する bringup ランチ
- `icart_mini_description`: URDF / RViz 設定を提供するロボットモデルパッケージ
- `icart_mini_leg_tracker`: LiDAR 点群から脚クラスタを追跡し `/cmd_vel` を生成するノード
- `icart_mini_ypspur_bridge`: `YPSpur` と ROS 2 の橋渡し（`/cmd_vel` からの制御・オドメトリ生成）
- `docker`: 開発・実行環境を統一するための Dockerfile / スクリプト
- `sh`: 実機 bringup を Docker で起動・停止する補助スクリプト (`start_icart.sh`, `stop_icart.sh`, `auto_start.sh`)

## セットアップ
### ワークスペース準備
```bash
$ mkdir -p ~/icart_ws/src
$ cd ~/icart_ws
# 本リポジトリを src/ に配置した状態で依存パッケージを取得
$ vcs import src < src/icart_mini_ros2/ros2.repos
```
リポジトリルートには依存パッケージの取得元をまとめた `ros2.repos` を同梱しています（`icart_mini_ros2`, `i-Cart`, `yp-spur`, `urg_node2`）。`vcs import` を使えば、このファイルに記載されたリビジョンで依存リポジトリを一括取得できます。

### YP-Spur のビルド
`icart_mini_ypspur_bridge` はホストに `YP-Spur` がインストール済みであることを前提とします。

```bash
$ cd ~/icart_ws
$ mkdir -p build
$ cd build
$ cmake ../src/yp-spur
$ make
$ sudo make install
$ sudo ldconfig
```

### colcon build
```bash
$ cd ~/icart_ws
$ colcon build --symlink-install
$ source install/setup.bash
# CYCLONEDDS の QoS 設定を共有する場合
$ export CYCLONEDDS_URI=$HOME/icart_ws/src/cyclonedds.xml
$ export ROS_DOMAIN_ID=99
```
特定パッケージのみを再ビルドしたい場合は `--packages-select icart_mini_leg_tracker` のように指定してください。

## Docker を利用する場合
```bash
$ cd ~/icart_ws/src/icart_mini_ros2/docker
$ docker build -t icart_mini_ros2:latest .
$ ./run.sh   # DISPLAY 共有・privileged・host network で起動
```
コンテナ起動後は `docker/before_bringup.sh` が `~/.bashrc` から自動で読み込まれ、`ROS_DOMAIN_ID=99` と `CYCLONEDDS_URI` が設定されます。`~/icart_ws` はホストと共有されるため、ホストでビルドした成果物をそのまま利用できます。

## Quick Start
### 1. 実機 Bringup
LiDAR / 低レベル制御 / ジョイスティックを起動します。
```bash
$ ros2 launch icart_mini_bringup icart_mini_bringup.launch.py
```
このlaunchは以下を順に起動します。
- `urg_node2`（/scan）
- `/root/icart_ws/build/ypspur-coordinator`（`/dev/ttyACM0` を使用）
- `ros2 run joy joy_node --ros-args --param device_id:=0`
- `ros2 run teleop_twist_joy teleop_node --params-file icart_mini_ypspur_bridge/config/teleop_twist_joy_f710_params.yaml`
- （3 秒遅延後）`ros2 run icart_mini_ypspur_bridge icart_mini_ypspur_bridge`

### 2. モデル表示 (任意)
```bash
$ ros2 launch icart_mini_description icart_mini_display.launch.py
```
`robot_state_publisher` と `rviz2`（事前設定済みレイアウト）を起動します。

### 3. 脚クラスタ追跡ノード
```bash
$ ros2 run icart_mini_leg_tracker leg_cluster_tracking_node
```
`Joy` の入力を監視し、追従開始ボタンが押されるまで LiDAR のみを監視します。追従開始後は `/cmd_vel` を出力し、`icart_mini_ypspur_bridge` 経由で駆動系を制御します。

### フォローミーモードの操作 (Logitech F710 デフォルト設定)
| 操作 | ボタン ID | 説明 |
| ---- | --------- | ---- |
| 非常停止 | 5 (RB) | `/cmd_vel` を即座に 0 にし、追従を停止 |
| 非常停止解除 | 4 (LB) | 非常停止状態を解除 |
| 追従開始 | 7 (Start) | 追従対象をリセットしフォローミー開始 |
| 追従停止 | 6 (Back) | 追従状態を終了しターゲット情報をクリア |

## ノード / トピック概要
| ノード | パッケージ | 役割 | 購読 | 発行 |
| ------ | ---------- | ---- | ---- | ---- |
| `leg_cluster_tracking_node` | `icart_mini_leg_tracker` | LiDAR 点群から脚クラスタを検出し追従制御を生成 | `/scan`, `/joy` | `/cmd_vel`, `/leg_tracker/cluster_markers`, `/leg_tracker/cluster_centers`, `/leg_tracker/cluster_infos`, `/leg_tracker/person_marker`, `/leg_tracker/is_lost_target` |
| `icart_mini_ypspur_bridge` | `icart_mini_ypspur_bridge` | `YPSpur` とのブリッジ（速度指令とオドメトリ） | `/cmd_vel` | `/odom`, `/joint_states`, TF (`odom` → `base_footprint`) |
| `urg_node2` | `urg_node2` | Hokuyo UST-10LX ドライバ | - | `/scan` |
| `teleop_node` | `teleop_twist_joy` | ジョイスティックから速度指令を生成（非常停止を含む） | `/joy` | `/cmd_vel` |

## icart_mini_leg_tracker の処理パイプライン
### Preprocessing (前処理)
- LiDAR の生データを座標変換し、極端に近い点群を `MAX_NOISE_DISTANCE_THRESH` で除去
- `MAX_SAMPLING_INTERVAL` 以内の点を間引いて計算量を削減
- `MAX_CLUSTER_DISTANCE` を超える点はクラスタ対象から除外

### Clustering (クラスタリング)
- PCL の `KdTree` + `EuclideanClusterExtraction` を使用
- `CLUSTER_TOLERANCE`、`MIN_CLUSTER_SIZE`、`MAX_CLUSTER_SIZE` でクラスタを選別
- 各クラスタの重心を算出し追跡処理に渡す

### Tracking (追跡)
- 過去フレームの重心と `cluster_id_history_` を利用して ID を安定化
- `LOST_CLUSTER_TIMEOUT` 以内であれば失われたクラスタを速度ベクトルから補間し再マッチ
- `smoothAndFilterVelocities` で速度履歴を平滑化し、静止判定 (`STATIC_SPEED_THRESHOLD` / `STATIC_FRAME_LIMIT`)

### Following (追従)
- 有効領域（前方 1.0 m × 横 ±0.4 m）内でターゲット候補を抽出
- 既存ターゲットの継続可否を最小距離と移動量 (`MOVEMENT_THRESHOLD`) から判定
- PID 制御（`KP_DIST`, `KI_DIST`, `KP_ANGLE`, `KI_ANGLE`）で前進・旋回速度を生成
- 追従対象ロスト時は `/leg_tracker/is_lost_target` を通知

### Visualization & Logging
- `MarkerHelper` でクラスタ点群・中心・ターゲットを `MarkerArray` として発行
- `ClusterInfoArray` でクラスタ ID・速度・静止判定を配信
- `logs/cluster_tracking_log.csv` にクラスタ履歴と速度を蓄積（CSVLogger）
<img src=.docs/imgs/clustering.png width=50%>

## カスタムメッセージ
- `ClusterInfo.msg`
  - `int32 id`
  - `geometry_msgs/Point center`
  - `geometry_msgs/Vector3 velocity`
  - `bool is_static`
  - `bool is_target`
- `ClusterInfoArray.msg`
  - `ClusterInfo[] clusters`

## Debug Tips
```bash
# 代表的なトピックを rosbag 収集
$ ros2 bag record /scan /tf /tf_static \
    /leg_tracker/cluster_markers /leg_tracker/cluster_centers \
    /leg_tracker/cluster_infos /leg_tracker/person_marker

# LiDAR を記録した bag を再生（速度 0.5 倍、開始時一時停止）
$ ros2 bag play my_bag --rate 0.5 --topics /scan --start-pause

# ClusterInfo を CSV 出力
$ ros2 topic echo /leg_tracker/cluster_infos --csv > cluster_infos.csv
```
ログファイルをリセットしたい場合は `logs/cluster_tracking_log.csv` を削除するか、ノード起動時に自動生成されるファイルを利用してください。

## 依存パッケージ
- [YP-Spur](https://github.com/openspur/yp-spur)
  - 上記ビルド手順を参照し、`make install` まで実施
  - 最新のyp-spurではなく、ros2.reposに記載されたバージョンをcloneする。
- [urg_node2](https://github.com/ShunjiHashimoto/urg_node2)
  - `colcon build --symlink-install --packages-select urg_node2`
- [i-Cart モデルデータ](https://github.com/BND-tc/i-Cart)
  - `icart_mini_description` の URDF・パラメータで使用

これらは `ros2.repos` に含まれているため `vcs import` で取得可能です。`apt` では `ros-humble-teleop-twist-joy`、`ros-humble-joy`、`ros-humble-pcl-ros` などを事前にインストールしておくとビルドがスムーズです。
