# icart_mini_ros2

屋内外用の小型移動ロボットフレーム「[i-Cart mini](https://t-frog.com/products/icart_mini/)」向けの ROS 2 パッケージ群です。

このリポジトリには、実機を動かすための bringup と、Gazebo + RViz 上で Follow me をデバッグするためのシミュレーション環境が含まれます。実機とシミュレーションでは起動手順が異なるため、この README では章を分けて説明します。

<img src=.docs/imgs/icart_mini.png width=40%>
<img src=.docs/imgs/icart_urdf.png width=38%> <img src=.docs/imgs/icart_rviz.png width=60%>

## パッケージ構成

| パッケージ | 役割 |
| --- | --- |
| `icart_mini_bringup` | 実機用の LiDAR、YP-Spur、ジョイスティック bringup |
| `icart_mini_description` | URDF、Gazebo 用ロボットモデル、RViz 設定、シミュレーション world |
| `icart_mini_leg_tracker` | LiDAR 点群から脚クラスタを追跡し、Follow me 用 `/cmd_vel` を生成 |
| `icart_mini_ypspur_bridge` | `YP-Spur` と ROS 2 の橋渡し |
| `docker` | 開発・シミュレーション実行用 Docker 環境 |

## 動作環境

### 実機

- Hardware: Raspberry Pi 5 + i-Cart mini 実機
- OS: Ubuntu 23.10
- ROS 2: Humble Hawksbill
- LiDAR: Hokuyo UST-10LX
- 駆動系: i-Cart mini 付属モータ、モータドライバ、YP-Spur
- Joystick: Logitech F710

### シミュレーション

- ROS 2: Humble Hawksbill
- Gazebo: Gazebo Fortress, Gazebo Classic 11
- RViz2
- Docker: 26.0.0 以降を推奨
- Joystick: Logitech F710

Gazebo Classic 11 は 2025年1月に EOL になっています。Follow me シミュレーションは Fortress 版への移行中で、通常の確認では Fortress 版 launch を使います。Classic 版 launch は移行期間中の比較・ロールバック用として残しています。

## 共通セットアップ

### ワークスペース準備

```bash
$ mkdir -p ~/icart_ws/src
$ cd ~/icart_ws
# 本リポジトリを src/ に配置した状態で依存パッケージを取得
$ vcs import src < src/icart_mini_ros2/ros2.repos
```
リポジトリルートには依存パッケージの取得元をまとめた `ros2.repos` を同梱しています（`icart_mini_ros2`, `i-Cart`, `yp-spur`, `urg_node2`）。`vcs import` を使えば、このファイルに記載されたリビジョンで依存リポジトリを一括取得できます。

### ビルド

```bash
cd ~/icart_ws
colcon build --symlink-install
source install/setup.bash
```

一部だけ再ビルドする場合:

```bash
colcon build --symlink-install --packages-select icart_mini_description icart_mini_leg_tracker
source install/setup.bash
```

ROS_DOMAIN_ID と CycloneDDS 設定を手動で合わせる場合:

```bash
export ROS_DOMAIN_ID=99
export CYCLONEDDS_URI=$HOME/icart_ws/src/cyclonedds.xml
```

## Docker 環境

シミュレーションは Docker 内での実行を前提にしています。`docker/run.sh` は以下を設定します。

- `--net=host`: ROS 2 / Gazebo の通信をホストと共有
- `--privileged`: 入力デバイスや Gazebo/RViz の実行を簡単にするため
- `/tmp/.X11-unix` と `~/.Xauthority`: Gazebo GUI / RViz の X11 表示
- `/dev/input` と `/dev/bus/usb`: Logitech F710 などのジョイスティック
- `~/icart_ws:/root/icart_ws`: ワークスペース共有

```bash
cd ~/icart_ws/src/icart_mini_ros2/docker
docker build -t icart_mini_ros2:latest .
./run.sh
```

Dockerfile には Fortress 版に必要な `ros_gz` / `ros_gz_sim` / `ros_gz_bridge` と、移行期間中の Classic 比較に使う `gazebo_ros_pkgs` の両方を含めています。

コンテナに入ったら:

```bash
cd /root/icart_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select icart_mini_description icart_mini_leg_tracker
source install/setup.bash
```

シミュレーションの起動:

```bash
ros2 launch icart_mini_leg_tracker follow_me_biped_sim_fortress.launch.py
```

障害物あり:

```bash
ros2 launch icart_mini_leg_tracker follow_me_obstacle_sim_fortress.launch.py
```

Classic 版で比較したい場合:

```bash
ros2 launch icart_mini_leg_tracker follow_me_biped_sim.launch.py
ros2 launch icart_mini_leg_tracker follow_me_obstacle_sim.launch.py
```

別ターミナルから既存コンテナに入る場合:

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source install/setup.bash
```

Gazebo GUI や RViz が表示されない場合は、ホスト側で X11 の許可を確認してください。

```bash
xhost +local:docker
```

Gazebo が `Address already in use` を出す場合は、前回の Gazebo が残っています。

```bash
# Fortress
pkill -f "gz sim"
pkill -f "ign gazebo"

# Classic
pkill -f gzserver
pkill -f gzclient
```

## Logitech F710

実機・シミュレーションともに、Logitech F710 は背面スイッチを `X` にしてください。`D` モードではボタン番号が変わり、RT など別のボタンが Follow me 開始として認識されることがあります。

コンテナ内で認識を確認します。

```bash
ros2 run joy joy_enumerate_devices
```

出力例:

```text
ID : GUID : GamePad : Mapped : Joystick Device Name
0  : ...  : true    : true   : Logitech Gamepad F710
```

この `ID` が launch の `joy_device_id` です。Linux の `/dev/input/js1` の番号とは一致しない場合があります。

単体確認:

```bash
ros2 run joy joy_node --ros-args --param device_id:=0
ros2 topic echo /joy
```

Follow me のボタン割り当て:

| 操作 | F710 | Button ID |
| --- | --- | --- |
| 非常停止 | RB | 5 |
| 非常停止解除 | LB | 4 |
| 追従開始 | Start | 7 |
| 追従停止 | Back | 6 |

## 実機 Bringup

実機では YP-Spur がホストにインストール済みであることを前提にします。

### YP-Spur のビルド

```bash
cd ~/icart_ws
mkdir -p build
cd build
cmake ../src/yp-spur
make
sudo make install
sudo ldconfig
```

### 実機起動

```bash
cd ~/icart_ws
source install/setup.bash
ros2 launch icart_mini_bringup icart_mini_bringup.launch.py
```

この launch は以下を起動します。

| 起動内容 | 主な役割 |
| --- | --- |
| `urg_node2` | Hokuyo UST-10LX から `/scan` を発行 |
| `ypspur-coordinator` | YP-Spur の低レベル制御 |
| `joy_node` | F710 の `/joy` を発行 |
| `teleop_twist_joy` | ジョイスティックから `/cmd_vel` を発行 |
| `icart_mini_ypspur_bridge` | `/cmd_vel` を YP-Spur に渡し、`/odom`、`/joint_states`、TF を発行 |

Follow me ノードを起動します。

```bash
ros2 run icart_mini_leg_tracker leg_cluster_tracking_node
```

`leg_cluster_tracking_node` は `/scan` と `/joy` を購読します。Start ボタンを押すまでは LiDAR のクラスタ検出を行い、追従開始後に `/cmd_vel` を発行します。

モデル表示だけ確認したい場合:

```bash
ros2 launch icart_mini_description icart_mini_display.launch.py
```

## シミュレーション Follow me

シミュレーションは Docker コンテナ内で実行します。Gazebo、RViz、icart モデル、LiDAR、倒立振子風の左右足円柱ターゲット、追従ノードをまとめて起動できます。

### ジョイスティックで確認する

通常の確認は Fortress 版 launch を使います。

```bash
ros2 launch icart_mini_leg_tracker follow_me_biped_sim_fortress.launch.py
```

起動直後:

- 左スティックで `icart_mini` を手動操作
- 速度指令は `/cmd_vel`

Start ボタン後:

- Follow me 開始
- 同じ左スティックで2本脚モデルを操作
- 2本脚モデル操作の速度指令は `/person/cmd_vel`
- ロボットは LiDAR の `/scan` から脚クラスタを追従

Back ボタン後:

- Follow me 停止
- 操作対象が `icart_mini` に戻る

障害物 world で確認する場合:

```bash
ros2 launch icart_mini_leg_tracker follow_me_obstacle_sim_fortress.launch.py
```

この launch はジョイスティック操作がデフォルトです。起動直後は `icart_mini` を操作し、Start ボタン後は左右足円柱のターゲットを操作します。スティック入力がゼロのときは足の踏み出しも停止します。
launch 起動時点では Follow me もターゲット移動も開始しません。Start ボタンで Follow me とターゲット操作を開始します。

Classic 版 launch は Gazebo Classic との差分確認やロールバック用に残しています。新しい確認や修正は Fortress 版を優先してください。

```bash
ros2 launch icart_mini_leg_tracker follow_me_biped_sim.launch.py
ros2 launch icart_mini_leg_tracker follow_me_obstacle_sim.launch.py
```

### 代表的な launch 引数

| launch | 主な用途 |
| --- | --- |
| `icart_mini_leg_tracker follow_me_biped_sim_fortress.launch.py` | Fortress の空 world で、左右足を交互に踏み出す倒立振子風ターゲットを使った Follow me |
| `icart_mini_leg_tracker follow_me_obstacle_sim_fortress.launch.py` | Fortress の障害物 world での倒立振子風ターゲット Follow me |
| `icart_mini_leg_tracker follow_me_biped_sim.launch.py` | Classic 版の比較・ロールバック用 Follow me |
| `icart_mini_leg_tracker follow_me_obstacle_sim.launch.py` | Classic 版の障害物 world 比較用 Follow me |
| `icart_mini_description icart_mini_display.launch.py` | RViz 上で icart モデルだけを確認 |

| 引数 | 例 | 説明 |
| --- | --- | --- |
| `gui` | `gui:=false` | Gazebo GUI の有無 |
| `use_rviz` | `use_rviz:=false` | RViz の有無 |
| `use_joy` | `use_joy:=true` | ジョイスティック操作の有無。デフォルトは `true` |
| `joy_device_id` | `joy_device_id:=0` | `joy_enumerate_devices` の ID |
| `initial_x` | `initial_x:=0.5` | ターゲット初期 x 位置 |
| `initial_y` | `initial_y:=0.0` | ターゲット初期 y 位置 |
| `update_rate` | `update_rate:=60.0` | 倒立振子風ターゲットの Gazebo 更新周期 |
| `step_length` | `step_length:=0.24` | 倒立振子風ターゲットの左右足の前後ステップ幅 |
| `step_frequency` | `step_frequency:=1.2` | 倒立振子風ターゲットのステップ周期 |

### Fortress 版の表示・トピック確認

Fortress 版 launch は RViz 設定 `follow_me_sim.rviz` を使い、Fixed Frame を `odom`、LaserScan を `/scan`、脚クラスタと人物マーカーを `/leg_tracker/*` の MarkerArray として表示します。

別ターミナルからコンテナへ入り、代表的な入出力を確認します。

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source install/setup.bash

ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
ros2 topic echo /leg_tracker/cluster_markers --once
ros2 topic echo /leg_tracker/person_marker --once
```

`ros2 topic echo /tf --once` で `odom -> base_footprint` が見え、RViz の Fixed Frame `odom` にエラーが出ないことを確認します。Start ボタン後は脚ターゲットが動き、`/leg_tracker/cluster_markers` と `/leg_tracker/person_marker` が更新されることを確認します。

## シミュレーションのモデル

Gazebo シミュレーションでは、リアルな人物モデルではなく円柱からなる簡易脚モデルを追従対象にします。Fortress 版の `follow_me_biped_sim_fortress.launch.py` では左右の脚を別エンティティとして spawn し、`/world/<world_name>/set_pose` service 経由で位置を更新します。倒立振子モデルの簡易表現として左右足が交互に前後するようにし、Gazebo 上には倒立振子リンクは表示せず、LiDAR に見える円柱だけを動かします。

## ノード / トピック概要

| ノード | 役割 | 購読 | 発行 |
| --- | --- | --- | --- |
| `leg_cluster_tracking_node` | LiDAR 点群から脚クラスタを検出し追従制御を生成 | `/scan`, `/joy`, `/follow_me/control` | `/cmd_vel`, `/leg_tracker/cluster_markers`, `/leg_tracker/cluster_centers`, `/leg_tracker/cluster_infos`, `/leg_tracker/person_marker`, `/leg_tracker/is_lost_target` |
| `joystick_follow_me_teleop.py` | シミュレーション用に F710 の操作対象をロボットと2本脚モデルで切替 | `/joy` | `/cmd_vel`, `/person/cmd_vel`, `/person/control` |
| `inverted_pendulum_biped_controller.py` | 左右足を交互に踏み出す倒立振子風ターゲットを移動 | `/person/cmd_vel`, `/person/control`, `/world/<world_name>/set_pose` | `/person/motion_event` |
| `icart_mini_ypspur_bridge` | 実機用 YP-Spur ブリッジ | `/cmd_vel` | `/odom`, `/joint_states`, TF |
| `urg_node2` | 実機 LiDAR ドライバ | - | `/scan` |

## icart_mini_leg_tracker の処理概要

### Preprocessing

- LiDAR の生データを座標変換し、極端に近い点群を除去
- 点群を間引いて計算量を削減
- 遠すぎる点をクラスタ対象から除外

### Clustering

- PCL の `KdTree` と `EuclideanClusterExtraction` を使用
- クラスタサイズと距離で脚候補を選別
- 各クラスタの重心を算出

### Tracking

- 過去フレームの重心と ID 履歴からクラスタ ID を安定化
- 一時的に見失ったクラスタを速度ベクトルから補間
- 速度履歴を平滑化し、静止判定を行う

### Following

- 有効領域内でターゲット候補を抽出
- 既存ターゲットの継続可否を距離と移動量から判定
- PID 制御で前進・旋回速度を生成
- 追従対象ロスト時は `/leg_tracker/is_lost_target` を通知

<img src=.docs/imgs/clustering.png width=50%>

## Rosbag / デバッグ

シミュレーション中に別ターミナルでコンテナへ入り、代表的なトピックを記録します。

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source install/setup.bash
ros2 run icart_mini_leg_tracker record_follow_me_bag.sh
```

出力先を指定する場合:

```bash
ros2 run icart_mini_leg_tracker record_follow_me_bag.sh /root/icart_ws/src/icart_mini_ros2/icart_mini_leg_tracker/rosbag/test_run
```

記録対象:

- `/scan`
- `/tf`, `/tf_static`
- `/odom`
- `/cmd_vel`
- `/joy`
- `/person/cmd_vel`, `/person/control`, `/person/motion_event`
- `/leg_tracker/cluster_markers`
- `/leg_tracker/cluster_centers`
- `/leg_tracker/cluster_infos`
- `/leg_tracker/person_marker`
- `/leg_tracker/is_lost_target`

LiDAR と joystick 入力を再生して追跡ノードを再デバッグする例:

```bash
ros2 run icart_mini_leg_tracker leg_cluster_tracking_node
ros2 bag play /path/to/bag --clock --rate 0.5 \
  --topics /scan /tf /tf_static /joy /person/cmd_vel /person/control /person/motion_event
```

再生結果の速度指令や追跡状態を別ターミナルで確認します。

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /leg_tracker/is_lost_target
ros2 topic echo /leg_tracker/cluster_infos
```

ClusterInfo を CSV として保存する例:

```bash
ros2 topic echo /leg_tracker/cluster_infos --csv > cluster_infos.csv
```

ログファイルをリセットしたい場合は、`icart_mini_leg_tracker/csv/cluster_tracking_log.csv` を削除してください。ノード起動時に必要なログファイルは再生成されます。

## トラブルシュート

### `joy_enumerate_devices` に F710 が出ない

- F710 の USB ドングルをホストに挿し直す
- ホストで `ls /dev/input/js*` を確認する
- コンテナを作り直す

```bash
cd ~/icart_ws/src/icart_mini_ros2/docker
docker rm -f icart_mini_ros2
./run.sh
```

### X モードでジョイスティック操作できない

`/dev/input/js1` と `joy_device_id:=1` は同じ意味ではありません。コンテナ内で `joy_enumerate_devices` を実行し、表示された `ID` を `joy_device_id` に指定してください。

### D モードで RT が追従開始になる

F710 の `D` モードではボタン番号が README の割り当てと変わります。背面スイッチを `X` にして、`joy_node` または launch を再起動してください。

### Gazebo が起動しない

前回の Gazebo が残っている場合があります。

```bash
# Fortress
pkill -f "gz sim"
pkill -f "ign gazebo"

# Classic
pkill -f gzserver
pkill -f gzclient
```

`bind: Address already in use` が続く場合は、Gazebo master のプロセスが残っていないか確認してください。

```bash
ps aux | grep -E "gz sim|ign gazebo|gzserver|gzclient|ros2 launch" | grep -v grep
```

### CycloneDDS が `wlan0: does not match an available interface` を出す

Docker 内で指定したネットワークインターフェース名が存在しない場合に出ます。コンテナ内のインターフェース名を確認し、`cyclonedds.xml` の設定を合わせてください。

```bash
ip addr
```

## 依存パッケージ

- [YP-Spur](https://github.com/openspur/yp-spur)
  - 実機で使用
  - `ros2.repos` に記載されたバージョンを使用
- [urg_node2](https://github.com/ShunjiHashimoto/urg_node2)
  - 実機 LiDAR で使用
- [i-Cart モデルデータ](https://github.com/BND-tc/i-Cart)
  - `icart_mini_description` の URDF・パラメータで使用

`apt` では `ros-humble-joy`、`ros-humble-teleop-twist-joy`、`ros-humble-pcl-ros`、Fortress 版に必要な `ros-humble-ros-gz` / `ros-humble-ros-gz-sim` / `ros-humble-ros-gz-bridge`、Classic 版比較用の `ros-humble-gazebo-ros-pkgs` などが必要です。Docker イメージを使う場合は Dockerfile 内でインストールされます。
