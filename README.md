# icart_mini_ros2

屋内外用の小型移動ロボットフレーム「[i-Cart mini](https://t-frog.com/products/icart_mini/)」向けの ROS 2 パッケージ群です。

このリポジトリには、実機を動かすための bringup と、Gazebo + RViz 上で Follow me をデバッグするためのシミュレーション環境が含まれます。README では新しいデバイスで最初に動かすための最小手順をまとめ、シミュレーションの詳細は [docs/simulation.md](docs/simulation.md) に分けています。

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
| `sh` | 実機 bringup を Docker で起動・停止する補助スクリプト |

## 動作環境

### 実機

- Hardware: Raspberry Pi 5 + i-Cart mini 実機
- OS: Ubuntu 23.10 確認済み
- ROS 2: Humble Hawksbill
- LiDAR: Hokuyo UST-10LX
- 駆動系: i-Cart mini 付属モータ、モータドライバ、YP-Spur
- Joystick: Logitech F710

Raspberry Pi への Ubuntu インストールは [Ubuntu for Raspberry Pi](https://ubuntu.com/download/raspberry-pi) を参照してください。Ubuntu 22.04 / 23.04 でセットアップする場合も、ROS 2 Humble と Docker の対応状況を確認してください。

### シミュレーション

- ROS 2: Humble Hawksbill
- Gazebo: Gazebo Fortress
- RViz2
- Docker: 26.0.0 以降を推奨
- Joystick: Logitech F710

シミュレーションの起動、launch 引数、rosbag デバッグは [docs/simulation.md](docs/simulation.md) を参照してください。

## 初回セットアップ

### OS と基本パッケージ

```bash
sudo apt update
sudo apt install -y git ssh python3-vcstool build-essential cmake
sudo ssh-keygen -A
```

`build-essential` は YP-Spur のビルドに必要です。`cmake ../src/yp-spur` で `No CMAKE_CXX_COMPILER could be found` が出る場合は、C++ コンパイラが入っていません。

### Docker

Docker Engine は [Docker 公式の Ubuntu 向け手順](https://docs.docker.com/engine/install/ubuntu/#installation-methods) に従ってインストールしてください。インストール後に Docker daemon を起動し、必要なら sudo なしで使えるようにします。

```bash
sudo systemctl start docker
sudo systemctl enable docker

sudo usermod -aG docker $USER
newgrp docker
docker run hello-world
```

`docker` group が反映されない場合は、いったんログアウトして入り直してください。

### ワークスペース準備

```bash
mkdir -p ~/icart_ws/src
cd ~/icart_ws
git clone git@github.com:ShunjiHashimoto/icart_mini_ros2.git src/icart_mini_ros2
vcs import src < src/icart_mini_ros2/ros2.repos
```

`ros2.repos` には `icart_mini_ros2`, `gazebo-ros-actor-plugin`, `i-Cart`, `yp-spur`, `urg-node` の取得元をまとめています。

`urg-node/urg_library` が空の場合は、urg_node2 の submodule を初期化してください。

```bash
git -C src/urg-node submodule update --init --recursive
```

Docker コンテナ内の root ユーザでホスト共有 workspace を扱うと、`vcs import` や `git` が `detected dubious ownership` を出す場合があります。その場合だけ safe directory に追加します。

```bash
git config --global --add safe.directory /root/icart_ws/src/icart_mini_ros2
```

### Docker コンテナ

```bash
cd ~/icart_ws/src/icart_mini_ros2/docker
docker build --network=host -t icart_mini_ros2:latest .
./run.sh
```

Docker build 後に `LegacyKeyValueFormat` の warning が出る場合があります。これは Dockerfile の古い `ENV` 記法に対する警告で、ビルドが成功していれば無視できます。

コンテナに入ったら、必要なパッケージをビルドします。

```bash
cd /root/icart_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gazebo_ros_actor_plugin icart_mini_description icart_mini_leg_tracker
source install/setup.bash
```

## 実機 Bringup

実機では YP-Spur と USB デバイス権限の準備が必要です。

### YP-Spur のビルド

現行 launch は `/root/icart_ws/build/ypspur-coordinator` を起動するため、Docker 内で実機 bringup する場合は workspace の `build` 直下に `ypspur-coordinator` がある状態にします。

```bash
cd ~/icart_ws
mkdir -p build
cd build
cmake ../src/yp-spur
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

`icart_mini_ypspur_bridge.cpp: fatal error: ypspur.h: No such file or directory` が出る場合は、YP-Spur の install が完了していません。上の `sudo make install` と `sudo ldconfig` まで実行してください。

### USB デバイス確認

YP-Spur のデバイスは通常 `/dev/ttyACM0` として見えます。

```bash
ls -l /dev/ttyACM0
sudo dmesg | grep ttyACM
groups
```

`/dev/ttyACM0` が `root dialout` の場合、ユーザを `dialout` group に追加してからログインし直します。

```bash
sudo usermod -aG dialout $USER
```

### ROS 2 パッケージのビルド

```bash
cd ~/icart_ws
colcon build --symlink-install
source install/setup.bash
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

## Logitech F710

実機・シミュレーションともに、Logitech F710 は背面スイッチを `X` にしてください。`D` モードではボタン番号が変わり、RT など別のボタンが Follow me 開始として認識されることがあります。

```bash
ros2 run joy joy_enumerate_devices
ros2 run joy joy_node --ros-args --param device_id:=0
ros2 topic echo /joy
```

`joy_enumerate_devices` の `ID` が launch の `joy_device_id` です。Linux の `/dev/input/js1` の番号とは一致しない場合があります。

| 操作 | F710 | Button ID |
| --- | --- | --- |
| 非常停止 | RB | 5 |
| 非常停止解除 | LB | 4 |
| 追従開始 | Start | 7 |
| 追従停止 | Back | 6 |

## 詳細ドキュメント

- [シミュレーション Follow me](docs/simulation.md)
- [All-in-One Wiki](Wiki.md)
- [.docs/wiki/Year-In-Review.md](.docs/wiki/Year-In-Review.md)
- [.docs/wiki/Roadmap.md](.docs/wiki/Roadmap.md)

## 依存パッケージ

- [YP-Spur](https://github.com/openspur/yp-spur): 実機で使用
- [urg_node2](https://github.com/ShunjiHashimoto/urg_node2): 実機 LiDAR で使用
- [i-Cart モデルデータ](https://github.com/BND-tc/i-Cart): URDF・パラメータで使用
- [gazebo-ros-actor-plugin](https://github.com/ShunjiHashimoto/gazebo-ros-actor-plugin): Fortress Actor 版の人物制御で使用

Docker イメージを使う場合、ROS 2 / Gazebo / RViz / joy / PCL などの apt 依存は Dockerfile 内でインストールされます。
