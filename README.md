# icart_mini_ros2
屋内外用の小型移動ロボットフレーム「[i-Cart mini](https://t-frog.com/products/icart_mini/)」のROS 2パッケージ
<img src=.docs/imgs/icart_mini.png width=40%>  
<img src=.docs/imgs/icart_urdf.png width=38%> <img src=.docs/imgs/icart_rviz.png width=60%>

# System Requirements
- Hardware: Raspberry Pi 5
- OS: Ubuntu23.10
- ROS 2: humble
- Docker: version 26.0.0
- Sensor: Hokuyo [UST-10LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16&utm_source=google&utm_medium=cpc&utm_campaign=[P-MAX]&gad_source=1&gclid=Cj0KCQiAwtu9BhC8ARIsAI9JHam6cR3BVtNZ746VwLahng9sImtlVbThGx0BkbivMfSW7eK9brOBjaYaAjHhEALw_wcB#spec)
- Battery: LONG 鉛蓄電池12V x2
  - Raspberry Pi 5用の電源として使用するために、DCDC24V->5Vに降圧する電圧レギュレータ（DROK製 型番：090011_JPN）を使用
- MotorDriver
- Motor

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

### Debug
```bash
$ ros2 bag record /leg_tracker/cluster_infos /scan /tf /tf_static /leg_tracker/cluster_centers /leg_tracker/cluster_markers /leg_tracker/person_marker   

$ ros2 bag play my_bag --rate 0.5 --topics /scan --pause
```

# Required Packages
### YP-Spur(YamabicoProject-Spur) 
[Yp-Spur](https://github.com/openspur/yp-spur)は、移動ロボット用のオープンソースソフトウェアで、ロボットのモーションコントロールをシンプルかつ効率的に行うためのツールです。ビルド方法はYp-SpurのGitHubページを参照ください。下記は一例です。       
```bash
$ cd icart_ws
$ mkdir build
$ cd build
$ cmake ../src/yp-spur
$ make
$ sudo make install
```
下記は、yp-spurをros2で使用するためのパッケージです。  
```bash
colcon build --symlink-install --packages-select icart_mini_ypspur_bridge
```

### urg_node2
[urg_node2](https://github.com/ShunjiHashimoto/urg_node2.git)は、Hokuyo製LiDARセンサーのデータをROS 2トピックとして配信するためのパッケージ  
```bash
$ colcon build --symlink-install --packages-select urg_node2
```

### i-Cart
[i-Cart](https://github.com/BND-tc/i-Cart)は、i-Cartシリーズのモデルデータおよびパラメータファイル

# Technical Overview
### Preprocessing (前処理)
LiDARデータを用いたクラスタ追跡を行う前に、データのノイズ除去や前処理を実施。

1. **ノイズ除去 (Noise Removal)**  
   - 点群内の極端に近い点をノイズとして除去 (閾値: `MAX_NOISE_DISTANCE_THRESH`)
   - 一定距離以下の点群をフィルタリングし、データの品質を向上

2. **ダウンサンプリング (Downsampling)**  
   - 隣接する点を間引き、データサイズを削減 (`MAX_SAMPLING_INTERVAL` を閾値として適用)
   - 計算コストを抑え、リアルタイム処理の効率を向上

---

### Clustering (クラスタリング)
前処理を施した点群データをもとに、**Euclidean Cluster Extraction** を用いてクラスタリングを行う。

1. **KD-Tree を用いた近傍探索**
   - PCLの `KdTree` を使用し、点群の空間構造を解析
   - クラスタ形成に必要な **クラスタ間の許容距離 (`CLUSTER_TOLERANCE`)** を設定

2. **クラスタサイズの制限**
   - `MIN_CLUSTER_SIZE` (最小点数) 以上、`MAX_CLUSTER_SIZE` (最大点数) 以下のクラスタのみ有効
   - 不適切なクラスタ (小さすぎる or 大きすぎる) を排除

3. **クラスタの中心座標を算出**
   - 各クラスタの点群を平均し、**クラスタの重心** を求める
   - 重心をもとに、次の追跡・追従処理を実施

---

### Tracking (クラスタ追跡)
前フレームのクラスタ情報と現在のクラスタ情報を比較し、**クラスタIDを一貫性を持って追跡** する。

1. **速度ベクトルを考慮したクラスタマッチング**
   - 各クラスタの移動ベクトル (`cluster_velocities_`) を計算し、過去のクラスタの予測位置を算出
   - `CLUSTER_MATCHED_THRESH` (クラスタマッチ閾値) を用いて、前フレームとの一致を判定

2. **失われたクラスタの復活**
   - 一時的に検出されなくなったクラスタを、**過去の速度ベクトルから予測** し、一致すれば復活させる (`LOST_CLUSTER_TIMEOUT` 以内)

3. **クラスタIDの履歴を活用**
   - `cluster_id_history_` を用いて、過去のIDを保持し、一貫性のあるIDを割り当てる
   - 過去5フレームのクラスタID履歴を保持し、急激なID変化を防止

---

### Following (クラスタ追従)
クラスタの中から **「追従すべき対象」** を選定し、ロボットの移動制御を行う。

1. **追従対象の決定**
   - **最も近いクラスタ** または **前回の対象に近いクラスタ** を優先
   - 急激な移動を防ぐため、**前回のクラスタと大きく離れた対象は無視**

2. **速度・旋回制御**
   - 追従対象との距離に応じて **前進速度 (`cmd_vel.linear.x`)** を調整
   - 追従対象の方向に向くように **旋回 (`cmd_vel.angular.z`)** を計算

3. **ジョイスティック操作による停止**
   - `Joy` メッセージの入力に応じて、ロボットの動作を即時停止

---

### Visualization (可視化)
処理結果を Rviz で確認するために、以下のデータを `visualization_msgs::msg::Marker` を使ってパブリッシュ:

1. **クラスタごとの点群 (`/leg_tracker/cluster_markers`)**
2. **クラスタの中心 (`/leg_tracker/cluster_centers`)**
3. **追従対象 (`/leg_tracker/target_marker`)**
<img src=.docs/imgs/clustering.png width=50%>

