# icart_mini_ros2 — All-in-One Wiki

> このファイルは .docs/wiki/*.md を結合して自動生成しています。
> 画像参照はリポジトリルート基準（.docs/imgs/）です。

## Table of Contents
- [Year-In-Review](#year-in-review)
- [Architecture](#architecture)
- [Setup](#setup)
- [Leg-Tracker](#leg-tracker)
- [YPSpur-Bridge](#ypspur-bridge)
- [Roadmap](#roadmap)
- [Changelog](#changelog)


---

## Year-In-Review

# 1年間の成果サマリ（Year in Review）

## ハイライト
- ROS 2 Humble でのロボット一式のBringupを確立（Docker化含む）
- 走行系：YP-Spur と ROS 2 の橋渡しノード実装（`/cmd_vel` → 走行、`/odom`・`/joint_states`・TF）
- センサ系：Hokuyo UST-10LX を urg_node2 で接続・可視化・デバッグ手順整備
- 追従系：LiDARベースの脚クラスタ検出・ID追跡・Follow-me 制御の実装（PID）
- 表示系：URDF（xacro）と RViz 構成を整理、可観測性向上
- Bringup まとめ起動（センサ/走行/ジョイ操作/追従ノード連携）の launch 化

## 現状のシステム構成
- Bringup: `icart_mini_bringup`
- 表示/URDF: `icart_mini_description`
- 走行ブリッジ: `icart_mini_ypspur_bridge`
- 追従（クラスタ追跡）: `icart_mini_leg_tracker`
- Docker での開発・実行環境整備（ビルドスクリプト、起動スクリプト）

## 実装済み機能（主要項目）
- 速度指令 `/cmd_vel` を YP-Spur 経由で走行系に反映
- オドメトリ `/odom` と `/tf`（`odom`→`base_footprint`）の配信
- ホイール状態 `/joint_states` の配信
- LiDAR `/scan` からのクラスタリング（PCL EuclideanClusterExtraction）
- クラスタ中心の推定、速度推定、ID一貫性維持、ロスト復帰
- Follow-me 制御（停止距離/角度・速度制限・PID・ジョイスティック非常停止）
- 可視化トピック（クラスタ点群・中心・対象）配信

## デモ・成果物（リンク用プレースホルダ）
- 実走動画（屋内）: TBD
- 実走動画（屋外）: TBD



## 課題と学び
- 近接・遮蔽時のIDスワップ対策（履歴・速度予測のさらなる強化余地）
- 動的環境でのロバスト性（多人数、交差、停止/再開）
- センサ/駆動レイテンシと制御ゲインのすり合わせ

## 1年のタイムライン

| 期間 | 四半期 | 主な取り組み |
|---|---|---|
| 2024-11-21 ~ 2025-02-18 | Q1 | 環境整備（Docker/依存パッケージ）、YP-Spur ブリッジ導入、Bringup/URDF/RViz の基盤整備 |
| 2025-02-20 ~ 2025-03-09 | Q2 | LiDARクラスタリング・トラッキング実装、Follow target/PID 制御の初期版 |
| 2025-03-10 ~ 2025-04-30 | Q3 | ログ/可視化/メッセージ拡充、ロスト復帰や選定ロジックの強化、安定化 |
| 2025-05-01 ~ 2025-09-23 | Q4 | パラメータチューニング（BLDC/DC切替含む）、追従性改善、各種バグ修正と最終調整 |


---

## Architecture

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


---

## Setup

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


---

## Leg-Tracker

# 脚クラスタ追跡と Follow-me

LiDAR `/scan` から脚候補をクラスタ化し、ID 一貫性を保ちながら追跡、最適な対象に追従します。可視化トピックで確認可能です。

## 機能概要
- 前処理: ノイズ除去（近接点除去）、ダウンサンプリング
- クラスタリング: PCL EuclideanClusterExtraction（KD-Tree）
- センター推定: クラスタ重心を計算
- トラッキング:
  - 速度推定（フレーム差分）、移動平均による平滑化
  - ロストクラスタの予測復帰（速度ベクトル・タイムアウト）
  - 有効領域フィルタ（前方・幅）
- 追従制御（PID）:
  - 距離・角度の誤差に基づく平滑制御
  - 角度大時は旋回優先、最小/最大速度制限
  - 非常停止（ジョイスティック）と追従開始/停止

## 主なトピック
- Sub: `/scan`, `/joy`
- Pub: `/cmd_vel`, `/leg_tracker/cluster_markers`, `/leg_tracker/cluster_centers`, `/leg_tracker/person_marker`, `/leg_tracker/cluster_infos`, `/leg_tracker/is_lost_target`

## 重要パラメータ（定数）
- クラスタリング: `CLUSTER_TOLERANCE=0.05`, `MIN_CLUSTER_SIZE=10`, `MAX_CLUSTER_SIZE=100`
- 有効距離: `MAX_CLUSTER_DISTANCE=2.5`
- マッチング閾値: `CLUSTER_MATCHED_THRESH=0.2`, `CLUSTER_LOST_MATCHED_THRESH=0.2`
- ロスト保持: `LOST_CLUSTER_TIMEOUT=1.0[s]`
- 追従停止距離: `STOP_DISTANCE_THRESHOLD=0.3[m]`
- 速度上限: `MAX_SPEED=0.25[m/s]`, `MAX_TURN_SPEED=π/4`
- PID: `KP_DIST=0.5, KI_DIST=0.01, KP_ANGLE=1.0, KI_ANGLE=0.001`

## 可視化
- `/leg_tracker/cluster_markers`: 各点に色
- `/leg_tracker/cluster_centers`: クラスタ中心（ID ラベル付き）
- `/leg_tracker/person_marker`: 追従対象の人型マーカー
- 画像: `.docs/imgs/clustering.png`

## コード参照
- `icart_mini_leg_tracker/src/leg_cluster_tracking.cpp`
- `icart_mini_leg_tracker/include/icart_mini_leg_tracker/leg_cluster_tracking.hpp`
- `icart_mini_leg_tracker/include/icart_mini_leg_tracker/utils/*`


---

## YPSpur-Bridge

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


---

## Roadmap

# 今後の計画（Roadmap）

## 改善候補（短期）
- 追従の安定化：IDスワップ時の復帰判定強化（近傍履歴/相関）
- 追従選定ロジック：前回対象との整合性とロボット近傍の重み付け最適化
- パラメータ外だし：クラスタ/追従/速度制限を YAML 化
- ログ整備：計測スクリプトと可視化ノートブックの追加

## 改善候補（中期）
- マルチターゲット環境のロバスト化（交錯、遮蔽、同時移動）
- 物体学習ベースの脚候補識別（クラスタ特徴＋学習器）
- センサフュージョン（IMU/オドメ/カメラ）で追従の頑健性向上

## 追加機能の案
- フェイルセーフ：通信断・センサ断時の状態機械と動作安全制御
- 速度プロファイルの最適化（滑らかさ、消費電力、安全距離）
- 導線生成（中間目標）との併用での追従品質向上

## 既知の課題
- 近接・遮蔽時にクラスタ ID が入れ替わるケース
- LiDAR 反射によるノイズ（光沢床/ガラス）と閾値依存
- デバイスごとの差（LiDAR レイアウト、マウント）



---

## Changelog

# 更新履歴（過去1年の要約）

> 参考：本ファイルは対外説明用の要約です。実際のコミットやタグに合わせ、必要に応じて加筆してください。

## v0.4（Q4）
- Follow-me PID チューニング、停止距離/角度対応を改善
- 追跡のロスト復帰ロジック安定化、可視化の整理
- Bringup 手順とデバッグコマンドを README/Wiki に集約

## v0.3（Q3）
- LiDAR クラスタリング/追跡の初期版実装
- 可視化（クラスタ点群・中心・対象）を Marker で提供
- 追従対象の初期選定・継続判定・IDマッピングを追加

## v0.2（Q2）
- YP-Spur ブリッジを実装（/cmd_vel, /odom, /joint_states, TF）
- URDF/RViz 構成整備、手動操作（joy/teleop_twist_joy）

## v0.1（Q1）
- Docker ベース開発環境の整備
- パッケージの雛形作成と Bringup のたたき台


