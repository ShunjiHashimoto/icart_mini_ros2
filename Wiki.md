# icart_mini_ros2 — All-in-One Wiki

## Table of Contents
- [Year-In-Review](#year-in-review)
- [Roadmap](#roadmap)
- [Changelog](#changelog)


---

## Year-In-Review

# 1年間の成果サマリ（Year in Review）

本ページは、icart_mini_ros2 の過去1年間の取り組みを振り返る報告用サマリです。

## ハイライト

### 走行
YP-Spur ブリッジが `/cmd_vel` → 走行を実現し、`/odom`・`/joint_states`・TF を配信。URDF+RViz で動作を可視化。  
<img src=../imgs/logi.png width=20%> + <img src=../imgs/icart_mini.png width=20%> + <img src=../imgs/icart_urdf.png width=20%>  
<img src="../videos/ypspur.gif" width="40%">

### センサ
Hokuyo UST-10LX を [urg_node2](https://github.com/Hokuyo-aut/urg_node2)（北陽電機公式 ROS 2 ドライバ）で接続し、/scan 配信や RViz 可視化、バッグ記録の手順を整備。   
<img src=../imgs/icart_rviz.png width=40%>

### 基板
24V→5V電源、インジケータ、電圧計測、ブザーを備えた Raspberry Pi 拡張ボードを活用。  
<img src=../imgs/kicad.png width=40%>

### 追従
LiDAR 脚クラスタ検出・ID追跡・Follow-me を実装。  
<img src="../videos/follow-me_sim.gif" width="50%">  
<img src="../videos/follow-me_real.gif" width="50%">  

### Docker / 開発環境
ロボット搭載 Raspberry Pi 上で Docker により環境を固定化し、別PCから同一設定で操作・可視化可能に整備。
![Docker Development Flow](../imgs/docker_overview.png)
1. **ロボット搭載の Raspberry Pi（ホスト）** が `docker run` を実行し、USBデバイス(`/dev/ttyACM0` 等)や X11/Wayland をコンテナへ共有。
2. **Docker Engine（Raspberry Pi 上）** が ROS 2 Humble コンテナを起動し、`icart_mini_ros2` と依存を固定。
3. **Dockerコンテナ内部** では `icart_mini_bringup` / `icart_mini_ypspur_bridge` / `icart_mini_leg_tracker` / `teleop_twist_joy` / `rviz` 等が動作。
4. **センサ・モータ系（LiDAR / YP-Spur / エンコーダ）** は USB パススルーで連携し、制御指令と計測データを ROS トピックに反映。
5. **別PCやタブレット** は WiFi 越しに `docker exec` / SSH / GUI 転送で接続し、rviz表示やデバッグ、ログ取得を同一環境で再現。


## 課題
- 障害物が多い環境下での追従性向上、主に乗り移り対策
- 充電回路、バッテリー電圧計測回路の修正

## 所感（開発メモ）
- 困ったことは特になし（安定して開発・運用）
- 初期に会社PCでYP-Spurを動かそうとした際に“煙くさい”事象があり要注意（以後は問題なし）
- 以降は安定して動作。急な暴走などはなし（感謝）
- 家の中で開発する分にはちょうどよい大きさで取り回しが良い
- 大学生のときに欲しかった、研究室ではKobukiというお掃除ロボットみたいなロボットを使用しており、使い勝手はいまいちだった。

## 1年のタイムライン

| 期間 | 四半期 | 主な取り組み |
|---|---|---|
| 2024-11-21 ~ 2025-02-18 | Q1 | 環境整備（Docker/依存パッケージ）、YP-Spur ブリッジ導入、Bringup/URDF/RViz の基盤整備 |
| 2025-02-20 ~ 2025-03-09 | Q2 | LiDARクラスタリング・トラッキング実装、Follow target/PID 制御の初期版 |
| 2025-03-10 ~ 2025-04-30 | Q3 | ログ/可視化/メッセージ拡充、ロスト復帰や選定ロジックの強化、安定化 |
| 2025-05-01 ~ 2025-09-23 | Q4 | パラメータチューニング（BLDC/DC切替含む）、追従性改善、各種バグ修正と最終調整 |


---

## Roadmap

# 今後の計画（Roadmap）

## 改善候補（短期）
- 追従の安定化：IDスワップ時の復帰判定強化（近傍履歴/相関）
- 追従選定ロジック：前回対象との整合性とロボット近傍の重み付け最適化
- パラメータ外だし：クラスタ/追従/速度制限を YAML 化
- ログ整備：計測スクリプトと可視化ノートブックの追加
- 充電回路の追加（安全機構・BMS連携・コネクタ設計）

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

