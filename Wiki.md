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
- 走行: YP-Spur ブリッジで `/cmd_vel` → 走行、`/odom`・`/joint_states`・TF を配信
  - URDF（xacro）と RViz 構成を整理し可観測性を向上  
  <img src=../imgs/icart_mini.png width=20%> →→ 
  <img src=../imgs/icart_urdf.png width=40%>
- センサ: Hokuyo UST-10LX(会社から借りているもの) を urg_node2 で接続、RViz 可視化  
  <img src=../imgs/icart_rviz.png width=80%>
- 基板：別のロボットで使用していたRaspberry Pi拡張ボードを使用  
  - 24V->5Vの電源回路、インジケータ、電圧計測、ブザー
  <img src=../imgs/kicad.png width=80%>
- 追従: LiDAR 脚クラスタ検出・ID追跡・Follow-me（PID）を実装
- Docker/開発環境: 環境非依存・バージョン固定・別PCからの可視化・セットアップ短縮・切り分け容易


## 現状のシステム構成
- 起動: `icart_mini_bringup`
  - Docker での開発・実行環境整備（ビルドスクリプト、起動スクリプト）
- 表示/URDF: `icart_mini_description`
- ypspurのROS 2へのラッパ: `icart_mini_ypspur_bridge`
- 追従: `icart_mini_leg_tracker`

## 実装済み機能
- 速度指令 
  - `/cmd_vel` を YP-Spur 経由で走行系に反映
  - オドメトリ `/odom` と `/tf`（`odom`→`base_footprint`）の配信
  - ホイール状態 `/joint_states` の配信
- LiDAR 
  - `/scan` からのクラスタリング（PCL EuclideanClusterExtraction）
- クラスタ中心の推定、速度推定、ID一貫性維持、ロスト復帰
- Follow-me 制御（停止距離/角度・速度制限・PID・ジョイスティック非常停止）
- 可視化トピック（クラスタ点群・中心・対象）配信

## デモ・成果物
- 実走動画（屋内）: TBD
- 実走動画（屋外）: TBD


## 課題と学び
- 近接・遮蔽時のIDスワップ対策（履歴・速度予測のさらなる強化余地）
- 動的環境でのロバスト性（多人数、交差、停止/再開）

## 所感（開発メモ）
- 困ったことは特になし（安定して開発・運用）
- 初期に会社PCでYP-Spurを動かそうとした際に“煙くさい”事象があり要注意（以後は問題なし）
- 以降は安定して動作。急な暴走などはなし（感謝）
- 家の中で開発する分にはちょうどよい大きさで取り回しが良い

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

