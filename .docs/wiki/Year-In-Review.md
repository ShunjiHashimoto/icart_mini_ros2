# 1年間の成果サマリ（Year in Review）

本ページは、icart_mini_ros2 の過去1年間の取り組みと到達点を振り返る報告用サマリです。コミュニティ共有や関係者説明、GitHub Wiki 掲載を前提に構成しています。

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
