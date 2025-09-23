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
