# 人物追従シミュレーション

Gazebo Fortress上の人物Actorを使い、i-Cart miniの人物追従を同じ条件で繰り返しデバッグする手順をまとめます。初回セットアップとDockerイメージの作成は[README](../README.md)を参照してください。

## 起動

Dockerコンテナへ入り、ワークスペースを読み込みます。

Actor 版 launch を使う場合だけ、ホスト側で `gazebo_ros_actor_plugin` を追加取得してからコンテナ内でビルドします。

```bash
cd ~/icart_ws
vcs import src < src/icart_mini_ros2/ros2_sim.repos
```

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gazebo_ros_actor_plugin icart_mini_description icart_mini_leg_tracker
source install/setup.bash
```

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source install/setup.bash
```

基準シナリオを起動します。

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py
```

Gazebo、RViz、人物Actor、LiDAR、追従ノードが起動します。起動から6秒後に追従と人物の移動が自動で始まり、人物は10秒間直進します。ジョイスティック操作は不要です。

別のシナリオを指定する場合:

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py \
  scenario:=leg_like_pillars_corridor
```

利用できる引数と選択肢を確認する場合:

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py --show-args
```

## デバッグシナリオ

| シナリオ | 確認内容 |
| --- | --- |
| `baseline_same_motion_no_pillars` | 障害物のない環境での直進を基準動作として確認 |
| `leg_like_pillars_corridor` | 脚に似た柱が左右に並ぶ通路を直進 |
| `leg_like_pillars_offset_corridor` | 左右で位置をずらした柱の間を直進 |
| `obstacle_orbit` | 障害物付近を左回りに周回 |
| `wide_corridor_orbit` | 広い通路で左右の周回と直進を連続実行 |
| `diagonal_walk` | 障害物のない環境で斜めに移動 |
| `front_crossing` | ロボット前方を横切る動作 |
| `turning_forward_walk` | 旋回しながら前進する動作 |
| `diagonal_walk_pillars_corridor` | 柱通路内で斜めに移動 |
| `front_crossing_pillars_corridor` | 柱通路内でロボット前方を横切る動作 |
| `turning_forward_walk_pillars_corridor` | 柱通路内で旋回しながら前進 |
| `diagonal_walk_pillars_offset_corridor` | 左右非対称な柱通路内で斜めに移動 |
| `front_crossing_pillars_offset_corridor` | 左右非対称な柱通路内で前方を横切る動作 |
| `turning_forward_walk_pillars_offset_corridor` | 左右非対称な柱通路内で旋回しながら前進 |
| `dense_mixed_obstacles_slalom` | 密集した複数形状の障害物間をスラローム |
| `front_crossing_box_gate` | 箱型ゲート付近でロボット前方を横切る動作 |
| `rectangular_panel_near_pass` | 矩形パネルの近くを直進 |

人物の移動条件を変える場合は、`motion_start_delay`、`motion_linear_speed`、`motion_duration`、`motion_publish_rate`、`motion_turn_angular_speed`を指定します。`motion_scenario:=auto`では、選択したシナリオの既定動作を使います。

自動移動を止めて手動操作する場合:

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py \
  run_actor_motion:=false use_joy:=true
```

## デバッグ結果の確認

RVizでは次の情報を確認します。

- `/scan`: LiDARが検出した人物の脚と障害物
- `/leg_tracker/cluster_markers`: 検出したクラスタ
- `/leg_tracker/person_marker`: 選択中の追従対象
- `/leg_tracker/debug_markers`: 現在位置、予測位置、再捕捉ゲート、棄却候補、追従状態

トピックを直接確認する場合:

```bash
ros2 topic echo /leg_tracker/is_lost_target
ros2 topic echo /leg_tracker/cluster_infos
ros2 topic echo /leg_tracker/debug_markers
```

TFやセンサ入力に問題がある場合:

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /tf --once
```

`/tf`に`odom -> base_footprint`があり、RVizのFixed Frame `odom`にエラーがないことを確認してください。

### デバッグCSV

追従ノードは次のファイルを起動時に作り直します。

```text
icart_mini_leg_tracker/csv/cluster_tracking_log.csv
```

CSVにはクラスタの位置・速度に加えて、次の再捕捉情報が記録されます。

- 追従状態と一時ロストからの経過時間
- 前回ターゲット位置と予測位置
- 再捕捉判定の実行有無と選択したクラスタID
- 選択理由、移動距離、ロボットからの距離、角度差
- 候補を棄却した理由と判定時刻

複数条件を比較する場合は、次の試行を始める前にCSVを別名で保存してください。

## 追従状態とパラメータ

追従ノードは対象を一時的に見失っても、すぐに別のクラスタへ乗り移らず、予測位置の近傍から再捕捉を試みます。

```text
Idle
  -> WaitingInitial
  -> Tracking
       -> TemporarilyLost -> Tracking（再捕捉成功）
                          -> Lost（再捕捉タイムアウト）

Stopped / EmergencyStop
```

しきい値は次のファイルで管理します。

```text
icart_mini_leg_tracker/config/leg_cluster_tracking_params.yaml
```

主な調整項目:

| パラメータ | 用途 |
| --- | --- |
| `target_reacquire_timeout` | 一時ロストから完全なLostへ遷移するまでの猶予時間 |
| `predicted_gate_distance` | 予測位置から再捕捉候補までの最大距離 |
| `max_target_angle_jump` | ターゲット継続判定で許容する角度変化 |
| `max_target_distance_jump` | ターゲット継続判定で許容する距離変化 |
| `leg_pair_min_distance` / `leg_pair_max_distance` | 二脚ペアとして許容する脚間距離 |
| `leg_pair_center_gate_distance` | 脚ペア中心と予測位置の最大距離 |
| `leg_pair_max_lateral_distance` | 柱をsecond脚と誤認しないための横方向上限 |
| `safety_stop_distance` | 人物または障害物への接近時に停止する距離 |

別の設定ファイルを使う場合:

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py \
  tracker_params_file:=/path/to/leg_cluster_tracking_params.yaml
```

全パラメータと既定値は、設定ファイル内のコメントを参照してください。

## 人物ActorとLiDAR検出

Gazeboには`DoctorFemaleWalk`の人物Actorを表示します。ただし、Fortressの`gpu_lidar`でActorのmeshを直接検出すると、歩行アニメーションに依存してscan形状が不安定になります。そのため、Actorの左右足に透明な円柱状の脚プロキシを同期させ、LiDARはこのプロキシを検出します。

脚プロキシは人物Actorを置き換える表示モデルではなく、LiDAR入力を再現可能にするための内部モデルです。通常は`proxy_visual:=hidden`のまま使用します。位置関係をGazebo上で確認するときだけ、次のように表示します。

```bash
ros2 launch icart_mini_leg_tracker \
  follow_me_actor_debug_scenario_fortress.launch.py \
  proxy_visual:=debug
```

## rosbag

シミュレーション中に別ターミナルから記録します。

```bash
docker exec -it icart_mini_ros2 bash
cd /root/icart_ws
source install/setup.bash
ros2 run icart_mini_leg_tracker record_follow_me_bag.sh
```

出力先を指定する場合:

```bash
ros2 run icart_mini_leg_tracker record_follow_me_bag.sh \
  /root/icart_ws/src/icart_mini_ros2/icart_mini_leg_tracker/rosbag/test_run
```

LiDAR入力を再生して追従ノードを確認する例:

```bash
ros2 run icart_mini_leg_tracker leg_cluster_tracking_node \
  --ros-args --param use_sim_time:=true

ros2 bag play /path/to/bag --clock --rate 0.5 \
  --topics /scan /tf /tf_static /joy /person/cmd_vel /person/control
```

## トラブルシュート

### Gazebo GUIやRVizが表示されない

ホスト側でX11の許可を確認します。

```bash
xhost +local:docker
```

### Gazeboが起動しない

前回のプロセスが残っている場合があります。

```bash
pkill -f "gz sim"
pkill -f "ign gazebo"
```

### RVizにLiDARやマーカーが表示されない

`/clock`、`/scan`、`/tf`がpublishされていることと、すべてのノードで`use_sim_time`が有効になっていることを確認します。

```bash
ros2 param get /leg_cluster_tracking_node use_sim_time
ros2 topic hz /scan
ros2 topic echo /tf --once
```

### CycloneDDSのネットワーク警告が出る

`wlan0: does not match an available interface`と表示された場合は、コンテナ内のインターフェース名と`cyclonedds.xml`の設定を合わせてください。

```bash
ip addr
```
