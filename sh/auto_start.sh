#!/usr/bin/env bash
set -eo pipefail


export PATH="$HOME/.local/bin:$PATH"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/icart_ws/install/setup.bash"

[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

LOG_DIR="$HOME/icart_ws/log/auto_start"
mkdir -p "$LOG_DIR"

nohup ros2 launch icart_mini_bringup icart_mini_bringup.launch.py \
  >"$LOG_DIR/bringup.log" 2>&1 &

nohup ros2 run icart_mini_leg_tracker led_status.py \
  >"$LOG_DIR/led_status.log" 2>&1 &

nohup ros2 run icart_mini_leg_tracker leg_cluster_tracking_node \
  >"$LOG_DIR/leg_tracker.log" 2>&1 &

wait
