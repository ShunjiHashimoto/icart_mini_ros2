#!/usr/bin/env bash
set -eo pipefail


export PATH="$HOME/.local/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <network-interface> <remote-pc-ip>"
  echo "Example: $0 wlan0 192.168.0.110"
  exit 2
fi

DDS_INTERFACE="$1"
DDS_PEER="$2"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/icart_ws/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 2 Humble setup was not found: ${ROS_SETUP}"
  exit 1
fi
if [ ! -f "$WS_SETUP" ]; then
  echo "Workspace setup was not found: ${WS_SETUP}"
  exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo "rmw_cyclonedds_cpp is not installed."
  exit 1
fi

source "${SCRIPT_DIR}/dds_config.sh"
icart_configure_dds "$DDS_INTERFACE" "$DDS_PEER"

LOG_DIR="$HOME/icart_ws/log/auto_start"
mkdir -p "$LOG_DIR"

nohup ros2 launch icart_mini_bringup icart_mini_bringup.launch.py \
  >"$LOG_DIR/bringup.log" 2>&1 &

nohup ros2 run icart_mini_leg_tracker led_status.py \
  >"$LOG_DIR/led_status.log" 2>&1 &

nohup ros2 run icart_mini_leg_tracker leg_cluster_tracking_node \
  >"$LOG_DIR/leg_tracker.log" 2>&1 &

wait
