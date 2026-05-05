#!/bin/bash
set -euo pipefail

OUTPUT_DIR="${1:-/root/icart_ws/src/icart_mini_ros2/icart_mini_leg_tracker/rosbag/follow_me_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$(dirname "$OUTPUT_DIR")"

ros2 bag record \
  -o "$OUTPUT_DIR" \
  /scan \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /joy \
  /person/cmd_vel \
  /person/control \
  /person/motion_event \
  /leg_tracker/cluster_markers \
  /leg_tracker/cluster_centers \
  /leg_tracker/cluster_infos \
  /leg_tracker/person_marker \
  /leg_tracker/is_lost_target
