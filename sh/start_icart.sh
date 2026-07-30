#!/bin/bash
set -e

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <network-interface> <remote-pc-ip>"
  echo "Example: $0 wlan0 192.168.0.110"
  exit 2
fi

# ブート直後の余裕を少し持たせる
sleep 5

export PATH="$HOME/.local/bin:$PATH"

cd ~/icart_ws/src/icart_mini_ros2/docker
./run.sh /root/icart_ws/src/icart_mini_ros2/sh/auto_start.sh "$@"
