#!/bin/bash
set -e

# ブート直後の余裕を少し持たせる
sleep 5

export PATH="$HOME/.local/bin:$PATH"

cd ~/icart_ws/src/icart_mini_ros2/docker
./run.sh bash -lc "~/icart_ws/src/icart_mini_ros2/auto_start.sh"
