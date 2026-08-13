#!/bin/bash
set -e  # エラーが発生したらスクリプトを終了

# ROS 2 の環境変数をセット
source /opt/ros/humble/setup.bash
if [ -f ~/icart_ws/install/setup.bash ]; then
    source ~/icart_ws/install/setup.bash
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

# YP-Spur is built explicitly in the mounted workspace so the bringup launch
# can use /root/icart_ws/build/ypspur-coordinator.

# コンテナが実行するコマンドを引き継ぐ
exec "$@"
