#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/icart_ws/install/setup.bash
export ROS_DOMAIN_ID=99
# コンテナが実行するコマンドを引き継ぐ
exec "$@"