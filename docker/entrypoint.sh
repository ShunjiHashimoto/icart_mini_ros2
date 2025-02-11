#!/bin/bash
set -e  # エラーが発生したらスクリプトを終了

# ROS 2 の環境変数をセット
source /opt/ros/humble/setup.bash
source ~/icart_ws/install/setup.bash
export ROS_DOMAIN_ID=99

# `yp-spur` のビルド済みディレクトリがあるか確認して `make install`
if [ -f "/root/icart_ws/build/Makefile" ]; then
    echo "Installing yp-spur..."
    cd /root/icart_ws/build
    make install
    ldconfig
    cd /root/icart_ws/src
else
    echo "Warning: /root/icart_ws/build/Makefile not found. Skipping installation. Please build yp-spur at local."
fi

# コンテナが実行するコマンドを引き継ぐ
exec "$@"