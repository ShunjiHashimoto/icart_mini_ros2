#!/bin/bash
set -e  # エラーが発生したらスクリプトを終了

# ROS 2 の環境変数をセット
source /opt/ros/humble/setup.bash
if [ -f ~/icart_ws/install/setup.bash ]; then
    source ~/icart_ws/install/setup.bash
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

# Install YP-Spur inside this container when the mounted workspace provides it.
#if [ ! -f "/usr/local/include/ypspur.h" ] && [ -f "/root/icart_ws/src/yp-spur/CMakeLists.txt" ]; then
#    echo "Installing yp-spur..."
#    rm -rf /tmp/yp-spur-build
#    mkdir -p /tmp/yp-spur-build
#    cd /tmp/yp-spur-build
#    cmake /root/icart_ws/src/yp-spur
#    make -j"$(nproc)"
#    make install
#    ldconfig
#    cd /root/icart_ws/src
#fi

# コンテナが実行するコマンドを引き継ぐ
exec "$@"
