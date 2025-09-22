#!/bin/bash

DOCKER_IMAGE="icart_mini_ros2:latest"
CONTAINER_NAME="icart_mini_ros2"

# 既存コンテナが残ってたら削除
#if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
#  echo "Removing existing container: $CONTAINER_NAME"
#  docker rm -f "$CONTAINER_NAME"
#fi

docker run --rm -it \
    --env DISPLAY=localhost:11.0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority:ro \
    --volume="${HOME}/icart_ws:/root/icart_ws" \
    --name=$CONTAINER_NAME \
    -e TZ=Asia/Tokyo \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash




# bash -c "source /opt/ros/humble/setup.bash && source /root/icart_ws/install/setup.bash && ros2 launch tang_bringup tang_bringup.launch.py"
