#!/bin/bash

DOCKER_IMAGE="icart_mini_ros2:latest"
CONTAINER_NAME="icart_mini_ros2"

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

