#!/bin/bash

DOCKER_IMAGE="icart_mini_navigation:latest"
CONTAINER_NAME="icart_mini_navigation"

docker run --rm -it \
    --env DISPLAY=localhost:11.0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority:ro \
    --volume="${HOME}/ros2_ws:/root/ros2_ws" \
    --name=$CONTAINER_NAME \
    -e TZ=Asia/Tokyo \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash