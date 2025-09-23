#!/bin/bash
set -e

CONTAINER_NAME="icart_mini_ros2"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Stopping container: ${CONTAINER_NAME}"
    docker stop "${CONTAINER_NAME}" || {
        echo "docker stop failed, forcing removal"
        docker rm -f "${CONTAINER_NAME}"
    }
else
    echo "Container ${CONTAINER_NAME} is not running."
fi
