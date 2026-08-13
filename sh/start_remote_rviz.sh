#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <network-interface> <robot-ip>"
  echo "Example: $0 wlp3s0 192.168.0.111"
  echo
  ip -br address 2>/dev/null || true
  exit 2
fi

DDS_INTERFACE="$1"
ROBOT_IP="$2"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble."
  exit 1
fi

source /opt/ros/humble/setup.bash

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

if ! ros2 pkg prefix icart_mini_description >/dev/null 2>&1; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon was not found. Install python3-colcon-common-extensions first."
    exit 1
  fi
  echo "Building icart_mini_description for the first run..."
  cd "${WORKSPACE_DIR}"
  colcon build --symlink-install --packages-select icart_mini_description
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo "rmw_cyclonedds_cpp is not installed."
  echo "Install ros-humble-rmw-cyclonedds-cpp."
  exit 1
fi

source "${SCRIPT_DIR}/dds_config.sh"
icart_configure_dds "$DDS_INTERFACE" "$ROBOT_IP"

ros2 daemon stop >/dev/null 2>&1 || true

echo "Waiting for robot topics..."
ros2 daemon start >/dev/null 2>&1 || true
DISCOVERY_TIMEOUT="${DISCOVERY_TIMEOUT:-15}"
if [[ ! "${DISCOVERY_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "DISCOVERY_TIMEOUT must be a positive integer." >&2
  exit 1
fi

ROBOT_TOPICS_FOUND=false
ELAPSED=0
while [ "${ELAPSED}" -lt "${DISCOVERY_TIMEOUT}" ]; do
  TOPICS="$(ros2 topic list 2>/dev/null || true)"
  if grep -Eq '^/(joint_states|odom|scan)$' <<<"${TOPICS}"; then
    ROBOT_TOPICS_FOUND=true
    break
  fi
  sleep 1
  ELAPSED=$((ELAPSED + 1))
done

if [ "${ROBOT_TOPICS_FOUND}" != true ]; then
  echo "Robot topics were not discovered." >&2
  echo "Check that robot bringup uses this PC as its DDS peer." >&2
  exit 1
fi

echo "${TOPICS}"

echo "Starting RViz2..."
exec ros2 launch icart_mini_description icart_mini_display.launch.py
