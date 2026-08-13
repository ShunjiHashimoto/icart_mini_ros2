#!/usr/bin/env bash

# Shared CycloneDDS validation and configuration for robot and remote RViz.

icart_is_ipv4() {
  local address="$1"
  local octets=()
  local octet
  local IFS=.

  read -r -a octets <<<"${address}"
  [ "${#octets[@]}" -eq 4 ] || return 1

  for octet in "${octets[@]}"; do
    [[ "${octet}" =~ ^[0-9]+$ ]] || return 1
    [ "$((10#${octet}))" -le 255 ] || return 1
  done
}

icart_interface_ipv4() {
  local interface="$1"

  if command -v ip >/dev/null 2>&1; then
    ip -4 -o address show dev "${interface}" | \
      awk 'NR == 1 {sub(/\/.*/, "", $4); print $4}'
    return
  fi

  if command -v ifconfig >/dev/null 2>&1; then
    ifconfig "${interface}" 2>/dev/null | \
      awk '$1 == "inet" {print $2; exit}'
    return
  fi

  echo "Neither ip nor ifconfig is installed." >&2
  return 1
}

icart_configure_dds() {
  local interface="$1"
  local peer_ip="$2"
  local local_ip

  if [[ ! "${interface}" =~ ^[[:alnum:]_.:-]+$ ]] || \
     [ ! -e "/sys/class/net/${interface}" ]; then
    echo "Network interface '${interface}' was not found." >&2
    if command -v ip >/dev/null 2>&1; then
      ip -br address 2>/dev/null || true
    elif command -v ifconfig >/dev/null 2>&1; then
      ifconfig -a 2>/dev/null || true
    fi
    return 1
  fi

  if ! icart_is_ipv4 "${peer_ip}"; then
    echo "Peer IP '${peer_ip}' is not a valid IPv4 address." >&2
    return 1
  fi

  local_ip="$(icart_interface_ipv4 "${interface}")"
  if [ -z "${local_ip}" ]; then
    echo "Network interface '${interface}' has no IPv4 address." >&2
    return 1
  fi

  if [ "${peer_ip}" = "${local_ip}" ]; then
    echo "Peer IP '${peer_ip}' is this PC's own address on ${interface}." >&2
    echo "Specify the other PC's address instead." >&2
    return 1
  fi

  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_DISCOVERY_SERVER
  export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface name=\"${interface}\"/></Interfaces></General><Discovery><Peers><Peer Address=\"${peer_ip}\"/></Peers></Discovery></Domain></CycloneDDS>"

  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "DDS interface=${interface} (${local_ip})"
  echo "DDS discovery peer=${peer_ip}"
}
