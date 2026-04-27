source /opt/ros/humble/setup.bash
if [ -f /root/icart_ws/install/setup.bash ]; then
  source /root/icart_ws/install/setup.bash
fi
export ROS_DOMAIN_ID=99
if [ -e /sys/class/net/wlan0 ] && [ -f /root/icart_ws/src/cyclonedds.xml ]; then
  export CYCLONEDDS_URI=/root/icart_ws/src/cyclonedds.xml
else
  unset CYCLONEDDS_URI
fi
