#!/bin/sh
set -eu
. /opt/ros/noetic/setup.sh
[ -f /root/catkin_ws/devel/setup.sh ] && . /root/catkin_ws/devel/setup.sh
mkdir -p /root/catkin_ws/maps
while true; do
  ts=$(date +%Y%m%d_%H%M%S)
  rosrun map_server map_saver -f /root/catkin_ws/maps/auto_latest >/tmp/map_save_latest.log 2>&1 || true
  cp /root/catkin_ws/maps/auto_latest.pgm /root/catkin_ws/maps/auto_$ts.pgm 2>/dev/null || true
  cp /root/catkin_ws/maps/auto_latest.yaml /root/catkin_ws/maps/auto_$ts.yaml 2>/dev/null || true
  sleep 60
done
