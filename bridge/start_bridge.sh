#!/bin/bash
exec >> /logs/bridge.log 2>&1

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
rosparam load /ros_bridge.yaml
while true; do
  echo "Starting parameter_bridge..."
  ros2 run ros1_bridge parameter_bridge
  echo "parameter_bridge crashed with exit code $?. Restarting in 2 seconds..." >&2
  sleep 2
done