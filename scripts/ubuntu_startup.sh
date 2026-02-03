#!/usr/bin/env bash

set -euo pipefail

XRCE_SESSION="uxrce_agent"
PX4_SESSION="px4_sitl"
AIRSIM_SESSION="airsim_ros"

screen -S "$XRCE_SESSION" -dm bash -lc 'echo "Starting uXRCE-DDS"; MicroXRCEAgent udp4 -p 8888; exec bash'
screen -S "$PX4_SESSION" -dm bash -lc 'echo "Starting PX4 SITL"; cd ~/PX4-Autopilot; make px4_sitl none_iris; exec bash'
screen -S "$AIRSIM_SESSION" -dm bash -lc 'source ~/Cosys-AirSim/ros2/install/setup.bash >/dev/null 2>&1 || true; echo "Starting AirSim ROS2 wrapper"; ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=192.168.128.1; exec bash'

echo "Sessions started:"
screen -ls