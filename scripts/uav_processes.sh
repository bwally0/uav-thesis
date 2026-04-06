#!/usr/bin/env bash
set -euo pipefail

XRCE_SESSION="uxrce_agent"
PX4_SESSION="px4_sitl"
AIRSIM_SESSION="airsim_ros"

start() {
    echo "Starting all processes..."
    screen -S "$XRCE_SESSION" -dm bash -lc 'echo "Starting uXRCE-DDS"; MicroXRCEAgent udp4 -p 8888; exec bash'
    sleep 2
    screen -S "$PX4_SESSION" -dm bash -lc 'echo "Starting PX4 SITL"; cd ~/PX4-Autopilot; make px4_sitl none_iris; exec bash'
    sleep 2
    screen -S "$AIRSIM_SESSION" -dm bash -lc 'source ~/Cosys-AirSim/ros2/install/setup.bash >/dev/null 2>&1 || true; echo "Starting AirSim ROS2 wrapper"; ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=192.168.128.1; exec bash'
    echo "Sessions started. Run 'screen -ls' to view."
}

stop() {
    echo "Stopping all processes..."
    screen -S "$XRCE_SESSION" -X quit 2>/dev/null || echo "uXRCE-DDS not running"
    screen -S "$PX4_SESSION" -X quit 2>/dev/null || echo "PX4 SITL not running"
    screen -S "$AIRSIM_SESSION" -X quit 2>/dev/null || echo "AirSim ROS not running"
    echo "All sessions stopped"
}

restart() {
    stop
    sleep 2
    start
}

status() {
    echo "Active screen sessions:"
    screen -ls | grep -E "uxrce_agent|px4_sitl|airsim_ros" || echo "No sessions found"
}

case "${1:-start}" in
    start)   start ;;
    stop)    stop ;;
    restart) restart ;;
    status)  status ;;
    *)       echo "Usage: $0 {start|stop|restart|status}"; exit 1 ;;
esac
