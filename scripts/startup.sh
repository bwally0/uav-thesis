#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$HOME/uav-thesis/drone_ws"

cd "$WS_DIR"

colcon build
source "$WS_DIR/install/setup.bash"

exec ros2 launch uav_control uav_control.launch.py