#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$HOME/uav-thesis/drone_ws"
START_SCRIPT="$WS_DIR/scripts/start_uav_control.sh"

cd "$WS_DIR"

colcon build --packages-select uav_control
source "$WS_DIR/install/setup.bash"

exec ros2 launch uav_control uav_control.launch.py