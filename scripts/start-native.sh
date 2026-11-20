#!/usr/bin/env bash
# Starts the robot without Docker (root, for GPIO/PWM access):
#   sudo bash scripts/start-native.sh [launch arguments ...]
# Robot settings (ROS_DOMAIN_ID, UGV_ARGS) are read from /etc/default/ugv.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OWNER_HOME="$(getent passwd "$(stat -c %U "$REPO")" | cut -d: -f6)"

[ -f /etc/default/ugv ] && set -a && source /etc/default/ugv && set +a
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/ugv}"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
export PYTHONPATH="$OWNER_HOME/ugv-gpio-venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  UGV_ARGS=${UGV_ARGS:-}"
# shellcheck disable=SC2086
exec ros2 launch ugv_bringup robot.launch.py ${UGV_ARGS:-} "$@"
