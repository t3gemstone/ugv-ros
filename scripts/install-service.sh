#!/usr/bin/env bash
# Installs the systemd service that starts the robot at boot:
#   sudo bash scripts/install-service.sh <robot-no>
# robot-no becomes ROS_DOMAIN_ID (1, 2, ...). Give each robot on the same network
# a different number; otherwise a command from one robot's UI drives the other too.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ID="${1:?usage: sudo bash scripts/install-service.sh <robot-no (1-101)>}"
if ! [[ "$ID" =~ ^[0-9]+$ ]] || [ "$ID" -gt 101 ]; then
  echo "robot-no must be a number between 0 and 101" >&2
  exit 1
fi
if [ "$(id -u)" -ne 0 ]; then
  echo "run with sudo" >&2
  exit 1
fi

if [ -f /etc/default/ugv ]; then
  sed -i "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ID/" /etc/default/ugv
else
  cat > /etc/default/ugv <<CONF
# UGV robot settings (read by scripts/start-native.sh).
# Every robot on the same network needs a different ROS_DOMAIN_ID.
ROS_DOMAIN_ID=$ID
# Extra launch arguments, e.g.: UGV_ARGS="line:=false" or
# UGV_ARGS="slam_mode:=localization map:=~/ugv_maps/salon nav:=true"
UGV_ARGS=""
CONF
fi

cat > /etc/systemd/system/ugv.service <<UNIT
[Unit]
Description=UGV ROS 2 robot and web UI
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash $REPO/scripts/start-native.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable --now ugv.service
echo "Service installed: ROS_DOMAIN_ID=$ID. Status: systemctl status ugv  Log: journalctl -u ugv -f"
