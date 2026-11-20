#!/usr/bin/env bash
# Native installation (without Docker) on the T3 Gemstone O1 (Ubuntu 24.04 ARM64).
#   cd ~/ugv-ros && bash scripts/install-native.sh
# Run as a normal user; the steps that need sudo ask for the password.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$HOME/ugv-gpio-venv"

echo '[1/5] ROS 2 apt source'
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  release=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest)
  url=$(python3 -c 'import json, sys; d = json.load(sys.stdin); print(next(a["browser_download_url"] for a in d["assets"] if a["name"].endswith(".noble_all.deb")))' <<<"$release")
  curl -fsSL "$url" -o /tmp/ros2-apt-source.deb
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
# Older T3 Gemstone O1 images lack noble-updates, which the ROS packages are built against.
if ! grep -RqsE '^deb .* noble-updates |^Suites:.*noble-updates' /etc/apt/sources.list /etc/apt/sources.list.d; then
  printf '%s\n' \
    'deb http://ports.ubuntu.com/ubuntu-ports noble-updates main universe multiverse' \
    'deb http://ports.ubuntu.com/ubuntu-ports noble-backports main universe multiverse' \
    'deb http://ports.ubuntu.com/ubuntu-ports noble-security main universe multiverse' \
    | sudo tee /etc/apt/sources.list.d/ugv-ubuntu-updates.list >/dev/null
fi

echo '[2/5] ROS 2 Jazzy and tools'
sudo apt-get update
sudo apt-get install -y --no-install-recommends ros-jazzy-ros-base python3-rosdep \
  python3-vcstool python3-colcon-common-extensions python3-venv python3-pip \
  build-essential cmake git curl

echo '[3/5] USB lidar driver (CP210x)'
# The minimal image lacks the module: the adapter shows up in lsusb but /dev/ttyUSB0 is not created.
if ! modinfo cp210x >/dev/null 2>&1; then
  driver="kernel-module-cp210x-$(uname -r)"
  if apt-cache show "$driver" >/dev/null 2>&1; then
    sudo apt-get install -y --no-install-recommends "$driver"
    sudo depmod -a
  else
    echo "WARNING: $driver not found; the USB lidar will not work (the GPIO lidar connection is not affected)." >&2
  fi
fi
sudo modprobe cp210x 2>/dev/null || true

echo '[4/5] libgpiod v2 (separate Python environment)'
# python3-libgpiod in apt is v1.6; reading the encoders needs the v2 event buffer.
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install --quiet 'gpiod>=2.2,<3'

echo '[5/5] Sources, dependencies and build'
cd "$REPO"
[ -d src/sllidar_ros2 ] || vcs import src < repos.yml
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
set +u
source /opt/ros/jazzy/setup.bash
set -u
# Sequential build with two jobs, to fit the board's memory.
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
echo
echo "Installation complete. To try it:  sudo bash $REPO/scripts/start-native.sh"
echo "To start it at boot:               sudo bash $REPO/scripts/install-service.sh <robot-no>"
