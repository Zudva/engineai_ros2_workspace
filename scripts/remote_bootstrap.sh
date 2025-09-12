#!/bin/bash
# Remote bootstrap script for first-time setup on a robot (x86 Ubuntu 22.04, ROS 2 Humble)
# Safe to re-run; skips installed components.

set -euo pipefail

WS_DIR="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
UBUNTU_CODENAME=$( . /etc/os-release && echo "$UBUNTU_CODENAME" )

echo "[remote_bootstrap] Workspace: $WS_DIR"

if [[ $UBUNTU_CODENAME != "jammy" ]]; then
  echo "[WARN] Expected Ubuntu 22.04 (jammy). Detected: $UBUNTU_CODENAME" >&2
fi

need_pkg() { dpkg -s "$1" >/dev/null 2>&1 || return 0; return 1; }

echo "[Step] Adding ROS 2 apt repository if missing"
if ! grep -q "packages.ros.org/ros2" /etc/apt/sources.list.d/*.list 2>/dev/null; then
  sudo apt update
  sudo apt install -y curl gnupg lsb-release
  sudo mkdir -p /etc/apt/keyrings
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros2.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

echo "[Step] Installing core packages"
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-empy python3-ament-package build-essential cmake git \
  libyaml-cpp-dev libeigen3-dev

echo "[Step] rosdep init/update"
sudo rosdep init 2>/dev/null || true
rosdep update || true

echo "[Step] Installing third_party libraries (if not present)"
if [[ ! -d /opt/engineai_robotics_third_party/lib ]]; then
  ( cd "$WS_DIR/third_party" && sudo bash install.sh )
else
  echo "  Skipping: /opt/engineai_robotics_third_party already exists"
fi

echo "[Step] Building workspace (example node set)"
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
./scripts/build_nodes.sh example || { echo "Build failed"; exit 1; }

cat <<EOF

Bootstrap complete.

To use this workspace now run:
  source /opt/ros/humble/setup.bash
  source "$WS_DIR"/install/setup.bash

Run an example node (e.g. body velocity publisher):
  ros2 run interface_example body_velocity_control_example

If you need the simulation on the robot (usually not necessary):
  ./scripts/build_nodes.sh sim

EOF