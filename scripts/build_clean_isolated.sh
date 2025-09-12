#!/usr/bin/env bash
# Clean isolated build to avoid conda/python contamination
# Avoid set -u before sourcing ROS (some setup scripts reference unset vars)
set -e -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[INFO] Deactivating conda environment ${CONDA_DEFAULT_ENV:-} for clean ROS2 build" >&2
  echo "Run manually: conda deactivate (repeat until gone). Aborting." >&2
  exit 3
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2 Humble not found at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
# Temporarily disable nounset while sourcing
set +u || true
source /opt/ros/humble/setup.bash
set -u

echo "[STEP] Cleaning build artifacts" >&2
rm -rf build/ install/ log/

echo "[STEP] Building interface_protocol" >&2
colcon build --packages-select interface_protocol --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo "[STEP] Building interface_example & mujoco_simulator" >&2
colcon build --packages-select interface_example mujoco_simulator --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[DONE] Build finished. Source with: source install/setup.bash" >&2
