#!/bin/bash
# Package ROS2 + third_party + workspace install for offline deployment.
# Usage: ./scripts/package_runtime.sh [output_dir]
set -euo pipefail

ROOT_DIR="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
OUT_DIR="${1:-$ROOT_DIR/offline_bundle}"
mkdir -p "$OUT_DIR"

stamp() { echo "[package_runtime] $*"; }

if [[ ! -d /opt/ros/humble ]]; then
  echo "ROS2 Humble not found at /opt/ros/humble" >&2; exit 1; fi

if [[ ! -d /opt/engineai_robotics_third_party/lib ]]; then
  echo "WARNING: third_party libs not found at /opt/engineai_robotics_third_party (continuing)" >&2; fi

if [[ ! -d "$ROOT_DIR/install" ]]; then
  echo "Workspace not built (missing install/). Run colcon build first." >&2; exit 1; fi

STAMP_FILE="$OUT_DIR/manifest.txt"
date > "$STAMP_FILE"
echo "Ubuntu: $(. /etc/os-release && echo $PRETTY_NAME)" >> "$STAMP_FILE"
echo "Arch: $(uname -m)" >> "$STAMP_FILE"
echo "Workspace: $ROOT_DIR" >> "$STAMP_FILE"

stamp "Creating tarballs (this may take a few minutes)"

sudo tar czf "$OUT_DIR/ros2_humble.tgz" -C /opt ros/humble
if [[ -d /opt/engineai_robotics_third_party ]]; then
  sudo tar czf "$OUT_DIR/third_party.tgz" -C /opt engineai_robotics_third_party
fi
tar czf "$OUT_DIR/workspace_install.tgz" -C "$ROOT_DIR" install

cat > "$OUT_DIR/README_OFFLINE.txt" <<'EOF'
Offline bundle contents:
  ros2_humble.tgz              -> /opt/ros/humble
  third_party.tgz (optional)   -> /opt/engineai_robotics_third_party
  workspace_install.tgz        -> <workspace>/install

On robot (as root for /opt extraction):
  sudo mkdir -p /opt/ros /opt/engineai_robotics_third_party
  sudo tar xzf ros2_humble.tgz -C /opt
  [ -f third_party.tgz ] && sudo tar xzf third_party.tgz -C /opt
  mkdir -p engineai_ros2_workspace
  tar xzf workspace_install.tgz -C engineai_ros2_workspace

Then in shell:
  source /opt/ros/humble/setup.bash
  source engineai_ros2_workspace/install/setup.bash
EOF

stamp "Done. Files in $OUT_DIR"
