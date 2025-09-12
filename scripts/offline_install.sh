#!/bin/bash
# Extract previously packaged offline bundle on target (no internet).
# Place tarballs in the same directory before running.
set -euo pipefail

SCRIPT_DIR="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd))"
cd "$SCRIPT_DIR"

bundle_dir="${1:-.}"
echo "[offline_install] Using bundle dir: $bundle_dir"

need_root() { if [[ $EUID -ne 0 ]]; then echo "Run with sudo for system paths." >&2; exit 1; fi; }

if [[ -f "$bundle_dir/ros2_humble.tgz" ]]; then
  echo "Extracting ros2_humble.tgz -> /opt"; need_root; mkdir -p /opt/ros; tar xzf "$bundle_dir/ros2_humble.tgz" -C /opt
else
  echo "ros2_humble.tgz not found (skipping)"; fi

if [[ -f "$bundle_dir/third_party.tgz" ]]; then
  echo "Extracting third_party.tgz -> /opt"; need_root; mkdir -p /opt; tar xzf "$bundle_dir/third_party.tgz" -C /opt
else
  echo "third_party.tgz not found (skipping)"; fi

if [[ -f "$bundle_dir/workspace_install.tgz" ]]; then
  echo "Extracting workspace_install.tgz -> ./install"; tar xzf "$bundle_dir/workspace_install.tgz" -C .
else
  echo "workspace_install.tgz not found"; exit 1; fi

cat <<EOF
Extraction complete.
Next steps:
  source /opt/ros/humble/setup.bash
  source $(pwd)/install/setup.bash
EOF
