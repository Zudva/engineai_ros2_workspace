#!/usr/bin/env bash
set -euo pipefail

echo "[third_party/install] Starting installation of third-party libs" >&2

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TAR_FILE="${SCRIPT_DIR}/engineai_robotics_third_party_libs.tar.gz"
TARGET_DIR="/opt"
DEST_DIR="/opt/engineai_robotics_third_party"

if [[ ! -f "$TAR_FILE" ]]; then
    echo "Error: Archive not found: $TAR_FILE" >&2
    exit 1
fi

if [[ $(id -u) -ne 0 ]]; then
    echo "Re-executing with sudo..." >&2
    exec sudo -E bash "$0" "$@"
fi

echo "Extracting to $TARGET_DIR ..." >&2
tar -xzf "$TAR_FILE" -C "$TARGET_DIR"

if [[ ! -d "$DEST_DIR/lib" ]]; then
    echo "Error: Destination $DEST_DIR/lib not found after extraction" >&2
    exit 2
fi

echo "Done. Libraries installed under $DEST_DIR" >&2
echo "You may add to ~/.bashrc: export ENGINEAI_ROBOTICS_THIRD_PARTY=$DEST_DIR" >&2
exit 0