#!/bin/bash
# Offline environment verifier for robot "cold box" deployment.
# Run in the workspace root AFTER unpacking bundles.

set -eo pipefail

ROOT_DIR="$(pwd)"
FAILED=0

section() { echo -e "\n==== $* ====\n"; }
ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
err() { echo "[ERR] $*"; FAILED=1; }

section "1. Checking ROS2 installation"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  ok "/opt/ros/humble/setup.bash present"
else
  err "ROS2 Humble not found at /opt/ros/humble"
fi

section "2. Checking third_party libraries"
if [[ -d /opt/engineai_robotics_third_party/lib ]]; then
  ls /opt/engineai_robotics_third_party/lib | head -n 5 || true
  ok "third_party libs present"
else
  warn "third_party libs directory missing (/opt/engineai_robotics_third_party/lib)"
fi

section "3. Checking workspace install tree"
if [[ -d install && -f install/setup.bash ]]; then
  ok "install/ exists"
else
  err "install/ directory missing or incomplete"
fi

section "4. Sourcing environment (test)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Some ROS setup scripts expect unset vars; temporarily disable -u if active
  if set -o | grep -q 'nounset *on'; then NOUNSET_WAS_ON=1; set +u; fi
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || err "Failed to source /opt/ros/humble/setup.bash"
  if [[ -f install/setup.bash ]]; then
    # shellcheck disable=SC1091
    source install/setup.bash || err "Failed to source install/setup.bash"
  else
    warn "install/setup.bash missing"
  fi
  [[ -n "${NOUNSET_WAS_ON:-}" ]] && set -u
  if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 CLI available"
  else
    err "ros2 CLI not in PATH after sourcing"
  fi
fi

section "5. Checking key packages"
PACKS=(interface_protocol interface_example)
for p in "${PACKS[@]}"; do
  if ros2 pkg list | grep -q "^$p$"; then
    ok "Package $p found"
  else
    err "Package $p NOT found"
  fi
done

section "6. Checking shared libraries linkage (sample)"
BIN="install/interface_example/lib/interface_example/rl_basic_example"
if [[ -x "$BIN" ]]; then
  if ldd "$BIN" | grep -qi "not found"; then
    err "Missing shared libs for $BIN"
    ldd "$BIN" | grep -i "not found" || true
  else
    ok "All linked libs resolved for rl_basic_example"
  fi
else
  warn "Binary $BIN not found (maybe not built)"
fi

section "7. Summary"
if [[ $FAILED -eq 0 ]]; then
  echo "Environment verification PASSED"
else
  echo "Environment verification FAILED. See errors above." >&2
fi
exit $FAILED
