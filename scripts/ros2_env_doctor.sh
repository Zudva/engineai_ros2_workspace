#!/usr/bin/env bash
# Quick diagnostic script for EngineAI ROS2 workspace
# Checks: ROS setup, Python packages, third-party libs, common pitfalls after using conda
set -e

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

ok() { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; }

echo "== EngineAI ROS2 Environment Doctor =="

# 1. ROS2 environment
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 found: $(command -v ros2)"
else
  fail "ros2 not in PATH. Did you 'source /opt/ros/humble/setup.bash'?"
fi

# 2. Python executable
PY_BIN=$(python3 -c 'import sys; print(sys.executable)' 2>/dev/null || true)
if [[ -n "$PY_BIN" ]]; then
  ok "Python: $PY_BIN"
else
  fail "Python3 not callable"
fi

# 3. ament_package & em (empy)
python3 - <<'EOF'
import importlib, sys
missing = []
for m in ["ament_package", "em"]:
    try:
        importlib.import_module(m)
    except Exception:
        missing.append(m)
if missing:
    print("__MISS__", ' '.join(missing))
EOF

if grep -q '__MISS__' <(python3 - <<'EOF'
import importlib
missing = []
for m in ["ament_package", "em"]:
    try: importlib.import_module(m)
    except Exception: missing.append(m)
print(' '.join(missing))
EOF
); then
  MISS=$(python3 - <<'EOF'
import importlib
missing = []
for m in ["ament_package", "em"]:
    try: importlib.import_module(m)
    except Exception: missing.append(m)
print(' '.join(missing))
EOF
)
  fail "Missing Python modules: $MISS (if using conda, add ROS site-packages or deactivate conda)"
else
  ok "ament_package & em present"
fi

# 4. ROS domain and RMW
[[ "$ROS_DOMAIN_ID" == "69" ]] && ok "ROS_DOMAIN_ID=69" || warn "ROS_DOMAIN_ID is '$ROS_DOMAIN_ID' (expected 69)"
[[ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" ]] && ok "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" || warn "RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION'"

# 5. Third-party libMNN
if [[ -f /opt/engineai_robotics_third_party/lib/libMNN.so ]]; then
  ok "libMNN.so found"
else
  fail "libMNN.so missing (/opt/engineai_robotics_third_party/lib/libMNN.so). Run src/third_party/install.sh"
fi

# 6. LD_LIBRARY_PATH contains third-party
if echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -q '/opt/engineai_robotics_third_party/lib'; then
  ok "LD_LIBRARY_PATH contains third-party lib path"
else
  warn "LD_LIBRARY_PATH missing /opt/engineai_robotics_third_party/lib"
fi

# 7. Workspace build artifacts
ROOT_DIR=$(realpath "$(dirname "$0")/..")
if [[ -d "$ROOT_DIR/build" ]]; then
  warn "build/ exists"
fi
if [[ -d "$ROOT_DIR/install" ]]; then
  warn "install/ exists"
fi

# 8. Check selected packages presence in source
for p in interface_protocol interface_example mujoco_simulator; do
  if [[ -d "$ROOT_DIR/src/$p" ]]; then
    ok "Source package '$p' present"
  else
    fail "Source package '$p' not found in src/"
  fi
done

echo "== Suggested next steps =="
echo "1. If Python modules missing: deactivate conda (run 'conda deactivate' until prompt clean)." \
     "Then 'source /opt/ros/humble/setup.bash' and rebuild." \
     "Or export PYTHONPATH to include ROS site-packages." \
     "2. Clean build: rm -rf build/ install/ log/ && colcon build --packages-select interface_protocol interface_example mujoco_simulator" \
     "3. Source: source install/setup.bash" \
     "4. Launch: ros2 launch interface_example rl_basic_example.launch.py"

echo "== Done =="
