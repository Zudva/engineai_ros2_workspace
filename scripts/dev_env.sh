#!/usr/bin/env bash
# Unified environment setup for EngineAI workspace.
# Source this (do NOT execute) from your shell or add to ~/.bashrc:
#   source /absolute/path/to/engineai_ros2_workspace/scripts/dev_env.sh
# Idempotent: safe to source multiple times.

_WS_ROOT="$(cd "${BASH_SOURCE[0]}"/.. && pwd)"
# If script moved, adjust: we expect scripts/ directory.

# 1. Source ROS2 if not yet
if [ -z "$ROS_DISTRO" ]; then
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
  else
    echo "[dev_env] ROS2 Humble not found at /opt/ros/humble. Install first." >&2
  fi
fi

# 2. Domain and RMW defaults (override with existing values if set before)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-69}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# 3. Third-party libs path
export ENGINEAI_ROBOTICS_THIRD_PARTY=${ENGINEAI_ROBOTICS_THIRD_PARTY:-/opt/engineai_robotics_third_party}
if [ -d "$ENGINEAI_ROBOTICS_THIRD_PARTY/lib" ]; then
  case ":$LD_LIBRARY_PATH:" in
    *":$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:"*) :;;
    *) export LD_LIBRARY_PATH="$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:$LD_LIBRARY_PATH";;
  esac
fi

# 4. Workspace overlay (only if built)
if [ -f "$_WS_ROOT/install/setup.bash" ]; then
  source "$_WS_ROOT/install/setup.bash"
else
  echo "[dev_env] Warning: workspace not built yet. Run ./scripts/build_nodes.sh sim" >&2
fi

# 5. Convenience shortcuts
alias build_sim='(cd "$_WS_ROOT" && ./scripts/build_nodes.sh sim && source install/setup.bash)'
alias build_example='(cd "$_WS_ROOT" && ./scripts/build_nodes.sh example && source install/setup.bash)'
alias run_idle='ros2 launch mujoco_simulator mujoco_idle.launch.py base_height:=1.1'

# 6. Prompt hint (append once per shell)
if [[ -z "$ENGINEAI_SHELL_TAG" ]]; then
  export ENGINEAI_SHELL_TAG=1
  export PS1="[engai] $PS1"
fi

# 7. Status line
echo "[dev_env] EngineAI environment loaded (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, third_party=$ENGINEAI_ROBOTICS_THIRD_PARTY)"
