#!/bin/bash

set -o pipefail

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"

# Define node lists for different hosts
# Nodes to build on example
EXAMPLE_NODES=(
    "interface_protocol"
    "rdk_framework"
    "interface_example"
)

# Nodes to build on Orin
APPLICATION_NODES=(
    "interface_protocol"
    "rdk_framework"
    "robot_manager"
    "sound_hub"
    "system_monitor"
)

SIM_NODES=(
    "mujoco_simulator"
    "interface_protocol"
    "interface_example"
)
TARGET_HOST="example"   # default node set
BUILD_TYPE="Release"    # default build type

# Simple arg parsing: first non-option = target set (example|app|sim), optional --build-type <type>
while [[ $# -gt 0 ]]; do
    case "$1" in
        example|app|sim)
            TARGET_HOST="$1"; shift ;;
        --build-type|-b)
            BUILD_TYPE="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [example|app|sim] [--build-type <Release|Debug|RelWithDebInfo>]";
            exit 0 ;;
        *)
            echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# Validate target host
if [[ "$TARGET_HOST" != "example" && "$TARGET_HOST" != "app" && "$TARGET_HOST" != "sim" ]]; then
    echo "Error: Invalid target host '$TARGET_HOST'"
    echo "Available hosts: example, app, sim"
    exit 1
fi

# Select the appropriate node list based on target host
if [[ "$TARGET_HOST" == "example" ]]; then
    NODES=("${EXAMPLE_NODES[@]}")
    echo "Building for example nodes"
elif [[ "$TARGET_HOST" == "app" ]]; then
    NODES=("${APPLICATION_NODES[@]}")
    echo "Building for application nodes"
elif [[ "$TARGET_HOST" == "sim" ]]; then
    NODES=("${SIM_NODES[@]}")
    echo "Building for simulation nodes"
fi

echo "Build type: $BUILD_TYPE"

# ---------------- Preflight: ensure ROS environment / colcon available ----------------
if ! command -v colcon >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        echo "colcon not found in PATH. Sourcing /opt/ros/humble/setup.bash ..."
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    fi
fi

if ! command -v colcon >/dev/null 2>&1; then
    cat <<'EOF'
Error: 'colcon' command not found.
You need a ROS 2 Humble build environment on this machine.
Install minimal dependencies (Ubuntu 22.04):
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions python3-rosdep \
         python3-vcstool build-essential cmake git wget python3-empy python3-ament-package \
         libyaml-cpp-dev libeigen3-dev
Or install full ROS 2 (desktop):
    sudo apt install -y ros-humble-desktop
Then run:
    sudo rosdep init || true
    rosdep update
Re-run this script afterwards.
EOF
    exit 1
fi

# Warn if workspace third_party libs might be missing
if [[ ! -d /opt/engineai_robotics_third_party/lib ]]; then
    echo "[WARN] third_party libraries not found at /opt/engineai_robotics_third_party/lib"
    echo "       If this is the first build on this host run: sudo bash third_party/install.sh"
fi

# Create packages argument for colcon build
PACKAGES_ARG=""
if [[ ${#NODES[@]} -gt 0 ]]; then
    PACKAGES_ARG="--packages-select ${NODES[*]}"
fi

# Create build directory if it doesn't exist
mkdir -p "$root_dir/build"

# Run colcon build with the selected options
echo "Running build with the following nodes: ${NODES[*]}"
cd "$root_dir" && \
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --build-base build \
    --install-base install \
    $PACKAGES_ARG

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "To use the built packages, run:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $root_dir/install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi
