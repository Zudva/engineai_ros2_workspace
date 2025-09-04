# Quick Start

This guide gets you from clone to first motion (simulation + real robot) quickly.

## 1. Prerequisites (Ubuntu 22.04)
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
  rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
```

Append to ~/.bashrc (only once):
```bash
echo -e '\nexport ROS_DOMAIN_ID=69' >> ~/.bashrc
echo -e 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

## 2. Clone & Prepare
```bash
git clone <repo_url> engineai_ros2_workspace
cd engineai_ros2_workspace
./src/third_party/install.sh    # build / stage vendor libs
```

## 3. Build (Simulation + Protocol + Examples)
```bash
./scripts/build_nodes.sh sim
source install/setup.bash
```

## 4. Run MuJoCo Simulation
```bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```
(Use `Ctrl+C` to exit.)

## 5. High‑Level Body Velocity Example (Real Robot)
Network: ensure wired Ethernet to robot subnet (192.168.0.0/24) and robot powered & in Basic Walk mode.
```bash
./scripts/build_nodes.sh example
source install/setup.bash
ros2 run interface_example body_velocity_control_example
```

## 6. Low‑Level RL Example
1. Put robot into pd-stand, then joint bridge mode (see SAFETY).
2. Run:
```bash
ros2 launch interface_example rl_basic_example.launch.py
```

## 7. PlotJuggler Monitoring
```bash
colcon build --packages-select interface_protocol
source install/setup.bash
ros2 run plotjuggler plotjuggler -n
```
Layout: `src/interface_protocol/pm_data_layout.xml`

## 8. Offline Deployment
See `OFFLINE_DEPLOYMENT.md` for packaging & transfer.

## 9. Common Environment Issues
| Symptom | Fix |
| --- | --- |
| `command not found: ros2` | Source `/opt/ros/humble/setup.bash` |
| Package not found | `source install/setup.bash` |
| Topic echo empty | Check network, `ROS_DOMAIN_ID`, firewall | 
| Shared lib errors | Re-run `./src/third_party/install.sh` |

## 10. Next Steps
Read: `ARCHITECTURE.md`, `DEVELOPMENT.md`, `SAFETY.md`.
