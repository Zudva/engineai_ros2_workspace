# Troubleshooting

Structured quick fixes for common problems across build, runtime, simulation, RL deployment, and offline packaging.

---
## 1. Environment / Build
| Symptom | Likely Cause | Fix / Command |
| --- | --- | --- |
| `command not found: ros2` | ROS2 not sourced | `source /opt/ros/humble/setup.bash` |
| Package not found | Workspace not sourced | `source install/setup.bash` |
| Python script not executable | Missing shebang / no +x | Add `#!/usr/bin/env python3` + `chmod +x file.py` |
| Python script runs but ros2 run not found | Not installed by CMake yet | Rebuild: `colcon build --packages-select interface_example` |
| CRLF shebang error (`python3\r`) | Windows line endings | `dos2unix script.py`; `.gitattributes` already enforces LF |
| Third-party `.so` load error | Not installed / wrong path | Re-run `src/third_party/install.sh` (with sudo) |
| `undefined reference` to Eigen / yaml-cpp | Missing find_package or link in CMake | Add to `find_package` + `ament_target_dependencies` |
| Extremely long build or stuck | Using conda python overlays | Deactivate conda, run clean build script |
| `ament_package` or `em` import error | Python path mismatch | Ensure system python, or install `python3-empy` / `python3-ament-package` |

### Clean Rebuild Recipe
```bash
rm -rf build/ install/ log/
colcon build --packages-select interface_protocol
source install/setup.bash
colcon build --packages-select interface_example mujoco_simulator
```

---
## 2. Networking / DDS
| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| No topics at all | Wrong `ROS_DOMAIN_ID` / not sourced | Export `ROS_DOMAIN_ID=69` both sides, re-source |
| Intermittent discovery | USB NIC or Wi-Fi interference | Use built-in Ethernet, avoid USB network adapters |
| Only local machine sees topics | `ROS_LOCALHOST_ONLY=1` still set | `unset ROS_LOCALHOST_ONLY` or set to `0` |
| Clock skew / TF issues | System time drift (no NTP) | Sync time (chrony/ntp) or manual `date` adjust |
| High DDS CPU usage | Too many reliable high-rate pubs | Use Best Effort QoS for high-rate sensor streams |

Quick check:
```bash
env | grep ROS_
ros2 topic list
ros2 node list
```

---
## 3. Runtime (Real Robot)
| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| No feedback topics | Network / domain mismatch | See Section 2 |
| Velocity command ignored | Wrong mode (still joystick or joint_bridge) | Ensure `motion_state` is high-level walk |
| Joint commands have no effect | Not in `joint_bridge` | Transition via controller combo (LB + CROSS_Y_LEFT) |
| Sudden stop after a few seconds | Command publisher died | Keep node alive or use systemd service |
| High latency `ros2 topic echo` | Wireless or congested link | Use wired Ethernet; reduce echo rate or use `ros2 topic hz` |
| Repeated emergency / motor disable | Safety triggers / watchdog | Check logs, ensure stable posture & no over-current |

Monitor:
```bash
ros2 topic echo /motion/motion_state
ros2 topic hz /hardware/joint_state
```

---
## 4. Simulation (MuJoCo)
| Symptom | Cause | Fix |
| --- | --- | --- |
| Multiple GUI windows | Launched twice / zombie process | Close extras; ensure one launch command running |
| Black window / no render | Missing GL / remote session headless | Use a machine with GPU / X; or set up headless (future flag) |
| Low FPS | CPU/GPU contention | Close other heavy apps; build in Release |
| Topics not visible | Forgot sourcing or different domain | Source install + set domain 69 |
| Robot reacts in sim but not hardware | Sim running only | Confirm not accidentally connecting to real network (set `ROS_LOCALHOST_ONLY=1`) |

---
## 5. RL Policy & Model Issues
| Symptom | Cause | Fix |
| --- | --- | --- |
| Node throws shape mismatch | Wrong observation/action dims | Ensure policy matches code expectations; replace correct `.mnn` |
| Model file not found | Path mismatch in YAML | Update config path or launch parameter |
| Slow inference | Debug build / no optimization | Rebuild Release; ensure third-party libs installed under `/opt` |
| NaN joint commands | Uninitialized observation or model error | Validate sensor inputs; add range checks before publish |

Add quick validation snippet (temporary) inside node to clamp outputs before sending to hardware.

---
## 6. Offline Deployment
| Symptom | Cause | Fix |
| --- | --- | --- |
| `package not found` | Not sourced `install/setup.bash` | Source again (order: ROS then workspace) |
| Missing `.so` after transfer | Incomplete tgz copy | Re-copy `third_party.tgz`; verify sha256 |
| Policy file missing | Not included in tar | Ensure build & packaging after adding model file |
| ABI mismatch error | ROS2 version differs | Rebuild on host matching robot ROS2 runtime |

### Integrity Verification
On host (before transfer):
```bash
sha256sum offline_bundle/*.tgz > offline_bundle/manifest.sha256
```
On robot (after copy):
```bash
cd ~/offline_bundle
sha256sum -c manifest.sha256
```

Only differing archives need re-transfer when updating.

---
## 7. Systemd Service Debug
| Symptom | Cause | Fix |
| --- | --- | --- |
| Service exits immediately | Missing environment sourcing | Wrap ExecStart in bash -c with both `source` commands |
| `ros2` not found in logs | ROS not sourced | Ensure `/opt/ros/humble/setup.bash` in ExecStart chain |
| Permission denied | Wrong user / file perms | Set `User=<non-root>` and `chmod +x` script |

Commands:
```bash
sudo journalctl -u engineai_example.service -e
sudo systemctl status engineai_example.service
```

---
## 8. Common CMake / Build Failures
| Error Snippet | Meaning | Remedy |
| --- | --- | --- |
| `Could NOT find yaml-cpp` | Missing dev package | `sudo apt install libyaml-cpp-dev` |
| `fatal error: Eigen/Dense` | Eigen headers absent | `sudo apt install libeigen3-dev` |
| `linker: cannot find -lMNN` | Third-party libs not located | Reinstall third_party; check `LD_LIBRARY_PATH` |
| `recipe for target failed` w/ missing symbol | Missing dependency link | Add to `ament_target_dependencies()` |

---
## 9. Diagnostics Command Set
```bash
ros2 doctor                 # Overall ROS environment check
ros2 topic list             # Current topics
ros2 topic info /motion/motion_state
ros2 interface show interface_protocol/msg/JointState
ros2 node list
ros2 topic hz /hardware/joint_state
env | grep -E 'ROS_|RMW'
```

---
## 10. Logging & Performance Tips
| Situation | Recommendation |
| --- | --- |
| High-rate callback logs spamming | Use `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, ...)` |
| Measuring latency | Time stamp outgoing header + compare receipt time |
| Large matrices printing | Avoid in realtime loop; print once at init |

---
## 11. Multi-Window MuJoCo Cause Explained
Launching twice (e.g., re-running command while old process not terminated) spawns additional GLFW contexts. Confirm single process with:
```bash
pgrep -a simulate
```
Kill extras: `kill <pid>`.

---
## 12. Headless / Remote Scenarios
Current repo lacks formal headless flag. Temporary workaround (may reduce stability):
```bash
XVFB_RUN=1 xvfb-run -s "-screen 0 1280x720x24" ros2 launch mujoco_simulator mujoco_simulator.launch.py
```
Planned: a launch argument to disable GUI initialization.

---
## 13. Quick Decision Tree
1. No topics? → Check domain + sourcing.
2. Topics ok, commands ignored? → Check motion_state.
3. Joint control inert? → Not in joint_bridge.
4. Build fails? → Missing deps / third_party install.
5. Offline run fails? → Source order + tar integrity.
6. Policy crash? → Dimension mismatch.

---
## 14. Still Stuck?
Collect:
```bash
env | grep -E 'ROS_|RMW'
ros2 doctor
ros2 topic list
ros2 node list
ros2 topic echo -n 5 /motion/motion_state
```
Provide logs + exact steps and open an issue / PR discussion.

