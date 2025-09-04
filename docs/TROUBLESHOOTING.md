# Troubleshooting

## Environment / Build
| Symptom | Cause | Fix |
| --- | --- | --- |
| `command not found: ros2` | Not sourced ROS2 | `source /opt/ros/humble/setup.bash` |
| Package not found | Not sourced workspace | `source install/setup.bash` |
| Python script not executable | Missing shebang or permissions | Ensure `#!/usr/bin/env python3` + `chmod +x` |
| Wrong Python line endings | CRLF from Windows | `.gitattributes` enforces LF; run `dos2unix` |
| Third party .so load error | Missing extraction | Re-run `third_party/install.sh` or untar archive |

## Runtime (Real Robot)
| Symptom | Cause | Fix |
| --- | --- | --- |
| No feedback topics | Network/firewall/domain id mismatch | Check `ROS_DOMAIN_ID`, ping IPs |
| Velocity cmd ignored | Wrong mode | Ensure motion_state != `joint_bridge` & motors enabled |
| Joint commands inert | Not in joint_bridge | Transition using controller combo |
| High latency echo | Wireless or USB NIC | Use built-in Ethernet |

## Simulation
| Symptom | Cause | Fix |
| --- | --- | --- |
| Multiple windows | Multiple launches | Close extras / ensure single launch |
| GUI fails on headless server | Missing display | (Planned) headless flag; else use XVFB |

## Offline Deployment
| Symptom | Cause | Fix |
| --- | --- | --- |
| `package not found` after transfer | Not sourced `install/` | `source ~/engineai_ros2_workspace/install/setup.bash` |
| Policy file not found | Path mismatch | Update YAML / launch param |
| ABI mismatch errors | Different ROS2 build | Rebuild with same distro libs |

## Diagnostics Commands
```bash
ros2 topic list
ros2 topic echo /motion/motion_state
ros2 node list
ros2 interface show interface_protocol/msg/JointState
```

## Logging Tips
Use `RCLCPP_INFO_THROTTLE` for high rate callbacks; avoid printing large matrices every cycle.

## Still Stuck?
Gather: output of `env | grep ROS_`, `ros2 doctor`, list of launched nodes. Open an issue with logs & reproduction steps.
