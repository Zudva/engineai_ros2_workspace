# Architecture

## Hardware Overview
| Component | Role |
| --- | --- |
| Nezha (x86) | High‑frequency motion control, low-level data processing |
| Jetson Orin (ARM) | Application logic, perception, higher-level behaviors |
| Robot Bus / MCU | Motor drivers, sensors (IMU, joints) |

## Software Layers
```
+-----------------------------+           +-----------------------------+
|  User Nodes / RL Policies   |  <----->  |  Visualization / Monitoring |
+-----------------------------+           +-----------------------------+
|        Interface Example Nodes (C++/Py) |  body vel / joint cmds    |
+-----------------------------+-----------+-----------------------------+
|         Interface Protocol (msgs, srvs, QoS)                        |
+--------------------------------------------------------------------+
|  ROS2 Middleware (CycloneDDS, RMW)                                  |
+--------------------------------------------------------------------+
|  Vendor / Third Party Libs (MuJoCo, Eigen, MNN, YAML, GLFW, etc.)   |
+--------------------------------------------------------------------+
|  Ubuntu 22.04 + Kernel + Drivers                                   |
```

## ROS2 Topics (Excerpt)
Full table: `src/interface_protocol/README.md`.
Key flows:
* `/motion/body_vel_cmd` (publish high-level velocity command)
* `/hardware/joint_command` (publish low-level joint targets)
* `/hardware/joint_state`, `/hardware/imu_info` (feedback)
* `/motion/motion_state` (mode / FSM status)

## Modes & FSM
Modes transition: Joystick -> Basic Walk (high-level) -> Joint Bridge (low-level). Safety conditions: stable stance, clear area, personnel aware. See `SAFETY.md`.

## Simulation Path
User Node -> (same interface topics) -> MuJoCo Adapter -> Physics -> Feedback Topics.
Sim and real robot share identical messages → enables sim2sim policy transfer.

## RL Policy Loading
`interface_example/rl_basic_example` loads `.mnn` model defined in `config/pm01/rl_basic/...` (adjust path via launch parameters). Abstraction isolates policy inference from ROS messaging.

## Offline Deployment Insert Point
Runtime tarballs reproduce: `/opt/ros/humble`, `/opt/engineai_robotics_third_party`, workspace `install/` tree. No build required on robot unless native policy compilation changes.

## Extension Points
| Area | How |
| --- | --- |
| New message | Add `.msg` in `interface_protocol/msg`, run colcon build |
| New example node | Add source/script in `interface_example`, update CMake if C++ |
| New RL policy | Drop model file & adjust config YAML / launch param |
| Headless sim | (Planned) Add flag to MuJoCo launch to disable GUI |

## Data Rates & Considerations
High-rate (>500Hz) joint & IMU topics demand low jitter → prefer x86 for heavy subscribers. Use QoS defaults from protocol package; avoid unnecessary reliable QoS for high-rate sensor streams.

## Future Improvements
* Headless simulation service mode
* Systemd templates for auto-start examples
* Policy hot-reload via parameter event
* Hash manifest for offline bundles
