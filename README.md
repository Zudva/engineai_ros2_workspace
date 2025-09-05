# EngineAI ROS Workspace

<img src="docs/images/pm01.jpg" alt="EngineAI Robot" width="600">

English | [Русский кратко](#russian-summary)

## Purpose
Modular ROS2 (Humble) workspace for EngineAI robot research & application:
* High‑level velocity control (publish body velocity commands)
* Low‑level joint / RL policy control (publish joint commands)
* MuJoCo based simulation for offline development & sim2sim
* Offline packaging & deployment to air‑gapped robots

## Quick Start
For a full walkthrough read: `docs/QUICKSTART.md`. Minimal host setup:
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
   rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
echo -e '\nexport ROS_DOMAIN_ID=69\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
git clone <this repo>
cd engineai_ros2_workspace
./src/third_party/install.sh   # fetch / build vendor libs
./scripts/build_nodes.sh sim   # build simulation + protocol
source install/setup.bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

High‑level body velocity example (real robot network connected):
```bash
./scripts/build_nodes.sh example
source install/setup.bash
ros2 run interface_example body_velocity_control_example
```

Offline deploy? See: `docs/OFFLINE_DEPLOYMENT.md`.

## Documentation Index
| Doc | Description |
| --- | --- |
| `docs/QUICKSTART.md` | Fast host setup & first runs |
| `docs/ARCHITECTURE.md` | Hardware / software layers, data flow |
| `docs/DEVELOPMENT.md` | High / low level dev, simulation, RL policy swapping |
| `docs/SAFETY.md` | Safety checklist & FSM usage |
| `docs/TROUBLESHOOTING.md` | Common issues & fixes |
| `docs/OFFLINE_DEPLOYMENT.md` | Air‑gapped packaging & install |
| `docs/CHANGELOG.md` | Versioned changes & Unreleased items |
| `src/interface_protocol/README.md` | ROS2 interface topics & message schema |
| `docs/CONTRIBUTING.md` | Contribution & branch workflow |

## Key Packages
* `interface_protocol`: Message & service definitions + shared protocol README.
* `interface_example`: C++ & Python examples, RL basic example, config & models.
* `simulation/mujoco`: MuJoCo integration + launcher.
### Simulator Parameters
Launch examples:
```bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py base_height:=1.1
```
Parameters (current):
* `base_height` – spawn floating base z (manual until contact alignment improves).

* `third_party`: Vendor libs bootstrap script.

## Finite State Machine (Modes)
Joystick → (High‑level) Basic Walk → (Low‑level) Joint Bridge. See `docs/SAFETY.md` for the annotated FSM image and activation steps.
<img src="docs/images/FSM.jpg" alt="FSM" width="600">

## Monitoring
PlotJuggler layout: `src/interface_protocol/pm_data_layout.xml`
```bash
colcon build --packages-select interface_protocol
source install/setup.bash
ros2 run plotjuggler plotjuggler -n
```

## Firmware
Latest firmware: https://github.com/engineai-robotics/engineai_firmware

## Contributing
Follow `docs/CONTRIBUTING.md`. Create feature branch, add docs/tests where relevant, open PR.

## License
See `LICENSE`.

## Russian Summary
<a id="russian-summary"></a>
Основное:
1. Установи ROS2 Humble, клонируй репозиторий
2. `./scripts/build_nodes.sh sim` + `source install/setup.bash`
3. Запусти симуляцию: `ros2 launch mujoco_simulator mujoco_simulator.launch.py`
4. Для реального робота: собери примеры и запусти `ros2 run interface_example body_velocity_control_example`
5. Офлайн деплой: см. `docs/OFFLINE_DEPLOYMENT.md`

Безопасность и переходы режимов: см. `docs/SAFETY.md`.
