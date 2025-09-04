# Development Guide

## Build Scripts
Use `./scripts/build_nodes.sh <mode>` where `<mode>` can be:
| Mode | Purpose |
| --- | --- |
| (empty) | Full build (protocol + examples + sim if present) |
| `sim` | Simulation related packages |
| `example` | Interface examples only |

Script performs environment checks and invokes `colcon build`.

## Adding a C++ Node
1. Create `src/interface_example/src/my_node.cc`
2. Add executable to `interface_example/CMakeLists.txt` using `add_executable` + `ament_target_dependencies`
3. Install via `install(TARGETS ...)`
4. Rebuild: `./scripts/build_nodes.sh example`

## Adding a Python Script Node
1. Place script in `interface_example/scripts/` with shebang `#!/usr/bin/env python3`
2. Make it executable: `chmod +x`
3. CMake auto-installs (pattern already present) creating both `.py` and extensionless launcher names using RENAME.
4. Rebuild & run: `ros2 run interface_example <script_name>`

## Messages / Services
Add `.msg` / `.srv` files under `interface_protocol/msg` or `srv` then rebuild. Avoid renaming existing message fields without versioning.

## Configuration & Models
YAML under `interface_example/config/pm01/...` supply parameters & RL policy file paths. Parameter classes live in `parameter/` (e.g., `rl_basic_param.*`). Update YAML then restart node (dynamic reload not yet implemented).

## Simulation Development
* Launch MuJoCo: `ros2 launch mujoco_simulator mujoco_simulator.launch.py`
* Shared topics with real robot allow exercising identical control nodes.
* Keep physics rate consistent when benchmarking policies.

## RL Policy Replacement
Replace `.mnn` file in `policies/` directory, keeping filename or adjusting config/launch param referencing it. Validate dimensions (observations, actions) match expected in node code.

## Coding Standards
* C++: Follow existing style + `.clang-format`
* Python: Use LF endings (enforced by `.gitattributes`)
* Avoid trailing whitespace; keep functions small & focused

## Testing (Recommended)
Add unit style tests (if future test framework added) for deterministic math utilities (e.g., rotation matrices) and parameter parsing.

## Performance Considerations
* Minimize per-callback dynamic allocations
* Use pre-allocated Eigen structures in hot loops
* Prefer QoS sensor data best effort for high rate topics

## Logging & Diagnostics
Use `RCLCPP_INFO/WARN/ERROR_THROTTLE` for high rate status messages. For structured debugging consider adding a lightweight diagnostics topic.

## Future Tooling
Planned additions:
* Headless sim flag in launch
* Policy hot-reload service
* Basic unit test harness
* CI lint + colcon build pipeline
