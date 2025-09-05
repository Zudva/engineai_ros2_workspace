# Changelog

All notable changes to this workspace will be documented here. The format loosely follows Keep a Changelog.

## [Unreleased]
- Parameterize MuJoCo spawn base height (`base_height` launch arg) and simplify standing initialization.
- Added idle arm animation script (`idle_animation_example`).
- Introduced body velocity → simple gait bridge in simulator (hip pitch sinus modulation when `/motion/body_vel_cmd` active).
- Added fallback idle shoulder animation inside simulator when no velocity commands.
- Improved initialization safety (timestamp guard, mutex-protected `SetModelAndData`, removed duplicate `mj_forward`).
- Added PD default gains (stiffness=300, damping=5) for neutral stand when no joint commands are received.
- Added basic neutral pose construction from `joint_test.yaml` targets.
- Added `base_height` parameter & removed experimental auto ground alignment after reverting due to mesh/geom offset.
- Added `idle_animation_example` installation & parameters (amplitude, frequency, indices autodetect).

## [0.1.0] - 2025-09-05
### Added
- Initial public structure: `interface_protocol`, `interface_example`, `simulation/mujoco`, third party bootstrap.
- MuJoCo simulator launch file with environment variable setup.
- Body velocity publisher example.
- RL basic example skeleton (policy runner and parameter loading).
- Joint test and hold position examples.
- Offline deployment helper scripts & docs skeleton.

### Changed
- Consolidated initialization logs and troubleshooting coverage in README index.

### Security / Safety
- Added FSM diagram (`docs/images/FSM.jpg`) reference and safety checklist doc pointer.

### Known Issues
- Ground contact visual mismatch (visual mesh vs collision) — spawn height must be tuned manually with `base_height`.
- Gait currently affects limited joints; no balance controller.

---
Guidelines: When adding new packages or externally visible topics / parameters, append an entry under Unreleased. On release, move them under a dated version section.
