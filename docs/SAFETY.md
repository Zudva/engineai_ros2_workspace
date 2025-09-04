# Safety & Mode Transitions

ALWAYS ensure a clear area, stable power, operator readiness.

## Visual FSM
See image: `docs/images/FSM.jpg`.
Modes: Joystick (default) -> Basic Walk (high-level) -> Joint Bridge (low-level).

## Entering Modes
| Target Mode | Action | Confirmation |
| --- | --- | --- |
| Basic Walk | LB + A after motors enabled | Robot stands stably |
| Joint Bridge | LB + (CROSS_Y_LEFT) from pd-stand | `/motion/motion_state` = `joint_bridge` |

## High-Level Control Checklist
1. Robot in Basic Walk
2. Network OK (ping Nezha IP)
3. Source workspace
4. Run body velocity node

## Low-Level Control Checklist
1. Start from pd-stand
2. Transition to joint_bridge
3. Stand clear (≥1m)
4. Launch RL / joint command node
5. Monitor `/hardware/joint_state` for anomalies

## Emergency Actions
| Scenario | Action |
| --- | --- |
| Unexpected motion | Emergency stop / disable motors (LB+RB release) |
| Loss of control node | Stop node; FSM should revert gracefully |
| Robot falling | Cut power if safe / physical support |

## Topic Monitoring
```bash
ros2 topic echo /motion/motion_state
ros2 topic hz /hardware/joint_state
```

## Common Safety Pitfalls
| Issue | Mitigation |
| --- | --- |
| Launching low-level without joint_bridge | Always verify motion_state first |
| Running both sim & real simultaneously | Set `ROS_LOCALHOST_ONLY=1` during sim |
| Overlapping velocity + joint commands | Use only one control mode at a time |

## Logging
Throttle log output in high rate callbacks to avoid console flooding delaying operator reaction.

## Future Enhancements
* Automatic posture recovery sequence
* Watchdog node supervising command rates
