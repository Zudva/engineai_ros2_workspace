#!/usr/bin/env python3
"""
Arm control example for MuJoCo simulation.

Publishes JointCommand messages to move only the arm joints (indices 13-17 left, 18-22 right) and head (23) while holding
other joints at their current positions taken from /hardware/joint_state. Designed to run WITHOUT the rl_basic_example node
(active simultaneous publishers will fight each other). If RL must run, extend this node to merge commands instead of overriding.

Joint index mapping (actuator order):
 0-5  : Left leg (hip pitch, hip roll, hip yaw, knee pitch, ankle pitch, ankle roll)
 6-11 : Right leg
 12   : Waist yaw
 13-17: Left arm (shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, elbow yaw)
 18-22: Right arm (shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, elbow yaw)
 23   : Head yaw

Usage:
  1. Source ROS2 + workspace.
  2. Launch simulator: ros2 launch mujoco_simulator mujoco_simulator.launch.py
  3. Run this script: ros2 run interface_example arm_control_example.py (or python3 arm_control_example.py)

Modes:
  --mode hold   : Move to target static pose (default)
  --mode wave   : Periodic waving with right elbow yaw (J22)

You can adjust target pose via constants TARGET_LEFT / TARGET_RIGHT below.
"""

import math
import time
import argparse
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from interface_protocol.msg import JointState, JointCommand

# Indices (keep in sync with serial_actuators.xml)
LEFT_ARM_IDX = [13, 14, 15, 16, 17]
RIGHT_ARM_IDX = [18, 19, 20, 21, 22]
HEAD_IDX = 23
NUM_JOINTS = 24

# Target pose (radians) - modest values inside joint limits
TARGET_LEFT = {
    13: 0.3,    # shoulder pitch L
    14: 0.4,    # shoulder roll L
    15: 0.0,    # shoulder yaw L
    16: -1.0,   # elbow pitch L (bend)
    17: 0.2,    # elbow yaw L
}
TARGET_RIGHT = {
    18: 0.3,
    19: -0.4,
    20: 0.0,
    21: -1.0,
    22: -0.2,
}
TARGET_HEAD = {HEAD_IDX: 0.0}

# Gains (simple PD)
DEFAULT_STIFFNESS = 400.0
DEFAULT_DAMPING = 5.0

class ArmControlNode(Node):
    def __init__(self, mode: str):
        super().__init__('arm_control_example')
        self.mode = mode

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/hardware/joint_state',
            self.joint_state_cb,
            10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointCommand,
            '/hardware/joint_command',
            10
        )

        self.latest_state: JointState | None = None
        self.sent_first = False
        self.start_time = self.get_clock().now()

        # 200 Hz control (MuJoCo integrator is fast; 200 is enough here)
        self.timer = self.create_timer(0.005, self.control_loop)
        self.get_logger().info(f'Arm control node started (mode={mode}). Waiting for joint state...')

    def joint_state_cb(self, msg: JointState):
        self.latest_state = msg

    def build_base_command_arrays(self) -> List[float]:
        # Start from current measured positions; if not available yet, zeros.
        if self.latest_state and len(self.latest_state.position) == NUM_JOINTS:
            return list(self.latest_state.position)
        return [0.0] * NUM_JOINTS

    def apply_target_pose(self, positions: List[float]):
        for k, v in TARGET_LEFT.items():
            positions[k] = v
        for k, v in TARGET_RIGHT.items():
            positions[k] = v
        for k, v in TARGET_HEAD.items():
            positions[k] = v

    def apply_wave(self, positions: List[float], t: float):
        # Base pose
        self.apply_target_pose(positions)
        # Add waving motion on right elbow yaw (index 22) small sinus
        amp = 0.4
        freq = 0.5  # Hz
        positions[22] = TARGET_RIGHT[22] + amp * math.sin(2 * math.pi * freq * t)

    def control_loop(self):
        if self.latest_state is None:
            return

        positions = self.build_base_command_arrays()

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        if self.mode == 'wave':
            self.apply_wave(positions, t)
        else:
            self.apply_target_pose(positions)

        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        # Full-length arrays required by consumer (simulation expects size == num_total_joints)
        cmd.position = positions
        cmd.velocity = [0.0] * NUM_JOINTS
        cmd.feed_forward_torque = [0.0] * NUM_JOINTS
        cmd.torque = [0.0] * NUM_JOINTS
        cmd.stiffness = [0.0] * NUM_JOINTS
        cmd.damping = [0.0] * NUM_JOINTS

        # Activate gains only for controlled joints (arms + head), keep others passive (0) so we don't freeze legs
        active_indices = LEFT_ARM_IDX + RIGHT_ARM_IDX + [HEAD_IDX]
        for i in active_indices:
            cmd.stiffness[i] = DEFAULT_STIFFNESS
            cmd.damping[i] = DEFAULT_DAMPING

        # parallel_parser_type left default (0)
        cmd.parallel_parser_type = 0

        self.joint_cmd_pub.publish(cmd)

        if not self.sent_first:
            self.get_logger().info('First arm JointCommand published.')
            self.sent_first = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['hold', 'wave'], default='hold', help='Control mode')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = ArmControlNode(mode=args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
