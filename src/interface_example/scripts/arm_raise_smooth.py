#!/usr/bin/env python3
"""Smooth arm raise motion node.

Features:
- Select left or right arm via parameter 'side': 'left' or 'right' (default left)
- S-curve (smooth 5th-order polynomial) time scaling to minimize jerk
- Parameters:
    target_angle (rad)      default: 1.0
    duration (s)            default: 2.0 (active motion phase)
    hold_time (s)           default: 1.0 (pose hold then exit)
    stiffness               default: 350
    damping                 default: 5
- Arms assumed contiguous joint block consistent with existing ordering used in examples.
  Adjust ARM_START / ARM_LEN if your model differs.

Joint ordering (assumed from earlier scripts):
  0..5  : left leg
  6..11 : right leg
  12    : torso
  13..17: left arm (5 joints)
  18..22: right arm (5 joints)
  23    : head

We raise the 2nd joint in the arm block (index +1) which likely corresponds to a shoulder roll / lateral lift.
Modify RAISE_LOCAL_INDEX if needed.
"""
import math
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointState, JointCommand
from std_msgs.msg import Header

ARM_START_LEFT = 13
ARM_START_RIGHT = 18
ARM_LEN = 5
RAISE_LOCAL_INDEX = 1   # local arm joint to actuate (e.g., shoulder roll)

class ArmRaiseSmooth(Node):
    def __init__(self):
        super().__init__('arm_raise_smooth')
        self.declare_parameter('side', 'left')
        self.declare_parameter('target_angle', 1.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('stiffness', 350.0)
        self.declare_parameter('damping', 5.0)

        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.target_angle = float(self.get_parameter('target_angle').value)
        self.duration = float(self.get_parameter('duration').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.kp = float(self.get_parameter('stiffness').value)
        self.kd = float(self.get_parameter('damping').value)

        if self.side not in ('left','right'):
            self.get_logger().warn("Invalid side parameter, defaulting to 'left'")
            self.side = 'left'

        self.pub = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/hardware/joint_state', self.state_cb, 10)
        self.start_positions = None
        self.start_time = None
        self.done = False
        self.phase_complete = False
        self.timer = None
        self.get_logger().info('Waiting for first joint state...')

    def state_cb(self, msg: JointState):
        if self.start_positions is not None:
            return
        self.start_positions = list(msg.position)
        n = len(self.start_positions)
        if n < ARM_START_RIGHT + ARM_LEN:
            self.get_logger().error(f'Joint count {n} smaller than expected; aborting.')
            return
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0/100.0, self.update)  # 100 Hz update
        self.get_logger().info('Starting smooth arm raise motion.')

    def s_curve(self, t: float) -> float:
        # Normalized time t in [0,1]; 5th-order polynomial 6t^5 -15t^4 +10t^3
        return 6*t**5 - 15*t**4 + 10*t**3

    def update(self):
        if self.done or self.start_positions is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if not self.phase_complete:
            phase = min(1.0, elapsed / self.duration)
            alpha = self.s_curve(phase)
        else:
            alpha = 1.0

        # Build command
        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()

        pos = list(self.start_positions)
        arm_start = ARM_START_LEFT if self.side == 'left' else ARM_START_RIGHT
        idx = arm_start + RAISE_LOCAL_INDEX
        base_val = self.start_positions[idx]
        pos[idx] = base_val + (self.target_angle - base_val) * alpha

        n = len(pos)
        cmd.position = pos
        cmd.velocity = [0.0]*n
        cmd.feed_forward_torque = [0.0]*n
        cmd.torque = [0.0]*n
        # During motion keep high stiffness; after motion keep same or reduce if desired
        cmd.stiffness = [self.kp]*n
        cmd.damping = [self.kd]*n

        self.pub.publish(cmd)

        if not self.phase_complete and elapsed >= self.duration:
            self.phase_complete = True
            self.hold_start = now
            self.get_logger().info('Target angle reached, holding...')

        if self.phase_complete:
            hold_elapsed = (now - self.hold_start).nanoseconds * 1e-9
            if hold_elapsed >= self.hold_time:
                self.get_logger().info('Hold complete. Exiting.')
                self.done = True
                self.create_timer(0.2, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = ArmRaiseSmooth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
