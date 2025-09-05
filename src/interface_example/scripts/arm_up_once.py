#!/usr/bin/env python3
"""Smoothly raise one arm (using provided example: joint index 2 to 1 rad) then hold.

Assumptions:
- Same joint ordering as other examples (24 actuated joints total).
- The example command given set position[2] = 1 and velocity[2] = 1 while others 0.
- We'll interpolate position[2] from current to TARGET over DURATION seconds with a smooth (cosine) profile.

High stiffness/damping applied only during motion; then optionally lower.
"""
import math
import time
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand, JointState
from std_msgs.msg import Header

TARGET_JOINT_INDEX = 2
TARGET_ANGLE = 1.0  # radians
DURATION = 2.0      # seconds for smooth motion
PUBLISH_RATE = 50.0 # Hz
HOLD_TIME = 0.5     # seconds of reinforcement after reaching target

HIGH_KP = 400.0
HIGH_KD = 5.0
HOLD_KP = 300.0
HOLD_KD = 4.0

class ArmUpOnce(Node):
    def __init__(self):
        super().__init__('arm_up_once')
        self.pub = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/hardware/joint_state', self.state_cb, 10)
        self.joint_count = None
        self.start_positions = None
        self.motion_started = False
        self.start_time = None
        self.timer = None
        self.finalized = False
        self.get_logger().info('Waiting for first JointState to start arm raise...')

    def state_cb(self, msg: JointState):
        if self.motion_started:
            return
        self.joint_count = len(msg.position)
        self.start_positions = list(msg.position)
        if TARGET_JOINT_INDEX >= self.joint_count:
            self.get_logger().error(f'Target joint index {TARGET_JOINT_INDEX} out of range {self.joint_count}')
            rclpy.shutdown()
            return
        self.motion_started = True
        self.start_time = self.get_clock().now()
        period = 1.0 / PUBLISH_RATE
        self.timer = self.create_timer(period, self.update)
        self.get_logger().info('Starting smooth arm raise...')

    def smooth_profile(self, t: float) -> float:
        # t in [0,1]
        # Use 0.5*(1 - cos(pi*t)) for smooth acceleration/deceleration
        return 0.5 * (1.0 - math.cos(math.pi * t))

    def update(self):
        if self.finalized:
            return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        phase = min(1.0, elapsed / DURATION)
        alpha = self.smooth_profile(phase)

        # Build command
        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        if self.start_positions is None or self.joint_count is None:
            return  # Should not happen, safety guard

        # Base pose remains start_positions
        pos = list(self.start_positions)
        start_val = self.start_positions[TARGET_JOINT_INDEX]
        target = start_val + (TARGET_ANGLE - start_val) * alpha
        pos[TARGET_JOINT_INDEX] = target

        cmd.position = pos
        n = self.joint_count
        cmd.velocity = [0.0]*n
        cmd.feed_forward_torque = [0.0]*n
        cmd.torque = [0.0]*n

        # Stiffness/damping ramp (optional: keep constant high during motion)
        if phase < 1.0:
            cmd.stiffness = [HIGH_KP]*n
            cmd.damping = [HIGH_KD]*n
        else:
            cmd.stiffness = [HOLD_KP]*n
            cmd.damping = [HOLD_KD]*n

        self.pub.publish(cmd)

        if phase >= 1.0:
            # Hold a bit then exit
            if elapsed >= DURATION + HOLD_TIME:
                self.get_logger().info('Arm raise complete. Exiting.')
                self.finalized = True
                self.create_timer(0.1, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = ArmUpOnce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
