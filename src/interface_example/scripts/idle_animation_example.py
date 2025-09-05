#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand, JointState
from std_msgs.msg import Header
import math

class IdleAnimation(Node):
    def __init__(self):
        super().__init__('idle_animation')
        # Parameters
        self.declare_parameter('amplitude', 0.25)
        self.declare_parameter('frequency', 0.8)  # Hz
        self.declare_parameter('left_indices', [13])
        self.declare_parameter('right_indices', [18])
        self.amp = self.get_parameter('amplitude').get_parameter_value().double_value
        self.freq = self.get_parameter('frequency').get_parameter_value().double_value
        self.arm_indices_left = list(self.get_parameter('left_indices').get_parameter_value().integer_array_value)
        self.arm_indices_right = list(self.get_parameter('right_indices').get_parameter_value().integer_array_value)

        qos = 10
        self.pub = self.create_publisher(JointCommand, '/hardware/joint_command', qos)
        self.state_sub = self.create_subscription(JointState, '/hardware/joint_state', self.state_cb, qos)
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50 Hz
        self.phase = 0.0
        self.have_initial = False
        self.neutral = []
        self.last_warn_indices = False
        self.get_logger().info('Idle animation publisher started (amp=%.3f freq=%.2fHz)' % (self.amp, self.freq))

    def state_cb(self, msg: JointState):
        if not self.have_initial:
            self.neutral = list(msg.position)
            self.have_initial = True
            n = len(self.neutral)
            # Auto-detect if provided indices are out of range
            def in_range(indices):
                return all(0 <= i < n for i in indices)
            if (not in_range(self.arm_indices_left)) or (not in_range(self.arm_indices_right)):
                # Fallback: split last 8 joints into two groups if possible
                span = min(8, n)
                base = n - span
                half = span // 2
                self.arm_indices_left = list(range(base, base + half))
                self.arm_indices_right = list(range(base + half, base + span))
                self.get_logger().warn(
                    f'Provided indices out of range; auto-selected left={self.arm_indices_left} right={self.arm_indices_right}')
            else:
                self.get_logger().info(f'Using configured indices left={self.arm_indices_left} right={self.arm_indices_right}')
            self.get_logger().info(f'Received initial joint state ({n} joints)')

    def timer_cb(self):
        if not self.have_initial:
            return
        # Advance phase based on frequency and timer period
        self.phase += 2*math.pi*self.freq*0.02
        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        # copy neutral
        cmd.position = list(self.neutral)
        cmd.velocity = [0.0]*len(self.neutral)
        cmd.feed_forward_torque = [0.0]*len(self.neutral)
        cmd.torque = [0.0]*len(self.neutral)
        cmd.stiffness = [300.0]*len(self.neutral)
        cmd.damping = [5.0]*len(self.neutral)
        # animate shoulders (if indices exist)
        n = len(cmd.position)
        # Safety clamp amplitude (avoid huge values if param changed at runtime)
        amp = min(max(self.amp, 0.0), 1.5)
        for idx in self.arm_indices_left:
            if 0 <= idx < n:
                cmd.position[idx] = self.neutral[idx] + amp*math.sin(self.phase)
        for idx in self.arm_indices_right:
            if 0 <= idx < n:
                cmd.position[idx] = self.neutral[idx] + amp*math.sin(self.phase + math.pi)
        # If none applied, warn once
        if not self.last_warn_indices and not any(0 <= i < n for i in self.arm_indices_left + self.arm_indices_right):
            self.get_logger().warn('No valid indices for animation; publishing neutral only')
            self.last_warn_indices = True
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = IdleAnimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
