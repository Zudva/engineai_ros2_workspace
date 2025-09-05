#!/usr/bin/env python3
"""Publish a single JointCommand with a reasonable standing pose.

Assumes 24 actuated joints in the order already used elsewhere.
Leg indices (0-5 left, 6-11 right) use a slight knee bend.
Arms neutral, head centered. High stiffness on all joints for a moment.
"""

import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand, JointState
from std_msgs.msg import Header
import time

STAND_POSE = [
    # Left leg (hip pitch, hip roll, hip yaw, knee pitch, ankle pitch, ankle roll)
     0.0,  0.5,  1.57, 0.6,  -0.3, 0.0,
    # Right leg
    -0.0, -0.5, -1.57, 0.6, -0.3, 0.0,
    # Torso
     0.0,
    # Left arm (shoulder yaw, shoulder roll (raise), shoulder pitch, elbow, wrist) -> outstretched
     0.0, 1.57, 0.0, 0.0, 0.0,
    # Right arm (mirror) -> outstretched
     0.0,-1.57, 0.0, 0.0, 0.0,
    # Head
     0.0
]

NUM_JOINTS = len(STAND_POSE)

class StandOnce(Node):
    def __init__(self):
        super().__init__('stand_up_once')
        self.pub = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/hardware/joint_state', self.state_cb, 10)
        self.sent = False
        self.get_logger().info('Stand-up node waiting for first joint state...')

    def state_cb(self, msg: JointState):
        if self.sent:
            return
        if len(msg.position) != NUM_JOINTS:
            self.get_logger().warn(f'Joint count {len(msg.position)} != expected {NUM_JOINTS}')
        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        n = len(msg.position)
        # Resize arrays to message size (fallback if mismatch)
        pose = STAND_POSE if len(STAND_POSE) == n else [0.0]*n
        cmd.position = list(pose)
        cmd.velocity = [0.0]*n
        cmd.feed_forward_torque = [0.0]*n
        cmd.torque = [0.0]*n
        cmd.stiffness = [400.0]*n
        cmd.damping = [5.0]*n
        self.pub.publish(cmd)
        self.get_logger().info('Stand pose command published.')
        # publish a couple more times to ensure latch effect
        for _ in range(3):
            time.sleep(0.05)
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(cmd)
        self.get_logger().info('Stand pose reinforcement done. Exiting.')
        self.sent = True
        # allow small delay then shutdown
        self.create_timer(0.2, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()

def main():
    rclpy.init()
    node = StandOnce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
