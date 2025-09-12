#!/usr/bin/env python3
"""Compare outgoing JointCommand with incoming JointState.

Usage:
  ros2 run (after installing) OR execute directly if in PATH:

  python3 scripts/joint_state_command_diff.py \
      --command-topic /hardware/joint_command \
      --state-topic /hardware/joint_state \
      --rate 5

Shows per-joint error = commanded_position - state_position, plus basic stats.

Exit with Ctrl+C.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import argparse
import math
import statistics
from interface_protocol.msg import JointCommand, JointState

class JointDiff(Node):
    def __init__(self, cmd_topic: str, state_topic: str, rate: float):
        super().__init__('joint_state_command_diff')
        qos = QoSProfile( reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST,
                          depth=10 )
        self.cmd_sub = self.create_subscription(JointCommand, cmd_topic, self.cmd_cb, qos)
        self.state_sub = self.create_subscription(JointState, state_topic, self.state_cb, qos)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.last_cmd = None
        self.last_state = None
        self.seq = 0

    def cmd_cb(self, msg: JointCommand):
        self.last_cmd = msg

    def state_cb(self, msg: JointState):
        self.last_state = msg

    def tick(self):
        if not self.last_cmd or not self.last_state:
            return
        pc = list(self.last_cmd.position)
        ps = list(self.last_state.position)
        n = min(len(pc), len(ps))
        if n == 0:
            return
        errs = [pc[i] - ps[i] for i in range(n)]
        abs_errs = [abs(e) for e in errs]
        mean_err = statistics.fmean(abs_errs)
        max_err = max(abs_errs)
        rms = math.sqrt(sum(e*e for e in errs) / n)
        # Compact line
        self.get_logger().info(
            f"{self.seq:05d} n={n} mean|e|={mean_err:.4f} max|e|={max_err:.4f} rms={rms:.4f} first3={errs[:3]}"
        )
        self.seq += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--command-topic', default='/hardware/joint_command')
    parser.add_argument('--state-topic', default='/hardware/joint_state')
    parser.add_argument('--rate', type=float, default=5.0, help='Print rate Hz')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = JointDiff(args.command_topic, args.state_topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
