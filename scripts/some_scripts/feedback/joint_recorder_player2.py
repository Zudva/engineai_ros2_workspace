#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointState, JointCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import sys
import threading

class JointRecorderPlayer(Node):
    def __init__(self, mode, filename="recorded_trajectory.txt"):
        super().__init__('joint_recorder_player')
        self.mode = mode
        self.filename = filename
        self.joint_indices = [0,1,2,3,4,5,6,7,8,9,10,11,12,13, 14, 15, 16, 18, 19, 20, 21,22,23]
        self.recording = False
        self.playing = False
        self.recorded_data = []  # [(timestamp, [pos13, pos14, ...]), ...]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        if self.mode == "record":
            self.get_logger().info("=== RECORD MODE === Press SPACE to start/stop recording, ESC to exit ===")
            self.subscription = self.create_subscription(
                JointState,
                '/hardware/joint_state',
                self.record_callback,
                qos_profile
            )
            # Запускаем поток для обработки клавиш
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()

        elif self.mode == "playback":
            self.load_trajectory()
            if not self.recorded_data:
                self.get_logger().error("No data to play! Exiting...")
                rclpy.shutdown()
                return

            self.publisher = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
            self.playback_start_time = None
            self.playback_index = 0
            self.timer = self.create_timer(0.001, self.playback_timer_callback)  # 1000 Hz check
            self.get_logger().info(f"=== PLAYBACK MODE === Loaded {len(self.recorded_data)} points. Starting...")

    def record_callback(self, msg):
        if not self.recording:
            return
        positions = msg.position
        try:
            joint_positions = [positions[i] for i in self.joint_indices]
            timestamp = time.time()
            self.recorded_data.append((timestamp, joint_positions))
            self.get_logger().info(f"Recorded: {joint_positions}")
        except IndexError:
            self.get_logger().warn("Joint index out of range, skipping...")

    def keyboard_listener(self):
        try:
            import tty, termios, sys
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                char = sys.stdin.read(1)
                if char == ' ':
                    self.recording = not self.recording
                    if self.recording:
                        self.get_logger().info("⏺ Recording STARTED")
                    else:
                        self.get_logger().info("⏹ Recording STOPPED")
                elif ord(char) == 27:  # ESC
                    self.get_logger().info("💾 Saving to file...")
                    self.save_trajectory()
                    self.get_logger().info("✅ Saved. Exiting...")
                    rclpy.shutdown()
                    break
        except ImportError:
            self.get_logger().warn("keyboard listener not available on this platform")
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")

    def save_trajectory(self):
        with open(self.filename, 'w') as f:
            for timestamp, positions in self.recorded_data:
                line = f"{timestamp:.6f} " + " ".join([f"{p:.6f}" for p in positions])
                f.write(line + "\n")
        self.get_logger().info(f"Saved {len(self.recorded_data)} points to {self.filename}")

    def load_trajectory(self):
        try:
            with open(self.filename, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) != len(self.joint_indices) + 1:
                        continue
                    timestamp = float(parts[0])
                    positions = [float(p) for p in parts[1:]]
                    self.recorded_data.append((timestamp, positions))
            self.get_logger().info(f"Loaded {len(self.recorded_data)} points from {self.filename}")
        except FileNotFoundError:
            self.get_logger().error(f"File {self.filename} not found!")

    def playback_timer_callback(self):
        if self.playback_index >= len(self.recorded_data):
            self.get_logger().info("⏹ Playback finished!")
            rclpy.shutdown()
            return

        current_time = time.time()
        if self.playback_start_time is None:
            self.playback_start_time = current_time

        target_timestamp, target_positions = self.recorded_data[self.playback_index]
        elapsed_playback_time = current_time - self.playback_start_time

        # Ждём нужного момента времени
        if elapsed_playback_time < (target_timestamp - self.recorded_data[0][0]):
            return

        # Отправляем команду
        msg = JointCommand()
        msg.position = [0.0] * 24
        for i, idx in enumerate(self.joint_indices):
            msg.position[idx] = target_positions[i]
        msg.velocity = [0.0] * 24
        msg.feed_forward_torque = [0.0] * 24
        msg.torque = [0.0] * 24
        msg.stiffness = [400.0] * 24
        msg.damping = [5.0] * 24
        msg.parallel_parser_type = 0

        self.publisher.publish(msg)
        self.get_logger().info(f"Playing [{self.playback_index+1}/{len(self.recorded_data)}]: {target_positions}")
        self.playback_index += 1

def main(args=None):
    if len(sys.argv) < 2 or sys.argv[1] not in ["record", "playback"]:
        print("Usage: ./joint_recorder_player.py [record|playback]")
        return

    mode = sys.argv[1]
    rclpy.init(args=args)
    node = JointRecorderPlayer(mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if mode == "record":
            node.save_trajectory()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
