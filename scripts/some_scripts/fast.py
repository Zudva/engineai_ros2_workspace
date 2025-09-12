#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand
import time

class JointSweepPublisher(Node):
    def __init__(self):
        super().__init__('joint_sweep_publisher')
        self.publisher = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.rate = 200  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Параметры движения
        self.sweep_duration = 5.0  # секунд на путь 0 → -1
        self.total_duration = self.sweep_duration * 2  # туда и обратно
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.running = True

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        if elapsed > self.total_duration:
            self.get_logger().info("Sweep completed. Shutting down...")
            self.running = False
            rclpy.shutdown()
            return

        # Вычисляем x: 0 → -1 → 0 с плавным ускорением/замедлением
        if elapsed <= self.sweep_duration:
            # Фаза 1: 0 → -1
            tau = elapsed / self.sweep_duration
            # S-кривая: плавный
