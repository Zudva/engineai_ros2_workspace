#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand
import time
import math
class JointSweepPublisher(Node):
    def __init__(self):
        super().__init__('joint_sweep_publisher')
        self.publisher = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.rate = 500  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Параметры движения
        self.sweep_duration = 3# секунд на путь 0 → -1
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

        # Вычисляем x: 0 → -1 → 0 с плавным ускорением/замедлением в ОБЕИХ фазах
        if elapsed <= self.sweep_duration:
            # Фаза 1: 0 → -1
            tau = elapsed / self.sweep_duration
            # S-кривая: плавный старт → быстрое движение в середине → плавный финиш
            smooth = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            #smooth = 0.5 * (1 - math.cos(math.pi * tau))
            x = -smooth  # от 0 до -1
        else:
            # Фаза 2: -1 → 0
            time_up = elapsed - self.sweep_duration
            tau = time_up / self.sweep_duration
            smooth = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            #smooth = 0.5 * (1 - math.cos(math.pi * tau))
            x = -1 + smooth  # от -1 до 0

        # Формируем сообщение
        msg = JointCommand()
        msg.position = [0.0] * 24
        msg.position[18] = x
        msg.position[19] = x
        msg.position[20] = x
        msg.position[21] = x
        msg.position[13] = x
        msg.position[14] = -x
        msg.position[15] = -x
        msg.position[16] = x

        msg.velocity = [0.0] * 24
        msg.feed_forward_torque = [0.0] * 24
        msg.torque = [0.0] * 24
        msg.stiffness = [400.0] * 24
        msg.damping = [5.0] * 24
        msg.parallel_parser_type = 0

        self.publisher.publish(msg)
        # Логируем только каждые 0.5 секунд, чтобы не засорять терминал
        if int(elapsed * 2) != int((elapsed - 1.0/self.rate) * 2):
            self.get_logger().info(f"Published x = {x:.3f} (elapsed: {elapsed:.2f}s)")

def main(args=None):
    rclpy.init(args=args)
    node = JointSweepPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
