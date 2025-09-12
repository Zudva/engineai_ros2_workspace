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

        # Вычисляем x: 0 → -1 → 0
        if elapsed <= self.sweep_duration:
            # Идём вниз: 0 → -1
            #x = - (elapsed / self.sweep_duration)
            # Используем:
            tau = elapsed / self.sweep_duration
            smooth = 10 * tau**3 - 15 * tau**4 + 6 * tau**5  # Полином пятой степени для S-кривой
            x = -smooth
        else:
            # Идём вверх: -1 → 0
            time_up = elapsed - self.sweep_duration
            x = -1 + (time_up / self.sweep_duration)

        # Формируем сообщение
        msg = JointCommand()
        msg.position = [0.0] * 24
        msg.position[18] = x  # индекс 21 — это 'x' в вашем примере
        msg.position[19] = x  # индекс 22 — это 'x' в вашем примере
        msg.position[20] = x  # индекс 23 — это 'x' в вашем примере
        msg.position[21] = x  # индекс 23 — это 'x' в вашем примере
        
        msg.position[13] = x  # индекс 21 — это 'x' в вашем примере
        msg.position[14] = -x  # индекс 21 — это 'x' в вашем примере
        msg.position[15] = -x  # индекс 22 — это 'x' в вашем примере
        msg.position[16] = x  # индекс 23 — это 'x' в вашем примере
        #msg.position[17] = x  # индекс 23 — это 'x' в вашем примере 
        
          
        msg.velocity = [0.0] * 24
        msg.feed_forward_torque = [0.0] * 24
        msg.torque = [0.0] * 24
        msg.stiffness = [400.0] * 24
        msg.damping = [5.0] * 24
        msg.parallel_parser_type = 0

        self.publisher.publish(msg)
        self.get_logger().info(f"Published x = {x:.3f}")


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
