#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointState  # Убедитесь, что тип сообщения именно такой

class JointFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('joint_feedback_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/hardware/joint_state',  # Топик, куда публикуется состояние
            self.listener_callback,
            10)
        self.subscription  # предотвращаем предупреждение

        # Суставы, за которыми мы следим (как в вашем publisher)
        self.monitored_joints = [13, 14, 15, 16, 18, 19, 20, 21]

    def listener_callback(self, msg):
        # Извлекаем позиции и скорости
        positions = msg.position
        velocities = msg.velocity

        # Формируем строку для вывода
        output_lines = ["=== Joint Feedback ==="]
        for idx in self.monitored_joints:
            if idx < len(positions) and idx < len(velocities):
                pos = positions[idx]
                vel = velocities[idx]
                output_lines.append(f"Joint {idx:2d} | Pos: {pos:8.4f} rad | Vel: {vel:8.4f} rad/s")
            else:
                output_lines.append(f"Joint {idx:2d} | Data N/A")

        # Выводим блоком, чтобы не мешать друг другу в консоли
        self.get_logger().info("\n".join(output_lines) + "\n" + "="*40)

def main(args=None):
    rclpy.init(args=args)
    subscriber = JointFeedbackSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
