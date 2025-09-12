#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand, GamepadKeys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class GamepadRightArmSmoother(Node):
    def __init__(self):
        super().__init__('gamepad_right_arm_smoother')
        
        # Публикуем команды на приводы
        self.publisher = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        
        # 👇 КОРРЕКТНЫЙ QoS — теперь совпадает с издателем (проверено: работает!)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # ✅ РАБОТАЕТ — ИСПОЛЬЗУЕМ ЭТО!
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            GamepadKeys,
            '/hardware/gamepad_keys',
            self.gamepad_callback,
            qos_profile
        )
        
        # Частота обновления — 500 Гц
        self.rate = 500
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # --- Параметры движения ---
        self.max_speed = 1.0         # м/с — максимальная скорость изменения позиции
        self.acceleration_time = 0.2 # время разгона до max_speed
        self.amplitude = 2.0         # ✅ МАКСИМАЛЬНАЯ АМПЛИТУДА: от -2.0 до +2.0
        
        # Текущие целевые значения от джойстика
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Текущие позиции (плавно приближаются к цели)
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Время последнего обновления
        self.last_update_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info("🎮 Правая рука готова к управлению через джойстик")
        self.get_logger().info(f"  Амплитуда: ±{self.amplitude}")
        self.get_logger().info("  X-axis (analog[4]): лево/право → поворот руки")
        self.get_logger().info("  Y-axis (analog[5]): вверх/вниз → инвертированное подъём/опускание")

    def gamepad_callback(self, msg):
        """Читаем аналоговые оси с джойстика и масштабируем до ±2.0"""
        if len(msg.analog_states) >= 6:
            # Принимаем значения от -1.0 до 1.0, масштабируем до ±2.0
            self.target_x = msg.analog_states[4] * self.amplitude
            # ✅ ИНВЕРТИРУЕМ Y: когда джойстик вверх → рука вниз
            self.target_y = -msg.analog_states[5] * self.amplitude

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt <= 0:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        # Ограничиваем максимальную скорость за один шаг
        max_step = self.max_speed * dt
        if abs(dx) > max_step:
            dx = max_step if dx > 0 else -max_step
        if abs(dy) > max_step:
            dy = max_step if dy > 0 else -max_step

        # --- S-кривая: плавное ускорение и замедление ---
        def apply_s_curve(step, target_distance):
            if abs(target_distance) < 1e-6:
                return step
            # progress = какую часть пути мы уже прошли?
            progress = min(1.0, abs(step) / abs(target_distance))
            # t = 1 - progress → 1 в начале, 0 в конце
            t = 1.0 - progress
            # S-кривая: 10t³ - 15t⁴ + 6t⁵
            smooth_factor = 10 * t**3 - 15 * t**4 + 6 * t**5
            return step * smooth_factor

        step_x = apply_s_curve(dx, self.target_x - self.current_x)
        step_y = apply_s_curve(dy, self.target_y - self.current_y)

        self.current_x += step_x
        self.current_y += step_y

        # --- Отправляем команду только на суставы 18 и 19 ---
        msg = JointCommand()
        msg.position = [0.0] * 24
        msg.velocity = [0.0] * 24
        msg.feed_forward_torque = [0.0] * 24
        msg.torque = [0.0] * 24
        msg.stiffness = [400.0] * 24
        msg.damping = [5.0] * 24
        msg.parallel_parser_type = 0

        msg.position[18] = self.current_x
        msg.position[19] = self.current_y

        self.publisher.publish(msg)

        # Логируем раз в 0.5 сек
        if int(current_time * 2) != int((current_time - dt) * 2):
            self.get_logger().info(f"🎯 Right Arm: X={self.current_x:.3f}, Y={self.current_y:.3f} | Target: ({self.target_x:.3f}, {self.target_y:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = GamepadRightArmSmoother()
    
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
