#!/usr/bin/env python3
"""
Arm Wave Example
=================
Публикует JointCommand для управления только суставами рук в симуляции MuJoCo.

Логика:
1. Ждём первый /hardware/joint_state, сохраняем исходные позиции всех 24 суставов.
2. Определяем индексы суставов рук (по структуре joint_test.yaml):
   - 0..5   : левая/правая нога (группа 1)
   - 6..11  : вторая нога (группа 2)
   - 12     : торс (группа 3)
   - 13..17 : левая рука (группа 4)
   - 18..22 : правая рука (группа 5)
   - 23     : голова (группа 6)
3. Генерируем синус для плечевых / локтевых суставов (все суставы рук) с ограниченной амплитудой.
4. Остальные суставы удерживаем в исходном положении.

Параметры (аргументы):
  --amp <rad>       Амплитуда (по умолчанию 0.3 рад)
  --freq <Hz>       Частота синуса (по умолчанию 0.5 Гц)
  --rate <Hz>       Частота публикации (по умолчанию 200 Гц)
  --stiff <num>     Жёсткость (по умолчанию 400)
  --damp <num>      Демпфирование (по умолчанию 5)

Безопасно останавливается по Ctrl+C (жёсткость при остановке оставляем — при необходимости можно обнулить).
"""

from __future__ import annotations
import argparse
import math
import sys
import time
from typing import List

import rclpy
from rclpy.node import Node
from interface_protocol.msg import JointCommand, JointState


ARM_LEFT_IDX = list(range(13, 18))   # 13..17
ARM_RIGHT_IDX = list(range(18, 23))  # 18..22
ALL_ARM_IDX = ARM_LEFT_IDX + ARM_RIGHT_IDX


class ArmWaveNode(Node):
    def __init__(self, amp: float, freq: float, rate: float, stiff: float, damp: float, duration: float):
        super().__init__('arm_wave_node')
        self.amp = amp
        self.freq = freq
        self.rate = rate
        self.period = 1.0 / rate
        self.stiff = stiff
        self.damp = damp
        self.duration = duration  # seconds (<=0 means infinite)
        self._have_initial = False
        self._initial_positions: List[float] = []
        self._t0 = time.time()

        # QoS depth 3 best-effort аналогично остальным
        self.sub_joint = self.create_subscription(
            JointState,
            '/hardware/joint_state',
            self._joint_state_cb,
            10
        )
        self.pub_cmd = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.timer = self.create_timer(self.period, self._on_timer)
        # периодическая проверка длительности (реже, чтобы не шуметь)
        if self.duration > 0:
            self.duration_timer = self.create_timer(0.25, self._check_duration)
        self.get_logger().info(
            f'Start arm wave: amp={amp} rad, freq={freq} Hz, rate={rate} Hz, joints={ALL_ARM_IDX}, duration={"inf" if self.duration<=0 else self.duration}s'
        )

    def _joint_state_cb(self, msg: JointState):
        if not self._have_initial:
            self._initial_positions = list(msg.position)
            self._have_initial = True
            self.get_logger().info(f'Captured initial joint_state ({len(self._initial_positions)} joints)')

    def _on_timer(self):
        if not self._have_initial:
            return
        now = time.time()
        t = now - self._t0
        phase = 2.0 * math.pi * self.freq * t
        cmd = JointCommand()
        n = len(self._initial_positions)
        # Базовые вектора удержания
        cmd.position = list(self._initial_positions)
        cmd.velocity = [0.0] * n
        cmd.feed_forward_torque = [0.0] * n
        cmd.torque = [0.0] * n
        cmd.stiffness = [self.stiff] * n
        cmd.damping = [self.damp] * n

        # Синус по всем суставам рук (можно задать разные фазы для левой/правой)
        for i in ARM_LEFT_IDX:
            # Левую делаем синус
            cmd.position[i] = self._initial_positions[i] + self.amp * math.sin(phase)
        for i in ARM_RIGHT_IDX:
            # Правую в противофазе
            cmd.position[i] = self._initial_positions[i] + self.amp * math.sin(phase + math.pi)

        self.pub_cmd.publish(cmd)

    def _check_duration(self):
        if self.duration <= 0:
            return
        elapsed = time.time() - self._t0
        if elapsed >= self.duration:
            # Останавливаем таймер качания, удерживаем последнюю позу
            if self.timer.is_ready():
                try:
                    self.timer.cancel()
                except Exception:
                    pass
            # Публикуем финальную команду удержания (жёсткость сохраняем)
            if self._have_initial:
                n = len(self._initial_positions)
                hold = JointCommand()
                hold.position = list(self._initial_positions)
                hold.velocity = [0.0] * n
                hold.feed_forward_torque = [0.0] * n
                hold.torque = [0.0] * n
                hold.stiffness = [self.stiff] * n
                hold.damping = [self.damp] * n
                self.pub_cmd.publish(hold)
            self.get_logger().info('Duration reached; stopping arm wave node')
            # Завершаем через shutdown
            rclpy.shutdown()


def parse_args(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--amp', type=float, default=0.3, help='Амплитуда (рад)')
    p.add_argument('--freq', type=float, default=0.5, help='Частота (Гц)')
    p.add_argument('--rate', type=float, default=200.0, help='Частота публикации (Гц)')
    p.add_argument('--stiff', type=float, default=400.0, help='Скаляр жёсткости всех суставов')
    p.add_argument('--damp', type=float, default=5.0, help='Скаляр демпфирования')
    p.add_argument('--duration', type=float, default=0.0, help='Продолжительность в секундах (0 или <0 = бесконечно)')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = ArmWaveNode(args.amp, args.freq, args.rate, args.stiff, args.damp, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
