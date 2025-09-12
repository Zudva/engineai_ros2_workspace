#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import CubicSpline
import sys
import argparse

def smooth_trajectory(input_file, output_file, target_rate=500, time_scale=1.0):
    """
    Сглаживает траекторию из файла с помощью кубических сплайнов.
    
    :param input_file: входной файл с сырыми данными
    :param output_file: выходной файл со сглаженными данными
    :param target_rate: целевая частота публикации (Гц)
    :param time_scale: масштаб времени (1.0 = оригинал, 2.0 = в 2 раза медленнее)
    """
    # Чтение данных
    timestamps = []
    joint_data = []  # список списков: [[j13, j14, ...], [j13, j14, ...], ...]

    try:
        with open(input_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 2:
                    continue
                timestamps.append(float(parts[0]))
                joint_positions = [float(x) for x in parts[1:]]
                joint_data.append(joint_positions)

        if len(timestamps) < 2:
            print("❌ Недостаточно точек для сглаживания!")
            return

        joint_data = np.array(joint_data)  # shape: (N, num_joints)
        timestamps = np.array(timestamps)
        original_duration = timestamps[-1] - timestamps[0]

        print(f"📊 Прочитано {len(timestamps)} точек. Длительность: {original_duration:.3f} сек.")

        # Создание нового равномерного временного ряда
        num_output_points = int(original_duration * target_rate * time_scale)
        if num_output_points < 2:
            num_output_points = 2

        new_timestamps = np.linspace(timestamps[0], timestamps[-1], num_output_points)
        print(f"🎯 Генерация {num_output_points} сглаженных точек на частоте ~{target_rate} Гц (масштаб времени: {time_scale}x)")

        # Сглаживание каждого сустава отдельно
        smoothed_data = []
        for joint_idx in range(joint_data.shape[1]):
            cs = CubicSpline(timestamps, joint_data[:, joint_idx], bc_type='natural')
            smoothed_joint = cs(new_timestamps)
            smoothed_data.append(smoothed_joint)

        smoothed_data = np.array(smoothed_data).T  # shape: (num_output_points, num_joints)

        # Сохранение в файл
        with open(output_file, 'w') as f:
            for i, t in enumerate(new_timestamps):
                # Масштабируем время, если нужно
                scaled_t = timestamps[0] + (t - timestamps[0]) * time_scale
                line = f"{scaled_t:.6f} " + " ".join([f"{p:.6f}" for p in smoothed_data[i]])
                f.write(line + "\n")

        print(f"✅ Сглаженная траектория сохранена в '{output_file}'")

    except FileNotFoundError:
        print(f"❌ Файл '{input_file}' не найден!")
    except Exception as e:
        print(f"❌ Ошибка: {e}")

def main():
    parser = argparse.ArgumentParser(description='Сглаживание траектории робота')
    parser.add_argument('input', help='Входной файл траектории (например, recorded_trajectory.txt)')
    parser.add_argument('-o', '--output', default='smoothed_trajectory.txt', help='Выходной файл (по умолчанию: smoothed_trajectory.txt)')
    parser.add_argument('-r', '--rate', type=int, default=500, help='Частота выходной траектории в Гц (по умолчанию: 500)')
    parser.add_argument('-s', '--scale', type=float, default=1.0, help='Масштаб времени (1.0=оригинал, 2.0=в 2 раза медленнее)')

    args = parser.parse_args()

    smooth_trajectory(args.input, args.output, args.rate, args.scale)

if __name__ == '__main__':
    main()