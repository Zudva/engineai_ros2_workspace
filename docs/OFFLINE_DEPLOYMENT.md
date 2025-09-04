# Offline Deployment (Robot Without Internet)

Кратко: собираем на ПК → пакуем → переносим → распаковываем → source → запускаем.

## 1. Подготовка на ПК (есть интернет)
```bash
cd /path/to/engineai_ros2_workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
./scripts/package_runtime.sh   # создаст offline_bundle/
```
В результате появятся файлы:
```
offline_bundle/ros2_humble.tgz
offline_bundle/third_party.tgz
offline_bundle/workspace_install.tgz
```

Если ROS2 Humble и third_party уже стоят на роботе и совпадают — можно не копировать соответствующие tgz.

## 2. Перенос на робота
Любой метод (scp / USB):
```bash
scp offline_bundle/*.tgz user@ROBOT_IP:~/offline_bundle/
```

## 3. Распаковка на роботе (офлайн)
Вариант A (автоскрипт):
```bash
cd ~
mkdir -p engineai_ros2_workspace
cp -r offline_bundle engineai_ros2_workspace/
cd engineai_ros2_workspace
sudo bash scripts/offline_install.sh offline_bundle
```

Вариант B (вручную):
```bash
cd ~/offline_bundle
sudo mkdir -p /opt/ros /opt/engineai_robotics_third_party
sudo tar xzf ros2_humble.tgz -C /opt          # если нужен ROS2
sudo tar xzf third_party.tgz -C /opt          # если нужны third_party
mkdir -p ~/engineai_ros2_workspace
tar xzf workspace_install.tgz -C ~/engineai_ros2_workspace
```

## 4. Активация окружения (каждая сессия)
```bash
source /opt/ros/humble/setup.bash
source ~/engineai_ros2_workspace/install/setup.bash
```

Проверка:
```bash
ros2 pkg list | grep interface_protocol
```

## 5. Запуск примеров
```bash
# Публикация команд скорости корпуса
ros2 run interface_example body_velocity_control_example

# RL пример (если сконфигурированы модель и параметры)
ros2 run interface_example rl_basic_example
```

## 6. Обновление без пересборки на роботе
На ПК после изменений:
```bash
colcon build --symlink-install
./scripts/package_runtime.sh
```
Передаем ТОЛЬКО новый `workspace_install.tgz` и на роботе:
```bash
cd ~/engineai_ros2_workspace
rm -rf install   # (опционально, чистая замена)
tar xzf /path/to/new/workspace_install.tgz -C .
```
Далее снова source (см. шаг 4).

## 7. Что делать если нужно добавить новый third_party или поменялся ROS2
Повторно создать полный набор tgz (не удаляя старые для отката) и заново распаковать соответствующие архивы.

## 8. Быстрый чеклист
1. Build + package на ПК
2. Копия tgz на робота
3. Распаковка (offline_install.sh)
4. `source /opt/ros/humble/setup.bash`
5. `source ~/engineai_ros2_workspace/install/setup.bash`
6. `ros2 run ...`

## 9. Типичные проблемы
| Симптом | Причина | Решение |
|---------|---------|---------|
| `command not found: ros2` | Не sourced ROS2 | `source /opt/ros/humble/setup.bash` |
| `package not found` при запуске | Не sourced install | `source install/setup.bash` |
| Ошибка загрузки .so third_party | Архив не распакован в /opt | Распаковать `third_party.tgz` |
| Несовместимость ABI | На ПК и роботе разные версии ROS2 | Пересобрать на ПК с той же версией ROS2, перепаковать |

## 10. Optional: systemd unit (пример)
Создать `/etc/systemd/system/engineai_example.service`:
```
[Unit]
Description=EngineAI Example Nodes
After=network.target

[Service]
Type=simple
User=user
Environment=ROS_DOMAIN_ID=69
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/user/engineai_ros2_workspace/install/setup.bash && ros2 run interface_example body_velocity_control_example'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```
Далее:
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now engineai_example.service
```
