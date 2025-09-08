## Синхронизация исходников на борт (Sync the code to Board)

Скрипт: `scripts/sync_src.sh`

Назначение: копирование выбранных подкаталогов (по умолчанию `src`, `scripts`) на целевое устройство (`nezha`, `orin`) через `rsync` с удалением устаревших файлов (`--delete`).

### Быстрый старт
```bash
./scripts/sync_src.sh nezha
```
Без аргумента используется пресет `nezha`.

### Конфигурация хостов
Ассоциативный массив `HOST_CONFIGS`: ключи `<name>,REMOTE_HOST`, `<name>,REMOTE_USER`, `<name>,REMOTE_PASSWORD`, `<name>,REMOTE_PATH`.
Добавление нового:
1. Скопируйте блок существующего.
2. Измените параметры (имя, IP, пользователь, пароль, путь).
3. (Опц.) Уберите пароль после перехода на ключи.

### Что синхронизируется
`SYNC_FILES` по умолчанию:
```
src
scripts
```
Можно добавить `configs`, `launch`, `docs` и т.д.

### Алгоритм
1. Определяет корень репозитория.
2. Валидирует выбранный хост.
3. Проверяет / устанавливает `sshpass`.
4. Создаёт при необходимости `REMOTE_PATH` на удалённой стороне.
5. Для каждого элемента выполняет `rsync -avz --delete` (для директорий с хвостовым `/`).

### Переход на SSH ключи
Пароль в коде — временно.
```bash
ssh-keygen -t ed25519 -f ~/.ssh/nezha_key -C nezha_sync
ssh-copy-id -i ~/.ssh/nezha_key.pub user@192.168.0.163
```
После чего замените вызовы с `sshpass` на:
```bash
rsync -avz -e "ssh -i $SYNC_SSH_KEY -o StrictHostKeyChecking=no" --delete src/ user@host:/path/src/
```
Экспортируйте `export SYNC_SSH_KEY=~/.ssh/nezha_key`.

### Частые проблемы
| Симптом | Причина | Решение |
|---------|---------|---------|
| Unknown target host | Неверное имя | Запустить без аргументов (список) |
| Permission denied | Пароль/ключ | Проверить `ssh user@host` |
| rsync: not found | Нет rsync на плате | `sudo apt-get install -y rsync` |
| Медленно | Сжатие не нужно | Удалить `-z` |
| Проверка перед синком | Нужен dry-run | Добавить `-n` |
| Лишние каталоги | Нет исключений | `--exclude '.git' --exclude '__pycache__'` |

### Расширения
- Флаг `--dry-run`.
- `--itemize-changes` для краткого diff.
- Архив tar для очень слабых CPU.

### Пример цикла
```bash
colcon build --packages-select interface_example
source install/setup.bash
./scripts/sync_src.sh nezha
ssh user@192.168.0.163 'cd /home/user/source/engineai_workspace && colcon build --packages-select interface_example'
ssh user@192.168.0.163 'source install/setup.bash && ros2 run interface_example arm_raise_smooth'
```

### Чеклист перед синком
1. Изменения сохранены.
2. Хост выбран верно.
3. Достаточно места (`df -h`).
4. Нет критичных процессов.
5. (Опц.) Dry-run.

---
См. также: `docs/JOINT_COMMAND_CLI.md` для контекста работы с командами суставов.
