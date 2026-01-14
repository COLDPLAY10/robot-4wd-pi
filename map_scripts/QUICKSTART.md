# Быстрый старт - Автономная навигация

## Установка

```bash
# Установите зависимости
pip3 install numpy matplotlib

# Для лидара
pip3 install pyserial

# Для работы с камерой (опционально)
pip3 install opencv-python torch torchvision
```

## Настройка лидара (если есть)

```bash
# Автоматическая настройка UART для лидара
sudo ./setup_lidar.sh

# Диагностика подключения
python3 diagnose_lidar.py

# Простой тест
python3 lidar.py
```

Подробнее: [README_LIDAR.md](README_LIDAR.md)

## Быстрые тесты

### 1. Тест компонентов системы

```bash
# Все тесты
python3 test_navigation.py all

# Отдельные тесты
python3 test_navigation.py slam
python3 test_navigation.py ultrasonic
python3 test_navigation.py camera
```

### 2. Демо без лидара (только ультразвук + камера)

```bash
# Исследование окружения
python3 demo_no_lidar.py explore

# Движение к точке (2, 3)
python3 demo_no_lidar.py goto 2.0 3.0
```

### 2a. Демо С лидаром (полная система)

```bash
# Исследование окружения с лидаром
python3 demo_with_lidar.py explore

# Движение к точке с лидаром
python3 demo_with_lidar.py goto 2.0 3.0
```

### 3. Полная система навигации

```bash
# Интерактивный режим
python3 autonomous_navigation.py --mode interactive --no-lidar

# Автоматическое исследование (30 секунд)
python3 autonomous_navigation.py --mode exploration --no-lidar --duration 30

# Движение к цели
python3 autonomous_navigation.py --mode navigation --goal 2.0 3.0 --no-lidar
```

### 4. Тест камеры

```bash
# Запуск визуализации сегментации
python3 camera_segmentation.py
```

### 5. Визуализация карты

```bash
# После исследования откройте карту
python3 visualize_map.py --map map_autosave_YYYYMMDD_HHMMSS.pkl

# Или сохраните изображение
python3 visualize_map.py --map map_save.pkl --save map_image.png
```

## Команды в интерактивном режиме

```
explore [время]  - начать исследование
goto X Y         - двигаться к точке
stop             - остановить
status           - показать статус
save [файл]      - сохранить карту
load [файл]      - загрузить карту
quit             - выход
```

## Примеры использования

### Пример 1: Исследование комнаты

```bash
python3 autonomous_navigation.py --mode exploration --no-lidar --duration 60
```

Робот будет медленно двигаться, строя карту комнаты в течение 60 секунд.

### Пример 2: Навигация с сохраненной картой

```bash
# Сначала исследуйте
python3 autonomous_navigation.py --mode exploration --no-lidar --duration 30

# Затем используйте карту для навигации
python3 autonomous_navigation.py --mode navigation --goal 3.0 2.0 --load-map map_autosave_*.pkl
```

### Пример 3: Интерактивный режим

```bash
python3 autonomous_navigation.py --mode interactive --no-lidar

>> explore 20    # Исследуем 20 секунд
>> status        # Проверяем статус
>> goto 1.0 1.5  # Едем к точке
>> save my_map   # Сохраняем карту
>> quit          # Выход
```

## Конфигурация

### Скорости движения

Отредактируйте `map/navigation_controller.py`:

```python
self.exploration_speed = 15      # Скорость исследования (0-100)
self.navigation_speed = 30       # Скорость навигации (0-100)
self.min_obstacle_distance = 0.3 # Минимальное расстояние до препятствия (м)
```

### Размер карты

Отредактируйте `map/slam_core.py`:

```python
map_width = 400          # Ширина карты (ячейки)
map_height = 400         # Высота карты (ячейки)
resolution = 0.05        # Размер ячейки (метры)
```

## Решение проблем

### Проблема: Камера не работает

```bash
# Проверьте ID камеры
ls /dev/video*

# Измените в camera_segmentation.py
CAMERA_ID = 0  # или 1, 2...
```

### Проблема: Ультразвук не читается

```bash
# Проверьте в test_navigation.py
python3 test_navigation.py ultrasonic
```

### Проблема: Робот не двигается

Проверьте подключение к плате управления в `car_adapter.py`.

## Структура проекта

```
robot-4wd-pi/
├── map/                           # Модули SLAM
│   ├── slam_core.py              # SLAM система
│   ├── sensor_fusion.py          # Объединение сенсоров
│   ├── path_planner.py           # Планирование пути
│   └── navigation_controller.py  # Контроллер навигации
├── autonomous_navigation.py      # Главный скрипт
├── demo_no_lidar.py             # Демо без лидара
├── camera_segmentation.py       # Работа с камерой
├── visualize_map.py             # Визуализация карты
├── test_navigation.py           # Тесты
└── README_NAVIGATION.md         # Подробная документация
```

## Полезные ссылки

- Подробная документация: `README_NAVIGATION.md`
- Примеры команд: `COMMANDS_CHEATSHEET.md`
- Описание скриптов: `README_SCRIPTS.md`

