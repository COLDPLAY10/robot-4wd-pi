# Шпаргалка команд для управления роботом

## Быстрый запуск

### Интерактивное меню
```bash
./run_robot.sh
```

---

## Лидар

### Настройка
```bash
# Автоматическая настройка UART
sudo ./setup_lidar.sh

# Диагностика
python3 diagnose_lidar.py

# Простой тест
python3 lidar.py
```

### Использование с лидаром
```bash
# Исследование с лидаром
python3 demo_with_lidar.py explore

# Навигация к точке
python3 demo_with_lidar.py goto 2.0 1.5
```

### Без лидара
```bash
# Исследование без лидара
python3 demo_no_lidar.py explore

# Навигация к точке
python3 demo_no_lidar.py goto 2.0 1.5
```

---

## Скрипт 1: Сбор данных (data_collector.py)

### Базовое использование
```bash
# Запуск с параметрами по умолчанию
python3 data_collector.py

# С указанием директории
python3 data_collector.py --output-dir corridor_data

# Полная настройка для коридора вуза
python3 data_collector.py \
    --output-dir diploma_data_2024 \
    --speed 18 \
    --photo-interval 0.4 \
    --forward-time 5.0 \
    --rotate-time 3.0
```

### Параметры
- `-o, --output-dir DIR` - директория для сохранения
- `-s, --speed N` - скорость движения (0-255)
- `-i, --photo-interval SEC` - интервал между фото
- `-f, --forward-time SEC` - время движения вперед
- `-r, --rotate-time SEC` - время поворота 360°

---

## Скрипт 2: Детекция препятствий (obstacle_detector.py)

### Базовое использование
```bash
# Детекция белых препятствий
python3 obstacle_detector.py --color white

# Детекция черных/темно-синих препятствий
python3 obstacle_detector.py --color black

# Детекция пестрых препятствий
python3 obstacle_detector.py --color mixed
```

### С сохранением фотографий
```bash
# Белые препятствия с фото
python3 obstacle_detector.py --color white --save-photos

# Черные препятствия с настройкой директории
python3 obstacle_detector.py --color black --save-photos --output-dir black_obstacles

# Пестрые препятствия с полной настройкой
python3 obstacle_detector.py \
    --color mixed \
    --save-photos \
    --output-dir mixed_test \
    --speed 12 \
    --scan-interval 0.3
```

### Параметры
- `-c, --color COLOR` - цвет препятствия: white/black/mixed **[ОБЯЗАТЕЛЬНО]**
- `-s, --save-photos` - сохранять фотографии
- `-o, --output-dir DIR` - директория для фото
- `--speed N` - скорость движения
- `--scan-interval SEC` - интервал сканирования

---

## Примеры для дипломной работы

### Сбор обучающей выборки в коридоре
```bash
# Запуск 1: Утренняя съемка
python3 data_collector.py \
    --output-dir data/morning_session \
    --speed 20 \
    --photo-interval 0.3 \
    --forward-time 6.0

# Запуск 2: Дневная съемка
python3 data_collector.py \
    --output-dir data/afternoon_session \
    --speed 20 \
    --photo-interval 0.3 \
    --forward-time 6.0

# Запуск 3: Вечерняя съемка
python3 data_collector.py \
    --output-dir data/evening_session \
    --speed 20 \
    --photo-interval 0.3 \
    --forward-time 6.0
```

### Тестирование детекции всех типов препятствий
```bash
# Создать директорию для теста
mkdir -p test_results

# Тест 1: Белые препятствия
python3 obstacle_detector.py \
    --color white \
    --save-photos \
    --output-dir test_results/white

# Тест 2: Черные препятствия
python3 obstacle_detector.py \
    --color black \
    --save-photos \
    --output-dir test_results/black

# Тест 3: Пестрые препятствия
python3 obstacle_detector.py \
    --color mixed \
    --save-photos \
    --output-dir test_results/mixed
```

### Сбор данных для конкретных условий освещения
```bash
# Яркое освещение - быстрее
python3 data_collector.py \
    --output-dir bright_light_data \
    --speed 25 \
    --photo-interval 0.5

# Тусклое освещение - медленнее и чаще
python3 data_collector.py \
    --output-dir dim_light_data \
    --speed 15 \
    --photo-interval 0.3
```

---

## Управление данными

### Просмотр собранных данных
```bash
# Количество фотографий в директории
ls -1 collected_data/*.jpg | wc -l

# Детальная информация о файлах
ls -lh collected_data/

# Просмотр изображения (если установлен feh или другой просмотрщик)
feh collected_data/
```

### Организация данных
```bash
# Создать структуру директорий
mkdir -p {collected_data,obstacle_photos}/{white,black,mixed}

# Переместить файлы по типам
mv obstacle_photos/*white* obstacle_photos/white/
mv obstacle_photos/*black* obstacle_photos/black/
mv obstacle_photos/*mixed* obstacle_photos/mixed/
```

### Очистка старых данных
```bash
# Осторожно! Удаляет все данные
rm -rf collected_data/*
rm -rf obstacle_photos/*

# Удалить файлы старше 7 дней
find collected_data/ -name "*.jpg" -mtime +7 -delete
find obstacle_photos/ -name "*.jpg" -mtime +7 -delete
```

---

## Отладка и тестирование

### Проверка камеры
```bash
# Список доступных камер
v4l2-ctl --list-devices

# Тест захвата кадра
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"
```

### Проверка ультразвукового датчика
```bash
# Тест чтения расстояния
python3 -c "from ultrasonic import read_distance_cm_from_bot; print(f'Distance: {read_distance_cm_from_bot()} cm')"
```

### Проверка моторов
```bash
# Тест движения
python3 -c "import car_adapter as ca; ca.move_forward(20); import time; time.sleep(2); ca.stop()"
```

### Проверка RGB LED
```bash
# Тест индикации
python3 -c "import RGB; RGB.set_red(); import time; time.sleep(1); RGB.off()"
```

---

## Безопасная остановка

В любой момент нажмите **Ctrl+C** для безопасной остановки робота.

Оба скрипта корректно обрабатывают сигнал прерывания:
- ✓ Моторы остановятся
- ✓ Камера закроется
- ✓ LED выключится
- ✓ Данные сохранятся

---

## Установка зависимостей

```bash
# Обновление pip
pip3 install --upgrade pip

# Установка необходимых библиотек
pip3 install opencv-python numpy

# Проверка установки
python3 -c "import cv2; import numpy; print('OK')"
```

---

## Системные требования

- Python 3.7+
- OpenCV (cv2)
- NumPy
- Raspbot_Lib (библиотека робота)
- Raspberry Pi с камерой
- Ультразвуковой датчик
- RGB LED индикатор

---

## Горячие клавиши

- **Ctrl+C** - остановка скрипта
- **Ctrl+Z** - приостановка (потом `fg` для продолжения)
- **Ctrl+D** - выход из интерактивного режима

---

## Полезные алиасы (добавить в ~/.bashrc)

```bash
# Алиасы для быстрого запуска
alias robot-menu='cd ~/PycharmProjects/robot-4wd-pi && ./run_robot.sh'
alias robot-collect='cd ~/PycharmProjects/robot-4wd-pi && python3 data_collector.py'
alias robot-white='cd ~/PycharmProjects/robot-4wd-pi && python3 obstacle_detector.py --color white'
alias robot-black='cd ~/PycharmProjects/robot-4wd-pi && python3 obstacle_detector.py --color black'
alias robot-mixed='cd ~/PycharmProjects/robot-4wd-pi && python3 obstacle_detector.py --color mixed'
```

После добавления: `source ~/.bashrc`

---

## Контакты и помощь

При возникновении проблем:
1. Проверьте подключение робота
2. Проверьте камеру и датчики
3. Посмотрите README_SCRIPTS.md
4. Проверьте логи в консоли

Удачи с дипломной работой! 🤖🎓

