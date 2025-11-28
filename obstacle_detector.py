#!/usr/bin/env python3
"""
Скрипт для детекции цветовых препятствий:
- Определяет препятствия по цвету: белый (светлый), черный (темно-синий), пестрый
- Использует камеру для детекции цвета
- Использует ультразвуковой датчик для подтверждения препятствия
- Поддержка сохранения фотографий при обнаружении препятствий

Использование:
    python3 obstacle_detector.py --color white [--save-photos] [--output-dir DIR]
    python3 obstacle_detector.py --color black [--save-photos]
    python3 obstacle_detector.py --color mixed [--save-photos]
"""

import argparse
import time
import os
import sys
from datetime import datetime
import signal

# Попытка импорта камеры и numpy
try:
    import cv2
    import numpy as np
    CAMERA_AVAILABLE = True
except ImportError as e:
    cv2 = None
    np = None
    print(f"ОШИБКА: Необходимые библиотеки не установлены: {e}")
    print("Установите: pip3 install opencv-python numpy")
    sys.exit(1)

# Импорты модулей проекта
import car_adapter as ca
from ultrasonic import read_distance_cm_from_bot
import RGB


# Конфигурация
CAMERA_ID = 0
FRAME_W = 640
FRAME_H = 480
DEFAULT_OUTPUT_DIR = "obstacle_photos"
DETECTION_DISTANCE_CM = 30.0  # расстояние для обнаружения препятствия
MIN_COLOR_RATIO = 0.15  # минимальный процент цвета на кадре для детекции

# Глобальные переменные
running = True
stop_requested = False

def signal_handler(sig, frame):
    """Обработчик сигналов для корректного завершения"""
    global running, stop_requested
    print("\n[INFO] Получен сигнал остановки...")
    running = False
    stop_requested = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Определения цветовых диапазонов в HSV
COLOR_RANGES = {
    'white': {
        'hsv_ranges': [((0, 0, 200), (180, 50, 255))],
        'description': 'Белый/Светлый',
        'rgb_led': (255, 255, 255)
    },
    'black': {
        # Темно-синий и черный
        'hsv_ranges': [
            ((0, 0, 0), (180, 255, 50)),      # Очень темные цвета
            ((100, 50, 0), (130, 255, 100))   # Темно-синие
        ],
        'description': 'Черный/Темно-синий',
        'rgb_led': (0, 0, 255)
    },
    'mixed': {
        # Пестрые цвета - комбинация разных цветов
        'hsv_ranges': [
            ((0, 100, 100), (15, 255, 255)),   # Красный
            ((15, 100, 100), (35, 255, 255)),  # Желтый
            ((35, 100, 100), (85, 255, 255)),  # Зеленый
            ((85, 100, 100), (130, 255, 255)), # Синий/Циан
            ((130, 100, 100), (170, 255, 255)) # Фиолетовый
        ],
        'description': 'Пестрый (разноцветный)',
        'rgb_led': (128, 0, 128)
    }
}

class ObstacleDetector:
    def __init__(self, target_color, save_photos=False, output_dir=DEFAULT_OUTPUT_DIR):
        self.target_color = target_color
        self.save_photos = save_photos
        self.output_dir = output_dir
        self.bot = ca.bot
        self.photo_count = 0
        self.detection_count = 0
        self.cap = None
        
        # Создание директории для фото
        if save_photos and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"[INFO] Создана директория: {output_dir}")
        
        # Инициализация камеры
        self.cap = cv2.VideoCapture(CAMERA_ID)
        if not self.cap.isOpened():
            print("[ERROR] Не удалось открыть камеру")
            sys.exit(1)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        print(f"[INFO] Камера инициализирована: {FRAME_W}x{FRAME_H}")
        
        # Инициализация ультразвукового датчика
        try:
            self.bot.Ctrl_Ulatist_Switch(1)
            time.sleep(0.1)
            print("[INFO] Ультразвуковой датчик включен")
        except Exception as e:
            print(f"[WARN] Ошибка инициализации датчика: {e}")
        
        # Информация о выбранном цвете
        color_info = COLOR_RANGES.get(target_color)
        if color_info:
            print(f"[INFO] Режим детекции: {color_info['description']}")
    
    def detect_color_in_frame(self, frame):
        """
        Определяет наличие целевого цвета в кадре
        Возвращает: (detected: bool, ratio: float, mask: np.array)
        """
        if frame is None:
            return False, 0.0, None
        
        # Преобразование в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Получение диапазонов для целевого цвета
        color_meta = COLOR_RANGES.get(self.target_color)
        if not color_meta:
            return False, 0.0, None
        
        # Создание маски для всех диапазонов
        mask_total = None
        for low, high in color_meta['hsv_ranges']:
            low_arr = np.array(low, dtype=np.uint8)
            high_arr = np.array(high, dtype=np.uint8)
            mask = cv2.inRange(hsv, low_arr, high_arr)
            
            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)
        
        if mask_total is None:
            return False, 0.0, None
        
        # Вычисление процента цвета на кадре
        total_pixels = mask_total.shape[0] * mask_total.shape[1]
        color_pixels = np.count_nonzero(mask_total)
        ratio = color_pixels / total_pixels if total_pixels > 0 else 0.0
        
        # Детекция, если процент превышает порог
        detected = ratio >= MIN_COLOR_RATIO
        
        return detected, ratio, mask_total
    
    def save_detection_photo(self, frame, distance, ratio):
        """Сохраняет фото с обнаружением препятствия"""
        if not self.save_photos or frame is None:
            return None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"obstacle_{self.target_color}_{self.photo_count:04d}_{timestamp}_dist_{distance:.1f}cm_ratio_{ratio:.2f}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        
        # Добавление информации на изображение
        info_text = f"Color: {self.target_color} | Dist: {distance:.1f}cm | Ratio: {ratio:.2%}"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imwrite(filepath, frame)
        self.photo_count += 1
        
        print(f"  [SAVED] {filename}")
        return filepath
    
    def check_obstacle(self):
        """
        Проверяет наличие препятствия по камере и датчику
        Возвращает: (obstacle_detected: bool, distance: float, color_ratio: float)
        """
        # Чтение расстояния
        try:
            distance = read_distance_cm_from_bot()
            if distance is None:
                distance = 999.0
        except Exception as e:
            distance = 999.0
        
        # Захват кадра
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return False, distance, 0.0
        
        # Детекция цвета
        color_detected, ratio, mask = self.detect_color_in_frame(frame)
        
        # Препятствие обнаружено, если:
        # 1. Обнаружен целевой цвет на камере
        # 2. Расстояние меньше порогового
        obstacle_detected = color_detected and distance < DETECTION_DISTANCE_CM
        
        # Сохранение фото при обнаружении
        if obstacle_detected:
            self.save_detection_photo(frame, distance, ratio)
        
        return obstacle_detected, distance, ratio
    
    def run_detection_loop(self, speed=15, scan_interval=0.2):
        """
        Основной цикл детекции с движением
        Робот медленно движется вперед и сканирует препятствия
        """
        print("\n" + "="*60)
        print(f"ДЕТЕКЦИЯ ПРЕПЯТСТВИЙ: {COLOR_RANGES[self.target_color]['description']}")
        print("="*60)
        print(f"Расстояние детекции: {DETECTION_DISTANCE_CM} см")
        print(f"Минимальный процент цвета: {MIN_COLOR_RATIO*100:.1f}%")
        if self.save_photos:
            print(f"Сохранение фото: {self.output_dir}")
        print("Нажмите Ctrl+C для остановки")
        print("="*60 + "\n")
        
        try:
            # Включение LED индикации целевого цвета
            color_meta = COLOR_RANGES.get(self.target_color)
            if color_meta and 'rgb_led' in color_meta:
                r, g, b = color_meta['rgb_led']
                RGB.set_rgb(r, g, b)
                time.sleep(0.3)
            
            last_detection_time = 0
            detection_cooldown = 2.0  # секунды между обнаружениями
            
            while not stop_requested:
                current_time = time.time()
                
                # Проверка препятствия
                obstacle_detected, distance, ratio = self.check_obstacle()
                
                if obstacle_detected:
                    # Препятствие обнаружено!
                    if current_time - last_detection_time > detection_cooldown:
                        self.detection_count += 1
                        print(f"\n[ОБНАРУЖЕНО #{self.detection_count}] Препятствие '{self.target_color}'!")
                        print(f"  Расстояние: {distance:.1f} см")
                        print(f"  Процент цвета: {ratio:.2%}")
                        
                        # Остановка и мигание LED
                        ca.stop_robot()
                        for _ in range(3):
                            RGB.off()
                            time.sleep(0.2)
                            if color_meta and 'rgb_led' in color_meta:
                                r, g, b = color_meta['rgb_led']
                                RGB.set_rgb(r, g, b)
                            time.sleep(0.2)
                        
                        # Объезд препятствия
                        print("  Объезд препятствия...")
                        ca.move_backward(20)
                        time.sleep(0.5)
                        ca.stop_robot()
                        time.sleep(0.2)
                        ca.rotate_right(20)
                        time.sleep(1.0)
                        ca.stop_robot()
                        time.sleep(0.2)
                        
                        last_detection_time = current_time
                        print("  Продолжаю движение...\n")
                else:
                    # Препятствие не обнаружено - продолжаем движение
                    ca.move_forward(speed)
                
                # Вывод информации в реальном времени
                status = f"\rРасст: {distance:>6.1f}см | Цвет: {ratio:>5.1%} | Обнаружено: {self.detection_count}"
                print(status, end='', flush=True)
                
                time.sleep(scan_interval)
        
        except Exception as e:
            print(f"\n[ERROR] Ошибка в цикле детекции: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            ca.stop_robot()
            time.sleep(0.1)
            ca.stop()
            RGB.off()
            
            print("\n" + "="*60)
            print(f"ЗАВЕРШЕНО!")
            print(f"Всего обнаружений: {self.detection_count}")
            if self.save_photos:
                print(f"Сохранено фото: {self.photo_count}")
                print(f"Директория: {self.output_dir}")
            print("="*60 + "\n")
    
    def cleanup(self):
        """Освобождение ресурсов"""
        if self.cap is not None:
            self.cap.release()
        ca.stop()
        RGB.off()

def main():
    parser = argparse.ArgumentParser(
        description="Детекция цветовых препятствий с использованием камеры и ультразвукового датчика"
    )
    parser.add_argument(
        "--color", "-c",
        type=str,
        required=True,
        choices=['white', 'black', 'mixed'],
        help="Цвет препятствия для детекции (white=светлый, black=темно-синий/черный, mixed=пестрый)"
    )
    parser.add_argument(
        "--save-photos", "-s",
        action="store_true",
        help="Сохранять фотографии при обнаружении препятствий"
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Директория для сохранения фото (по умолчанию: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=15,
        help="Скорость движения (по умолчанию: 15)"
    )
    parser.add_argument(
        "--scan-interval",
        type=float,
        default=0.2,
        help="Интервал сканирования в секундах (по умолчанию: 0.2)"
    )
    
    args = parser.parse_args()
    
    # Проверка бота
    if ca.bot is None:
        print("[ERROR] Объект bot не найден. Проверьте подключение к роботу.")
        sys.exit(1)
    
    # Создание детектора
    detector = ObstacleDetector(
        target_color=args.color,
        save_photos=args.save_photos,
        output_dir=args.output_dir
    )
    
    try:
        # Запуск цикла детекции
        detector.run_detection_loop(
            speed=args.speed,
            scan_interval=args.scan_interval
        )
    finally:
        detector.cleanup()

if __name__ == "__main__":
    main()

