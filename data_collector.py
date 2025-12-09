#!/usr/bin/env python3
"""
Скрипт для сбора данных для диплома:
- Робот двигается по прямой
- Делает поворот на месте (360 градусов)
- Снова двигается по прямой
- Во время движения делает фотографии с данными ультразвукового датчика

Использование:
    python3 data_collector.py [--output-dir DIR] [--speed SPEED] [--photo-interval SEC]
"""

import argparse
import time
import os
import sys
from datetime import datetime
import signal

# Импорты модулей проекта
import car_adapter as ca
from ultrasonic import read_distance_cm_from_bot

# Попытка импорта камеры
try:
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    cv2 = None
    print("ПРЕДУПРЕЖДЕНИЕ: OpenCV не установлен. Камера не будет использоваться.")
    CAMERA_AVAILABLE = False

# Конфигурация по умолчанию
DEFAULT_OUTPUT_DIR = "collected_data"
DEFAULT_SPEED = 20
DEFAULT_PHOTO_INTERVAL = 0.5  # секунд между фотографиями
DEFAULT_FORWARD_TIME = 3.0    # время движения вперед в секундах
DEFAULT_ROTATE_TIME = 2.5     # время для полного поворота на 360°
CAMERA_ID = 0
FRAME_W = 640
FRAME_H = 480

# Глобальные переменные
running = True
stop_requested = False

def signal_handler(sig, frame):
    """Обработчик сигналов для корректного завершения"""
    global running, stop_requested
    print("\n[INFO] Получен сигнал остановки, завершаю работу...")
    running = False
    stop_requested = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class DataCollector:
    def __init__(self, output_dir, speed, photo_interval):
        self.output_dir = output_dir
        self.speed = speed
        self.photo_interval = photo_interval
        self.bot = ca.bot
        self.photo_count = 0
        self.cap = None
        
        # Создание директории для сохранения данных
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"[INFO] Создана директория: {output_dir}")
        
        # Инициализация камеры
        if CAMERA_AVAILABLE:
            self.cap = cv2.VideoCapture(CAMERA_ID)
            if not self.cap.isOpened():
                print("[WARN] Не удалось открыть камеру")
                self.cap = None
            else:
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
    
    def capture_photo_with_distance(self):
        """Делает фото и сохраняет его с данными расстояния"""
        if self.cap is None or not CAMERA_AVAILABLE:
            return None
        
        # Получение расстояния
        try:
            distance = read_distance_cm_from_bot()
            if distance is None:
                distance = -1.0
        except Exception as e:
            print(f"[ERROR] Ошибка чтения датчика: {e}")
            distance = -1.0
        
        # Захват кадра
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("[WARN] Не удалось захватить кадр")
            return None
        
        # Формирование имени файла
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"photo_{self.photo_count:04d}_{timestamp}_dist_{distance:.1f}cm.jpg"
        filepath = os.path.join(self.output_dir, filename)
        
        # Сохранение
        cv2.imwrite(filepath, frame)
        self.photo_count += 1
        
        print(f"[PHOTO] {filename}")
        return filepath
    
    def move_forward(self, duration):
        """Движение вперед с фотографированием"""
        print(f"[ACTION] Движение вперед на {duration:.1f} секунд...")
        ca.move_forward(self.speed)
        
        start_time = time.time()
        next_photo_time = start_time
        
        while time.time() - start_time < duration and not stop_requested:
            current_time = time.time()
            
            if current_time >= next_photo_time:
                self.capture_photo_with_distance()
                next_photo_time = current_time + self.photo_interval
            
            time.sleep(0.05)
        
        ca.stop_robot()
        time.sleep(0.2)
        print("[ACTION] Остановка")
    
    def rotate_360(self, duration):
        """Поворот на 360 градусов на месте с фотографированием"""
        print(f"[ACTION] Поворот на 360° на месте ({duration:.1f} секунд)...")
        ca.rotate_right(self.speed)
        
        start_time = time.time()
        next_photo_time = start_time
        
        while time.time() - start_time < duration and not stop_requested:
            current_time = time.time()
            
            if current_time >= next_photo_time:
                self.capture_photo_with_distance()
                next_photo_time = current_time + self.photo_interval
            
            time.sleep(0.05)
        
        ca.stop_robot()
        time.sleep(0.2)
        print("[ACTION] Остановка после поворота")
    
    def run_sequence(self, forward_time, rotate_time):
        """Выполнение полной последовательности действий"""
        print("\n" + "="*60)
        print("НАЧАЛО СБОРА ДАННЫХ")
        print("="*60)
        print(f"Выходная директория: {self.output_dir}")
        print(f"Скорость: {self.speed}")
        print(f"Интервал фото: {self.photo_interval} сек")
        print("="*60 + "\n")
        
        try:
            # Фаза 1: Движение вперед
            if not stop_requested:
                self.move_forward(forward_time)
            
            # Небольшая пауза
            if not stop_requested:
                time.sleep(0.5)
            
            # Фаза 2: Поворот на 360 градусов
            if not stop_requested:
                self.rotate_360(rotate_time)
            
            # Небольшая пауза
            if not stop_requested:
                time.sleep(0.5)
            
            # Фаза 3: Снова движение вперед
            if not stop_requested:
                self.move_forward(forward_time)
            
        except Exception as e:
            print(f"\n[ERROR] Ошибка во время выполнения: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Гарантированная остановка
            ca.stop_robot()
            time.sleep(0.1)
            ca.stop()
            
            print("\n" + "="*60)
            print(f"ЗАВЕРШЕНО! Собрано фотографий: {self.photo_count}")
            print(f"Данные сохранены в: {self.output_dir}")
            print("="*60 + "\n")
    
    def cleanup(self):
        """Освобождение ресурсов"""
        if self.cap is not None:
            self.cap.release()
        ca.stop()

def main():
    parser = argparse.ArgumentParser(
        description="Сбор данных для диплома с камеры и ультразвукового датчика"
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Директория для сохранения фото (по умолчанию: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "--speed", "-s",
        type=int,
        default=DEFAULT_SPEED,
        help=f"Скорость движения (по умолчанию: {DEFAULT_SPEED})"
    )
    parser.add_argument(
        "--photo-interval", "-i",
        type=float,
        default=DEFAULT_PHOTO_INTERVAL,
        help=f"Интервал между фото в секундах (по умолчанию: {DEFAULT_PHOTO_INTERVAL})"
    )
    parser.add_argument(
        "--forward-time", "-f",
        type=float,
        default=DEFAULT_FORWARD_TIME,
        help=f"Время движения вперед в секундах (по умолчанию: {DEFAULT_FORWARD_TIME})"
    )
    parser.add_argument(
        "--rotate-time", "-r",
        type=float,
        default=DEFAULT_ROTATE_TIME,
        help=f"Время поворота на 360° в секундах (по умолчанию: {DEFAULT_ROTATE_TIME})"
    )
    
    args = parser.parse_args()
    
    # Проверка бота
    if ca.bot is None:
        print("[ERROR] Объект bot не найден. Проверьте подключение к роботу.")
        sys.exit(1)
    
    # Создание коллектора
    collector = DataCollector(
        output_dir=args.output_dir,
        speed=args.speed,
        photo_interval=args.photo_interval
    )
    
    try:
        # Запуск последовательности
        collector.run_sequence(
            forward_time=args.forward_time,
            rotate_time=args.rotate_time
        )
    finally:
        collector.cleanup()

if __name__ == "__main__":
    main()

