#!/usr/bin/env python3
"""
Драйвер для лидара Yahboom T-MINI Plus
Поддерживает чтение данных сканирования через UART
"""

import serial
import time
import struct
import numpy as np
from typing import List, Tuple, Optional
import threading


class LidarDriver:
    """
    Драйвер для лидара Yahboom T-MINI Plus

    Лидар возвращает данные в виде массива углов и расстояний.
    Протокол: обычно через UART на скорости 230400 или 115200 бaud
    """

    def __init__(self, port='/dev/ttyAMA0', baudrate=230400, timeout=1.0):
        """
        Args:
            port: последовательный порт (обычно /dev/ttyAMA0 или /dev/ttyUSB0)
            baudrate: скорость передачи данных
            timeout: таймаут чтения
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.is_running = False
        self.last_scan = []
        self.lock = threading.Lock()

        # Параметры лидара
        self.min_distance = 0.15  # минимальная дистанция в метрах
        self.max_distance = 12.0   # максимальная дистанция в метрах
        self.angle_resolution = 1.0  # разрешение по углу в градусах

        print(f"[Lidar] Инициализация на порту {port} со скоростью {baudrate}")

    def connect(self) -> bool:
        """
        Подключение к лидару

        Returns:
            True если подключение успешно
        """
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            time.sleep(0.5)  # Даем время на инициализацию

            # Очищаем буфер
            if self.serial_port.in_waiting > 0:
                self.serial_port.reset_input_buffer()

            print(f"[Lidar] Подключен к {self.port}")
            return True

        except serial.SerialException as e:
            print(f"[Lidar] Ошибка подключения: {e}")
            return False
        except Exception as e:
            print(f"[Lidar] Неожиданная ошибка: {e}")
            return False

    def disconnect(self):
        """Отключение от лидара"""
        self.stop_scan()

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("[Lidar] Отключен")
            except Exception as e:
                print(f"[Lidar] Ошибка при отключении: {e}")

    def start_scan(self):
        """Запуск непрерывного сканирования в отдельном потоке"""
        if self.is_running:
            print("[Lidar] Сканирование уже запущено")
            return

        if not self.serial_port or not self.serial_port.is_open:
            if not self.connect():
                print("[Lidar] Не удалось подключиться")
                return

        self.is_running = True
        self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.scan_thread.start()
        print("[Lidar] Сканирование запущено")

    def stop_scan(self):
        """Остановка сканирования"""
        if self.is_running:
            self.is_running = False
            if hasattr(self, 'scan_thread'):
                self.scan_thread.join(timeout=2.0)
            print("[Lidar] Сканирование остановлено")

    def _scan_loop(self):
        """Основной цикл сканирования (выполняется в отдельном потоке)"""
        while self.is_running:
            try:
                scan_data = self._read_single_scan()
                if scan_data:
                    with self.lock:
                        self.last_scan = scan_data
            except Exception as e:
                print(f"[Lidar] Ошибка чтения: {e}")
                time.sleep(0.1)

    def _read_single_scan(self) -> Optional[List[Tuple[float, float]]]:
        """
        Чтение одного полного скана

        Returns:
            Список кортежей (угол_в_радианах, расстояние_в_метрах)
            или None при ошибке
        """
        if not self.serial_port or not self.serial_port.is_open:
            return None

        try:
            # Ищем заголовок пакета
            # Для Yahboom T-MINI Plus обычно используется протокол с заголовком
            # Формат пакета зависит от конкретной модели
            # Здесь используется типичный формат для китайских лидаров

            # Ждем начала пакета (обычно 0x54 или 0xAA 0x55)
            header = self._wait_for_header()
            if not header:
                return None

            # Читаем данные пакета
            scan_data = self._parse_packet()
            return scan_data

        except Exception as e:
            print(f"[Lidar] Ошибка парсинга: {e}")
            return None

    def _wait_for_header(self, max_attempts=100) -> bool:
        """
        Ожидание заголовка пакета

        Протокол для T-MINI Plus:
        - Заголовок: 0x54 (84 decimal) - стандартный для многих китайских лидаров
        """
        for _ in range(max_attempts):
            if not self.serial_port or not self.serial_port.is_open:
                return False

            byte = self.serial_port.read(1)
            if len(byte) == 0:
                continue

            if byte[0] == 0x54:  # Типичный заголовок
                return True

        return False

    def _parse_packet(self) -> List[Tuple[float, float]]:
        """
        Парсинг пакета данных от лидара

        Типичный формат пакета для китайских 2D лидаров:
        [Header(1)] [Length(1)] [Speed(2)] [StartAngle(2)] [Data(N*3)] [EndAngle(2)] [Timestamp(2)] [CRC(1)]

        Где Data содержит N измерений по 3 байта: [Distance(2)] [Intensity(1)]
        """
        scan_data = []

        try:
            # Читаем длину пакета (обычно следует после заголовка)
            length_byte = self.serial_port.read(1)
            if len(length_byte) == 0:
                return scan_data

            packet_length = length_byte[0]

            # Читаем остальные данные пакета
            packet_data = self.serial_port.read(packet_length - 1)
            if len(packet_data) < packet_length - 1:
                return scan_data

            # Парсим данные (формат зависит от конкретной модели)
            # Для упрощения используем базовый формат

            # Скорость вращения (2 байта)
            speed = struct.unpack('<H', packet_data[0:2])[0]

            # Начальный угол (2 байта, в 0.01 градуса)
            start_angle_raw = struct.unpack('<H', packet_data[2:4])[0]
            start_angle = (start_angle_raw / 100.0) * np.pi / 180.0  # в радианы

            # Вычисляем количество точек данных
            # Обычно: (длина - заголовок - углы - CRC) / 3
            data_length = packet_length - 8  # вычитаем служебные байты
            num_points = data_length // 3

            if num_points <= 0:
                return scan_data

            # Конечный угол
            end_angle_offset = 4 + num_points * 3
            if end_angle_offset + 1 < len(packet_data):
                end_angle_raw = struct.unpack('<H', packet_data[end_angle_offset:end_angle_offset+2])[0]
                end_angle = (end_angle_raw / 100.0) * np.pi / 180.0
            else:
                end_angle = start_angle + num_points * (1.0 * np.pi / 180.0)

            # Вычисляем шаг угла
            if num_points > 1:
                angle_step = (end_angle - start_angle) / (num_points - 1)
            else:
                angle_step = 0

            # Парсим точки данных
            for i in range(num_points):
                offset = 4 + i * 3
                if offset + 2 < len(packet_data):
                    # Расстояние (2 байта, в мм)
                    distance_mm = struct.unpack('<H', packet_data[offset:offset+2])[0]
                    distance_m = distance_mm / 1000.0

                    # Фильтруем недопустимые значения
                    if self.min_distance <= distance_m <= self.max_distance:
                        angle = start_angle + i * angle_step
                        # Нормализуем угол в диапазон [0, 2*pi]
                        angle = angle % (2 * np.pi)
                        scan_data.append((angle, distance_m))

            return scan_data

        except Exception as e:
            print(f"[Lidar] Ошибка парсинга пакета: {e}")
            return scan_data

    def get_scan(self) -> List[Tuple[float, float]]:
        """
        Получение последнего скана

        Returns:
            Список кортежей (угол_в_радианах, расстояние_в_метрах)
        """
        with self.lock:
            return self.last_scan.copy()

    def get_scan_cartesian(self) -> List[Tuple[float, float]]:
        """
        Получение последнего скана в декартовых координатах

        Returns:
            Список кортежей (x, y) в метрах
        """
        scan_data = self.get_scan()
        cartesian_data = []

        for angle, distance in scan_data:
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            cartesian_data.append((x, y))

        return cartesian_data

    def get_closest_obstacle(self, angle_min=None, angle_max=None) -> Optional[Tuple[float, float]]:
        """
        Получение ближайшего препятствия в заданном секторе

        Args:
            angle_min: минимальный угол в радианах (None = 0)
            angle_max: максимальный угол в радианах (None = 2*pi)

        Returns:
            Кортеж (угол, расстояние) или None
        """
        scan_data = self.get_scan()

        if not scan_data:
            return None

        if angle_min is None:
            angle_min = 0
        if angle_max is None:
            angle_max = 2 * np.pi

        closest = None
        min_distance = float('inf')

        for angle, distance in scan_data:
            if angle_min <= angle <= angle_max:
                if distance < min_distance:
                    min_distance = distance
                    closest = (angle, distance)

        return closest

    def __enter__(self):
        """Поддержка контекстного менеджера"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Поддержка контекстного менеджера"""
        self.disconnect()


def test_lidar():
    """Тестовая функция для проверки работы лидара"""
    print("="*60)
    print("ТЕСТ ЛИДАРА YAHBOOM T-MINI PLUS")
    print("="*60)

    # Попробуем разные порты
    ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyS0']

    lidar = None
    for port in ports:
        print(f"\nПопытка подключения к {port}...")
        lidar = LidarDriver(port=port)
        if lidar.connect():
            break
        lidar = None

    if not lidar:
        print("\n[ERROR] Не удалось подключиться к лидару")
        print("Проверьте:")
        print("  1. Подключен ли лидар к Raspberry Pi")
        print("  2. Правильность порта (обычно /dev/ttyAMA0)")
        print("  3. Включен ли UART в raspi-config")
        return

    try:
        lidar.start_scan()

        print("\nЧтение данных (10 секунд)...")
        start_time = time.time()

        while time.time() - start_time < 10:
            scan = lidar.get_scan()

            if scan:
                print(f"\r[Lidar] Точек: {len(scan):4d}", end='', flush=True)

                # Проверяем ближайшее препятствие впереди (±30 градусов)
                front_obstacle = lidar.get_closest_obstacle(
                    angle_min=-np.pi/6,
                    angle_max=np.pi/6
                )

                if front_obstacle:
                    angle, dist = front_obstacle
                    print(f" | Впереди: {dist*100:.1f} см   ", end='', flush=True)

            time.sleep(0.1)

        print("\n\nПоследний скан (декартовы координаты):")
        cartesian = lidar.get_scan_cartesian()
        for i, (x, y) in enumerate(cartesian[:10]):  # Показываем первые 10 точек
            print(f"  Точка {i}: x={x:.3f}м, y={y:.3f}м")

        if len(cartesian) > 10:
            print(f"  ... и еще {len(cartesian) - 10} точек")

    except KeyboardInterrupt:
        print("\n\nПрервано пользователем")

    finally:
        lidar.disconnect()
        print("\n" + "="*60)
        print("ТЕСТ ЗАВЕРШЕН")
        print("="*60)


if __name__ == '__main__':
    test_lidar()

