#!/usr/bin/env python3

import serial
import time
import math
import threading
from typing import List, Tuple, Optional


class LidarDriver:
    """
    Исправленный драйвер для T-MINI Plus
    """
    
    def __init__(self, port='/dev/ttyUSB1', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_running = False
        self.scan_data = []
        self.lock = threading.Lock()
        
        # Для отладки
        self.packet_count = 0
        self.angle_test_results = []
        
        print(f"[Lidar] Инициализация на {port}, скорость {baudrate}")
    def connect(self):
        """Подключение"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            print(f"[Lidar] Подключено к {self.port}")
            return True
            
        except Exception as e:
            print(f"[Lidar] Ошибка подключения: {e}")
            return False

    def start_scan(self):
        """Запуск сканирования"""
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False
        
        try:
            print("[Lidar] Запуск сканирования...")
            
            # Стандартные команды для T-MINI
            self.serial.write(b'\xA5\x65')  # Stop
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            
            self.serial.write(b'\xA5\x52')  # Motor on
            time.sleep(1.0)  # Даем время на раскрутку
            
            self.serial.write(b'\xA5\x60')  # Start scan
            time.sleep(0.2)
            
            # Запускаем поток чтения
            self.is_running = True
            thread = threading.Thread(target=self._read_thread, daemon=True)
            thread.start()
            
            print("[Lidar] Сканирование запущено")
            return True
            
        except Exception as e:
            print(f"[Lidar] Ошибка запуска: {e}")
            return False

    def _read_thread(self):
        """Поток чтения данных"""
        buffer = bytearray()
        
        while self.is_running and self.serial and self.serial.is_open:
            try:
                # Читаем данные
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                
                # Обрабатываем пакеты
                while len(buffer) >= 9:
                    # Ищем заголовок AA 55
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        packet = bytes(buffer[:9])
                        self._parse_packet_fixed(packet)
                        buffer = buffer[9:]
                        self.packet_count += 1
                    else:
                        # Пропускаем 1 байт
                        buffer = buffer[1:]
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"[ERROR] В потоке чтения: {e}")
                buffer.clear()
                time.sleep(0.1)

    def _parse_packet_fixed(self, packet):
        """
        Парсинг пакета T-MINI Plus
        """
        if len(packet) != 9:
            return

        if self.packet_count < 5:
            self._analyze_packet(packet)

        distance_mm = packet[4] | (packet[5] << 8)

        angle_deg_v1 = packet[6] * 360.0 / 256.0

        angle_raw = packet[6] | (packet[7] << 8)
        angle_deg_v2 = (angle_raw / 100.0) % 360.0

        if abs(angle_deg_v1 - 102.4) > 5 and abs(angle_deg_v1 - 56.25) > 5:
            angle_deg = angle_deg_v1
        elif abs(angle_deg_v2 - 102.4) > 5 and abs(angle_deg_v2 - 56.25) > 5:
            angle_deg = angle_deg_v2
        else:
            angle_deg = angle_deg_v1

        distance_m = distance_mm / 1000.0

        if 0.05 <= distance_m <= 6.0:
            angle_rad = math.radians(angle_deg)

            with self.lock:
                self.scan_data.append((angle_rad, distance_m))
                if len(self.scan_data) > 360:
                    self.scan_data = self.scan_data[-360:]

            if self.packet_count % 100 == 0:
                print(f"[Lidar] Пакет {self.packet_count}: {angle_deg:.1f}° = {distance_m:.2f}м")

    def _analyze_packet(self, packet):
        """Анализ пакета для отладки"""
        print(f"\n[DEBUG] Пакет {self.packet_count}:")
        print(f"  HEX: {packet.hex()}")
        print(f"  Байты: {' '.join([f'{b:02x}' for b in packet])}")

        distance_mm = packet[4] | (packet[5] << 8)
        angle_byte = packet[6]
        angle_word = packet[6] | (packet[7] << 8)

        print(f"  Расстояние: {distance_mm}мм = {distance_mm/10:.1f}см")
        print(f"  Угол (байт6): {angle_byte} = {angle_byte*360/256:.1f}°")
        print(f"  Угол (байт6-7): {angle_word} = {angle_word/100:.1f}°")

    def get_scan(self) -> List[Tuple[float, float]]:
        """
        Получить текущий скан в радианах (для совместимости с NavigationController)

        Returns:
            List[Tuple[float, float]]: список (угол_в_радианах, расстояние_в_метрах)
        """
        with self.lock:
            return list(self.scan_data)

    def get_scan_degrees(self):
        """Получить скан в градусах"""
        with self.lock:
            # Конвертируем в градусы
            scan_deg = [(math.degrees(a) % 360, d) for a, d in self.scan_data]
            # Сортируем по углу
            scan_deg.sort(key=lambda x: x[0])
            return scan_deg

    def print_scan_info(self):
        """Вывести информацию о скане"""
        scan = self.get_scan_degrees()
        
        if not scan:
            print("[SCAN] Нет данных")
            return
        
        print(f"\n[SCAN] Всего точек: {len(scan)}")

        angle_groups = {}
        for angle, dist in scan:
            angle_int = int(round(angle))
            if angle_int not in angle_groups:
                angle_groups[angle_int] = []
            angle_groups[angle_int].append(dist)

        print("Углы с данными:")
        angles_with_data = sorted(angle_groups.keys())
        for angle in angles_with_data:
            count = len(angle_groups[angle])
            avg_dist = sum(angle_groups[angle]) / count
            print(f"  {angle:3d}°: {count:3d} точек, среднее {avg_dist*100:5.1f} см")
        
        # Проверяем покрытие
        if angles_with_data:
            coverage = len(angles_with_data) / 360.0 * 100
            print(f"\nПокрытие: {coverage:.1f}% ({len(angles_with_data)} из 360 градусов)")
            
            if coverage > 50:
                print("✓ Лидар работает нормально")
            else:
                print("⚠ Лидар покрывает не весь круг")

    def stop_scan(self):
        """Остановка"""
        if not self.is_running:
            return
        
        print("[Lidar] Остановка...")
        self.is_running = False
        time.sleep(0.2)

        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b'\xA5\x65')
                time.sleep(0.1)
                self.serial.write(b'\xA5\x50')
                time.sleep(0.1)
            except Exception as e:
                print(f"[Lidar] Ошибка при остановке: {e}")

        print("[Lidar] Сканирование остановлено")

    def force_stop(self):
        """Принудительная остановка (для экстренных случаев)"""
        print("[Lidar] Принудительная остановка...")
        self.is_running = False

        if self.serial and self.serial.is_open:
            try:
                for _ in range(3):
                    self.serial.write(b'\xA5\x65')
                    time.sleep(0.05)
                    self.serial.write(b'\xA5\x50')
                    time.sleep(0.05)
            except:
                pass

        print("[Lidar] Остановлен")

    def disconnect(self):
        """Отключение"""
        self.stop_scan()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[Lidar] Отключен")


def test_angle_finding():
    """Тест для поиска правильного угла"""
    print("="*60)
    print("ПОИСК ПРАВИЛЬНОГО УГЛА T-MINI PLUS")
    print("="*60)
    
    testLidar = LidarDriver('/dev/ttyUSB1', 230400)

    if not testLidar.connect():
        print("Не удалось подключиться")
        return
    
    print("\n1. Пробуем разные команды и смотрим ответ...")
    
    # Тестовые команды
    test_commands = [
        (b'\xA5\x52', "Включить мотор"),
        (b'\xA5\x50', "Выключить мотор"),
        (b'\xA5\x20', "Тестовая команда"),
        (b'\xA5\x60', "Начать сканирование"),
    ]
    
    for cmd, desc in test_commands:
        print(f"  Отправка: {desc}")
        testLidar.serial.write(cmd)
        time.sleep(0.2)
        
        if testLidar.serial.in_waiting:
            response = testLidar.serial.read(testLidar.serial.in_waiting)
            print(f"    Ответ: {response.hex()[:50]}...")
    
    print("\n2. Запускаем сканирование и собираем данные...")
    
    if testLidar.start_scan():
        print("Собираем данные 10 секунд...")
        time.sleep(10)
        
        testLidar.stop_scan()
        
        print("\n3. Анализируем собранные данные...")
        testLidar.print_scan_info()
        
        # Показываем примеры точек
        scan = testLidar.get_scan_degrees()
        if scan:
            print("\nПримеры точек:")
            step = max(1, len(scan) // 10)
            for i in range(0, len(scan), step):
                if i < len(scan):
                    angle, dist = scan[i]
                    print(f"  {angle:6.1f}°: {dist*100:5.1f} см")
    else:
        print("Не удалось запустить сканирование")
    
    testLidar.disconnect()


def test_simple_scan():
    """Простой тест сканирования"""
    print("="*60)
    print("ПРОСТОЙ ТЕСТ СКАНИРОВАНИЯ")
    print("="*60)
    
    testLidar = LidarDriver('/dev/ttyUSB1', 230400)

    if not testLidar.connect():
        return
    
    if testLidar.start_scan():
        try:
            # Собираем данные
            for i in range(20):  # 10 секунд
                time.sleep(0.5)
                
                scan = testLidar.get_scan_degrees()
                if scan:
                    print(f"[{i+1}/20] Точек: {len(scan)}")
                    
                    # Минимальное расстояние
                    if scan:
                        dists = [d for _, d in scan]
                        min_dist = min(dists) * 100  # в см
                        max_dist = max(dists) * 100
                        print(f"  Расстояния: {min_dist:.1f}-{max_dist:.1f} см")
                        
                        # Проверяем углы
                        angles = [a for a, _ in scan]
                        if angles:
                            min_angle = min(angles)
                            max_angle = max(angles)
                            print(f"  Углы: {min_angle:.1f}°-{max_angle:.1f}°")
                else:
                    print(f"[{i+1}/20] Нет данных")
                    
        except KeyboardInterrupt:
            print("\nПрервано пользователем")
        finally:
            testLidar.stop_scan()
    else:
        print("Не удалось запустить сканирование")
    
    testLidar.disconnect()


def brute_force_angle_find():
    """
    Перебор всех возможных вариантов парсинга угла
    """
    print("="*60)
    print("ПЕРЕБОР ВАРИАНТОВ УГЛА")
    print("="*60)
    
    # Примеры пакетов из вашего лога
    test_packets = [
        "aa5500282da0b5a820",  # Пакет 0
        "aa550028fba871b102",  # Пакет 1
        "aa55000bb7b1010031",  # Пакет 2
        "aa553101df2adf2a50",  # Пакет с углом 305 (0x0131)
    ]
    
    for packet_hex in test_packets:
        packet = bytes.fromhex(packet_hex)
        print(f"\nАнализ пакета: {packet_hex}")
        
        # Перебираем все возможные варианты где может быть угол
        for i in range(2, 9):  # Байты 2-8
            # Вариант 1: один байт как угол (0-255 -> 0-360)
            angle1 = packet[i] * 360.0 / 256.0
            
            # Вариант 2: два байта как угол (little-endian)
            if i+1 < 9:
                angle_word = packet[i] | (packet[i+1] << 8)
                
                # Разные преобразования
                conversions = [
                    ("/100", angle_word / 100.0),
                    ("*360/65535", angle_word * 360.0 / 65535.0),
                    ("/64", angle_word / 64.0),
                    ("raw", angle_word),
                ]
                
                for conv_name, angle_deg in conversions:
                    if 0 <= angle_deg <= 360:
                        # Проверяем, не постоянное ли это значение
                        if abs(angle_deg - 102.4) > 0.1 and abs(angle_deg - 56.25) > 0.1:
                            print(f"  Байты[{i}:{i+1}] {conv_name}: {angle_deg:.2f}° (сырое: {angle_word})")
            
            # Проверяем одиночный байт
            if 0 <= angle1 <= 360 and abs(angle1 - 102.4) > 0.1:
                print(f"  Байт[{i}] как угол: {angle1:.2f}° (значение: {packet[i]})")


if __name__ == '__main__':
    print("Выберите тест:")
    print("1. Поиск правильного угла (рекомендуется)")
    print("2. Простой тест сканирования")
    print("3. Перебор вариантов угла")
    print("4. Проверить другой протокол (попробовать 115200)")
    
    choice = input("Введите 1-4: ").strip()
    
    if choice == '1':
        test_angle_finding()
    elif choice == '2':
        test_simple_scan()
    elif choice == '3':
        brute_force_angle_find()
    elif choice == '4':
        # Пробуем другую скорость
        print("\nПробуем скорость 115200...")
        lidar = LidarDriver('/dev/ttyUSB1', 115200)
        if lidar.connect():
            print("✓ Подключено на 115200")
            lidar.disconnect()
        else:
            print("✗ Не работает на 115200")
    else:
        test_angle_finding()