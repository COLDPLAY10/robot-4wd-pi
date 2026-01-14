#!/usr/bin/env python3
"""
Диагностика лидара - быстрая проверка подключения и работы
"""

import sys
import os
import time

def check_uart_ports():
    """Проверка наличия UART портов"""
    print("\n" + "="*60)
    print("ПРОВЕРКА UART ПОРТОВ")
    print("="*60)

    ports = ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0']
    found_ports = []

    for port in ports:
        if os.path.exists(port):
            print(f"✓ {port} найден")
            found_ports.append(port)

            # Проверка прав
            if os.access(port, os.R_OK | os.W_OK):
                print(f"  └─ Права доступа: OK")
            else:
                print(f"  └─ Права доступа: НЕТ (нужно: sudo chmod 666 {port})")
        else:
            print(f"✗ {port} не найден")

    if not found_ports:
        print("\n⚠ UART порты не найдены!")
        print("Возможные причины:")
        print("1. UART не включен в raspi-config")
        print("2. Драйверы не загружены")
        print("\nРешение: sudo ./setup_lidar.sh")
        return []

    return found_ports


def check_serial_module():
    """Проверка наличия модуля pyserial"""
    print("\n" + "="*60)
    print("ПРОВЕРКА PYTHON МОДУЛЕЙ")
    print("="*60)

    try:
        import serial
        print(f"✓ pyserial установлен (версия: {serial.VERSION})")
        return True
    except ImportError:
        print("✗ pyserial не установлен")
        print("\nУстановка: pip3 install pyserial")
        return False


def test_lidar_connection(ports):
    """Тестирование подключения лидара"""
    print("\n" + "="*60)
    print("ТЕСТИРОВАНИЕ ПОДКЛЮЧЕНИЯ ЛИДАРА")
    print("="*60)

    try:
        from lidar import LidarDriver
    except ImportError as e:
        print(f"✗ Ошибка импорта драйвера: {e}")
        return False

    for port in ports:
        print(f"\nПопытка подключения к {port}...")

        try:
            # Пробуем разные скорости
            for baudrate in [230400, 115200, 256000]:
                print(f"  Скорость: {baudrate}...")

                lidar = LidarDriver(port=port, baudrate=baudrate, timeout=2.0)

                if lidar.connect():
                    print(f"  ✓ Подключено!")

                    # Пробуем читать данные
                    print("  Запуск сканирования...")
                    lidar.start_scan()

                    time.sleep(2)  # Даем время на сбор данных

                    scan = lidar.get_scan()

                    if scan and len(scan) > 0:
                        print(f"  ✓ Данные получены: {len(scan)} точек")

                        # Показываем примеры данных
                        print("\n  Примеры данных (первые 5 точек):")
                        for i, (angle, distance) in enumerate(scan[:5]):
                            print(f"    {i+1}. Угол: {angle:.2f}рад ({angle*180/3.14159:.1f}°), Расстояние: {distance:.3f}м")

                        # Проверяем ближайшее препятствие
                        import numpy as np
                        front = lidar.get_closest_obstacle(-np.pi/6, np.pi/6)
                        if front:
                            print(f"\n  Ближайшее препятствие впереди: {front[1]:.2f}м")

                        lidar.disconnect()

                        print("\n" + "="*60)
                        print("✓ ЛИДАР РАБОТАЕТ КОРРЕКТНО!")
                        print("="*60)
                        print(f"\nИспользуйте: port='{port}', baudrate={baudrate}")
                        return True
                    else:
                        print("  ✗ Данные не получены")
                        lidar.disconnect()
                else:
                    print("  ✗ Не удалось подключиться")

        except Exception as e:
            print(f"  ✗ Ошибка: {e}")

    print("\n" + "="*60)
    print("✗ НЕ УДАЛОСЬ ПОДКЛЮЧИТЬСЯ К ЛИДАРУ")
    print("="*60)
    return False


def print_troubleshooting():
    """Вывод рекомендаций по устранению проблем"""
    print("\n" + "="*60)
    print("УСТРАНЕНИЕ ПРОБЛЕМ")
    print("="*60)
    print("""
1. UART не включен:
   sudo raspi-config
   → Interface Options → Serial Port
   → Login shell: NO, Hardware: YES
   → Reboot

2. Нет прав доступа:
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyAMA0
   Затем выйдите и зайдите снова

3. Консоль занимает порт:
   Запустите: sudo ./setup_lidar.sh

4. Проверьте подключение:
   - TX лидара → RX (GPIO 15, pin 10) RPi
   - RX лидара → TX (GPIO 14, pin 8) RPi
   - VCC → 5V
   - GND → GND

5. Проверьте питание:
   - Лидар должен светиться
   - Лидар должен вращаться

6. Попробуйте другой порт:
   - /dev/ttyAMA0 (основной UART)
   - /dev/ttyS0 (альтернативный)
   - /dev/ttyUSB0 (USB адаптер)

7. Попробуйте другую скорость:
   - 230400 (стандарт для T-MINI Plus)
   - 115200 (альтернатива)
   - 256000 (некоторые модели)
    """)


def main():
    """Главная функция диагностики"""
    print("\n" + "="*70)
    print(" "*15 + "ДИАГНОСТИКА ЛИДАРА YAHBOOM T-MINI PLUS")
    print("="*70)

    # Проверка 1: UART порты
    ports = check_uart_ports()

    # Проверка 2: Python модули
    serial_ok = check_serial_module()

    if not ports or not serial_ok:
        print_troubleshooting()
        sys.exit(1)

    # Проверка 3: Подключение лидара
    success = test_lidar_connection(ports)

    if not success:
        print_troubleshooting()
        sys.exit(1)

    print("\n✓ Диагностика завершена успешно!")
    print("Можете использовать лидар в своих скриптах.")


if __name__ == '__main__':
    main()

