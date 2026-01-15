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

    ports = ['/dev/ttyUSB1', '/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/ttyS0']  # Изменил порядок, сначала USB
    found_ports = []

    for port in ports:
        if os.path.exists(port):
            print(f"✓ {port} найден")
            found_ports.append(port)

            # Проверка прав
            try:
                if os.access(port, os.R_OK | os.W_OK):
                    print(f"  └─ Права доступа: OK")
                else:
                    print(f"  └─ Права доступа: НЕТ (нужно: sudo chmod 666 {port})")
            except:
                print(f"  └─ Не удалось проверить права")

        else:
            print(f"✗ {port} не найден")

    if not found_ports:
        print("\n⚠ UART порты не найдены!")
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
        return False


def test_lidar_connection(ports):
    """Тестирование подключения лидара"""
    print("\n" + "="*60)
    print("ТЕСТИРОВАНИЕ ПОДКЛЮЧЕНИЯ ЛИДАРА")
    print("="*60)

    try:
        # Пытаемся импортировать драйвер
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from lidar import LidarDriver
        print("✓ Драйвер лидара загружен")
    except ImportError as e:
        print(f"✗ Ошибка импорта драйвера: {e}")
        print(f"  Путь поиска: {sys.path[-1]}")
        return False
    except Exception as e:
        print(f"✗ Другая ошибка импорта: {e}")
        return False

    for port in ports:
        print(f"\nПопытка подключения к {port}...")

        try:
            # Пробуем разные скорости
            for baudrate in [230400, 115200, 256000, 9600]:
                print(f"  Скорость: {baudrate}...")

                lidar = LidarDriver(port=port, baudrate=baudrate, timeout=2.0)

                if lidar.connect():
                    print(f"  ✓ Подключено!")

                    # Пробуем читать данные
                    print("  Запуск сканирования...")
                    if lidar.start_scan():
                        print("  ✓ Сканирование запущено")
                        
                        # Даем время на сбор данных
                        time.sleep(2)
                        
                        # Проверяем наличие данных
                        scan = lidar.get_scan()
                        print(f"  Получено точек: {len(scan) if scan else 0}")

                        if scan and len(scan) > 0:
                            print(f"  ✓ Данные получены: {len(scan)} точек")

                            # Показываем примеры данных
                            print("\n  Примеры данных (первые 3 точки):")
                            for i, (angle, distance) in enumerate(scan[:3]):
                                angle_deg = angle * 180 / 3.14159
                                print(f"    {i+1}. Угол: {angle:.2f}рад ({angle_deg:.1f}°), Расстояние: {distance:.3f}м")

                            # Проверяем расстояние вперед
                            front = lidar.get_front_distance()
                            if front:
                                print(f"\n  Расстояние вперед (±15°): {front:.2f}м")
                            else:
                                print(f"\n  Нет данных вперед")

                            lidar.stop_scan()
                            lidar.disconnect()

                            print("\n" + "="*60)
                            print("✓ ЛИДАР РАБОТАЕТ КОРРЕКТНО!")
                            print("="*60)
                            print(f"\nИспользуйте: port='{port}', baudrate={baudrate}")
                            return True
                        else:
                            print("  ✗ Данные не получены")
                            lidar.stop_scan()
                            lidar.disconnect()
                    else:
                        print("  ✗ Не удалось запустить сканирование")
                        lidar.disconnect()
                else:
                    print("  ✗ Не удалось подключиться")

        except Exception as e:
            print(f"  ✗ Ошибка: {e}")
            import traceback
            print(f"  Детали: {traceback.format_exc()}")

    print("\n" + "="*60)
    print("✗ НЕ УДАЛОСЬ ПОДКЛЮЧИТЬСЯ К ЛИДАРУ")
    print("="*60)
    return False


def check_lidar_raw_data():
    """Проверка сырых данных лидара"""
    print("\n" + "="*60)
    print("ПРОВЕРКА СЫРЫХ ДАННЫХ ЛИДАРА")
    print("="*60)
    
    try:
        import serial
        
        port = '/dev/ttyUSB1'
        if not os.path.exists(port):
            print(f"✗ Порт {port} не найден")
            return False
            
        print(f"Открываю {port} на скорости 230400...")
        
        # Просто читаем сырые данные без парсинга
        with serial.Serial(port, 230400, timeout=2.0) as ser:
            ser.reset_input_buffer()
            
            # Отправляем команду старта
            ser.write(b'\xA5\x60')
            time.sleep(0.5)
            
            # Читаем данные
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"Получено {len(data)} байт:")
                print(f"HEX: {data.hex()[:100]}...")
                
                # Ищем паттерны
                if b'\xAA\x55' in data:
                    print("✓ Найден паттерн AA 55")
                elif b'\xA5\x5A' in data:
                    print("✓ Найден паттерн A5 5A")
                else:
                    print("✗ Неизвестный формат данных")
                    
                # Останавливаем
                ser.write(b'\xA5\x65')
                return True
            else:
                print("✗ Нет данных от лидара")
                return False
                
    except Exception as e:
        print(f"✗ Ошибка: {e}")
        return False


def print_troubleshooting():
    """Вывод рекомендаций по устранению проблем"""
    print("\n" + "="*60)
    print("УСТРАНЕНИЕ ПРОБЛЕМ")
    print("="*60)
    print("""
1. Проверьте подключение кабеля:
   - Убедитесь, что лидар подключен к USB порту
   - Попробуйте другой USB порт

2. Проверьте питание:
   - Лидар должен светиться
   - Лидар должен издавать звук вращения

3. Проверьте драйверы USB:
   - lsusb | grep -i lidar
   - dmesg | grep ttyUSB

4. Проверьте права:
   - sudo chmod 666 /dev/ttyUSB1
   - sudo usermod -a -G dialout $USER

5. Попробуйте другой порт:
   - /dev/ttyUSB1 (обычно лидар)
   - /dev/ttyUSB0
   - ls -la /dev/ttyUSB*

6. Попробуйте другую скорость:
   - 230400 (стандарт)
   - 115200
   - 256000
   
7. Если лидар не вращается:
   - Проверьте питание 5V
   - Мотор лидара может быть отключен
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

    if not ports:
        print("\n⚠ Порты не найдены, проверяем только сырые данные...")
        success = check_lidar_raw_data()
        if not success:
            print_troubleshooting()
            sys.exit(1)
        else:
            sys.exit(0)
            
    if not serial_ok:
        print("\nУстановите pyserial: pip3 install pyserial")
        sys.exit(1)

    # Проверка 3: Сначала сырые данные
    print("\nСначала проверяем сырые данные...")
    if check_lidar_raw_data():
        print("✓ Сырые данные есть, проверяем драйвер...")
    else:
        print("✗ Нет сырых данных, проблема в подключении")
        print_troubleshooting()
        sys.exit(1)

    # Проверка 4: Подключение через драйвер
    success = test_lidar_connection(ports)

    if not success:
        print("\nНе удалось получить данные через драйвер, но сырые данные есть.")
        print("Возможно, проблема в парсинге данных.")
        print_troubleshooting()
        
        # Проверка версии драйвера
        print("\nПроверьте файл lidar.py:")
        print("1. Убедитесь, что метод _reading_thread() корректно парсит данные")
        print("2. Проверьте формат пакетов: AA 55 ...")
        print("3. Уменьшите минимальное расстояние в фильтре: if 0.05 < distance_m < 6.0")
        
        sys.exit(1)

    print("\n✓ Диагностика завершена успешно!")
    print("Можете использовать лидар в своих скриптах.")


if __name__ == '__main__':
    main()