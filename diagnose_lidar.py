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

    ports = ['/dev/oradar', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB1', '/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/ttyS0']
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

                            # Расстояние примерно "вперёд" (±15° вокруг 0°).
                            # get_scan() возвращает (угол_рад, дистанция_м).
                            import math as _m
                            front_pts = [d for a, d in scan
                                         if abs(((_m.degrees(a) + 180) % 360) - 180) <= 15]
                            if front_pts:
                                print(f"\n  Расстояние вперёд (±15°): {min(front_pts):.2f}м")
                            else:
                                print(f"\n  Нет точек в секторе ±15° вперёд")

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


# Лидар — YDLidar T-mini Plus: команды A5 xx, фреймы с заголовком AA 55
# (см. lidar.py и samples/ANALYSIS.md). MS200-сигнатуру (54 2C) оставляем
# вторичной проверкой на случай другого экземпляра железа.
_YD_START = b'\xA5\x60'   # старт сканирования
_YD_MOTOR = b'\xA5\x52'   # мотор (последовательность из lidar.py)
_YD_STOP = b'\xA5\x65'    # стоп — посылаем после пробы, чтобы не оставлять мотор
# Заголовки ВЫНЕСЕНЫ в константы: внутри f-string обратный слэш
# (b'\xAA\x55') запрещён в Python < 3.12 — отсюда был SyntaxError.
_YD_HDR = bytes([0xAA, 0x55])
_MS200_HDR = bytes([0x54, 0x2C])
# Скорости для перебора. T-mini Plus штатно 230400, остальные — на случай
# иной прошивки; поток сплошных 0x00 = признак чужого устройства или
# несовпадения скорости.
_BAUDS = [230400, 460800, 921600, 256000, 115200, 128000]


def _probe_port_for_lidar(port):
    """Перебрать скорости, послать YDLidar-команды старта и поискать заголовки
    AA 55 (YDLidar) или 54 2C (MS200). Возвращает (True, baud) при успехе,
    иначе (False, None)."""
    import serial
    for baud in _BAUDS:
        try:
            with serial.Serial(port, baud, timeout=2.0) as ser:
                ser.reset_input_buffer()
                try:
                    ser.write(_YD_STOP); ser.flush(); time.sleep(0.1)
                    ser.reset_input_buffer()
                    ser.write(_YD_MOTOR); ser.flush(); time.sleep(0.5)
                    ser.write(_YD_START); ser.flush()
                except Exception:
                    pass
                time.sleep(1.5)  # время на раскрутку мотора и поток данных
                data = ser.read(8192)
                try:
                    ser.write(_YD_STOP); ser.flush()
                except Exception:
                    pass
        except Exception as e:
            print(f"  {port} @ {baud}: ошибка открытия — {e}")
            return (False, None)

        if not data:
            print(f"  {port} @ {baud}: 0 байт (тишина)")
            continue
        # ≥3 заголовков, чтобы случайное совпадение байтов не сошло за лидар
        n_yd = data.count(_YD_HDR)
        if n_yd >= 3:
            print(f"  {port} @ {baud}: ✓ ЛИДАР (YDLidar)! заголовок AA 55 x {n_yd} "
                  f"({len(data)} байт)")
            return (True, baud)
        n_ms = data.count(_MS200_HDR)
        if n_ms >= 3:
            print(f"  {port} @ {baud}: ✓ ЛИДАР (MS200)! заголовок 54 2C x {n_ms} "
                  f"({len(data)} байт) — это Oradar MS200, текущий драйвер "
                  f"lidar.py (YDLidar) с ним не работает!")
            return (True, baud)
        # Чуть-чуть HEX для диагностики (без backslash в f-string).
        head = data[:20].hex()
        zeros = data.count(0)
        print(f"  {port} @ {baud}: {len(data)} байт, не похоже на лидар "
              f"(нулей {zeros}/{len(data)}, начало {head})")
    return (False, None)


def check_lidar_raw_data():
    """Проверка сырых данных лидара на ВСЕХ портах и скоростях."""
    print("\n" + "="*60)
    print("ПРОВЕРКА СЫРЫХ ДАННЫХ ЛИДАРА (скан портов x скоростей)")
    print("="*60)

    import glob
    candidates = (['/dev/oradar']
                  + sorted(glob.glob('/dev/ttyACM*'))
                  + sorted(glob.glob('/dev/ttyUSB*'))
                  + ['/dev/ttyAMA0'])
    seen = set()
    candidates = [p for p in candidates
                  if os.path.exists(p) and not (p in seen or seen.add(p))]

    if not candidates:
        print("✗ Ни одного serial-порта не найдено вообще.")
        print("  Лидар не перечислен в системе — проверьте `lsusb` и кабель/USB-хаб.")
        return False

    print(f"Порты-кандидаты: {', '.join(candidates)}\n")
    for port in candidates:
        ok, baud = _probe_port_for_lidar(port)
        if ok:
            print(f"\n✓ ЛИДАР НАЙДЕН: порт {port}, скорость {baud}")
            print(f"  Пропишите этот порт/скорость в lidar.py и navigation_controller.")
            return True

    print("\n✗ Ни на одном порту/скорости нет потока лидара (AA 55 / 54 2C).")
    print("  Лидар — это CP210x (10c4:ea60). Если все CP210x-порты")
    print("  молчат или шлют нули — проверьте кабель лидара и его 5V-питание.")
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
