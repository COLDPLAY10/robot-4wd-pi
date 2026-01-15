#!/usr/bin/env python3
"""
Демонстрационный скрипт автономной навигации с лидаром
Использует лидар Yahboom T-MINI Plus для построения карты и навигации
"""

import sys
import os
import time
import signal

# ИСПРАВЛЕНО: Добавляем путь к родительской директории
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from map.navigation_controller import NavigationController
import car_adapter as ca

running = True


def signal_handler(sig, frame):
    """Обработчик для корректного завершения"""
    global running
    print("\n[INFO] Остановка...")
    running = False


def exploration_with_lidar_demo():
    """
    Демонстрация навигации с лидаром:
    - Робот использует лидар для построения карты
    - Объединяет данные лидара, ультразвука и камеры
    - Строит детальную карту окружения
    """
    print("\n" + "="*60)
    print("ДЕМО: АВТОНОМНАЯ НАВИГАЦИЯ С ЛИДАРОМ")
    print("="*60)
    print("Робот будет исследовать окружение")
    print("Используется: ЛИДАР + ультразвук + камера")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    # Инициализация контроллера с лидаром
    controller = NavigationController(
        use_lidar=True,
        use_camera=True,
        use_ultrasonic=True
    )

    # Запуск режима исследования
    controller.start_exploration()

    last_status_time = time.time()
    status_interval = 5.0  # Выводить статус каждые 5 секунд

    try:
        while running:
            # Обновление системы навигации
            controller.update()

            # Периодический вывод статуса
            current_time = time.time()
            if current_time - last_status_time >= status_interval:
                status = controller.get_status()
                print("\n" + "-"*60)
                print(f"Режим: {status['mode']}")
                pos = status['position']
                print(f"Позиция: x={pos[0]:.2f}м, y={pos[1]:.2f}м, θ={pos[2]:.2f}рад")
                if status['obstacle_distance'] is not None:
                    print(f"Препятствие впереди: {status['obstacle_distance']:.2f}м")
                print("-"*60)
                last_status_time = current_time

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("\n[INFO] Остановка по запросу пользователя")
    finally:
        controller.stop()

        # Сохраняем карту
        map_file = f"map_exploration_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(map_file)

        print(f"\n[INFO] Карта сохранена: {map_file}")
        print(f"[INFO] Используйте: python3 visualize_map.py --map {map_file}")


def goto_with_lidar_demo(target_x: float, target_y: float):
    """
    Демонстрация навигации к цели с лидаром

    Args:
        target_x, target_y: координаты цели в метрах
    """
    print("\n" + "="*60)
    print("ДЕМО: НАВИГАЦИЯ К ЦЕЛИ С ЛИДАРОМ")
    print("="*60)
    print(f"Цель: ({target_x:.2f}, {target_y:.2f})")
    print("Используется: ЛИДАР + ультразвук + камера")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    # Инициализация контроллера
    controller = NavigationController(
        use_lidar=True,
        use_camera=True,
        use_ultrasonic=True
    )

    # Устанавливаем цель
    controller.set_goal(target_x, target_y)

    last_status_time = time.time()
    status_interval = 2.0

    try:
        while running:
            controller.update()

            status = controller.get_status()

            current_time = time.time()
            if current_time - last_status_time >= status_interval:
                print("\n" + "-"*60)
                print(f"Режим: {status['mode']}")
                pos = status['position']
                print(f"Позиция: x={pos[0]:.2f}м, y={pos[1]:.2f}м")
                print(f"Цель: x={target_x:.2f}м, y={target_y:.2f}м")
                print(f"Waypoint: {status['waypoint']}")
                print("-"*60)
                last_status_time = current_time

            # Проверяем, достигли ли цели
            if status['mode'] == 'idle':
                print("\n[SUCCESS] Цель достигнута!")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Остановка по запросу пользователя")
    finally:
        controller.stop()

        # Сохраняем карту
        map_file = f"map_navigation_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(map_file)

        print(f"\n[INFO] Карта сохранена: {map_file}")
        print(f"[INFO] Используйте: python3 visualize_map.py --map {map_file}")


def main():
    """Главная функция"""
    signal.signal(signal.SIGINT, signal_handler)

    if len(sys.argv) < 2:
        print("Использование:")
        print("  python3 demo_with_lidar.py explore")
        print("  python3 demo_with_lidar.py goto X Y")
        print()
        print("Примеры:")
        print("  python3 demo_with_lidar.py explore")
        print("  python3 demo_with_lidar.py goto 2.0 1.0")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode == "explore" or mode == "exploration":
        exploration_with_lidar_demo()

    elif mode == "goto":
        if len(sys.argv) < 4:
            print("Ошибка: для режима goto требуются координаты X Y")
            print("Использование: python3 demo_with_lidar.py goto X Y")
            sys.exit(1)

        try:
            target_x = float(sys.argv[2])
            target_y = float(sys.argv[3])
            goto_with_lidar_demo(target_x, target_y)
        except ValueError:
            print("Ошибка: координаты должны быть числами")
            sys.exit(1)

    else:
        print(f"Неизвестный режим: {mode}")
        print("Доступные режимы: explore, goto")
        sys.exit(1)


if __name__ == '__main__':
    main()

