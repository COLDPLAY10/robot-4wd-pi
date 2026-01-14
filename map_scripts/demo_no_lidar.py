#!/usr/bin/env python3
"""
Демонстрационный скрипт автономной навигации без лидара
Использует только ультразвуковой датчик и камеру
"""

import sys
import time
import signal

from map.navigation_controller import NavigationController
import car_adapter as ca

running = True


def signal_handler(sig, frame):
    """Обработчик для корректного завершения"""
    global running
    print("\n[INFO] Остановка...")
    running = False


def simple_exploration_demo():
    """
    Простая демонстрация:
    - Робот едет вперед
    - При обнаружении препятствия поворачивает
    - Строит карту окружения
    """
    print("\n" + "="*60)
    print("ДЕМО: АВТОНОМНАЯ НАВИГАЦИЯ БЕЗ ЛИДАРА")
    print("="*60)
    print("Робот будет исследовать окружение")
    print("Используется: ультразвук + камера (если доступна)")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    # Инициализация контроллера без лидара
    controller = NavigationController(
        use_lidar=False,
        use_camera=True,
        use_ultrasonic=True
    )

    # Запуск режима исследования
    controller.start_exploration()

    last_status_time = time.time()
    iteration = 0

    try:
        while running:
            # Обновляем контроллер
            controller.update()

            # Выводим статус каждые 5 секунд
            current_time = time.time()
            if current_time - last_status_time > 5.0:
                status = controller.get_status()
                iteration += 1

                print(f"\n--- Итерация {iteration} ---")
                print(f"Режим: {status['mode']}")
                print(f"Позиция: x={status['position'][0]:.2f}, y={status['position'][1]:.2f}")

                if status['obstacle_distance']:
                    print(f"Препятствие: {status['obstacle_distance']:.2f}м")
                else:
                    print("Путь свободен")

                last_status_time = current_time

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Прервано пользователем")

    finally:
        controller.stop()

        # Сохраняем карту
        map_file = f"map_demo_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(map_file)

        print(f"\n[INFO] Карта сохранена: {map_file}")
        print("[INFO] Используйте: python3 visualize_map.py --map {map_file}")


def navigate_to_goal_demo(x, y):
    """
    Демонстрация навигации к цели

    Args:
        x, y: координаты цели
    """
    print("\n" + "="*60)
    print("ДЕМО: НАВИГАЦИЯ К ЦЕЛИ БЕЗ ЛИДАРА")
    print("="*60)
    print(f"Целевая точка: ({x:.2f}, {y:.2f})")
    print("Используется: ультразвук + камера")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    # Инициализация
    controller = NavigationController(
        use_lidar=False,
        use_camera=True,
        use_ultrasonic=True
    )

    # Установка цели
    controller.set_goal(x, y)

    last_status_time = time.time()

    try:
        while running:
            controller.update()

            current_time = time.time()
            if current_time - last_status_time > 3.0:
                status = controller.get_status()

                print(f"\nПозиция: ({status['position'][0]:.2f}, {status['position'][1]:.2f})")
                print(f"Цель: ({x:.2f}, {y:.2f})")
                print(f"Waypoint: {status['waypoint']}")

                if status['mode'] == 'idle':
                    print("\n[SUCCESS] Цель достигнута!")
                    break

                last_status_time = current_time

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Прервано пользователем")

    finally:
        controller.stop()

        # Сохраняем карту
        map_file = f"map_navigation_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(map_file)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Проверка бота
    if ca.bot is None:
        print("[ERROR] Объект bot не найден")
        sys.exit(1)

    # Выбор демо
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        if command == 'explore':
            simple_exploration_demo()

        elif command == 'goto' and len(sys.argv) >= 4:
            try:
                x = float(sys.argv[2])
                y = float(sys.argv[3])
                navigate_to_goal_demo(x, y)
            except ValueError:
                print("[ERROR] Неверные координаты")
                print("Использование: python3 demo_no_lidar.py goto X Y")

        else:
            print("Неизвестная команда")
            print("\nИспользование:")
            print("  python3 demo_no_lidar.py explore")
            print("  python3 demo_no_lidar.py goto X Y")
    else:
        # По умолчанию - исследование
        simple_exploration_demo()


if __name__ == "__main__":
    main()

