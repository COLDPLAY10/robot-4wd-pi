#!/usr/bin/env python3
"""
Главный скрипт автономной навигации робота
Реализует SLAM и автономное движение с объездом препятствий

Использование:
    python3 autonomous_navigation.py [--mode MODE] [--goal X Y] [--no-lidar] [--no-camera]

Режимы:
    exploration - исследование окружения
    navigation  - движение к цели

Примеры:
    # Режим исследования без лидара
    python3 autonomous_navigation.py --mode exploration --no-lidar

    # Движение к точке (2, 3)
    python3 autonomous_navigation.py --mode navigation --goal 2.0 3.0
"""

import argparse
import time
import signal
import sys
import os

# Добавляем текущую директорию в путь
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from map.navigation_controller import NavigationController, NavigationMode
import car_adapter as ca

# Глобальные переменные
controller = None
running = True


def signal_handler(sig, frame):
    """Обработчик сигналов для корректного завершения"""
    global running
    print("\n[INFO] Получен сигнал остановки...")
    running = False


def print_status(navigation_controller: NavigationController):
    """Вывод статуса системы"""
    status = navigation_controller.get_status()

    print("\n" + "="*60)
    print("СТАТУС СИСТЕМЫ")
    print("="*60)
    print(f"Режим: {status['mode']}")
    print(f"Позиция: x={status['position'][0]:.2f}м, y={status['position'][1]:.2f}м, θ={status['position'][2]:.2f}рад")

    if status['goal']:
        print(f"Цель: x={status['goal'][0]:.2f}м, y={status['goal'][1]:.2f}м")
        print(f"Waypoint: {status['waypoint']}")

    if status['obstacle_distance'] is not None:
        print(f"Препятствие впереди: {status['obstacle_distance']:.2f}м")
    else:
        print("Препятствие впереди: не обнаружено")

    print("="*60)


def exploration_mode(navigation_controller: NavigationController, duration: float = None):
    """
    Режим исследования окружения

    Args:
        navigation_controller: контроллер навигации
        duration: длительность в секундах (None = бесконечно)
    """
    print("\n" + "="*60)
    print("РЕЖИМ АНАЛИЗА СРЕДЫ")
    print("="*60)
    print("Робот будет медленно двигаться и строить карту")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    navigation_controller.start_exploration()

    start_time = time.time()
    last_status_time = start_time

    try:
        while running:
            # Обновление контроллера
            navigation_controller.update()

            # Вывод статуса каждые 5 секунд
            if time.time() - last_status_time > 5.0:
                print_status(navigation_controller)
                last_status_time = time.time()

            # Проверка времени выполнения
            if duration and (time.time() - start_time) > duration:
                print("\n[INFO] Время исследования истекло")
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Прервано пользователем")

    finally:
        navigation_controller.stop()


def navigation_mode(navigation_controller: NavigationController, goal_x: float, goal_y: float):
    """
    Режим навигации к цели

    Args:
        :param navigation_controller: контроллер навигации
        :param goal_y:  координаты цели
        :param goal_x:  координаты цели
    """
    print("\n" + "="*60)
    print("РЕЖИМ НАВИГАЦИИ К ЦЕЛИ")
    print("="*60)
    print(f"Целевая точка: ({goal_x:.2f}, {goal_y:.2f})")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    navigation_controller.set_goal(goal_x, goal_y)

    last_status_time = time.time()

    try:
        while running:
            # Обновление контроллера
            navigation_controller.update()

            # Вывод статуса каждые 3 секунды
            if time.time() - last_status_time > 3.0:
                print_status(navigation_controller)
                last_status_time = time.time()

            # Проверка достижения цели
            status = navigation_controller.get_status()
            if status['mode'] == 'idle':
                print("\n[SUCCESS] Цель достигнута!")
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Прервано пользователем")

    finally:
        navigation_controller.stop()


def interactive_mode(navigation_controller: NavigationController):
    """
    Интерактивный режим с командами
    """
    print("\n" + "="*60)
    print("ИНТЕРАКТИВНЫЙ РЕЖИМ")
    print("="*60)
    print("Доступные команды:")
    print("  explore [время]    - начать исследование (время в секундах, опционально)")
    print("  goto X Y           - двигаться к точке (X, Y)")
    print("  stop               - остановить робота")
    print("  status             - показать статус")
    print("  save [файл]        - сохранить карту")
    print("  load [файл]        - загрузить карту")
    print("  quit               - выход")
    print("="*60 + "\n")

    while running:
        try:
            cmd = input(">> ").strip().split()

            if not cmd:
                continue

            command = cmd[0].lower()

            if command == 'quit' or command == 'exit':
                break

            elif command == 'explore':
                duration = float(cmd[1]) if len(cmd) > 1 else None
                exploration_mode(navigation_controller, duration)

            elif command == 'goto':
                if len(cmd) < 3:
                    print("[ERROR] Использование: goto X Y")
                    continue
                try:
                    x = float(cmd[1])
                    y = float(cmd[2])
                    navigation_mode(navigation_controller, x, y)
                except ValueError:
                    print("[ERROR] Неверные координаты")

            elif command == 'stop':
                navigation_controller.stop()
                print("[INFO] Робот остановлен")

            elif command == 'status':
                print_status(navigation_controller)

            elif command == 'save':
                filename = cmd[1] if len(cmd) > 1 else "map_save.pkl"
                navigation_controller.save_map(filename)

            elif command == 'load':
                filename = cmd[1] if len(cmd) > 1 else "map_save.pkl"
                try:
                    navigation_controller.load_map(filename)
                    print(f"[INFO] Карта загружена: {filename}")
                except Exception as e:
                    print(f"[ERROR] Ошибка загрузки: {e}")

            else:
                print(f"[ERROR] Неизвестная команда: {command}")

        except KeyboardInterrupt:
            print("\n[INFO] Выход...")
            break
        except Exception as e:
            print(f"[ERROR] {e}")

    navigation_controller.stop()


def main():
    global controller, running

    parser = argparse.ArgumentParser(
        description="Система автономной навигации робота с SLAM"
    )

    parser.add_argument(
        "--mode", "-m",
        type=str,
        choices=['exploration', 'navigation', 'interactive'],
        default='interactive',
        help="Режим работы (по умолчанию: interactive)"
    )

    parser.add_argument(
        "--goal", "-g",
        nargs=2,
        type=float,
        metavar=('X', 'Y'),
        help="Координаты цели для режима navigation"
    )

    parser.add_argument(
        "--duration", "-d",
        type=float,
        help="Длительность исследования в секундах"
    )

    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="Отключить лидар"
    )

    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Отключить камеру"
    )

    parser.add_argument(
        "--no-ultrasonic",
        action="store_true",
        help="Отключить ультразвуковой датчик"
    )

    parser.add_argument(
        "--load-map",
        type=str,
        metavar="FILE",
        help="Загрузить карту из файла"
    )

    args = parser.parse_args()

    # Установка обработчика сигналов
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if ca.bot is None:
        print("[ERROR] Объект bot не найден. Проверьте подключение к роботу.")
        sys.exit(1)

    # Создание контроллера
    print("\n[INFO] Инициализация системы навигации...")
    controller = NavigationController(
        use_lidar=not args.no_lidar,
        use_camera=not args.no_camera,
        use_ultrasonic=not args.no_ultrasonic
    )

    # Загрузка карты если указано
    if args.load_map:
        try:
            controller.load_map(args.load_map)
        except Exception as e:
            print(f"[WARN] Не удалось загрузить карту: {e}")

    try:
        # Выполнение в зависимости от режима
        if args.mode == 'exploration':
            exploration_mode(controller, args.duration)

        elif args.mode == 'navigation':
            if not args.goal:
                print("[ERROR] Для режима navigation требуется указать --goal X Y")
                sys.exit(1)
            navigation_mode(controller, args.goal[0], args.goal[1])

        elif args.mode == 'interactive':
            interactive_mode(controller)

    finally:
        # Сохранение карты при выходе
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            map_file = f"map_autosave_{timestamp}.pkl"
            controller.save_map(map_file)
        except Exception as e:
            print(f"[WARN] Не удалось сохранить карту: {e}")

        controller.stop()
        ca.stop()

        print("\n[INFO] Завершение работы")


if __name__ == "__main__":
    main()

