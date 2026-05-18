#!/usr/bin/env python3
"""
Демонстрационный скрипт автономной навигации с лидаром.

Два режима:
  explore                        — строим карту с нуля (SLAM, scan-to-map ICP)
  explore --map FILE.pkl         — продолжаем картировать поверх существующей карты
  goto X Y --map FILE.pkl        — едем к точке (X, Y) по готовой карте (localization)

При localization робота нужно физически поставить в точку (0, 0, 0)
относительно карты — это начальное условие для scan matcher'а.
"""

import argparse
import os
import signal
import sys
import time

# Импорт из родительской директории
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from map.navigation_controller import NavigationController

running = True


def signal_handler(sig, frame):
    """Обработчик для корректного завершения"""
    global running
    print("\n[INFO] Остановка...")
    running = False


def _run_loop(controller: NavigationController,
              status_interval: float,
              stop_when_idle: bool,
              extra_status_lines=lambda c: ()):
    """Общий главный цикл для explore и goto."""
    last_status_time = time.time()
    try:
        while running:
            controller.update()

            current_time = time.time()
            if current_time - last_status_time >= status_interval:
                status = controller.get_status()
                print("\n" + "-"*60)
                print(f"Режим: {status['mode']}")
                pos = status['position']
                print(f"Позиция: x={pos[0]:.2f}м, y={pos[1]:.2f}м, θ={pos[2]:.2f}рад")
                if status.get('obstacle_distance') is not None:
                    print(f"Препятствие впереди: {status['obstacle_distance']:.2f}м")
                # Метрики scan matcher — то, ради чего весь рефакторинг
                sm = getattr(controller.slam, 'scan_matcher', None)
                if sm is not None:
                    stats = sm.get_stats()
                    print(f"Scan match: {controller.slam.scan_match_accepted}/"
                          f"{controller.slam.scan_match_total} принято "
                          f"(reason последнего: {stats['last_reason']}, "
                          f"iters={stats['last_iters']})")
                for line in extra_status_lines(controller):
                    print(line)
                print("-"*60)
                last_status_time = current_time

            if stop_when_idle and controller.get_status()['mode'] == 'idle':
                print("\n[SUCCESS] Цель достигнута!")
                break

            time.sleep(0.05)  # 20 Hz
    except KeyboardInterrupt:
        print("\n[INFO] Остановка по запросу пользователя")


def exploration_with_lidar_demo(map_file=None):
    """
    Режим картирования: робот исследует, scan-to-map ICP корректирует позу,
    карта накапливается и в конце сохраняется в .pkl.
    """
    print("\n" + "="*60)
    print("РЕЖИМ: КАРТИРОВАНИЕ (mapping + scan-to-map ICP)")
    print("="*60)
    print(f"Карта: {'продолжаем ' + map_file if map_file else 'строим с нуля'}")
    print("Используется: ЛИДАР + scan matcher + ультразвук + камера")
    print("Нажмите Ctrl+C для остановки и сохранения карты")
    print("="*60 + "\n")

    controller = NavigationController(
        use_lidar=True,
        use_camera=True,
        use_ultrasonic=True,
        mapping_mode='mapping',
        map_file=map_file,
    )

    controller.start_exploration()

    try:
        _run_loop(controller, status_interval=5.0, stop_when_idle=False)
    finally:
        controller.stop()
        out_file = f"map_exploration_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(out_file)
        print(f"\n[INFO] Карта сохранена: {out_file}")
        print(f"[INFO] Запуск навигации: "
              f"python3 demo_with_lidar.py goto X Y --map {out_file}")


def goto_with_lidar_demo(target_x: float, target_y: float, map_file: str):
    """
    Режим локализации: загружаем готовую карту, scan matcher держит позу,
    DWA ведёт к цели. Карта не модифицируется.
    """
    if not os.path.exists(map_file):
        print(f"[ERROR] Карта не найдена: {map_file}")
        sys.exit(1)

    print("\n" + "="*60)
    print("РЕЖИМ: НАВИГАЦИЯ ПО ГОТОВОЙ КАРТЕ (localization-only)")
    print("="*60)
    print(f"Карта: {map_file}")
    print(f"Цель: ({target_x:.2f}, {target_y:.2f})")
    print("ВАЖНО: поставьте робота в (0, 0, 0) карты перед запуском")
    print("Используется: ЛИДАР + scan matcher (карта НЕ обновляется)")
    print("Нажмите Ctrl+C для остановки")
    print("="*60 + "\n")

    controller = NavigationController(
        use_lidar=True,
        use_camera=True,
        use_ultrasonic=True,
        mapping_mode='localization',
        map_file=map_file,
    )

    controller.set_goal(target_x, target_y)

    def goal_status(c):
        s = c.get_status()
        return (f"Цель: x={target_x:.2f}м, y={target_y:.2f}м",
                f"Waypoint: {s.get('waypoint')}")

    try:
        _run_loop(controller, status_interval=2.0,
                  stop_when_idle=True,
                  extra_status_lines=goal_status)
    finally:
        controller.stop()
        # В localization режиме мы не модифицируем карту, но сохраняем pkl
        # с обновлённой траекторией — это полезно для отчёта и отладки.
        out_file = f"trajectory_{time.strftime('%Y%m%d_%H%M%S')}.pkl"
        controller.save_map(out_file)
        print(f"\n[INFO] Траектория сохранена: {out_file}")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(
        description="SLAM-навигация с лидаром: картирование или езда по готовой карте",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_explore = sub.add_parser('explore', help='построить карту окружения')
    p_explore.add_argument('--map', dest='map_file', default=None,
                           help='опционально: дополнить существующую карту .pkl')

    p_goto = sub.add_parser('goto', help='ехать к точке по готовой карте')
    p_goto.add_argument('x', type=float, help='X цели в метрах')
    p_goto.add_argument('y', type=float, help='Y цели в метрах')
    p_goto.add_argument('--map', dest='map_file', required=True,
                        help='путь к .pkl карте (обязателен для localization)')

    args = parser.parse_args()

    if args.cmd == 'explore':
        exploration_with_lidar_demo(map_file=args.map_file)
    elif args.cmd == 'goto':
        goto_with_lidar_demo(args.x, args.y, args.map_file)


if __name__ == '__main__':
    main()