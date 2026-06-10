#!/usr/bin/env python3
"""
Калибровка одометрии — интерактивный замер MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED
и WHEEL_BASE для map/navigation_controller.py.

Запуск (на Raspberry Pi с подключённым роботом):
    python3 map_scripts/calibrate_odometry.py            # все три замера
    python3 map_scripts/calibrate_odometry.py forward    # только линейная скорость
    python3 map_scripts/calibrate_odometry.py rotate     # только угловая скорость
    python3 map_scripts/calibrate_odometry.py wheelbase  # только колёсная база

Опционально:
    --pwm 180        PWM, на котором проводить замер (по умолчанию 200)
    --duration 1.0   длительность одного прогона в секундах
    --trials 3       сколько прогонов усреднить

Перед запуском:
    1. Ровный пол, минимум 1.5–2 м свободного пути спереди.
    2. Заряженная батарея (скорость зависит от напряжения).
    3. Рулетка для линейного замера, маркер/лента и транспортир — для углового.
"""

import argparse
import math
import os
import sys
import time

# Делаем car_adapter импортируемым, если запускают из map_scripts/.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import car_adapter as ca

# Должно совпадать с PWM_FULL в map/navigation_controller.py.
PWM_FULL = 255.0

DEFAULT_PWM = 200
DEFAULT_DURATION = 1.0
DEFAULT_TRIALS = 3
WARMUP_DELAY = 2.0  # сек, чтобы пользователь успел отойти от робота


def countdown(seconds: float):
    for i in range(int(seconds), 0, -1):
        print(f"  Старт через {i}...", flush=True)
        time.sleep(1.0)


def measure_linear(pwm: int, duration: float, trials: int) -> float:
    """
    Замер линейной скорости.
    Делает trials прогонов move_forward(pwm) длиной duration секунд,
    после каждого спрашивает пройденное расстояние в см.
    Усредняет, линейно экстраполирует к PWM=255.
    """
    print("\n" + "=" * 60)
    print(f"ЛИНЕЙНАЯ СКОРОСТЬ: PWM={pwm}, длительность={duration}с, попыток={trials}")
    print("=" * 60)
    print("Робот должен ехать прямо. Свободно минимум 1.5–2 м.")
    input("Нажмите Enter для старта замеров...")

    speeds = []  # м/с при заданном pwm
    for trial in range(1, trials + 1):
        print(f"\n--- Попытка {trial}/{trials} ---")
        print("Поставьте робота на стартовую отметку (носом куда поедет).")
        input("Enter — поехали...")
        countdown(WARMUP_DELAY)

        ca.move_forward(pwm)
        time.sleep(duration)
        ca.stop_robot()

        # Доли секунды на остановку — пусть инерция уляжется.
        time.sleep(0.5)

        raw = input("  Сколько см проехал? (Enter — пропустить попытку): ").strip()
        if not raw:
            print("  Пропуск.")
            continue
        try:
            dist_cm = float(raw.replace(",", "."))
        except ValueError:
            print("  Не понял число — пропуск.")
            continue
        v = (dist_cm / 100.0) / duration
        print(f"  → {v:.3f} м/с при PWM={pwm}")
        speeds.append(v)

    if not speeds:
        print("\nНет валидных замеров.")
        return 0.0

    v_avg = sum(speeds) / len(speeds)
    v_full = v_avg * (PWM_FULL / pwm)  # линейная экстраполяция (см. оговорку ниже)
    print(f"\n[Линейная] среднее при PWM={pwm}: {v_avg:.3f} м/с")
    print(f"[Линейная] экстраполяция к PWM={PWM_FULL:.0f}: {v_full:.3f} м/с")
    return v_full


def measure_angular(pwm: int, duration: float, trials: int) -> float:
    """
    Замер угловой скорости поворота на месте.
    Делает trials прогонов rotate_left(pwm) длиной duration секунд,
    после каждого спрашивает угол поворота в градусах (может быть > 360).
    Совет: наклейте кусок ленты по линии носа робота — будет проще оценить угол на полу.
    """
    print("\n" + "=" * 60)
    print(f"УГЛОВАЯ СКОРОСТЬ: PWM={pwm}, длительность={duration}с, попыток={trials}")
    print("=" * 60)
    print("Робот будет поворачиваться на месте. Совет: пометьте носом стартовое направление.")
    input("Нажмите Enter для старта замеров...")

    omegas = []  # рад/с при заданном pwm
    for trial in range(1, trials + 1):
        print(f"\n--- Попытка {trial}/{trials} ---")
        print("Выровняйте робота по стартовой метке.")
        input("Enter — поворачиваем...")
        countdown(WARMUP_DELAY)

        ca.rotate_left(pwm)
        time.sleep(duration)
        ca.stop_robot()

        time.sleep(0.5)

        raw = input("  На сколько градусов повернулся? (можно >360, Enter — пропуск): ").strip()
        if not raw:
            print("  Пропуск.")
            continue
        try:
            deg = float(raw.replace(",", "."))
        except ValueError:
            print("  Не понял число — пропуск.")
            continue
        omega = math.radians(deg) / duration
        print(f"  → {omega:.3f} рад/с ({deg:.0f}°/с) при PWM={pwm}")
        omegas.append(omega)

    if not omegas:
        print("\nНет валидных замеров.")
        return 0.0

    omega_avg = sum(omegas) / len(omegas)
    omega_full = omega_avg * (PWM_FULL / pwm)
    print(f"\n[Угловая] среднее при PWM={pwm}: {omega_avg:.3f} рад/с")
    print(f"[Угловая] экстраполяция к PWM={PWM_FULL:.0f}: {omega_full:.3f} рад/с "
          f"({math.degrees(omega_full):.0f}°/с)")
    return omega_full


def measure_wheel_base() -> float:
    """Эффективная база диф-модели — без автоматики, только запрос замеров."""
    print("\n" + "=" * 60)
    print("ЭФФЕКТИВНАЯ БАЗА (мекалум)")
    print("=" * 60)
    print("Для мекалум-шасси WHEEL_BASE = колея + межосевое расстояние:")
    print("  колея     — между центрами левого и правого колеса (мм)")
    print("  межосевое — между центрами переднего и заднего колеса (мм)")
    print("Введите СУММУ этих двух замеров.")
    raw = input("Колея + межосевое, мм (Enter — пропуск): ").strip()
    if not raw:
        return 0.0
    try:
        mm = float(raw.replace(",", "."))
    except ValueError:
        print("Не понял число — пропуск.")
        return 0.0
    return mm / 1000.0


def print_summary(results: dict):
    print("\n" + "=" * 60)
    print("РЕЗУЛЬТАТЫ — впишите в map/navigation_controller.py")
    print("=" * 60)
    if not results:
        print("Замеров нет.")
        return
    print("Скопируйте в класс NavigationController:\n")
    for k in ('MAX_LINEAR_SPEED', 'MAX_ANGULAR_SPEED', 'WHEEL_BASE'):
        if k in results:
            print(f"    {k} = {results[k]:.3f}")
    print("\nЗамечание: использовалась линейная экстраполяция от рабочего PWM к 255.")
    print("Это разумное приближение в среднем диапазоне PWM, но при PWM<30")
    print("моторы могут не страгиваться (мёртвая зона). Для точности можно повторить")
    print("замер на разных PWM и сравнить v/PWM.")


def main():
    parser = argparse.ArgumentParser(description="Калибровка одометрии робота.")
    parser.add_argument("mode", nargs="?", default="all",
                        choices=["all", "forward", "rotate", "wheelbase"],
                        help="что замерять (по умолчанию: all)")
    parser.add_argument("--pwm", type=int, default=DEFAULT_PWM,
                        help=f"PWM для замера (по умолчанию {DEFAULT_PWM})")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help=f"длительность одного прогона, сек (по умолчанию {DEFAULT_DURATION})")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help=f"сколько прогонов усреднять (по умолчанию {DEFAULT_TRIALS})")
    args = parser.parse_args()

    if ca.bot is None:
        print("[ERROR] Объект bot не найден. Скрипт нужно запускать на Raspberry Pi с подключённым роботом.")
        sys.exit(1)

    if not (10 <= args.pwm <= 255):
        print(f"[ERROR] --pwm должен быть в [10, 255], получено {args.pwm}.")
        sys.exit(1)
    if args.duration <= 0 or args.duration > 5:
        print(f"[ERROR] --duration должен быть в (0, 5] сек, получено {args.duration}.")
        sys.exit(1)
    if args.trials < 1:
        print(f"[ERROR] --trials должен быть >= 1, получено {args.trials}.")
        sys.exit(1)

    results = {}
    try:
        if args.mode in ("forward", "all"):
            v = measure_linear(args.pwm, args.duration, args.trials)
            if v > 0:
                results["MAX_LINEAR_SPEED"] = v
        if args.mode in ("rotate", "all"):
            w = measure_angular(args.pwm, args.duration, args.trials)
            if w > 0:
                results["MAX_ANGULAR_SPEED"] = w
        if args.mode in ("wheelbase", "all"):
            wb = measure_wheel_base()
            if wb > 0:
                results["WHEEL_BASE"] = wb
    except KeyboardInterrupt:
        print("\n[INFO] Прервано пользователем — останавливаю робота.")
    finally:
        try:
            ca.stop_robot()
        except Exception:
            pass

    print_summary(results)


if __name__ == "__main__":
    main()