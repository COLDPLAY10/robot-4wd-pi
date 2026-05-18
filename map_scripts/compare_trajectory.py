#!/usr/bin/env python3
"""
Сравнение двух траекторий из одной поездки:
  - position_history       — траектория после коррекции scan matcher (SLAM)
  - odom_only_history      — что было бы по чистой колёсной одометрии

Скрипт рисует обе на одном графике поверх занятой карты и считает метрики,
которые имеет смысл показать комиссии:
  - конечное расхождение (drift) одометрии и SLAM
  - длина пути обеих кривых
  - для замкнутого маршрута: невязка "стартовая точка ↔ конечная точка"
    (для SLAM при правильной локализации она должна быть близка к нулю)

Использование:
    python3 compare_trajectory.py --map map_exploration_YYYYMMDD.pkl
    python3 compare_trajectory.py --map map.pkl --output report.png
"""

import argparse
import os
import pickle
import sys

# .pkl содержит ссылки на map.slam_core.Position. На дев-машине без Raspbot_Lib
# обычный `from map import ...` упадёт через цепочку __init__.py → navigation_controller
# → car_adapter → Raspbot_Lib. Поэтому импортируем slam_core напрямую, минуя пакет,
# и регистрируем модуль под именем 'map.slam_core' чтобы pickle его нашёл.
import importlib.util
import types

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO_ROOT)
if 'map' not in sys.modules:
    _pkg = types.ModuleType('map')
    _pkg.__path__ = [os.path.join(_REPO_ROOT, 'map')]
    sys.modules['map'] = _pkg
if 'map.scan_matcher' not in sys.modules:
    _spec = importlib.util.spec_from_file_location(
        'map.scan_matcher', os.path.join(_REPO_ROOT, 'map', 'scan_matcher.py'))
    _m = importlib.util.module_from_spec(_spec); sys.modules['map.scan_matcher'] = _m
    _spec.loader.exec_module(_m)
if 'map.slam_core' not in sys.modules:
    _spec = importlib.util.spec_from_file_location(
        'map.slam_core', os.path.join(_REPO_ROOT, 'map', 'slam_core.py'))
    _m = importlib.util.module_from_spec(_spec); sys.modules['map.slam_core'] = _m
    _spec.loader.exec_module(_m)

import matplotlib
matplotlib.use('Agg')  # без GUI — скрипт работает по SSH на Pi
import matplotlib.pyplot as plt
import numpy as np


def _path_length(history) -> float:
    """Суммарная длина пути по точкам Position."""
    if len(history) < 2:
        return 0.0
    pts = np.array([(p.x, p.y) for p in history])
    return float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))))


def _final_drift(slam_history, odom_history) -> float:
    """Расстояние между конечными точками SLAM и чистой одометрии."""
    if not slam_history or not odom_history:
        return float('nan')
    s = slam_history[-1]
    o = odom_history[-1]
    return float(np.hypot(s.x - o.x, s.y - o.y))


def _loop_closure_error(history) -> float:
    """Расстояние между стартом и финишем — для замкнутого маршрута."""
    if len(history) < 2:
        return float('nan')
    return float(np.hypot(history[-1].x - history[0].x,
                          history[-1].y - history[0].y))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', dest='map_file', required=True,
                        help='путь к .pkl карте, сохранённой demo_with_lidar.py')
    parser.add_argument('--output', dest='output', default=None,
                        help='куда сохранить PNG (по умолчанию <map>_compare.png)')
    parser.add_argument('--no-map', action='store_true',
                        help='не рисовать фон карты, только траектории')
    args = parser.parse_args()

    if not os.path.exists(args.map_file):
        print(f"[ERROR] Файл не найден: {args.map_file}")
        sys.exit(1)

    with open(args.map_file, 'rb') as f:
        data = pickle.load(f)

    slam_hist = data.get('position_history', [])
    odom_hist = data.get('odom_only_history', [])

    if not slam_hist:
        print("[ERROR] В .pkl нет position_history")
        sys.exit(1)
    if not odom_hist:
        print("[WARN] В .pkl нет odom_only_history — это карта старого формата, "
              "сравнение невозможно. Сделайте новую запись.")
        sys.exit(1)

    # === Метрики ===
    len_slam = _path_length(slam_hist)
    len_odom = _path_length(odom_hist)
    drift = _final_drift(slam_hist, odom_hist)
    loop_slam = _loop_closure_error(slam_hist)
    loop_odom = _loop_closure_error(odom_hist)

    sm_total = data.get('scan_match_total', 0)
    sm_ok = data.get('scan_match_accepted', 0)
    sm_rate = (sm_ok / sm_total * 100.0) if sm_total else 0.0

    print("=" * 70)
    print(f"Файл:                          {args.map_file}")
    print(f"Режим:                         {data.get('mapping_mode', '?')}")
    print(f"Длина пути по SLAM:            {len_slam:.2f} м "
          f"({len(slam_hist)} точек)")
    print(f"Длина пути по одометрии:       {len_odom:.2f} м "
          f"({len(odom_hist)} точек)")
    print(f"Расхождение SLAM ↔ одометрия:  {drift*100:.1f} см "
          f"(на финальной точке)")
    print(f"Loop-closure (SLAM):           {loop_slam*100:.1f} см")
    print(f"Loop-closure (одометрия):      {loop_odom*100:.1f} см")
    print(f"Scan match:                    {sm_ok}/{sm_total} "
          f"({sm_rate:.1f}% принято)")
    print("=" * 70)

    # === Рисуем ===
    fig, ax = plt.subplots(figsize=(10, 10))

    if not args.no_map:
        grid = data['grid']
        res = data['resolution']
        ox = data['origin_x']
        oy = data['origin_y']
        h, w = grid.shape
        extent = [(0 - ox) * res, (w - ox) * res,
                  (0 - oy) * res, (h - oy) * res]
        # Только то что точно известно (свободно / занято), остальное прозрачно
        masked = np.ma.masked_where((grid > 30) & (grid < 70), grid)
        ax.imshow(masked, cmap='gray_r', origin='lower', extent=extent,
                  vmin=0, vmax=100, alpha=0.5, interpolation='nearest')

    odom_xy = np.array([(p.x, p.y) for p in odom_hist])
    slam_xy = np.array([(p.x, p.y) for p in slam_hist])

    ax.plot(odom_xy[:, 0], odom_xy[:, 1], '--', color='tab:orange',
            linewidth=2, label=f'Одометрия ({len_odom:.1f}м)')
    ax.plot(slam_xy[:, 0], slam_xy[:, 1], '-', color='tab:blue',
            linewidth=2, label=f'SLAM ({len_slam:.1f}м)')

    ax.plot(*slam_xy[0], 'go', markersize=12, label='старт')
    ax.plot(*slam_xy[-1], 'b^', markersize=12, label='финиш (SLAM)')
    ax.plot(*odom_xy[-1], 'r^', markersize=12, label='финиш (одометрия)')

    ax.set_xlabel("X, м")
    ax.set_ylabel("Y, м")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

    title = (f"Одометрия vs SLAM   |   drift = {drift*100:.1f}см   |   "
             f"scan-match {sm_rate:.0f}%")
    ax.set_title(title, fontsize=12, fontweight='bold')

    metrics = (f"длина SLAM:      {len_slam:.2f} м\n"
               f"длина одометрии: {len_odom:.2f} м\n"
               f"loop SLAM:       {loop_slam*100:.1f} см\n"
               f"loop одометрия:  {loop_odom*100:.1f} см\n"
               f"scan-match:      {sm_ok}/{sm_total} ({sm_rate:.0f}%)")
    ax.text(0.02, 0.02, metrics, transform=ax.transAxes,
            fontsize=10, family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85),
            verticalalignment='bottom')

    out = args.output or (os.path.splitext(args.map_file)[0] + '_compare.png')
    plt.tight_layout()
    plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"График сохранён: {out}")


if __name__ == '__main__':
    main()
