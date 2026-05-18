#!/usr/bin/env python3
"""
Визуализация occupancy grid в PNG.

Раньше matplotlib-код жил прямо в SLAM.save_map и занимал ~140 строк.
Вынесен сюда, чтобы slam_core.py отвечал только за алгоритм SLAM, а
визуализация была опциональной (matplotlib импортируется лениво).
"""

from typing import Iterable, Optional, Tuple

import numpy as np


def _compute_inflation_overlay(grid: np.ndarray,
                               inflation_radius_m: float,
                               resolution: float,
                               occupied_threshold: float = 65.0) -> Optional[np.ndarray]:
    """
    Бинарная маска "клетки, попадающие в inflation-зону, но НЕ занятые сами".
    Это именно тот пояс вокруг препятствий, который A* запрещает.

    Возвращает None если cv2/scipy нет — оверлей просто не рисуем.
    """
    if inflation_radius_m <= 0:
        return None
    radius_px = max(1, int(np.ceil(inflation_radius_m / resolution)))
    occupied = (grid >= occupied_threshold).astype(np.uint8)
    try:
        import cv2  # type: ignore
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1))
        inflated = cv2.dilate(occupied, kernel)
    except ImportError:
        return None
    # Только пояс — без самих стен (стены и так чёрные на карте)
    return (inflated.astype(bool) & ~occupied.astype(bool))


def render_map_png(grid: np.ndarray,
                   resolution: float,
                   origin_x: int,
                   origin_y: int,
                   current_position: Tuple[float, float, float],
                   position_history: Optional[Iterable] = None,
                   png_filename: str = 'map.png',
                   title_suffix: str = '',
                   inflation_radius_m: float = 0.0) -> bool:
    """
    Нарисовать карту + траекторию в PNG.

    Args:
        grid: H×W float, значения 0..100 (occupancy probability)
        resolution: размер ячейки в метрах
        origin_x, origin_y: индексы мирового нуля в сетке
        current_position: (x, y, theta) текущая поза робота в метрах/радианах
        position_history: iterable объектов с атрибутами .x .y (траектория)
        png_filename: куда сохранить PNG
        title_suffix: что добавить к заголовку (например, "режим: localization")
        inflation_radius_m: если > 0 — нарисовать полупрозрачный пояс
                            вокруг препятствий шириной radius, чтобы видеть,
                            какую зону A* считает непроходимой по геометрии робота.

    Returns:
        True если PNG сохранён, False если matplotlib недоступен.
    """
    try:
        import matplotlib
        # На Pi обычно нет GUI — заставляем неинтерактивный backend.
        matplotlib.use('Agg', force=True)
        import matplotlib.colors as mcolors
        import matplotlib.pyplot as plt
    except ImportError:
        print("[map_visualizer] matplotlib не установлен — PNG не создан")
        return False

    display_map = grid
    height, width = display_map.shape

    colors = ['darkgreen', 'green', 'lightgreen', 'gray', 'orange', 'red', 'darkred']
    cmap = mcolors.LinearSegmentedColormap.from_list('occupancy', colors, N=100)

    # Авто-кроп: показываем только исследованную область + траекторию,
    # иначе при сетке 400×400 (20×20 м) полезная зона 1–3 м выглядит крошечной.
    margin = 20
    explored_mask = np.abs(display_map - 50.0) > 1.0
    ys_e, xs_e = np.where(explored_mask)

    path_gx, path_gy = [], []
    if position_history:
        for pos in position_history:
            gx = int(pos.x / resolution + origin_x)
            gy = int(pos.y / resolution + origin_y)
            if 0 <= gx < width and 0 <= gy < height:
                path_gx.append(gx)
                path_gy.append(gy)

    if len(xs_e) > 0 or path_gx:
        x_min = min((xs_e.min() if len(xs_e) else width), min(path_gx) if path_gx else width)
        x_max = max((xs_e.max() if len(xs_e) else 0), max(path_gx) if path_gx else 0)
        y_min = min((ys_e.min() if len(ys_e) else height), min(path_gy) if path_gy else height)
        y_max = max((ys_e.max() if len(ys_e) else 0), max(path_gy) if path_gy else 0)
        x_min = max(0, x_min - margin)
        x_max = min(width, x_max + margin)
        y_min = max(0, y_min - margin)
        y_max = min(height, y_max + margin)
        side = max(x_max - x_min, y_max - y_min)
        cx = (x_min + x_max) // 2
        cy = (y_min + y_max) // 2
        x_min = max(0, cx - side // 2)
        x_max = min(width, x_min + side)
        y_min = max(0, cy - side // 2)
        y_max = min(height, y_min + side)
    else:
        x_min, x_max, y_min, y_max = 0, width, 0, height

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
    extent = [0, width, 0, height]

    # Inflation-оверлей — общий для обеих панелей.
    # Маскируем нули, чтобы оставалась только сама "толстая" зона, без фона.
    inflation_mask = _compute_inflation_overlay(display_map, inflation_radius_m, resolution)
    if inflation_mask is not None:
        infl_layer = np.ma.masked_where(~inflation_mask, inflation_mask.astype(float))
        inflation_cmap = mcolors.ListedColormap(['red'])
        inflation_label = f'Inflation ({inflation_radius_m*100:.0f}см)'
    else:
        infl_layer = None
        inflation_cmap = None
        inflation_label = None

    # === ЛЕВАЯ ПАНЕЛЬ: основная карта ===
    im1 = ax1.imshow(display_map, cmap=cmap, origin='lower',
                     vmin=0, vmax=100, extent=extent, interpolation='nearest')

    if infl_layer is not None:
        ax1.imshow(infl_layer, cmap=inflation_cmap, origin='lower',
                   extent=extent, alpha=0.35, interpolation='nearest', zorder=5)

    cx_m, cy_m, ctheta_m = current_position
    px = int(cx_m / resolution + origin_x)
    py = int(cy_m / resolution + origin_y)
    ax1.plot(px, py, 'o', markersize=20, label='Робот',
             markerfacecolor='blue', markeredgecolor='white',
             markeredgewidth=3, zorder=10)

    arrow_len = 15
    dx = arrow_len * np.cos(ctheta_m)
    dy = arrow_len * np.sin(ctheta_m)
    ax1.arrow(px, py, dx, dy, head_width=8, head_length=5,
              fc='blue', ec='white', linewidth=2, zorder=11)

    if len(path_gx) > 1:
        ax1.plot(path_gx, path_gy, 'cyan', alpha=0.8, linewidth=3,
                 label=f'Путь ({len(path_gx)} точек)', zorder=9)

    title = f"SLAM карта\nРазрешение: {resolution}м/ячейка"
    if title_suffix:
        title += f"  |  {title_suffix}"
    ax1.set_title(title, fontsize=14, fontweight='bold')
    ax1.set_xlabel("Ячейки сетки (X)", fontsize=12)
    ax1.set_ylabel("Ячейки сетки (Y)", fontsize=12)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_aspect('equal')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.2, linestyle='--', color='white')

    cbar1 = plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
    cbar1.set_label('Вероятность занятости\n(0=свободно, 100=занято)', fontsize=10)

    # === ПРАВАЯ ПАНЕЛЬ: бинарная карта ===
    binary_map = np.zeros_like(display_map)
    binary_map[display_map < 30] = 0
    binary_map[display_map >= 70] = 2
    binary_map[(display_map >= 30) & (display_map < 70)] = 1

    cmap_binary = mcolors.ListedColormap(['white', 'gray', 'black'])
    bounds = [0, 0.5, 1.5, 2.5]
    norm = mcolors.BoundaryNorm(bounds, cmap_binary.N)

    im2 = ax2.imshow(binary_map, cmap=cmap_binary, norm=norm,
                     origin='lower', extent=extent, interpolation='nearest')

    if infl_layer is not None:
        ax2.imshow(infl_layer, cmap=inflation_cmap, origin='lower',
                   extent=extent, alpha=0.35, interpolation='nearest', zorder=5)
        # Маркер для легенды на бинарной карте — там цветов мало, оверлей виден
        ax2.plot([], [], 's', color='red', alpha=0.5, markersize=12,
                 label=inflation_label)
        ax2.legend(loc='upper right', fontsize=10)

    ax2.plot(px, py, 'o', markersize=20,
             markerfacecolor='red', markeredgecolor='white',
             markeredgewidth=3, zorder=10)
    ax2.arrow(px, py, dx, dy, head_width=8, head_length=5,
              fc='red', ec='white', linewidth=2, zorder=11)

    if len(path_gx) > 1:
        ax2.plot(path_gx, path_gy, 'b-', alpha=0.7, linewidth=2, zorder=9)

    ax2.set_title("Бинарная карта (упрощённая)", fontsize=14, fontweight='bold')
    ax2.set_xlabel("Ячейки сетки (X)", fontsize=12)
    ax2.set_ylabel("Ячейки сетки (Y)", fontsize=12)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3, linestyle='--')

    cbar2 = plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04,
                         ticks=[0.25, 1, 1.75])
    cbar2.ax.set_yticklabels(['Свободно', 'Неизвестно', 'Занято'])

    # Сводка
    total_cells = width * height
    free_cells = int(np.sum(display_map < 30))
    occupied_cells = int(np.sum(display_map >= 70))
    unknown_cells = total_cells - free_cells - occupied_cells
    explored_pct = ((free_cells + occupied_cells) / total_cells) * 100

    stats_text = (
        f"Статистика карты:\n"
        f"Размер: {width}×{height} ({total_cells} ячеек)\n"
        f"Исследовано: {explored_pct:.1f}%\n"
        f"Свободно: {free_cells} ({free_cells/total_cells*100:.1f}%)\n"
        f"Занято: {occupied_cells} ({occupied_cells/total_cells*100:.1f}%)\n"
        f"Неизвестно: {unknown_cells} ({unknown_cells/total_cells*100:.1f}%)\n"
        f"Пройдено: {len(path_gx)} позиций\n"
        f"Робот: ({cx_m:.2f}, {cy_m:.2f}, {np.rad2deg(ctheta_m):.0f}°)"
    )
    fig.text(0.5, 0.02, stats_text, ha='center', fontsize=10,
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
             family='monospace')

    plt.tight_layout(rect=[0, 0.12, 1, 1])
    plt.savefig(png_filename, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return True
