#!/usr/bin/env python3
"""
Скрипт визуализации карты SLAM
Отображает построенную карту, траекторию робота и ориентиры

Использование:
    python3 visualize_map.py [--map FILE] [--save IMAGE]
"""

import argparse
import sys
import os
import pickle
import numpy as np

# Проверка наличия matplotlib
try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Arrow
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("[ERROR] matplotlib не установлен")
    print("Установите: pip install matplotlib")
    sys.exit(1)

# Попытка импорта OpenCV для сохранения
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


def load_map(filename: str) -> dict:
    """Загрузка карты из файла"""
    with open(filename, 'rb') as f:
        data = pickle.load(f)
    return data


def visualize_map(map_data: dict, save_path: str = None):
    """
    Визуализация карты

    Args:
        map_data: данные карты
        save_path: путь для сохранения (опционально)
    """
    grid = map_data['grid']
    resolution = map_data['resolution']
    origin_x = map_data['origin_x']
    origin_y = map_data['origin_y']
    pose_history = map_data.get('position_history', [])
    landmarks = map_data.get('landmarks', [])

    # Создание фигуры
    fig, ax = plt.subplots(figsize=(12, 10))

    # Отображение карты
    # 0 - свободно (белый), 100 - занято (черный), 50 - неизвестно (серый)
    display_grid = np.copy(grid)
    display_grid = 255 - (display_grid * 2.55)  # инвертируем для отображения

    ax.imshow(display_grid, cmap='gray', origin='lower', vmin=0, vmax=255)

    # Отображение начала координат
    ax.plot(origin_x, origin_y, 'go', markersize=10, label='Начало координат')

    # Отображение траектории робота
    if pose_history:
        trajectory_x = []
        trajectory_y = []

        for pose in pose_history:
            gx, gy = world_to_grid(pose.x, pose.y, origin_x, origin_y, resolution)
            trajectory_x.append(gx)
            trajectory_y.append(gy)

        ax.plot(trajectory_x, trajectory_y, 'b-', linewidth=2, alpha=0.7, label='Траектория')

        # Текущая позиция робота
        if trajectory_x and trajectory_y:
            last_pose = pose_history[-1]
            gx, gy = world_to_grid(last_pose.x, last_pose.y, origin_x, origin_y, resolution)

            # Рисуем робота
            robot_circle = Circle((gx, gy), radius=5, color='red', fill=True, alpha=0.7)
            ax.add_patch(robot_circle)

            # Направление робота
            arrow_length = 10
            dx = arrow_length * np.cos(last_pose.theta)
            dy = arrow_length * np.sin(last_pose.theta)
            ax.arrow(gx, gy, dx, dy, head_width=3, head_length=3, fc='red', ec='red')

            ax.plot(gx, gy, 'ro', markersize=8, label='Текущая позиция')

    # Отображение ориентиров
    if landmarks:
        landmark_x = []
        landmark_y = []

        for lm in landmarks:
            gx, gy = world_to_grid(lm.x, lm.y, origin_x, origin_y, resolution)
            landmark_x.append(gx)
            landmark_y.append(gy)

        ax.plot(landmark_x, landmark_y, 'y*', markersize=12, label='Ориентиры')

    # Настройка осей
    ax.set_xlabel('X (пиксели)')
    ax.set_ylabel('Y (пиксели)')
    ax.set_title('Карта SLAM')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Добавление информации
    info_text = f"Разрешение: {resolution}м/ячейка\n"
    info_text += f"Размер карты: {grid.shape[1]}x{grid.shape[0]}\n"
    info_text += f"Точек траектории: {len(pose_history)}\n"
    info_text += f"Ориентиров: {len(landmarks)}"

    ax.text(0.02, 0.98, info_text,
            transform=ax.transAxes,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            fontsize=10)

    plt.tight_layout()

    # Сохранение или отображение
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] Карта сохранена: {save_path}")
    else:
        plt.show()


def world_to_grid(x: float, y: float, origin_x: int, origin_y: int, resolution: float):
    """Преобразование мировых координат в координаты сетки"""
    gx = int(x / resolution + origin_x)
    gy = int(y / resolution + origin_y)
    return gx, gy


def export_map_to_image(map_data: dict, output_path: str):
    """
    Экспорт карты в изображение (PNG/JPG) с использованием OpenCV
    """
    if not CV2_AVAILABLE:
        print("[WARN] OpenCV недоступен, используйте matplotlib для сохранения")
        return

    grid = map_data['grid']

    # Преобразуем в изображение
    img = np.copy(grid)
    img = 255 - (img * 2.55)
    img = img.astype(np.uint8)

    cv2.imwrite(output_path, img)
    print(f"[INFO] Карта экспортирована: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Визуализация карты SLAM"
    )

    parser.add_argument(
        "--map", "-m",
        type=str,
        required=True,
        help="Файл карты (.pkl)"
    )

    parser.add_argument(
        "--save", "-s",
        type=str,
        help="Сохранить визуализацию в файл (PNG)"
    )

    parser.add_argument(
        "--export",
        type=str,
        help="Экспортировать только карту в изображение (PNG/JPG)"
    )

    args = parser.parse_args()

    # Проверка существования файла
    if not os.path.exists(args.map):
        print(f"[ERROR] Файл не найден: {args.map}")
        sys.exit(1)

    # Загрузка карты
    print(f"[INFO] Загрузка карты: {args.map}")
    try:
        map_data = load_map(args.map)
    except Exception as e:
        print(f"[ERROR] Ошибка загрузки карты: {e}")
        sys.exit(1)

    # Экспорт карты в изображение
    if args.export:
        export_map_to_image(map_data, args.export)

    # Визуализация
    visualize_map(map_data, save_path=args.save)


if __name__ == "__main__":
    main()

