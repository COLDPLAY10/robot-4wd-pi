#!/usr/bin/env python3
"""
Ядро системы SLAM (Simultaneous Localization and Mapping)
Поддерживает работу с лидаром и без него (используя камеру и ультразвук)
"""

import numpy as np
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple
import pickle
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import os


@dataclass
class Position:
    """Позиция и ориентация робота"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # угол в радианах
    timestamp: float = 0.0


@dataclass
class LandMark:
    """Ориентир на карте"""
    x: float
    y: float
    feature_type: str  # 'wall', 'obstacle', 'corner'
    confidence: float = 1.0


class OccupancyGrid:
    """Сеточная карта окружения"""

    def __init__(self, width=400, height=400, resolution=0.05):
        """
        Args:
            width: ширина карты в ячейках
            height: высота карты в ячейках
            resolution: размер ячейки в метрах
        """
        self.width = width
        self.height = height
        self.resolution = resolution

        # 0 - неизвестно, 0-50 - свободно, 51-100 - занято
        self.grid = np.ones((height, width), dtype=np.float32) * 50
        self.origin_x = width // 2
        self.origin_y = height // 2

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Преобразование мировых координат в координаты сетки"""
        gx = int(x / self.resolution + self.origin_x)
        gy = int(y / self.resolution + self.origin_y)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Преобразование координат сетки в мировые"""
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def update_cell(self, x: float, y: float, occupied: bool, confidence: float = 0.7):
        """Обновление вероятности занятости ячейки (логарифмический метод)"""
        gx, gy = self.world_to_grid(x, y)

        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return

        current_value = self.grid[gy, gx]

        if occupied:

            delta = confidence * 15
            self.grid[gy, gx] = min(95, current_value + delta)
        else:
            delta = confidence * 8
            self.grid[gy, gx] = max(5, current_value - delta)

    def is_occupied(self, x: float, y: float, threshold: float = 60) -> bool:
        """Проверка занятости точки"""
        gx, gy = self.world_to_grid(x, y)

        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return True  # За пределами карты считаем занятым

        return self.grid[gy, gx] > threshold

    def get_map_array(self) -> np.ndarray:
        """Получение массива карты для визуализации"""
        return self.grid.copy()


class SLAM:
    """
    Система SLAM для робота
    """

    def __init__(self, map_width=400, map_height=400, resolution=0.05, use_lidar=True):
        """
        Args:
            map_width: ширина карты в ячейках
            map_height: высота карты в ячейках
            resolution: размер ячейки в метрах
            use_lidar: использовать ли лидар
        """
        self.use_lidar = use_lidar
        self.map = OccupancyGrid(map_width, map_height, resolution)

        # Текущая позиция робота
        self.current_position = Position(x=0.0, y=0.0, theta=0.0, timestamp=time.time())

        # История позиций
        self.position_history = deque(maxlen=1000)
        self.position_history.append(self.current_position)

        # Ориентиры
        self.landmarks = []

        # Параметры одометрии
        self.last_update_time = time.time()
        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.process_noise = np.diag([0.1, 0.1, 0.05])  # x, y, theta
        self.measurement_noise = np.diag([0.5, 0.5])    # измерения расстояний

        print(f"[SLAM] Инициализирована карта {map_width}x{map_height}, разрешение: {resolution}м")
        print(f"[SLAM] Режим лидара: {'включен' if use_lidar else 'выключен'}")

    def update_odometry(self, left_speed: float, right_speed: float, dt: float):
        """
        Обновление одометрии на основе скоростей колес

        Args:
            left_speed: скорость левых колес (м/с)
            right_speed: скорость правых колес (м/с)
            dt: временной интервал (сек)
        """
        wheel_base = 0.15  # TODO расстояние между колесами в метрах

        # Линейная и угловая скорость
        v = (left_speed + right_speed) / 2.0
        omega = (right_speed - left_speed) / wheel_base

        # Обновление позиции
        self.current_position.x += v * np.cos(self.current_position.theta) * dt
        self.current_position.y += v * np.sin(self.current_position.theta) * dt
        self.current_position.theta += omega * dt

        # Нормализация угла
        self.current_position.theta = self._normalize_angle(self.current_position.theta)
        self.current_position.timestamp = time.time()

        self.velocity = v
        self.angular_velocity = omega

        # Сохранение в историю
        self.position_history.append(Position(
            x=self.current_position.x,
            y=self.current_position.y,
            theta=self.current_position.theta,
            timestamp=self.current_position.timestamp
        ))

    def update_with_lidar(self, scan_data: List[Tuple[float, float]]):
        """
        Обновление карты данными лидара

        Args:
            scan_data: список точек в полярных координатах [(angle, distance), ...]
                      angle в радианах, distance в метрах
        """
        if not self.use_lidar:
            return

        robot_x = self.current_position.x
        robot_y = self.current_position.y
        robot_theta = self.current_position.theta

        for angle, distance in scan_data:
            if distance <= 0 or distance > 10.0:  # игнорируем невалидные измерения
                continue

            # Глобальный угол точки
            global_angle = robot_theta + angle

            # Координаты препятствия
            obs_x = robot_x + distance * np.cos(global_angle)
            obs_y = robot_y + distance * np.sin(global_angle)

            # Отмечаем препятствие
            self.map.update_cell(obs_x, obs_y, occupied=True, confidence=0.9)

            # Отмечаем свободное пространство по лучу
            num_steps = int(distance / self.map.resolution)
            for i in range(1, num_steps):
                t = i / num_steps
                free_x = robot_x + t * distance * np.cos(global_angle)
                free_y = robot_y + t * distance * np.sin(global_angle)
                self.map.update_cell(free_x, free_y, occupied=False, confidence=0.5)

    def update_with_ultrasonic(self, distance: float, sensor_angle: float = 0.0):
        """
        Обновление карты данными ультразвукового датчика

        Args:
            distance: расстояние в метрах
            sensor_angle: угол датчика относительно робота (радианы)
        """
        if distance <= 0 or distance > 4.0:  # макс дальность ультразвука ~4м
            return

        robot_x = self.current_position.x
        robot_y = self.current_position.y
        robot_theta = self.current_position.theta

        # Глобальный угол
        global_angle = robot_theta + sensor_angle

        # Координаты препятствия
        obs_x = robot_x + distance * np.cos(global_angle)
        obs_y = robot_y + distance * np.sin(global_angle)

        # Ультразвук менее точен, поэтому меньшая уверенность
        self.map.update_cell(obs_x, obs_y, occupied=True, confidence=0.6)

        # Свободное пространство
        num_steps = max(2, int(distance / (self.map.resolution * 2)))
        for i in range(1, num_steps):
            t = i / num_steps
            free_x = robot_x + t * distance * np.cos(global_angle)
            free_y = robot_y + t * distance * np.sin(global_angle)
            self.map.update_cell(free_x, free_y, occupied=False, confidence=0.4)

    def update_with_camera_segmentation(self, segmentation_mask: np.ndarray,
                                       camera_fov: float = 60.0,
                                       max_distance: float = 3.0):
        """
        Обновление карты на основе семантической сегментации

        Args:
            segmentation_mask: маска сегментации (H x W), где значения:
                              0 - пол (свободно), 1 - препятствие, 2 - стена, 3 - сетка
            camera_fov: угол обзора камеры в градусах
            max_distance: максимальная дальность проекции в метрах
        """
        h, w = segmentation_mask.shape
        fov_rad = np.deg2rad(camera_fov)

        robot_x = self.current_position.x
        robot_y = self.current_position.y
        robot_theta = self.current_position.theta

        # Проходим по колонкам изображения
        for col in range(0, w, 5):  # шаг 5 для ускорения
            # Угол луча относительно робота
            angle_offset = (col / w - 0.5) * fov_rad
            global_angle = robot_theta + angle_offset

            # Ищем препятствие в колонке (снизу вверх)
            obstacle_found = False
            distance_estimate = max_distance

            for row in range(h - 1, h // 2, -1):  # нижняя половина изображения
                val = segmentation_mask[row, col]

                if val > 0:  # препятствие, стена или сетка
                    # Оценка расстояния по вертикальной позиции в кадре
                    # Чем ниже в кадре, тем ближе
                    distance_estimate = max_distance * (1.0 - (h - row) / (h / 2))
                    distance_estimate = max(0.3, min(max_distance, distance_estimate))
                    obstacle_found = True
                    break

            if obstacle_found:
                # Координаты препятствия
                obs_x = robot_x + distance_estimate * np.cos(global_angle)
                obs_y = robot_y + distance_estimate * np.sin(global_angle)

                self.map.update_cell(obs_x, obs_y, occupied=True, confidence=0.7)
            else:
                # Свободное пространство
                for d in np.linspace(0.2, max_distance * 0.8, 5):
                    free_x = robot_x + d * np.cos(global_angle)
                    free_y = robot_y + d * np.sin(global_angle)
                    self.map.update_cell(free_x, free_y, occupied=False, confidence=0.3)

    def add_landmark(self, x: float, y: float, feature_type: str, confidence: float = 1.0):
        """Добавление ориентира на карту"""
        landmark = LandMark(x=x, y=y, feature_type=feature_type, confidence=confidence)
        self.landmarks.append(landmark)

    def get_pose(self) -> Position:
        """Получение текущей позиции робота"""
        return Position(
            x=self.current_position.x,
            y=self.current_position.y,
            theta=self.current_position.theta,
            timestamp=self.current_position.timestamp
        )

    def get_map(self) -> OccupancyGrid:
        """Получение карты"""
        return self.map

    def is_path_clear(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Проверка свободности пути между двумя точками"""
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        num_checks = max(10, int(dist / self.map.resolution))

        for i in range(num_checks + 1):
            t = i / num_checks
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if self.map.is_occupied(x, y):
                return False

        return True

    def save_map(self, filename: str):
        """Сохранение карты в файл (PKL + PNG визуализация)"""

        base_filename = os.path.splitext(filename)[0]
        pkl_filename = base_filename + '.pkl'
        png_filename = 'slam_map.png'

        data = {
            'grid': self.map.grid,
            'resolution': self.map.resolution,
            'origin_x': self.map.origin_x,
            'origin_y': self.map.origin_y,
            'position_history': list(self.position_history),
            'landmarks': self.landmarks,
            'use_lidar': self.use_lidar
        }

        with open(pkl_filename, 'wb') as f:
            pickle.dump(data, f)
        print(f"[SLAM] Данные карты сохранены: {pkl_filename}")

        # 2. Создаем PNG визуализацию
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

        # === ЛЕВАЯ ПАНЕЛЬ: Основная карта ===
        display_map = self.map.grid.copy()
        height, width = display_map.shape

        colors = ['darkgreen', 'green', 'lightgreen', 'gray', 'orange', 'red', 'darkred']
        n_bins = 100
        cmap = mcolors.LinearSegmentedColormap.from_list('occupancy', colors, N=n_bins)

        extent = [0, width, 0, height]
        im1 = ax1.imshow(display_map, cmap=cmap, origin='lower',
                        vmin=0, vmax=100, extent=extent, interpolation='nearest')

        # Добавляем позицию робота
        px, py = self.map.world_to_grid(self.current_position.x, self.current_position.y)
        ax1.plot(px, py, 'o', markersize=20, label='Робот',
                markerfacecolor='blue', markeredgecolor='white', markeredgewidth=3, zorder=10)

        # Стрелка направления робота
        arrow_len = 15
        dx = arrow_len * np.cos(self.current_position.theta)
        dy = arrow_len * np.sin(self.current_position.theta)
        ax1.arrow(px, py, dx, dy, head_width=8, head_length=5,
                 fc='blue', ec='white', linewidth=2, zorder=11)

        # Добавляем историю пути
        path_x = []
        path_y = []
        if len(self.position_history) > 1:
            for pos in self.position_history:
                gx, gy = self.map.world_to_grid(pos.x, pos.y)
                path_x.append(gx)
                path_y.append(gy)
            ax1.plot(path_x, path_y, 'cyan', alpha=0.8, linewidth=3,
                    label=f'Путь ({len(path_x)} точек)', zorder=9)

        ax1.set_title(f"SLAM карта\nРазрешение: {self.map.resolution}м/ячейка", fontsize=14, fontweight='bold')
        ax1.set_xlabel("Ячейки сетки (X)", fontsize=12)
        ax1.set_ylabel("Ячейки сетки (Y)", fontsize=12)
        ax1.legend(loc='upper right', fontsize=10)
        ax1.grid(True, alpha=0.2, linestyle='--', color='white')

        cbar1 = plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
        cbar1.set_label('Вероятность занятости\n(0=свободно, 100=занято)', fontsize=10)

        # === ПРАВАЯ ПАНЕЛЬ: Бинарная картат ===
        binary_map = np.zeros_like(display_map)
        binary_map[display_map < 30] = 0    # Свободно (белое)
        binary_map[display_map >= 70] = 2   # Занято (черное)
        binary_map[(display_map >= 30) & (display_map < 70)] = 1  # Неизвестно (серое)

        cmap_binary = mcolors.ListedColormap(['white', 'gray', 'black'])
        bounds = [0, 0.5, 1.5, 2.5]
        norm = mcolors.BoundaryNorm(bounds, cmap_binary.N)

        im2 = ax2.imshow(binary_map, cmap=cmap_binary, norm=norm,
                        origin='lower', extent=extent, interpolation='nearest')

        # Робот на бинарной карте
        ax2.plot(px, py, 'o', markersize=20,
                markerfacecolor='red', markeredgecolor='white', markeredgewidth=3, zorder=10)
        ax2.arrow(px, py, dx, dy, head_width=8, head_length=5,
                 fc='red', ec='white', linewidth=2, zorder=11)

        # Путь на бинарной карте
        if len(self.position_history) > 1:
            ax2.plot(path_x, path_y, 'b-', alpha=0.7, linewidth=2, zorder=9)

        ax2.set_title("Бинарная карта (упрощенная)", fontsize=14, fontweight='bold')
        ax2.set_xlabel("Ячейки сетки (X)", fontsize=12)
        ax2.set_ylabel("Ячейки сетки (Y)", fontsize=12)
        ax2.grid(True, alpha=0.3, linestyle='--')

        # Colorbar для бинарной карты
        cbar2 = plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04,
                            ticks=[0.25, 1, 1.75])
        cbar2.ax.set_yticklabels(['Свободно', 'Неизвестно', 'Занято'])

        total_cells = width * height
        free_cells = np.sum(display_map < 30)
        occupied_cells = np.sum(display_map >= 70)
        unknown_cells = total_cells - free_cells - occupied_cells
        explored_pct = ((free_cells + occupied_cells) / total_cells) * 100

        stats_text = (
            f"Статистика карты:\n"
            f"Размер: {width}×{height} ({total_cells} ячеек)\n"
            f"Исследовано: {explored_pct:.1f}%\n"
            f"Свободно: {free_cells} ({free_cells/total_cells*100:.1f}%)\n"
            f"Занято: {occupied_cells} ({occupied_cells/total_cells*100:.1f}%)\n"
            f"Неизвестно: {unknown_cells} ({unknown_cells/total_cells*100:.1f}%)\n"
            f"Пройдено: {len(self.position_history)} позиций\n"
            f"Робот: ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {np.rad2deg(self.current_position.theta):.0f}°)"
        )

        fig.text(0.5, 0.02, stats_text, ha='center', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                family='monospace')

        plt.tight_layout(rect=[0, 0.12, 1, 1])
        plt.savefig(png_filename, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()

        print(f"[SLAM] Визуализация сохранена: {png_filename}")
        print(f"[SLAM] Карта: {width}×{height}, Исследовано: {explored_pct:.1f}%")
        print(f"[SLAM] Робот: ({px}, {py}), Путь: {len(self.position_history)} точек")

    def load_map(self, filename: str):
        """Загрузка карты из файла"""
        import pickle
        with open(filename, 'rb') as f:
            data = pickle.load(f)

        self.map.grid = data['grid']
        self.map.resolution = data['resolution']
        self.map.origin_x = data['origin_x']
        self.map.origin_y = data['origin_y']
        self.position_history = deque(data['position_history'], maxlen=1000)
        self.landmarks = data['landmarks']

        if self.position_history:
            self.current_position = self.position_history[-1]

        print(f"[SLAM] Карта загружена: {filename}")

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

