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
        """Обновление вероятности занятости ячейки"""
        gx, gy = self.world_to_grid(x, y)

        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return

        if occupied:
            # Увеличиваем вероятность занятости
            self.grid[gy, gx] = min(100, self.grid[gy, gx] + confidence * 30)
        else:
            # Уменьшаем вероятность занятости (свободная область)
            self.grid[gy, gx] = max(0, self.grid[gy, gx] - confidence * 10)

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
      """Сохранение карты в файл - ТОЛЬКО PNG"""
      import matplotlib.pyplot as plt
      import numpy as np
      import os
      
      # Убедимся что расширение .png
      if not filename.endswith('.png'):
          filename = filename + '.png'
      
      # Фиксированное имя файла (перезаписывается каждый раз)
      fixed_filename = "slam_map.png"
      
      print(f"[SLAM] Сохранение карты как PNG: {fixed_filename}")
      
      # Создаем визуализацию карты
      plt.figure(figsize=(12, 10))
      
      # Преобразуем карту для отображения
      # 0-50 = свободно, 51-100 = занято, 50 = неизвестно
      display_map = self.map.grid.copy()
      
      # Нормализуем для изображения (0-1)
      # Инвертируем: занято = темное, свободно = светлое
      normalized_map = (100.0 - display_map) / 100.0
      
      # Определяем границы отображения
      height, width = display_map.shape
      extent = [0, width, 0, height]
      
      plt.imshow(normalized_map, cmap='gray', origin='lower', 
                vmin=0, vmax=1, extent=extent)
      
      # Добавляем позицию робота
      px, py = self.map.world_to_grid(self.current_position.x, self.current_position.y)
      plt.plot(px, py, 'ro', markersize=15, label='Робот', markerfacecolor='red', markeredgecolor='black')
      
      # Добавляем историю пути
      if len(self.position_history) > 1:
          path_x = []
          path_y = []
          for pos in self.position_history:
              px, py = self.map.world_to_grid(pos.x, pos.y)
              path_x.append(px)
              path_y.append(py)
          plt.plot(path_x, path_y, 'b-', alpha=0.7, linewidth=2, label='Путь')
      
      plt.title(f"SLAM карта\nРазрешение: {self.map.resolution}м/пиксель")
      plt.xlabel("Пиксели (X)")
      plt.ylabel("Пиксели (Y)")
      plt.legend(loc='upper right')
      plt.colorbar(label="0=занято, 1=свободно")
      plt.grid(True, alpha=0.3, linestyle='--')
      
      # Сохраняем с высоким качеством
      plt.savefig(fixed_filename, dpi=150, bbox_inches='tight', facecolor='white')
      plt.close()
      
      print(f"[SLAM] ✅ Карта сохранена: {fixed_filename}")
      print(f"[SLAM] Размер: {width}x{height}, Позиция робота: ({px}, {py})")

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

