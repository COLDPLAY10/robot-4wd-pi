#!/usr/bin/env python3
"""
Модуль объединения данных сенсоров (Sensor Fusion)
Комбинирует данные от лидара, камеры и ультразвукового датчика
"""

import numpy as np
from typing import Optional, Dict, Any
from dataclasses import dataclass
import time


@dataclass
class SensorData:
    """Данные от одного сенсора"""
    sensor_type: str  # 'lidar', 'camera', 'ultrasonic'
    data: Any
    confidence: float
    timestamp: float


class SensorFusion:
    """
    Класс для объединения данных от различных сенсоров
    Реализует приоритетную логику выбора данных
    """

    def __init__(self, use_lidar=True, use_camera=True, use_ultrasonic=True):
        """
        Args:
            use_lidar: использовать лидар
            use_camera: использовать камеру
            use_ultrasonic: использовать ультразвуковой датчик
        """
        self.use_lidar = use_lidar
        self.use_camera = use_camera
        self.use_ultrasonic = use_ultrasonic

        # Буферы данных сенсоров
        self.lidar_data = None
        self.camera_data = None
        self.ultrasonic_data = None

        # Время последнего обновления
        self.last_lidar_update = 0
        self.last_camera_update = 0
        self.last_ultrasonic_update = 0

        # Приоритеты сенсоров
        self.sensor_priority = {
            'lidar': 3,        # наивысший приоритет для твердых препятствий
            'camera': 2,       # средний приоритет, видит то что лидар не видит
            'ultrasonic': 1    # низкий приоритет, дополнительная безопасность
        }

        # Таймауты данных (секунды)
        self.data_timeout = 1.0

        print(f"[SensorFusion] Инициализирована")
        print(f"  Лидар: {'да' if use_lidar else 'нет'}")
        print(f"  Камера: {'да' if use_camera else 'нет'}")
        print(f"  Ультразвук: {'да' if use_ultrasonic else 'нет'}")

    def update_lidar(self, scan_data):
        """
        Обновление данных лидара

        Args:
            scan_data: список точек [(angle, distance), ...]
        """
        if not self.use_lidar:
            return

        self.lidar_data = SensorData(
            sensor_type='lidar',
            data=scan_data,
            confidence=0.9,
            timestamp=time.time()
        )
        self.last_lidar_update = time.time()

    def update_camera(self, segmentation_mask):
        """
        Обновление данных камеры

        Args:
            segmentation_mask: маска сегментации (numpy array)
        """
        if not self.use_camera:
            return

        self.camera_data = SensorData(
            sensor_type='camera',
            data=segmentation_mask,
            confidence=0.7,
            timestamp=time.time()
        )
        self.last_camera_update = time.time()

    def update_ultrasonic(self, distance: float):
        """
        Обновление данных ультразвукового датчика

        Args:
            distance: расстояние до препятствия в метрах
        """
        if not self.use_ultrasonic:
            return

        self.ultrasonic_data = SensorData(
            sensor_type='ultrasonic',
            data=distance,
            confidence=0.6,
            timestamp=time.time()
        )
        self.last_ultrasonic_update = time.time()

    def get_obstacle_distance(self, direction: str = 'front') -> Optional[float]:
        """
        Получение расстояния до ближайшего препятствия

        Args:
            direction: направление ('front', 'left', 'right', 'back')

        Returns:
            расстояние в метрах или None
        """
        current_time = time.time()
        distances = []

        # Проверяем лидар (наивысший приоритет)
        if self.use_lidar and self.lidar_data is not None:
            if current_time - self.last_lidar_update < self.data_timeout:
                # Извлекаем минимальное расстояние из сканов лидара
                min_dist = self._extract_lidar_distance(self.lidar_data.data, direction)
                if min_dist is not None:
                    distances.append((min_dist, self.sensor_priority['lidar']))

        # Проверяем ультразвук
        if self.use_ultrasonic and self.ultrasonic_data is not None:
            if current_time - self.last_ultrasonic_update < self.data_timeout:
                if direction == 'front':  # ультразвук обычно смотрит вперед
                    dist = self.ultrasonic_data.data
                    if dist > 0 and dist < 4.0:
                        distances.append((dist, self.sensor_priority['ultrasonic']))

        # Проверяем камеру
        if self.use_camera and self.camera_data is not None:
            if current_time - self.last_camera_update < self.data_timeout:
                # Оценка расстояния по сегментации
                dist = self._estimate_distance_from_segmentation(
                    self.camera_data.data, direction
                )
                if dist is not None:
                    distances.append((dist, self.sensor_priority['camera']))

        # Выбираем расстояние с наивысшим приоритетом
        if not distances:
            return None

        # Сортируем по приоритету (убывание)
        distances.sort(key=lambda x: x[1], reverse=True)

        # Если есть данные от высокоприоритетных сенсоров, используем их
        # Иначе берем минимальное расстояние (безопаснее)
        if distances[0][1] >= self.sensor_priority['lidar']:
            return distances[0][0]
        else:
            return min(d[0] for d in distances)

    def is_obstacle_detected(self, min_distance: float = 0.3) -> bool:
        """
        Проверка наличия препятствия впереди

        Args:
            min_distance: минимальное безопасное расстояние в метрах

        Returns:
            True если препятствие обнаружено
        """
        distance = self.get_obstacle_distance('front')

        if distance is None:
            return False

        return distance < min_distance

    def get_free_directions(self, min_distance: float = 0.5) -> Dict[str, bool]:
        """
        Получение свободных направлений

        Args:
            min_distance: минимальное безопасное расстояние

        Returns:
            словарь {направление: свободно}
        """
        directions = ['front', 'left', 'right', 'back']
        free_dirs = {}

        for direction in directions:
            dist = self.get_obstacle_distance(direction)
            free_dirs[direction] = dist is None or dist > min_distance

        return free_dirs

    def get_all_sensor_data(self) -> Dict[str, Optional[SensorData]]:
        """Получение всех данных сенсоров"""
        current_time = time.time()

        result = {}

        if self.use_lidar:
            if self.lidar_data and current_time - self.last_lidar_update < self.data_timeout:
                result['lidar'] = self.lidar_data
            else:
                result['lidar'] = None

        if self.use_camera:
            if self.camera_data and current_time - self.last_camera_update < self.data_timeout:
                result['camera'] = self.camera_data
            else:
                result['camera'] = None

        if self.use_ultrasonic:
            if self.ultrasonic_data and current_time - self.last_ultrasonic_update < self.data_timeout:
                result['ultrasonic'] = self.ultrasonic_data
            else:
                result['ultrasonic'] = None

        return result

    def _extract_lidar_distance(self, scan_data, direction: str) -> Optional[float]:
        """Извлечение расстояния из данных лидара для заданного направления"""
        if not scan_data:
            return None

        # Определяем диапазон углов для направления
        angle_ranges = {
            'front': (-np.pi/6, np.pi/6),      # ±30 градусов
            'right': (-np.pi/2, -np.pi/6),     # -90 до -30
            'left': (np.pi/6, np.pi/2),        # 30 до 90
            'back': (2*np.pi/3, 4*np.pi/3)     # 120 до 240
        }

        if direction not in angle_ranges:
            return None

        min_angle, max_angle = angle_ranges[direction]
        min_dist = float('inf')

        for angle, distance in scan_data:
            # Нормализуем угол
            angle = self._normalize_angle(angle)

            if min_angle <= angle <= max_angle:
                if 0 < distance < min_dist:
                    min_dist = distance

        return min_dist if min_dist != float('inf') else None

    def _estimate_distance_from_segmentation(self, mask, direction: str) -> Optional[float]:
        """Оценка расстояния на основе сегментации"""
        if mask is None or mask.size == 0:
            return None

        h, w = mask.shape

        # Определяем область интереса для направления
        if direction == 'front':
            roi = mask[h//2:h, w//3:2*w//3]
        elif direction == 'left':
            roi = mask[h//2:h, 0:w//3]
        elif direction == 'right':
            roi = mask[h//2:h, 2*w//3:w]
        else:
            return None

        # Ищем первое препятствие (снизу вверх)
        for row in range(roi.shape[0] - 1, -1, -1):
            if np.any(roi[row] > 0):
                # Оценка расстояния по вертикальной позиции
                distance = 3.0 * (1.0 - (roi.shape[0] - row) / roi.shape[0])
                return max(0.2, distance)

        return None

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

