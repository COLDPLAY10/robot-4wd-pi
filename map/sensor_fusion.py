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

        # Таймаут актуальности данных (секунды). Должен быть БОЛЬШЕ периода
        # самого медленного источника — инференс глубины на Pi ~1–3с. При 1.0с
        # данные ультразвука/камеры успевали «протухнуть» за время одного тика
        # с камерой, и слияние их выбрасывало («ультразвук не сканируется»).
        self.data_timeout = 2.5

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

    def update_camera(self, obstacle_distances):
        """
        Обновление данных камеры (depth) для реактивного слоя.

        Args:
            obstacle_distances: словарь секторных расстояний до ближайшего
                препятствия, например {'front': 0.42, 'left': None, 'right': None}.
                Получается из карты глубины через
                camera_perception.obstacle_distances_by_sector.

        Примечание: раньше сюда подавалась маска сегментации «пол/стены».
        Тот подход (camera_segmentation/) удалён; камера теперь даёт метрическую
        глубину, поэтому реактивный слой работает с готовыми расстояниями.
        """
        if not self.use_camera:
            return

        self.camera_data = SensorData(
            sensor_type='camera',
            data=obstacle_distances or {},
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

        # --- Лидар: основной дальномер ---
        lidar_dist = None
        if (self.use_lidar and self.lidar_data is not None
                and current_time - self.last_lidar_update < self.data_timeout):
            lidar_dist = self._extract_lidar_distance(self.lidar_data.data, direction)

        # --- Ультразвук: только вперёд ---
        ultrasonic_dist = None
        if (self.use_ultrasonic and self.ultrasonic_data is not None
                and current_time - self.last_ultrasonic_update < self.data_timeout
                and direction == 'front'):
            dist = self.ultrasonic_data.data
            if 0.0 < dist < 4.0:
                ultrasonic_dist = dist

        # Базовая оценка по «надёжным» дальномерам (лидар + ультразвук).
        base = self._combine_range_sensors(lidar_dist, ultrasonic_dist)

        # --- Камера (depth): секторные расстояния из карты глубины ---
        camera_dist = None
        if (self.use_camera and self.camera_data is not None
                and current_time - self.last_camera_update < self.data_timeout):
            camera_dist = self.camera_data.data.get(direction)

        # Слияние камеры с базовой оценкой.
        # Принцип безопасности: камера может только СОКРАТИТЬ расстояние (сделать
        # поведение осторожнее), но никогда не объявляет «свободно» в обход лидара.
        if direction == 'front':
            # Фронт: камера в полном доверии — берём минимум. Камера-глубина
            # видит низкие препятствия, которые 2D-лидар на своей высоте пропускает.
            return self._min_ignore_none(base, camera_dist)

        # Бока: камера слабая (горизонтальный FOV ~60°, край кадра шумит) —
        # используем её только как gap-fill там, где у лидара нет данных.
        if base is not None:
            return base
        return camera_dist

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
        """
        Извлечение расстояния из данных лидара для заданного направления.
        Использует медианную фильтрацию для устойчивости к шуму.
        """
        if not scan_data:
            return None

        # Определяем диапазон углов для направления
        angle_ranges = {
            'front': (-np.pi/6, np.pi/6),      # ±30 градусов
            'right': (-np.pi/2, -np.pi/6),     # -90 до -30
            'left': (np.pi/6, np.pi/2),        # 30 до 90
            'back': None                         # обрабатывается отдельно
        }

        if direction not in angle_ranges:
            return None

        # Минимальное доверительное расстояние лидара — точки ближе считаются шумом
        LIDAR_MIN_RELIABLE_DIST = 0.12  # 12 см — ближе этого T-MINI Plus шумит

        sector_distances = []

        for angle, distance in scan_data:
            # Отбрасываем заведомо шумные точки (слишком близкие)
            if distance < LIDAR_MIN_RELIABLE_DIST:
                continue

            angle = self._normalize_angle(angle)

            if direction == 'back':
                if abs(angle) > 5 * np.pi / 6:
                    sector_distances.append(distance)
            else:
                min_angle, max_angle = angle_ranges[direction]
                if min_angle <= angle <= max_angle:
                    sector_distances.append(distance)

        if not sector_distances:
            return None

        # Медианная фильтрация: берём медиану нижних 25% точек (ближайших),
        # но только если их достаточно (минимум 3 точки), иначе — одиночный выброс
        sector_distances.sort()
        if len(sector_distances) < 3:
            # Мало точек — недостаточно данных, не доверяем
            return None

        # Берём нижний квартиль и считаем его медиану
        q1_count = max(3, len(sector_distances) // 4)
        closest_points = sector_distances[:q1_count]
        median_dist = float(np.median(closest_points))

        return median_dist

    @staticmethod
    def _combine_range_sensors(lidar_dist: Optional[float],
                               ultrasonic_dist: Optional[float]) -> Optional[float]:
        """
        Кросс-валидация лидара и ультразвука — двух «надёжных» дальномеров.
        Лидар основной; ультразвук уточняет вблизи и страхует от шума лидара.
        """
        if lidar_dist is not None and ultrasonic_dist is not None:
            # Лидар говорит «очень близко», а ультразвук «свободно» — лидар
            # шумит, доверяем ультразвуку.
            if lidar_dist < 0.20 and ultrasonic_dist > 0.40:
                print(f"[SensorFusion] ⚠️ Конфликт: лидар={lidar_dist:.2f}м, "
                      f"ультразвук={ultrasonic_dist:.2f}м → доверяем ультразвуку")
                return ultrasonic_dist
            # Оба примерно согласны — средневзвешенное (лидар точнее).
            return lidar_dist * 0.6 + ultrasonic_dist * 0.4
        if lidar_dist is not None:
            return lidar_dist
        return ultrasonic_dist

    @staticmethod
    def _min_ignore_none(*values) -> Optional[float]:
        """Минимум из переданных значений, игнорируя None. None — если все None."""
        present = [v for v in values if v is not None]
        return min(present) if present else None

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

