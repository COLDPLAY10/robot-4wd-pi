#!/usr/bin/env python3
"""
Ядро системы SLAM (Simultaneous Localization and Mapping)
Поддерживает работу с лидаром и без него (используя камеру и ультразвук)
"""

import os
import pickle
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# matplotlib импортируется лениво через map_visualizer — на Pi без X-сервера
# модуль SLAM должен импортироваться без ошибок даже если matplotlib не запустится.


# Длина историй траекторий: при ~20 Гц одометрии 1000 точек — всего ~50 с,
# длинные прогоны обрезались на PNG и в compare_trajectory. 20000 ≈ 15-30 мин.
_HISTORY_MAXLEN = 20000


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
        """Обновление вероятности занятости ячейки (логарифмический метод log-odds)"""
        gx, gy = self.world_to_grid(x, y)

        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return

        # Текущее значение вероятности [0..100] -> p [0.01..0.99]
        current_value = self.grid[gy, gx]
        p = np.clip(current_value / 100.0, 0.01, 0.99)

        # Текущий log-odds
        l_prev = np.log(p / (1.0 - p))

        # Log-odds измерения на основе confidence
        if occupied:
            p_meas = 0.5 + confidence * 0.45  # confidence=0.9 -> p_meas=0.905
        else:
            p_meas = 0.5 - confidence * 0.45  # confidence=0.5 -> p_meas=0.275

        p_meas = np.clip(p_meas, 0.01, 0.99)
        l_meas = np.log(p_meas / (1.0 - p_meas))

        # Обновление: вычитаем prior (log-odds 50% = 0), т.к. prior уже учтён в l_prev
        l_new = l_prev + l_meas

        # Ограничиваем log-odds для предотвращения чрезмерного насыщения
        l_new = np.clip(l_new, -5.0, 5.0)

        # Обратно в вероятность [0..100]
        p_new = 1.0 / (1.0 + np.exp(-l_new))
        self.grid[gy, gx] = p_new * 100.0

    def update_cells_batch(self, xs: np.ndarray, ys: np.ndarray,
                           occupied: bool, confidence: float):
        """
        Векторное log-odds обновление массива точек (мировые координаты).

        Зачем: лидарный оборот — ~600 лучей × десятки free-ячеек; поштучный
        update_cell (np.log/np.exp на скаляр) занимал 0.1–0.4 с на Pi и
        просаживал контур управления. Батч делает то же за миллисекунды.
        Дубликаты ячеек ВНУТРИ батча схлопываются: один скан голосует за
        ячейку один раз (поштучный вариант насыщал ячейку повторами).
        """
        gx = (xs / self.resolution + self.origin_x).astype(np.int32)
        gy = (ys / self.resolution + self.origin_y).astype(np.int32)
        m = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        if not m.any():
            return
        flat = np.unique(gy[m].astype(np.int64) * self.width + gx[m])
        gyu = (flat // self.width).astype(np.int32)
        gxu = (flat % self.width).astype(np.int32)

        p = np.clip(self.grid[gyu, gxu] / 100.0, 0.01, 0.99)
        l_prev = np.log(p / (1.0 - p))
        if occupied:
            p_meas = 0.5 + confidence * 0.45
        else:
            p_meas = 0.5 - confidence * 0.45
        p_meas = min(max(p_meas, 0.01), 0.99)
        l_new = np.clip(l_prev + np.log(p_meas / (1.0 - p_meas)), -5.0, 5.0)
        self.grid[gyu, gxu] = (100.0 / (1.0 + np.exp(-l_new))).astype(np.float32)

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
    Система SLAM для робота.

    Режимы работы (mapping_mode):
      'mapping'      — строим карту и одновременно корректируем позу
                        scan-to-map матчингом (если лидар включён).
      'localization' — карта загружается заранее, в неё больше ничего не пишем;
                        scan-to-map матчинг уточняет позу относительно готовой карты.
    """

    def __init__(self, map_width=400, map_height=400, resolution=0.05,
                 use_lidar=True, mapping_mode='mapping',
                 enable_scan_matching=True, wheel_base=0.097):
        """
        Args:
            map_width: ширина карты в ячейках
            map_height: высота карты в ячейках
            resolution: размер ячейки в метрах
            use_lidar: использовать ли лидар
            mapping_mode: 'mapping' | 'localization' — см. docstring класса
            enable_scan_matching: подключать ли scan matcher (для дебага можно
                                  выключить и работать только по одометрии)
            wheel_base: эффективная база диф-модели (м). Для мекалум-шасси это
                        сумма колеи и межосевого расстояния (см. WHEEL_BASE в
                        NavigationController). ОБЯЗАТЕЛЬНО должна совпадать с
                        тем, что использует NavigationController при расчёте
                        left/right_speed, иначе omega восстанавливается с
                        ошибкой и scan matcher не сходится.
        """
        if mapping_mode not in ('mapping', 'localization'):
            raise ValueError(f"mapping_mode должен быть 'mapping' или 'localization', "
                             f"передано: {mapping_mode!r}")

        self.use_lidar = use_lidar
        self.mapping_mode = mapping_mode
        self.wheel_base = float(wheel_base)
        self.map = OccupancyGrid(map_width, map_height, resolution)

        # Scan matcher подключаем только если есть лидар: без сканов он бесполезен.
        # Hector-style scan-to-map matching превращает dead-reckoning в настоящий SLAM.
        self.scan_matcher = None
        if use_lidar and enable_scan_matching:
            from .scan_matcher import HectorScanMatcher
            self.scan_matcher = HectorScanMatcher(resolution=resolution)

        # Текущая позиция робота — может быть скорректирована scan matcher'ом
        self.current_position = Position(x=0.0, y=0.0, theta=0.0, timestamp=time.time())

        # История позиций (SLAM, с коррекцией scan matcher).
        # ВАЖНО: кладём именно копию, иначе первая запись будет мутировать
        # вместе с current_position — и loop-closure всегда будет равен 0.
        self.position_history = deque(maxlen=_HISTORY_MAXLEN)
        self.position_history.append(Position(
            x=self.current_position.x, y=self.current_position.y,
            theta=self.current_position.theta,
            timestamp=self.current_position.timestamp))

        # Параллельный трекер чистой одометрии — копит дрейф независимо от
        # scan matcher. Нужен для отчёта "одометрия vs SLAM" на защите.
        self.odom_only_position = Position(x=0.0, y=0.0, theta=0.0, timestamp=time.time())
        self.odom_only_history = deque(maxlen=_HISTORY_MAXLEN)
        self.odom_only_history.append(Position(
            x=0.0, y=0.0, theta=0.0,
            timestamp=self.odom_only_position.timestamp))

        # Ориентиры
        self.landmarks = []

        # Параметры одометрии
        self.last_update_time = time.time()
        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.process_noise = np.diag([0.1, 0.1, 0.05])  # x, y, theta
        self.measurement_noise = np.diag([0.5, 0.5])    # измерения расстояний

        # Счётчики работы scan matcher — полезны и для отладки, и для дипломного отчёта.
        self.scan_match_total = 0
        self.scan_match_accepted = 0

        print(f"[SLAM] Инициализирована карта {map_width}x{map_height}, разрешение: {resolution}м")
        print(f"[SLAM] Режим лидара: {'включен' if use_lidar else 'выключен'}")
        print(f"[SLAM] Режим работы: {self.mapping_mode}, "
              f"scan matching: {'да' if self.scan_matcher is not None else 'нет'}")

    def update_odometry(self, left_speed: float, right_speed: float, dt: float):
        """
        Обновление одометрии на основе скоростей колес

        Args:
            left_speed: скорость левых колес (м/с)
            right_speed: скорость правых колес (м/с)
            dt: временной интервал (сек)
        """
        # Линейная и угловая скорость (диф-drive модель).
        # self.wheel_base ОБЯЗАН совпадать с тем, что использует контроллер
        # при формировании left/right_speed — иначе omega приходит с ошибкой.
        v = (left_speed + right_speed) / 2.0
        omega = (right_speed - left_speed) / self.wheel_base

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

        # Параллельно ведём чистую одометрию (без коррекции scan matcher).
        # Стартует с того же (0,0,0), интегрирует те же (v, omega), но никогда
        # не правится scan matcher'ом — копит весь дрейф колёс честно.
        op = self.odom_only_position
        op.x += v * np.cos(op.theta) * dt
        op.y += v * np.sin(op.theta) * dt
        op.theta = self._normalize_angle(op.theta + omega * dt)
        op.timestamp = self.current_position.timestamp
        self.odom_only_history.append(Position(
            x=op.x, y=op.y, theta=op.theta, timestamp=op.timestamp
        ))

    def update_with_lidar(self, scan_data: List[Tuple[float, float]]):
        """
        Один шаг SLAM по лидарному скану:
          1. (если есть scan matcher) уточняем позу через scan-to-map matching
             относительно current_position (которую обычно поставила одометрия)
          2. (если режим mapping) рисуем точки скана в occupancy grid

        Args:
            scan_data: список точек в полярных координатах [(angle, distance), ...]
                      angle в радианах, distance в метрах
        """
        if not self.use_lidar:
            return

        # ---- Шаг 1: коррекция позы scan matcher'ом ----
        if self.scan_matcher is not None:
            # Пересчёт likelihood field — не на каждом тике (дорого), а с
            # throttle'ом внутри матчера. В localization карта неизменна —
            # поле строится один раз (в load_map) и не пересчитывается.
            if self.mapping_mode == 'mapping' and self.scan_matcher.needs_refresh():
                self.scan_matcher.update_likelihood_field(self.map)

            if self.scan_matcher.has_field():
                self.scan_match_total += 1
                prior = (self.current_position.x,
                         self.current_position.y,
                         self.current_position.theta)
                result = self.scan_matcher.match(scan_data, prior)
                if result is not None:
                    (mx, my, mt), _score = result
                    self.current_position.x = mx
                    self.current_position.y = my
                    self.current_position.theta = mt
                    self.current_position.timestamp = time.time()
                    # Перепишем последнюю запись истории, чтобы траектория
                    # отражала уточнённую позу, а не одометрию.
                    if self.position_history:
                        self.position_history[-1] = Position(
                            x=mx, y=my, theta=mt,
                            timestamp=self.current_position.timestamp,
                        )
                    self.scan_match_accepted += 1

        # ---- Шаг 2: обновление карты (только в режиме mapping) ----
        if self.mapping_mode != 'mapping':
            return

        robot_x = self.current_position.x
        robot_y = self.current_position.y
        robot_theta = self.current_position.theta

        # Векторная запись скана (см. update_cells_batch): препятствия одним
        # батчем, free-лучи — общей сеткой параметров с маской по длине луча.
        pts = np.asarray([(a, d) for a, d in scan_data if 0.0 < d <= 10.0],
                         dtype=np.float32)
        if len(pts) == 0:
            return
        global_angles = robot_theta + pts[:, 0]
        dists = pts[:, 1]
        dx = dists * np.cos(global_angles)
        dy = dists * np.sin(global_angles)

        # 1) Препятствия
        self.map.update_cells_batch(robot_x + dx, robot_y + dy,
                                    occupied=True, confidence=0.9)

        # 2) Свободное пространство вдоль лучей: для каждого луча шаги
        #    i/num_steps, i=1..num_steps-1; сетка (лучи × max_steps) с маской.
        steps = np.maximum(1, (dists / self.map.resolution).astype(np.int32))
        max_steps = int(steps.max())
        if max_steps > 1:
            i_idx = np.arange(1, max_steps, dtype=np.float32)[None, :]
            valid = i_idx < steps[:, None]
            t = i_idx / steps[:, None].astype(np.float32)
            free_x = robot_x + t * dx[:, None]
            free_y = robot_y + t * dy[:, None]
            self.map.update_cells_batch(free_x[valid], free_y[valid],
                                        occupied=False, confidence=0.5)

    def update_with_ultrasonic(self, distance: float, sensor_angle: float = 0.0):
        """
        Обновление карты данными ультразвукового датчика
        Ультразвук имеет конус ~30°, моделируем несколькими лучами

        Args:
            distance: расстояние в метрах
            sensor_angle: угол датчика относительно робота (радианы)
        """
        if self.mapping_mode != 'mapping':
            return  # в localization-режиме карта read-only
        if distance <= 0 or distance > 4.0:  # макс дальность ультразвука ~4м
            return

        robot_x = self.current_position.x
        robot_y = self.current_position.y
        robot_theta = self.current_position.theta

        # Конус ультразвука ~30° (±15°), разбиваем на несколько лучей
        cone_half_angle = np.deg2rad(15)
        num_rays = 7  # лучей в конусе

        for i in range(num_rays):
            # Угол луча внутри конуса
            ray_offset = -cone_half_angle + (2 * cone_half_angle) * i / (num_rays - 1)
            global_angle = robot_theta + sensor_angle + ray_offset

            # Расстояние по краям конуса может быть чуть больше (геометрия)
            ray_distance = distance / np.cos(ray_offset) if abs(ray_offset) < np.deg2rad(14) else distance

            # Координаты препятствия
            obs_x = robot_x + ray_distance * np.cos(global_angle)
            obs_y = robot_y + ray_distance * np.sin(global_angle)

            # Уверенность ниже по краям конуса
            edge_factor = 1.0 - abs(ray_offset) / cone_half_angle
            ray_confidence = 0.4 + 0.2 * edge_factor  # от 0.4 на краю до 0.6 в центре

            self.map.update_cell(obs_x, obs_y, occupied=True, confidence=ray_confidence)

            # Свободное пространство по лучу
            num_steps = max(2, int(ray_distance / (self.map.resolution * 2)))
            for j in range(1, num_steps):
                t = j / num_steps
                free_x = robot_x + t * ray_distance * np.cos(global_angle)
                free_y = robot_y + t * ray_distance * np.sin(global_angle)
                self.map.update_cell(free_x, free_y, occupied=False, confidence=0.3 * edge_factor + 0.1)

    def update_with_camera_depth(self,
                                 depth_map: np.ndarray,
                                 intrinsics,
                                 mount,
                                 pixel_stride: int = 8,
                                 min_obstacle_height_m: float = 0.03,
                                 max_obstacle_height_m: float = 0.50,
                                 max_range_m: float = 6.0,
                                 map_write_max_range_m: Optional[float] = None,
                                 robot_pose: Optional[Tuple[float, float, float]] = None):
        """
        Обновление карты препятствий из карты глубины (Depth-Anything-V2).

        Логика:
          1. Обратная проекция пикселей глубины в мировые координаты.
          2. Фильтр по высоте: только точки на 3-50 см над полом считаются
             препятствиями для тележки.
          3. Эти точки записываются в occupancy grid; вдоль луча от робота к
             каждой точке клетки помечаются как свободные.

        Args:
            depth_map: H×W float, метрическая глубина в метрах
            intrinsics: CameraIntrinsics из camera_perception
            mount: CameraMount из camera_perception
            pixel_stride: брать каждый N-й пиксель (компромисс точность/скорость)
            min/max_obstacle_height_m: диапазон высот, классифицируемых как препятствие
            max_range_m: предел анализа сцены — возвращаемое облако препятствий
                         (реактивный слой, выбор направления) считается до него
            map_write_max_range_m: предел ЗАПИСИ В КАРТУ; None = max_range_m.
                         Монокулярная глубина с дистанцией шумит всё сильнее,
                         а occupied-ячейка (conf 0.7) сразу блокирует A* —
                         поэтому в карту пишем консервативнее, чем анализируем.
            robot_pose: поза робота (x, y, theta) В МОМЕНТ ЗАХВАТА КАДРА.
                         Инференс глубины занимает 0.3–3 с в фоновом потоке —
                         к моменту потребления робот успевает уехать, и проекция
                         по текущей позе сдвигает облако на пройденный путь.
                         None = текущая поза (для совместимости со старыми
                         вызовами, где кадр обрабатывался синхронно).
        """
        if depth_map is None or depth_map.size == 0:
            return np.zeros((0, 3), dtype=np.float32)

        # Ленивый импорт — slam_core не должен жёстко тянуть camera_perception
        from camera_perception.projection import (backproject_depth_to_world,
                                                   filter_obstacles_by_height)

        if robot_pose is not None:
            robot_x, robot_y, robot_theta = robot_pose
        else:
            robot_x = self.current_position.x
            robot_y = self.current_position.y
            robot_theta = self.current_position.theta

        points = backproject_depth_to_world(
            depth_map, intrinsics, mount,
            robot_pose=(robot_x, robot_y, robot_theta),
            pixel_stride=pixel_stride,
            min_depth_m=0.15,
            max_depth_m=max_range_m,
            use_lower_half_only=True,
        )
        obstacles = filter_obstacles_by_height(points,
                                               min_height_m=min_obstacle_height_m,
                                               max_height_m=max_obstacle_height_m)

        # Запись в карту — только в режиме построения; в localization карта
        # read-only. Облако препятствий при этом возвращается ВСЕГДА: оно нужно
        # реактивному слою (sensor_fusion) и в режиме навигации к цели тоже.
        if self.mapping_mode == 'mapping':
            map_cap = (map_write_max_range_m if map_write_max_range_m is not None
                       else max_range_m)

            # 1. Препятствия: пишем точку как occupied — но только ближние,
            #    дальняя монокулярная глубина слишком шумная для карты.
            if len(obstacles) > 0:
                obs_dist = np.hypot(obstacles[:, 0] - robot_x,
                                    obstacles[:, 1] - robot_y)
                near_obstacles = obstacles[obs_dist <= map_cap]
            else:
                near_obstacles = obstacles
            for ox, oy, _oz in near_obstacles:
                self.map.update_cell(float(ox), float(oy),
                                     occupied=True, confidence=0.7)

            # 2. Свободное пространство по лучу от робота к каждой точке глубины
            #    (НЕ только препятствие — любая depth-точка значит "до неё пусто").
            #    Лучи — тоже только в пределах map_cap, и с подвыборкой,
            #    чтобы не записывать слишком много клеток.
            if len(points) > 0:
                pts_dist = np.hypot(points[:, 0] - robot_x,
                                    points[:, 1] - robot_y)
                ray_candidates = points[pts_dist <= map_cap]
            else:
                ray_candidates = points
            ray_stride = max(1, pixel_stride * 2)
            ray_points = ray_candidates[::ray_stride]
            for tx, ty, _tz in ray_points:
                dist = float(np.hypot(tx - robot_x, ty - robot_y))
                if dist < self.map.resolution:
                    continue
                num_steps = max(2, int(dist / (self.map.resolution * 2)))
                for i in range(1, num_steps):
                    t = i / num_steps
                    free_x = robot_x + t * (tx - robot_x)
                    free_y = robot_y + t * (ty - robot_y)
                    self.map.update_cell(free_x, free_y,
                                         occupied=False, confidence=0.3)

        return obstacles

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

    def save_map(self, filename: str, inflation_radius_m: float = 0.15):
        """
        Сохранение карты в файл (PKL + PNG визуализация).

        Args:
            filename: путь к файлу (расширение игнорируется, .pkl и .png
                      записываются рядом)
            inflation_radius_m: радиус, на который PNG-визуализация раздует
                                препятствия полупрозрачным красным — это та
                                запретная зона, по которой A* отказывается ехать.
                                Совпадает с дефолтами PathPlanner (0.10+0.05м).
                                Передайте 0, чтобы не рисовать оверлей.
        """

        base_filename = os.path.splitext(filename)[0]
        pkl_filename = base_filename + '.pkl'
        png_filename = base_filename + '.png'

        data = {
            'grid': self.map.grid,
            'resolution': self.map.resolution,
            'origin_x': self.map.origin_x,
            'origin_y': self.map.origin_y,
            'position_history': list(self.position_history),
            'odom_only_history': list(self.odom_only_history),
            'landmarks': self.landmarks,
            'use_lidar': self.use_lidar,
            'mapping_mode': self.mapping_mode,
            'scan_match_total': self.scan_match_total,
            'scan_match_accepted': self.scan_match_accepted,
        }

        with open(pkl_filename, 'wb') as f:
            pickle.dump(data, f)
        print(f"[SLAM] Данные карты сохранены: {pkl_filename}")

        # Визуализация — лениво, через отдельный модуль. Если matplotlib
        # недоступен (например, headless Pi без установленного пакета) —
        # тихо пропускаем, .pkl всё равно сохранён.
        try:
            from .map_visualizer import render_map_png
            ok = render_map_png(
                grid=self.map.grid,
                resolution=self.map.resolution,
                origin_x=self.map.origin_x,
                origin_y=self.map.origin_y,
                current_position=(self.current_position.x,
                                  self.current_position.y,
                                  self.current_position.theta),
                position_history=list(self.position_history),
                png_filename=png_filename,
                title_suffix=f"режим: {self.mapping_mode}",
                inflation_radius_m=inflation_radius_m,
            )
            if ok:
                print(f"[SLAM] Визуализация сохранена: {png_filename}")
        except Exception as e:
            print(f"[SLAM] Не удалось отрисовать PNG: {e}")

    def load_map(self, filename: str, reset_pose: bool = False):
        """
        Загрузка карты из файла.

        Args:
            filename: путь к .pkl карты
            reset_pose: если True, ставим робота в (0,0,0); иначе подтягиваем
                        последнюю позицию из истории, как в старом поведении.
                        Для localization чаще нужно reset_pose=True — робот
                        стартует с известного места, не из конца чужой траектории.
        """
        import pickle
        with open(filename, 'rb') as f:
            data = pickle.load(f)

        # Размеры карты должны совпасть с инициализированными — иначе сетка
        # ляжет на чужие индексы. Лучше упасть сразу с понятной ошибкой.
        loaded_grid = data['grid']
        if loaded_grid.shape != self.map.grid.shape:
            raise ValueError(
                f"[SLAM] Размер карты не совпадает: загружено {loaded_grid.shape}, "
                f"инициализировано {self.map.grid.shape}. Пересоздайте SLAM с тем же "
                f"map_width/map_height."
            )

        self.map.grid = loaded_grid
        self.map.resolution = data['resolution']
        self.map.origin_x = data['origin_x']
        self.map.origin_y = data['origin_y']
        self.landmarks = data['landmarks']

        if reset_pose:
            # Считаем что робот сейчас стоит в (0,0) — оператор должен
            # физически поставить его в эту точку перед запуском.
            # Кладём в history именно копии (см. __init__ — иначе loop-closure = 0).
            now = time.time()
            self.current_position = Position(x=0.0, y=0.0, theta=0.0, timestamp=now)
            self.position_history = deque(maxlen=_HISTORY_MAXLEN)
            self.position_history.append(Position(x=0.0, y=0.0, theta=0.0, timestamp=now))

            self.odom_only_position = Position(x=0.0, y=0.0, theta=0.0, timestamp=now)
            self.odom_only_history = deque(maxlen=_HISTORY_MAXLEN)
            self.odom_only_history.append(Position(x=0.0, y=0.0, theta=0.0, timestamp=now))
        else:
            self.position_history = deque(data['position_history'], maxlen=_HISTORY_MAXLEN)
            if self.position_history:
                # Копия, не алиас: иначе первый же update_odometry мутирует
                # загруженную точку истории (та же ловушка, что в __init__)
                last = self.position_history[-1]
                self.current_position = Position(
                    x=last.x, y=last.y, theta=last.theta, timestamp=last.timestamp)
            # odom_only_history появилось позже формата — поддерживаем старые .pkl
            saved_odom = data.get('odom_only_history')
            if saved_odom:
                self.odom_only_history = deque(saved_odom, maxlen=_HISTORY_MAXLEN)
                self.odom_only_position = self.odom_only_history[-1]

        # Если scan matcher включён — сразу строим likelihood field, чтобы
        # первый же скан мог уточнить позу относительно загруженной карты.
        if self.scan_matcher is not None:
            self.scan_matcher.update_likelihood_field(self.map)

        print(f"[SLAM] Карта загружена: {filename} (режим: {self.mapping_mode})")

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

