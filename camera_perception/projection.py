#!/usr/bin/env python3
"""
Обратная проекция пикселей карты глубины в мировые координаты.

Pipeline:
  пиксель (u, v) + depth d
    → точка в системе камеры: (X_c, Y_c, Z_c) через pinhole-модель
    → точка в системе робота: учитывая монтаж камеры (высота, тилт, offset)
    → точка в мировой системе: учитывая позу робота (x, y, θ) из SLAM

Дальше SLAM сам решает, отмечать ли её как препятствие в occupancy grid.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class CameraIntrinsics:
    """
    Параметры pinhole-камеры.

    Дефолты — типичная USB-камера 640×480 с горизонтальным FOV ≈ 60°.
    Для точности нужна калибровка шахматной доской (cv2.calibrateCamera),
    но для робота-тележки на ровном полу эти дефолты работают приемлемо.
    """
    width: int = 640
    height: int = 480
    fx: float = 554.0   # ≈ width / (2*tan(hfov/2)) для hfov=60°
    fy: float = 554.0
    cx: float = 320.0
    cy: float = 240.0

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float = 60.0) -> 'CameraIntrinsics':
        """Сконструировать из FOV — самое практичное при отсутствии калибровки."""
        f = width / (2.0 * np.tan(np.deg2rad(hfov_deg) / 2.0))
        return cls(width=width, height=height, fx=f, fy=f,
                   cx=width / 2.0, cy=height / 2.0)


@dataclass
class CameraMount:
    """
    Геометрия монтажа камеры на роботе.

    Система координат робота:
      x — вперёд, y — влево, z — вверх (правая тройка, как в ROS).
    Параметры камеры заданы в системе робота.
    """
    height_m: float = 0.10        # высота над полом
    forward_offset_m: float = 0.05 # смещение вперёд от центра робота
    tilt_rad: float = 0.0          # наклон вниз: 0 = горизонтально, >0 = смотрит вниз


def backproject_depth_to_world(
    depth_map: np.ndarray,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    robot_pose: tuple,
    pixel_stride: int = 8,
    min_depth_m: float = 0.15,
    max_depth_m: float = 3.0,
    use_lower_half_only: bool = True,
    return_pixels: bool = False,
):
    """
    Спроецировать пиксели depth-карты в мировые координаты.

    Args:
        depth_map: H×W float, глубина в метрах (от камеры по оси Z_camera)
        intrinsics: pinhole-параметры
        mount: геометрия монтажа камеры на роботе
        robot_pose: (x, y, theta) робота в мировой системе
        pixel_stride: брать каждый N-й пиксель — на Pi 640×480 это 4800 точек при stride=8
        min_depth_m, max_depth_m: фильтр валидных глубин
        use_lower_half_only: брать только нижнюю половину кадра (стол, пол, ножки)
                            — там реально лежат препятствия, верх кадра это потолок/далёкие стены
        return_pixels: вернуть также пиксельные координаты точек — нужно
                       отладочной визуализации, чтобы подсветить на кадре
                       пиксели, ставшие препятствиями

    Returns:
        N×3 float32 массив (x_world, y_world, z_world);
        при return_pixels=True — кортеж (points N×3, pixels N×2 int32 (u, v)).
        Пустые массивы, если ничего не спроецировалось.
    """
    h, w = depth_map.shape
    rx, ry, rtheta = robot_pose

    # Стартовая y — либо середина кадра, либо 0 (весь кадр)
    y_start = h // 2 if use_lower_half_only else 0

    # Сетка пикселей с шагом
    vs = np.arange(y_start, h, pixel_stride)
    us = np.arange(0, w, pixel_stride)
    uu, vv = np.meshgrid(us, vs)  # каждая shape (len(vs), len(us))
    uu = uu.flatten()
    vv = vv.flatten()

    # Глубина в этих пикселях
    z_cam = depth_map[vv, uu].astype(np.float32)

    # Фильтр валидных глубин
    valid = (z_cam >= min_depth_m) & (z_cam <= max_depth_m)
    if not valid.any():
        empty = np.zeros((0, 3), dtype=np.float32)
        if return_pixels:
            return empty, np.zeros((0, 2), dtype=np.int32)
        return empty
    uu = uu[valid]; vv = vv[valid]; z_cam = z_cam[valid]

    # Pinhole back-projection:
    # x_cam = (u - cx) * z / fx, y_cam = (v - cy) * z / fy
    # В нашей камере: x_cam вправо, y_cam вниз, z_cam вперёд.
    x_cam = (uu - intrinsics.cx) * z_cam / intrinsics.fx
    y_cam = (vv - intrinsics.cy) * z_cam / intrinsics.fy

    x_w, y_w, z_w = camera_to_world(x_cam, y_cam, z_cam, mount,
                                    (rx, ry, rtheta))

    points = np.column_stack([x_w, y_w, z_w]).astype(np.float32)
    if return_pixels:
        return points, np.column_stack([uu, vv]).astype(np.int32)
    return points


def camera_to_world(x_cam, y_cam, z_cam, mount, robot_pose):
    """
    Перевести точки из системы КАМЕРЫ (x вправо, y вниз, z вперёд) в МИРОВУЮ,
    учитывая монтаж камеры (высота/тилт/вынос) и позу робота.

    Векторно и скалярно работает одинаково (numpy broadcasting). Вынесено из
    backproject_depth_to_world, чтобы тот же переход переиспользовал floor_fusion
    (препятствия из seg+depth) — единая формула, без расхождений.

    Args:
        x_cam, y_cam, z_cam: координаты в системе камеры (массивы или скаляры)
        mount: CameraMount (height_m, forward_offset_m, tilt_rad)
        robot_pose: (x, y, theta) робота в мировой системе

    Returns:
        (x_w, y_w, z_w) — той же формы, что вход.
    """
    rx, ry, rtheta = robot_pose

    # Переход в систему робота (x_r вперёд, y_r влево, z_r вверх).
    # Сначала "горизонтальная" камера: x_r = z_cam, y_r = -x_cam, z_r = -y_cam
    # Затем поворот по тилту вокруг оси y_r:
    #   x_r' =  cos(tilt) * x_r + sin(tilt) * z_r
    #   z_r' = -sin(tilt) * x_r + cos(tilt) * z_r
    cos_t = np.cos(mount.tilt_rad)
    sin_t = np.sin(mount.tilt_rad)

    x_r_h = z_cam
    y_r_h = -x_cam
    z_r_h = -y_cam

    x_r = cos_t * x_r_h + sin_t * z_r_h + mount.forward_offset_m
    y_r = y_r_h
    z_r = -sin_t * x_r_h + cos_t * z_r_h + mount.height_m

    # Поворот в мировую систему (по θ робота) + сдвиг.
    cos_th = np.cos(rtheta)
    sin_th = np.sin(rtheta)
    x_w = rx + cos_th * x_r - sin_th * y_r
    y_w = ry + sin_th * x_r + cos_th * y_r
    z_w = z_r  # робот ездит по плоскому полу, мировой z = роботовский z
    return x_w, y_w, z_w


def filter_obstacles_by_height(
    world_points: np.ndarray,
    min_height_m: float = 0.03,
    max_height_m: float = 0.50,
    return_mask: bool = False,
):
    """
    Отфильтровать точки по высоте над полом — это и есть "препятствия для тележки".

    Точки ниже min_height — пол (там же ездим, не препятствие).
    Точки выше max_height — потолок / верх дверей / люстры (тележке всё равно).

    Args:
        world_points: N×3 (x, y, z) в мировой системе
        min_height_m, max_height_m: диапазон z, который считается препятствием
        return_mask: вернуть также булеву маску по входным точкам (для отладки —
                     какие именно пиксели стали препятствиями)
    """
    if len(world_points) == 0:
        if return_mask:
            return world_points, np.zeros(0, dtype=bool)
        return world_points
    z = world_points[:, 2]
    mask = (z >= min_height_m) & (z <= max_height_m)
    if return_mask:
        return world_points[mask], mask
    return world_points[mask]


def nearest_in_depth_band(
    depth_map: np.ndarray,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    band_halfheight_frac: float = 0.05,
    central_width_frac: float = 0.55,
    near_percentile: float = 10.0,
    min_valid_px: int = 30,
    min_depth_m: float = 0.15,
    max_depth_m: float = 6.0,
) -> Optional[float]:
    """
    «Максимальная близость объекта по курсу» прямо из карты глубины,
    БЕЗ обратной проекции и фильтра высот — страховочный канал.

    Зачем: основной пайплайн классифицирует препятствия по высоте точек
    (3–50 см над полом). Если облако получилось редким — сетка, прозрачное,
    тонкое, ошибка наклона камеры — фронт может промолчать. Эта функция
    смотрит на УЗКУЮ ГОРИЗОНТАЛЬНУЮ ПОЛОСУ вокруг линии горизонта (оптической
    оси с поправкой на tilt): пол в эти строки кадра попадает только на
    большой дальности (для камеры на высоте 12.5 см и полосы ±5% кадра —
    дальше ~2.9 м), поэтому всё БЛИЗКОЕ в полосе — вертикальное препятствие.

    Наивный минимум по всему кадру так не работает: внизу кадра всегда пол
    на 0.3–0.5 м — был бы вечный ложный стоп.

    Args:
        band_halfheight_frac: полувысота полосы в долях высоты кадра
        central_width_frac: доля ширины кадра по центру (куда едем)
        near_percentile: робастный «минимум» (отсекает битые пиксели глубины)
        min_valid_px: минимум валидных пикселей в полосе, иначе None

    Returns:
        Дистанция до ближайшего объекта по курсу ОТ ЦЕНТРА РОБОТА (м),
        или None (полоса пустая/невалидная).
    """
    h, w = depth_map.shape[:2]

    # Линия горизонта с учётом наклона камеры: tilt > 0 (вниз) поднимает
    # горизонт в кадре выше центра.
    horizon_row = int(round(intrinsics.cy - intrinsics.fy * np.tan(mount.tilt_rad)))
    half_band = max(2, int(round(h * band_halfheight_frac)))
    r0 = max(0, horizon_row - half_band)
    r1 = min(h, horizon_row + half_band)
    if r1 - r0 < 3:
        return None  # горизонт ушёл за кадр (сильный tilt) — канал не работает

    half_w = max(2, int(round(w * central_width_frac / 2)))
    c0 = max(0, w // 2 - half_w)
    c1 = min(w, w // 2 + half_w)

    band = depth_map[r0:r1, c0:c1]
    valid = band[(band >= min_depth_m) & (band <= max_depth_m)]
    if valid.size < min_valid_px:
        return None

    z_cam = float(np.percentile(valid, near_percentile))
    # Глубина — вдоль оптической оси камеры; до центра робота добавляем
    # вынос камеры (cos(tilt) ≈ 1 для малых углов учтён явно).
    return z_cam * float(np.cos(mount.tilt_rad)) + mount.forward_offset_m


def obstacle_distances_by_sector(
    obstacles_world: np.ndarray,
    robot_pose: tuple,
    front_halfwidth_rad: float = np.pi / 6,   # ±30°, как сектор front у лидара
    side_max_rad: float = np.pi / 2,          # боковые секторы до ±90°
    min_range_m: float = 0.12,                # шумовой порог (само-обзор/артефакты)
    max_range_m: float = 6.0,
    min_points: int = 3,
    near_percentile: float = 15.0,
    corridor_halfwidth_m: Optional[float] = None,
) -> dict:
    """
    Оценить расстояние до ближайшего препятствия в секторах front/left/right
    по облаку препятствий (мировые координаты, уже отфильтрованных по высоте).

    Предназначено для реактивного слоя (sensor_fusion), а не для карты. Секторы
    заданы относительно текущего курса робота, как у лидара: front — это ±30°.

    ВАЖНО: горизонтальный FOV камеры ~60° (±30°), поэтому реально заполняется
    почти только сектор front. Боковые секторы клиппируются краем кадра и обычно
    пусты — это ожидаемо, а не ошибка.

    КОРИДОРНАЯ МЕТРИКА (corridor_halfwidth_m): дополнительная оценка «сколько
    можно проехать прямо». Берутся точки, чьё ПОПЕРЕЧНОЕ смещение в системе
    робота не превышает полуширины коридора (полкорпуса + запас), независимо
    от пеленга — и из них считается продольная дистанция. Это закрывает два
    кейса, где угловой сектор слаб:
      - угловое препятствие «под колесо»: близкий предмет на пеленге >30°
        в сектор front не попадает, но в полосу движения — попадает;
      - стена под острым углом (диагональный подъезд): её ближний край
        лежит в коридоре, даже если основная масса точек в боковом секторе.
    Итоговый front = min(сектор, коридор).

    Робастная оценка ближнего препятствия — перцентиль near_percentile по
    дальностям точек сектора: защищает от одиночных выбросов глубины (которые
    иначе вызывали бы ложную экстренную остановку). Требуется минимум
    min_points точек, иначе сектор считается пустым (None).

    Args:
        obstacles_world: N×3 (x, y, z) в мировой системе, уже по высоте.
        robot_pose: (x, y, theta) робота в мировой системе.
        corridor_halfwidth_m: полуширина коридора движения (м); None — выкл.

    Returns:
        {'front': dist|None, 'left': dist|None, 'right': dist|None,
         'front_corridor': dist|None}  — front уже учитывает коридор.
    """
    result = {'front': None, 'left': None, 'right': None, 'front_corridor': None}
    if obstacles_world is None or len(obstacles_world) == 0:
        return result

    rx, ry, rtheta = robot_pose
    dx = obstacles_world[:, 0] - rx
    dy = obstacles_world[:, 1] - ry
    ranges = np.hypot(dx, dy)

    # Пеленг относительно курса робота: 0 — прямо по курсу, + — влево.
    bearings = np.arctan2(dy, dx) - rtheta
    bearings = (bearings + np.pi) % (2 * np.pi) - np.pi  # нормализация в [-pi, pi]

    valid = (ranges >= min_range_m) & (ranges <= max_range_m)

    sectors = {
        'front': np.abs(bearings) <= front_halfwidth_rad,
        'left':  (bearings > front_halfwidth_rad) & (bearings <= side_max_rad),
        'right': (bearings < -front_halfwidth_rad) & (bearings >= -side_max_rad),
    }

    for name, sec_mask in sectors.items():
        sel = ranges[valid & sec_mask]
        if sel.size >= min_points:
            result[name] = float(np.percentile(sel, near_percentile))

    # Коридор: точки в полосе движения робота, дистанция — продольная (x_r).
    if corridor_halfwidth_m is not None:
        cos_t = np.cos(rtheta)
        sin_t = np.sin(rtheta)
        x_r = cos_t * dx + sin_t * dy    # вперёд по курсу
        y_r = -sin_t * dx + cos_t * dy   # влево от курса
        in_corridor = ((x_r >= min_range_m) & (x_r <= max_range_m)
                       & (np.abs(y_r) <= corridor_halfwidth_m))
        sel = x_r[in_corridor]
        if sel.size >= min_points:
            corridor = float(np.percentile(sel, near_percentile))
            result['front_corridor'] = corridor
            if result['front'] is None or corridor < result['front']:
                result['front'] = corridor

    return result
