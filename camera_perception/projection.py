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
) -> np.ndarray:
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

    Returns:
        N×3 float32 массив (x_world, y_world, z_world).
        Пустой массив shape (0, 3) если ничего не спроецировалось.
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
        return np.zeros((0, 3), dtype=np.float32)
    uu = uu[valid]; vv = vv[valid]; z_cam = z_cam[valid]

    # Pinhole back-projection:
    # x_cam = (u - cx) * z / fx, y_cam = (v - cy) * z / fy
    # В нашей камере: x_cam вправо, y_cam вниз, z_cam вперёд.
    x_cam = (uu - intrinsics.cx) * z_cam / intrinsics.fx
    y_cam = (vv - intrinsics.cy) * z_cam / intrinsics.fy

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

    return np.column_stack([x_w, y_w, z_w]).astype(np.float32)


def filter_obstacles_by_height(
    world_points: np.ndarray,
    min_height_m: float = 0.03,
    max_height_m: float = 0.50,
) -> np.ndarray:
    """
    Отфильтровать точки по высоте над полом — это и есть "препятствия для тележки".

    Точки ниже min_height — пол (там же ездим, не препятствие).
    Точки выше max_height — потолок / верх дверей / люстры (тележке всё равно).

    Args:
        world_points: N×3 (x, y, z) в мировой системе
        min_height_m, max_height_m: диапазон z, который считается препятствием
    """
    if len(world_points) == 0:
        return world_points
    z = world_points[:, 2]
    mask = (z >= min_height_m) & (z <= max_height_m)
    return world_points[mask]
