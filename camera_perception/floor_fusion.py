#!/usr/bin/env python3
"""
Слияние сегментации пола и глубины: «препятствие = НЕ пол И приподнято».

Зачем именно так (проверено на test_pictures, см. scripts/eval_test_pictures.py):
  - Сегментация (SegFormer) — ПЕРВИЧНЫЙ классификатор «пол / не-пол». Она не
    зависит от калибровки камеры и от абсолютного масштаба глубины (костыль ÷4),
    и уверенно отделяет вертикальные тонкие объекты (ножки стола/стула, косяк),
    которые 2D-лидар по высоте пропускает — это и есть главная ценность камеры.
  - Высота над полом — НЕ классификатор, а грубый ГЕЙТ. Монокулярная глубина
    «смазывает» тонкое/блестящее (металлическая ножка высотой 0.5 м читалась как
    0.15 м над полом), поэтому точному значению высоты доверять нельзя. Высота
    нужна лишь чтобы (а) не считать препятствием плоский промах сегментации
    (полированный бетон, который сеть зовёт не-полом) — иначе робот вставал бы на
    ровном месте; (б) отсечь потолок/верх стен по верхнему порогу.

Контракт классификации точки (по ВСЕМУ кадру, не только нижней половине):
    obstacle = (НЕ floor  И  min_h ≤ h ≤ max_h)
               ИЛИ (strong_h ≤ h ≤ max_h)        # объект в цвет пола: seg ошибся,
                                                  # но геометрия явно поднята
    free     = floor  И  h < min_h
где h — высота над ПЛОСКОСТЬЮ, подогнанной по floor-пикселям этого кадра
(а не над mount-калибровкой) — это снимает зависимость от наклона камеры и
масштаба глубины: h относительна внутри кадра.

max_h (≈0.6 м) — ceiling-guard: заменяет прежнюю эвристику «только нижняя
половина кадра». Робот низкий, всё выше max_h колесу не угроза.

Если плоскость подогнать нельзя (мало floor-пикселей: робот уткнулся в стену,
промах сегментации, тёмный кадр) — функция возвращает None. Вызывающий ОБЯЗАН
откатиться на legacy height-filter (slam_core.update_with_camera_depth), а НЕ
считать, что препятствий нет. Это самый момент, когда препятствие важнее всего.

Чистый numpy: ни onnxruntime, ни cv2, ни зависимости от camera_segmentation —
маска приходит готовым bool-массивом. Импортируется и тестируется без железа.
"""

from typing import Optional

import numpy as np

from .projection import camera_to_world


def _fit_plane(points: np.ndarray):
    """PCA-плоскость: centroid + нормаль (собств. вектор наим. с.з.). Та же
    математика, что в camera_segmentation.floor_calibration._fit_plane."""
    centroid = points.mean(axis=0)
    q = points - centroid
    _, eigvecs = np.linalg.eigh(q.T @ q)
    return eigvecs[:, 0], centroid


def fit_floor_plane_camera(
    depth_map: np.ndarray,
    floor_mask: np.ndarray,
    intrinsics,
    pixel_stride: int = 8,
    min_depth_m: float = 0.2,
    max_depth_m: float = 4.0,
    inlier_thresh_m: float = 0.04,
    min_points: int = 200,
):
    """
    Подогнать плоскость пола в системе КАМЕРЫ по floor-пикселям.

    Returns:
        (normal, centroid) — нормаль ориентирована «к камере» (вверх), так что
        высота точки над полом h = (P - centroid) @ normal > 0 для приподнятого.
        None — если floor-пикселей мало для устойчивой подгонки.
    """
    if depth_map is None or floor_mask is None:
        return None
    h, w = depth_map.shape[:2]
    vs = np.arange(0, h, pixel_stride)
    us = np.arange(0, w, pixel_stride)
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel(); vv = vv.ravel()

    z = depth_map[vv, uu].astype(np.float32)
    sel = floor_mask[vv, uu] & (z >= min_depth_m) & (z <= max_depth_m)
    if int(sel.sum()) < min_points:
        return None
    uu = uu[sel]; vv = vv[sel]; z = z[sel]

    x = (uu - intrinsics.cx) * z / intrinsics.fx
    y = (vv - intrinsics.cy) * z / intrinsics.fy
    points = np.column_stack([x, y, z]).astype(np.float32)

    # Подгонка + один робастный отсев (NN уже выбрал пол — хватает trimming).
    normal, centroid = _fit_plane(points)
    resid = np.abs((points - centroid) @ normal)
    inliers = resid < inlier_thresh_m
    if int(inliers.sum()) >= max(min_points // 2, 50):
        normal, centroid = _fit_plane(points[inliers])

    # Нормаль — «к камере» (вверх): centroid пола ниже центра (y>0), для нормали
    # вверх n·centroid < 0; если знак другой — разворачиваем.
    if float(normal @ centroid) > 0:
        normal = -normal
    return normal, centroid


def classify_pixels(
    depth_map: np.ndarray,
    floor_mask: np.ndarray,
    intrinsics,
    plane,
    pixel_stride: int = 8,
    min_depth_m: float = 0.15,
    max_depth_m: float = 6.0,
    min_obstacle_height_m: float = 0.03,
    max_obstacle_height_m: float = 0.60,
    strong_obstacle_height_m: float = 0.30,
):
    """
    Классифицировать подвыборку пикселей кадра по контракту слияния.

    Args:
        plane: (normal, centroid) из fit_floor_plane_camera.

    Returns:
        dict с полями (все — выровненные 1-D массивы по валидным пикселям):
          'uu','vv'        — пиксельные координаты (для визуализации/проекции)
          'x_cam','y_cam','z_cam' — координаты в системе камеры
          'height'         — высота над плоскостью пола, м
          'obstacle'       — bool-маска «препятствие» по контракту
          'free'           — bool-маска «подтверждённый пол» (floor И низко)
        Пустые массивы, если валидных пикселей нет.
    """
    normal, centroid = plane
    h, w = depth_map.shape[:2]
    vs = np.arange(0, h, pixel_stride)
    us = np.arange(0, w, pixel_stride)
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel(); vv = vv.ravel()

    z = depth_map[vv, uu].astype(np.float32)
    valid = (z >= min_depth_m) & (z <= max_depth_m)
    uu = uu[valid]; vv = vv[valid]; z = z[valid]
    is_floor = floor_mask[vv, uu]

    x = (uu - intrinsics.cx) * z / intrinsics.fx
    y = (vv - intrinsics.cy) * z / intrinsics.fy
    pts = np.column_stack([x, y, z]).astype(np.float32)
    height = (pts - centroid) @ normal

    in_band = (height >= min_obstacle_height_m) & (height <= max_obstacle_height_m)
    strong = (height >= strong_obstacle_height_m) & (height <= max_obstacle_height_m)
    obstacle = ((~is_floor) & in_band) | strong
    free = is_floor & (height < min_obstacle_height_m)

    return {
        'uu': uu, 'vv': vv,
        'x_cam': x, 'y_cam': y, 'z_cam': z,
        'height': height,
        'is_floor': is_floor,
        'obstacle': obstacle,
        'free': free,
    }


def fuse_camera_obstacles(
    depth_map: np.ndarray,
    floor_mask: np.ndarray,
    intrinsics,
    mount,
    robot_pose,
    pixel_stride: int = 8,
    min_depth_m: float = 0.15,
    max_range_m: float = 6.0,
    min_obstacle_height_m: float = 0.03,
    max_obstacle_height_m: float = 0.60,
    strong_obstacle_height_m: float = 0.30,
    plane_min_points: int = 200,
    return_debug: bool = False,
) -> Optional[np.ndarray]:
    """
    Облако препятствий (мировые координаты) из глубины + маски пола по контракту.

    Returns:
        N×3 float32 (x_world, y_world, z_world) точек-препятствий.
        None — если плоскость пола не подогналась (мало floor-пикселей) →
        вызывающий ОБЯЗАН откатиться на legacy height-filter.
        При return_debug=True — кортеж (points, info), где info — словарь из
        classify_pixels (для отладочной визуализации).
    """
    if depth_map is None or floor_mask is None or depth_map.size == 0:
        return None
    if floor_mask.shape[:2] != depth_map.shape[:2]:
        return None

    plane = fit_floor_plane_camera(
        depth_map, floor_mask, intrinsics,
        pixel_stride=pixel_stride, max_depth_m=min(4.0, max_range_m),
        min_points=plane_min_points)
    if plane is None:
        return None

    info = classify_pixels(
        depth_map, floor_mask, intrinsics, plane,
        pixel_stride=pixel_stride,
        min_depth_m=min_depth_m, max_depth_m=max_range_m,
        min_obstacle_height_m=min_obstacle_height_m,
        max_obstacle_height_m=max_obstacle_height_m,
        strong_obstacle_height_m=strong_obstacle_height_m)

    obs = info['obstacle']
    if obs.any():
        x_w, y_w, z_w = camera_to_world(
            info['x_cam'][obs], info['y_cam'][obs], info['z_cam'][obs],
            mount, robot_pose)
        points = np.column_stack([x_w, y_w, z_w]).astype(np.float32)
    else:
        points = np.zeros((0, 3), dtype=np.float32)

    if return_debug:
        return points, info
    return points
