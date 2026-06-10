#!/usr/bin/env python3
"""
Отладочная визуализация depth-восприятия.

Собирает один JPEG «что видит камера и что из этого сделала нейронка»:
  слева  — исходный кадр RGB;
  справа — depth-карта цветом (близко = тепло), поверх красным подсвечены
           пиксели, прошедшие фильтр высот 3–50 см — то, что пишется в карту
           и питает реактивный слой;
  снизу  — строка с секторными дистанциями (front/corridor/left/right).

Используется NavigationController'ом (CAMERA_DEBUG_SAVE_ENABLED): раз в
N секунд кадр сохраняется в camera_debug/. По этим кадрам видно, ПОЧЕМУ
робот остановился или, наоборот, почему не заметил препятствие.
"""

from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:  # на железе cv2 обязателен, ветка для дев-машин без него
    cv2 = None  # type: ignore
    _HAS_CV2 = False

from .projection import (CameraIntrinsics, CameraMount,
                         backproject_depth_to_world, filter_obstacles_by_height)


def render_depth_debug(
    frame_bgr: np.ndarray,
    depth_map: np.ndarray,
    intrinsics: CameraIntrinsics,
    mount: CameraMount,
    robot_pose: tuple,
    sector_distances: Optional[dict] = None,
    pixel_stride: int = 4,
    min_obstacle_height_m: float = 0.03,
    max_obstacle_height_m: float = 0.50,
    min_depth_m: float = 0.15,
    max_depth_m: float = 6.0,
    max_vis_depth_m: float = 6.0,
) -> Optional[np.ndarray]:
    """
    Собрать отладочный кадр: RGB | depth-colormap + маска препятствий.

    Проекция и фильтр высот здесь намеренно те же, что в боевом пайплайне
    (backproject_depth_to_world + filter_obstacles_by_height) — картинка
    показывает ровно то, что видит SLAM/реактивный слой, а не похожую копию.
    stride по умолчанию плотнее боевого (4 против 8) — для наглядности маски.

    Returns:
        BGR-изображение (H × 2W) или None, если cv2 недоступен.
    """
    if not _HAS_CV2:
        return None

    h, w = depth_map.shape[:2]

    # --- Правая половина: depth цветом. Близкое = тёплое (инвертируем). ---
    d = np.clip(depth_map, 0.0, max_vis_depth_m) / max_vis_depth_m
    depth_u8 = ((1.0 - d) * 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

    # --- Маска препятствий: тот же пайплайн, что и в бою ---
    points, pixels = backproject_depth_to_world(
        depth_map, intrinsics, mount,
        robot_pose=robot_pose,
        pixel_stride=pixel_stride,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        use_lower_half_only=True,
        return_pixels=True,
    )
    _, obstacle_mask = filter_obstacles_by_height(
        points,
        min_height_m=min_obstacle_height_m,
        max_height_m=max_obstacle_height_m,
        return_mask=True,
    )
    obstacle_px = pixels[obstacle_mask] if len(pixels) else pixels

    # Красные квадратики на месте пикселей-препятствий (размер = stride,
    # чтобы маска визуально была сплошной)
    half = max(1, pixel_stride // 2)
    for u, v in obstacle_px:
        cv2.rectangle(depth_color,
                      (int(u) - half, int(v) - half),
                      (int(u) + half, int(v) + half),
                      (0, 0, 255), thickness=-1)

    # Линия горизонта (с учётом tilt) и граница «нижней половины кадра»
    horizon = int(round(intrinsics.cy - intrinsics.fy * np.tan(mount.tilt_rad)))
    cv2.line(depth_color, (0, horizon), (w - 1, horizon), (255, 255, 255), 1)
    cv2.line(depth_color, (0, h // 2), (w - 1, h // 2), (200, 200, 200), 1)
    # Рамка полосы страховочного канала nearest_in_depth_band (±5% кадра,
    # центральные 55% ширины) — то, где меряется «ближайший объект по курсу»
    half_band = max(2, int(round(h * 0.05)))
    half_w = max(2, int(round(w * 0.55 / 2)))
    cv2.rectangle(depth_color,
                  (w // 2 - half_w, max(0, horizon - half_band)),
                  (w // 2 + half_w, min(h - 1, horizon + half_band)),
                  (255, 255, 0), 1)

    # --- Левая половина: исходный кадр (приводим к размеру depth) ---
    frame = frame_bgr
    if frame.shape[:2] != (h, w):
        frame = cv2.resize(frame, (w, h))

    combined = np.hstack([frame, depth_color])

    # --- Подпись: секторные дистанции и счётчик точек ---
    def _fmt(val) -> str:
        return f"{val:.2f}m" if val is not None else "--"

    sd = sector_distances or {}
    label = (f"front {_fmt(sd.get('front'))}  corridor {_fmt(sd.get('front_corridor'))}  "
             f"band {_fmt(sd.get('front_band'))}  "
             f"left {_fmt(sd.get('left'))}  right {_fmt(sd.get('right'))}  "
             f"obst px {len(obstacle_px)}")
    cv2.rectangle(combined, (0, 0), (combined.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(combined, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)

    return combined
