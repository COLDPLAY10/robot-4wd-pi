#!/usr/bin/env python3
"""
Отладочная визуализация сегментации пола.

Собирает один кадр «что увидела сеть»: слева исходный RGB, справа он же с
зелёной заливкой пиксели-пола и подписью оценки калибровки (наклон/высота
камеры). По этим кадрам после тестового заезда видно, корректно ли сеть
отделяет пол и что она путает (дыры в маске, ковёр, блики).

Используется офлайн-скриптом scripts/eval_floor_segmentation.py и (при
FLOOR_SEG_ENABLED) NavigationController'ом. Стиль — как camera_perception/
debug_viz.py.
"""

from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore
    _HAS_CV2 = False


def render_floor_debug(
    frame_bgr: np.ndarray,
    floor_mask: np.ndarray,
    calibration=None,
    alpha: float = 0.45,
) -> Optional[np.ndarray]:
    """
    Собрать отладочный кадр: RGB | RGB + зелёная заливка пола.

    Args:
        frame_bgr: H×W×3 uint8 BGR
        floor_mask: H×W bool, True = пол (из FloorSegmenter)
        calibration: FloorCalibration или None — выводится в подпись
        alpha: прозрачность зелёной заливки

    Returns:
        BGR-изображение (H × 2W) или None, если cv2 недоступен.
    """
    if not _HAS_CV2:
        return None

    h, w = floor_mask.shape[:2]
    frame = frame_bgr
    if frame.shape[:2] != (h, w):
        frame = cv2.resize(frame, (w, h))

    # Зелёная заливка только по пикселям пола.
    vis = frame.copy()
    if floor_mask.any():
        green = np.zeros_like(frame)
        green[:, :] = (0, 255, 0)
        blended = cv2.addWeighted(frame, 1.0 - alpha, green, alpha, 0)
        vis[floor_mask] = blended[floor_mask]

    combined = np.hstack([frame, vis])

    floor_pct = 100.0 * float(floor_mask.sum()) / float(floor_mask.size)
    if calibration is not None:
        c = calibration
        flag = 'OK' if getattr(c, 'ok', False) else 'НЕНАДЁЖНО'
        label = (f"floor {floor_pct:.0f}%  |  tilt {c.tilt_deg:+.1f}°  "
                 f"h {c.height_m*100:.1f}см  rms {c.rms_residual_m*1000:.0f}мм  "
                 f"inlier {c.inlier_ratio:.2f}  [{flag}]")
    else:
        label = f"floor {floor_pct:.0f}%  |  калибровка: нет данных"

    cv2.rectangle(combined, (0, 0), (combined.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(combined, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return combined