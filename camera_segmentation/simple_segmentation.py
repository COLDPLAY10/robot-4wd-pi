"""
Простая сегментация на основе цвета
"""

import cv2
import numpy as np

from .config import CLASS_FLOOR, CLASS_OBSTACLE, CLASS_WALL


def simple_color_segmentation(frame: np.ndarray) -> np.ndarray:
    """
    Простая сегментация на основе цвета и яркости

    Args:
        frame: входной кадр BGR

    Returns:
        маска сегментации (H x W)
    """
    h, w = frame.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)

    # Конвертируем в grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Определяем пол (светлые области внизу кадра)
    floor_region = frame[h*2//3:h, :]
    floor_brightness = cv2.cvtColor(floor_region, cv2.COLOR_BGR2GRAY)
    floor_mask = floor_brightness > 100
    mask[h*2//3:h, :][floor_mask] = CLASS_FLOOR

    # Определяем препятствия (темные области)
    obstacle_mask = gray < 50
    mask[obstacle_mask] = CLASS_OBSTACLE

    # Определяем стены (границы)
    edges = cv2.Canny(gray, 50, 150)
    wall_mask = edges > 0
    mask[wall_mask] = CLASS_WALL

    return mask



