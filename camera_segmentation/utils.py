"""
Утилиты для обработки изображений
"""

import cv2
import numpy as np


def apply_color_map(mask: np.ndarray, color_map: dict) -> np.ndarray:
    """
    Применение цветовой карты к маске

    Args:
        mask: маска сегментации
        color_map: словарь {class_id: (B, G, R)}

    Returns:
        цветное изображение
    """
    h, w = mask.shape
    colored = np.zeros((h, w, 3), dtype=np.uint8)

    for class_id, color in color_map.items():
        colored[mask == class_id] = color

    return colored



