"""
Пакет для обработки изображений с камеры и семантической сегментации
Интеграция с нейросетевыми моделями для навигации робота
"""

from .segmentation import CameraSegmentation
from .config import *

__all__ = [
    'CameraSegmentation',
    'CAMERA_ID',
    'FRAME_W',
    'FRAME_H',
    'CLASS_FLOOR',
    'CLASS_OBSTACLE',
    'CLASS_WALL',
    'CLASS_MESH'
]

__version__ = '1.0.0'

