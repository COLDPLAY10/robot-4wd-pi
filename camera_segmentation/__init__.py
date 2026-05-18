"""
DEPRECATED: семантическая сегментация "пол / стены / препятствия".

Заменён на camera_perception/ (Depth-Anything-V2 + обратная проекция). Здесь
остался только цветовой baseline и data_collector — для исследовательских
скриптов и сравнения в дипломном отчёте. В NavigationController не используется.
"""

from .segmentation import CameraSegmentation
from .config import *

__all__ = [
    'CameraSegmentation',
    'CAMERA_ID',
    'FRAME_W',
    'FRAME_H',
    'CLASS_UNKNOWN',
    'CLASS_FLOOR',
    'CLASS_OBSTACLE',
    'CLASS_WALL',
    'CLASS_MESH'
]

__version__ = '1.0.0'

