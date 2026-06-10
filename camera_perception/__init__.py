"""
Восприятие через камеру: монокулярная оценка глубины (Depth-Anything-V2).

Заменяет старый модуль camera_segmentation/. Вместо "пол / стены / препятствия"
работаем напрямую с картой глубины — это надёжнее для робота, потому что:
  - модель pretrained на 62М изображений, не требует своего датасета и обучения
  - глубина "видит" то что 2D-лидар пропускает (низкие препятствия, стекло, лестницы вниз)
  - выход напрямую интерпретируется как occupancy grid через обратную проекцию
"""

from .depth_estimator import DepthEstimator
from .projection import (
    CameraIntrinsics,
    CameraMount,
    backproject_depth_to_world,
    filter_obstacles_by_height,
    nearest_in_depth_band,
    obstacle_distances_by_sector,
)
from .debug_viz import render_depth_debug

__all__ = [
    'DepthEstimator',
    'CameraIntrinsics',
    'CameraMount',
    'backproject_depth_to_world',
    'filter_obstacles_by_height',
    'nearest_in_depth_band',
    'obstacle_distances_by_sector',
    'render_depth_debug',
]
