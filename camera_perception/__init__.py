"""
Восприятие через камеру: монокулярная оценка глубины (Depth-Anything-V2).

Основной канал восприятия робота. Модуль camera_segmentation/ переписан под
нейросетевую сегментацию пола (SegFormer) и дополняет это depth-восприятие.
Работаем напрямую с картой глубины — это надёжно для робота, потому что:
  - модель pretrained на 62М изображений, не требует своего датасета и обучения
  - глубина "видит" то что 2D-лидар пропускает (низкие препятствия, стекло, лестницы вниз)
  - выход напрямую интерпретируется как occupancy grid через обратную проекцию
"""

from .depth_estimator import DepthEstimator
from .projection import (
    CameraIntrinsics,
    CameraMount,
    backproject_depth_to_world,
    camera_to_world,
    filter_obstacles_by_height,
    nearest_in_depth_band,
    obstacle_distances_by_sector,
)
from .floor_fusion import (
    fuse_camera_obstacles,
    fit_floor_plane_camera,
    classify_pixels,
)
from .debug_viz import render_depth_debug

__all__ = [
    'DepthEstimator',
    'CameraIntrinsics',
    'CameraMount',
    'backproject_depth_to_world',
    'camera_to_world',
    'filter_obstacles_by_height',
    'nearest_in_depth_band',
    'obstacle_distances_by_sector',
    'fuse_camera_obstacles',
    'fit_floor_plane_camera',
    'classify_pixels',
    'render_depth_debug',
]
