"""
Нейросетевая сегментация пола (SegFormer-B0, ADE20K) + автокалибровка камеры.

Назначение модуля:
  - FloorSegmenter      — ONNX-инференс SegFormer, бинарная маска пола;
  - estimate_floor_plane — наклон/высота камеры подгонкой плоскости по
                           floor-пикселям (автокалибровка, закрывает вопрос §6);
  - render_floor_debug   — overlay «кадр | маска пола + оценка калибровки» для
                           офлайн-разбора эффективности после тестового заезда.

По умолчанию в боевом контуре ВЫКЛЮЧЕНО (NavigationController.FLOOR_SEG_ENABLED
= False). Основной сценарий — офлайн-оценка на дев-машине по сохранённым кадрам
(scripts/eval_floor_segmentation.py). Экспорт модели: scripts/setup_segformer_model.py.

Заменяет старый цветовой baseline (CameraSegmentation, simple_color_segmentation),
который никогда не использовался в навигации и удалён.
"""

from .floor_segmenter import FloorSegmenter
from .floor_calibration import estimate_floor_plane, FloorCalibration
from .floor_debug import render_floor_debug
from .config import SEGFORMER_MODEL_PATH, SEGFORMER_INPUT_SIZE, FLOOR_CLASS_IDS

__all__ = [
    'FloorSegmenter',
    'estimate_floor_plane',
    'FloorCalibration',
    'render_floor_debug',
    'SEGFORMER_MODEL_PATH',
    'SEGFORMER_INPUT_SIZE',
    'FLOOR_CLASS_IDS',
]

__version__ = '2.0.0'