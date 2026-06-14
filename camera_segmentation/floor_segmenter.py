#!/usr/bin/env python3
"""
Нейросетевая сегментация пола через ONNX-модель SegFormer-B0 (ADE20K).

Архитектурный выбор — тот же, что у camera_perception/depth_estimator.py:
ONNX Runtime, а не PyTorch (на Pi не нужен torch ~700 МБ, только onnxruntime
~30 МБ; инференс на ARM CPU оптимизирован). Экспорт .pth -> .onnx делается
единожды на дев-машине скриптом scripts/setup_segformer_model.py.

Модель: SegFormer-B0, дообученный на ADE20K (150 классов). Выдаёт логиты в
разрешении H/4 × W/4 (для входа 512 — 128×128). Здесь делаем апсемплинг,
argmax и оставляем только класс(ы) "пол" (FLOOR_CLASS_IDS из config) — на
выходе бинарная floor-маска в разрешении исходного кадра.

ПРОИЗВОДИТЕЛЬНОСТЬ: B0 на Pi 5 CPU ~0.2-0.5 с при входе 512. Это ВТОРАЯ
нейросеть поверх Depth-Anything-V2 — на роботе их инференс надо чередовать,
а не гонять обе каждый кадр (иначе FPS depth уполовинивается). По умолчанию
весь модуль выключен (NavigationController.FLOOR_SEG_ENABLED=False); основной
сценарий использования — ОФЛАЙН-оценка на дев-машине по сохранённым кадрам
(scripts/eval_floor_segmentation.py).
"""

import os
import time
from typing import Optional, Sequence, Tuple

import numpy as np

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore
    _HAS_CV2 = False

from .config import SEGFORMER_MODEL_PATH, SEGFORMER_INPUT_SIZE, FLOOR_CLASS_IDS

# Нормализация ImageNet — та же, что у Depth-Anything (общая конвенция ViT).
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


class FloorSegmenter:
    """
    Инференс семантической сегментации пола через ONNX. Один экземпляр на робот;
    модель грузится лениво при первом вызове segment_floor.
    """

    def __init__(self,
                 model_path: Optional[str] = None,
                 input_size: int = SEGFORMER_INPUT_SIZE,
                 floor_class_ids: Sequence[int] = FLOOR_CLASS_IDS,
                 num_threads: int = 2):
        """
        Args:
            model_path: путь к ONNX (см. scripts/setup_segformer_model.py)
            input_size: размер входа (квадрат, кратный 4; для ade-512-512 — 512)
            floor_class_ids: индексы ADE20K, считаемые "полом" (3 = 'floor')
            num_threads: ONNX Runtime intra-op threads. На Pi 5 2-3 (конкурирует
                         со SLAM/DWA/depth — не задирать).
        """
        self.model_path = model_path or SEGFORMER_MODEL_PATH
        self.input_size = input_size
        self.floor_class_ids = tuple(floor_class_ids)
        self.num_threads = num_threads

        self._session = None  # ленивая инициализация
        self._input_name: Optional[str] = None
        self._output_name: Optional[str] = None

        # Метрики для отчёта
        self.last_inference_ms: float = 0.0
        self.total_inferences: int = 0

    def _load(self) -> bool:
        """Загружает ONNX-сессию. False, если файла нет или onnxruntime недоступен."""
        if self._session is not None:
            return True
        if not os.path.exists(self.model_path):
            print(f"[FloorSegmenter] Модель не найдена: {self.model_path}")
            print(f"[FloorSegmenter] Запустите: python3 scripts/setup_segformer_model.py")
            return False
        try:
            import onnxruntime as ort
        except ImportError:
            print("[FloorSegmenter] onnxruntime не установлен (pip install onnxruntime)")
            return False

        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = self.num_threads
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(
            self.model_path, sess_options=sess_opts,
            providers=['CPUExecutionProvider'],
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        print(f"[FloorSegmenter] Загружено: {self.model_path}, "
              f"floor_class_ids={self.floor_class_ids}, threads={self.num_threads}")
        return True

    def is_ready(self) -> bool:
        return self._session is not None or self._load()

    def _preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        """BGR (H,W,3) uint8 -> NCHW float32, нормализация ImageNet, вход input_size²."""
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_size, self.input_size),
                             interpolation=cv2.INTER_LINEAR)
        normalized = (resized.astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
        nchw = normalized.transpose(2, 0, 1)[np.newaxis, ...]
        return np.ascontiguousarray(nchw, dtype=np.float32)

    def segment_floor(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Получить бинарную маску пола для кадра.

        Args:
            frame_bgr: H×W×3 uint8 BGR (как из cv2.VideoCapture)

        Returns:
            H×W bool, True = пол. None, если модель не загружена / сбой.
            Разрешение совпадает с входным кадром.
        """
        if not _HAS_CV2 or not self.is_ready():
            return None
        try:
            orig_h, orig_w = frame_bgr.shape[:2]
            inp = self._preprocess(frame_bgr)

            t0 = time.time()
            outputs = self._session.run([self._output_name], {self._input_name: inp})
            self.last_inference_ms = (time.time() - t0) * 1000.0
            self.total_inferences += 1

            logits = outputs[0]                       # (1, C, h/4, w/4)
            class_map_small = np.argmax(logits[0], axis=0).astype(np.int32)  # (h/4, w/4)
            floor_small = np.isin(class_map_small, self.floor_class_ids)

            # Апсемплинг маски до размера кадра — ближайшим соседом (классы дискретны).
            floor_mask = cv2.resize(floor_small.astype(np.uint8), (orig_w, orig_h),
                                    interpolation=cv2.INTER_NEAREST).astype(bool)
            return floor_mask
        except Exception as e:
            print(f"[FloorSegmenter] Ошибка инференса: {e}")
            return None

    def segment_classes(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Полная карта классов ADE20K (не только пол) в разрешении кадра — для
        отладки/исследования: какие классы модель путает с полом, где «дыры»
        в маске. int32 H×W. В боевом контуре не нужна, используется офлайн.
        """
        if not _HAS_CV2 or not self.is_ready():
            return None
        try:
            orig_h, orig_w = frame_bgr.shape[:2]
            inp = self._preprocess(frame_bgr)
            outputs = self._session.run([self._output_name], {self._input_name: inp})
            class_map_small = np.argmax(outputs[0][0], axis=0).astype(np.int32)
            return cv2.resize(class_map_small, (orig_w, orig_h),
                              interpolation=cv2.INTER_NEAREST).astype(np.int32)
        except Exception as e:
            print(f"[FloorSegmenter] Ошибка инференса: {e}")
            return None

    def get_stats(self) -> dict:
        """Сводка производительности — для дипломного отчёта."""
        return {
            'inferences': self.total_inferences,
            'last_ms': self.last_inference_ms,
            'last_fps': (1000.0 / self.last_inference_ms) if self.last_inference_ms > 0 else 0.0,
            'model': self.model_path,
        }