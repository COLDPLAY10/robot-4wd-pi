#!/usr/bin/env python3
"""
Монокулярная оценка глубины через ONNX-модель Depth-Anything-V2.

Архитектурный выбор: ONNX Runtime, а не PyTorch:
  - на Pi не нужен torch (~700 МБ) — только onnxruntime (~30 МБ)
  - int8/fp16 квантизация работает из коробки
  - инференс ARM CPU оптимизирован
Конвертация .pth → .onnx делается единожды на дев-машине скриптом
scripts/setup_depth_model.py — на Pi кладём готовый ONNX.

Рекомендуемая модель: Depth-Anything-V2-Metric-Indoor-Small.
  - 25M параметров, ~96 МБ FP32 / ~25 МБ INT8
  - метрическая глубина в метрах для помещений
  - 2–5 FPS на Pi 5 CPU без акселератора
"""

import os
import time
from typing import Optional, Tuple

import cv2
import numpy as np


# Стандартные параметры Depth-Anything-V2: квадратный вход кратный 14
# (под patch ViT), нормализация по ImageNet.
DEFAULT_INPUT_SIZE = 518
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


class DepthEstimator:
    """
    Инференс монокулярной глубины через ONNX. Один экземпляр на весь робот:
    модель загружается лениво при первом вызове get_depth_map.
    """

    def __init__(self,
                 model_path: Optional[str] = None,
                 input_size: int = DEFAULT_INPUT_SIZE,
                 num_threads: int = 2,
                 max_depth_m: float = 10.0):
        """
        Args:
            model_path: путь к ONNX-файлу (см. scripts/setup_depth_model.py)
            input_size: размер входа модели (кратный 14)
            num_threads: ONNX Runtime intra-op threads. На Pi 5 (4 ядра) 2-3 хорошо;
                         больше — конкурирует со SLAM и DWA.
            max_depth_m: всё что дальше — игнорируем (модель экстраполирует плохо)
        """
        # Дефолт — абсолютный путь относительно репо (camera_perception/../models/).
        # Иначе CWD-зависимый относительный путь ломается при запуске из map_scripts/.
        if model_path is None:
            model_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'models', 'depth_anything_v2_small.onnx',
            )
        self.model_path = model_path
        self.input_size = input_size
        self.num_threads = num_threads
        self.max_depth_m = max_depth_m

        self._session = None  # ленивая инициализация
        self._input_name: Optional[str] = None
        self._output_name: Optional[str] = None

        # Метрики для отчёта
        self.last_inference_ms: float = 0.0
        self.total_inferences: int = 0

    def _load(self) -> bool:
        """Загружает ONNX-сессию. Возвращает False если файла нет или onnxruntime недоступен."""
        if self._session is not None:
            return True
        if not os.path.exists(self.model_path):
            print(f"[DepthEstimator] Модель не найдена: {self.model_path}")
            print(f"[DepthEstimator] Запустите: python3 scripts/setup_depth_model.py")
            return False
        try:
            import onnxruntime as ort
        except ImportError:
            print("[DepthEstimator] onnxruntime не установлен (pip install onnxruntime)")
            return False

        # На Pi приоритет CPU; CUDA/CoreML добавятся автоматически если есть.
        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = self.num_threads
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        providers = ['CPUExecutionProvider']
        self._session = ort.InferenceSession(
            self.model_path, sess_options=sess_opts, providers=providers
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        in_shape = self._session.get_inputs()[0].shape
        out_shape = self._session.get_outputs()[0].shape
        print(f"[DepthEstimator] Загружено: {self.model_path}")
        print(f"[DepthEstimator] In: {self._input_name} {in_shape}, "
              f"Out: {self._output_name} {out_shape}, threads={self.num_threads}")
        return True

    def is_ready(self) -> bool:
        """Можем ли инференсить."""
        return self._session is not None or self._load()

    def _preprocess(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """BGR (H,W,3) uint8 → NCHW float32 normalized, плюс исходный размер для resize обратно."""
        orig_h, orig_w = frame_bgr.shape[:2]

        # BGR → RGB → float [0,1] → resize → normalize → NCHW
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_size, self.input_size),
                             interpolation=cv2.INTER_CUBIC)
        normalized = (resized.astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
        nchw = normalized.transpose(2, 0, 1)[np.newaxis, ...]  # (1, 3, H, W)
        return np.ascontiguousarray(nchw, dtype=np.float32), (orig_h, orig_w)

    def get_depth_map(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Получить depth map для кадра.

        Args:
            frame_bgr: H×W×3 uint8 BGR (как из cv2.VideoCapture)

        Returns:
            H×W float32, метрическая глубина в метрах (для metric-indoor варианта).
            Для relative-варианта — обратная нормированная глубина (надо
            интерпретировать как порядок ранжирования, не как метры).
            None если модель не загружена или произошёл сбой.
        """
        if not self.is_ready():
            return None

        try:
            inp, (orig_h, orig_w) = self._preprocess(frame_bgr)

            t0 = time.time()
            outputs = self._session.run([self._output_name], {self._input_name: inp})
            self.last_inference_ms = (time.time() - t0) * 1000.0
            self.total_inferences += 1

            depth = outputs[0]
            # Squeeze batch/channel: (1, H, W) или (1, 1, H, W) → (H, W)
            depth = np.squeeze(depth)

            # Resize обратно к исходному размеру кадра
            if depth.shape != (orig_h, orig_w):
                depth = cv2.resize(depth, (orig_w, orig_h),
                                   interpolation=cv2.INTER_CUBIC)

            # Обрезаем нефизичные значения
            depth = np.clip(depth, 0.0, self.max_depth_m).astype(np.float32)
            return depth

        except Exception as e:
            print(f"[DepthEstimator] Ошибка инференса: {e}")
            return None

    def get_stats(self) -> dict:
        """Сводка производительности — пригодится в дипломном отчёте."""
        return {
            'inferences': self.total_inferences,
            'last_ms': self.last_inference_ms,
            'last_fps': (1000.0 / self.last_inference_ms) if self.last_inference_ms > 0 else 0.0,
            'model': self.model_path,
        }
