#!/usr/bin/env python3
"""
Класс для работы с камерой и семантической сегментацией
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple

from .config import *


class CameraSegmentation:
    """
    Класс для получения семантической сегментации с камеры
    """

    def __init__(self, camera_id=CAMERA_ID, width=FRAME_W, height=FRAME_H, use_model=False, model=None):
        """
        Args:
            camera_id: ID камеры
            width: ширина кадра
            height: высота кадра
            use_model: использовать модель для сегментации
            model: объект модели (если None, будет использована заглушка)
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.use_model = use_model
        self.model = model
        self.cap = None

        # Инициализация камеры
        self._init_camera()

    def _init_camera(self):
        """Инициализация камеры"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                print("[CameraSeg] Не удалось открыть камеру")
                self.cap = None
                return

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            # Прогрев камеры
            for _ in range(5):
                self.cap.read()
                time.sleep(0.1)

            print(f"[CameraSeg] Камера инициализирована: {self.width}x{self.height}")

        except Exception as e:
            print(f"[CameraSeg] Ошибка инициализации камеры: {e}")
            self.cap = None

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Получение кадра с камеры

        Returns:
            Кадр BGR или None
        """
        if self.cap is None or not self.cap.isOpened():
            return None

        ret, frame = self.cap.read()

        if not ret or frame is None:
            return None

        return frame

    def get_segmentation_mask(self) -> Optional[np.ndarray]:
        """
        Получение маски сегментации

        Returns:
            Маска (H x W) с классами или None
        """
        frame = self.get_frame()

        if frame is None:
            return None

        # Выбор режима сегментации
        if self.use_model and self.model is not None:
            # Используем модель для сегментации
            mask = self._predict_with_model(frame)
        else:
            # Используем простую сегментацию
            from .simple_segmentation import simple_color_segmentation
            mask = simple_color_segmentation(frame)

        return mask

    def _predict_with_model(self, frame: np.ndarray) -> np.ndarray:
        """
        Предсказание с помощью модели (заглушка)

        Args:
            frame: входной кадр BGR

        Returns:
            маска сегментации
        """
        # TODO: Реализовать предсказание с реальной моделью

        print("[CameraSeg] Используется заглушка модели")
        from .simple_segmentation import simple_color_segmentation
        return simple_color_segmentation(frame)

    def get_obstacle_distance_estimate(self, mask: np.ndarray,
                                       fov_deg: float = CAMERA_FOV_DEG) -> Tuple[float, float, float]:
        """
        Оценка расстояния до препятствий в трех направлениях

        Args:
            mask: маска сегментации
            fov_deg: угол обзора камеры

        Returns:
            (left_dist, center_dist, right_dist) в метрах
        """
        h, w = mask.shape

        # Разделяем на 3 зоны
        left_zone = mask[h//2:h, 0:w//3]
        center_zone = mask[h//2:h, w//3:2*w//3]
        right_zone = mask[h//2:h, 2*w//3:w]

        left_dist = self._estimate_distance_from_mask(left_zone)
        center_dist = self._estimate_distance_from_mask(center_zone)
        right_dist = self._estimate_distance_from_mask(right_zone)

        return left_dist, center_dist, right_dist

    def _estimate_distance_from_mask(self, mask_region: np.ndarray) -> float:
        """
        Оценка расстояния по маске региона

        Args:
            mask_region: регион маски

        Returns:
            расстояние в метрах
        """
        h = mask_region.shape[0]

        # Ищем первое препятствие снизу вверх
        for row in range(h - 1, -1, -1):
            if np.any(mask_region[row] > CLASS_FLOOR):
                # Чем ниже в кадре, тем ближе
                distance = MAX_DISTANCE_ESTIMATE * (1.0 - (h - row) / h)
                return max(MIN_DISTANCE_ESTIMATE, distance)

        return MAX_DISTANCE_ESTIMATE  # Нет препятствий

    def visualize_segmentation(self, frame: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        Визуализация сегментации

        Args:
            frame: исходный кадр
            mask: маска сегментации

        Returns:
            кадр с наложенной маской
        """
        # Создаем цветную маску
        color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)

        for class_id, color in CLASS_COLORS.items():
            color_mask[mask == class_id] = color

        # Накладываем на исходный кадр
        result = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)

        return result

    def release(self):
        """Освобождение ресурсов"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None



