# Camera Segmentation Package

Пакет для семантической сегментации изображений с камеры робота.

## Структура

```
camera_segmentation/
├── __init__.py              # Инициализация
├── config.py                # Конфигурация
├── segmentation.py          # Класс CameraSegmentation
├── simple_segmentation.py   # Простая сегментация (fallback)
└── utils.py                 # Утилиты обработки
```

## Использование

```python
from camera_segmentation import CameraSegmentation

seg = CameraSegmentation()
mask = seg.get_segmentation_mask()  # Получить маску сегментации
seg.release()
```

## Классы сегментации

- **0** - Пол (свободно)
- **1** - Препятствие
- **2** - Стена
- **3** - Сетка

## Интеграция с навигацией

```python
from camera_segmentation import CameraSegmentation

# В navigation_controller.py
self.camera_seg = CameraSegmentation()
mask = self.camera_seg.get_segmentation_mask()
self.slam.update_with_camera_segmentation(mask)
```


