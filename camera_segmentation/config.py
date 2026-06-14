"""
Конфигурация нейросетевой сегментации пола (SegFormer-B0, ADE20K).
"""

import os

# Путь к ONNX-модели (экспорт: scripts/setup_segformer_model.py).
# Абсолютный относительно корня репо — иначе ломается при запуске из map_scripts/.
SEGFORMER_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'segformer_b0_ade.onnx',
)

# Размер входа модели (квадрат, как при обучении ade-512-512).
SEGFORMER_INPUT_SIZE = 512

# Индексы класса "пол" в ADE20K — ПРОВЕРЕНЫ через model.config.id2label
# (scripts/setup_segformer_model.py): 3 = 'floor'. Ковёр (28 = 'rug') по умолчанию
# НЕ считаем полом — добавьте 28, если робот ездит по ковровому покрытию.
FLOOR_CLASS_IDS = (3,)