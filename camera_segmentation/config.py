"""
Конфигурация камеры и параметры сегментации
"""

# Конфигурация камеры
CAMERA_ID = 0
FRAME_W = 640
FRAME_H = 480

# Классы сегментации
CLASS_FLOOR = 0      # Пол (свободное пространство)
CLASS_OBSTACLE = 1   # Препятствие
CLASS_WALL = 2       # Стена
CLASS_MESH = 3       # Сетчатая конструкция

# Цвета для визуализации классов
CLASS_COLORS = {
    CLASS_FLOOR: (0, 255, 0),      # Зеленый
    CLASS_OBSTACLE: (0, 0, 255),   # Красный
    CLASS_WALL: (255, 0, 0),       # Синий
    CLASS_MESH: (255, 255, 0)      # Голубой
}

# Параметры оценки расстояния
MAX_DISTANCE_ESTIMATE = 3.0  # метров
MIN_DISTANCE_ESTIMATE = 0.2  # метров
CAMERA_FOV_DEG = 60.0        # угол обзора камеры


