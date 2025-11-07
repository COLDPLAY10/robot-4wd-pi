import cv2
import numpy as np

def detect_obstacle_by_color(white_ratio_thr):
    """Определяет наличие белого препятствия по видеопотоку
    Возвращает:
      True — если белый объект занимает больше white_ratio_thr площади кадра
      False — если нет
      None — если камера не доступна"""
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Ошибка: камера недоступна")
        return None

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Ошибка: не удалось получить кадр")
        return None

    # Переводим в HSV (удобнее выделять цвета)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Диапазон для белого
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 25, 255])

    #Маска белых пикселей 
    mask = cv2.inRange(hsv, lower_white, upper_white)
    white_ratio = np.sum(mask > 0) / mask.size

    print(f"Доля белого цвета: {white_ratio:.3f}")

    return white_ratio > white_ratio_thr

