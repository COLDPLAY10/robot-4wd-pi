import cv2
import numpy as np

def detect_obstacle_by_color():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Ошибка: камера не найдена.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Преобразуем изображение в HSV (удобнее для выделения цветов)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Диапазон белого цвета (можно подстроить под освещение)
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])

        # Маска — выделяем белые области
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Считаем, сколько белых пикселей
        white_area = cv2.countNonZero(mask)
        total_area = frame.shape[0] * frame.shape[1]
        white_ratio = white_area / total_area

        # Порог: если белого больше 5% кадра — считаем препятствием
        if white_ratio > 0.05:
            print("1 (вижу белое препятствие)")
        else:
            print("0 (путь чист)")

        # Показываем картинку (для отладки)
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask (white detection)", mask)

        # Нажми q чтобы выйти
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_obstacle_by_color()
