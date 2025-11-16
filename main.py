import threading
import time
import signal
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from car_adapter import bot
from ultrasonic import read_distance_cm_from_bot, require_bot_or_exit
from RGB import set_green, set_red, off

#Настройки 
CAMERA_INDEX = 0
WINDOW_NAME = "Camera and Ultrasonic"
FONT_COLOR_BGR = (0, 255, 0)
ALERT_COLOR_BGR = (0, 0, 255)
TEXT_POS = (10, 30)
FPS_TARGET = 30
OBSTACLE_THRESHOLD_CM = 40.0
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_SIZE = 20

try:
    PIL_FONT = ImageFont.truetype(FONT_PATH, FONT_SIZE)
except Exception:
    PIL_FONT = ImageFont.load_default()

#Параметры движения
NEAR_DISTANCE_CM = 30.0
FAR_DISTANCE_CM = 50.0
SPEED_FORWARD = 20
SPEED_BACK = 12
TURN_TIME = 0.45
BACK_TIME = 0.30
SENSOR_POLL = 0.12
CONFIRM_COUNT = 2
BIAS_LEFT = 1.0
BIAS_RIGHT = 0.6

STATE_FORWARD = "FORWARD"
STATE_BACKING = "BACKING"
STATE_ROTATING = "ROTATING"
STATE_STOPPED = "STOPPED"

#Глобальные переменные
running = True
stop_requested = False
latest_distance_cm = None
distance_lock = threading.Lock()
obstacle_present = False

sensor_thread = None
drive_thread = None

#Обработчик сигналов
def _signal_handler(sig, frame):
    global running, stop_requested
    running = False
    stop_requested = True
    try:
        # немедленный стоп
        for _ in range(3):
            bot.Ctrl_Muto(0, 0); bot.Ctrl_Muto(1, 0)
            bot.Ctrl_Muto(2, 0); bot.Ctrl_Muto(3, 0)
            time.sleep(0.03)
    except Exception:
        pass
    try:
        off()
    except Exception:
        pass

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

# поток датчика
def sensor_loop(poll_interval=0.1):
    global running, latest_distance_cm
    try:
        require_bot_or_exit()
    except Exception:
        pass
    while running and not stop_requested:
        try:
            d = read_distance_cm_from_bot()
        except Exception:
            d = None
        with distance_lock:
            latest_distance_cm = d
        time.sleep(poll_interval)

#Управление моторами
def stop_all():
    try:
        bot.Ctrl_Muto(0, 0); bot.Ctrl_Muto(1, 0)
        bot.Ctrl_Muto(2, 0); bot.Ctrl_Muto(3, 0)
    except Exception:
        pass

def forward(speed):
    try:
        pwm_l = int(speed * BIAS_LEFT)
        pwm_r = int(speed * BIAS_RIGHT)
        bot.Ctrl_Muto(0, pwm_l); bot.Ctrl_Muto(1, pwm_l)
        bot.Ctrl_Muto(2, pwm_r); bot.Ctrl_Muto(3, pwm_r)
    except Exception:
        pass

def backward(speed):
    try:
        bot.Ctrl_Muto(0, -int(speed)); bot.Ctrl_Muto(1, -int(speed))
        bot.Ctrl_Muto(2, -int(speed)); bot.Ctrl_Muto(3, -int(speed))
    except Exception:
        pass

def rotate_left(speed):
    try:
        bot.Ctrl_Muto(0, -int(speed)); bot.Ctrl_Muto(1, -int(speed))
        bot.Ctrl_Muto(2, int(speed));  bot.Ctrl_Muto(3, int(speed))
    except Exception:
        pass

# --- Поток движения ---
def drive_loop():
    global running, latest_distance_cm, stop_requested
    # включаем ультразвук
    try:
        bot.Ctrl_Ulatist_Switch(1)
        time.sleep(0.12)
    except Exception:
        pass

    # стартовый стоп
    for _ in range(3):
        stop_all()
        time.sleep(0.05)

    try:
        set_green()
    except Exception:
        pass

    state = STATE_FORWARD
    action_until = 0.0
    near_count = 0
    med_count = 0

    while running and not stop_requested:
        now = time.time()
        with distance_lock:
            cm = latest_distance_cm

        if state == STATE_BACKING:
            if now >= action_until:
                state = STATE_ROTATING
                action_until = now + TURN_TIME
                stop_all(); time.sleep(0.06)
                rotate_left(SPEED_FORWARD)
                try:
                    set_red()
                except Exception:
                    pass
            else:
                time.sleep(SENSOR_POLL)
                continue

        elif state == STATE_ROTATING:
            if now >= action_until:
                state = STATE_FORWARD
                stop_all(); time.sleep(0.06)
                forward(SPEED_FORWARD)
                try:
                    set_green()
                except Exception:
                    pass
            else:
                time.sleep(SENSOR_POLL)
                continue

        elif state == STATE_FORWARD:
            forward(SPEED_FORWARD)

            if cm is None:
                near_count = 0; med_count = 0
                time.sleep(SENSOR_POLL)
                continue

            try:
                dis_cm = float(cm)
            except Exception:
                dis_cm = None

            if dis_cm is None:
                near_count = 0; med_count = 0
                time.sleep(SENSOR_POLL)
                continue

            if dis_cm < NEAR_DISTANCE_CM:
                if near_count == 0:
                    stop_all(); time.sleep(0.08)
                near_count += 1; med_count = 0
            else:
                near_count = 0

            if NEAR_DISTANCE_CM <= dis_cm <= FAR_DISTANCE_CM:
                if med_count == 0:
                    stop_all(); time.sleep(0.06)
                med_count += 1
            else:
                med_count = 0

            if near_count >= CONFIRM_COUNT:
                print(f"Подтверждён близкий объект {dis_cm:.1f} см → откат назад")
                stop_all(); time.sleep(0.06)
                backward(SPEED_BACK)
                action_until = now + BACK_TIME
                state = STATE_BACKING
                near_count = 0; med_count = 0
                try:
                    set_red()
                except Exception:
                    pass
                continue

            if med_count >= CONFIRM_COUNT:
                print(f"Подтверждён объект в средней зоне {dis_cm:.1f} см → поворот")
                stop_all(); time.sleep(0.06)
                state = STATE_ROTATING
                action_until = now + TURN_TIME
                rotate_left(SPEED_FORWARD)
                try:
                    set_red()
                except Exception:
                    pass
                med_count = 0; near_count = 0
                continue

        time.sleep(SENSOR_POLL)

    # выключаем ультразвук при выходе потока движения
    try:
        bot.Ctrl_Ulatist_Switch(0)
    except Exception:
        pass
    # финальный стоп
    try:
        for _ in range(3):
            stop_all()
            time.sleep(0.05)
    except Exception:
        pass
    # === ДОБАВЛЕНО: выключаем подсветку при выходе
    try:
        off()
    except Exception:
        pass

#Рисование текста
def draw_text(frame, text, pos, color_bgr, font=PIL_FONT):
    img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
    draw.text(pos, text, font=font, fill=color_rgb)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

#Основной цикл камеры
def main():
    global running, stop_requested, sensor_thread, drive_thread, latest_distance_cm, obstacle_present

    # Потоки датчика и движения
    sensor_thread = threading.Thread(target=sensor_loop, daemon=True)
    drive_thread = threading.Thread(target=drive_loop, daemon=True)
    sensor_thread.start()
    drive_thread.start()

    # Инициализация камеры
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Не удалось открыть камеру:", CAMERA_INDEX)
        running = False
        stop_requested = True
        # ожидание завершения потоков
        try:
            sensor_thread.join(timeout=1.0)
            drive_thread.join(timeout=1.0)
        except Exception:
            pass
        return

    # FPS и задержка
    try:
        cap.set(cv2.CAP_PROP_FPS, FPS_TARGET)
    except Exception:
        pass
    wait_ms = max(1, int(1000 / FPS_TARGET))

    try:
        while running and not stop_requested:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            with distance_lock:
                d_cm = latest_distance_cm

            if d_cm is None:
                dist_text = "Расстояние: N/A"
                color = FONT_COLOR_BGR
            else:
                dist_text = f"Расстояние: {d_cm:.1f} см"
                color = ALERT_COLOR_BGR if d_cm <= OBSTACLE_THRESHOLD_CM else FONT_COLOR_BGR

            frame = draw_text(frame, dist_text, TEXT_POS, color)
            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(wait_ms) & 0xFF
            if key in (ord('q'), 27):
                running = False
                stop_requested = True
                break

            # Завершение по крестику окна
            try:
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    running = False
                    stop_requested = True
                    break
            except Exception:
                pass

            # Сообщение об обнаружении препятствия один раз
            if d_cm is not None and d_cm <= OBSTACLE_THRESHOLD_CM:
                if not obstacle_present:
                    obstacle_present = True
                    print(f"Обнаружено препятствие (<= {OBSTACLE_THRESHOLD_CM:.1f} см)")
            else:
                obstacle_present = False

    finally:
        # Флаги остановки
        running = False
        stop_requested = True

        # Освобождение камеры и окна
        try:
            cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        # Жёсткий стоп моторов
        try:
            for _ in range(5):
                stop_all()
                time.sleep(0.05)
        except Exception:
            pass

        # Выключаем подсветку
        try:
            off()
        except Exception:
            pass

        # Ожидание завершения потоков
        try:
            if sensor_thread is not None:
                sensor_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if drive_thread is not None:
                drive_thread.join(timeout=1.0)
        except Exception:
            pass

if __name__ == "__main__":
    main()
