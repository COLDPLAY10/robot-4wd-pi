import threading
import time
import signal

from car_adapter import bot
from ultrasonic import read_distance_cm_from_bot, require_bot_or_exit

# Пороги в сантиметрах
NEAR_DISTANCE_CM = 20.0
FAR_DISTANCE_CM = 42.5

# Скорости
SPEED_FORWARD = 20
SPEED_BACK = 12

# Тайминги
TURN_TIME = 0.45
BACK_TIME = 0.30
SENSOR_POLL = 0.12

# Подтверждение последовательными показаниями
CONFIRM_COUNT = 2

# Коэффициенты для компенсации разницы сторон
BIAS_LEFT = 1.0
BIAS_RIGHT = 0.55

# Состояния
STATE_FORWARD = "FORWARD"
STATE_BACKING = "BACKING"
STATE_ROTATING = "ROTATING"
STATE_STOPPED = "STOPPED"

# Глобальные переменные для потока датчика
running = True
latest_distance_cm = None
distance_lock = threading.Lock()

stop_requested = False
def _signal_handler(sig, frame):
    global stop_requested, running
    stop_requested = True
    running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

# Поток для чтения ультразвука
def sensor_loop(poll_interval=0.08):
    global running, latest_distance_cm
    try:
        require_bot_or_exit()
    except Exception:
        pass
    while running:
        try:
            cm = read_distance_cm_from_bot()
        except Exception:
            cm = None
        with distance_lock:
            latest_distance_cm = cm
        time.sleep(poll_interval)

# Управление моторами
def set_motors(m0, m1, m2, m3):
    try:
        bot.Ctrl_Muto(0, int(m0)); bot.Ctrl_Muto(1, int(m1))
        bot.Ctrl_Muto(2, int(m2)); bot.Ctrl_Muto(3, int(m3))
    except Exception as e:
        print("Исключение в set_motors ", e)

def stop_all():
    try:
        set_motors(0, 0, 0, 0)
        try:
            bot.Ctrl_Car(0,1,0); bot.Ctrl_Car(1,1,0)
            bot.Ctrl_Car(2,1,0); bot.Ctrl_Car(3,1,0)
        except Exception:
            pass
    except Exception:
        pass

def forward(speed):
    try:
        pwm_l = int(speed * BIAS_LEFT)
        pwm_r = int(speed * BIAS_RIGHT)
        bot.Ctrl_Muto(0, pwm_l)
        bot.Ctrl_Muto(1, pwm_l)
        bot.Ctrl_Muto(2, pwm_r)
        bot.Ctrl_Muto(3, pwm_r)
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

# Основной контроллер
def main():
    global running, latest_distance_cm, stop_requested

    sensor_thread = threading.Thread(target=sensor_loop, daemon=True)
    sensor_thread.start()

    if bot is None:
        print("экземпляр bot не найден завершаю работу")
        running = False
        sensor_thread.join(timeout=0.5)
        return

    try:
        bot.Ctrl_Ulatist_Switch(1)
        time.sleep(0.12)
    except Exception:
        pass

    for _ in range(3):
        stop_all()
        time.sleep(0.05)

    state = STATE_FORWARD
    action_until = 0.0
    near_count = 0
    med_count = 0

    try:
        while not stop_requested:
            now = time.time()
            with distance_lock:
                cm = latest_distance_cm

            if state == STATE_BACKING:
                if now >= action_until:
                    state = STATE_ROTATING
                    action_until = now + TURN_TIME
                    stop_all()
                    time.sleep(0.06)
                    rotate_left(SPEED_FORWARD)
                else:
                    time.sleep(SENSOR_POLL)
                    continue

            elif state == STATE_ROTATING:
                if now >= action_until:
                    state = STATE_FORWARD
                    stop_all()
                    time.sleep(0.06)
                    forward(SPEED_FORWARD)
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
                    print(f"Подтверждён близкий объект {dis_cm:.1f} см откат назад")
                    stop_all(); time.sleep(0.06)
                    backward(SPEED_BACK)
                    action_until = now + BACK_TIME
                    state = STATE_BACKING
                    near_count = 0; med_count = 0
                    continue

                if med_count >= CONFIRM_COUNT:
                    print(f"Подтверждён объект в средней зоне {dis_cm:.1f} см поворот")
                    stop_all(); time.sleep(0.06)
                    state = STATE_ROTATING
                    action_until = now + TURN_TIME
                    rotate_left(SPEED_FORWARD)
                    med_count = 0; near_count = 0
                    continue

            elif state == STATE_STOPPED:
                stop_all()
                with distance_lock:
                    cm2 = latest_distance_cm
                if cm2 is not None:
                    try:
                        if float(cm2) > FAR_DISTANCE_CM:
                            state = STATE_FORWARD
                            forward(SPEED_FORWARD)
                    except Exception:
                        pass
                time.sleep(SENSOR_POLL)
                continue

            time.sleep(SENSOR_POLL)

    finally:
        running = False
        stop_requested = True
        try:
            for _ in range(3):
                stop_all(); time.sleep(0.05)
        except Exception:
            pass
        try:
            bot.Ctrl_Ulatist_Switch(0)
        except Exception:
            pass
        sensor_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
