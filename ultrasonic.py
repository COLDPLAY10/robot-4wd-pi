import RPi.GPIO as GPIO
import time

""" Пины 
TRIG = 23
ECHO = 24"""

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def _pulse_time(timeout=0.02):
    #Возвращает (pulse_start, pulse_end) или (None, None) при тайм ауте
    start = time.monotonic()
    while GPIO.input(ECHO) == 0:
        if time.monotonic() - start > timeout:
            return None, None
    pulse_start = time.monotonic()

    while GPIO.input(ECHO) == 1:
        if time.monotonic() - pulse_start > timeout:
            return None, None
    pulse_end = time.monotonic()
    return pulse_start, pulse_end

def get_distance(num_samples=3, timeout=0.02):
    """ Возвращает усреднённую дистанцию в см или None при ошибке
    timeout - максимальное время ожидания эха"""
    distances = []
    for _ in range(num_samples):
        # посылаем импульс
        GPIO.output(TRIG, False)
        time.sleep(0.00005)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start, pulse_end = _pulse_time(timeout=timeout)
        if pulse_start is None or pulse_end is None:
            return None  # тайм-аут — нет эха
        duration = pulse_end - pulse_start
        dist = duration * 17150
        distances.append(dist)
        time.sleep(0.01)  # короткая пауза между измерениями

    return round(sum(distances) / len(distances), 2)

def detect(threshold_cm=20, **kwargs):
    """Возвращает True если препятствие ближе threshold_cm, False если нет, None при ошибке."""
    dist = get_distance(**kwargs)
    if dist is None:
        print("Ошибка измерения")
        return None
    print(f"Distance: {dist} cm")
    return dist < threshold_cm

if __name__ == "__main__":
    try:
        while True:
            res = detect(threshold_cm=20, num_samples=3, timeout=0.02)
            if res is True:
                print("Обнаружено препятствие")
            elif res is False:
                print("Нет препятсвтие")
            else:
                print("Ошибка измерения")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("GPIO очищен")
