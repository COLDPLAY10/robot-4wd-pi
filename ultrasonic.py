import RPi.GPIO as GPIO
import time

# Настройка пинов (один раз)
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def _pulse_time(timeout):
    #Возвращает время старта и конца импульса, либо (None, None) при таймауте
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


def get_distance(num_samples, timeout):
    #Измеряет расстояние (усреднённо), возвращает см или None при ошибке
    distances = []
    for _ in range(num_samples):
        GPIO.output(TRIG, False)
        time.sleep(0.00005)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start, pulse_end = _pulse_time(timeout)
        if pulse_start is None or pulse_end is None:
            return None

        duration = pulse_end - pulse_start
        dist = duration * 17150
        distances.append(dist)
        time.sleep(0.01)

    return round(sum(distances) / len(distances), 2)


def detect(threshold_cm, num_samples, timeout):
    #Возвращает True если препятствие ближе threshold_cm, False если нет
    dist = get_distance(num_samples=num_samples, timeout=timeout)
    if dist is None:
        print("Ошибка измерения")
        return None

    print(f"Distance: {dist:.2f} cm")
    return dist < threshold_cm
