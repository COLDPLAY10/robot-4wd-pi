import RPi.GPIO as GPIO
import time

# Настройка пинов
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Отправляем короткий импульс
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Ждём отклика
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Вычисляем дистанцию (см)
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

# Тест работы
try:
    while True:
        dist = get_distance()
        if dist < 20:  # меньше 20 см — препятствие
            print("1 (вижу препятствие) | Расстояние:", dist, "см")
        else:
            print("0 (чисто) | Расстояние:", dist, "см")
        time.sleep(0.3)

except KeyboardInterrupt:
    GPIO.cleanup()
