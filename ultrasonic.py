import time
import sys

# Библиотека для общение с МК
try:
    import car_adapter as mc
    bot = getattr(mc, 'bot', None)
except Exception:
    bot = None

# Троттлинг лога ошибок чтения: при сбое I2C read_distance_cm_from_bot зовётся
# ~20 раз/с из контура управления — без троттла консоль заливается одинаковыми
# '[Ошибка чтения] ...' (обходя троттл _log_sensor_error в контроллере, т.к.
# наружу уходит None, а не исключение). Печатаем не чаще раза в N секунд.
_ERROR_LOG_INTERVAL_S = 2.0
_last_error_log = 0.0

def require_bot_or_exit():
    if bot is None:
        print("Ошибка: не найден объект bot.")
        sys.exit(1)

def read_distance_cm_from_bot():
    try:
        try:
            bot.Ctrl_Ulatist_Switch(1)
        except Exception:
            pass

        high = bot.read_data_array(0x1b, 1)[0]
        low  = bot.read_data_array(0x1a, 1)[0]
        mm = (high << 8) | low
        return mm / 10.0
    except Exception as ex:
        global _last_error_log
        now = time.time()
        if now - _last_error_log >= _ERROR_LOG_INTERVAL_S:
            _last_error_log = now
            print(f"[Ошибка чтения УЗ] {ex} (троттл {_ERROR_LOG_INTERVAL_S}с)")
        return None
