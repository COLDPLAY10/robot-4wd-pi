import time
import sys

# Библиотека для общение с МК 
try:
    import car_adapter as mc
    bot = getattr(mc, 'bot', None)
except Exception:
    bot = None

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
        print(f"[Ошибка чтения] {ex}")
        return None
