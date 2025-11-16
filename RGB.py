import time

# Пытаемся получить объект bot из car_adapter
try:
    import car_adapter as ca
    _bot = getattr(ca, "bot", None)   
    if _bot is None:
        _bot = getattr(ca, "Bot", None)  
        if callable(_bot) and not hasattr(_bot, "Ctrl_Muto"):
            _bot = _bot()   
            ca.bot = _bot   
except Exception:
    _bot = None

# Словарь индексов цветов
COLOR_INDEX_MAP = {
    "red": 0,
    "green": 1,
    "blue": 2,
    "yellow": 3,
    "purple": 4,
    "cyan": 5,
    "white": 6,
    "off": 7,
}

def get_bot():
    global _bot
    return _bot

def set_color_index(idx: int) -> bool:
    bot = get_bot()
    if bot is None:
        return False
    try:
        if idx == 7:
            bot.Ctrl_WQ2812_ALL(0, 0)      # выключить
        else:
            bot.Ctrl_WQ2812_ALL(1, int(idx))  # включить цвет
        return True
    except Exception:
        return False
        
#установка конкретного цвета 
def set_rgb(r: int, g: int, b: int) -> bool:
    bot = get_bot()
    if bot is None:
        return False
    try:
        bot.Ctrl_WQ2812_brightness_ALL(int(r), int(g), int(b))
        return True
    except Exception:
        return False

def off() -> bool:
    return set_color_index(COLOR_INDEX_MAP["off"])

def set_red() -> bool:
    return set_color_index(COLOR_INDEX_MAP["red"])

def set_green() -> bool:
    return set_color_index(COLOR_INDEX_MAP["green"])
