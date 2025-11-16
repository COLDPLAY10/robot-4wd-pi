import cv2
import numpy as np
import time
import sys
import traceback

CAMERA_ID = 0
FRAME_W = 640
FRAME_H = 360
SHOW_WINDOW = True
DEBOUNCE_SEC = 0.4
SLEEP_BETWEEN_FRAMES = 0.03
MIN_RATIO_TO_TRIGGER = 0.03
# ------------------------------------------------

# Карта цветов: имя -> {'hsv_ranges': [((low),(high)), ...], 'rgb': (R,G,B)}
COLOR_MAP = {
    'white': {
        'hsv_ranges': [((0, 0, 200), (180, 40, 255))],
        'rgb': (255, 255, 255)
    },
    'red': {
        'hsv_ranges': [((0, 100, 100), (8, 255, 255)), ((160, 100, 100), (180, 255, 255))],
        'rgb': (255, 0, 0)
    },
    'green': {
        'hsv_ranges': [((35, 50, 50), (85, 255, 255))],
        'rgb': (0, 255, 0)
    },
    'blue': {
        'hsv_ranges': [((90, 50, 50), (140, 255, 255))],
        'rgb': (0, 0, 255)
    },
    'yellow': {
        'hsv_ranges': [((15, 100, 100), (35, 255, 255))],
        'rgb': (255, 200, 0)
    },
}

COLOR_PRIORITY = ['white', 'red', 'green', 'blue', 'yellow']

bot = None
try:
    import McLumk_Wheel_Sports as mc
    bot = getattr(mc, 'bot', None)
except Exception as e:
    print("Ошибка импорта McLumk_Wheel_Sports:", e)
    bot = None

def safe_call(name, *args):
    if bot is None:
        return False, "no bot"
    fn = getattr(bot, name, None)
    if not callable(fn):
        return False, "no_attr"
    try:
        res = fn(*args)
        return True, res
    except Exception as e:
        return False, str(e)

def set_backlight_rgb(rgb):
    r, g, b = rgb
    ok, info = safe_call('Ctrl_WQ2812_brightness_ALL', r, g, b)
    if ok:
        print(f"LED -> {rgb} (brightness_ALL)")
        return True
    ok2, info2 = safe_call('Ctrl_WQ2812_Alone', r, g, b)
    if ok2:
        print(f"LED -> {rgb} (Alone)")
        return True
    # Низкоуровневый резерв
    safe_call('write_u8', 4, 1)
    time.sleep(0.02)
    safe_call('write_u8', 5, max(r, g, b))
    print(f"LED -> попытки низкоуровневого включения для {rgb} отправлены")
    return False

def turn_off_backlight():
    calls = [
        ('Ctrl_WQ2812_brightness_ALL', (0, 0, 0)),
        ('Ctrl_WQ2812_Alone', (0, 0, 0)),
        ('Ctrl_WQ2812_ALL', (0, 0)),
        ('write_u8', (5, 0)),
        ('write_u8', (4, 0)),
        ('Ctrl_Ulatist_Switch', (0,))
    ]
    for name, args in calls:
        ok, info = safe_call(name, *args)
        print(f"OFF {name}{args} -> {ok}: {info}")
        time.sleep(0.02)

# Вспомогательные функции детекции 
def mask_for_ranges(hsv, ranges):
    mask_total = None
    for low, high in ranges:
        low_arr = np.array(low, dtype=np.uint8)
        high_arr = np.array(high, dtype=np.uint8)
        m = cv2.inRange(hsv, low_arr, high_arr)
        mask_total = m if mask_total is None else cv2.bitwise_or(mask_total, m)
    return mask_total

def detect_colors(frame):
    small = cv2.resize(frame, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    results = {}
    for cname, meta in COLOR_MAP.items():
        try:
            mask = mask_for_ranges(hsv, meta['hsv_ranges'])
            if mask is None:
                mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
            # маска бинарная 
            if mask.dtype != np.uint8:
                mask = mask.astype(np.uint8)
            if mask.max() <= 1:
                mask = (mask * 255).astype(np.uint8)
            ratio = np.count_nonzero(mask) / (mask.shape[0] * mask.shape[1])
            results[cname] = (ratio, mask)
        except Exception as e:
            print(f"[ERROR] detect color {cname}: {e}")
            results[cname] = (0.0, np.zeros((FRAME_H, FRAME_W), dtype=np.uint8))
    return results, small

def choose_color(results):
    active = [c for c in COLOR_PRIORITY if results.get(c, (0, None))[0] >=


MIN_RATIO_TO_TRIGGER]
    if active:
        return active[0]
    candidates = [(c, results[c][0]) for c in results if results[c][0] >= MIN_RATIO_TO_TRIGGER]
    if not candidates:
        return None
    return max(candidates, key=lambda x: x[1])[0]

def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("Ошибка: камера не доступна. Проверь CAMERA_ID и физическое подключение.")
        return

    led_state = False
    current_color = None
    last_change = 0

    print("Запуск детекции цветов. Ctrl+C для выхода.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            results, small = detect_colors(frame)
            chosen = choose_color(results)
            now = time.time()

            if chosen is not None:
                if (not led_state) or (current_color != chosen and (now - last_change) > DEBOUNCE_SEC):
                    last_change = now
                    current_color = chosen
                    led_state = True
                    rgb = COLOR_MAP[chosen]['rgb']
                    print(f"\n[{time.strftime('%H:%M:%S')}] Detected {chosen} ratio={results[chosen][0]:.3f} -> set {rgb}")
                    try:
                        set_backlight_rgb(rgb)
                    except Exception as e:
                        print(f"[ERROR] set_backlight_rgb: {e}")
            else:
                if led_state and (now - last_change) > DEBOUNCE_SEC:
                    last_change = now
                    print(f"\n[{time.strftime('%H:%M:%S')}] Color disappeared -> turning off")
                    try:
                        turn_off_backlight()
                    except Exception as e:
                        print(f"[ERROR] turn_off_backlight: {e}")
                    led_state = False
                    current_color = None

            status = current_color if led_state else "OFF"
            ratios = " ".join([f"{c}:{results[c][0]:.3f}" for c in COLOR_PRIORITY])
            print(f"\r{status} | {ratios}", end='', flush=True)

            # Безопасное отображение окна с кадром и маской
            if SHOW_WINDOW:
                display_mask = np.zeros_like(small)
                try:
                    if current_color:
                        mask = results.get(current_color, (None, None))[1]
                        if mask is None:
                            print(f"\n[WARN] Маска для {current_color} пуста, показываю пустую маску.")
                            mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
                        if mask.dtype != np.uint8:
                            mask = mask.astype(np.uint8)
                        if mask.ndim == 2:
                            if mask.max() <= 1:
                                mask = (mask * 255).astype(np.uint8)
                            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        else:
                            mask_bgr = mask
                        display_mask = mask_bgr
                    if display_mask.shape != small.shape:
                        display_mask = cv2.resize(display_mask, (small.shape[1], small.shape[0]))
                    summary = np.hstack((small, display_mask))
                    cv2.imshow('frame | mask', summary)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except Exception as e:
                    print(f"\n[ERROR] при отображении маски: {e}")
                    try:
                        cv2.imshow('frame | mask', small)


                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    except Exception:
                        pass

            time.sleep(SLEEP_BETWEEN_FRAMES)

    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
    finally:
        try:
            if led_state:
                turn_off_backlight()
        except Exception:
            pass
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        print("Завершено.")

if __name__ == "__main__":
    main()







