import ultrasonic
import camera

DIST_THRESHOLD_CM = 60        # порог расстояния для препятствия
WHITE_RATIO_THRESHOLD = 0.05  # доля белого для камеры
NUM_SAMPLES = 3               # измерений для усреднения
TIMEOUT = 0.02                # таймаут для эха

def main():
    ultrasonic_result = ultrasonic.detect(
        threshold_cm=DIST_THRESHOLD_CM,
        num_samples=NUM_SAMPLES,
        timeout=TIMEOUT
    )

    camera_result = camera.detect_obstacle_by_color(
        white_ratio_thr=WHITE_RATIO_THRESHOLD
    )

    print("\nРезультаты:")
    print(f"  Ультразвук: {ultrasonic_result}")
    print(f"  Камера: {camera_result}")

    if ultrasonic_result and camera_result:
        print("\n Обнаружено препятствие")
    elif ultrasonic_result or camera_result:
        print("\n Возможное препятствие")
    else:
        print("\n Препятствий не обнаружено")

if __name__ == "__main__":
    main()
