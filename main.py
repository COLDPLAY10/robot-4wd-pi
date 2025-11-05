import ultrasonic
import camera

def main():
    
    # Получаем данные с датчиков
    ultrasonic_detected = ultrasonic.detect()  # 1 — видит, 0 — нет
    camera_detected = camera.detect_obstacle_by_color()  # 1 — видит, 0 — нет

    print(f"Ультразвук: {ultrasonic_detected}")
    print(f"Камера: {camera_detected}")

    # Проверяем, оба ли видят препятствие
    if ultrasonic_detected == 1 and camera_detected == 1:
        print("⚠️  Обнаружено препятствие (оба датчика подтверждают)")
    elif ultrasonic_detected == 1:
        print("🔊 Обнаружено препятствие ультразвуком")
    elif camera_detected == 1:
        print("📷 Обнаружено препятствие камерой")
    else:
        print("✅ Препятствий не обнаружено")

if __name__ == "__main__":
    main()

