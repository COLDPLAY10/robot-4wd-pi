# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Что это

Автономный робот на платформе Yahboom Raspbot V2 (4WD-мекалум, Raspberry Pi 5): SLAM по лидару Yahboom T-MINI Plus, depth-восприятие камерой (Depth-Anything-V2 ONNX), ультразвук, A*-планирование, реактивный объезд препятствий. Код и комментарии — на русском, придерживайся этого.

Тестов и линтера в проекте нет. Запуск целиком возможен только на Pi: `car_adapter.py` импортирует `Raspbot_Lib` (ставится отдельно по инструкции Yahboom) на уровне модуля, поэтому на дев-машине всё, что тянет `car_adapter`/`navigation_controller`, не импортируется. Без железа можно импортировать и проверять: `map/slam_core.py`, `map/scan_matcher.py`, `map/path_planner.py`, `map/sensor_fusion.py`, `map/map_visualizer.py`, `camera_perception/*`. У `scan_matcher` и `path_planner` есть numpy-fallback'и на случай отсутствия cv2.

## Команды

```bash
pip3 install -r requirements.txt          # numpy, pyserial, opencv, onnxruntime, matplotlib

# Основной сценарий (на роботе, из map_scripts/):
python3 demo_with_lidar.py explore                      # построить карту → .pkl + .png по Ctrl+C
python3 demo_with_lidar.py explore --map old.pkl        # дополнить существующую карту
python3 demo_with_lidar.py goto X Y --map map.pkl       # ехать к цели по готовой карте (localization)

python3 autonomous_navigation.py                        # интерактивный режим (explore/goto/save/load)
python3 demo_no_lidar.py explore                        # деградированный режим: УЗ + камера

# Утилиты:
python3 diagnose_lidar.py                               # диагностика UART/лидара
python3 map_scripts/calibrate_odometry.py               # калибровка MAX_LINEAR/ANGULAR_SPEED
python3 map_scripts/visualize_map.py map.pkl            # перерисовать карту
python3 map_scripts/compare_trajectory.py map.pkl       # дрейф одометрии vs SLAM, loop-closure
python3 scripts/setup_depth_model.py                    # экспорт .pth → ONNX (на дев-машине, нужны torch+transformers)
```

Модель глубины лежит в `models/depth_anything_v2_small.onnx` (не в git).

## Архитектура

Поток данных:

```
lidar.py / ultrasonic.py / камера+DepthEstimator
        │ (NavigationController._update_sensors, ~20 Гц)
        ├──► SLAM (map/slam_core.py) ──► OccupancyGrid ──► PathPlanner (A* по inflated grid)
        └──► SensorFusion (реактивный слой, секторные дистанции front/left/right/back)
                    │
        NavigationController (конечный автомат) ──► car_adapter.py ──► PWM моторов
```

Два контура работают параллельно и не надо их путать:
- **Картографический** (SLAM → A*): медленный, глобальный, через occupancy grid.
- **Реактивный** (SensorFusion): мгновенные секторные расстояния для стопов/объезда, в карту не пишет.

### SLAM (`map/slam_core.py`)

- `OccupancyGrid` 400×400 × 0.05 м (20×20 м), значения 0..100, prior 50, обновление log-odds с клампом ±5. Старт робота = центр карты = мировое (0,0).
- **Одометрия открытого контура**: энкодеров нет, скорость оценивается из PWM (`MAX_LINEAR_SPEED`, `MAX_ANGULAR_SPEED` в `NavigationController`). Ошибка ±25% и хуже — точность позы целиком держится на scan matcher'е.
- **Scan matcher** (`map/scan_matcher.py`, Hector-style): likelihood field = distance transform от occupied-ячеек (порог 65), Gauss-Newton по билинейно интерполированному полю. Гейты отказа: residual > 0.18 м, прыжок позы > 0.8 м / 45° от одометрического прайра, мало точек в «виденной» области. При отказе матча поза остаётся одометрической — это штатно.
- Параллельно ведётся `odom_only_history` (чистая одометрия без коррекций) — для отчётных сравнений, не трогать при правках логики.
- **Режимы**: `mapping` (пишем карту + правим позу) и `localization` (карта из .pkl read-only, робот физически ставится в (0,0,0), `reset_pose=True`). Все `update_with_*` проверяют режим перед записью в карту.
- Запись в карту: лидар (occupied 0.9 / free вдоль луча 0.5), ультразвук (конус ±15° семью лучами), камера (depth → обратная pinhole-проекция `camera_perception/projection.py` → фильтр по высоте 3–50 см над полом → occupied 0.7). `update_with_camera_depth` ВСЕГДА возвращает облако препятствий, даже в localization — оно нужно реактивному слою.

### Приоритеты сенсоров (`map/sensor_fusion.py`)

Иерархия доверия: **лидар (0.9) > камера (0.7) > ультразвук (0.6)**, таймаут данных 1 с. Логика `get_obstacle_distance`:
- База = лидар (медиана нижнего квартиля точек сектора, точки <12 см отбрасываются как шум) ⊕ ультразвук (только front; конфликт «лидар <0.2 м, УЗ >0.4 м» → верим УЗ; иначе 0.6/0.4 средневзвешенное).
- **Принцип безопасности камеры**: по фронту берётся `min(база, камера)` — камера может только *сократить* дистанцию (она видит низкие препятствия, недоступные 2D-лидару), но никогда не объявляет «свободно» в обход лидара. По бокам камера — только gap-fill при отсутствии лидара. Отключение реактивной камеры одним флагом: `CAMERA_REACTIVE_ENABLED` в `NavigationController`.
- **Коридорная метрика камеры** (`obstacle_distances_by_sector`, `corridor_halfwidth_m`): кроме угловых секторов, фронт-дистанция учитывает точки в полосе шириной робота независимо от пеленга — ловит угловые препятствия «под колесо» и диагональные стены. Дальности камеры раздельные: анализ сцены до `CAMERA_MAX_RANGE_M` (6 м), запись в карту до `CAMERA_MAP_MAX_RANGE_M` (3 м) — дальняя монокулярная глубина шумит, а occupied-ячейка блокирует A*.
- **Страховочный канал `nearest_in_depth_band`**: «ближайший объект по курсу» прямо из depth-карты, без проекции и фильтра высот — робастный минимум в полосе у линии горизонта (±5% кадра, центральные 55% ширины; пол в этих строках физически дальше ~3 м, ложных стопов от пола нет). Ловит сетки/прозрачное/редкие облака, где секторный канал молчит. Вливается в `front` камеры через min.
- **Отладка камеры**: `CAMERA_DEBUG_SAVE_ENABLED` в `NavigationController` — раз в `CAMERA_DEBUG_PERIOD_S` пишет в `camera_debug/` кадр+depth с красной маской пикселей-препятствий (`camera_perception/debug_viz.py`); ротация `CAMERA_DEBUG_KEEP`.
- **Выбор направления**: при препятствии/объезде контроллер поворачивает в сторону большего простора (`_choose_turn_direction`/`_side_clearance` по секторам SensorFusion), а не всегда налево.

### Inflation (`map/path_planner.py`)

- Перед планированием препятствия (ячейки ≥ 65) раздуваются `cv2.dilate` эллипсом радиуса `ROBOT_RADIUS + SAFETY_MARGIN` (0.12 + 0.08 = 20 см), throttle пересчёта 0.5 с.
- A* (8-связный) и проверка коллизий DWA ходят по inflated grid **центром робота** — размер робота уже учтён в раздувании, никаких проверок по окружности.
- Стартовая клетка в inflated-зоне разрешена (`allow_inflated=True`) — робот там физически стоит, off-by-one пиксель не должен ронять планирование. Целевая — нет.
- Тот же радиус рисуется красным поясом на PNG карты (`save_map` → `map_visualizer.render_map_png`) — визуализация обязана совпадать с тем, по чему реально планирует A*.

### Навигация (`map/navigation_controller.py`)

Конечный автомат: `IDLE → EXPLORATION | NAVIGATION → OBSTACLE_AVOIDANCE`. Каждый `update()` → `_tick()` (сенсоры + одометрия + SLAM, dt capped 0.5 с) → поведение режима:
- **EXPLORATION** — реактивное блуждание: время езды вперёд пропорционально дистанции до препятствия; <0.4 м — поворот; <0.15 м — экстренный стоп + назад; `consecutive_rotations` (макс 3) разрывает зацикливание.
- **NAVIGATION** — следование A*-пути по waypoint'ам: угол >20° — поворот на месте, иначе отрезки по 0.3 с с проверками; waypoint засчитан при 0.2 м.
- **OBSTACLE_AVOIDANCE** — назад → объезд через свободную сторону из SensorFusion → перепланирование A*; неудача → EXPLORATION.

Все движения — через `_safe_move_forward`/`_safe_rotate`, которые обязаны тикать `_tick()` каждые 0.1–0.2 с, иначе одометрия не накапливается. При добавлении любого блокирующего движения сохраняй этот паттерн: `ca.команда()` → `_set_movement_command()` → `self.last_update_time = time.time()` → цикл с `_tick()`.

Плавность движения (важно не сломать):
- **Инференс глубины — в фоновом потоке** (`_DepthWorker`): главный цикл НЕ блокируется на нейронке, `_update_sensors` неблокирующе забирает последний результат; поза для проекции облака снимается в момент захвата кадра. Не возвращай инференс в `_tick()`.
- **Круиз**: в EXPLORATION `_safe_move_forward(..., stop_at_end=False)` не останавливает моторы между последовательными «вперёд»; стоп — только при препятствии/повороте. `_safe_rotate` и blocked-ветка сами останавливают робота, если он на ходу.
- **`forward_steer`**: подруливание дифференциалом бортов (`ca.move_param_forward`, p>0 → правый борт → влево). Одометрия моделирует его в `_update_odometry` (v×(1+p/200), ω=v·p/100/WHEEL_BASE) — любое новое «кривое» движение обязано получить такую же модель, иначе scan matcher теряет прайр. Используется: увод к простору на круизе (`_steer_to_free_space`, только если обе стороны измерены) и докрутка курса ≤20° в NAVIGATION (3%/°, кап ±60%).
- **Плавное замедление**: `_speed_for_distance` — линейный спад PWM от 0.7 м до `MIN_MOVE_PWM=22` у 0.4 м.

### Константы — single source of truth

`WHEEL_BASE`, `ROBOT_RADIUS`, `SAFETY_MARGIN`, `MAX_LINEAR/ANGULAR_SPEED`, параметры камеры — определены в `NavigationController` и явно прокидываются в SLAM и PathPlanner при инициализации. `WHEEL_BASE` критичен: SLAM восстанавливает omega из скоростей бортов тем же значением, рассинхрон ломает сходимость scan matcher'а. Меняй только в одном месте.

## Что важно знать о состоянии кода

- **Лидар — YDLidar T-mini Plus** (пакеты `AA 55`, команды `A5 60`/`A5 65`), а НЕ Oradar MS200, как считал PR #7. `lidar.py` — рабочий драйвер с честным парсером протокола. Калибровочные константы (`DISTANCE_DIVISOR`, `CLOCKWISE`, `ANGLE_OFFSET_DEG`) проверяются на железе тестом `python3 lidar.py`. Протокол и разбор карт-«колец» от старого парсера — `samples/ANALYSIS.md`.
- **DWA в `path_planner.py` (`compute_velocity_command`) — мёртвый код**: реализован, но NavigationController его не вызывает, локальное управление сделано дискретными командами. Не описывай его как работающую часть контура.
- **`camera_segmentation/` — deprecated**, заменён depth-восприятием (`camera_perception/`); `update_with_camera_segmentation` в slam_core оставлен для совместимости со старыми скриптами.
- Корневые `main.py`, `obstacle_detector.py`, `drive_ultrasonic.py` — старые реактивные скрипты без SLAM.
- README в `map_scripts/` местами устарел (упоминает сегментацию и несуществующие флаги `--lidar`; в argparse — `--no-lidar` и т.п.).
- Карты сохраняются как `.pkl` (грид + траектории + статистика) + `.png`; формат `.pkl` поддерживает старые файлы без `odom_only_history`.
