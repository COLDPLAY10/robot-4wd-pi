#!/usr/bin/env python3
"""
Контроллер навигации робота
Управляет режимами работы и координирует все компоненты
"""

import threading
import queue
import time
import numpy as np
from enum import Enum
from typing import Optional, Tuple
import sys
import os

# Добавляем родительскую директорию в путь для импорта
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import car_adapter as ca
from map.slam_core import SLAM, Position
from map.sensor_fusion import SensorFusion
from map.path_planner import PathPlanner, PathPoint
from map.sensor_recorder import SensorRecorder


class NavigationMode(Enum):
    """Режимы работы робота"""
    IDLE = "idle"                    # Ожидание
    EXPLORATION = "exploration"       # Анализ среды
    NAVIGATION = "navigation"         # Движение по маршруту
    OBSTACLE_AVOIDANCE = "avoidance" # Объезд препятствий


class _DepthWorker(threading.Thread):
    """
    Фоновый поток инференса глубины.

    Зачем: get_depth_map на Pi занимает 0.3–3 с. Раньше он вызывался синхронно
    из _tick() — контур управления замирал: робот либо «думал» стоя, либо ехал
    вслепую без проверок препятствий. Теперь поток сам читает камеру и держит
    последний результат; основной цикл забирает его НЕблокирующе и всегда
    остаётся на частоте лидара/ультразвука.

    Поза снимается В МОМЕНТ ЗАХВАТА кадра: проекция облака должна считаться
    от позы, в которой кадр снят, а не от позы на момент потребления — робот
    успевает уехать за время инференса.

    Известный компромисс: get_pose читает x/y/theta без блокировки, пока
    главный поток их мутирует — возможна «рваная» поза из соседних тиков.
    При наших скоростях (≤0.16 м/с, тик ≤0.2 с) ошибка миллиметровая, на фоне
    шума монокулярной глубины пренебрежима. Решили не тащить лок в SLAM.
    """

    def __init__(self, cap, estimator, get_pose, period_s: float):
        super().__init__(daemon=True, name="depth-worker")
        self._cap = cap
        self._estimator = estimator
        self._get_pose = get_pose
        self._period_s = period_s
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._result = None  # (frame, depth, pose, timestamp) — для SLAM/реактива
        self._latest = None  # (frame_copy, pose, depth|None, ts) — для лога кадров
        self.inference_count = 0

    def run(self):
        while not self._stop_evt.is_set():
            t0 = time.time()
            try:
                pose = self._get_pose()
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    self._stop_evt.wait(0.2)
                    continue
                depth = self._estimator.get_depth_map(frame)
                ts = time.time()
                with self._lock:
                    # Последний кадр доступен ВСЕГДА (даже когда инференс не дал
                    # глубину) — нужен периодическому логгеру кадров. copy(): cv2
                    # может переиспользовать буфер на следующем read(), а кадр
                    # уходит в фоновую запись и живёт дольше этого такта.
                    self._latest = (frame.copy(), pose, depth, ts)
                    if depth is not None:
                        self._result = (frame, depth, pose, ts)
                if depth is not None:
                    self.inference_count += 1
            except Exception as e:
                print(f"[DepthWorker] Ошибка: {type(e).__name__}: {e}")
                self._stop_evt.wait(0.5)
            # Темп: не чаще period_s (если инференс быстрее — ждём)
            elapsed = time.time() - t0
            if elapsed < self._period_s:
                self._stop_evt.wait(self._period_s - elapsed)

    def take_latest(self, newer_than: float):
        """Последний результат, если он свежее newer_than; иначе None."""
        with self._lock:
            if self._result is not None and self._result[3] > newer_than:
                return self._result
        return None

    def peek_frame(self):
        """
        Последний прочитанный кадр (frame, pose, depth|None, ts) — для лога.
        Доступен, даже когда инференс глубины не дал результата (модель медленная
        или недоступна): лог кадров не должен зависеть от глубины.
        """
        with self._lock:
            return self._latest

    def stop(self, timeout: float = 2.0):
        self._stop_evt.set()
        if self.is_alive():
            self.join(timeout=timeout)


class _SegWorker(threading.Thread):
    """
    Фоновый поток сегментации пола (SegFormer). ОТДЕЛЬНЫЙ от _DepthWorker и
    медленнее него — намеренно: depth и страховочный канал nearest_in_depth_band
    (ловит ножки/тонкое для БЫСТРОЙ реакции) не должны наследовать латентность
    второй сети. Слияние seg+depth идёт на СВЕЖЕЙ глубине и ПОСЛЕДНЕЙ маске:
    floor/не-floor пространственно стабилен ~секунду при ≤0.16 м/с, а высота
    берётся из свежей глубины и свежей подгонки плоскости.

    Берёт кадр из _DepthWorker.peek_frame() (не открывает камеру второй раз),
    сегментирует, публикует последнюю маску. При недоступной модели поток не
    запускается — слияние тихо откатывается на legacy height-filter.
    """

    def __init__(self, peek_frame, segmenter, period_s: float):
        super().__init__(daemon=True, name="seg-worker")
        self._peek = peek_frame
        self._segmenter = segmenter
        self._period_s = period_s
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._mask = None          # (floor_mask, ts)
        self._last_frame_ts = None
        self.inference_count = 0

    def run(self):
        while not self._stop_evt.is_set():
            t0 = time.time()
            try:
                snap = self._peek()
                # snap = (frame, pose, depth|None, ts) из _DepthWorker
                if snap is not None and snap[3] != self._last_frame_ts:
                    frame = snap[0]
                    self._last_frame_ts = snap[3]
                    mask = self._segmenter.segment_floor(frame)
                    if mask is not None:
                        with self._lock:
                            self._mask = (mask, time.time())
                        self.inference_count += 1
            except Exception as e:
                print(f"[SegWorker] Ошибка: {type(e).__name__}: {e}")
                self._stop_evt.wait(0.5)
            elapsed = time.time() - t0
            if elapsed < self._period_s:
                self._stop_evt.wait(self._period_s - elapsed)

    def latest_mask(self, max_age_s: float):
        """Последняя маска пола, если свежее max_age_s; иначе None (→ fallback)."""
        with self._lock:
            if self._mask is not None and (time.time() - self._mask[1]) <= max_age_s:
                return self._mask[0]
        return None

    def stop(self, timeout: float = 2.0):
        self._stop_evt.set()
        if self.is_alive():
            self.join(timeout=timeout)


class _ImageLogger(threading.Thread):
    """
    Фоновый писатель кадров камеры на диск (results/<запуск>/segmentation/).

    Зачем поток: cv2.imwrite + np.save(~0.6 МБ) + ротация каталога — это десятки-
    сотни мс на SD-карте. Раньше они шли СИНХРОННО в контуре управления
    (_update_sensors), морозя робота на запись: пока пишется кадр, моторы в
    круизе не остановлены и робот едет вслепую. Теперь главный цикл лишь кладёт
    кадр в очередь, а диск трогает этот поток.

    Запись надёжная и ГРОМКАЯ: первый успешный кадр и первая ошибка печатаются с
    полным путём — если каталог недоступен, это видно сразу, а не теряется молча.
    """

    def __init__(self, keep: int, max_queue: int = 64):
        super().__init__(daemon=True, name="image-logger")
        self._q: "queue.Queue" = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._keep = keep
        self._announced_ok = False
        self._announced_err = False
        self.written = 0
        self.dropped = 0

    def submit(self, out_dir: str, base: str, frame_bgr, depth):
        """Неблокирующе: при переполнении очереди кадр отбрасывается (счётчик)."""
        try:
            self._q.put_nowait((out_dir, base, frame_bgr, depth))
        except queue.Full:
            self.dropped += 1

    def run(self):
        import cv2  # на роботе есть (используется камерой)
        while True:
            try:
                out_dir, base, frame, depth = self._q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    break
                continue
            try:
                os.makedirs(out_dir, exist_ok=True)
                jpg_path = os.path.join(out_dir, base + '.jpg')
                if not cv2.imwrite(jpg_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                    raise OSError(f"cv2.imwrite вернул False для {jpg_path}")
                if depth is not None:
                    np.save(os.path.join(out_dir, base + '.npy'),
                            depth.astype(np.float16))
                self.written += 1
                if not self._announced_ok:
                    self._announced_ok = True
                    print(f"[ImageLogger] Кадры пишутся в {out_dir} "
                          f"(первый: {base}.jpg)")
                self._rotate(out_dir)
            except Exception as e:
                if not self._announced_err:
                    self._announced_err = True
                    print(f"[ImageLogger] !!! ОШИБКА записи кадра в {out_dir}: "
                          f"{type(e).__name__}: {e}")

    def _rotate(self, out_dir: str):
        """Держим только `keep` последних кадров; .npy удаляем парно с .jpg."""
        try:
            jpgs = sorted(f for f in os.listdir(out_dir)
                          if f.startswith('cap_') and f.endswith('.jpg'))
            for old in jpgs[:-self._keep]:
                stem = old[:-4]
                for ext in ('.jpg', '.npy'):
                    try:
                        os.remove(os.path.join(out_dir, stem + ext))
                    except OSError:
                        pass
        except OSError:
            pass

    def stop(self, timeout: float = 3.0):
        self._stop.set()
        if self.is_alive():
            self.join(timeout=timeout)


class NavigationController:
    """
    Основной контроллер навигации
    Интегрирует SLAM, планирование и управление
    """

    def __init__(self, use_lidar=False, use_camera=True, use_ultrasonic=True,
                 mapping_mode='mapping', map_file=None):
        """
        Args:
            use_lidar: использовать лидар
            use_camera: использовать камеру
            use_ultrasonic: использовать ультразвуковой датчик
            mapping_mode: 'mapping' — строим карту, 'localization' — едем по готовой
            map_file: путь к .pkl карте; обязателен для localization, опционален
                      для mapping (если задан — продолжаем уже существующую карту)
        """
        print("\n" + "="*60)
        print("ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ НАВИГАЦИИ")
        print("="*60)

        if mapping_mode == 'localization' and not map_file:
            raise ValueError("Режим localization требует map_file с готовой картой")

        self.use_lidar = use_lidar
        self.use_camera = use_camera
        self.use_ultrasonic = use_ultrasonic
        self.mapping_mode = mapping_mode

        # Все результаты этого запуска — в одну подпапку по времени старта:
        # results/<запуск>/{maps,depth,segmentation}/. Единый источник правды по
        # путям; каталоги создаются лениво при первой записи (пустой запуск не
        # плодит папок). save_map кладёт сюда карты, отладка камеры — в depth/,
        # захваты/overlay сегментации — в segmentation/.
        self.run_stamp = time.strftime('%Y%m%d-%H%M%S')
        self.run_dir = os.path.join(self.RESULTS_DIR, self.run_stamp)
        self.maps_dir = os.path.join(self.run_dir, 'maps')
        self.depth_debug_dir = os.path.join(self.run_dir, 'depth')
        self.seg_dir = os.path.join(self.run_dir, 'segmentation')
        print(f"[NavController] Результаты запуска: {self.run_dir}")

        # Рекордер сырых данных сенсоров → results/<запуск>/sensors/. Пишет в
        # фоне, на контур не влияет; файлы создаются лениво при первом событии.
        self.sensors_dir = os.path.join(self.run_dir, 'sensors')
        self.recorder = SensorRecorder(self.sensors_dir,
                                       enabled=self.SENSOR_RECORD_ENABLED)
        self.recorder.start()

        # Фоновый лог кадров камеры → results/<запуск>/segmentation/. Каталог
        # создаётся и проверяется на запись СРАЗУ (а не лениво в цикле): если
        # путь недоступен, видно на старте.
        self._init_image_logging()

        # Инициализация компонентов.
        # WHEEL_BASE прокидываем в SLAM явно — single source of truth, иначе
        # omega восстанавливается с ошибкой 35-50% и scan matcher не сходится.
        self.slam = SLAM(
            map_width=400,
            map_height=400,
            resolution=0.05,
            use_lidar=use_lidar,
            mapping_mode=mapping_mode,
            wheel_base=self.WHEEL_BASE,
        )

        # Загрузка карты — в localization обязательно, в mapping опционально
        if map_file:
            # В localization робота считаем стоящим в (0,0,0) на старте.
            # В mapping подтягиваем последнюю позицию из истории, чтобы
            # дополнять старую карту.
            self.slam.load_map(map_file, reset_pose=(mapping_mode == 'localization'))

        self.sensor_fusion = SensorFusion(
            use_lidar=use_lidar,
            use_camera=use_camera,
            use_ultrasonic=use_ultrasonic
        )

        # ROBOT_RADIUS/SAFETY_MARGIN — single source of truth, чтобы inflation
        # в карте соответствовал реальному footprint тележки.
        self.path_planner = self._make_path_planner()

        # Режим работы
        self.mode = NavigationMode.IDLE

        # Параметры движения (откалибровано для 4WD)
        self.exploration_speed = 30      # Скорость для исследования (PWM, мин. для реального движения)
        self.navigation_speed = 40       # Скорость для навигации по маршруту
        # Поворот на месте — самое тяжёлое движение мекалума (скрэб колёс):
        # PWM ниже порога трогания (22) даёт фантомный поворот в одометрии
        # при стоящих колёсах.
        self.turn_speed = 25             # Скорость поворота
        self.min_obstacle_distance = 0.4 # Минимальное расстояние до препятствия (м)
        # Дистанции меряются ОТ ЦЕНТРА робота; нос ~0.12 м впереди. УЗ при этом
        # меряет от своего места (≈ нос), поэтому запас по факту ещё больше.
        # 0.28 → ~16 см от носа: с учётом латентности цикла и тормозного пути
        # робот не утыкается в препятствие (нога/порог) носом. Старое 0.20 =
        # ~8 см от носа — успевал коснуться до полной остановки.
        self.critical_distance = 0.28    # Критическое расстояние для экстренной остановки
        # Порог прерывания ПОВОРОТА на месте — отдельный и более тесный, чем
        # forward-critical: вращение не двигает робота вперёд, и поворачивать
        # надо именно ОТ близкого препятствия. Если приравнять к 0.28, робот
        # станет слишком робким, чтобы отвернуться от объекта в ~0.25 м.
        self.rotate_safety_distance = 0.20
        self.goal_tolerance = 0.2        # допустимое отклонение от цели (м)

        # Текущая цель
        self.current_goal = None
        self.current_path = []
        self.current_waypoint_idx = 0

        # Время последнего обновления
        self.last_update_time = time.time()
        # Метка времени последнего ПОТРЕБЛЁННОГО результата фонового
        # инференса глубины (см. _DepthWorker / _update_sensors).
        self._depth_consumed_ts = 0.0

        # Отслеживание движения для одометрии
        self.current_command = "stop"  # stop, forward, forward_steer, tank, backward, rotate_left, rotate_right
        self.current_speed = 0   # PWM 0-255
        self.current_steer = 0.0  # процент увода для forward_steer (см. move_param_forward)
        self.current_tank = (0, 0)  # PWM бортов для команды tank (DWA)

        # Состояние DWA-навигации
        self._dwa_vel = (0.0, 0.0)   # последняя командованная (v, omega)
        self._dwa_stall_count = 0    # подряд тактов «нет проходимой траектории»

        # Счетчик последовательных поворотов (для предотвращения зацикливания)
        self.consecutive_rotations = 0
        self.max_consecutive_rotations = 3  # Максимум 3 поворота подряд

        # Робот
        self.bot = ca.bot

        # Лидар
        self.lidar = None
        if self.use_lidar:
            self._init_lidar()

        # Инициализация датчиков
        self._init_sensors()

        print(f"[NavController] Скорости: исследование={self.exploration_speed}, "
              f"навигация={self.navigation_speed}, поворот={self.turn_speed}")
        print(f"[NavController] Дистанции: мин={self.min_obstacle_distance}м, "
              f"крит={self.critical_distance}м")
        print("="*60)
        print("СИСТЕМА ГОТОВА К РАБОТЕ")
        print("="*60 + "\n")

    # ===== Параметры камеры для depth-based perception =====
    # На дев-машине без камеры можно тестировать через подмену camera_cap.
    CAMERA_DEVICE_ID = 0           # /dev/video0 на Pi
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_HFOV_DEG = 60.0         # типичный USB-веб для робота
    # --- Серво-кронштейн камеры (pan/tilt) ---
    # При старте восприятия приводим голову камеры в известное положение.
    # Без этого геометрия проекции глубины не определена: яхбумовский автостарт
    # (reservo.py, pan=90/tilt=25) на SD-системе отсутствует, и камера остаётся
    # там, куда её повернули в прошлый раз. Управление — через тот же
    # Raspbot_Lib (bot.Ctrl_Servo), новых зависимостей нет.
    CAMERA_SERVO_PAN_ID = 1
    CAMERA_SERVO_TILT_ID = 2
    CAMERA_SERVO_PAN_DEG = 90    # центр по горизонтали
    # 25 — штатное рабочее положение Yahboom. ВАЖНО: если при tilt=25 взгляд
    # не строго горизонтален, вписать фактический наклон оптической оси в
    # CAMERA_MOUNT_TILT_RAD (>0 = вниз) — см. ROBOT_CHECKLIST.md.
    CAMERA_SERVO_TILT_DEG = 25
    CAMERA_MOUNT_HEIGHT_M = 0.125  # высота объектива над полом (замер)
    # Из замеров: длина 0.209 м (нос камеры → задний край колеса), задний свес
    # от центра вращения ≈ 0.089 м → объектив ≈ 0.12 м впереди центра.
    # Уточнить после замера диаметра колеса (ROBOT_CHECKLIST.md).
    CAMERA_MOUNT_FORWARD_M = 0.12  # смещение вперёд от центра робота
    CAMERA_MOUNT_TILT_RAD = 0.0    # 0 = горизонтально, >0 = смотрит вниз
    CAMERA_PIXEL_STRIDE = 8        # обрабатывать каждый 8-й пиксель
    # Анализ сцены — на всю практическую дальность модели (metric-indoor,
    # дальше ~6 м растёт шум, DepthEstimator клампит на 10 м): реактивный слой
    # и выбор направления видят всё изображение. А вот ЗАПИСЬ В КАРТУ —
    # консервативная: occupied-ячейка (conf 0.7) сразу блокирует A*, поэтому
    # дальние шумные точки в карту не пускаем.
    CAMERA_MAX_RANGE_M = 6.0       # анализ: реактив + выбор направления
    CAMERA_MAP_MAX_RANGE_M = 3.0   # запись в occupancy grid
    # Полуширина «коридора движения» для камеры: полкорпуса + запас. Точки в
    # этой полосе считаются преградой по фронту НЕЗАВИСИМО от пеленга — ловит
    # угловые препятствия «под колесо» и стены при диагональном подъезде.
    CAMERA_CORRIDOR_HALFWIDTH_M = 0.125  # ≈ ROBOT_WIDTH/2 + 4 см запаса
    # --- Реактивный слой камеры (depth → объезд препятствий) ---
    # Камера-глубина видит низкие препятствия, которые 2D-лидар на своей высоте
    # пропускает. ВКЛЮЧЕНО консервативно: камера может только сделать поведение
    # осторожнее (фронт — min с лидаром), но не объявляет «свободно» в обход
    # лидара. Монокулярная глубина шумит и «плавает» по масштабу — если в поле
    # пойдут ложные стопы, выставить CAMERA_REACTIVE_ENABLED=False без правки кода.
    CAMERA_REACTIVE_ENABLED = True
    CAMERA_REACTIVE_MIN_RANGE_M = 0.12  # шумовой порог: ближе игнорируем (артефакты)
    # --- Отладка камеры: сохранение кадров с маской нейронки ---
    # При включении раз в CAMERA_DEBUG_PERIOD_S в results/<запуск>/depth/ пишется
    # JPEG: слева исходный кадр, справа depth-цветом с красной маской препятствий
    # (те пиксели, что реально пошли в карту/реактив) и секторными дистанциями.
    # Включать на время отладки: каждый кадр — лишние ~50-100 мс на Pi.
    CAMERA_DEBUG_SAVE_ENABLED = False
    CAMERA_DEBUG_PERIOD_S = 3.0
    CAMERA_DEBUG_KEEP = 200        # сколько последних кадров хранить
    # Минимальный период между инференсами глубины (сек) — темп фонового
    # потока _DepthWorker. Инференс на Pi занимает 0.3–3 с; контур управления
    # он не блокирует (поток), но CPU делит с остальными — 0.7 с не даёт
    # нейронке съесть все ядра.
    CAMERA_MIN_PERIOD_S = 0.7
    # Путь к модели абсолютный — относительный (CWD-зависимый) сломается,
    # если запускать demo_with_lidar.py из map_scripts/, из других папок и т.п.
    DEPTH_MODEL_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'depth_anything_v2_small.onnx',
    )

    # ===== Слияние сегментации пола и глубины (seg ∩ depth) =====
    # Главная ценность камеры: ловить то, что 2D-лидар по высоте пропускает
    # (ножки стола/стула, низкие/высокие препятствия вне плоскости луча).
    # Контракт — camera_perception/floor_fusion.py: препятствие = НЕ floor (seg)
    # И приподнято над плоскостью пола (весь кадр); объект в цвет пола ловится
    # strong-клаузой. Высота над ПЛОСКОСТЬЮ (подгонка по floor-пикселям), а не над
    # mount-калибровкой — не зависит от наклона камеры и масштаба глубины.
    #
    # БЕЗОПАСНОСТЬ: на быструю реактивную остановку НЕ влияет — та идёт по
    # лидару/УЗ и страховочному depth-каналу nearest_in_depth_band (он БЕЗ
    # сегментации, на такте глубины). Сегментация крутится в ОТДЕЛЬНОМ медленном
    # потоке (_SegWorker), маска кэшируется; слияние на свежей глубине + последней
    # маске. Нет маски / не подогналась плоскость → ОТКАТ на legacy height-filter
    # (см. slam_core.update_with_camera_depth), никогда не «нет препятствий».
    CAMERA_FLOOR_FUSION_ENABLED = True
    CAMERA_FLOOR_FUSION_PERIOD_S = 0.8    # темп seg-потока. Близко к темпу глубины
                                          # (0.7 с) — обе сети делят 4 ядра Pi:
                                          # СЛЕДИТЬ за FPS глубины на первом заезде,
                                          # при просадке вернуть к 1.0–1.5 с.
    CAMERA_FLOOR_MASK_MAX_AGE_S = 3.0     # маска старше — считаем протухшей → fallback
    CAMERA_STRONG_OBSTACLE_HEIGHT_M = 0.30  # выше — препятствие даже если seg сказал «пол»
    # Верхний порог высоты препятствия (ceiling-guard вместо «нижней половины»):
    # выше — потолок/верх стен, низкому роботу не угроза. Прокидывается явно в
    # update_with_camera_depth (и в fusion, и в legacy-fallback) — без него slam
    # брал бы свой дефолт 0.50, а контракт проверялся при 0.60. Держать в синхроне
    # с тем, на чём гоняли tests/eval.
    CAMERA_MAX_OBSTACLE_HEIGHT_M = 0.60

    # ===== Каталог результатов: всё пишется в results/<запуск>/ =====
    # Один запуск контроллера = одна подпапка по времени старта, внутри:
    #   maps/         — карты .pkl + .png (см. save_map),
    #   depth/        — отладочные кадры depth-восприятия,
    #   segmentation/ — сырые захваты (cap_*) и overlay сегментации (floor_*).
    # Конкретные пути на запуск считаются в __init__ (self.run_dir/maps_dir/...);
    # здесь только корень. Каталоги создаются лениво, при первой записи.
    RESULTS_DIR = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results',
    )

    # Запись сырых данных сенсоров (lidar raw+scan, УЗ, команды (v,ω,dt), поза,
    # отметки камеры) в results/<запуск>/sensors/ для офлайн-реплея и будущих
    # тестов. ВКЛЮЧЕНА по умолчанию; пишет в фоне (map/sensor_recorder.py), на
    # контур управления не влияет. Сырые байты лидара — для тестов парсера.
    SENSOR_RECORD_ENABLED = True

    # ===== Сегментация пола (SegFormer-B0) — ОТДЕЛЬНЫЙ канал =====
    # Вторая нейросеть поверх Depth-Anything. На АЛГОРИТМ навигации не влияет: в
    # карту и реактивный слой ничего не пишет (это будущий этап). Сейчас лишь
    # копит сырые кадры для офлайн-разбора. Захваты/overlay пишутся в
    # results/<запуск>/segmentation/ — отдельно от depth-отладки (results/.../depth/).
    # Основной сценарий — офлайн: снять кадры заездом, прогнать на дев-машине
    # scripts/eval_floor_segmentation.py.
    #
    #  FLOOR_SEG_CAPTURE_RAW — дёшево писать СЫРЫЕ кадры (RGB[+глубина]) в
    #    results/<запуск>/segmentation/ для офлайн-оценки; модель на роботе НЕ нужна.
    #    ВКЛЮЧЕНО уже сейчас — данные копятся с каждого заезда (на навигацию не
    #    влияет: только запись файла раз в FLOOR_SEG_PERIOD_S секунд).
    #  FLOOR_SEG_ENABLED — гонять SegFormer + калибровку прямо на роботе (тяжело,
    #    уполовинит FPS depth; нужен models/segformer_b0_ade.onnx). В карту/реактив
    #    НЕ пишет — только overlay в ту же segmentation/ (запись в карту — отдельный
    #    будущий этап: нужен правильный depth-based контракт; старый наивный метод
    #    update_with_camera_segmentation был баговый и удалён).
    FLOOR_SEG_CAPTURE_RAW = True
    FLOOR_SEG_CAPTURE_DEPTH = True   # рядом с RGB класть глубину (.npy float16) —
                                     # она уже посчитана depth-потоком, офлайн не пересчитывать
    FLOOR_SEG_ENABLED = False
    FLOOR_SEG_PERIOD_S = 2.0         # как часто захватывать/сегментировать, сек
    FLOOR_SEG_KEEP = 1000            # хранить столько последних кадров (данных лучше много)
    FLOOR_SEG_MODEL_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'segformer_b0_ade.onnx',
    )

    def _init_camera_servos(self):
        """
        Привести серво-кронштейн камеры в известное положение (pan/tilt).
        Безопасно без железа: на дев-машине bot отсутствует — просто выходим.
        """
        if self.bot is None:
            return
        try:
            self.bot.Ctrl_Servo(self.CAMERA_SERVO_PAN_ID, self.CAMERA_SERVO_PAN_DEG)
            self.bot.Ctrl_Servo(self.CAMERA_SERVO_TILT_ID, self.CAMERA_SERVO_TILT_DEG)
            # Даём серво доехать, чтобы первые кадры не снимались «в движении»
            time.sleep(0.5)
            print(f"[NavController] Серво камеры: pan={self.CAMERA_SERVO_PAN_DEG}°, "
                  f"tilt={self.CAMERA_SERVO_TILT_DEG}°")
        except Exception as e:
            print(f"[NavController] Не удалось выставить серво камеры: {e}")

    def _init_depth_perception(self):
        """Инициализация камеры + модели глубины при первом обращении."""
        from camera_perception import (DepthEstimator, CameraIntrinsics,
                                        CameraMount)
        # Сначала ставим голову в известное положение — иначе tilt/pan кадра
        # не соответствуют CameraMount и проекция глубины уезжает.
        self._init_camera_servos()
        try:
            import cv2  # type: ignore
            cap = cv2.VideoCapture(self.CAMERA_DEVICE_ID)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)
            # Минимальный буфер кадров: read() должен отдавать СВЕЖИЙ кадр,
            # а не залежавшийся в очереди V4L2 (иначе облако препятствий
            # отстаёт от реальности на глубину буфера).
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            if cap.isOpened():
                self.camera_cap = cap
                print(f"[NavController] Камера открыта: "
                      f"{self.CAMERA_WIDTH}×{self.CAMERA_HEIGHT}")
            else:
                self.camera_cap = None
                print("[NavController] Камера не открылась — perception отключён")
        except Exception as e:
            self.camera_cap = None
            print(f"[NavController] Ошибка камеры: {e}")

        self.depth_estimator = DepthEstimator(model_path=self.DEPTH_MODEL_PATH)
        self.camera_intrinsics = CameraIntrinsics.from_fov(
            self.CAMERA_WIDTH, self.CAMERA_HEIGHT, hfov_deg=self.CAMERA_HFOV_DEG)
        self.camera_mount = CameraMount(
            height_m=self.CAMERA_MOUNT_HEIGHT_M,
            forward_offset_m=self.CAMERA_MOUNT_FORWARD_M,
            tilt_rad=self.CAMERA_MOUNT_TILT_RAD,
        )
        self._last_camera_debug_time = 0.0

        # Фоновый инференс: контур управления больше не блокируется на
        # нейронке (см. _DepthWorker). Без камеры поток не нужен.
        self._depth_worker = None
        self._seg_worker = None
        if self.camera_cap is not None:
            self._depth_worker = _DepthWorker(
                self.camera_cap, self.depth_estimator,
                get_pose=lambda: (self.slam.current_position.x,
                                  self.slam.current_position.y,
                                  self.slam.current_position.theta),
                period_s=self.CAMERA_MIN_PERIOD_S,
            )
            self._depth_worker.start()
            print("[NavController] Поток инференса глубины запущен")

            # Слияние seg+depth: отдельный медленный поток сегментации пола.
            # Берёт кадры из depth-потока (peek_frame), не открывает камеру
            # повторно. Если модель не загрузилась — поток не стартуем, слияние
            # тихо откатится на legacy height-filter.
            if self.CAMERA_FLOOR_FUSION_ENABLED:
                try:
                    from camera_segmentation import FloorSegmenter
                    segmenter = FloorSegmenter(model_path=self.FLOOR_SEG_MODEL_PATH)
                    if segmenter.is_ready():
                        self._seg_worker = _SegWorker(
                            self._depth_worker.peek_frame, segmenter,
                            period_s=self.CAMERA_FLOOR_FUSION_PERIOD_S,
                        )
                        self._seg_worker.start()
                        print("[NavController] Поток сегментации пола (слияние seg+depth) запущен")
                    else:
                        print("[NavController] SegFormer не загрузился — слияние выкл, "
                              "depth работает по legacy height-filter")
                except Exception as e:
                    print(f"[NavController] Слияние seg+depth недоступно ({e}); "
                          "depth по legacy height-filter")

    def _save_camera_debug_frame(self, frame, depth, pose, cam_dists):
        """
        Сохранить отладочный кадр (RGB | depth + маска препятствий) в
        results/<запуск>/depth/, не чаще CAMERA_DEBUG_PERIOD_S. Старые кадры сверх
        CAMERA_DEBUG_KEEP удаляются, чтобы не забить SD-карту.
        """
        now = time.time()
        if now - self._last_camera_debug_time < self.CAMERA_DEBUG_PERIOD_S:
            return
        self._last_camera_debug_time = now

        import cv2  # на роботе уже есть (используется для камеры)
        from camera_perception import render_depth_debug

        img = render_depth_debug(
            frame, depth, self.camera_intrinsics, self.camera_mount,
            robot_pose=pose,
            sector_distances=cam_dists,
            max_depth_m=self.CAMERA_MAX_RANGE_M,
            max_vis_depth_m=self.CAMERA_MAX_RANGE_M,
        )
        if img is None:
            return

        os.makedirs(self.depth_debug_dir, exist_ok=True)
        fname = time.strftime('debug_%Y%m%d_%H%M%S') + f"_{int(now * 1000) % 1000:03d}.jpg"
        cv2.imwrite(os.path.join(self.depth_debug_dir, fname), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Ротация: держим только последние CAMERA_DEBUG_KEEP кадров
        try:
            files = sorted(f for f in os.listdir(self.depth_debug_dir)
                           if f.startswith('debug_') and f.endswith('.jpg'))
            for old in files[:-self.CAMERA_DEBUG_KEEP]:
                os.remove(os.path.join(self.depth_debug_dir, old))
        except OSError:
            pass

    def _rotate_dir(self, directory: str, prefix: str, keep: int):
        """Оставить в каталоге только `keep` последних файлов с данным префиксом."""
        try:
            files = sorted(f for f in os.listdir(directory) if f.startswith(prefix))
            for old in files[:-keep]:
                try:
                    os.remove(os.path.join(directory, old))
                except OSError:
                    pass
        except OSError:
            pass

    def _init_image_logging(self):
        """
        Поднять фоновый логгер кадров камеры (results/<запуск>/segmentation/).

        Каталог создаётся и ПРОВЕРЯЕТСЯ на запись прямо здесь, на старте: если
        путь недоступен (права/носитель/только-чтение), это печатается сразу
        громкой строкой — раньше ошибка записи терялась в _log_sensor_error
        внутри цикла, и пользователь видел «кадры не пишутся» без причины.

        Запись кадров декуплена от инференса глубины (см. _maybe_log_camera_frame):
        кадр сохраняется, даже если модель глубины недоступна или медленная.
        """
        self._image_logger = None
        self._last_floor_seg_time = 0.0
        if not self.FLOOR_SEG_CAPTURE_RAW:
            return
        print(f"[NavController] Лог кадров камеры → {self.seg_dir}")
        try:
            os.makedirs(self.seg_dir, exist_ok=True)
            probe = os.path.join(self.seg_dir, '.write_test')
            with open(probe, 'w') as f:
                f.write('ok')
            os.remove(probe)
        except OSError as e:
            print(f"[NavController] !!! Каталог кадров НЕдоступен для записи: "
                  f"{type(e).__name__}: {e} — кадры писаться НЕ будут")
            return
        self._image_logger = _ImageLogger(keep=self.FLOOR_SEG_KEEP)
        self._image_logger.start()

    def _maybe_log_camera_frame(self, worker):
        """
        Раз в FLOOR_SEG_PERIOD_S положить последний кадр камеры (+глубину, если
        есть) в очередь фонового логгера. От инференса глубины НЕ зависит — кадр
        читает поток камеры (_DepthWorker), главный цикл лишь забирает снимок и
        отдаёт логгеру. Диск тут не трогается → контур управления не морозится.
        """
        logger = getattr(self, '_image_logger', None)
        if logger is None or worker is None:
            return
        now = time.time()
        if now - getattr(self, '_last_floor_seg_time', 0.0) < self.FLOOR_SEG_PERIOD_S:
            return
        snap = worker.peek_frame()
        if snap is None:
            return
        frame, _pose, depth, _ts = snap
        self._last_floor_seg_time = now
        base = 'cap_' + time.strftime('%Y%m%d_%H%M%S') + f"_{int(now * 1000) % 1000:03d}"
        # float16: глубину уже посчитал depth-поток — офлайн не пересчитываем.
        depth_to_save = depth if (self.FLOOR_SEG_CAPTURE_DEPTH and depth is not None) else None
        logger.submit(self.seg_dir, base, frame, depth_to_save)
        # Лёгкая отметка кадра в единой шкале сенсоров (сам кадр — в segmentation/).
        self.recorder.record_camera(
            now, base + '.jpg', (base + '.npy') if depth_to_save is not None else None)

    def _maybe_floor_segmentation(self, frame, depth):
        """
        Сегментация пола (SegFormer) — ОТДЕЛЬНЫЙ канал, на алгоритм навигации не
        влияет. Карту и реактивный слой НЕ трогает. По умолчанию выключен
        (FLOOR_SEG_ENABLED=False → мгновенный выход).

        Сырой захват кадров (FLOOR_SEG_CAPTURE_RAW) переехал в фоновый логгер
        (_maybe_log_camera_frame / _ImageLogger) — он не блокирует контур и не
        зависит от наличия глубины. Здесь остался только тяжёлый боевой инференс.
        """
        if not self.FLOOR_SEG_ENABLED:
            return
        now = time.time()
        if now - getattr(self, '_last_floor_seg_run', 0.0) < self.FLOOR_SEG_PERIOD_S:
            return
        self._last_floor_seg_run = now

        import cv2  # на роботе есть (используется камерой)
        ts = time.strftime('%Y%m%d_%H%M%S') + f"_{int(now * 1000) % 1000:03d}"

        # Боевой инференс на роботе (тяжело) — только overlay в лог, в карту НЕ пишем.
        if self.FLOOR_SEG_ENABLED:
            try:
                if not hasattr(self, '_floor_segmenter'):
                    from camera_segmentation import FloorSegmenter
                    self._floor_segmenter = FloorSegmenter(
                        model_path=self.FLOOR_SEG_MODEL_PATH)
                mask = self._floor_segmenter.segment_floor(frame)
                if mask is not None:
                    from camera_segmentation import (estimate_floor_plane,
                                                     render_floor_debug)
                    cal = estimate_floor_plane(depth, mask, self.camera_intrinsics,
                                               pixel_stride=self.CAMERA_PIXEL_STRIDE)
                    if cal is not None and cal.ok:
                        print(f"[FloorSeg] tilt={cal.tilt_deg:+.1f}° "
                              f"h={cal.height_m * 100:.1f}см inlier={cal.inlier_ratio:.2f}")
                    os.makedirs(self.seg_dir, exist_ok=True)
                    overlay = render_floor_debug(frame, mask, cal)
                    if overlay is not None:
                        cv2.imwrite(os.path.join(self.seg_dir, f'floor_{ts}.jpg'),
                                    overlay, [cv2.IMWRITE_JPEG_QUALITY, 88])
                        self._rotate_dir(self.seg_dir, 'floor_',
                                         self.FLOOR_SEG_KEEP)
            except Exception as e:
                self._log_sensor_error('floor_seg', e)

    def _init_sensors(self):
        """Инициализация датчиков"""
        if self.use_ultrasonic and self.bot is not None:
            try:
                self.bot.Ctrl_Ulatist_Switch(1)
                time.sleep(0.1)
                print("[NavController] Ультразвуковой датчик включен")
            except Exception as e:
                print(f"[NavController] Ошибка инициализации ультразвука: {e}")

    def _init_lidar(self):
      """Инициализация лидара"""
      try:
          sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
          from lidar import LidarDriver

          # Перебираем ВСЕ реально существующие serial-порты, а не жёсткий
          # список: иначе можно пропустить порт лидара (был случай — лидар на
          # ttyUSB2, а список заканчивался на ttyUSB1). /dev/oradar (symlink из
          # udev) пробуем первым, затем все ttyACM* и ttyUSB* (динамически), и в
          # конце аппаратные UART.
          import glob
          ports = (['/dev/oradar']
                   + sorted(glob.glob('/dev/ttyACM*'))
                   + sorted(glob.glob('/dev/ttyUSB*'))
                   + ['/dev/ttyAMA0', '/dev/ttyS0'])
          # без дубликатов, сохраняя порядок
          seen = set()
          ports = [p for p in ports if not (p in seen or seen.add(p))]

          # Сколько ждать реальных фреймов лидара после команды старта. Мотору
          # нужно время раскрутиться (~10 об/с), поэтому ждём щедро — иначе
          # отвергнем настоящий лидар как «нет данных».
          FRAME_WAIT_S = 3.0
          # Перебираем и скорости: на этом роботе адаптер лидара — CP210x, и
          # поток сплошных 0x00 на 230400 означал, что реальная скорость другая.
          # 230400 — штатная для YDLidar T-mini Plus, остальные — на случай иной прошивки.
          BAUDS = [230400, 460800, 921600, 256000, 115200]

          for port in ports:
            for baud in BAUDS:
              try:
                  print(f"[NavController] Пробую лидар на {port} @ {baud}...")
                  self.lidar = LidarDriver(port=port, baudrate=baud)

                  if not self.lidar.connect():
                      self.lidar = None
                      break  # порт не открылся — другие скорости не помогут

                  self.lidar.start_scan()

                  # Проверяем, что это действительно лидар: ждём валидные фреймы.
                  deadline = time.time() + FRAME_WAIT_S
                  while time.time() < deadline and self.lidar.packet_count == 0:
                      time.sleep(0.1)

                  if self.lidar.packet_count > 0:
                      print(f"[NavController] Лидар подключен на {port} @ {baud} "
                            f"(принято фреймов: {self.lidar.packet_count})")
                      # Отвод сырых байтов в рекордер (для тестов парсера)
                      self.lidar.set_raw_sink(self.recorder.record_lidar_raw)
                      return

                  print(f"[NavController] {port} @ {baud}: валидных фреймов лидара нет")
                  self.lidar.disconnect()
                  self.lidar = None
              except Exception as e:
                  print(f"[NavController] Ошибка на {port} @ {baud}: {e}")
                  if self.lidar is not None:
                      try:
                          self.lidar.disconnect()
                      except Exception:
                          pass
                  self.lidar = None

          print("[NavController] Не удалось подключиться к лидару")
          print("[NavController] Работа продолжится без лидара")
          self.use_lidar = False
  
      except ImportError as e:
          print(f"[NavController] Ошибка импорта драйвера лидара: {e}")
          print("[NavController] Работа продолжится без лидара")
          self.use_lidar = False
      except Exception as e:
          print(f"[NavController] Неожиданная ошибка при инициализации лидара: {e}")
          print("[NavController] Работа продолжится без лидара")
          self.use_lidar = False

    def set_goal(self, x: float, y: float):
        """
        Установка целевой точки

        Args:
            x, y: координаты цели в метрах
        """
        print(f"\n[NavController] Новая цель: ({x:.2f}, {y:.2f})")
        self.current_goal = (x, y)

        # Планируем глобальный путь
        current_pose = self.slam.get_pose()
        path = self.path_planner.plan_global_path(
            (current_pose.x, current_pose.y),
            self.current_goal
        )

        if path:
            self.current_path = path
            self.current_waypoint_idx = 0
            self._dwa_vel = (0.0, 0.0)
            self._dwa_stall_count = 0
            self._dwa_replan_count = 0
            self._avoid_count = 0
            self.mode = NavigationMode.NAVIGATION
            print(f"[NavController] Путь построен: {len(path)} точек")
        else:
            self._goal_unreachable("путь не построен")

    def _goal_unreachable(self, reason: str):
        """
        Терминальный исход «до цели не доехать». В localization блуждание
        бессмысленно: карта read-only, проход в ней не «откроется», а demo
        ждёт IDLE — раньше робот в этой ситуации навсегда уходил в
        EXPLORATION и скрипт не завершался. В mapping исследование оставляем:
        карта пополняется, проход может найтись.
        """
        if self.mapping_mode == 'localization':
            print(f"[NavController] ⛔ ЦЕЛЬ НЕДОСТИЖИМА ({reason}) — останавливаюсь")
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            self.current_goal = None
            self.current_path = []
            self.mode = NavigationMode.IDLE
        else:
            print(f"[NavController] Цель пока недостижима ({reason}) — исследую")
            self.mode = NavigationMode.EXPLORATION

    def start_exploration(self):
        """Запуск режима исследования среды"""
        print("\n[NavController] Запуск режима анализа среды")
        self.mode = NavigationMode.EXPLORATION

    def update(self):
        """
        Основной цикл обновления
        Должен вызываться регулярно
        """
        # Сенсоры + одометрия одной точкой входа (см. _tick)
        self._tick()

        # Выполняем действия в зависимости от режима
        if self.mode == NavigationMode.EXPLORATION:
            self._safe_exploration_behavior()
        elif self.mode == NavigationMode.NAVIGATION:
            self._safe_navigation_behavior()
        elif self.mode == NavigationMode.OBSTACLE_AVOIDANCE:
            self._safe_avoidance_behavior()

    # Минимум секунд между повторными логами одной и той же ошибки сенсора.
    # _update_sensors зовётся 20Hz, без throttle лог будет залит однотипными
    # исключениями. С throttle ошибка видна, но не топит полезные сообщения.
    _SENSOR_ERROR_LOG_INTERVAL_S = 2.0

    def _log_sensor_error(self, sensor: str, exc: Exception) -> None:
        """Залогировать ошибку сенсора с throttle (не чаще раза в N секунд)."""
        if not hasattr(self, '_last_sensor_error'):
            self._last_sensor_error = {}
        now = time.time()
        last = self._last_sensor_error.get(sensor, 0.0)
        if now - last >= self._SENSOR_ERROR_LOG_INTERVAL_S:
            self._last_sensor_error[sensor] = now
            print(f"[NavController] Ошибка сенсора '{sensor}': "
                  f"{type(exc).__name__}: {exc}")

    def _update_sensors(self):
        """Обновление данных сенсоров.

        Инференс глубины крутится в ФОНОВОМ потоке (_DepthWorker) — здесь
        только неблокирующее потребление готового результата. Контур
        управления всегда остаётся на частоте лидара/ультразвука, без
        замираний на нейронке и слепых окон при движении.
        """

        # 1) Камера: забрать готовый результат фонового инференса (если есть).
        if self.use_camera:
            try:
                # Инициализация при первом вызове: cv2.VideoCapture +
                # ONNX-модель + фоновый поток.
                if not hasattr(self, 'depth_estimator'):
                    self._init_depth_perception()

                worker = self._depth_worker
                result = (worker.take_latest(self._depth_consumed_ts)
                          if worker is not None else None)
                if result is not None:
                    frame, depth, pose, ts = result
                    self._depth_consumed_ts = ts
                    # Слияние seg+depth: последняя маска пола из медленного
                    # seg-потока (None если поток выкл / маска протухла / модель
                    # недоступна → update_with_camera_depth откатится на legacy
                    # height-filter). Это НЕ на критическом пути быстрой остановки.
                    floor_mask = (self._seg_worker.latest_mask(
                                      self.CAMERA_FLOOR_MASK_MAX_AGE_S)
                                  if self._seg_worker is not None else None)
                    # update_with_camera_depth пишет в карту (только в
                    # mapping, и только до CAMERA_MAP_MAX_RANGE_M) и
                    # ВОЗВРАЩАЕТ облако препятствий на всю дальность анализа —
                    # доступно и в localization.
                    # pose — поза НА МОМЕНТ ЗАХВАТА кадра (см. _DepthWorker).
                    obstacles = self.slam.update_with_camera_depth(
                        depth, self.camera_intrinsics, self.camera_mount,
                        pixel_stride=self.CAMERA_PIXEL_STRIDE,
                        max_range_m=self.CAMERA_MAX_RANGE_M,
                        map_write_max_range_m=self.CAMERA_MAP_MAX_RANGE_M,
                        robot_pose=pose,
                        floor_mask=floor_mask,
                        max_obstacle_height_m=self.CAMERA_MAX_OBSTACLE_HEIGHT_M,
                        strong_obstacle_height_m=self.CAMERA_STRONG_OBSTACLE_HEIGHT_M,
                    )
                    # Реактивный слой: секторные расстояния + коридор
                    # движения из того же облака → в sensor_fusion.
                    cam_dists = None
                    if self.CAMERA_REACTIVE_ENABLED:
                        from camera_perception import (
                            nearest_in_depth_band,
                            obstacle_distances_by_sector)
                        cam_dists = obstacle_distances_by_sector(
                            obstacles, pose,
                            min_range_m=self.CAMERA_REACTIVE_MIN_RANGE_M,
                            max_range_m=self.CAMERA_MAX_RANGE_M,
                            corridor_halfwidth_m=self.CAMERA_CORRIDOR_HALFWIDTH_M,
                        )
                        # Страховочный канал «ближайший объект по курсу»
                        # прямо из глубины (полоса у горизонта, без фильтра
                        # высот) — ловит сетки/прозрачное/редкие облака,
                        # где секторный канал молчит.
                        band = nearest_in_depth_band(
                            depth, self.camera_intrinsics,
                            self.camera_mount,
                            max_depth_m=self.CAMERA_MAX_RANGE_M,
                        )
                        cam_dists['front_band'] = band
                        if band is not None and (
                                cam_dists['front'] is None
                                or band < cam_dists['front']):
                            cam_dists['front'] = band
                        self.sensor_fusion.update_camera(cam_dists)
                    # Отладочные кадры: что увидела нейронка и что из
                    # этого стало препятствиями.
                    if self.CAMERA_DEBUG_SAVE_ENABLED:
                        self._save_camera_debug_frame(
                            frame, depth, pose, cam_dists)
                    # Сегментация пола (SegFormer) — отдельный канал, по
                    # умолчанию выключен; на навигацию не влияет.
                    self._maybe_floor_segmentation(frame, depth)
                # Периодический лог кадров камеры — НЕ зависит от того, дал ли
                # инференс глубину в этом такте: пишем последний прочитанный
                # кадр (см. _maybe_log_camera_frame). Запись — в фоне.
                self._maybe_log_camera_frame(worker)
            except Exception as e:
                self._log_sensor_error('camera', e)

        # 2) Лидар (чтение из буфера — дёшево).
        if self.use_lidar and self.lidar is not None:
            try:
                lidar_scan = self.lidar.get_scan()
                if lidar_scan and len(lidar_scan) > 0:
                    # В слияние — СЫРОЙ скан: под-12см точки нужны детектору
                    # «прижатой стены» (sensor_fusion сам их интерпретирует)
                    self.sensor_fusion.update_lidar(lidar_scan)

                    # В SLAM: только НОВЫЙ оборот (иначе один скан пишется в
                    # карту 2-3 раза до следующего оборота — насыщает log-odds
                    # шумом и жжёт CPU) и только при умеренном вращении
                    # (точки оборота копятся ~150 мс и при ω>0.6 рад/с
                    # размазываются на 5-15° — мажут карту и матчер).
                    rev = getattr(self.lidar, 'revolution_count', None)
                    is_new_rev = rev is None or rev != getattr(self, '_last_lidar_rev', None)
                    # Рекордер: парсенный скан на КАЖДЫЙ новый оборот (полнее
                    # гейта SLAM ниже); сырые байты пишутся отдельно из потока лидара.
                    if rev is not None and rev != getattr(self, '_last_recorded_rev', None):
                        self._last_recorded_rev = rev
                        self.recorder.record_lidar_scan(time.time(), rev, lidar_scan)
                    _, omega_now = self._commanded_velocity()
                    if is_new_rev and abs(omega_now) <= 0.6:
                        filtered_scan = [(a, d) for a, d in lidar_scan if d >= 0.12]
                        if filtered_scan:
                            self._last_lidar_rev = rev
                            self.slam.update_with_lidar(filtered_scan)
            except Exception as e:
                self._log_sensor_error('lidar', e)

        # 3) Ультразвук — ПОСЛЕДНИМ, чтобы быть максимально свежим к моменту,
        #    когда поведение тут же вызовет get_obstacle_distance('front').
        if self.use_ultrasonic:
            try:
                from ultrasonic import read_distance_cm_from_bot
                distance_cm = read_distance_cm_from_bot()
                if distance_cm is not None and distance_cm > 0:
                    distance_m = distance_cm / 100.0
                    self.sensor_fusion.update_ultrasonic(distance_m)
                    self.recorder.record_ultrasonic(time.time(), distance_m)
                    # В карту — не чаще 2 Гц: конус из 7 лучей каждым тиком
                    # насыщал дуги тем же показанием
                    now_us = time.time()
                    if now_us - getattr(self, '_last_us_map_write', 0.0) >= 0.5:
                        self._last_us_map_write = now_us
                        self.slam.update_with_ultrasonic(distance_m)
            except Exception as e:
                self._log_sensor_error('ultrasonic', e)

    # ===== Физические параметры платформы (Yahboom Raspbot V2, замеры) =====
    # Габариты и масса — реальные замеры тележки; моторы — паспорт TT-моторов.
    ROBOT_WIDTH = 0.16583       # полная ширина по колёсам, м
    TRACK_WIDTH = 0.140         # колея: между центрами левого и правого колеса, м (замер)
    WHEEL_BASE_LONG = 0.1172    # межосевое расстояние (центры передних/задних колёс), м
    ROBOT_LENGTH = 0.20874      # от носа камеры до заднего края колеса, м
    ROBOT_MASS_KG = 1.0         # справочно: в одометрии напрямую не используется
    MOTOR_TORQUE_NM = 0.8       # справочно: момент мотора
    MOTOR_RPM_NO_LOAD = 245.0   # об/мин на валу БЕЗ нагрузки, ±10%
    # Диаметр колеса — ЗАМЕРИТЬ штангенциркулем (см. ROBOT_CHECKLIST.md).
    # 0.060 — типовое для мекалум-колёс Raspbot. Через него паспортные обороты
    # переводятся в скорость: v_no_load = π·D·rpm/60 ≈ π·0.060·245/60 ≈ 0.77 м/с.
    WHEEL_DIAMETER = 0.060

    # ===== Калибровка одометрии =====
    # Энкодеров/PID нет — скорость оценивается из PWM. Паспорт даёт ВЕРХНЮЮ
    # оценку (без нагрузки); под весом ~1 кг и на реальном полу фактическая
    # скорость ниже (~0.75–0.85 от паспортной). Финальные значения даёт
    # map_scripts/calibrate_odometry.py — замер главнее паспорта.
    PWM_FULL = 255.0
    # Паспортный потолок 0.77 м/с × коэффициент нагрузки ~0.78 ≈ 0.6.
    MAX_LINEAR_SPEED = 0.6     # м/с при PWM=255 (уточнить калибровкой)
    # Поворот на месте: omega = 2·v_колеса / (колея + межосевое) — мекалум-
    # кинематика. Потолок: 2·0.77 / 0.257 ≈ 6.0 рад/с; с нагрузкой и
    # проскальзыванием при развороте реально ниже: ~6.0·0.78 ≈ 4.7.
    MAX_ANGULAR_SPEED = 4.7    # рад/с при PWM=255 (уточнить калибровкой)
    # Эффективная база диф-модели (v,omega)↔(left,right) для мекалум-шасси:
    # из кинематики поворота на месте это СУММА колеи и межосевого, 2·(lx+ly).
    # Для самой одометрии критична только СОГЛАСОВАННОСТЬ этого значения между
    # контроллером и SLAM (конвертация туда-обратно).
    WHEEL_BASE = TRACK_WIDTH + WHEEL_BASE_LONG   # ≈ 0.257 м
    # Радиус робота для inflation cost-map — из замеров: задний свес от центра
    # вращения ≈ межосевое/2 + R_колеса ≈ 0.089 м, нос камеры ≈ 0.209−0.089 ≈
    # ≈ 0.12 м вперёд, полуширина 0.083 м → худшая точка ≈ 0.13 м.
    # На защите легко показать: чем толще красный пояс на PNG карты, тем безопаснее
    # построенный путь, но и больше шанс "узких" коридоров стать непроходимыми.
    ROBOT_RADIUS = 0.13
    SAFETY_MARGIN = 0.08

    def _update_odometry(self, dt: float):
        """
        Обновление одометрии на основе текущей команды движения.
        Должен вызываться часто во время движения (см. _tick).
        """
        if dt <= 0.0:
            return

        v, omega = self._commanded_velocity()

        # Конвертация (v, omega) → скорости бортов для diff-drive модели в SLAM.
        # SLAM сам пересчитает обратно: v=(l+r)/2, omega=(r-l)/wheel_base.
        half_base = self.WHEEL_BASE / 2.0
        left_speed = v - omega * half_base
        right_speed = v + omega * half_base

        self.slam.update_odometry(left_speed, right_speed, dt)

    def _commanded_velocity(self) -> Tuple[float, float]:
        """
        Модель актуатора: (v, omega), которые робот исполняет ПРЯМО СЕЙЧАС
        по активной команде. Единственное место, где команда переводится в
        скорости, — используется одометрией и гейтами записи в карту.
        """
        # PWM в car_adapter лежит в диапазоне 0..255, current_speed хранится так же.
        pwm_factor = min(1.0, max(0.0, self.current_speed / self.PWM_FULL))
        v_max = self.MAX_LINEAR_SPEED * pwm_factor
        omega_max = self.MAX_ANGULAR_SPEED * pwm_factor

        cmd = self.current_command

        if cmd == "forward":
            v, omega = v_max, 0.0
        elif cmd == "forward_steer":
            # Подруливание дифференциалом бортов (move_param_forward):
            # при ЛЮБОМ знаке p один из бортов УСИЛИВАЕТСЯ на |p|% — средняя
            # скорость всегда растёт (v×(1+|p|/200)); знак p задаёт только
            # сторону (p>0 → правый борт → влево, omega>0). Модель обязана
            # совпадать с тем, что реально делает car_adapter, иначе
            # одометрия поедет мимо scan matcher'а.
            p = self.current_steer
            v = v_max * (1.0 + abs(p) / 200.0)
            omega = (v_max * (p / 100.0)) / self.WHEEL_BASE
        elif cmd == "tank":
            # DWA: борта заданы напрямую — восстанавливаем (v, omega) из PWM
            l_pwm, r_pwm = self.current_tank
            v_l = self.MAX_LINEAR_SPEED * l_pwm / self.PWM_FULL
            v_r = self.MAX_LINEAR_SPEED * r_pwm / self.PWM_FULL
            v = (v_l + v_r) / 2.0
            omega = (v_r - v_l) / self.WHEEL_BASE
        elif cmd == "backward":
            v, omega = -v_max, 0.0
        elif cmd == "rotate_left":
            v, omega = 0.0, omega_max
        elif cmd == "rotate_right":
            v, omega = 0.0, -omega_max
        else:
            # stop, либо боковое мекалум-движение — в одометрии не учитываем
            v, omega = 0.0, 0.0

        return v, omega

    def _tick(self, dt_max: float = 0.5):
        """
        Один шаг обновления: сенсоры + одометрия + SLAM.
        Должен вызываться периодически и из блокирующих помощников движения,
        чтобы одометрия накапливалась пока команда активна.
        """
        now = time.time()
        dt = now - self.last_update_time
        if dt > dt_max:
            dt = dt_max  # защита от больших dt после блокирующих участков
        self.last_update_time = now
        # Сначала одометрия, потом сенсоры: скан снят «сейчас» и должен
        # матчиться против позы, продвинутой на текущий dt, — иначе прайр
        # матчера систематически отстаёт на тик.
        self._update_odometry(dt)
        self._update_sensors()
        # Команда/поза/dt — критично для достоверного реплея: scan matcher
        # использует одометрический прайр между сканами.
        v_cmd, omega_cmd = self._commanded_velocity()
        pos = self.slam.current_position
        self.recorder.record_command(now, v_cmd, omega_cmd, dt)
        self.recorder.record_pose(now, pos.x, pos.y, pos.theta)

    def _set_movement_command(self, command: str, speed: int, steer: float = 0.0,
                              tank=(0, 0)):
        """
        Установка команды движения для отслеживания одометрии

        Args:
            command: тип команды (forward, forward_steer, tank, backward,
                     rotate_left, rotate_right, stop)
            speed: скорость PWM (0-255) — номинал борта ДО усиления steer'ом
            steer: для forward_steer — процент усиления борта, как в
                   car_adapter.move_param_forward (p>0 → правый борт → влево)
            tank: для tank — подписанные PWM бортов (left, right)
        """
        self.current_command = command
        self.current_speed = speed
        self.current_steer = float(steer)
        self.current_tank = tank

    # ===== Параметры плавного движения =====
    SLOWDOWN_START_M = 0.7   # с этой дистанции начинаем плавно сбрасывать PWM
    MIN_MOVE_PWM = 22        # ниже TT-моторы у порога трогания (стиктион)
    CRUISE_STEER_MAX = 20.0  # макс. увод к простору на круизе, % борта

    # ===== DWA: локальный планировщик в режиме NAVIGATION =====
    # Один флаг отката: если в поле DWA ведёт себя странно — False возвращает
    # прежнее дискретное поведение (повороты на месте + отрезки).
    DWA_ENABLED = True
    DWA_MAX_SPEED_M_S = 0.16   # потолок линейной скорости DWA (≈ PWM 68)
    DWA_MAX_YAW_RATE = 1.2     # потолок угловой скорости, рад/с
    # Порог объезда ОБЯЗАН быть выше «полосы замирания» квантования:
    # v_cap = 0.5·(front−0.20) обнуляет команду при front < ~0.30 (порог
    # трогания 22 PWM = 0.052 м/с); при 0.28 робот замирал на 0.30, не
    # доходя до порога объезда, и вечно перепланировал.
    DWA_BLOCK_FRONT_M = 0.32   # реактивный фронт ближе — стоп и режим объезда
    DWA_STALL_LIMIT = 12       # тактов подряд без движения → перепланирование
    DWA_REPLAN_LIMIT = 2       # подряд безрезультатных перепланирований → объезд
    DWA_NO_DATA_TICKS = 5      # тактов без фронт-данных → ползущий кап

    def _send_velocity_command(self, v: float, omega: float,
                               allow_bump: bool = True):
        """
        (v, omega) → PWM бортов → моторы. Возвращает ФАКТИЧЕСКИ исполненную
        пару (v, omega) после квантования мёртвой зоны моторов — её же
        получает одометрия (команда tank) и следующий такт DWA как current_vel.

        Квантование: |PWM| < MIN_MOVE_PWM/2 → 0 (колесо всё равно не тронется),
        иначе < MIN_MOVE_PWM → подтягиваем до порога трогания. Честная модель
        актуатора важнее точного следования идеальной дуге — DWA всё равно
        перепланирует на следующем такте.

        allow_bump=False: подтяжку вверх запрещаем (округляем в 0) — нужно,
        когда реактивный кап скорости НИЖЕ порога трогания: иначе квантование
        пробивало бы кап в 1.5–2 раза у самого препятствия.
        """
        half_base = self.WHEEL_BASE / 2.0
        v_l = v - omega * half_base
        v_r = v + omega * half_base

        def to_pwm(speed_ms: float) -> int:
            pwm = speed_ms / self.MAX_LINEAR_SPEED * self.PWM_FULL
            if abs(pwm) < self.MIN_MOVE_PWM / 2.0:
                return 0
            if abs(pwm) < self.MIN_MOVE_PWM:
                return int(np.sign(pwm) * self.MIN_MOVE_PWM) if allow_bump else 0
            return int(np.clip(round(pwm), -255, 255))

        l_pwm, r_pwm = to_pwm(v_l), to_pwm(v_r)
        if l_pwm == 0 and r_pwm == 0:
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            return 0.0, 0.0

        ca.move_tank(l_pwm, r_pwm)
        self._set_movement_command("tank", max(abs(l_pwm), abs(r_pwm)),
                                   tank=(l_pwm, r_pwm))
        # Фактические скорости после квантования
        v_l_q = self.MAX_LINEAR_SPEED * l_pwm / self.PWM_FULL
        v_r_q = self.MAX_LINEAR_SPEED * r_pwm / self.PWM_FULL
        return (v_l_q + v_r_q) / 2.0, (v_r_q - v_l_q) / self.WHEEL_BASE

    def _apply_forward(self, pwm: int, steer: float):
        """Послать моторную команду «вперёд» (с уводом или без) + одометрия."""
        if abs(steer) < 1.0:
            ca.move_forward(pwm)
            self._set_movement_command("forward", pwm)
        else:
            ca.move_param_forward(pwm, steer)
            self._set_movement_command("forward_steer", pwm, steer=steer)

    def _speed_for_distance(self, base_pwm: int, distance) -> int:
        """
        Плавное замедление: полный ход дальше SLOWDOWN_START_M, линейный
        спад к MIN_MOVE_PWM у min_obstacle_distance. Вместо бинарного
        «едем/стоп» — робот подкатывается к препятствию мягко.
        """
        if distance is None or distance >= self.SLOWDOWN_START_M:
            return base_pwm
        if distance <= self.min_obstacle_distance:
            return self.MIN_MOVE_PWM
        frac = ((distance - self.min_obstacle_distance)
                / (self.SLOWDOWN_START_M - self.min_obstacle_distance))
        return int(round(max(self.MIN_MOVE_PWM, base_pwm * (0.5 + 0.5 * frac))))

    def _steer_to_free_space(self) -> float:
        """
        Мягкий увод к большему простору на круизе (п. «двигаться в свободное
        пространство»). Возвращает процент для move_param_forward:
        p>0 → усиление правого борта → увод ВЛЕВО.

        Уводим только когда ОБЕ стороны измерены и дисбаланс заметный —
        на None не реагируем (отсутствие отражений не повод вилять).
        """
        left = self.sensor_fusion.get_obstacle_distance('left')
        right = self.sensor_fusion.get_obstacle_distance('right')
        if left is None or right is None:
            return 0.0
        total = left + right
        if total <= 0.0:
            return 0.0
        imbalance = (left - right) / total  # >0 — слева свободнее
        if abs(imbalance) < 0.2:
            return 0.0  # почти симметрично — едем прямо
        return float(np.clip(imbalance * 2.0, -1.0, 1.0) * self.CRUISE_STEER_MAX)

    def _safe_move_forward(self, max_duration=1.0, check_interval=0.1,
                           speed=None, stop_at_end=True, steer_to_free=False):
        """
        Безопасное движение вперед с постоянной проверкой препятствий.

        Args:
            max_duration: максимальное время движения (сек)
            check_interval: интервал проверки препятствий (сек)
            speed: базовый PWM (None → exploration_speed)
            stop_at_end: False = «круиз» — если препятствий не было, моторы
                         НЕ останавливаются по истечении времени; следующий
                         вызов продолжит движение бесшовно. При любом
                         препятствии/ошибке стоп происходит всегда.
            steer_to_free: мягко уводить к стороне с большим простором
                           (одометрия учитывает увод через forward_steer)

        Returns:
            bool: True если движение завершено безопасно, False если было препятствие
        """
        start_time = time.time()
        safe = True
        check_count = 0
        no_data_streak = 0

        base_pwm = int(speed if speed is not None else self.exploration_speed)
        current_pwm = base_pwm
        current_steer = 0.0

        print(f"[SAFE_MOVE] Вперёд на {max_duration:.1f}с"
              f"{' (круиз)' if not stop_at_end else ''}")
        self._apply_forward(current_pwm, current_steer)
        # Сбрасываем точку отсчёта одометрии: интегрируем с этого момента
        self.last_update_time = time.time()

        try:
            while time.time() - start_time < max_duration:
                # Тикаем одометрию/SLAM И читаем сенсоры СРАЗУ, до ожидания:
                # объект (нога) мог появиться только что — нельзя ехать вслепую
                # первые check_interval. Ожидание перенесено в КОНЕЦ итерации.
                self._tick()
                check_count += 1

                obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')
                # Если слияние молчит — пробуем ультразвук напрямую
                if obstacle_distance is None and self.use_ultrasonic:
                    try:
                        from ultrasonic import read_distance_cm_from_bot
                        us_cm = read_distance_cm_from_bot()
                        if us_cm is not None and us_cm > 0:
                            obstacle_distance = us_cm / 100.0
                            print(f"[SAFE_MOVE] Ультразвук: {obstacle_distance:.2f}м")
                    except Exception:
                        pass

                if obstacle_distance is None:
                    no_data_streak += 1
                    print(f"[SAFE_MOVE] ⚠️ Проверка {check_count}: Нет данных датчиков")
                    # 3 проверки ПОДРЯД без данных — останавливаемся
                    if no_data_streak >= 3:
                        print("[SAFE_MOVE] Нет данных 3 проверки подряд, останавливаюсь")
                        safe = False
                        break
                else:
                    no_data_streak = 0

                    # КРИТИЧЕСКОЕ расстояние — немедленная остановка
                    if obstacle_distance < self.critical_distance:
                        print(f"[SAFE_MOVE] ⚠️ КРИТИЧЕСКОЕ РАССТОЯНИЕ! {obstacle_distance:.2f}м")
                        safe = False
                        break

                    # Очень близко (меньше половины min_distance) — стоп
                    if obstacle_distance < self.min_obstacle_distance:
                        closeness = (self.min_obstacle_distance - obstacle_distance) / self.min_obstacle_distance
                        if closeness > 0.5:
                            print(f"[SAFE_MOVE] Очень близко: {obstacle_distance:.2f}м")
                            safe = False
                            break

                    # Плавное замедление по дистанции + мягкий увод к простору
                    target_pwm = self._speed_for_distance(base_pwm, obstacle_distance)
                    target_steer = self._steer_to_free_space() if steer_to_free else 0.0
                    if (abs(target_pwm - current_pwm) >= 3
                            or abs(target_steer - current_steer) >= 5.0):
                        current_pwm, current_steer = target_pwm, target_steer
                        self._apply_forward(current_pwm, current_steer)
                        if current_pwm != base_pwm:
                            print(f"[SAFE_MOVE] Замедляюсь: PWM={current_pwm} "
                                  f"(препятствие {obstacle_distance:.2f}м)")
                        if abs(current_steer) >= 5.0:
                            print(f"[SAFE_MOVE] Увод к простору: {current_steer:+.0f}%")

                    # Все OK, продолжаем
                    if check_count % 5 == 0:  # Реже выводим
                        print(f"[SAFE_MOVE] Двигаюсь... препятствие: {obstacle_distance:.2f}м")

                # Ожидание В КОНЦЕ: первая проверка уже прошла без слепого хода.
                time.sleep(check_interval)

        except Exception as e:
            print(f"[SAFE_MOVE] Ошибка при движении: {e}")
            safe = False

        finally:
            if safe and not stop_at_end:
                # Круиз: моторы продолжают крутиться, команда остаётся
                # активной — одометрия копится через _tick() в update().
                print(f"[SAFE_MOVE] ✓ Сегмент пройден, продолжаю движение "
                      f"({check_count} проверок)")
            else:
                ca.stop_robot()
                self._set_movement_command("stop", 0)
                time.sleep(0.1)  # дать тележке осесть после стопа (было 0.3 — лишняя вялость)

        if safe:
            if stop_at_end:
                print(f"[SAFE_MOVE] ✓ Движение завершено безопасно ({check_count} проверок)")
        else:
            print(f"[SAFE_MOVE] ✗ Движение прервано из-за препятствия")

        return safe

    def _safe_rotate(self, direction: str,
                     max_duration: float = 1.0,
                     check_interval: float = 0.2) -> bool:
        """
        Безопасный поворот на месте: крутимся ≤ max_duration, прерываемся если
        что-то подъехало ближе critical_distance впереди. Одометрия тикает
        каждый check_interval, так что угол доезжает до SLAM/scan-matcher'а.

        Args:
            direction: 'left' | 'right'
            max_duration: предел времени поворота, сек
            check_interval: период тика и проверки сенсоров, сек

        Returns:
            True если поворот завершён без срабатывания safety-предела.
        """
        # С появлением круиза сюда можно прийти НА ХОДУ — резкий реверс
        # бортов из движения бьёт по редукторам. Сначала останавливаемся.
        if self.current_command not in ("stop",):
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.15)

        if direction == 'left':
            ca.rotate_left(self.turn_speed)
            cmd = "rotate_left"
        elif direction == 'right':
            ca.rotate_right(self.turn_speed)
            cmd = "rotate_right"
        else:
            raise ValueError(f"direction должен быть 'left'|'right', получено: {direction!r}")

        self._set_movement_command(cmd, self.turn_speed)
        self.last_update_time = time.time()

        start_time = time.time()
        safe = True
        try:
            while time.time() - start_time < max_duration:
                time.sleep(check_interval)
                self._tick()  # одометрия + SLAM пока крутимся
                distance = self.sensor_fusion.get_obstacle_distance('front')
                if distance and distance < self.rotate_safety_distance:
                    print(f"[SAFETY] Слишком близко при повороте: {distance:.2f}м")
                    safe = False
                    break
        except Exception as e:
            print(f"[SAFETY] Ошибка при повороте: {e}")
            safe = False
        finally:
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            # Секторные дистанции камеры сняты в СТАРОМ курсе — после
            # доворота её «front» смотрит вбок; сбрасываем до нового кадра.
            self.sensor_fusion.invalidate_camera()
            time.sleep(0.2)
        return safe

    # Совместимость со старыми вызовами — пусть остаются как тонкие алиасы.
    def _safe_rotate_left(self, max_duration: float = 1.0, check_interval: float = 0.2) -> bool:
        return self._safe_rotate('left', max_duration, check_interval)

    def _safe_rotate_right(self, max_duration: float = 1.0, check_interval: float = 0.2) -> bool:
        return self._safe_rotate('right', max_duration, check_interval)

    def _safe_backward(self, duration: float, check_interval: float = 0.2) -> bool:
        """
        Отъезд назад с контролем ЗАДНЕЙ дистанции (лидар видит 360°, сектор
        'back' обязан участвовать — раньше все отъезды были слепыми).

        Returns:
            False — сзади препятствие (отъезд отменён или прерван).
        """
        back = self.sensor_fusion.get_obstacle_distance('back')
        if back is not None and back < 0.25:
            print(f"[SAFETY] Сзади занято ({back:.2f}м) — отъезд отменён")
            return False

        ca.move_backward(self.exploration_speed)
        self._set_movement_command("backward", self.exploration_speed)
        self.last_update_time = time.time()
        ok = True
        end = time.time() + duration
        while time.time() < end:
            time.sleep(min(check_interval, max(0.0, end - time.time())))
            self._tick()
            back = self.sensor_fusion.get_obstacle_distance('back')
            if back is not None and back < 0.20:
                print(f"[SAFETY] Препятствие сзади ({back:.2f}м) — стоп")
                ok = False
                break
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)
        return ok

    def _emergency_stop_and_back(self):
        """Экстренная остановка и отъезд назад"""
        print("[SAFETY] ⚠️ ЭКСТРЕННАЯ ОСТАНОВКА!")

        # Немедленная остановка
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)

        # Отъезжаем немного назад с накоплением одометрии и контролем тыла
        print("[SAFETY] Отъезжаю назад...")
        self._safe_backward(0.8)

    def _timed_drive(self, duration: float, check_interval: float = 0.2):
        """
        Выполнить уже стартованную команду в течение duration секунд,
        тикая одометрию/SLAM. Команду должны выставить ДО вызова.
        """
        end = time.time() + duration
        while time.time() < end:
            sleep_for = min(check_interval, max(0.0, end - time.time()))
            if sleep_for > 0:
                time.sleep(sleep_for)
            self._tick()

    # ===== БЕЗОПАСНЫЕ ПОВЕДЕНИЯ =====

    def _safe_exploration_behavior(self):
        """БЕЗОПАСНОЕ поведение в режиме исследования"""
        
        # Получаем данные с датчиков
        obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')
        
        # Отладка — не чаще раза в секунду (чтобы не засорять лог)
        now = time.time()
        if not hasattr(self, '_last_nav_debug_time'):
            self._last_nav_debug_time = 0
        if now - self._last_nav_debug_time >= 1.0:
            if obstacle_distance is not None:
                print(f"[NAV_DEBUG] Препятствие: {obstacle_distance:.2f}м")
            else:
                print("[NAV_DEBUG] Нет данных от датчиков")
            self._last_nav_debug_time = now

        # Если нет данных от датчиков
        if obstacle_distance is None:
            # Проверяем не зациклились ли мы
            if self.consecutive_rotations >= self.max_consecutive_rotations:
                print("[NAV_DEBUG] Много поворотов подряд, пробую короткое движение вперед...")
                self.consecutive_rotations = 0
                # Короткое пробное движение
                if self._safe_move_forward(0.5):
                    print("[NAV_DEBUG] Движение успешно")
                else:
                    print("[NAV_DEBUG] Препятствие обнаружено, поворачиваю")
                    self._safe_rotate_left(0.8)
                    self.consecutive_rotations += 1
            else:
                print("[NAV_DEBUG] Нет данных - короткий поворот для сканирования...")
                self._safe_rotate_left(0.7)
                self.consecutive_rotations += 1
            return
        
        # ЭКСТРЕННАЯ ОСТАНОВКА если слишком близко
        if obstacle_distance < self.critical_distance:
            self._emergency_stop_and_back()
            self._safe_rotate(self._choose_turn_direction(), 1.0)
            self.consecutive_rotations += 1
            return

        # Проверяем расстояние для нормального движения
        if obstacle_distance < self.min_obstacle_distance:
            print(f"[NAV_DEBUG] Препятствие близко: {obstacle_distance:.2f}м")

            # С круизом сюда можно прийти на ходу — сначала остановка
            if self.current_command != "stop":
                ca.stop_robot()
                self._set_movement_command("stop", 0)
                time.sleep(0.2)

            # Проверяем не зациклились ли мы на поворотах
            if self.consecutive_rotations >= self.max_consecutive_rotations:
                print("[NAV_DEBUG] ⚠️ Зацикливание на поворотах! Пробую объезд...")
                self.consecutive_rotations = 0

                # Отъезжаем назад с тиками одометрии и контролем тыла
                print("[NAV_DEBUG] Отъезжаю назад...")
                self._safe_backward(0.7)

                # Ломаем зацикливание: крутим в ПРОТИВОПОЛОЖНУЮ от «свободной»
                # стороны — в свободную мы уже навертелись и упёрлись.
                opposite = ('right' if self._choose_turn_direction() == 'left'
                            else 'left')
                print(f"[NAV_DEBUG] Поворот в обратную сторону ({opposite})...")
                self._safe_rotate(opposite, 1.2)
            else:
                # Поворачиваем в сторону большего свободного пространства
                direction = self._choose_turn_direction()
                print(f"[NAV_DEBUG] Поворачиваю {'налево' if direction == 'left' else 'направо'} "
                      f"к свободному пространству (L={self._side_clearance('left'):.2f}, "
                      f"R={self._side_clearance('right'):.2f})...")
                self._safe_rotate(direction, 1.0)
                self.consecutive_rotations += 1

        else:
            print(f"[NAV_DEBUG] ✓ Свободный путь: {obstacle_distance:.2f}м")

            self.consecutive_rotations = 0

            # Вычисляем безопасное время движения
            # Чем дальше препятствие, тем дольше можем ехать
            safe_time = min(2.5, (obstacle_distance - self.min_obstacle_distance) * 3)
            safe_time = max(0.6, safe_time)

            print(f"[NAV_DEBUG] Движение вперед на {safe_time:.1f} сек...")

            # Круиз: без остановки на стыке сегментов (stop_at_end=False) и
            # с мягким уводом в сторону большего простора — «движение в
            # свободное пространство» на ходу, одометрия увод учитывает.
            if not self._safe_move_forward(safe_time, stop_at_end=False,
                                           steer_to_free=True):
                # Если обнаружено препятствие во время движения
                print("[NAV_DEBUG] Препятствие обнаружено во время движения")
                self._safe_rotate(self._choose_turn_direction(), 1.0)
                self.consecutive_rotations += 1
            else:
                print("[NAV_DEBUG] ✓ Движение завершено успешно")

    def _side_clearance(self, direction: str) -> float:
        """
        Простор сбоку (м) для выбора направления. None (нет точек в секторе)
        трактуем как простор: у лидара отсутствие отражений в секторе чаще
        означает «далеко до всего», чем «нет данных».
        """
        d = self.sensor_fusion.get_obstacle_distance(direction)
        return d if d is not None else float('inf')

    def _choose_turn_direction(self) -> str:
        """
        Куда поворачивать при препятствии впереди: в сторону БОЛЬШЕГО
        свободного пространства по секторным данным sensor fusion.
        При равенстве (включая «оба неизвестны») — налево, как раньше.
        """
        return ('right'
                if self._side_clearance('right') > self._side_clearance('left')
                else 'left')

    def _safe_navigation_behavior(self):
        """Поведение в режиме навигации: DWA (по умолчанию) или legacy-автомат."""
        if self.DWA_ENABLED:
            self._dwa_navigation_step()
        else:
            self._legacy_navigation_behavior()

    def _dwa_navigation_step(self):
        """
        Один НЕблокирующий такт DWA-навигации: вызывается из update() на
        каждом тике (10–20 Гц). Робот непрерывно течёт по дугам к текущему
        waypoint'у A*, плавно огибая препятствия по карте; реактивный слой
        остаётся страховкой поверх (экстренный стоп + кап скорости).
        """
        # --- Путь пройден? ---
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            print("[NavController] Цель достигнута")
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            self._dwa_vel = (0.0, 0.0)
            self.mode = NavigationMode.IDLE
            return

        pose = self.slam.get_pose()
        waypoint = self.current_path[self.current_waypoint_idx]
        dist = float(np.hypot(waypoint.x - pose.x, waypoint.y - pose.y))

        # --- Waypoint достигнут → следующий (без остановки) ---
        if dist < self.goal_tolerance:
            self.current_waypoint_idx += 1
            print(f"[NavController] Waypoint {self.current_waypoint_idx}/"
                  f"{len(self.current_path)} достигнут")
            return

        # --- Реактивная страховка поверх DWA ---
        # DWA объезжает то, что ЕСТЬ В КАРТЕ; свежие/динамические препятствия
        # (особенно в localization, где карта read-only) видит только
        # реактивный слой — он и стопит.
        front = self.sensor_fusion.get_obstacle_distance('front')
        if front is not None and front < self.DWA_BLOCK_FRONT_M:
            print(f"[NavController] Препятствие по курсу ({front:.2f}м), объезд")
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            self._dwa_vel = (0.0, 0.0)
            self.mode = NavigationMode.OBSTACLE_AVOIDANCE
            time.sleep(0.2)
            return
        # Кап скорости по фронту: чем ближе препятствие, тем медленнее
        # позволяем DWA ехать (0.5 м/с² торможения с запасом).
        v_cap = None
        if front is not None:
            v_cap = max(0.0, 0.5 * (front - 0.20))
            self._dwa_no_data_count = 0
        else:
            # Сенсоры молчат: DWA видит только карту, свежие препятствия —
            # нет. Полсекунды молчания → ползущий кап как страховка
            # (в localization новое препятствие в карте НЕ появится).
            self._dwa_no_data_count = getattr(self, '_dwa_no_data_count', 0) + 1
            if self._dwa_no_data_count >= self.DWA_NO_DATA_TICKS:
                v_cap = 0.06

        # --- Шаг DWA ---
        v, omega = self.path_planner.compute_velocity_command(
            (pose.x, pose.y, pose.theta),
            self._dwa_vel,
            (waypoint.x, waypoint.y),
            v_max_cap=v_cap,
        )

        # Кап ниже порога трогания моторов → запрещаем квантованию
        # подтягивать PWM вверх (иначе пробьёт кап у самого препятствия)
        floor_speed = self.MIN_MOVE_PWM / self.PWM_FULL * self.MAX_LINEAR_SPEED
        allow_bump = v_cap is None or v_cap >= floor_speed
        self._dwa_vel = self._send_velocity_command(v, omega, allow_bump=allow_bump)

        # Stall-детектор: DWA не нашёл траекторию (вернул 0,0) ЛИБО команда
        # заквантовалась в ноль (DWA «прижался» к препятствию, скорость упала
        # ниже порога трогания моторов) — в обоих случаях робот фактически
        # стоит, и через DWA_STALL_LIMIT тактов перепланируем A*. Повторные
        # безрезультатные перепланирования (по той же карте они возвращают
        # тот же путь) эскалируются в физический объезд.
        if self._dwa_vel == (0.0, 0.0):
            self._dwa_stall_count += 1
            if self._dwa_stall_count >= self.DWA_STALL_LIMIT:
                self._dwa_stall_count = 0
                self._dwa_replan_count = getattr(self, '_dwa_replan_count', 0) + 1
                if self._dwa_replan_count > self.DWA_REPLAN_LIMIT:
                    print("[NavController] DWA: перепланирования не помогают — объезд")
                    self._dwa_replan_count = 0
                    self.mode = NavigationMode.OBSTACLE_AVOIDANCE
                    return
                print("[NavController] DWA: застой — перепланирую путь")
                new_path = self.path_planner.plan_global_path(
                    (pose.x, pose.y), self.current_goal)
                if new_path:
                    self.current_path = new_path
                    self.current_waypoint_idx = 0
                else:
                    self.mode = NavigationMode.OBSTACLE_AVOIDANCE
        else:
            self._dwa_stall_count = 0
            self._dwa_replan_count = 0

    def _legacy_navigation_behavior(self):
        """Прежний дискретный автомат навигации (откат при DWA_ENABLED=False)."""

        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            # Путь пройден
            print("[NavController] Цель достигнута")
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            self.mode = NavigationMode.IDLE
            return

        # Текущая позиция
        pose = self.slam.get_pose()

        # Текущая промежуточная точка
        waypoint = self.current_path[self.current_waypoint_idx]

        # Расстояние до waypoint
        dx = waypoint.x - pose.x
        dy = waypoint.y - pose.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Если достигли waypoint, переходим к следующей
        if distance < self.goal_tolerance:
            self.current_waypoint_idx += 1
            print(f"[NavController] Waypoint {self.current_waypoint_idx}/{len(self.current_path)} достигнут")
            return

        # Проверяем препятствия перед движением
        obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')

        if obstacle_distance is not None and obstacle_distance < self.min_obstacle_distance:
            # Препятствие на пути - переходим в режим объезда
            print(f"[NavController] Препятствие обнаружено ({obstacle_distance:.2f}м), объезд")
            self.mode = NavigationMode.OBSTACLE_AVOIDANCE
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.3)
            return

        # Вычисляем направление к waypoint
        target_angle = np.arctan2(dy, dx)
        angle_diff = self._normalize_angle(target_angle - pose.theta)

        # Если угол большой, сначала поворачиваемся
        if abs(angle_diff) > np.deg2rad(20):
            if angle_diff > 0:
                print(f"[NavController] Поворачиваю налево {np.degrees(angle_diff):.0f}°")
                self._safe_rotate_left(min(1.5, abs(angle_diff) / 2.0))
            else:
                print(f"[NavController] Поворачиваю направо {np.degrees(abs(angle_diff)):.0f}°")
                self._safe_rotate_right(min(1.5, abs(angle_diff) / 2.0))
        else:
            # Едем вперед с частыми проверками
            print(f"[NavController] Двигаюсь к waypoint ({distance:.2f}м)")

            # Двигаемся короткими отрезками с проверкой
            move_success = True
            move_start = time.time()

            while move_success and time.time() - move_start < 1.5:
                # Проверяем перед движением
                check_dist = self.sensor_fusion.get_obstacle_distance('front')
                if check_dist and check_dist < self.min_obstacle_distance:
                    print(f"[NavController] Препятствие на пути: {check_dist:.2f}м")
                    move_success = False
                    break

                # Подруливание на ходу: малую угловую ошибку (≤20°) закрываем
                # дифференциалом бортов БЕЗ остановки — вместо прежнего
                # стоп-поворот-стоп у каждого waypoint'а. Усиление 3%/°,
                # потолок ±60% — закрывает 20° примерно за секунду хода.
                pose_now = self.slam.get_pose()
                err = self._normalize_angle(
                    np.arctan2(waypoint.y - pose_now.y,
                               waypoint.x - pose_now.x) - pose_now.theta)
                steer = float(np.clip(np.degrees(err) * 3.0, -60.0, 60.0))
                self._apply_forward(self.navigation_speed,
                                    steer if abs(steer) >= 5.0 else 0.0)
                self.last_update_time = time.time()
                self._timed_drive(0.3, check_interval=0.1)

                # Проверяем снова
                check_dist = self.sensor_fusion.get_obstacle_distance('front')
                if check_dist and check_dist < self.min_obstacle_distance:
                    ca.stop_robot()
                    self._set_movement_command("stop", 0)
                    print(f"[NavController] Обнаружено препятствие во время движения: {check_dist:.2f}м")
                    move_success = False
                    break

            # Останавливаемся
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.2)

            if not move_success:
                self.mode = NavigationMode.OBSTACLE_AVOIDANCE

    def _safe_avoidance_behavior(self):
        """Безопасное поведение при объезде препятствий"""

        print("[NavController] Безопасный объезд препятствия")

        # Повторные объезды у одной и той же точки = путь физически
        # заблокирован (в localization перепланирование по статичной карте
        # возвращает тот же маршрут — без лимита это вечные качели).
        pose0 = self.slam.get_pose()
        last = getattr(self, '_avoid_last_pos', None)
        if last is not None and np.hypot(pose0.x - last[0], pose0.y - last[1]) < 0.4:
            self._avoid_count = getattr(self, '_avoid_count', 0) + 1
        else:
            self._avoid_count = 1
        self._avoid_last_pos = (pose0.x, pose0.y)
        if self.current_goal and self._avoid_count > 3:
            self._avoid_count = 0
            self._goal_unreachable("трижды объезжаю в одном месте")
            return
        
        # Сначала отъезжаем немного назад (с контролем заднего сектора)
        print("[NavController] Отъезжаю назад...")
        self._safe_backward(0.5)
        
        # Получаем свободные направления
        free_dirs = self.sensor_fusion.get_free_directions(self.min_obstacle_distance)

        # Выбираем сторону объезда: из свободных — ту, где БОЛЬШЕ простора
        # (двигаемся в максимально свободное пространство, а не всегда налево).
        order = sorted(('left', 'right'),
                       key=self._side_clearance, reverse=True)
        side = next((s for s in order if free_dirs.get(s, False)), None)

        if side is not None:
            back = 'right' if side == 'left' else 'left'
            print(f"[NavController] Объезжаю {'слева' if side == 'left' else 'справа'} "
                  f"(простор L={self._side_clearance('left'):.2f}, "
                  f"R={self._side_clearance('right'):.2f})")
            self._safe_rotate(side, 0.8)
            safe_moved = self._safe_move_forward(1.0)
            if safe_moved:
                self._safe_rotate(back, 0.8)
        else:
            print("[NavController] Нет свободных направлений, поворачиваю на 180°")
            self._safe_rotate_left(2.0)
        
        # Пытаемся перепланировать путь
        if self.current_goal:
            pose = self.slam.get_pose()
            new_path = self.path_planner.plan_global_path(
                (pose.x, pose.y),
                self.current_goal
            )

            if new_path:
                self.current_path = new_path
                self.current_waypoint_idx = 0
                # Свежий старт DWA после манёвра
                self._dwa_vel = (0.0, 0.0)
                self._dwa_stall_count = 0
                self._dwa_replan_count = 0
                self.mode = NavigationMode.NAVIGATION
                print("[NavController] Путь перепланирован")
            else:
                self._goal_unreachable("перепланирование не нашло пути")
        else:
            # Нет цели, возвращаемся к исследованию
            self.mode = NavigationMode.EXPLORATION

    def stop(self):
        """Остановка робота"""
        print("\n[NavController] ⚠️ ОСТАНОВКА")
        ca.stop_robot()
        self.mode = NavigationMode.IDLE

        # Освобождаем ресурсы лидара
        if self.lidar is not None:
            try:
                print("[NavController] Останавливаю лидар...")
                
                # Сначала обычная остановка
                self.lidar.stop_scan()
                time.sleep(0.5)
                
                # Если есть метод force_stop, используем его
                if hasattr(self.lidar, 'force_stop'):
                    self.lidar.force_stop()
                    time.sleep(0.5)
                
                # Потом отключаем
                self.lidar.disconnect()
                
            except Exception as e:
                print(f"[NavController] Ошибка остановки лидара: {e}")
        
        # Останавливаем фоновый инференс и освобождаем камеру
        if getattr(self, '_seg_worker', None) is not None:
            try:
                self._seg_worker.stop()
            except Exception:
                pass
        if getattr(self, '_depth_worker', None) is not None:
            try:
                print("[NavController] Останавливаю поток инференса...")
                self._depth_worker.stop()
            except Exception:
                pass
        if getattr(self, 'camera_cap', None) is not None:
            try:
                self.camera_cap.release()
            except Exception:
                pass

        # Фоновый логгер кадров — дописать остаток очереди и закрыть.
        if getattr(self, '_image_logger', None) is not None:
            try:
                self._image_logger.stop()
                if self._image_logger.dropped:
                    print(f"[NavController] Лог кадров: записано "
                          f"{self._image_logger.written}, отброшено "
                          f"{self._image_logger.dropped} (очередь переполнялась)")
            except Exception:
                pass

        # Рекордер — последним: лидар и depth-поток уже остановлены, дописываем
        # остаток очереди и закрываем файлы.
        if getattr(self, 'recorder', None) is not None:
            try:
                self.recorder.stop()
                if self.recorder.dropped:
                    print(f"[NavController] Рекордер: записано {self.recorder.written}, "
                          f"отброшено {self.recorder.dropped} (очередь переполнялась)")
            except Exception:
                pass

    def save_map(self, filename: str) -> str:
        """
        Сохранение карты (.pkl + .png). inflation на PNG рисуем тем же радиусом,
        который A* использует на самом деле.

        Голое имя файла (без каталога) кладём в maps/ этого запуска
        (results/<запуск>/maps/) — чтобы не сорить в текущей папке. Явный путь
        (с каталогом) уважаем как есть. Загрузку (load_map / goto --map) это не
        трогает: туда передаётся явный путь. Возвращаем фактический путь .pkl,
        чтобы вызывающий код напечатал реальное местоположение.
        """
        if not os.path.dirname(filename):
            os.makedirs(self.maps_dir, exist_ok=True)
            filename = os.path.join(self.maps_dir, filename)
        self.slam.save_map(filename,
                           inflation_radius_m=self.ROBOT_RADIUS + self.SAFETY_MARGIN)
        return filename

    def _make_path_planner(self) -> PathPlanner:
        """
        Создать PathPlanner с параметрами робота (footprint + пределы DWA).
        Используется и в __init__, и в load_map — конфигурация одна.
        """
        planner = PathPlanner(
            self.slam.map,
            robot_radius=self.ROBOT_RADIUS,
            inflation_margin=self.SAFETY_MARGIN,
        )
        # Пределы DWA — от реальной кинематики (single source of truth здесь).
        # Скоростной потолок умеренный: безопасная демонстрационная езда.
        planner.max_speed = self.DWA_MAX_SPEED_M_S
        planner.max_yaw_rate = self.DWA_MAX_YAW_RATE
        planner.max_accel = 0.4
        planner.max_delta_yaw_rate = 2.0
        planner.window_dt = 0.3
        planner.predict_time = 2.2
        planner.dt = 0.15
        return planner

    def load_map(self, filename: str):
        """Загрузка карты"""
        self.slam.load_map(filename)
        # Обновляем планировщик с теми же параметрами, что и при __init__
        self.path_planner = self._make_path_planner()

    def get_status(self) -> dict:
        """Получение статуса системы"""
        pose = self.slam.get_pose()

        return {
            'mode': self.mode.value,
            'position': (pose.x, pose.y, pose.theta),
            'goal': self.current_goal,
            'waypoint': f"{self.current_waypoint_idx}/{len(self.current_path)}" if self.current_path else "N/A",
            'obstacle_distance': self.sensor_fusion.get_obstacle_distance('front')
        }

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def quick_safety_test(self):
        """Быстрый тест безопасности движений"""
        print("="*60)
        print("ТЕСТ БЕЗОПАСНОСТИ - ОЧЕНЬ МЕДЛЕННОЕ ДВИЖЕНИЕ")
        print("="*60)
        
        # Тест медленного движения
        print("\n1. Медленное движение вперед (PWM=30)...")
        ca.move_forward(30)
        time.sleep(1.5)
        ca.stop_robot()
        time.sleep(0.5)
        
        print("\n2. Медленный поворот (PWM=20)...")
        ca.rotate_left(20)
        time.sleep(1.0)
        ca.stop_robot()
        time.sleep(0.5)
        
        print("\n3. Медленное движение назад (PWM=30)...")
        ca.move_backward(30)
        time.sleep(1.0)
        ca.stop_robot()
        
        print("\n✓ Тест завершен. Скорости безопасные.")


# Быстрый тест для проверки
if __name__ == "__main__":
    nav = NavigationController(use_lidar=False, use_camera=False, use_ultrasonic=True)
    nav.quick_safety_test()