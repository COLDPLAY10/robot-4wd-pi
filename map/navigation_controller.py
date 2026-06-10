#!/usr/bin/env python3
"""
Контроллер навигации робота
Управляет режимами работы и координирует все компоненты
"""

import threading
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
    """

    def __init__(self, cap, estimator, get_pose, period_s: float):
        super().__init__(daemon=True, name="depth-worker")
        self._cap = cap
        self._estimator = estimator
        self._get_pose = get_pose
        self._period_s = period_s
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._result = None  # (frame, depth, pose, timestamp)
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
                if depth is not None:
                    with self._lock:
                        self._result = (frame, depth, pose, time.time())
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

    def stop(self, timeout: float = 2.0):
        self._stop_evt.set()
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
        self.path_planner = PathPlanner(
            self.slam.map,
            robot_radius=self.ROBOT_RADIUS,
            inflation_margin=self.SAFETY_MARGIN,
        )

        # Режим работы
        self.mode = NavigationMode.IDLE

        # Параметры движения (откалибровано для 4WD)
        self.exploration_speed = 30      # Скорость для исследования (PWM, мин. для реального движения)
        self.navigation_speed = 40       # Скорость для навигации по маршруту
        self.turn_speed = 20             # Скорость поворота
        self.min_obstacle_distance = 0.4 # Минимальное расстояние до препятствия (м)
        self.critical_distance = 0.15    # Критическое расстояние для экстренной остановки
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
        self.current_command = "stop"  # stop, forward, forward_steer, backward, rotate_left, rotate_right
        self.current_speed = 0   # PWM 0-255
        self.current_steer = 0.0  # процент увода для forward_steer (см. move_param_forward)

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
    # При включении раз в CAMERA_DEBUG_PERIOD_S в camera_debug/ пишется JPEG:
    # слева исходный кадр, справа depth-цветом с красной маской препятствий
    # (те пиксели, что реально пошли в карту/реактив) и секторными дистанциями.
    # Включать на время отладки: каждый кадр — лишние ~50-100 мс на Pi.
    CAMERA_DEBUG_SAVE_ENABLED = False
    CAMERA_DEBUG_PERIOD_S = 3.0
    CAMERA_DEBUG_KEEP = 200        # сколько последних кадров хранить
    CAMERA_DEBUG_DIR = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'camera_debug',
    )
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

    def _save_camera_debug_frame(self, frame, depth, pose, cam_dists):
        """
        Сохранить отладочный кадр (RGB | depth + маска препятствий) в
        camera_debug/, не чаще CAMERA_DEBUG_PERIOD_S. Старые кадры сверх
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

        os.makedirs(self.CAMERA_DEBUG_DIR, exist_ok=True)
        fname = time.strftime('debug_%Y%m%d_%H%M%S') + f"_{int(now * 1000) % 1000:03d}.jpg"
        cv2.imwrite(os.path.join(self.CAMERA_DEBUG_DIR, fname), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Ротация: держим только последние CAMERA_DEBUG_KEEP кадров
        try:
            files = sorted(f for f in os.listdir(self.CAMERA_DEBUG_DIR)
                           if f.startswith('debug_') and f.endswith('.jpg'))
            for old in files[:-self.CAMERA_DEBUG_KEEP]:
                os.remove(os.path.join(self.CAMERA_DEBUG_DIR, old))
        except OSError:
            pass

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
            self.mode = NavigationMode.NAVIGATION
            print(f"[NavController] Путь построен: {len(path)} точек")
        else:
            print("[NavController] Не удалось построить путь, переход в режим исследования")
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
                    # update_with_camera_depth пишет в карту (только в
                    # mapping, и только до CAMERA_MAP_MAX_RANGE_M) и
                    # ВОЗВРАЩАЕТ облако препятствий по высоте на всю
                    # дальность анализа — доступно и в localization.
                    # pose — поза НА МОМЕНТ ЗАХВАТА кадра (см. _DepthWorker).
                    obstacles = self.slam.update_with_camera_depth(
                        depth, self.camera_intrinsics, self.camera_mount,
                        pixel_stride=self.CAMERA_PIXEL_STRIDE,
                        max_range_m=self.CAMERA_MAX_RANGE_M,
                        map_write_max_range_m=self.CAMERA_MAP_MAX_RANGE_M,
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
            except Exception as e:
                self._log_sensor_error('camera', e)

        # 2) Лидар (чтение из буфера — дёшево).
        if self.use_lidar and self.lidar is not None:
            try:
                lidar_scan = self.lidar.get_scan()
                if lidar_scan and len(lidar_scan) > 0:
                    # Фильтрация шума: отбрасываем точки ближе 12 см (шум T-MINI Plus)
                    filtered_scan = [(a, d) for a, d in lidar_scan if d >= 0.12]
                    if filtered_scan:
                        self.sensor_fusion.update_lidar(filtered_scan)
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

        # PWM в car_adapter лежит в диапазоне 0..255, current_speed хранится так же.
        pwm_factor = min(1.0, max(0.0, self.current_speed / self.PWM_FULL))
        v_max = self.MAX_LINEAR_SPEED * pwm_factor
        omega_max = self.MAX_ANGULAR_SPEED * pwm_factor

        cmd = self.current_command

        if cmd == "forward":
            v, omega = v_max, 0.0
        elif cmd == "forward_steer":
            # Подруливание дифференциалом бортов (move_param_forward):
            # p>0 усиливает ПРАВЫЙ борт на p% → средняя скорость растёт на
            # p/2%, а робот заворачивает ВЛЕВО (omega>0). Модель обязана
            # совпадать с тем, что реально делает car_adapter, иначе
            # одометрия поедет мимо scan matcher'а.
            p = self.current_steer
            v = v_max * (1.0 + p / 200.0)
            omega = (v_max * (p / 100.0)) / self.WHEEL_BASE
        elif cmd == "backward":
            v, omega = -v_max, 0.0
        elif cmd == "rotate_left":
            v, omega = 0.0, omega_max
        elif cmd == "rotate_right":
            v, omega = 0.0, -omega_max
        else:
            # stop, либо боковое мекалум-движение — в одометрии не учитываем
            v, omega = 0.0, 0.0

        # Конвертация (v, omega) → скорости бортов для diff-drive модели в SLAM.
        # SLAM сам пересчитает обратно: v=(l+r)/2, omega=(r-l)/wheel_base.
        half_base = self.WHEEL_BASE / 2.0
        left_speed = v - omega * half_base
        right_speed = v + omega * half_base

        self.slam.update_odometry(left_speed, right_speed, dt)

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
        self._update_sensors()
        self._update_odometry(dt)

    def _set_movement_command(self, command: str, speed: int, steer: float = 0.0):
        """
        Установка команды движения для отслеживания одометрии

        Args:
            command: тип команды (forward, forward_steer, backward,
                     rotate_left, rotate_right, stop)
            speed: скорость PWM (0-255) — номинал борта ДО усиления steer'ом
            steer: для forward_steer — процент усиления борта, как в
                   car_adapter.move_param_forward (p>0 → правый борт → влево)
        """
        self.current_command = command
        self.current_speed = speed
        self.current_steer = float(steer)

    # ===== Параметры плавного движения =====
    SLOWDOWN_START_M = 0.7   # с этой дистанции начинаем плавно сбрасывать PWM
    MIN_MOVE_PWM = 22        # ниже TT-моторы у порога трогания (стиктион)
    CRUISE_STEER_MAX = 20.0  # макс. увод к простору на круизе, % борта

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

    def _safe_move_forward(self, max_duration=1.0, check_interval=0.2,
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
                # Ждем до следующей проверки
                time.sleep(check_interval)
                # Тикаем SLAM/одометрию пока движемся, иначе позиция/угол не накапливаются
                self._tick()
                check_count += 1

                # Проверяем датчики
                obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')

                # Если нет данных от датчиков, используем ультразвук напрямую
                if obstacle_distance is None and self.use_ultrasonic:
                    try:
                        from ultrasonic import read_distance_cm_from_bot
                        us_cm = read_distance_cm_from_bot()
                        if us_cm is not None and us_cm > 0:
                            obstacle_distance = us_cm / 100.0
                            print(f"[SAFE_MOVE] Ультразвук: {obstacle_distance:.2f}м")
                    except:
                        pass

                if obstacle_distance is None:
                    no_data_streak += 1
                    print(f"[SAFE_MOVE] ⚠️ Проверка {check_count}: Нет данных датчиков")
                    # 3 проверки ПОДРЯД без данных — останавливаемся
                    if no_data_streak >= 3:
                        print("[SAFE_MOVE] Нет данных 3 проверки подряд, останавливаюсь")
                        safe = False
                        break
                    continue  # Продолжаем движение, но с осторожностью
                no_data_streak = 0

                # КРИТИЧЕСКОЕ расстояние - немедленная остановка
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
                time.sleep(0.3)

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
                if distance and distance < self.critical_distance:
                    print(f"[SAFETY] Слишком близко при повороте: {distance:.2f}м")
                    safe = False
                    break
        except Exception as e:
            print(f"[SAFETY] Ошибка при повороте: {e}")
            safe = False
        finally:
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.2)
        return safe

    # Совместимость со старыми вызовами — пусть остаются как тонкие алиасы.
    def _safe_rotate_left(self, max_duration: float = 1.0, check_interval: float = 0.2) -> bool:
        return self._safe_rotate('left', max_duration, check_interval)

    def _safe_rotate_right(self, max_duration: float = 1.0, check_interval: float = 0.2) -> bool:
        return self._safe_rotate('right', max_duration, check_interval)

    def _emergency_stop_and_back(self):
        """Экстренная остановка и отъезд назад"""
        print("[SAFETY] ⚠️ ЭКСТРЕННАЯ ОСТАНОВКА!")

        # Немедленная остановка
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)

        # Отъезжаем немного назад с накоплением одометрии
        print("[SAFETY] Отъезжаю назад...")
        ca.move_backward(self.exploration_speed)
        self._set_movement_command("backward", self.exploration_speed)
        self.last_update_time = time.time()
        self._timed_drive(0.8, check_interval=0.2)

        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)

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

                # Отъезжаем назад с тиками одометрии
                print("[NAV_DEBUG] Отъезжаю назад...")
                ca.move_backward(self.exploration_speed)
                self._set_movement_command("backward", self.exploration_speed)
                self.last_update_time = time.time()
                self._timed_drive(0.7)
                ca.stop_robot()
                self._set_movement_command("stop", 0)
                time.sleep(0.3)

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
        """Безопасное поведение в режиме навигации по маршруту"""

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
        
        # Сначала отъезжаем немного назад
        print("[NavController] Отъезжаю назад...")
        ca.move_backward(self.exploration_speed)
        self._set_movement_command("backward", self.exploration_speed)
        self.last_update_time = time.time()
        self._timed_drive(0.5)
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)
        
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
                self.mode = NavigationMode.NAVIGATION
                print("[NavController] Путь перепланирован")
            else:
                # Не удалось перепланировать, возвращаемся к исследованию
                print("[NavController] Не удалось перепланировать, продолжаю исследование")
                self.mode = NavigationMode.EXPLORATION
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

        # Освобождаем ресурсы камеры (legacy-сегментация)
        if hasattr(self, 'camera_seg') and self.camera_seg is not None:
            try:
                self.camera_seg.release()
            except:
                pass

    def save_map(self, filename: str):
        """Сохранение карты — inflation на PNG показываем тем же радиусом, который A* использует на самом деле."""
        self.slam.save_map(filename,
                           inflation_radius_m=self.ROBOT_RADIUS + self.SAFETY_MARGIN)

    def load_map(self, filename: str):
        """Загрузка карты"""
        self.slam.load_map(filename)
        # Обновляем планировщик с теми же footprint-параметрами, что и при __init__
        self.path_planner = PathPlanner(
            self.slam.map,
            robot_radius=self.ROBOT_RADIUS,
            inflation_margin=self.SAFETY_MARGIN,
        )

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