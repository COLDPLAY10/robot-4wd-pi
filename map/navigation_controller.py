#!/usr/bin/env python3
"""
Контроллер навигации робота
Управляет режимами работы и координирует все компоненты
"""

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


class NavigationController:
    """
    Основной контроллер навигации
    Интегрирует SLAM, планирование и управление
    """

    def __init__(self, use_lidar=False, use_camera=True, use_ultrasonic=True):
        """
        Args:
            use_lidar: использовать лидар
            use_camera: использовать камеру
            use_ultrasonic: использовать ультразвуковой датчик
        """
        print("\n" + "="*60)
        print("ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ НАВИГАЦИИ")
        print("="*60)

        self.use_lidar = use_lidar
        self.use_camera = use_camera
        self.use_ultrasonic = use_ultrasonic

        # Инициализация компонентов
        self.slam = SLAM(
            map_width=400,
            map_height=400,
            resolution=0.05,
            use_lidar=use_lidar
        )

        self.sensor_fusion = SensorFusion(
            use_lidar=use_lidar,
            use_camera=use_camera,
            use_ultrasonic=use_ultrasonic
        )

        self.path_planner = PathPlanner(self.slam.map)

        # Режим работы
        self.mode = NavigationMode.IDLE

        # Параметры движения
        self.exploration_speed = 15      # медленная скорость для анализа
        self.navigation_speed = 30       # быстрая скорость для маршрута
        self.min_obstacle_distance = 0.3 # минимальное расстояние до препятствия (м)
        self.goal_tolerance = 0.2        # допустимое отклонение от цели (м)

        # Текущая цель
        self.current_goal = None
        self.current_path = []
        self.current_waypoint_idx = 0

        # Время последнего обновления
        self.last_update_time = time.time()

        # Отслеживание движения для одометрии
        self.current_command = "stop"  # stop, forward, backward, left, right, rotate_left, rotate_right
        self.current_speed = 0  # PWM 0-100

        # Робот
        self.bot = ca.bot

        # Лидар
        self.lidar = None
        if self.use_lidar:
            self._init_lidar()

        # Инициализация датчиков
        self._init_sensors()

        print("="*60)
        print("СИСТЕМА ГОТОВА К РАБОТЕ")
        print("="*60 + "\n")

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

            # Пробуем подключиться к лидару на разных портах
            ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyS0']

            for port in ports:
                try:
                    print(f"[NavController] Попытка подключения к лидару на {port}...")
                    self.lidar = LidarDriver(port=port, baudrate=230400)

                    if self.lidar.connect():
                        self.lidar.start_scan()
                        print(f"[NavController] Лидар подключен на {port}")
                        return
                    else:
                        self.lidar = None
                except Exception as e:
                    print(f"[NavController] Ошибка на порту {port}: {e}")
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
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Обновляем данные сенсоров
        self._update_sensors()

        # Обновляем одометрию (примерно)
        self._update_odometry(dt)

        # Выполняем действия в зависимости от режима
        if self.mode == NavigationMode.EXPLORATION:
            self._exploration_behavior()
        elif self.mode == NavigationMode.NAVIGATION:
            self._navigation_behavior()
        elif self.mode == NavigationMode.OBSTACLE_AVOIDANCE:
            self._avoidance_behavior()

    def _update_sensors(self):
        """Обновление данных сенсоров"""

        # Ультразвуковой датчик
        if self.use_ultrasonic:
            try:
                from ultrasonic import read_distance_cm_from_bot
                distance_cm = read_distance_cm_from_bot()
                if distance_cm is not None and distance_cm > 0:
                    distance_m = distance_cm / 100.0
                    self.sensor_fusion.update_ultrasonic(distance_m)
                    self.slam.update_with_ultrasonic(distance_m)
            except Exception as e:
                pass  # Игнорируем ошибки чтения

        # Лидар
        if self.use_lidar and self.lidar is not None:
            try:
                lidar_scan = self.lidar.get_scan()
                if lidar_scan and len(lidar_scan) > 0:
                    self.sensor_fusion.update_lidar(lidar_scan)
                    self.slam.update_with_lidar(lidar_scan)
            except Exception as e:
                pass  # Игнорируем ошибки чтения лидара

        # Камера и сегментация
        if self.use_camera:
            try:
                # Инициализируем камеру при первом вызове
                if not hasattr(self, 'camera_seg'):
                    from camera_segmentation import CameraSegmentation
                    self.camera_seg = CameraSegmentation()

                # Получаем маску сегментации
                segmentation = self.camera_seg.get_segmentation_mask()

                if segmentation is not None:
                    self.sensor_fusion.update_camera(segmentation)
                    self.slam.update_with_camera_segmentation(segmentation)
            except Exception as e:
                pass  # Игнорируем ошибки камеры

    def _update_odometry(self, dt: float):
        """
        Обновление одометрии на основе команд движения
        """
        # Параметры робота (примерные)
        max_speed = 0.3  # максимальная скорость м/с при PWM=100
        wheel_base = 0.15  # расстояние между колесами в метрах

        # Преобразуем PWM (0-100) в скорость (м/с)
        speed_factor = self.current_speed / 100.0
        linear_speed = max_speed * speed_factor

        # Оценка скоростей левых и правых колес на основе команды
        if self.current_command == "forward":
            left_speed = linear_speed
            right_speed = linear_speed

        elif self.current_command == "backward":
            left_speed = -linear_speed
            right_speed = -linear_speed

        elif self.current_command == "rotate_left":
            # Поворот на месте влево
            left_speed = -linear_speed * 0.5
            right_speed = linear_speed * 0.5

        elif self.current_command == "rotate_right":
            # Поворот на месте вправо
            left_speed = linear_speed * 0.5
            right_speed = -linear_speed * 0.5

        elif self.current_command == "left":
            # Движение влево (омни-колеса)
            left_speed = -linear_speed * 0.3
            right_speed = linear_speed * 0.3

        elif self.current_command == "right":
            # Движение вправо (омни-колеса)
            left_speed = linear_speed * 0.3
            right_speed = -linear_speed * 0.3

        else:  # stop или неизвестная команда
            left_speed = 0.0
            right_speed = 0.0

        # Обновляем SLAM
        self.slam.update_odometry(left_speed, right_speed, dt)

    def _set_movement_command(self, command: str, speed: int):
        """
        Установка команды движения для отслеживания одометрии

        Args:
            command: тип команды (forward, backward, left, right, rotate_left, rotate_right, stop)
            speed: скорость PWM (0-100)
        """
        self.current_command = command
        self.current_speed = speed

    def _exploration_behavior(self):
        """Поведение в режиме исследования"""

        # Проверяем наличие препятствий впереди
        obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')

        if obstacle_distance is not None and obstacle_distance < self.min_obstacle_distance:
            # Препятствие близко - останавливаемся и поворачиваем
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.2)

            # Ищем свободное направление
            free_dirs = self.sensor_fusion.get_free_directions(self.min_obstacle_distance)

            if free_dirs.get('left', False):
                print("[NavController] Поворот налево")
                ca.rotate_left(self.exploration_speed)
                self._set_movement_command("rotate_left", self.exploration_speed)
                time.sleep(1.0)
            elif free_dirs.get('right', False):
                print("[NavController] Поворот направо")
                ca.rotate_right(self.exploration_speed)
                self._set_movement_command("rotate_right", self.exploration_speed)
                time.sleep(1.0)
            else:
                print("[NavController] Разворот на 180°")
                ca.rotate_right(self.exploration_speed)
                self._set_movement_command("rotate_right", self.exploration_speed)
                time.sleep(2.0)

            ca.stop_robot()
            self._set_movement_command("stop", 0)
        else:
            ca.move_forward(self.exploration_speed)
            self._set_movement_command("forward", self.exploration_speed)

    def _navigation_behavior(self):
        """Поведение в режиме навигации по маршруту"""

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

        # Проверяем препятствия
        obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')

        if obstacle_distance is not None and obstacle_distance < self.min_obstacle_distance * 1.5:
            # Препятствие на пути - переходим в режим объезда
            print("[NavController] Препятствие обнаружено, объезд")
            self.mode = NavigationMode.OBSTACLE_AVOIDANCE
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            return

        # Вычисляем направление к waypoint
        target_angle = np.arctan2(dy, dx)
        angle_diff = self._normalize_angle(target_angle - pose.theta)

        # Если угол большой, сначала поворачиваемся
        if abs(angle_diff) > np.deg2rad(15):
            if angle_diff > 0:
                ca.rotate_left(self.navigation_speed)
                self._set_movement_command("rotate_left", self.navigation_speed)
            else:
                ca.rotate_right(self.navigation_speed)
                self._set_movement_command("rotate_right", self.navigation_speed)
        else:
            # Едем вперед с коррекцией направления
            # Процентная коррекция на основе угла
            correction = np.clip(angle_diff * 100 / np.pi, -50, 50)
            ca.move_param_forward(self.navigation_speed, correction)
            self._set_movement_command("forward", self.navigation_speed)

    def _avoidance_behavior(self):
        """Поведение при объезде препятствий"""

        # Получаем свободные направления
        free_dirs = self.sensor_fusion.get_free_directions(self.min_obstacle_distance)

        if free_dirs.get('left', False):
            ca.move_left(self.exploration_speed)
            time.sleep(0.5)
        elif free_dirs.get('right', False):
            ca.move_right(self.exploration_speed)
            time.sleep(0.5)
        else:
            # Отступаем назад
            ca.move_backward(self.exploration_speed)
            time.sleep(0.5)

        ca.stop_robot()

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
                # Не удалось перепланировать, продолжаем объезд
                pass
        else:
            # Нет цели, возвращаемся к исследованию
            self.mode = NavigationMode.EXPLORATION

    def stop(self):
        """Остановка робота"""
        print("\n[NavController] Остановка")
        ca.stop_robot()
        self.mode = NavigationMode.IDLE

        # Освобождаем ресурсы лидара
        if self.lidar is not None:
            try:
                self.lidar.disconnect()
            except:
                pass

        # Освобождаем ресурсы камеры
        if hasattr(self, 'camera_seg') and self.camera_seg is not None:
            try:
                self.camera_seg.release()
            except:
                pass

    def save_map(self, filename: str):
        """Сохранение карты"""
        self.slam.save_map(filename)

    def load_map(self, filename: str):
        """Загрузка карты"""
        self.slam.load_map(filename)
        # Обновляем планировщик
        self.path_planner = PathPlanner(self.slam.map)

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

