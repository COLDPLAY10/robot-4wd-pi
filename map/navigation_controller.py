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

        # Параметры движения (УМЕНЬШЕННЫЕ для безопасности!)
        self.exploration_speed = 10      # ОЧЕНЬ медленная скорость для анализа (было 15)
        self.navigation_speed = 15       # Медленная скорость для маршрута (было 30)
        self.turn_speed = 8              # Скорость поворота
        self.min_obstacle_distance = 0.4 # Увеличенное минимальное расстояние (было 0.3)
        self.critical_distance = 0.2     # Критическое расстояние для экстренной остановки
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

        print(f"[NavController] Скорости: исследование={self.exploration_speed}, "
              f"навигация={self.navigation_speed}, поворот={self.turn_speed}")
        print(f"[NavController] Дистанции: мин={self.min_obstacle_distance}м, "
              f"крит={self.critical_distance}м")
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
          ports = ['/dev/ttyUSB1', '/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/ttyS0']  # ← /dev/ttyUSB1 ПЕРВЫМ!
  
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
            self._safe_exploration_behavior()
        elif self.mode == NavigationMode.NAVIGATION:
            self._safe_navigation_behavior()
        elif self.mode == NavigationMode.OBSTACLE_AVOIDANCE:
            self._safe_avoidance_behavior()

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

        if self.use_lidar and self.lidar is not None:
            try:
                lidar_scan = self.lidar.get_scan()
                
                if lidar_scan and len(lidar_scan) > 0:
                    self.sensor_fusion.update_lidar(lidar_scan)
                    self.slam.update_with_lidar(lidar_scan)
            except Exception as e:
                pass

    def _update_odometry(self, dt: float):
        """
        Обновление одометрии на основе команд движения
        """
        # Параметры робота (примерные)
        max_speed = 0.15  # УМЕНЬШЕННАЯ максимальная скорость м/с при PWM=100
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

    def _safe_move_forward(self, max_duration=1.0, check_interval=0.2):
        """
        Безопасное движение вперед с постоянной проверкой препятствий
        
        Args:
            max_duration: максимальное время движения (сек)
            check_interval: интервал проверки препятствий (сек)
        
        Returns:
            bool: True если движение завершено безопасно, False если было препятствие
        """
        start_time = time.time()
        safe = True
        
        # Начинаем движение ОЧЕНЬ МЕДЛЕННО
        print(f"[SAFE_MOVE] Начинаю движение вперед на {max_duration:.1f}с")
        ca.move_forward(self.exploration_speed)
        self._set_movement_command("forward", self.exploration_speed)
        
        try:
            check_count = 0
            while time.time() - start_time < max_duration:
                # Ждем до следующей проверки
                time.sleep(check_interval)
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
                    # Если все еще нет данных, будем осторожны
                    print(f"[SAFE_MOVE] ⚠️ Проверка {check_count}: Нет данных датчиков")
                    
                    # После 3 проверок без данных - останавливаемся
                    if check_count >= 3:
                        print("[SAFE_MOVE] Слишком много проверок без данных, останавливаюсь")
                        safe = False
                        break
                    continue  # Продолжаем движение, но с осторожностью
                
                print(f"[SAFE_MOVE] Проверка {check_count}: препятствие = {obstacle_distance:.2f}м")
                
                # КРИТИЧЕСКОЕ расстояние - немедленная остановка
                if obstacle_distance < self.critical_distance:
                    print(f"[SAFE_MOVE] ⚠️ КРИТИЧЕСКОЕ РАССТОЯНИЕ! {obstacle_distance:.2f}м")
                    safe = False
                    break
                
                # Близкое препятствие - плавная остановка
                if obstacle_distance < self.min_obstacle_distance:
                    # Вычисляем насколько близко
                    closeness = (self.min_obstacle_distance - obstacle_distance) / self.min_obstacle_distance
                    
                    if closeness > 0.5:  # Очень близко (меньше половины min_distance)
                        print(f"[SAFE_MOVE] Очень близко: {obstacle_distance:.2f}м")
                        safe = False
                        break
                    else:
                        # Просто замедляемся
                        print(f"[SAFE_MOVE] Замедляюсь: {obstacle_distance:.2f}м")
                        # Можно добавить плавное замедление здесь
                
                # Все OK, продолжаем
                if check_count % 5 == 0:  # Реже выводим
                    print(f"[SAFE_MOVE] Двигаюсь... препятствие: {obstacle_distance:.2f}м")
                
        except Exception as e:
            print(f"[SAFE_MOVE] Ошибка при движении: {e}")
            safe = False
            
        finally:
            # Всегда останавливаемся плавно
            print("[SAFE_MOVE] Завершение движения...")
            
            # Плавная остановка
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.3)
            
        if safe:
            print(f"[SAFE_MOVE] ✓ Движение завершено безопасно ({check_count} проверок)")
        else:
            print(f"[SAFE_MOVE] ✗ Движение прервано из-за препятствия")
            
        return safe

    def _safe_rotate_left(self, max_duration=1.0, check_interval=0.2):
        """
        Безопасный поворот налево с проверкой
        
        Args:
            max_duration: максимальное время поворота (сек)
            check_interval: интервал проверки (сек)
        
        Returns:
            bool: True если поворот завершен безопасно
        """
        start_time = time.time()
        safe = True
        
        # Медленный поворот
        ca.rotate_left(self.turn_speed)
        self._set_movement_command("rotate_left", self.turn_speed)
        
        try:
            while time.time() - start_time < max_duration:
                time.sleep(check_interval)
                
                # Проверяем, не стало ли что-то слишком близко спереди
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

    def _safe_rotate_right(self, max_duration=1.0, check_interval=0.2):
        """Безопасный поворот направо"""
        start_time = time.time()
        safe = True
        
        ca.rotate_right(self.turn_speed)
        self._set_movement_command("rotate_right", self.turn_speed)
        
        try:
            while time.time() - start_time < max_duration:
                time.sleep(check_interval)
                
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

    def _emergency_stop_and_back(self):
        """Экстренная остановка и отъезд назад"""
        print("[SAFETY] ⚠️ ЭКСТРЕННАЯ ОСТАНОВКА!")
        
        # Немедленная остановка
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)
        
        # Отъезжаем немного назад
        print("[SAFETY] Отъезжаю назад...")
        ca.move_backward(self.exploration_speed)
        self._set_movement_command("backward", self.exploration_speed)
        time.sleep(0.8)
        
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)

    # ===== БЕЗОПАСНЫЕ ПОВЕДЕНИЯ =====

    def _safe_exploration_behavior(self):
        """БЕЗОПАСНОЕ поведение в режиме исследования"""
        
        # Получаем данные с датчиков
        obstacle_distance = self.sensor_fusion.get_obstacle_distance('front')
        
        # Отладка
        print(f"[NAV_DEBUG] Препятствие: {obstacle_distance:.2f}м")
        
        if obstacle_distance is None:
            print("[NAV_DEBUG] Нет данных от датчиков. Безопасный поворот...")
            self._safe_rotate_left(1.0)
            return
        
        # ЭКСТРЕННАЯ ОСТАНОВКА если слишком близко
        if obstacle_distance < self.critical_distance:
            self._emergency_stop_and_back()
            self._safe_rotate_left(1.0)
            return
        
        # Проверяем расстояние для нормального движения
        if obstacle_distance < self.min_obstacle_distance:
            print(f"[NAV_DEBUG] Препятствие на {obstacle_distance:.2f}м")
            print("[NAV_DEBUG] Безопасный поворот налево...")
            
            # Останавливаемся перед поворотом
            ca.stop_robot()
            self._set_movement_command("stop", 0)
            time.sleep(0.2)
            
            # Медленный поворот
            self._safe_rotate_left(1.0)
            
        else:
            print(f"[NAV_DEBUG] Свободно! {obstacle_distance:.2f}м")
            
            # Вычисляем безопасное время движения
            # Чем ближе препятствие, тем короче движение
            safe_time = min(2.0, (obstacle_distance - self.min_obstacle_distance) * 4)
            safe_time = max(0.5, safe_time)  # Минимум 0.5 секунды
            
            print(f"[NAV_DEBUG] Еду {safe_time:.1f} секунд...")
            
            # Безопасное движение вперед
            if not self._safe_move_forward(safe_time):
                # Если обнаружено препятствие во время движения
                print("[NAV_DEBUG] Обнаружено препятствие во время движения")
                self._safe_rotate_left(1.0)

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
                
                # Двигаемся короткий отрезок
                ca.move_forward(self.navigation_speed)
                self._set_movement_command("forward", self.navigation_speed)
                time.sleep(0.3)
                
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
        time.sleep(0.5)
        ca.stop_robot()
        self._set_movement_command("stop", 0)
        time.sleep(0.3)
        
        # Получаем свободные направления
        free_dirs = self.sensor_fusion.get_free_directions(self.min_obstacle_distance)
        
        # Выбираем безопасное направление
        if free_dirs.get('left', False):
            print("[NavController] Объезжаю слева")
            self._safe_rotate_left(0.8)
            safe_moved = self._safe_move_forward(1.0)
            
            if safe_moved:
                self._safe_rotate_right(0.8)
        elif free_dirs.get('right', False):
            print("[NavController] Объезжаю справа")
            self._safe_rotate_right(0.8)
            safe_moved = self._safe_move_forward(1.0)
            
            if safe_moved:
                self._safe_rotate_left(0.8)
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

    def quick_safety_test(self):
        """Быстрый тест безопасности движений"""
        print("="*60)
        print("ТЕСТ БЕЗОПАСНОСТИ - ОЧЕНЬ МЕДЛЕННОЕ ДВИЖЕНИЕ")
        print("="*60)
        
        # Тест очень медленного движения
        print("\n1. Очень медленное движение вперед (PWM=10)...")
        ca.move_forward(10)
        time.sleep(1.5)
        ca.stop_robot()
        time.sleep(0.5)
        
        print("\n2. Очень медленный поворот (PWM=8)...")
        ca.rotate_left(8)
        time.sleep(1.0)
        ca.stop_robot()
        time.sleep(0.5)
        
        print("\n3. Очень медленное движение назад (PWM=10)...")
        ca.move_backward(10)
        time.sleep(1.0)
        ca.stop_robot()
        
        print("\n✓ Тест завершен. Скорости безопасные.")


# Быстрый тест для проверки
if __name__ == "__main__":
    nav = NavigationController(use_lidar=False, use_camera=False, use_ultrasonic=True)
    nav.quick_safety_test()