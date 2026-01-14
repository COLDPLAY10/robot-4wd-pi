#!/usr/bin/env python3
"""
Планировщик маршрутов для робота
Реализует A* и Dynamic Window Approach (DWA)
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import heapq


@dataclass
class PathPoint:
    """Точка маршрута"""
    x: float
    y: float
    theta: Optional[float] = None


class PathPlanner:
    """
    Планировщик маршрутов
    - Глобальное планирование: A*
    - Локальное планирование: DWA
    """

    def __init__(self, occupancy_map):
        """
        Args:
            occupancy_map: OccupancyGrid из slam_core
        """
        self.map = occupancy_map
        self.current_global_path = []

        # Параметры DWA
        self.max_speed = 0.3        # м/с
        self.min_speed = 0.0
        self.max_yaw_rate = 1.0     # рад/с
        self.max_accel = 0.5        # м/с²
        self.max_delta_yaw_rate = 1.5  # рад/с²
        self.dt = 0.1               # шаг симуляции
        self.predict_time = 2.0     # время предсказания

        # Веса для DWA
        self.heading_weight = 1.0   # вес направления к цели
        self.clearance_weight = 0.5 # вес удаления от препятствий
        self.velocity_weight = 0.3  # вес скорости

        print("[PathPlanner] Инициализирован")

    def plan_global_path(self, start: Tuple[float, float],
                        goal: Tuple[float, float]) -> List[PathPoint]:
        """
        Глобальное планирование маршрута с помощью A*

        Args:
            start: начальная точка (x, y)
            goal: целевая точка (x, y)

        Returns:
            список точек PathPoint
        """
        print(f"[PathPlanner] Планирование пути: {start} -> {goal}")

        # Преобразуем в координаты сетки
        start_grid = self.map.world_to_grid(start[0], start[1])
        goal_grid = self.map.world_to_grid(goal[0], goal[1])

        # Проверка валидности точек
        if not self._is_valid_grid_point(start_grid):
            print("[PathPlanner] Стартовая точка невалидна")
            return []

        if not self._is_valid_grid_point(goal_grid):
            print("[PathPlanner] Целевая точка невалидна")
            return []

        # A* алгоритм
        path_grid = self._astar(start_grid, goal_grid)

        if not path_grid:
            print("[PathPlanner] Путь не найден")
            return []

        # Преобразуем обратно в мировые координаты
        path = []
        for gx, gy in path_grid:
            x, y = self.map.grid_to_world(gx, gy)
            path.append(PathPoint(x=x, y=y))

        # Упрощаем путь (убираем лишние точки на прямых участках)
        path = self._simplify_path(path)

        self.current_global_path = path
        print(f"[PathPlanner] Путь построен: {len(path)} точек")

        return path

    def compute_velocity_command(self, current_pose: Tuple[float, float, float],
                                 current_vel: Tuple[float, float],
                                 goal: Tuple[float, float]) -> Tuple[float, float]:
        """
        Вычисление команд управления с помощью DWA

        Args:
            current_pose: (x, y, theta)
            current_vel: (v, omega) - линейная и угловая скорость
            goal: целевая точка (x, y)

        Returns:
            (v, omega) - оптимальная линейная и угловая скорость
        """
        # Динамическое окно возможных скоростей
        dw = self._calculate_dynamic_window(current_vel)

        # Оцениваем все возможные траектории
        best_v = 0.0
        best_omega = 0.0
        best_score = -float('inf')

        # Шаги для поиска
        v_samples = np.linspace(dw[0], dw[1], 10)
        omega_samples = np.linspace(dw[2], dw[3], 15)

        for v in v_samples:
            for omega in omega_samples:
                # Симуляция траектории
                trajectory = self._predict_trajectory(current_pose, v, omega)

                # Проверка на столкновения
                if self._check_collision(trajectory):
                    continue

                # Оценка траектории
                score = self._evaluate_trajectory(trajectory, goal, v)

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_omega = omega

        return best_v, best_omega

    def _astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* алгоритм поиска пути"""

        def heuristic(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        # Очередь с приоритетом: (f_score, g_score, node)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        closed_set = set()

        # Направления движения (8 направлений)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                # Восстанавливаем путь
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            closed_set.add(current)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self._is_valid_grid_point(neighbor):
                    continue

                if neighbor in closed_set:
                    continue

                # Стоимость перехода
                move_cost = np.sqrt(dx*dx + dy*dy)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        return []  # Путь не найден

    def _calculate_dynamic_window(self, current_vel: Tuple[float, float]) -> Tuple[float, float, float, float]:
        """
        Вычисление динамического окна возможных скоростей

        Returns:
            (v_min, v_max, omega_min, omega_max)
        """
        v, omega = current_vel

        # Ограничения по ускорению
        v_min = max(self.min_speed, v - self.max_accel * self.dt)
        v_max = min(self.max_speed, v + self.max_accel * self.dt)

        omega_min = max(-self.max_yaw_rate, omega - self.max_delta_yaw_rate * self.dt)
        omega_max = min(self.max_yaw_rate, omega + self.max_delta_yaw_rate * self.dt)

        return v_min, v_max, omega_min, omega_max

    def _predict_trajectory(self, pose: Tuple[float, float, float],
                           v: float, omega: float) -> List[Tuple[float, float, float]]:
        """
        Предсказание траектории робота

        Args:
            pose: (x, y, theta)
            v: линейная скорость
            omega: угловая скорость

        Returns:
            список точек (x, y, theta)
        """
        trajectory = []
        x, y, theta = pose

        num_steps = int(self.predict_time / self.dt)

        for _ in range(num_steps):
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += omega * self.dt
            trajectory.append((x, y, theta))

        return trajectory

    def _check_collision(self, trajectory: List[Tuple[float, float, float]],
                        safety_margin: float = 0.2) -> bool:
        """
        Проверка траектории на столкновения

        Args:
            trajectory: список точек (x, y, theta)
            safety_margin: дополнительный отступ от препятствий

        Returns:
            True если есть столкновение
        """
        for x, y, _ in trajectory:
            if self.map.is_occupied(x, y, threshold=50):
                return True

            # Проверка вокруг точки (с учетом размера робота)
            robot_radius = 0.15 + safety_margin
            num_checks = 8
            for i in range(num_checks):
                angle = 2 * np.pi * i / num_checks
                check_x = x + robot_radius * np.cos(angle)
                check_y = y + robot_radius * np.sin(angle)

                if self.map.is_occupied(check_x, check_y, threshold=50):
                    return True

        return False

    def _evaluate_trajectory(self, trajectory: List[Tuple[float, float, float]],
                            goal: Tuple[float, float], velocity: float) -> float:
        """
        Оценка качества траектории

        Args:
            trajectory: список точек
            goal: целевая точка
            velocity: скорость движения

        Returns:
            оценка (чем выше, тем лучше)
        """
        if not trajectory:
            return -float('inf')

        # Последняя точка траектории
        end_x, end_y, end_theta = trajectory[-1]

        # 1. Оценка направления к цели
        dist_to_goal = np.sqrt((goal[0] - end_x)**2 + (goal[1] - end_y)**2)
        heading_score = -dist_to_goal  # чем ближе к цели, тем лучше

        # 2. Оценка удаления от препятствий
        clearance_score = self._calculate_clearance(trajectory)

        # 3. Оценка скорости (предпочитаем большую скорость)
        velocity_score = velocity / self.max_speed

        # Итоговая оценка
        total_score = (self.heading_weight * heading_score +
                      self.clearance_weight * clearance_score +
                      self.velocity_weight * velocity_score)

        return total_score

    def _calculate_clearance(self, trajectory: List[Tuple[float, float, float]]) -> float:
        """Вычисление минимального расстояния до препятствий вдоль траектории"""
        min_clearance = float('inf')

        for x, y, _ in trajectory:
            # Проверяем расстояние до ближайшего препятствия
            clearance = self._distance_to_nearest_obstacle(x, y)
            min_clearance = min(min_clearance, clearance)

        # Нормализуем (максимум 2 метра)
        return min(min_clearance / 2.0, 1.0)

    def _distance_to_nearest_obstacle(self, x: float, y: float) -> float:
        """Расстояние до ближайшего препятствия"""
        search_radius = 2.0
        num_samples = 16

        min_dist = search_radius

        for i in range(num_samples):
            angle = 2 * np.pi * i / num_samples
            for r in np.linspace(0.1, search_radius, 10):
                check_x = x + r * np.cos(angle)
                check_y = y + r * np.sin(angle)

                if self.map.is_occupied(check_x, check_y):
                    min_dist = min(min_dist, r)
                    break

        return min_dist

    def _simplify_path(self, path: List[PathPoint], epsilon: float = 0.1) -> List[PathPoint]:
        """
        Упрощение пути (удаление лишних точек)
        Используется алгоритм Ramer-Douglas-Peucker
        """
        if len(path) < 3:
            return path

        # Простое упрощение: удаляем точки на прямых участках
        simplified = [path[0]]

        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_p = path[i + 1]

            # Проверяем, лежат ли точки примерно на одной прямой
            dist_to_line = self._point_to_line_distance(
                (curr.x, curr.y),
                (prev.x, prev.y),
                (next_p.x, next_p.y)
            )

            if dist_to_line > epsilon:
                simplified.append(curr)

        simplified.append(path[-1])

        return simplified

    def _is_valid_grid_point(self, point: Tuple[int, int]) -> bool:
        """Проверка валидности точки на сетке"""
        gx, gy = point

        if not (0 <= gx < self.map.width and 0 <= gy < self.map.height):
            return False

        # Проверка на занятость (с небольшим порогом)
        return self.map.grid[gy, gx] < 70

    @staticmethod
    def _point_to_line_distance(point, line_start, line_end):
        """Расстояние от точки до прямой"""
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        # Вектор прямой
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return np.sqrt((px - x1)**2 + (py - y1)**2)

        # Проекция точки на прямую
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

        # Ближайшая точка на отрезке
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return np.sqrt((px - closest_x)**2 + (py - closest_y)**2)

