#!/usr/bin/env python3
"""
Планировщик маршрутов для робота
Реализует A* и Dynamic Window Approach (DWA)
"""

import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
import heapq

import numpy as np

# cv2.dilate — быстрый путь для inflation; на дев-машине без opencv падаем
# на numpy-фоллбэк через итеративный 8-связный max-filter.
try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore
    _HAS_CV2 = False


def _binary_dilate_numpy(binary: np.ndarray, radius_px: int) -> np.ndarray:
    """8-связное расширение бинарной маски на radius_px пикселей."""
    out = binary.copy()
    for _ in range(radius_px):
        prev = out
        out = prev.copy()
        out[:, 1:] |= prev[:, :-1]
        out[:, :-1] |= prev[:, 1:]
        out[1:, :] |= prev[:-1, :]
        out[:-1, :] |= prev[1:, :]
        out[1:, 1:] |= prev[:-1, :-1]
        out[1:, :-1] |= prev[:-1, 1:]
        out[:-1, 1:] |= prev[1:, :-1]
        out[:-1, :-1] |= prev[1:, 1:]
    return out


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

    def __init__(self, occupancy_map,
                 robot_radius: float = 0.10,
                 inflation_margin: float = 0.05,
                 occupied_threshold: float = 65.0,
                 inflation_refresh_seconds: float = 0.5):
        """
        Args:
            occupancy_map: OccupancyGrid из slam_core
            robot_radius: радиус робота (м) — клетки в пределах этого расстояния от
                          препятствия запрещены для A* и DWA
            inflation_margin: дополнительный запас безопасности (м)
            occupied_threshold: 0..100, выше — клетка считается препятствием
                                для inflation (отделяем от threshold проверки is_occupied)
            inflation_refresh_seconds: throttle для пересчёта inflated grid
        """
        self.map = occupancy_map
        self.current_global_path = []

        # Параметры DWA (реальные пределы прокидывает NavigationController —
        # single source of truth по скоростям там)
        self.max_speed = 0.15       # м/с
        self.min_speed = 0.0
        self.max_yaw_rate = 1.2     # рад/с
        self.max_accel = 0.4        # м/с²
        self.max_delta_yaw_rate = 2.0  # рад/с²
        self.dt = 0.15              # шаг симуляции траектории
        self.predict_time = 2.2     # горизонт предсказания, с
        self.window_dt = 0.3        # «ширина» динамического окна по времени

        # Веса для DWA
        self.heading_weight = 1.0   # вес прогресса к цели
        self.align_weight = 0.8     # вес ориентации на цель в конце дуги
        self.clearance_weight = 0.5 # вес удаления от препятствий
        self.velocity_weight = 0.3  # вес скорости

        # Inflation cost-map — расширяем препятствия на размер робота, чтобы
        # A* не строил пути, по которым робот не пролезет геометрически.
        self.robot_radius = robot_radius
        self.inflation_margin = inflation_margin
        self.occupied_threshold = occupied_threshold
        self.inflation_refresh_seconds = inflation_refresh_seconds
        self._inflated_grid: Optional[np.ndarray] = None
        # Поле расстояний до ближайшего препятствия (м) — clearance для DWA
        # за O(1) на точку вместо лучевого сканирования карты.
        self._clearance_field: Optional[np.ndarray] = None
        self._inflation_built_at: float = 0.0

        print(f"[PathPlanner] Инициализирован "
              f"(robot_radius={robot_radius:.2f}м, margin={inflation_margin:.2f}м)")

    # ---------------------------------------------------------------- inflation

    def _ensure_inflation(self, force: bool = False) -> None:
        """Перестроить inflated grid из текущей карты, не чаще раза в N секунд."""
        now = time.time()
        if (not force) and self._inflated_grid is not None and \
                (now - self._inflation_built_at) < self.inflation_refresh_seconds:
            return

        radius_total = self.robot_radius + self.inflation_margin
        radius_px = max(1, int(np.ceil(radius_total / self.map.resolution)))

        occupied = (self.map.grid >= self.occupied_threshold).astype(np.uint8)

        if _HAS_CV2:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1)
            )
            inflated = cv2.dilate(occupied, kernel)
        else:
            inflated = _binary_dilate_numpy(occupied.astype(bool), radius_px).astype(np.uint8)

        self._inflated_grid = inflated

        # Поле расстояний от СЫРЫХ препятствий (не inflated) — clearance-метрика
        # DWA. Если препятствий нет, поле не нужно.
        if occupied.any():
            binary = np.where(occupied > 0, 0, 255).astype(np.uint8)
            if _HAS_CV2:
                dist_px = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
            else:
                from .scan_matcher import _distance_transform_numpy
                dist_px = _distance_transform_numpy(binary)
            self._clearance_field = dist_px.astype(np.float32) * self.map.resolution
        else:
            self._clearance_field = None

        self._inflation_built_at = now

    def _is_blocked(self, gx: int, gy: int) -> bool:
        """Заблокирована ли клетка после inflation."""
        if not (0 <= gx < self.map.width and 0 <= gy < self.map.height):
            return True
        return bool(self._inflated_grid[gy, gx])

    # ---------------------------------------------------------------- planning

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

        # Перестраиваем inflation от текущей карты — A* поедет по нему.
        self._ensure_inflation()

        # Преобразуем в координаты сетки
        start_grid = self.map.world_to_grid(start[0], start[1])
        goal_grid = self.map.world_to_grid(goal[0], goal[1])

        # Стартовая точка часто оказывается "впритык" к стене после inflation —
        # робот физически находится там, поэтому off-by-один пиксель не должен
        # ронять планирование. Если старт в inflated-зоне, считаем его валидным.
        if not self._is_valid_grid_point(start_grid, allow_inflated=True):
            print("[PathPlanner] Стартовая точка невалидна (за границей карты)")
            return []

        if not self._is_valid_grid_point(goal_grid):
            print("[PathPlanner] Целевая точка невалидна (в inflated-зоне или вне карты)")
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
                                 goal: Tuple[float, float],
                                 v_max_cap: Optional[float] = None) -> Tuple[float, float]:
        """
        Локальный планировщик DWA: выбрать (v, omega) на ближайший такт.

        Векторизованная реализация: все кандидаты окна симулируются разом
        закрытой формой дуги, коллизии — по inflated grid (размер робота уже
        учтён в раздувании), clearance — по полю расстояний за O(1) на точку.
        На Pi укладывается в единицы миллисекунд — пригодно для вызова на
        каждом тике контура (10–20 Гц).

        Args:
            current_pose: (x, y, theta)
            current_vel: (v, omega) — текущая (последняя командованная) скорость
            goal: целевая точка (x, y) — обычно текущий waypoint A*
            v_max_cap: реактивное ограничение скорости сверху (м/с) — например,
                       по фронтальной дистанции sensor fusion; None = без капа

        Returns:
            (v, omega) — лучшая пара; (0, 0) если ни одна траектория не проходит
            (всё в коллизии) — вызывающий обязан трактовать это как «стоп».
        """
        # Свежий inflated grid + clearance field перед оценкой траекторий
        self._ensure_inflation()
        if self._inflated_grid is None:
            return 0.0, 0.0

        x0, y0, th0 = current_pose
        v0, w0 = current_vel

        # --- Динамическое окно: достижимые скорости с учётом ускорений ---
        v_lo = max(self.min_speed, v0 - self.max_accel * self.window_dt)
        v_hi = min(self.max_speed, v0 + self.max_accel * self.window_dt)
        if v_max_cap is not None:
            v_hi = min(v_hi, max(0.0, v_max_cap))
            v_lo = min(v_lo, v_hi)
        w_lo = max(-self.max_yaw_rate, w0 - self.max_delta_yaw_rate * self.window_dt)
        w_hi = min(self.max_yaw_rate, w0 + self.max_delta_yaw_rate * self.window_dt)

        V, W = np.meshgrid(np.linspace(v_lo, v_hi, 7),
                           np.linspace(w_lo, w_hi, 13))
        V = V.ravel().astype(np.float32)   # M кандидатов
        W = W.ravel().astype(np.float32)

        # --- Траектории закрытой формой: матрица (M, N) точек дуг ---
        n_steps = max(2, int(self.predict_time / self.dt))
        t = (np.arange(1, n_steps + 1, dtype=np.float32) * self.dt)[None, :]
        Vc = V[:, None]
        Wc = W[:, None]
        w_safe = np.where(np.abs(Wc) < 1e-6, 1e-6, Wc)
        th = th0 + Wc * t
        arc_x = x0 + Vc / w_safe * (np.sin(th) - np.sin(th0))
        arc_y = y0 - Vc / w_safe * (np.cos(th) - np.cos(th0))
        lin_x = x0 + Vc * t * np.cos(th0)
        lin_y = y0 + Vc * t * np.sin(th0)
        straight = np.abs(Wc) < 1e-6
        X = np.where(straight, lin_x, arc_x)
        Y = np.where(straight, lin_y, arc_y)

        # --- Коллизии: по НЕПРЕРЫВНОМУ полю расстояний, не по бинарной решётке ---
        # Бинарный inflated grid у границ хрупок: ceil радиуса в пикселях
        # завышает раздувание почти на клетку, и робот, легально стоящий у
        # границы (шум позы на железе!), видит «коллизию» в первой же точке
        # любой дуги → вечный (0,0)-дедлок. Поэтому DWA проверяет точки по
        # полю расстояний: запрещено всё ближе radius_total к препятствию,
        # НО не дальше, чем робот уже находится (escape-правило: из «тесного»
        # места можно выскальзывать вдоль границы, нельзя углубляться). Тот же
        # принцип, что allow_inflated у стартовой точки A*.
        gx = (X / self.map.resolution + self.map.origin_x).astype(np.int32)
        gy = (Y / self.map.resolution + self.map.origin_y).astype(np.int32)
        inside = ((gx >= 0) & (gx < self.map.width)
                  & (gy >= 0) & (gy < self.map.height))
        blocked = np.ones(X.shape, dtype=bool)  # вне карты = заблокировано

        if self._clearance_field is None:
            # Препятствий на карте нет вообще
            blocked[inside] = False
        else:
            radius_total = self.robot_radius + self.inflation_margin
            rgx, rgy = self.map.world_to_grid(x0, y0)
            if 0 <= rgx < self.map.width and 0 <= rgy < self.map.height:
                clear_here = float(self._clearance_field[rgy, rgx])
            else:
                clear_here = radius_total
            # Смягчённый порог действует только на НАЧАЛЬНОМ участке дуги
            # (не больше половины горизонта — иначе при коротком горизонте
            # 0.35 м «окно» 0.4 м покрывало всю дугу и робот мог легально
            # ползти впритирку к стене бесконечно). Достаточно, чтобы
            # выскользнуть из тесного места; дальше — полный клиренс
            # (боковое касание реактивный слой не страхует).
            low_thr = min(radius_total, clear_here)
            escape_len = min(0.4, 0.5 * self.max_speed * self.predict_time)
            dist_along = Vc * t  # (M, N): пройденный путь вдоль дуги
            threshold = np.where(dist_along <= escape_len, low_thr, radius_total)
            blocked[inside] = (self._clearance_field[gy[inside], gx[inside]]
                               < threshold[inside] - 1e-6)

        feasible = ~blocked.any(axis=1)
        if not feasible.any():
            return 0.0, 0.0

        # --- Clearance: минимум поля расстояний вдоль дуги, насыщение 0.5 м ---
        if self._clearance_field is not None:
            cf = np.zeros(X.shape, dtype=np.float32)
            cf[inside] = self._clearance_field[gy[inside], gx[inside]]
            clearance_score = np.clip(cf.min(axis=1) / 0.5, 0.0, 1.0)
        else:
            clearance_score = np.ones(len(V), dtype=np.float32)

        # --- Прогресс к цели + ориентация на цель в конце дуги ---
        dist_now = float(np.hypot(goal[0] - x0, goal[1] - y0))
        ex, ey, eth = X[:, -1], Y[:, -1], th[:, -1]
        dist_end = np.hypot(goal[0] - ex, goal[1] - ey)
        progress = np.clip(
            (dist_now - dist_end) / max(1e-6, self.max_speed * self.predict_time),
            -1.0, 1.0)
        ang_err = np.arctan2(goal[1] - ey, goal[0] - ex) - eth
        ang_err = np.abs(np.arctan2(np.sin(ang_err), np.cos(ang_err)))
        align = (np.pi - ang_err) / np.pi
        velocity_score = V / max(1e-6, self.max_speed)

        total = (self.heading_weight * progress
                 + self.align_weight * align
                 + self.clearance_weight * clearance_score
                 + self.velocity_weight * velocity_score)
        total[~feasible] = -np.inf

        best = int(np.argmax(total))
        return float(V[best]), float(W[best])

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

    def _simplify_path(self, path: List[PathPoint]) -> List[PathPoint]:
        """
        Упрощение пути методом line-of-sight («натяжение нити»): от текущей
        точки тянем прямой сегмент к самой дальней точке пути, до которой он
        проходит по СВОБОДНЫМ клеткам inflated grid, — она становится
        следующим waypoint'ом.

        Прежняя реализация (проверка отклонения точки от хорды к СОСЕДНЕЙ
        точке) была сломана: отклонение от хорды до соседа всегда меньше
        клетки, поэтому ЛЮБОЙ путь схлопывался в прямую «старт-финиш» — даже
        сквозь стены. LOS-вариант гарантирует, что каждый сегмент проходим.
        """
        if len(path) < 3:
            return path

        simplified = [path[0]]
        i = 0
        last = len(path) - 1
        while i < last:
            j = last
            while j > i + 1 and not self._segment_free(path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified

    def _segment_free(self, a: PathPoint, b: PathPoint) -> bool:
        """Проходит ли отрезок a→b целиком по свободным клеткам inflated grid."""
        dist = float(np.hypot(b.x - a.x, b.y - a.y))
        n = max(2, int(dist / (self.map.resolution * 0.5)))
        for k in range(n + 1):
            t = k / n
            gx, gy = self.map.world_to_grid(a.x + t * (b.x - a.x),
                                            a.y + t * (b.y - a.y))
            if self._is_blocked(gx, gy):
                return False
        return True

    def _is_valid_grid_point(self, point: Tuple[int, int],
                             allow_inflated: bool = False) -> bool:
        """
        Проверка валидности точки на сетке.

        Args:
            point: (gx, gy) в координатах сетки
            allow_inflated: если True, разрешаем inflated-зону (для стартовой
                            позиции робота — он уже в этой клетке физически)
        """
        gx, gy = point

        if not (0 <= gx < self.map.width and 0 <= gy < self.map.height):
            return False

        if allow_inflated:
            # Только защита от настоящих препятствий, без расширения
            return self.map.grid[gy, gx] < 70

        # Полный inflated test — после _ensure_inflation()
        if self._inflated_grid is None:
            # До первого _ensure_inflation падаем на старый поведение,
            # чтобы тесты вне планировщика работали как раньше.
            return self.map.grid[gy, gx] < 70
        return not bool(self._inflated_grid[gy, gx])

