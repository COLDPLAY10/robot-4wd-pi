#!/usr/bin/env python3
"""
Hector-style scan-to-map matching на 2D occupancy grid.

Идея: занятые клетки карты задают "likelihood field" — поле расстояний
до ближайшего препятствия (через distance transform). Для текущего скана
ищем позу робота (x, y, theta), при которой сумма расстояний от точек
скана до ближайших препятствий минимальна. Решаем Gauss-Newton'ом с
билинейной интерполяцией поля и его градиента.

Из этого получаем настоящий SLAM: позиция корректируется по сканам,
а не накапливает ошибку одометрии бесконечно.

Литература: S. Kohlbrecher et al., "A Flexible and Scalable SLAM System
with Full 3D Motion Estimation" (Hector SLAM), 2011.
"""

import time
from typing import List, Optional, Tuple

import numpy as np

# cv2 нужен для быстрого distanceTransform. Если его нет (например, дев-машина
# без opencv) — используем медленный numpy-fallback. На Raspberry Pi opencv
# обязателен — он уже в requirements.txt.
try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore
    _HAS_CV2 = False


def _distance_transform_numpy(binary: np.ndarray, max_iters: int = 400) -> np.ndarray:
    """
    Приближённый евклидов distance transform на чистом numpy.
    Итеративный 8-связный min-filter с шагами (1, √2).

    Используется только как fallback, когда cv2 недоступен. Сходимость за
    max_iters итераций — для карт 400×400 с препятствиями на расстоянии
    ≤max_iters пикселей даёт верный результат.

    Args:
        binary: H×W uint8, 0 = препятствие, !=0 = свободно
    Returns:
        float32 H×W, расстояние в пикселях
    """
    INF = np.float32(1e6)
    d = np.where(binary == 0, np.float32(0.0), INF)
    sq2 = np.float32(np.sqrt(2.0))
    one = np.float32(1.0)
    for _ in range(max_iters):
        prev = d
        d = d.copy()
        d[:, 1:] = np.minimum(d[:, 1:], prev[:, :-1] + one)
        d[:, :-1] = np.minimum(d[:, :-1], prev[:, 1:] + one)
        d[1:, :] = np.minimum(d[1:, :], prev[:-1, :] + one)
        d[:-1, :] = np.minimum(d[:-1, :], prev[1:, :] + one)
        d[1:, 1:] = np.minimum(d[1:, 1:], prev[:-1, :-1] + sq2)
        d[1:, :-1] = np.minimum(d[1:, :-1], prev[:-1, 1:] + sq2)
        d[:-1, 1:] = np.minimum(d[:-1, 1:], prev[1:, :-1] + sq2)
        d[:-1, :-1] = np.minimum(d[:-1, :-1], prev[1:, 1:] + sq2)
        if np.array_equal(d, prev):
            break
    return d


class HectorScanMatcher:
    """
    Scan-to-map matcher по полю расстояний.
    Один и тот же класс работает и в режиме картирования, и в режиме
    локализации по готовой карте — отличие только в том, кто и когда
    обновляет occupancy grid снаружи.
    """

    def __init__(self,
                 resolution: float,
                 occupied_threshold: float = 65.0,
                 known_threshold: float = 1.5,
                 max_iters: int = 14,
                 convergence_tol: float = 1e-4,
                 field_refresh_seconds: float = 0.4,
                 match_acceptance_m: float = 0.18,
                 min_scan_points: int = 30,
                 max_pose_jump_m: float = 0.8,
                 max_pose_jump_rad: float = np.deg2rad(45)):
        """
        Args:
            resolution: размер ячейки карты в метрах
            occupied_threshold: значение 0..100, выше которого ячейка считается препятствием
            known_threshold: насколько ячейка должна отклоняться от 50 (prior), чтобы
                             считать её "виденной" — точки в незнакомые области выбрасываем
            max_iters: максимум итераций Gauss-Newton. GN на likelihood field
                       сходится за единицы итераций (есть convergence_tol —
                       ранний выход). 14 вместо 30 срезает латентность тика: на
                       плохих сканах матчер упирался в 30 итераций каждый оборот
                       (~tens of ms на горячем пути), не улучшая результат
            convergence_tol: норма шага, после которой останавливаемся
            field_refresh_seconds: как часто пересчитывать distance transform
            match_acceptance_m: средняя дистанция точек скана до карты, выше — матч плохой
            min_scan_points: минимум точек скана, иначе матч не делаем
            max_pose_jump_m, max_pose_jump_rad: ограничение прыжка позы относительно прайра —
                             защита от расхождения матчера; больше — откат на одометрию
        """
        self.resolution = resolution
        self.occupied_threshold = occupied_threshold
        self.known_threshold = known_threshold
        self.max_iters = max_iters
        self.convergence_tol = convergence_tol
        self.field_refresh_seconds = field_refresh_seconds
        self.match_acceptance_m = match_acceptance_m
        self.min_scan_points = min_scan_points
        self.max_pose_jump_m = max_pose_jump_m
        self.max_pose_jump_rad = max_pose_jump_rad

        # Кэш likelihood field и его градиента
        self._dist_field: Optional[np.ndarray] = None
        self._grad_x: Optional[np.ndarray] = None
        self._grad_y: Optional[np.ndarray] = None
        self._known_mask: Optional[np.ndarray] = None
        self._field_built_at: float = 0.0
        self._field_origin: Tuple[int, int] = (0, 0)
        self._field_shape: Tuple[int, int] = (0, 0)

        # Статистика для отладки / защиты диплома
        self.last_iterations: int = 0
        self.last_match_score: Optional[float] = None
        self.last_match_accepted: bool = False
        self.last_match_reason: str = ""
        self.total_matches: int = 0
        self.accepted_matches: int = 0

    # ------------------------------------------------------------------ field

    def has_field(self) -> bool:
        return self._dist_field is not None

    def needs_refresh(self) -> bool:
        return (not self.has_field()) or (
            time.time() - self._field_built_at > self.field_refresh_seconds
        )

    def update_likelihood_field(self, occupancy_grid) -> None:
        """
        Пересчитать likelihood field из текущей карты.

        Args:
            occupancy_grid: объект с атрибутами grid (H×W float 0..100),
                            origin_x, origin_y, width, height
        """
        grid = occupancy_grid.grid

        # Препятствия = 0, всё остальное = 255 — distanceTransform даёт
        # расстояние до ближайшего нуля (в пикселях).
        binary = np.where(grid >= self.occupied_threshold, 0, 255).astype(np.uint8)

        # Если препятствий нет вообще, distanceTransform даёт большие константы —
        # матчинг бессмыслен, помечаем поле как отсутствующее.
        if np.all(binary == 255):
            self._dist_field = None
            self._grad_x = None
            self._grad_y = None
            self._known_mask = None
            return

        if _HAS_CV2:
            dist_px = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        else:
            dist_px = _distance_transform_numpy(binary)
        self._dist_field = dist_px.astype(np.float32) * self.resolution  # в метрах

        # Градиент likelihood field: единицы — метров на пиксель.
        # np.gradient возвращает (d/d_axis0, d/d_axis1) = (d/d_row, d/d_col) = (d/dy, d/dx)
        gy, gx = np.gradient(self._dist_field)
        self._grad_x = gx.astype(np.float32)
        self._grad_y = gy.astype(np.float32)

        # Маска "виденных" клеток — где SLAM реально что-то знает.
        # Помогает не штрафовать точки скана, попавшие в "0.5 неизвестно".
        self._known_mask = (np.abs(grid - 50.0) > self.known_threshold)

        self._field_origin = (occupancy_grid.origin_x, occupancy_grid.origin_y)
        self._field_shape = (occupancy_grid.height, occupancy_grid.width)
        self._field_built_at = time.time()

    # ------------------------------------------------------------------ match

    def match(self,
              scan_data: List[Tuple[float, float]],
              prior_pose: Tuple[float, float, float],
              ) -> Optional[Tuple[Tuple[float, float, float], float]]:
        """
        Уточнить позу робота по скану.

        Args:
            scan_data: список (angle_rad, distance_m)
            prior_pose: (x, y, theta) — лучший прогноз позы из одометрии

        Returns:
            ((x, y, theta), match_score) если матч принят, иначе None.
            match_score — средний residual в метрах.
        """
        self.total_matches += 1
        self.last_match_accepted = False
        self.last_match_score = None

        if not self.has_field():
            self.last_match_reason = "no_field"
            return None
        if len(scan_data) < self.min_scan_points:
            self.last_match_reason = "scan_too_small"
            return None

        # Конвертация скана в локальные декартовы координаты робота.
        # Отбрасываем явно битые точки.
        pts = []
        for a, d in scan_data:
            if 0.15 < d < 10.0:
                pts.append((d * np.cos(a), d * np.sin(a)))
        if len(pts) < self.min_scan_points:
            self.last_match_reason = "filtered_scan_too_small"
            return None
        local = np.asarray(pts, dtype=np.float32)

        ox, oy = self._field_origin
        h, w = self._field_shape
        inv_res = 1.0 / self.resolution

        x, y, theta = prior_pose
        prior_x, prior_y, prior_theta = prior_pose

        last_step = np.inf
        for it in range(self.max_iters):
            self.last_iterations = it + 1

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            # Точки скана в мировых координатах
            wx = x + cos_t * local[:, 0] - sin_t * local[:, 1]
            wy = y + sin_t * local[:, 0] + cos_t * local[:, 1]

            # В пиксели карты
            px = wx * inv_res + ox
            py = wy * inv_res + oy

            x0 = np.floor(px).astype(np.int32)
            y0 = np.floor(py).astype(np.int32)

            # Только точки, у которых вся 2×2 окрестность билинейного сэмпла
            # внутри карты и попадает в уже виденную область.
            inside = (x0 >= 0) & (x0 < w - 1) & (y0 >= 0) & (y0 < h - 1)
            if inside.sum() < self.min_scan_points:
                self.last_match_reason = "no_points_inside_map"
                return None

            x0i = x0[inside]
            y0i = y0[inside]

            known = (self._known_mask[y0i, x0i] |
                     self._known_mask[y0i + 1, x0i] |
                     self._known_mask[y0i, x0i + 1] |
                     self._known_mask[y0i + 1, x0i + 1])
            if known.sum() < self.min_scan_points:
                self.last_match_reason = "no_points_in_known_area"
                return None

            # Финальный отбор индексов
            keep_local_idx = np.where(inside)[0][known]
            x0k = x0i[known]
            y0k = y0i[known]
            dx = (px[keep_local_idx] - x0k).astype(np.float32)
            dy = (py[keep_local_idx] - y0k).astype(np.float32)

            # Билинейная выборка значения и градиента
            def _sample(field: np.ndarray) -> np.ndarray:
                v00 = field[y0k, x0k]
                v01 = field[y0k, x0k + 1]
                v10 = field[y0k + 1, x0k]
                v11 = field[y0k + 1, x0k + 1]
                return ((1.0 - dy) * ((1.0 - dx) * v00 + dx * v01) +
                        dy * ((1.0 - dx) * v10 + dx * v11))

            residuals = _sample(self._dist_field)
            # ∂M/∂world: np.gradient — это (м на пиксель), ∂/∂world = ∂/∂pixel × inv_res
            grad_world_x = _sample(self._grad_x) * inv_res
            grad_world_y = _sample(self._grad_y) * inv_res

            # ∂world/∂theta для каждой точки
            lx = local[keep_local_idx, 0]
            ly = local[keep_local_idx, 1]
            d_wx_dtheta = -sin_t * lx - cos_t * ly
            d_wy_dtheta = cos_t * lx - sin_t * ly

            # Jacobian (N×3): ∂residual/∂(x, y, theta)
            j_theta = grad_world_x * d_wx_dtheta + grad_world_y * d_wy_dtheta
            J = np.column_stack([grad_world_x, grad_world_y, j_theta]).astype(np.float32)

            # Levenberg-Marquardt-подобный шаг: H + λI для устойчивости
            H = J.T @ J
            H[0, 0] += 1e-3
            H[1, 1] += 1e-3
            H[2, 2] += 1e-3
            g = J.T @ residuals

            try:
                delta = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                self.last_match_reason = "singular_hessian"
                return None

            x += float(delta[0])
            y += float(delta[1])
            theta += float(delta[2])
            theta = float(np.arctan2(np.sin(theta), np.cos(theta)))

            step_norm = float(np.linalg.norm(delta))
            if step_norm < self.convergence_tol:
                break
            # Защита от расхождения: если шаги растут, выходим
            if step_norm > 5.0 * last_step and it > 3:
                self.last_match_reason = "diverged"
                return None
            last_step = step_norm

        # Финальная оценка качества по принятым точкам
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        wx = x + cos_t * local[:, 0] - sin_t * local[:, 1]
        wy = y + sin_t * local[:, 0] + cos_t * local[:, 1]
        px = wx * inv_res + ox
        py = wy * inv_res + oy
        ix = np.clip(np.round(px).astype(np.int32), 0, w - 1)
        iy = np.clip(np.round(py).astype(np.int32), 0, h - 1)
        known_final = self._known_mask[iy, ix]
        if known_final.sum() < self.min_scan_points:
            self.last_match_reason = "no_points_known_final"
            return None
        score = float(np.mean(self._dist_field[iy[known_final], ix[known_final]]))
        self.last_match_score = score

        # Гейт качества
        if score > self.match_acceptance_m:
            self.last_match_reason = f"score_too_high({score:.3f}>{self.match_acceptance_m})"
            return None

        # Гейт большого прыжка относительно прайра — защита от схождения в чужой минимум
        d_pos = float(np.hypot(x - prior_x, y - prior_y))
        d_theta = abs(np.arctan2(np.sin(theta - prior_theta), np.cos(theta - prior_theta)))
        if d_pos > self.max_pose_jump_m or d_theta > self.max_pose_jump_rad:
            self.last_match_reason = (
                f"pose_jump(d={d_pos:.2f}m, dθ={np.rad2deg(d_theta):.1f}°)"
            )
            return None

        self.last_match_accepted = True
        self.last_match_reason = "ok"
        self.accepted_matches += 1
        return (x, y, theta), score

    def get_stats(self) -> dict:
        """Сводка для логов и отчётов."""
        rate = (self.accepted_matches / self.total_matches) if self.total_matches else 0.0
        return {
            "total": self.total_matches,
            "accepted": self.accepted_matches,
            "acceptance_rate": rate,
            "last_score": self.last_match_score,
            "last_iters": self.last_iterations,
            "last_reason": self.last_match_reason,
        }