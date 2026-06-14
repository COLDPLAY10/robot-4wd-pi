#!/usr/bin/env python3
"""
Автокалибровка наклона и высоты камеры по сегментированному полу.

Идея: нейросеть (FloorSegmenter) даёт маску пикселей пола. Берём ИХ глубину,
обратно проецируем в 3D и подгоняем плоскость. Из ориентации и удаления этой
плоскости восстанавливаем фактический наклон (tilt) и высоту (height) камеры.
Закрывает открытый вопрос §6 (горизонтален ли взгляд при tilt серво 25°):
в конфиге сейчас CAMERA_MOUNT_TILT_RAD = 0.0, хотя серво стоит под 25° — это
противоречие и разрешает калибровка.

КРИТИЧНО — проекция здесь в систему КАМЕРЫ, БЕЗ применения mount (в отличие от
camera_perception.projection.backproject_depth_to_world, который применяет
height/tilt). Если применить mount, а потом «восстановить» его из плоскости,
получишь обратно ровно то, что уже задано в конфиге, — самосбывающийся
результат, который выглядит правдоподобно и ничего не значит. Поэтому mount
выводится ИЗ плоскости, а не закладывается в неё.

Система координат камеры (как в projection.py): x — вправо, y — ВНИЗ,
z — вперёд (вдоль оптической оси), глубина = z.

Связь наклона с нормалью пола (вывод из прямого преобразования camera->robot
в projection.py): для камеры с наклоном t нормаль пола в кадре камеры,
направленная к камере (вверх), равна
    n = (0, -cos t, -sin t).
Отсюда  tilt = atan2(-n_z, -n_y),  высота = |n · centroid| (перпендикуляр от
оптического центра до плоскости пола = вертикальная высота, т.к. пол
горизонтален).
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class FloorCalibration:
    """Результат подгонки плоскости пола. tilt_rad/height_m — paste-ready в mount."""
    tilt_rad: float          # наклон камеры вниз (>0 = смотрит вниз), рад
    tilt_deg: float          # то же в градусах — для удобства
    height_m: float          # высота оптического центра над полом, м
    normal: np.ndarray       # нормаль плоскости в системе камеры (к камере), (3,)
    n_points: int            # точек пола, по которым велась подгонка
    inlier_ratio: float      # доля inliers после робастного отсева
    rms_residual_m: float    # RMS расстояния inliers до плоскости, м
    roll_abs: float          # |n_x| — индикатор крена/мусора (в норме ~0)
    ok: bool                 # прошла ли подгонка гейты качества


def _fit_plane(points: np.ndarray):
    """PCA-подгонка плоскости: centroid + нормаль (собств. вектор наим. с.з.)."""
    centroid = points.mean(axis=0)
    q = points - centroid
    cov = q.T @ q
    eigvals, eigvecs = np.linalg.eigh(cov)   # eigh: с.з. по возрастанию
    normal = eigvecs[:, 0]                    # наименьшее с.з. = нормаль плоскости
    return normal, centroid


def estimate_floor_plane(
    depth_map: np.ndarray,
    floor_mask: np.ndarray,
    intrinsics,
    pixel_stride: int = 8,
    min_depth_m: float = 0.2,
    max_depth_m: float = 4.0,
    inlier_thresh_m: float = 0.04,
    min_points: int = 200,
    max_tilt_for_ok_deg: float = 60.0,
) -> Optional[FloorCalibration]:
    """
    Оценить наклон/высоту камеры по floor-маске и карте глубины.

    Args:
        depth_map: H×W float, метрическая глубина (м) вдоль оптической оси
        floor_mask: H×W bool, True = пиксель пола (из FloorSegmenter)
        intrinsics: CameraIntrinsics (fx, fy, cx, cy) — БЕЗ mount
        pixel_stride: подвыборка пикселей (скорость)
        min/max_depth_m: валидный диапазон глубины пола
        inlier_thresh_m: порог отсева outliers (битая глубина, не-пол в маске)
        min_points: минимум точек пола, иначе None
        max_tilt_for_ok_deg: правдоподобный предел наклона; больше — ok=False

    Returns:
        FloorCalibration, либо None если точек пола слишком мало.
    """
    if depth_map is None or floor_mask is None:
        return None
    h, w = depth_map.shape[:2]

    vs = np.arange(0, h, pixel_stride)
    us = np.arange(0, w, pixel_stride)
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel(); vv = vv.ravel()

    z = depth_map[vv, uu].astype(np.float32)
    sel = floor_mask[vv, uu] & (z >= min_depth_m) & (z <= max_depth_m)
    if int(sel.sum()) < min_points:
        return None
    uu = uu[sel]; vv = vv[sel]; z = z[sel]

    # Обратная проекция в систему КАМЕРЫ (без mount): x — вправо, y — вниз, z — вперёд.
    x = (uu - intrinsics.cx) * z / intrinsics.fx
    y = (vv - intrinsics.cy) * z / intrinsics.fy
    points = np.column_stack([x, y, z]).astype(np.float32)

    # Подгонка + один проход робастного отсева (NN уже выбрал пол, поэтому
    # хватает лёгкого trimming, не полноценный RANSAC).
    normal, centroid = _fit_plane(points)
    resid = np.abs((points - centroid) @ normal)
    inliers = resid < inlier_thresh_m
    n_total = points.shape[0]
    if int(inliers.sum()) >= max(min_points // 2, 50):
        normal, centroid = _fit_plane(points[inliers])
        resid_in = np.abs((points[inliers] - centroid) @ normal)
    else:
        resid_in = resid

    # Ориентируем нормаль К КАМЕРЕ (вверх): для горизонтальной камеры это (0,-1,0),
    # тогда n·centroid = -height < 0. Если знак другой — разворачиваем.
    if float(normal @ centroid) > 0:
        normal = -normal

    d = float(normal @ centroid)            # знаковое удаление плоскости от центра
    height_m = abs(d)
    tilt_rad = float(np.arctan2(-normal[2], -normal[1]))
    roll_abs = float(abs(normal[0]))        # боковая компонента нормали ~0 в норме
    rms = float(np.sqrt(np.mean(resid_in ** 2))) if resid_in.size else float('nan')
    inlier_ratio = float(inliers.sum()) / float(n_total)

    ok = (inlier_ratio >= 0.5
          and rms < inlier_thresh_m
          and roll_abs < 0.25
          and abs(np.degrees(tilt_rad)) <= max_tilt_for_ok_deg)

    return FloorCalibration(
        tilt_rad=tilt_rad,
        tilt_deg=float(np.degrees(tilt_rad)),
        height_m=height_m,
        normal=normal.astype(np.float32),
        n_points=n_total,
        inlier_ratio=inlier_ratio,
        rms_residual_m=rms,
        roll_abs=roll_abs,
        ok=ok,
    )


def _self_test():
    """
    Синтетическая проверка математики: генерируем depth-карту пола для камеры
    с ИЗВЕСТНЫМ наклоном/высотой (прямая модель), прогоняем оценку обратно и
    сверяем. Прямая модель: для пикселя (u,v) луч r=((u-cx)/fx,(v-cy)/fy,1),
    пол n·P=-H с n=(0,-cos t,-sin t) даёт z = H/(cos t·(v-cy)/fy + sin t).
    """
    from types import SimpleNamespace

    w, h = 640, 480
    intr = SimpleNamespace(fx=554.0, fy=554.0, cx=320.0, cy=240.0)
    rng = np.random.default_rng(0)

    print("tilt°  H(м) | оценка tilt°  H(м)  rms(мм) inlier ok")
    all_ok = True
    for true_tilt_deg, true_h in [(0.0, 0.125), (15.0, 0.125), (25.0, 0.125),
                                  (35.0, 0.20), (10.0, 0.30)]:
        t = np.radians(true_tilt_deg)
        depth = np.zeros((h, w), dtype=np.float32)
        mask = np.zeros((h, w), dtype=bool)
        vs = np.arange(h); us = np.arange(w)
        uu, vv = np.meshgrid(us, vs)
        denom = np.cos(t) * (vv - intr.cy) / intr.fy + np.sin(t)
        good = denom > 1e-3
        z = np.where(good, true_h / np.where(good, denom, 1.0), 0.0)
        in_range = good & (z > 0.2) & (z < 4.0)
        # шум глубины ~1 см — проверяем робастность отсева
        depth[in_range] = z[in_range] + rng.normal(0, 0.01, size=z.shape)[in_range]
        mask[in_range] = True

        cal = estimate_floor_plane(depth, mask, intr, pixel_stride=6)
        if cal is None:
            print(f"{true_tilt_deg:5.1f} {true_h:.3f} | НЕТ ТОЧЕК"); all_ok = False; continue
        d_tilt = abs(cal.tilt_deg - true_tilt_deg)
        d_h = abs(cal.height_m - true_h)
        passed = d_tilt < 1.0 and d_h < 0.01 and cal.ok
        all_ok &= passed
        print(f"{true_tilt_deg:5.1f} {true_h:.3f} | {cal.tilt_deg:9.2f} "
              f"{cal.height_m:.3f} {cal.rms_residual_m*1000:6.1f} "
              f"{cal.inlier_ratio:5.2f} {cal.ok}  "
              f"{'OK' if passed else 'FAIL (Δtilt=%.2f° Δh=%.3f)' % (d_tilt, d_h)}")
    print("\nИТОГ:", "все сходится" if all_ok else "ЕСТЬ РАСХОЖДЕНИЯ")


if __name__ == '__main__':
    _self_test()