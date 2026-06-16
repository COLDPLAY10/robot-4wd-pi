#!/usr/bin/env python3
"""
Unit-тесты контракта слияния пол(seg)+глубина — camera_perception/floor_fusion.py.

Без ONNX и без cv2: depth-карта и floor-маска строятся синтетически как
геометрически согласованные плоскости/слои, поэтому подгонка плоскости и высоты
над ней точные. Проверяются ВСЕ ветки контракта:
  - floor & низко            -> free, НЕ препятствие;
  - НЕ floor & приподнято    -> препятствие (ножка стула/стола);
  - НЕ floor & плоско         -> НЕ препятствие (промах seg на полированном полу,
                                 робот не должен вставать на ровном месте);
  - floor & сильно поднято   -> препятствие через strong-клаузу (объект в цвет пола);
  - высоко (потолок)          -> исключено верхним порогом max_height;
  - мало floor-пикселей       -> None (вызывающий откатывается на legacy).

Запуск: python3 tests/test_floor_fusion.py   (или pytest tests/)
"""

import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from camera_perception import CameraIntrinsics, CameraMount, fuse_camera_obstacles
from camera_perception.floor_fusion import fit_floor_plane_camera, classify_pixels

W, H = 640, 480
INTR = CameraIntrinsics.from_fov(W, H, hfov_deg=60.0)
MOUNT = CameraMount(height_m=0.125, forward_offset_m=0.12, tilt_rad=0.0)
H_FLOOR = 0.6  # «высота» пола по оси y камеры (y вниз): floor-пиксели на y=H_FLOOR

# Прямоугольные области кадра (v0, v1, u0, u1), желаемая высота над полом h, floor?
# Высота h задаёт y_cam = H_FLOOR - h; глубина считается из (u,v,y_cam).
REGIONS = {
    'floor':            dict(v=(340, 470), u=(0, 300),   h=0.0, floor=True),
    'leg':              dict(v=(340, 470), u=(300, 360), h=0.20, floor=False),
    'terrazzo_miss':    dict(v=(340, 470), u=(360, 440), h=0.0, floor=False),
    'floor_color_box':  dict(v=(340, 470), u=(440, 520), h=0.40, floor=True),
    'ceiling':          dict(v=(120, 180), u=(0, 200),   h=1.0, floor=False),
}


def build_scene():
    """Собрать (depth_map H×W, floor_mask H×W) из REGIONS. Глубина согласована
    с pinhole-моделью: y_cam = (v-cy)*z/fy => z = (H_FLOOR-h)*fy/(v-cy)."""
    depth = np.zeros((H, W), dtype=np.float32)
    floor = np.zeros((H, W), dtype=bool)
    vv = np.arange(H)[:, None].astype(np.float32)
    denom = (vv - INTR.cy)  # H×1
    for reg in REGIONS.values():
        v0, v1 = reg['v']; u0, u1 = reg['u']
        y_target = H_FLOOR - reg['h']
        # z = y_target * fy / (v - cy), по строкам; избегаем деления на 0.
        with np.errstate(divide='ignore', invalid='ignore'):
            d = np.where(np.abs(denom) > 1e-3, y_target * INTR.fy / denom, 0.0)
        d = np.clip(d, 0.0, 1e4).astype(np.float32)  # отрицательные/битые -> 0 (invalid)
        block = np.zeros((H, W), dtype=bool)
        block[v0:v1, u0:u1] = True
        depth[block] = np.broadcast_to(d, (H, W))[block]
        if reg['floor']:
            floor[block] = True
    return depth, floor


def _region_mask(info, reg):
    """Какие из классифицированных точек попали в область reg."""
    v0, v1 = reg['v']; u0, u1 = reg['u']
    return ((info['vv'] >= v0) & (info['vv'] < v1)
            & (info['uu'] >= u0) & (info['uu'] < u1))


def test_plane_fit_recovers_horizontal_floor():
    depth, floor = build_scene()
    plane = fit_floor_plane_camera(depth, floor, INTR, pixel_stride=8)
    assert plane is not None, "плоскость должна подогнаться (floor-пикселей много)"
    normal, centroid = plane
    # Пол горизонтальный y=H_FLOOR => нормаль ≈ (0,-1,0), centroid.y ≈ H_FLOOR.
    assert abs(abs(normal[1]) - 1.0) < 1e-3, f"нормаль не вертикальна: {normal}"
    assert abs(centroid[1] - H_FLOOR) < 0.02, f"centroid.y={centroid[1]:.3f}"


def test_contract_branches():
    depth, floor = build_scene()
    plane = fit_floor_plane_camera(depth, floor, INTR, pixel_stride=8)
    info = classify_pixels(depth, floor, INTR, plane, pixel_stride=8,
                           min_obstacle_height_m=0.03,
                           max_obstacle_height_m=0.60,
                           strong_obstacle_height_m=0.30)

    def frac_obstacle(name):
        m = _region_mask(info, REGIONS[name])
        assert m.sum() > 0, f"в области {name} нет классифицированных точек"
        return info['obstacle'][m].mean()

    def frac_free(name):
        m = _region_mask(info, REGIONS[name])
        return info['free'][m].mean()

    # floor & низко -> свободно, не препятствие
    assert frac_obstacle('floor') < 0.02, "пол ошибочно помечен препятствием"
    assert frac_free('floor') > 0.95, "пол не помечен свободным"

    # НЕ floor & приподнято (ножка) -> препятствие
    assert frac_obstacle('leg') > 0.95, "ножка не распознана препятствием"

    # НЕ floor & плоско (промах seg на полированном полу) -> НЕ препятствие
    assert frac_obstacle('terrazzo_miss') < 0.02, \
        "плоский промах сегментации ошибочно стал препятствием (робот встанет)"

    # floor & сильно поднято (объект в цвет пола) -> препятствие через strong
    assert frac_obstacle('floor_color_box') > 0.95, \
        "поднятый объект в цвет пола пропущен (seg сказал пол, но он торчит)"

    # высоко (потолок) -> исключено max_height
    assert frac_obstacle('ceiling') < 0.02, "потолок ошибочно стал препятствием"


def test_fuse_returns_world_points():
    depth, floor = build_scene()
    pts = fuse_camera_obstacles(depth, floor, INTR, MOUNT, robot_pose=(0.0, 0.0, 0.0),
                                pixel_stride=8)
    assert pts is not None and len(pts) > 0, "должны быть точки-препятствия"
    assert pts.shape[1] == 3 and pts.dtype == np.float32


def test_fuse_returns_none_without_floor():
    """Мало floor-пикселей -> None: вызывающий обязан откатиться на legacy."""
    depth, floor = build_scene()
    no_floor = np.zeros_like(floor)
    assert fuse_camera_obstacles(depth, no_floor, INTR, MOUNT, (0, 0, 0)) is None


def test_fuse_rejects_shape_mismatch():
    depth, floor = build_scene()
    assert fuse_camera_obstacles(depth, floor[:, :100], INTR, MOUNT, (0, 0, 0)) is None


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  ok  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
        except Exception as e:
            failed += 1
            print(f"ERR   {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} тестов прошло")
    return failed


if __name__ == '__main__':
    sys.exit(1 if _run_all() else 0)
