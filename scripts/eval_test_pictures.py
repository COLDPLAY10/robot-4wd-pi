#!/usr/bin/env python3
"""
ОФЛАЙН-разбор кадров, снятых телефоном как симуляция камеры робота.

Зачем: на последнем заезде камера не работала, поэтому сцены пола и области
движения сняты телефоном вручную (test_pictures/). Скрипт приводит эти фото к
формату кадра робота и прогоняет на них ОБЕ нейросети восприятия, чтобы понять,
как они реально отрабатывают на таких сценах:
  - Depth-Anything-V2 (монокулярная глубина, camera_perception/);
  - SegFormer-B0 ADE20K (сегментация пола, camera_segmentation/).

Конвейер приведения кадра (фото телефона -> кадр робота):
  1. Обрезать нижнюю полосу с геометкой/временем (водяной знак телефона).
  2. Центральный кроп до пропорции камеры робота 4:3 (без искажения геометрии).
  3. Ресайз до CAMERA_WIDTH x CAMERA_HEIGHT (640x480) — ровно то, что отдаёт
     VideoCapture робота в боевом контуре.

По каждому кадру собирается сравнительная панель 2x2:
  [кадр робота] [глубина]
  [пол]         [совмещённо: глубина + контур/заливка пола]
Плюс агрегатная сводка (floor%, диапазон глубины, тайминги инференса) в CSV.

Зависимости: onnxruntime + opencv + numpy (torch НЕ нужен — обе модели в ONNX).
Модели: models/depth_anything_v2_small.onnx, models/segformer_b0_ade.onnx.

Использование:
  python3 scripts/eval_test_pictures.py
  python3 scripts/eval_test_pictures.py --src test_pictures --out results/test_pictures_eval
  python3 scripts/eval_test_pictures.py --watermark-frac 0.05 --limit 3
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

# Запуск из scripts/: добавить корень репо в путь импорта.
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

import cv2

from camera_perception import DepthEstimator, CameraIntrinsics
from camera_perception.floor_fusion import fit_floor_plane_camera, classify_pixels
from camera_segmentation import FloorSegmenter, FLOOR_CLASS_IDS

# Формат кадра робота — single source of truth держит NavigationController,
# здесь дублируем значения как дефолты CLI (см. CAMERA_WIDTH/HEIGHT там).
ROBOT_W, ROBOT_H = 640, 480


def to_robot_frame(img, watermark_frac, robot_w=ROBOT_W, robot_h=ROBOT_H):
    """
    Фото телефона -> кадр робота: срез водяного знака снизу, центральный кроп до
    пропорции робота, ресайз. Возвращает BGR robot_h x robot_w.
    """
    h, w = img.shape[:2]
    # 1. Срезать нижнюю полосу с геометкой/временем.
    cut = int(round(h * watermark_frac))
    if cut > 0:
        img = img[: h - cut]
    h, w = img.shape[:2]

    # 2. Центральный кроп до пропорции робота (4:3 при 640x480).
    target_ar = robot_w / robot_h
    cur_ar = w / h
    if cur_ar > target_ar:
        # слишком широкое — режем по бокам
        new_w = int(round(h * target_ar))
        x0 = (w - new_w) // 2
        img = img[:, x0:x0 + new_w]
    else:
        # слишком высокое (наш случай — портрет) — режем сверху/снизу
        new_h = int(round(w / target_ar))
        y0 = (h - new_h) // 2
        img = img[y0:y0 + new_h, :]

    # 3. Ресайз до разрешения робота.
    return cv2.resize(img, (robot_w, robot_h), interpolation=cv2.INTER_AREA)


def colorize_depth(depth):
    """Глубина -> цветная карта (JET). Ближнее = тёплое (как в debug_viz)."""
    d = depth.astype(np.float32)
    lo, hi = float(np.nanmin(d)), float(np.nanmax(d))
    if hi - lo < 1e-6:
        norm = np.zeros_like(d)
    else:
        norm = (d - lo) / (hi - lo)
    depth_u8 = ((1.0 - norm) * 255).astype(np.uint8)  # ближнее -> 255 -> красное
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET), lo, hi


def overlay_floor(frame, floor_mask, alpha=0.45):
    """Кадр + зелёная полупрозрачная заливка пикселей пола."""
    vis = frame.copy()
    if floor_mask.any():
        green = np.zeros_like(frame)
        green[:, :] = (0, 255, 0)
        blended = cv2.addWeighted(frame, 1.0 - alpha, green, alpha, 0)
        vis[floor_mask] = blended[floor_mask]
    return vis


def combine_depth_floor(depth_color, floor_mask, alpha=0.35):
    """
    Совмещённый вид: цветная глубина + зелёная заливка пола + белый контур
    границы пола. Идея навигации: floor = свободно (зелёное), цвет = дистанция
    до всего, контур = граница проезжей зоны.
    """
    vis = depth_color.copy()
    if floor_mask.any():
        green = np.zeros_like(depth_color)
        green[:, :] = (0, 255, 0)
        blended = cv2.addWeighted(depth_color, 1.0 - alpha, green, alpha, 0)
        vis[floor_mask] = blended[floor_mask]
        # Контур границы пола.
        edges = cv2.morphologyEx(floor_mask.astype(np.uint8), cv2.MORPH_GRADIENT,
                                 np.ones((3, 3), np.uint8))
        vis[edges > 0] = (255, 255, 255)
    return vis


def render_fusion(frame, depth, floor_mask, intrinsics, stride=6):
    """
    Итоговый вид слияния по контракту floor_fusion: пол(свободно) — зелёный,
    препятствие — КРАСНЫМ С ГРАДИЕНТОМ ПО РАССТОЯНИЮ (ближнее — чистый красный,
    дальнее — к жёлтому). Высота над плоскостью, подогнанной по floor-пикселям
    (классификация в системе камеры, mount/поза не нужны). Если плоскость не
    подогналась — кадр с пометкой «нет плоскости → legacy».

    Дистанция берётся из z (глубина вдоль оптической оси) тех же пикселей —
    напоминание, что в облаке препятствий расстояние есть, fusion-классификация
    лишь не кодирует его сама.
    """
    vis = frame.copy()
    plane = fit_floor_plane_camera(depth, floor_mask, intrinsics, pixel_stride=stride)
    if plane is None:
        return vis, None
    info = classify_pixels(depth, floor_mask, intrinsics, plane, pixel_stride=stride)
    half = max(1, stride // 2)
    free = info['free']; obst = info['obstacle']
    for u, v in np.column_stack([info['uu'][free], info['vv'][free]]):
        cv2.rectangle(vis, (u - half, v - half), (u + half, v + half), (0, 200, 0), -1)

    # Препятствия: цвет по дистанции. t=0 (ближнее) -> (0,0,255) красный;
    # t=1 (дальнее) -> (0,255,255) жёлтый (BGR). Нормировка по диапазону самих
    # препятствий — максимальный контраст на кадре.
    z_obs = info['z_cam'][obst]
    if z_obs.size:
        z0, z1 = float(z_obs.min()), float(z_obs.max())
        rng = max(1e-3, z1 - z0)
        uu_o = info['uu'][obst]; vv_o = info['vv'][obst]
        for u, v, z in zip(uu_o, vv_o, z_obs):
            t = (z - z0) / rng
            color = (0, int(255 * t), 255)  # BGR: G растёт с дальностью
            cv2.rectangle(vis, (u - half, v - half), (u + half, v + half), color, -1)
    obst_pct = 100.0 * float(obst.sum()) / float(max(1, obst.size))
    return vis, obst_pct


def label(img, text, scale=0.6):
    """Подпись с тёмной подложкой в левом верхнем углу панели."""
    out = img.copy()
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(out, (0, 0), (tw + 12, th + 12), (0, 0, 0), -1)
    cv2.putText(out, text, (6, th + 6), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    p = argparse.ArgumentParser(
        description='Разбор телефонных фото как кадров робота: глубина + сегментация пола',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--src', default=os.path.join(_REPO, 'test_pictures'),
                   help='каталог с фото (*.jpg/*.jpeg/*.png)')
    p.add_argument('--out', default=os.path.join(_REPO, 'results', 'test_pictures_eval'),
                   help='каталог для панелей сравнения и results.csv')
    p.add_argument('--watermark-frac', type=float, default=0.05,
                   help='доля высоты снизу под обрезку водяного знака (0..1)')
    p.add_argument('--hfov', type=float, default=60.0,
                   help='горизонтальный FOV камеры робота, ° (как CAMERA_HFOV_DEG)')
    p.add_argument('--floor-class-ids', default=None,
                   help='переопределить классы пола ADE20K через запятую (напр. "3,28")')
    p.add_argument('--seg-model', default=None, help='путь к ONNX SegFormer')
    p.add_argument('--depth-model', default=None, help='путь к ONNX Depth-Anything')
    p.add_argument('--limit', type=int, default=0, help='обработать только N фото (0 = все)')
    p.add_argument('--save-parts', action='store_true',
                   help='помимо панели сохранять каждый вид отдельным файлом')
    args = p.parse_args()

    exts = ('*.jpg', '*.jpeg', '*.JPG', '*.png', '*.PNG')
    files = sorted({f for e in exts for f in glob.glob(os.path.join(args.src, e))})
    if args.limit > 0:
        files = files[:args.limit]
    if not files:
        print(f"[test-pics] Нет изображений в {args.src}")
        sys.exit(1)
    print(f"[test-pics] Фото к обработке: {len(files)}")

    floor_ids = (tuple(int(x) for x in args.floor_class_ids.split(','))
                 if args.floor_class_ids else FLOOR_CLASS_IDS)
    segmenter = FloorSegmenter(model_path=args.seg_model, floor_class_ids=floor_ids)
    if not segmenter.is_ready():
        print("[test-pics] SegFormer не загрузился — см. scripts/setup_segformer_model.py")
        sys.exit(2)
    depth_est = DepthEstimator(model_path=args.depth_model)
    if not depth_est.is_ready():
        print("[test-pics] Depth-Anything не загрузился — см. scripts/setup_depth_model.py")
        sys.exit(2)

    os.makedirs(args.out, exist_ok=True)
    rows = []

    for i, fpath in enumerate(files):
        raw = cv2.imread(fpath)
        if raw is None:
            print(f"[test-pics] не читается: {fpath}")
            continue
        name = os.path.splitext(os.path.basename(fpath))[0]

        frame = to_robot_frame(raw, args.watermark_frac)
        depth = depth_est.get_depth_map(frame)
        floor_mask = segmenter.segment_floor(frame)
        if depth is None or floor_mask is None:
            print(f"[test-pics] инференс не удался: {name}")
            continue

        depth_color, d_lo, d_hi = colorize_depth(depth)
        floor_vis = overlay_floor(frame, floor_mask)
        intr = CameraIntrinsics.from_fov(ROBOT_W, ROBOT_H, hfov_deg=args.hfov)
        fusion_vis, obst_pct = render_fusion(frame, depth, floor_mask, intr)
        floor_pct = 100.0 * float(floor_mask.sum()) / float(floor_mask.size)

        # Панель 2x2 с подписями: кадр | глубина / пол | слияние (свободно/препятствие).
        tl = label(frame, f"robot frame {ROBOT_W}x{ROBOT_H}")
        tr = label(depth_color, f"depth {d_lo:.2f}..{d_hi:.2f} (red=near)")
        bl = label(floor_vis, f"floor seg  {floor_pct:.0f}%")
        fusion_lbl = ("no plane -> legacy" if obst_pct is None
                      else f"fusion: floor=green  obst {obst_pct:.0f}% (near=red far=yellow)")
        br = label(fusion_vis, fusion_lbl)
        panel = np.vstack([np.hstack([tl, tr]), np.hstack([bl, br])])
        cv2.imwrite(os.path.join(args.out, f"panel_{name}.jpg"), panel,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])

        if args.save_parts:
            cv2.imwrite(os.path.join(args.out, f"{name}_frame.jpg"), frame)
            cv2.imwrite(os.path.join(args.out, f"{name}_depth.jpg"), depth_color)
            cv2.imwrite(os.path.join(args.out, f"{name}_floor.jpg"), floor_vis)
            cv2.imwrite(os.path.join(args.out, f"{name}_fusion.jpg"), fusion_vis)

        rows.append([name, f"{floor_pct:.1f}", f"{d_lo:.3f}", f"{d_hi:.3f}",
                     ('' if obst_pct is None else f"{obst_pct:.1f}"),
                     f"{depth_est.last_inference_ms:.0f}",
                     f"{segmenter.last_inference_ms:.0f}"])
        print(f"[test-pics] {i+1}/{len(files)} {name}: floor {floor_pct:.0f}%, "
              f"obst {('--' if obst_pct is None else f'{obst_pct:.0f}%')}, "
              f"depth {d_lo:.2f}..{d_hi:.2f}, "
              f"{depth_est.last_inference_ms:.0f}мс/{segmenter.last_inference_ms:.0f}мс")

    csv_path = os.path.join(args.out, 'results.csv')
    with open(csv_path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['frame', 'floor_pct', 'depth_min', 'depth_max',
                     'obstacle_pct', 'depth_ms', 'seg_ms'])
        wr.writerows(rows)

    print(f"\n[test-pics] Панели сравнения и {os.path.basename(csv_path)} -> {args.out}/")
    if rows:
        fp = np.array([float(r[1]) for r in rows])
        print(f"[test-pics] floor%: медиана {np.median(fp):.0f}  "
              f"[{fp.min():.0f}..{fp.max():.0f}]")


if __name__ == '__main__':
    main()
