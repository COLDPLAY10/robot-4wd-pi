#!/usr/bin/env python3
"""
ОФЛАЙН-оценка сегментации пола по кадрам, снятым во время тестового заезда.

Сценарий: на роботе включён захват сырых кадров (NavigationController.
FLOOR_SEG_CAPTURE_RAW = True) — он пишет их в results/<запуск>/segmentation/.
Катаются, копируют каталог запуска results/<запуск>/ на дев-машину и запускают
этот скрипт. Тяжёлый SegFormer и калибровка считаются ЗДЕСЬ (на роботе их можно
не гонять вовсе) — так сбор данных и тяжёлый инференс развязаны.

Что делает по каждому кадру:
  1. Сегментирует пол (FloorSegmenter, SegFormer-B0).
  2. Берёт глубину: сохранённый рядом .npy (если захватывали) либо прогоняет
     Depth-Anything-V2 на том же RGB.
  3. Оценивает наклон/высоту камеры (estimate_floor_plane) — автокалибровка.
  4. Кладёт overlay «кадр | маска пола + калибровка» в out-каталог.

В конце:
  - CSV со всеми данными по кадрам (floor%, tilt, height, inlier, rms, ok);
  - агрегат по НАДЁЖНЫМ кадрам и paste-ready значения CAMERA_MOUNT_TILT_RAD /
    CAMERA_MOUNT_HEIGHT_M для NavigationController (медиана — наклон фиксирован
    серво, один кадр шумит, нужна статистика).

Зависимости: onnxruntime + opencv + numpy (torch НЕ нужен — обе модели в ONNX).
Модели: models/segformer_b0_ade.onnx (setup_segformer_model.py) и
models/depth_anything_v2_small.onnx (setup_depth_model.py).

Использование:
  python3 scripts/eval_floor_segmentation.py          # авто-выбор последнего запуска
  python3 scripts/eval_floor_segmentation.py --captures-dir results/<запуск>/segmentation
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

from camera_perception import CameraIntrinsics, DepthEstimator
from camera_segmentation import (FloorSegmenter, estimate_floor_plane,
                                  render_floor_debug, FLOOR_CLASS_IDS)


def main():
    p = argparse.ArgumentParser(
        description='Офлайн-оценка сегментации пола + автокалибровка по кадрам заезда',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--captures-dir', default=None,
                   help='каталог с кадрами cap_*.jpg (+ опц. cap_*.npy глубины). '
                        'По умолчанию — segmentation/ последнего запуска в results/')
    p.add_argument('--out-dir', default=None,
                   help='куда писать overlay-кадры и results.csv. По умолчанию — '
                        'eval/ рядом с захватами (results/<запуск>/eval/)')
    p.add_argument('--hfov', type=float, default=60.0,
                   help='горизонтальный FOV камеры, ° (как CAMERA_HFOV_DEG)')
    p.add_argument('--seg-model', default=None, help='путь к ONNX SegFormer')
    p.add_argument('--depth-model', default=None, help='путь к ONNX Depth-Anything')
    p.add_argument('--floor-class-ids', default=None,
                   help='переопределить классы пола, через запятую (напр. "3,28")')
    p.add_argument('--pixel-stride', type=int, default=8)
    p.add_argument('--limit', type=int, default=0, help='обработать только N кадров (0 = все)')
    args = p.parse_args()

    # captures-dir по умолчанию — segmentation/ последнего запуска в results/.
    if args.captures_dir is None:
        cand = sorted(glob.glob(os.path.join(_REPO, 'results', '*', 'segmentation')))
        if not cand:
            print("[eval] Не найдено results/*/segmentation. Укажите --captures-dir "
                  "или сделайте заезд с FLOOR_SEG_CAPTURE_RAW=True.")
            sys.exit(1)
        args.captures_dir = cand[-1]  # формат запуска YYYYmmdd-HHMMSS сортируется хронологически
        print(f"[eval] captures-dir по умолчанию (последний запуск): {args.captures_dir}")
    # out-dir по умолчанию — eval/ рядом с захватами (results/<запуск>/eval/).
    if args.out_dir is None:
        args.out_dir = os.path.join(os.path.dirname(os.path.abspath(args.captures_dir)), 'eval')

    frames = sorted(glob.glob(os.path.join(args.captures_dir, 'cap_*.jpg')))
    if args.limit > 0:
        frames = frames[:args.limit]
    if not frames:
        print(f"[eval] Нет кадров cap_*.jpg в {args.captures_dir}")
        sys.exit(1)
    print(f"[eval] Кадров к обработке: {len(frames)}")

    floor_ids = (tuple(int(x) for x in args.floor_class_ids.split(','))
                 if args.floor_class_ids else FLOOR_CLASS_IDS)
    segmenter = FloorSegmenter(model_path=args.seg_model, floor_class_ids=floor_ids)
    if not segmenter.is_ready():
        print("[eval] SegFormer не загрузился — см. scripts/setup_segformer_model.py")
        sys.exit(2)
    depth_estimator = DepthEstimator(model_path=args.depth_model)  # лениво

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, 'results.csv')
    rows = []
    tilts, heights = [], []

    for i, fpath in enumerate(frames):
        frame = cv2.imread(fpath)
        if frame is None:
            print(f"[eval] не читается: {fpath}")
            continue
        h, w = frame.shape[:2]
        intr = CameraIntrinsics.from_fov(w, h, hfov_deg=args.hfov)

        floor_mask = segmenter.segment_floor(frame)
        if floor_mask is None:
            print(f"[eval] сегментация не удалась: {fpath}")
            continue

        # Глубина: сохранённый .npy рядом или прогон Depth-Anything на RGB.
        npy = os.path.splitext(fpath)[0] + '.npy'
        if os.path.exists(npy):
            depth = np.load(npy).astype(np.float32)
            if depth.shape[:2] != (h, w):
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
        else:
            depth = depth_estimator.get_depth_map(frame)
        if depth is None:
            print(f"[eval] нет глубины для {fpath} (запустите setup_depth_model.py)")
            continue

        cal = estimate_floor_plane(depth, floor_mask, intr, pixel_stride=args.pixel_stride)

        name = os.path.basename(fpath)
        floor_pct = 100.0 * float(floor_mask.sum()) / float(floor_mask.size)
        if cal is not None:
            rows.append([name, f"{floor_pct:.1f}", f"{cal.tilt_deg:.2f}",
                         f"{cal.height_m:.4f}", f"{cal.inlier_ratio:.3f}",
                         f"{cal.rms_residual_m*1000:.1f}", f"{cal.roll_abs:.3f}",
                         int(cal.ok)])
            if cal.ok:
                tilts.append(cal.tilt_deg); heights.append(cal.height_m)
        else:
            rows.append([name, f"{floor_pct:.1f}", '', '', '', '', '', 0])

        overlay = render_floor_debug(frame, floor_mask, cal)
        if overlay is not None:
            cv2.imwrite(os.path.join(args.out_dir, 'overlay_' + name), overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
        if (i + 1) % 10 == 0:
            print(f"[eval] {i+1}/{len(frames)}")

    with open(csv_path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['frame', 'floor_pct', 'tilt_deg', 'height_m',
                     'inlier_ratio', 'rms_mm', 'roll_abs', 'ok'])
        wr.writerows(rows)

    print(f"\n[eval] Overlay-кадры и {csv_path} записаны в {args.out_dir}/")
    print(f"[eval] Сегментация: lat {segmenter.last_inference_ms:.0f} мс/кадр "
          f"({segmenter.total_inferences} инференсов)")

    if not tilts:
        print("[eval] Нет НАДЁЖНЫХ кадров (ok) — калибровка не выводится. "
              "Смотрите overlay: возможно, пол не виден или маска рваная.")
        return

    tilt_arr = np.array(tilts); h_arr = np.array(heights)
    tilt_med = float(np.median(tilt_arr)); h_med = float(np.median(h_arr))
    print(f"\n[eval] Надёжных кадров: {len(tilts)}/{len(frames)}")
    print(f"[eval] tilt°:  медиана {tilt_med:+.2f}  "
          f"[{tilt_arr.min():+.2f}..{tilt_arr.max():+.2f}]  σ={tilt_arr.std():.2f}")
    print(f"[eval] высота: медиана {h_med*100:.1f}см  "
          f"[{h_arr.min()*100:.1f}..{h_arr.max()*100:.1f}]  σ={h_arr.std()*100:.1f}см")
    print("\n[eval] Вставьте в NavigationController (физические параметры камеры):")
    print(f"    CAMERA_MOUNT_TILT_RAD = {np.radians(tilt_med):.4f}   # = {tilt_med:.1f}° (автокалибровка)")
    print(f"    CAMERA_MOUNT_HEIGHT_M = {h_med:.4f}   # автокалибровка по полу")


if __name__ == '__main__':
    main()