#!/usr/bin/env python3
"""
Подготовка ONNX-модели Depth-Anything-V2 для робота.

Запускается ОДИН РАЗ на дев-машине с интернетом и torch/transformers.
Результат — файл models/depth_anything_v2_small.onnx, который потом
копируется на Raspberry Pi (там torch не нужен — только onnxruntime).

Что делает:
  1. Скачивает с HuggingFace pretrained веса (вариант на твой выбор)
  2. Экспортирует через torch.onnx.export в ONNX-opset 17
  3. Сохраняет в models/depth_anything_v2_small.onnx

Варианты (--variant):
  metric-indoor (дефолт) — Depth-Anything-V2-Metric-Indoor-Small.
                           Выдаёт глубину В МЕТРАХ для помещений.
                           Это то, что нужно для интеграции с SLAM.
  relative              — Depth-Anything-V2-Small. Выдаёт инверсную
                           относительную глубину; нужно отдельно
                           калибровать масштаб.

Использование:
  # На дев-машине:
  pip install --user --break-system-packages torch transformers
  python3 scripts/setup_depth_model.py
  # Скопировать модель на Pi:
  scp models/depth_anything_v2_small.onnx pi@raspberry:~/Диплом/robot-4wd-pi/models/
"""

import argparse
import os
import sys


VARIANTS = {
    'metric-indoor': 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
    'relative': 'depth-anything/Depth-Anything-V2-Small-hf',
}

DEFAULT_OUTPUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'depth_anything_v2_small.onnx'
)


def main():
    parser = argparse.ArgumentParser(
        description='Скачать и экспортировать Depth-Anything-V2-Small в ONNX',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--variant', choices=list(VARIANTS.keys()),
                        default='metric-indoor',
                        help='какой вариант модели использовать')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help='куда сохранить .onnx (по умолчанию models/depth_anything_v2_small.onnx)')
    parser.add_argument('--input-size', type=int, default=518,
                        help='размер входа модели (кратный 14)')
    parser.add_argument('--opset', type=int, default=17,
                        help='ONNX opset version')
    args = parser.parse_args()

    try:
        import torch
        from transformers import AutoModelForDepthEstimation
    except ImportError as e:
        print(f"[setup] Не установлены torch/transformers: {e}")
        print("[setup] Установите: pip install torch transformers")
        sys.exit(1)

    model_id = VARIANTS[args.variant]
    print(f"[setup] Вариант: {args.variant}")
    print(f"[setup] HuggingFace ID: {model_id}")
    print(f"[setup] Скачиваю и загружаю модель...")

    model = AutoModelForDepthEstimation.from_pretrained(model_id)
    model.eval()

    # Dummy-вход для трейса. DA-V2 ожидает (B, 3, H, W).
    dummy_input = torch.randn(1, 3, args.input_size, args.input_size)

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)

    print(f"[setup] Экспорт в ONNX (opset={args.opset}, legacy TorchScript exporter)...")
    # dynamo=False — старый TorchScript-экспортер. Он stable и кладёт ВСЕ веса
    # в один .onnx файл (новый dynamo-экспортер выносит веса в .onnx.data,
    # что усложняет копирование на Pi и не нужно для 100МБ модели).
    torch.onnx.export(
        model,
        dummy_input,
        args.output,
        input_names=['pixel_values'],
        output_names=['predicted_depth'],
        opset_version=args.opset,
        do_constant_folding=True,
        dynamo=False,
    )

    # Уберём оставшийся external-data файл, если был от прошлого запуска
    data_file = args.output + '.data'
    if os.path.exists(data_file):
        os.remove(data_file)
        print(f"[setup] Удалён старый external-data файл: {data_file}")

    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"[setup] Сохранено: {args.output} ({size_mb:.1f} МБ)")

    # Валидация: модель должна загружаться onnxruntime и прогонять dummy.
    print("[setup] Валидация модели через onnxruntime...")
    try:
        import onnxruntime as ort
        import numpy as np
        sess = ort.InferenceSession(args.output, providers=['CPUExecutionProvider'])
        dummy_np = np.random.randn(1, 3, args.input_size, args.input_size).astype(np.float32)
        out = sess.run(None, {sess.get_inputs()[0].name: dummy_np})
        print(f"[setup] OK: инференс прошёл, выход shape={out[0].shape}, "
              f"dtype={out[0].dtype}")
    except ImportError:
        print("[setup] onnxruntime не установлен — пропускаю валидацию")
    except Exception as e:
        print(f"[setup] ВНИМАНИЕ: модель не работает в onnxruntime: {e}")
        sys.exit(2)

    print(f"[setup] Готово. Скопируйте {args.output} на Pi и запустите "
          f"python3 demo_with_lidar.py explore")


if __name__ == '__main__':
    main()
