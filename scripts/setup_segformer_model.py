#!/usr/bin/env python3
"""
Подготовка ONNX-модели SegFormer-B0 (семантическая сегментация пола) для робота.

Запускается ОДИН РАЗ на дев-машине с интернетом и torch/transformers.
Результат — файл models/segformer_b0_ade.onnx, который потом копируется на
Raspberry Pi (там torch не нужен — только onnxruntime), либо используется
офлайн-скриптом scripts/eval_floor_segmentation.py на дев-машине.

Архитектурный выбор — тот же, что у Depth-Anything-V2 (см. setup_depth_model.py):
ONNX Runtime вместо PyTorch (на Pi не нужен torch ~700 МБ, только onnxruntime).

Что делает:
  1. Скачивает с HuggingFace pretrained SegFormer-B0, дообученный на ADE20K.
  2. ВЫВОДИТ id2label модели и находит индекс(ы) класса "floor" — их надо
     прописать в camera_segmentation/config.py (FLOOR_CLASS_IDS). Хардкодить
     "floor = 3" нельзя: индекс зависит от конкретного чекпойнта.
  3. Экспортирует через torch.onnx.export в ONNX (один файл, без external data).
  4. Валидирует загрузку в onnxruntime и прогоняет dummy-инференс.

ВАЖНО про выход модели: SegFormer отдаёт логиты в разрешении H/4 × W/4
(для входа 512 — это 128×128, 150 классов ADE20K). Апсемплинг до размера
кадра и argmax делает уже camera_segmentation/floor_segmenter.py — в ONNX
кладём сырые логиты, граф остаётся минимальным.

Использование:
  pip install --user --break-system-packages torch transformers
  python3 scripts/setup_segformer_model.py
  # Скопировать модель на Pi (если планируется боевой инференс на роботе):
  scp models/segformer_b0_ade.onnx pi@raspberry:~/Диплом/robot-4wd-pi/models/
"""

import argparse
import os
import sys


VARIANTS = {
    # B0 — самый лёгкий SegFormer (3.7M параметров), главный кандидат для Pi.
    'b0': 'nvidia/segformer-b0-finetuned-ade-512-512',
    # B1 — точнее, тяжелее; запасной вариант, если B0 мажет пол.
    'b1': 'nvidia/segformer-b1-finetuned-ade-512-512',
}

DEFAULT_OUTPUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models', 'segformer_b0_ade.onnx'
)

# Подстроки, по которым ищем "пол" в id2label (ADE20K). Класс floor — основной;
# rug/carpet — ковёр на полу, по которому робот тоже ездит. Решение, включать
# ли ковёр в "пол", остаётся за тем, кто настраивает (см. вывод скрипта).
FLOOR_LABEL_HINTS = ('floor', 'flooring')
FLOORLIKE_LABEL_HINTS = ('rug', 'carpet', 'mat')


def main():
    parser = argparse.ArgumentParser(
        description='Скачать и экспортировать SegFormer (ADE20K) в ONNX',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--variant', choices=list(VARIANTS.keys()), default='b0',
                        help='вариант SegFormer (b0 — самый лёгкий, дефолт)')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help='куда сохранить .onnx (по умолчанию models/segformer_b0_ade.onnx)')
    parser.add_argument('--input-size', type=int, default=512,
                        help='размер входа модели (квадрат, как при обучении: 512)')
    parser.add_argument('--opset', type=int, default=17, help='ONNX opset version')
    args = parser.parse_args()

    try:
        import torch
        from transformers import AutoModelForSemanticSegmentation
    except ImportError as e:
        print(f"[setup] Не установлены torch/transformers: {e}")
        print("[setup] Установите: pip install torch transformers")
        sys.exit(1)

    model_id = VARIANTS[args.variant]
    print(f"[setup] Вариант: {args.variant}")
    print(f"[setup] HuggingFace ID: {model_id}")
    print(f"[setup] Скачиваю и загружаю модель...")

    model = AutoModelForSemanticSegmentation.from_pretrained(model_id)
    model.eval()

    # --- Ключевой шаг: вывести id2label и найти класс "пол" ---
    id2label = model.config.id2label
    print(f"\n[setup] Классов в модели: {len(id2label)}")
    floor_ids = [i for i, name in id2label.items()
                 if any(h in name.lower() for h in FLOOR_LABEL_HINTS)]
    floorlike_ids = [i for i, name in id2label.items()
                     if any(h in name.lower() for h in FLOORLIKE_LABEL_HINTS)]
    print(f"[setup] Класс(ы) 'пол' (floor): "
          + ", ".join(f"{i} = '{id2label[i]}'" for i in floor_ids))
    if floorlike_ids:
        print(f"[setup] Похожие на пол (rug/carpet/mat): "
              + ", ".join(f"{i} = '{id2label[i]}'" for i in floorlike_ids))
    print(f"[setup] -> впишите в camera_segmentation/config.py:")
    print(f"[setup]    FLOOR_CLASS_IDS = {floor_ids}"
          f"  # + {floorlike_ids} если считать ковёр полом")

    # Dummy-вход для трейса: SegFormer ожидает (B, 3, H, W), нормализация ImageNet.
    dummy_input = torch.randn(1, 3, args.input_size, args.input_size)

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)

    print(f"\n[setup] Экспорт в ONNX (opset={args.opset}, legacy TorchScript exporter)...")
    # dynamo=False — старый экспортер кладёт все веса в один .onnx (новый выносит
    # в .onnx.data, что усложняет копирование на Pi и не нужно для лёгкой модели).
    torch.onnx.export(
        model,
        dummy_input,
        args.output,
        input_names=['pixel_values'],
        output_names=['logits'],
        opset_version=args.opset,
        do_constant_folding=True,
        dynamo=False,
    )
    data_file = args.output + '.data'
    if os.path.exists(data_file):
        os.remove(data_file)
        print(f"[setup] Удалён внешний data-файл: {data_file}")

    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"[setup] Сохранено: {args.output} ({size_mb:.1f} МБ)")

    # Валидация: модель загружается в onnxruntime и прогоняет dummy.
    print("[setup] Валидация через onnxruntime...")
    try:
        import numpy as np
        import onnxruntime as ort
        sess = ort.InferenceSession(args.output, providers=['CPUExecutionProvider'])
        dummy_np = np.random.randn(1, 3, args.input_size, args.input_size).astype(np.float32)
        out = sess.run(None, {sess.get_inputs()[0].name: dummy_np})
        print(f"[setup] OK: инференс прошёл, выход shape={out[0].shape}, dtype={out[0].dtype}")
        print(f"[setup] (логиты H/4×W/4×{len(id2label)} классов — апсемплинг в floor_segmenter.py)")
    except ImportError:
        print("[setup] onnxruntime не установлен — пропускаю валидацию")
    except Exception as e:
        print(f"[setup] ВНИМАНИЕ: модель не работает в onnxruntime: {e}")
        sys.exit(2)

    print(f"\n[setup] Готово. Дальше: пропишите FLOOR_CLASS_IDS в config.py "
          f"и запустите офлайн-оценку scripts/eval_floor_segmentation.py")


if __name__ == '__main__':
    main()