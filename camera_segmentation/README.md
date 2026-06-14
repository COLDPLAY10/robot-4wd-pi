# camera_segmentation — нейросетевая сегментация пола

Семантическая сегментация пола нейросетью **SegFormer-B0** (ADE20K) +
автокалибровка наклона/высоты камеры по сегментированному полу.

Модуль переписан: старый цветовой baseline (`CameraSegmentation`,
`simple_color_segmentation`) никогда не использовался в навигации и удалён.

## Зачем

- **floor-маска** — пиксели пола нейросетью; видит то, чего не даёт геометрия
  глубины (сетки, блики, тени), и отделяет «не-пол на полу» = препятствие.
- **автокалибровка** — по floor-пикселям подгоняется плоскость пола в системе
  камеры, из неё восстанавливаются фактические наклон и высота камеры. Закрывает
  открытый вопрос: в `NavigationController` стоит `CAMERA_MOUNT_TILT_RAD = 0.0`,
  хотя серво камеры под 25° — калибровка показывает реальный угол.

## Структура

```
config.py            SEGFORMER_MODEL_PATH, SEGFORMER_INPUT_SIZE, FLOOR_CLASS_IDS
floor_segmenter.py   FloorSegmenter — ONNX-инференс SegFormer → bool-маска пола
floor_calibration.py estimate_floor_plane → наклон/высота камеры (камера-кадр, без mount)
floor_debug.py       render_floor_debug → overlay «кадр | маска пола + калибровка»
```

## Статус интеграции

**На алгоритм навигации не влияет** (в карту и реактивный слой не пишет). Два
флага в `NavigationController`:

- `FLOOR_SEG_CAPTURE_RAW` (**`True`**) — дёшево писать сырые кадры (RGB + глубина)
  для офлайн-оценки; модель на роботе не нужна, на навигацию не влияет.
- `FLOOR_SEG_ENABLED` (`False`) — гонять SegFormer + калибровку прямо на роботе
  (тяжело, вторая нейросеть уполовинит FPS depth). Пишет только overlay-лог,
  **в карту и реактивный слой не пишет** — это отдельный будущий этап.

Все логи запуска — в `results/<запуск>/`: захваты и overlay сегментации в
`segmentation/`, отладка depth-восприятия отдельно в `depth/` (не пересекаются).

## Использование

**1. Экспорт модели (один раз, на дев-машине с torch+transformers):**

```bash
python3 scripts/setup_segformer_model.py          # → models/segformer_b0_ade.onnx
```

Скрипт печатает `id2label` и индекс класса пола (проверено: `3 = 'floor'`,
ковёр `28 = 'rug'` — добавить в `FLOOR_CLASS_IDS`, если ездим по ковру).

**2. Сбор данных заездом (на роботе):** захват уже включён
(`FLOOR_SEG_CAPTURE_RAW = True`) — покататься и скопировать каталог запуска
`results/<запуск>/` на дев-машину.

**3. Офлайн-оценка (на дев-машине):**

```bash
python3 scripts/eval_floor_segmentation.py        # авто-выбор последнего запуска
```

Без аргументов берёт `segmentation/` последнего запуска в `results/`, кладёт
overlay-кадры + `results.csv` в `results/<запуск>/eval/` и печатает paste-ready
`CAMERA_MOUNT_TILT_RAD` / `CAMERA_MOUNT_HEIGHT_M` (медиана по надёжным кадрам).

**Программно:**

```python
from camera_segmentation import FloorSegmenter, estimate_floor_plane, render_floor_debug

seg = FloorSegmenter()
mask = seg.segment_floor(frame_bgr)                 # H×W bool, True = пол
cal = estimate_floor_plane(depth_map, mask, intrinsics)   # наклон/высота камеры
overlay = render_floor_debug(frame_bgr, mask, cal)  # картинка для разбора
```

## Проверка

Математика калибровки валидируется на синтетике:

```bash
python3 -m camera_segmentation.floor_calibration   # самотест: tilt/height recovery
```