#!/usr/bin/env python3
"""
Unit-тесты КОНТРАКТА когерентной пары _SegWorker — фикс B5 (спаривание по кадру).

B5: раньше слияние склеивало САМУЮ СВЕЖУЮ глубину с ПОСЛЕДНЕЙ маской из ДРУГОГО,
более старого кадра → на движении пиксель (u,v) маски и глубины смотрел на разные
точки мира → фантомы/пропуски. Фикс: _SegWorker хранит маску ВМЕСТЕ с глубиной и
позой ЕЁ ЖЕ кадра (когерентная пара), потребитель сливает только эту пару.

Проверяется реальный класс _SegWorker (не реплика): map.navigation_controller
импортируется со стабом Raspbot_Lib (камеры/моторов на дев-машине нет). Если
транзитивная зависимость недоступна — тест помечается пропущенным (exit 0), чтобы
не валить CI на окружении без неё.

Контракт:
  - latest_pair() == None, пока пары нет;
  - на кадре С глубиной хранится ИМЕННО (mask, depth, pose, frame_ts) того кадра;
  - кадр БЕЗ глубины пропускается (сегментатор не зовётся, пара не пишется);
  - возраст пары считается по frame_ts (захват кадра), а не по времени маски.

Запуск: python3 tests/test_seg_pairing.py   (или pytest tests/)
"""

import os
import sys
import time
import types

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

# Стабы железо-зависимостей, чтобы импортировать navigation_controller вне Pi.
if 'Raspbot_Lib' not in sys.modules:
    _rl = types.ModuleType('Raspbot_Lib')

    class _Bot:
        def __getattr__(self, _n):
            return lambda *a, **k: None
    _rl.Raspbot = _Bot
    sys.modules['Raspbot_Lib'] = _rl
try:
    import serial  # noqa: F401
except ImportError:
    sys.modules['serial'] = types.ModuleType('serial')

try:
    from map.navigation_controller import _SegWorker, NavigationController
    _IMPORT_OK = True
except Exception as e:  # noqa: BLE001 — окружение без транзитивной зависимости
    print(f"SKIP: не удалось импортировать navigation_controller ({type(e).__name__}: {e})")
    _IMPORT_OK = False


class _FakeSeg:
    """Сегментатор-заглушка: считает вызовы, возвращает фиксированную маску."""
    def __init__(self):
        self.calls = 0

    def segment_floor(self, frame_bgr):
        self.calls += 1
        return np.ones((4, 4), dtype=bool)


def _frame():
    return np.zeros((4, 4, 3), dtype=np.uint8)


def _drive_once(worker, timeout_s=3.0):
    """Запустить поток, дождаться первой пары (или таймаута), остановить."""
    worker.start()
    deadline = time.time() + timeout_s
    while worker.latest_pair(1e9) is None and time.time() < deadline:
        time.sleep(0.01)
    worker.stop()


def test_latest_pair_none_initially():
    seg = _FakeSeg()
    w = _SegWorker(lambda: None, seg, period_s=0.005)
    assert w.latest_pair(1e9) is None


def test_stores_coherent_pair_from_same_frame():
    """Хранится ИМЕННО (mask, depth, pose, frame_ts) одного кадра."""
    seg = _FakeSeg()
    depth = np.full((4, 4), 1.5, dtype=np.float32)
    pose = (1.0, 2.0, 0.5)
    frame_ts = time.time()  # реалистичная метка: latest_pair не отбракует по возрасту
    # peek_frame отдаёт снап в порядке _DepthWorker._latest: (frame, pose, depth, ts)
    snap = (_frame(), pose, depth, frame_ts)
    w = _SegWorker(lambda: snap, seg, period_s=0.005)
    _drive_once(w)

    pair = w.latest_pair(1e9)
    assert pair is not None, "пара должна сохраниться для кадра с глубиной"
    mask, d, p, ts = pair
    assert mask.shape == (4, 4) and mask.dtype == bool
    assert d is depth, "глубина должна быть ИЗ ТОГО ЖЕ кадра (тот же объект)"
    assert p == pose
    assert ts == frame_ts, "frame_ts — захват кадра, не время записи маски"


def test_skips_frame_without_depth():
    """Кадр без глубины: сегментатор не зовётся, пара не пишется."""
    seg = _FakeSeg()
    snap = (_frame(), (0.0, 0.0, 0.0), None, 999.0)  # depth=None
    w = _SegWorker(lambda: snap, seg, period_s=0.005)
    w.start()
    time.sleep(0.2)  # много итераций периода
    w.stop()
    assert w.latest_pair(1e9) is None, "без глубины пара не должна писаться"
    assert seg.calls == 0, "сегментацию незачем гонять на кадре без глубины"


def test_age_measured_by_frame_ts():
    """Свежесть — по frame_ts кадра, а не по времени записи маски."""
    seg = _FakeSeg()
    w = _SegWorker(lambda: None, seg, period_s=0.005)
    mask = np.ones((4, 4), dtype=bool)
    depth = np.zeros((4, 4), dtype=np.float32)
    # frame_ts «старый» на 10 с — пара должна считаться протухшей
    w._mask = (mask, depth, (0.0, 0.0, 0.0), time.time() - 10.0)
    assert w.latest_pair(3.0) is None
    # свежий frame_ts — пара отдаётся
    w._mask = (mask, depth, (0.0, 0.0, 0.0), time.time())
    assert w.latest_pair(3.0) is not None


# ───── Решение карто-канала _camera_fusion_decision (дедуп + kill-switch) ─────
# Тестируем РЕАЛЬНЫЙ метод через __new__ (без тяжёлого __init__): он использует
# только self._fused_source_ts.

def _nav():
    obj = NavigationController.__new__(NavigationController)
    obj._fused_source_ts = None
    return obj


def _decide(nav, pair, fresh_depth, fresh_pose, fresh_ts):
    return NavigationController._camera_fusion_decision(
        nav, pair, fresh_depth, fresh_pose, fresh_ts)


def test_decision_fusion_off_writes_every_tick_with_no_mask():
    """Kill-switch инвариант: нет пары (слияние выкл) → legacy на свежей глубине,
    floor_mask=None, запись КАЖДЫЙ depth-такт (как в прежнем коде)."""
    nav = _nav()
    dA, dB = np.zeros((2, 2)), np.ones((2, 2))
    w1, m1, d1, p1 = _decide(nav, None, dA, (0.0, 0.0, 0.0), 1.0)
    assert w1 is True and m1 is None and d1 is dA and p1 == (0.0, 0.0, 0.0)
    w2, m2, d2, _ = _decide(nav, None, dB, (1.0, 0.0, 0.0), 2.0)
    assert w2 is True and m2 is None and d2 is dB  # ts вырос → снова пишем


def test_decision_dedups_lingering_pair():
    """Та же пара (один frame_ts) пишется в карту ОДИН раз; берётся глубина пары."""
    nav = _nav()
    mask, depth_seg = np.ones((2, 2), dtype=bool), np.full((2, 2), 0.5)
    pair = (mask, depth_seg, (5.0, 5.0, 1.0), 100.0)
    w1, m1, d1, p1 = _decide(nav, pair, np.zeros((2, 2)), (9.0, 9.0, 9.0), 3.0)
    assert w1 is True and m1 is mask and d1 is depth_seg and p1 == (5.0, 5.0, 1.0)
    w2, _, _, _ = _decide(nav, pair, np.zeros((2, 2)), (9.0, 9.0, 9.0), 4.0)
    assert w2 is False  # тот же frame_ts источника → дедуп, повторно не пишем


def test_decision_transitions_trigger_write():
    """Переходы пара→legacy и legacy→пара всегда дают новую запись."""
    nav = _nav()
    mask, dS = np.ones((2, 2), dtype=bool), np.zeros((2, 2))
    pair = (mask, dS, (0.0, 0.0, 0.0), 100.0)
    assert _decide(nav, pair, dS, (0.0, 0.0, 0.0), 3.0)[0] is True   # пара
    w, m, _, _ = _decide(nav, None, dS, (0.0, 0.0, 0.0), 4.0)        # пара→legacy
    assert w is True and m is None
    w, m, _, _ = _decide(nav, pair, dS, (0.0, 0.0, 0.0), 5.0)        # legacy→пара
    assert w is True and m is mask


def _run_all():
    if not _IMPORT_OK:
        return 0  # пропуск, не провал
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
