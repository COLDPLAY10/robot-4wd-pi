#!/usr/bin/env python3
"""
Unit-тесты продюсерской свежести лидара — lidar.py (фикс A2).

Контракт: get_scan() отдаёт последний полный оборот ВЕЧНО (никогда не []),
поэтому отличить свежий лидар от зависшего по самому скану нельзя. Драйвер
ведёт метку времени публикации каждого оборота, а seconds_since_last_scan()
её отдаёт — потребитель (navigation_controller) по ней решает, кормить ли
реактивный слой или считать лидар отказавшим.

Проверяется на синтетических пакетах протокола YDLidar (AA 55), без железа:
  - до первого оборота seconds_since_last_scan() == None;
  - публикация оборота ставит метку (возраст ~0);
  - без новых оборотов возраст РАСТЁТ, а get_scan() остаётся замороженным
    (ровно то состояние, в котором старый код считал лидар «свежим»);
  - fallback-публикация (>2000 точек без стартовых пакетов) тоже ставит метку.

Запуск: python3 tests/test_lidar_freshness.py   (или pytest tests/)
"""

import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

# lidar.py делает `import serial` на уровне модуля, но обращается к serial.*
# только внутри connect(), который тесты не вызывают. На CI/дев-машине без
# pyserial подставляем пустую заглушку, чтобы импорт прошёл.
try:
    import serial  # noqa: F401
except ImportError:
    import types
    sys.modules['serial'] = types.ModuleType('serial')

from lidar import LidarDriver


def _packet(ct: int, lsn: int = 1, raw_dist: int = 4000) -> bytes:
    """
    Синтетический 3-байтный пакет YDLidar (intensity, dist_L, dist_H).
    ct&0x01 — стартовый пакет нового оборота. raw_dist/4/1000 = дистанция, м
    (на этом экземпляре делитель 4 → 4000 = 1.0 м, в валидном диапазоне).
    CS для 3-байтного формата драйвер не проверяет.
    """
    b = bytearray(b'\xAA\x55')
    b += bytes([ct & 0xFF, lsn & 0xFF])   # CT, LSN
    b += bytes([0, 0, 0, 0, 0, 0])        # FSA, LSA, CS
    for _ in range(lsn):
        b += bytes([0x00, raw_dist & 0xFF, (raw_dist >> 8) & 0xFF])
    return bytes(b)


def _new_driver() -> LidarDriver:
    # connect() не вызываем — порт не открывается, парсер тестируем напрямую.
    return LidarDriver(port='/dev/null')


def test_age_none_before_first_revolution():
    """До публикации первого оборота свежесть неизвестна -> None."""
    d = _new_driver()
    assert d.seconds_since_last_scan() is None


def test_publish_sets_timestamp():
    """
    Первый стартовый пакет нечего публиковать (буфер оборота пуст),
    второй — публикует накопленный оборот и ставит метку времени.
    """
    d = _new_driver()
    d._handle_packet(_packet(0x01), 0x01, 1, 3)
    assert d.seconds_since_last_scan() is None, "первый старт-пакет не публикует"
    assert d.revolution_count == 0

    d._handle_packet(_packet(0x01), 0x01, 1, 3)
    age = d.seconds_since_last_scan()
    assert age is not None and 0.0 <= age < 1.0, f"ожидался свежий штамп, получено {age}"
    assert d.revolution_count == 1
    assert len(d.get_scan()) == 1


def test_frozen_scan_age_grows():
    """
    Без новых оборотов возраст РАСТЁТ, но get_scan() продолжает отдавать
    последний оборот — это и есть состояние «зависшего» лидара, которое
    раньше читалось как свежее (баг A2).
    """
    d = _new_driver()
    d._handle_packet(_packet(0x01), 0x01, 1, 3)
    d._handle_packet(_packet(0x01), 0x01, 1, 3)
    age1 = d.seconds_since_last_scan()
    frozen = d.get_scan()

    time.sleep(0.2)
    age2 = d.seconds_since_last_scan()
    assert age2 >= age1 + 0.15, f"возраст должен расти: {age1} -> {age2}"
    assert d.get_scan() == frozen, "get_scan() остаётся замороженным (как и прежде)"


def test_fallback_publish_sets_timestamp():
    """
    Fallback-ветка (>2000 точек без стартовых пакетов) тоже обязана ставить
    метку — иначе свежесть в этом деградированном пути ложно протухала бы.
    """
    d = _new_driver()
    # Пакеты БЕЗ стартового бита: точки копятся в _current_rev до >2000.
    pkt = _packet(0x00, lsn=160)
    for _ in range(13):  # 13 * 160 = 2080 > 2000
        d._handle_packet(pkt, 0x00, 160, 3)
    age = d.seconds_since_last_scan()
    assert age is not None and age < 1.0, f"fallback не поставил метку: {age}"
    assert len(d.get_scan()) > 2000


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
