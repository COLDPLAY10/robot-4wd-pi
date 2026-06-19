#!/usr/bin/env python3
"""
Контрактные тесты ЛОГИКИ защитных веток навигации — фиксы A4 и A5.

ВАЖНО про природу этих тестов. Авторитетный код — в map_scripts/demo_with_lidar.py
(_run_loop) и map/navigation_controller.py (_safe_exploration_behavior). Оба модуля
на уровне импорта тянут car_adapter -> Raspbot_Lib (есть только на Pi) и тяжёлую
инициализацию, поэтому в CI/на дев-машине не импортируются. Здесь воспроизведена
1:1 ТА ЖЕ управляющая логика (с пометкой источника), и тест пиннит её поведение:
если меняешь соответствующую ветку в боевом коде — синхронизируй и реплику здесь.
Это исполняемая спецификация контракта, а не интеграционный тест боевого модуля.

Покрытие:
  A4 (demo_with_lidar._run_loop): единичный сбой в такте не роняет заезд, но
     ОБЯЗАТЕЛЬНО стопит моторы; N ошибок подряд -> контролируемый выход;
     KeyboardInterrupt не глотается except Exception (Ctrl+C работает).
  A5 (navigation_controller, критическая ветка EXPLORATION): при зацикливании
     (consecutive_rotations >= max) робот разворачивается в ПРОТИВОПОЛОЖНУЮ
     сторону и сбрасывает счётчик; типовой путь (counter < max) не изменился.

Запуск: python3 tests/test_navigation_guards.py   (или pytest tests/)
"""

import sys


# ───────────────────────── A4: защитный главный цикл ─────────────────────────
# Реплика структуры map_scripts/demo_with_lidar.py:_run_loop (тело такта в
# try/except Exception со стопом моторов и счётчиком ошибок подряд).

class _SpyCA:
    def __init__(self):
        self.stop_calls = 0

    def stop_robot(self):
        self.stop_calls += 1


class _FakeController:
    """update() проигрывает сценарий действий; get_status управляем."""
    def __init__(self, script):
        self.script = script
        self.i = 0
        self._mode = 'exploration'
        self._goal = (1.0, 1.0)

    def update(self):
        act = self.script[self.i] if self.i < len(self.script) else 'ok'
        self.i += 1
        if act == 'raise':
            raise ValueError("синтетический сбой в такте")
        if act == 'kbd':
            raise KeyboardInterrupt()
        if act == 'idle':
            self._mode = 'idle'

    def get_status(self):
        return {'mode': self._mode, 'goal': self._goal}


def _run_loop_core(controller, ca, stop_when_idle, max_iters=500):
    """Управляющая логика _run_loop без печати/sleep. Возвращает исход."""
    consecutive_errors = 0
    MAX_CONSECUTIVE_ERRORS = 10
    result = None
    iters = 0
    try:
        while iters < max_iters:
            iters += 1
            try:
                controller.update()
                if stop_when_idle and controller.get_status()['mode'] == 'idle':
                    result = ('success' if controller.get_status().get('goal') is not None
                              else 'unreachable')
                    break
                consecutive_errors = 0
            except Exception:
                try:
                    ca.stop_robot()
                except Exception:
                    pass
                consecutive_errors += 1
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    result = 'aborted'
                    break
    except KeyboardInterrupt:
        result = 'keyboard_interrupt'
    return result, consecutive_errors


def test_a4_single_error_survived_and_motors_stopped():
    ca = _SpyCA()
    c = _FakeController(['ok', 'raise', 'ok', 'ok', 'idle'])
    result, errs = _run_loop_core(c, ca, stop_when_idle=True)
    assert result == 'success', result
    assert ca.stop_calls == 1, ca.stop_calls          # моторы стопнуты на сбое
    assert errs == 0, errs                              # счётчик сброшен успешным тактом


def test_a4_consecutive_errors_abort():
    ca = _SpyCA()
    c = _FakeController(['raise'] * 15)
    result, errs = _run_loop_core(c, ca, stop_when_idle=False)
    assert result == 'aborted', result
    assert errs == 10, errs
    assert ca.stop_calls == 10, ca.stop_calls


def test_a4_keyboard_interrupt_not_swallowed():
    """except Exception НЕ ловит KeyboardInterrupt (он BaseException) -> Ctrl+C жив."""
    ca = _SpyCA()
    c = _FakeController(['ok', 'kbd'])
    result, errs = _run_loop_core(c, ca, stop_when_idle=False)
    assert result == 'keyboard_interrupt', result
    assert ca.stop_calls == 0, "на KeyboardInterrupt stop_robot из except Exception не зовётся"


def test_a4_sporadic_errors_do_not_abort():
    ca = _SpyCA()
    c = _FakeController(['raise', 'ok', 'raise', 'ok', 'raise', 'ok', 'idle'])
    result, errs = _run_loop_core(c, ca, stop_when_idle=True)
    assert result == 'success', result   # счётчик сбрасывается -> до выхода не доходит


# ──────────────── A5: анти-зацикливание в критической ветке ────────────────
# Реплика критической ветки map/navigation_controller.py:_safe_exploration_behavior
# (if obstacle_distance < critical_distance). Источник правды — там; здесь пиннится
# выбор направления и работа счётчика.

class _FakeNav:
    def __init__(self, choose_dir):
        self.consecutive_rotations = 0
        self.max_consecutive_rotations = 3
        self._choose = choose_dir
        self.rotations = []   # направления _safe_rotate по тактам
        self.backs = 0        # вызовы _emergency_stop_and_back

    def _choose_turn_direction(self):
        return self._choose

    def _emergency_stop_and_back(self):
        self.backs += 1

    def _safe_rotate(self, direction, duration):
        self.rotations.append(direction)

    def critical_branch(self):
        # === 1:1 с боевой веткой (после фикса A5) ===
        self._emergency_stop_and_back()
        if self.consecutive_rotations >= self.max_consecutive_rotations:
            self.consecutive_rotations = 0
            opposite = 'right' if self._choose_turn_direction() == 'left' else 'left'
            self._safe_rotate(opposite, 1.2)
        else:
            self._safe_rotate(self._choose_turn_direction(), 1.0)
            self.consecutive_rotations += 1


def test_a5_escape_turns_opposite_after_max_rotations():
    """Свободная сторона left: 3 поворота туда, затем РАЗВОРОТ вправо + сброс."""
    nav = _FakeNav('left')
    for _ in range(5):
        nav.critical_branch()
    assert nav.rotations == ['left', 'left', 'left', 'right', 'left'], nav.rotations
    assert nav.backs == 5, nav.backs                  # отъезд КАЖДЫЙ критический такт
    assert nav.consecutive_rotations == 1


def test_a5_escape_direction_mirrors_choice():
    """Свободная сторона right -> эскейп крутит left."""
    nav = _FakeNav('right')
    for _ in range(4):
        nav.critical_branch()
    assert nav.rotations == ['right', 'right', 'right', 'left'], nav.rotations


def test_a5_normal_path_unchanged_below_max():
    """При counter < max ветка идентична прежней (поворот к свободному + инкремент)."""
    nav = _FakeNav('left')
    nav.critical_branch()
    nav.critical_branch()
    assert nav.rotations == ['left', 'left']
    assert nav.consecutive_rotations == 2


def test_a5_escape_fires_exactly_at_max():
    nav = _FakeNav('left')
    fired_at = None
    for i in range(1, 5):
        before = nav.consecutive_rotations
        nav.critical_branch()
        if before >= nav.max_consecutive_rotations:
            fired_at = i
            break
    assert fired_at == 4, fired_at


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
