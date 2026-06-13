#!/usr/bin/env python3
"""
Драйвер лидара YDLidar T-mini Plus (комплект Yahboom).

ВАЖНО про железо: лидар отвечает пакетами с заголовком AA 55 — это протокол
семейства YDLidar (X4/T-mini), а НЕ Oradar MS200. Разбор протокола и причин
прежних карт-«колец» — samples/ANALYSIS.md.

Команды старта/остановки (A5 60 / A5 65 / A5 52) оставлены ровно те, что
в прежнем драйвере — с ними лидар реально запускался. Заменён только парсер:
прежний резал поток на ФИКСИРОВАННЫЕ 9 байт и читал поля заголовка как
«дистанцию и угол», из-за чего углы были шумом и карты рисовались
концентрическими «кольцами» вокруг робота.

Формат пакета (переменная длина):
  byte 0-1 : AA 55 — заголовок
  byte 2   : CT  — тип пакета; bit0=1 — стартовый пакет нового оборота
  byte 3   : LSN — число сэмплов в пакете
  byte 4-5 : FSA — начальный угол пакета: (raw >> 1) / 64 — градусы
  byte 6-7 : LSA — конечный угол пакета:  (raw >> 1) / 64 — градусы
  byte 8-9 : CS  — контрольная сумма: XOR всех 16-битных LE-слов пакета
  далее    : LSN сэмплов по 2 ИЛИ 3 байта (см. авто-детект ниже):
             2 байта: дистанция_мм = raw / 4 (X4-стиль)
             3 байта: [интенсивность, dist_L, dist_H], дистанция_мм = raw

Углы точек интерполируются между FSA и LSA. Корректность разбора углов
подтверждена реальными пакетами этого лидара: соседние пакеты дают
непрерывную развёртку 320.3°→337.4°→338.0°→354.9°→(переход через 0)→2.0°.

Размер сэмпла (2 или 3 байта) авто-детектится по позиции следующего
заголовка AA 55 — у YDLidar-моделей он разный, а документации на конкретную
ревизию у нас нет. Делитель дистанции при необходимости калибруется:
поставить робота в 1 м от стены и запустить `python3 lidar.py` — тест
покажет варианты интерпретации.

API совместим со старым драйвером: connect / start_scan / get_scan /
get_scan_degrees / print_scan_info / stop_scan / force_stop / disconnect.
get_scan() возвращает [(угол_рад, дистанция_м)] в системе робота
(0 — вперёд, против часовой положительное), как ждут SLAM и SensorFusion.
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import serial

PROTOCOL_HEADER = b'\xAA\x55'
HEADER_SIZE = 10
MAX_LSN = 160          # больше — заведомо ложный заголовок
_WAIT_MORE = -1        # сигнал «в буфере мало данных, ждём»


class LidarDriver:
    """Драйвер YDLidar T-mini Plus с честным парсером протокола."""

    # ===== Калибровка ориентации/масштаба (поправить по результатам теста) =====
    # YDLidar считает углы ПО ЧАСОВОЙ (вид сверху), SLAM ждёт против часовой.
    # Если карта получается зеркальной — выставить False.
    CLOCKWISE = True
    # Угол установки лидара на роботе: на сколько градусов «ноль» лидара
    # повёрнут относительно «вперёд» робота (по данным лидара, до знака).
    ANGLE_OFFSET_DEG = 0.0
    # Делитель сырой дистанции до миллиметров. None = авто по размеру сэмпла
    # (2 байта → 4.0, X4-стиль; 3 байта → 1.0). Если тест у стены в 1 м
    # показывает 4 м — поставить 4.0 явно; если 0.25 м — поставить 1.0.
    DISTANCE_DIVISOR: Optional[float] = None
    # Валидный диапазон дистанций, м (T-mini Plus: 0.02–12 м по паспорту)
    MIN_RANGE_M = 0.02
    MAX_RANGE_M = 16.0

    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 230400):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.is_running = False
        self.lock = threading.Lock()

        # Последний ПОЛНЫЙ оборот — то, что отдаёт get_scan()
        self.scan_data: List[Tuple[float, float]] = []
        # Текущий оборот (копится между стартовыми пакетами CT&1)
        self._current_rev: List[Tuple[float, float]] = []

        # Счётчики. packet_count = только ВАЛИДНЫЕ пакеты — по нему
        # NavigationController отличает настоящий лидар от чужого устройства.
        self.packet_count = 0
        self.bad_checksum_count = 0
        self.revolution_count = 0

        # Авто-детект размера сэмпла (2 или 3 байта)
        self.sample_size: Optional[int] = None
        self._layout_votes = {2: 0, 3: 0}
        self.distance_divisor: Optional[float] = self.DISTANCE_DIVISOR

        print(f"[Lidar] YDLidar T-mini Plus на {port}, скорость {baudrate}")

    # ------------------------------------------------------------ подключение

    def connect(self) -> bool:
        """Открыть serial-порт."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            print(f"[Lidar] Подключено к {self.port}")
            return True
        except Exception as e:
            print(f"[Lidar] Ошибка подключения: {e}")
            return False

    def start_scan(self) -> bool:
        """Запустить мотор и поток данных, поднять поток чтения."""
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False
        try:
            print("[Lidar] Запуск сканирования...")
            # Последовательность из прежнего драйвера — с ней лидар
            # гарантированно запускался на этом роботе. Не менять без нужды.
            self.serial.write(b'\xA5\x65')  # стоп (сброс состояния)
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            self.serial.write(b'\xA5\x52')  # мотор
            time.sleep(1.0)                 # раскрутка
            self.serial.write(b'\xA5\x60')  # старт сканирования
            time.sleep(0.2)

            self.is_running = True
            thread = threading.Thread(target=self._read_thread, daemon=True)
            thread.start()
            print("[Lidar] Сканирование запущено")
            return True
        except Exception as e:
            print(f"[Lidar] Ошибка запуска: {e}")
            return False

    # ------------------------------------------------------------ чтение/парсинг

    def _read_thread(self):
        """Фоновый поток: читает порт и скармливает байты парсеру."""
        buf = bytearray()
        while self.is_running and self.serial and self.serial.is_open:
            try:
                waiting = self.serial.in_waiting
                chunk = self.serial.read(waiting if waiting else 1)
                if chunk:
                    buf.extend(chunk)
                    self._process_buffer(buf)
                # Защита от разрастания при полной рассинхронизации
                if len(buf) > 65536:
                    del buf[:32768]
            except Exception as e:
                print(f"[Lidar] Ошибка в потоке чтения: {e}")
                buf.clear()
                time.sleep(0.1)

    def _process_buffer(self, buf: bytearray):
        """Выделить из буфера все полные пакеты и распарсить их."""
        while True:
            idx = buf.find(PROTOCOL_HEADER)
            if idx < 0:
                # Заголовка нет — оставляем хвост (вдруг AA пришёл, 55 в пути)
                if len(buf) > 1:
                    del buf[:len(buf) - 1]
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < HEADER_SIZE:
                return  # заголовок неполный — ждём байтов

            ct, lsn = buf[2], buf[3]
            if not (1 <= lsn <= MAX_LSN):
                del buf[:2]  # ложный заголовок внутри данных
                continue

            size = self.sample_size
            if size is None:
                size = self._detect_layout(buf, lsn)
                if size == _WAIT_MORE:
                    return
                if size is None:
                    del buf[:2]
                    continue

            need = HEADER_SIZE + lsn * size
            if len(buf) < need:
                return  # пакет неполный — ждём

            packet = bytes(buf[:need])
            del buf[:need]
            self._handle_packet(packet, ct, lsn, size)

    def _detect_layout(self, buf: bytearray, lsn: int) -> Optional[int]:
        """
        Определить размер сэмпла (2 или 3 байта) по позиции следующего
        заголовка AA 55. Голосуем по нескольким пакетам, потом фиксируем.

        Returns:
            2 | 3 — размер для текущего пакета;
            _WAIT_MORE — мало данных в буфере;
            None — ложный заголовок (следующего AA 55 нет ни в одном варианте).
        """
        need3 = HEADER_SIZE + lsn * 3 + 2
        if len(buf) < need3:
            return _WAIT_MORE

        matches = [s for s in (2, 3)
                   if buf[HEADER_SIZE + lsn * s: HEADER_SIZE + lsn * s + 2] == PROTOCOL_HEADER]
        if not matches:
            return None
        if len(matches) == 2:
            # Совпали оба (редкость) — решает контрольная сумма 2-байтного варианта
            cand = 2 if self._checksum_ok(buf, lsn) else 3
        else:
            cand = matches[0]

        self._layout_votes[cand] += 1
        if self.sample_size is None and self._layout_votes[cand] >= 6:
            self.sample_size = cand
            if self.distance_divisor is None:
                self.distance_divisor = 4.0 if cand == 2 else 1.0
            print(f"[Lidar] Формат сэмпла определён: {cand} байт/точка, "
                  f"делитель дистанции {self.distance_divisor}")
        return cand

    @staticmethod
    def _checksum_ok(buf, lsn: int) -> bool:
        """CS = XOR всех 16-битных LE-слов пакета (кроме самого CS). Для 2-байтных сэмплов."""
        cs_calc = (0x55AA
                   ^ (buf[2] | (buf[3] << 8))
                   ^ (buf[4] | (buf[5] << 8))
                   ^ (buf[6] | (buf[7] << 8)))
        for i in range(lsn):
            o = HEADER_SIZE + 2 * i
            cs_calc ^= buf[o] | (buf[o + 1] << 8)
        return cs_calc == (buf[8] | (buf[9] << 8))

    def _handle_packet(self, packet: bytes, ct: int, lsn: int, size: int):
        """Распарсить один пакет: углы интерполяцией FSA→LSA, дистанции из сэмплов."""
        if size == 2 and not self._checksum_ok(packet, lsn):
            self.bad_checksum_count += 1
            return

        fsa = packet[4] | (packet[5] << 8)
        lsa = packet[6] | (packet[7] << 8)
        start_deg = (fsa >> 1) / 64.0
        end_deg = (lsa >> 1) / 64.0
        span = end_deg - start_deg
        if span < 0:
            span += 360.0  # пакет через 0°

        divisor = self.distance_divisor or (4.0 if size == 2 else 1.0)

        points = []
        for i in range(lsn):
            o = HEADER_SIZE + size * i
            if size == 2:
                raw = packet[o] | (packet[o + 1] << 8)
            else:
                # 3 байта: [интенсивность, dist_L, dist_H]
                raw = packet[o + 1] | (packet[o + 2] << 8)
            if raw == 0:
                continue  # невалидное измерение
            dist_m = (raw / divisor) / 1000.0
            if not (self.MIN_RANGE_M <= dist_m <= self.MAX_RANGE_M):
                continue

            frac = i / (lsn - 1) if lsn > 1 else 0.0
            angle_deg = (start_deg + span * frac + self.ANGLE_OFFSET_DEG) % 360.0
            angle_rad = math.radians(angle_deg)
            if self.CLOCKWISE:
                angle_rad = -angle_rad
            # нормализация в [-pi, pi] — так ждут sensor_fusion и scan_matcher
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            points.append((angle_rad, dist_m))

        with self.lock:
            if ct & 0x01:
                # Стартовый пакет нового оборота: публикуем накопленный оборот
                if self._current_rev:
                    self.scan_data = self._current_rev
                    self.revolution_count += 1
                self._current_rev = []
            self._current_rev.extend(points)
            # Fallback: если стартовые пакеты не приходят, не копим бесконечно
            if len(self._current_rev) > 2000:
                self.scan_data = self._current_rev
                self._current_rev = []

        self.packet_count += 1
        if self.packet_count % 500 == 0:
            print(f"[Lidar] Пакетов: {self.packet_count}, оборотов: {self.revolution_count}, "
                  f"битых CS: {self.bad_checksum_count}")

    # ------------------------------------------------------------ доступ к данным

    def get_scan(self) -> List[Tuple[float, float]]:
        """
        Последний ПОЛНЫЙ оборот: [(угол_рад, дистанция_м), ...].
        Угол в системе робота: 0 — вперёд, против часовой положительное,
        диапазон [-pi, pi].

        До первого полного оборота возвращает [] — частичный сектор в SLAM
        и слияние отдавать нельзя: «пустые» направления читаются как простор.
        """
        with self.lock:
            return list(self.scan_data)

    def get_scan_degrees(self) -> List[Tuple[float, float]]:
        """Скан в градусах [0..360), отсортированный по углу."""
        with self.lock:
            data = list(self.scan_data) if self.scan_data else list(self._current_rev)
        scan_deg = [(math.degrees(a) % 360.0, d) for a, d in data]
        scan_deg.sort(key=lambda x: x[0])
        return scan_deg

    def print_scan_info(self):
        """Сводка по текущему скану: покрытие по углам, дистанции."""
        scan = self.get_scan_degrees()
        if not scan:
            print("[SCAN] Нет данных")
            return

        print(f"\n[SCAN] Точек в обороте: {len(scan)}")
        angles = set(int(a) for a, _ in scan)
        coverage = len(angles) / 360.0 * 100.0
        dists = [d for _, d in scan]
        print(f"Покрытие: {coverage:.1f}% ({len(angles)} из 360 градусов)")
        print(f"Дистанции: мин {min(dists):.2f} м, медиана "
              f"{sorted(dists)[len(dists)//2]:.2f} м, макс {max(dists):.2f} м")
        print(f"Пакетов: {self.packet_count}, оборотов: {self.revolution_count}, "
              f"битых CS: {self.bad_checksum_count}, "
              f"формат: {self.sample_size} байт/сэмпл, делитель: {self.distance_divisor}")
        if coverage > 50:
            print("✓ Лидар работает нормально")
        else:
            print("⚠ Лидар покрывает не весь круг")

    # ------------------------------------------------------------ остановка

    def stop_scan(self):
        """Остановить сканирование и мотор."""
        if not self.is_running:
            return
        print("[Lidar] Остановка...")
        self.is_running = False
        time.sleep(0.2)
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b'\xA5\x65')  # стоп сканирования
                time.sleep(0.1)
                self.serial.write(b'\xA5\x50')  # мотор офф (как в прежнем драйвере)
                time.sleep(0.1)
            except Exception as e:
                print(f"[Lidar] Ошибка при остановке: {e}")
        print("[Lidar] Сканирование остановлено")

    def force_stop(self):
        """Принудительная остановка (для экстренных случаев)."""
        print("[Lidar] Принудительная остановка...")
        self.is_running = False
        if self.serial and self.serial.is_open:
            try:
                for _ in range(3):
                    self.serial.write(b'\xA5\x65')
                    time.sleep(0.05)
                    self.serial.write(b'\xA5\x50')
                    time.sleep(0.05)
            except Exception:
                pass
        print("[Lidar] Остановлен")

    def disconnect(self):
        """Остановка и закрытие порта."""
        self.stop_scan()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[Lidar] Отключен")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# ================================================================== тест

def test_scan(port: str = '/dev/ttyUSB1', baudrate: int = 230400,
              duration: float = 10.0):
    """
    Тест на роботе: поток, покрытие, и КАЛИБРОВКА ДИСТАНЦИИ.
    Поставьте робота передом к стене на измеренном расстоянии (например 1.00 м)
    и сравните вывод: какой из вариантов делителя совпал с рулеткой — тот и
    прописать в LidarDriver.DISTANCE_DIVISOR (если авто-выбор ошибся).
    """
    print("=" * 60)
    print("ТЕСТ YDLIDAR T-MINI PLUS")
    print("=" * 60)

    lidar = LidarDriver(port, baudrate)
    if not lidar.connect():
        print("Не удалось подключиться")
        return
    if not lidar.start_scan():
        print("Не удалось запустить сканирование")
        lidar.disconnect()
        return

    try:
        t_end = time.time() + duration
        while time.time() < t_end:
            time.sleep(1.0)
            scan = lidar.get_scan()
            front = [d for a, d in scan if abs(a) <= math.radians(15)]
            front_med = sorted(front)[len(front) // 2] if front else None
            print(f"точек: {len(scan):4d} | пакетов: {lidar.packet_count:5d} | "
                  f"оборотов: {lidar.revolution_count:3d} | битых CS: {lidar.bad_checksum_count:3d} | "
                  f"фронт (±15°): {f'{front_med:.2f} м' if front_med else '—'}")

        lidar.print_scan_info()

        scan = lidar.get_scan()
        front = sorted(d for a, d in scan if abs(a) <= math.radians(15))
        if front:
            med = front[len(front) // 2]
            div = lidar.distance_divisor or 1.0
            print("\nКалибровка дистанции (медиана фронта ±15°):")
            print(f"  текущий делитель {div}: {med:.2f} м")
            print(f"  если бы делитель {div * 4}: {med / 4:.2f} м")
            print(f"  если бы делитель {div / 4}: {med * 4:.2f} м")
            print("Сравните с рулеткой и при расхождении поправьте "
                  "LidarDriver.DISTANCE_DIVISOR.")
    except KeyboardInterrupt:
        print("\nПрервано пользователем")
    finally:
        lidar.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Тест драйвера YDLidar T-mini Plus")
    parser.add_argument('--port', default='/dev/ttyUSB1')
    parser.add_argument('--baud', type=int, default=230400)
    parser.add_argument('--duration', type=float, default=10.0,
                        help="длительность теста, сек")
    args = parser.parse_args()
    test_scan(args.port, args.baud, args.duration)
