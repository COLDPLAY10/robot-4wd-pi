#!/usr/bin/env python3
"""
Драйвер для лидара Yahboom T-Mini Plus (Oradar MS200).

API совместим со старым lidar.py — класс LidarDriver с методами
connect, start_scan, get_scan, get_scan_degrees, stop_scan, disconnect,
print_scan_info, force_stop.

Протокол MS200 (47-байтный фрейм):
  byte  0       : 0x54 (header)
  byte  1       : 0x2C (ver=2, points=12)
  bytes 2-3     : speed, uint16 LE (deg/s; делить на 360 -> Hz)
  bytes 4-5     : start_angle, uint16 LE (0.01°)
  bytes 6-41    : 12 точек × (uint16 distance_mm, uint8 confidence)
  bytes 42-43   : end_angle, uint16 LE (0.01°)
  bytes 44-45   : timestamp, uint16 LE
  byte  46      : crc8
"""

import math
import os
import struct
import threading
import time
from typing import List, Optional, Tuple

import serial


FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C
POINTS_PER_FRAME = 12
FRAME_SIZE = 47

_CRC_TABLE = bytes([
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8,
])


def _crc8(buf: bytes) -> int:
    crc = 0
    for b in buf:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def _resolve_port(preferred: str) -> str:
    if preferred and os.path.exists(preferred):
        return preferred
    for fallback in ('/dev/oradar', '/dev/ttyACM0', '/dev/ttyACM1'):
        if os.path.exists(fallback):
            return fallback
    return preferred


class LidarDriver:
    """Драйвер MS200 (T-Mini Plus). Стримит точки, собирает их в полные обороты."""

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 230400,
                 timeout: float = 1.0,
                 min_distance_m: float = 0.05, max_distance_m: float = 12.0,
                 min_confidence: int = 10):
        self.port = _resolve_port(port)
        self.baudrate = baudrate
        self.timeout = timeout
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m
        self.min_confidence = min_confidence

        self.serial: Optional[serial.Serial] = None
        self.is_running = False
        self._thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        self.scan_data: List[Tuple[float, float]] = []
        self._current_rotation: List[Tuple[float, float, int]] = []
        self._prev_angle_deg: Optional[float] = None

        self.packet_count = 0
        self.bad_crc_count = 0
        self.speed_hz = 0.0

        print(f"[Lidar] MS200 на {self.port} @ {baudrate}")

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.2)
            self.serial.reset_input_buffer()
            print(f"[Lidar] Подключено к {self.port}")
            return True
        except Exception as e:
            print(f"[Lidar] Ошибка подключения к {self.port}: {e}")
            return False

    def start_scan(self) -> bool:
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False

        # MS200 льёт данные сама как только подаётся питание — никаких команд запуска не нужно.
        self.is_running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("[Lidar] Чтение запущено")
        return True

    def _read_loop(self):
        buf = bytearray()
        while self.is_running and self.serial and self.serial.is_open:
            try:
                chunk = self.serial.read(256)
                if chunk:
                    buf.extend(chunk)

                while len(buf) >= FRAME_SIZE:
                    idx = self._find_header(buf)
                    if idx < 0:
                        del buf[:-1]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE:
                        break

                    frame = bytes(buf[:FRAME_SIZE])
                    if _crc8(frame[:FRAME_SIZE - 1]) != frame[FRAME_SIZE - 1]:
                        self.bad_crc_count += 1
                        del buf[0]
                        continue

                    self._parse_frame(frame)
                    del buf[:FRAME_SIZE]
                    self.packet_count += 1

            except Exception as e:
                print(f"[Lidar] Ошибка в потоке: {e}")
                time.sleep(0.05)

    @staticmethod
    def _find_header(buf: bytearray) -> int:
        i = 0
        end = len(buf) - 1
        while i < end:
            if buf[i] == FRAME_HEADER and buf[i + 1] == FRAME_VER_LEN:
                return i
            i += 1
        return -1

    def _parse_frame(self, frame: bytes):
        speed = struct.unpack_from('<H', frame, 2)[0]
        start_angle_raw = struct.unpack_from('<H', frame, 4)[0]
        end_angle_raw = struct.unpack_from('<H', frame, 42)[0]

        self.speed_hz = speed / 360.0

        diff = (end_angle_raw + 36000 - start_angle_raw) % 36000
        step_deg = (diff / (POINTS_PER_FRAME - 1)) / 100.0
        start_deg = start_angle_raw / 100.0

        for i in range(POINTS_PER_FRAME):
            off = 6 + i * 3
            distance_mm, confidence = struct.unpack_from('<HB', frame, off)

            angle_deg = start_deg + i * step_deg
            if angle_deg >= 360.0:
                angle_deg -= 360.0

            # Детектируем переход через 0° → завершение оборота.
            # Делаем это до фильтра, чтобы момент wrap'а не зависел от качества точки.
            if (self._prev_angle_deg is not None and
                    self._prev_angle_deg - angle_deg > 180.0):
                if self._current_rotation:
                    with self.lock:
                        self.scan_data = [
                            (math.radians(a), d) for a, d, _ in self._current_rotation
                        ]
                    self._current_rotation = []
            self._prev_angle_deg = angle_deg

            distance_m = distance_mm / 1000.0
            if (confidence < self.min_confidence or
                    distance_m < self.min_distance_m or
                    distance_m > self.max_distance_m):
                continue

            self._current_rotation.append((angle_deg, distance_m, confidence))

    def get_scan(self) -> List[Tuple[float, float]]:
        """Снимок последнего полного оборота: список (угол_рад, дистанция_м)."""
        with self.lock:
            return list(self.scan_data)

    def get_scan_degrees(self) -> List[Tuple[float, float]]:
        """То же, но угол в градусах, отсортировано по углу."""
        with self.lock:
            deg = [(math.degrees(a) % 360.0, d) for a, d in self.scan_data]
        deg.sort(key=lambda x: x[0])
        return deg

    def print_scan_info(self):
        scan = self.get_scan_degrees()
        if not scan:
            print("[SCAN] Нет данных")
            return

        print(f"\n[SCAN] точек: {len(scan)}, скорость: {self.speed_hz:.1f} Гц, "
              f"фреймов принято: {self.packet_count}, CRC ошибок: {self.bad_crc_count}")

        groups: dict = {}
        for angle, dist in scan:
            key = int(round(angle))
            groups.setdefault(key, []).append(dist)

        for angle in sorted(groups.keys()):
            ds = groups[angle]
            print(f"  {angle:3d}°: {len(ds):3d} точек, среднее {sum(ds)/len(ds)*100:5.1f} см")

        coverage = len(groups) / 360.0 * 100
        print(f"\nПокрытие: {coverage:.1f}% ({len(groups)}/360 градусов)")

    def stop_scan(self):
        if not self.is_running:
            return
        print("[Lidar] Остановка чтения...")
        self.is_running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        print("[Lidar] Остановлено")

    def force_stop(self):
        self.is_running = False
        self._thread = None

    def disconnect(self):
        self.stop_scan()
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
            print("[Lidar] Порт закрыт")


def test_simple_scan():
    print("=" * 60)
    print("ПРОСТОЙ ТЕСТ MS200")
    print("=" * 60)

    lidar = LidarDriver()
    if not lidar.start_scan():
        print("Не удалось запустить")
        return

    try:
        for i in range(20):
            time.sleep(0.5)
            scan = lidar.get_scan_degrees()
            if scan:
                dists = [d for _, d in scan]
                angles = [a for a, _ in scan]
                print(f"[{i+1}/20] точек: {len(scan)}  "
                      f"дист: {min(dists)*100:.1f}-{max(dists)*100:.1f} см  "
                      f"углы: {min(angles):.1f}-{max(angles):.1f}°  "
                      f"скорость: {lidar.speed_hz:.1f} Гц")
            else:
                print(f"[{i+1}/20] нет данных")
        lidar.print_scan_info()
    except KeyboardInterrupt:
        pass
    finally:
        lidar.disconnect()


if __name__ == '__main__':
    test_simple_scan()
