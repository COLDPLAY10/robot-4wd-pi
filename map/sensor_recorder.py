#!/usr/bin/env python3
"""
Запись сырых данных сенсоров для офлайн-реплея и будущих тестов.

Пишет в results/<запуск>/sensors/:
  lidar_raw.bin  — сырые байты порта лидара, кадрированы как
                   <d:timestamp><I:len><len байт>. Реплей через парсер драйвера
                   тестирует САМ драйвер — такой тест поймал бы «кольца» сразу.
  events.jsonl   — по строке JSON на событие: разобранный скан лидара,
                   ультразвук, команда (v,ω,dt), поза, отметка кадра камеры.
                   Единая временная шкала всех сенсоров; реплей через
                   SLAM/fusion/planner офлайн.

Запись НЕблокирующая: продюсеры (главный цикл и поток чтения лидара) кладут
события в очередь, фоновый поток пишет на диск. Очередь ограничена — при
переполнении событие отбрасывается (счётчик dropped), чтобы НИКОГДА не стопорить
контур управления и поток лидара (потеря сэмпла безопаснее зависания).

Камера: тяжёлые кадры RGB/глубины уже пишет захват сегментации в
results/<запуск>/segmentation/; сюда кладётся лишь лёгкое событие camera со
ссылками на эти файлы, чтобы все сенсоры были на одной шкале времени.

Файлы открываются ЛЕНИВО — при первом событии: запуск без данных не плодит
пустых каталогов.
"""

import json
import os
import queue
import struct
import threading
import time


class SensorRecorder:
    """Неблокирующий рекордер сырых данных сенсоров (фоновый поток-писатель)."""

    def __init__(self, out_dir: str, enabled: bool = True, max_queue: int = 20000):
        self.out_dir = out_dir
        self.enabled = enabled
        self._q: "queue.Queue" = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._thread = None
        self._raw_f = None
        self._ev_f = None
        self.dropped = 0      # событий отброшено из-за переполнения очереди
        self.written = 0      # событий записано на диск

    def start(self):
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._run, name='sensor-recorder',
                                        daemon=True)
        self._thread.start()

    # --- продюсеры: неблокирующие, зовутся из любого потока ---
    def _put(self, item):
        if not self.enabled:
            return
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def record_lidar_raw(self, t: float, chunk: bytes):
        """Сырые байты порта лидара (зовётся из потока чтения драйвера)."""
        self._put(('R', float(t), bytes(chunk)))

    def record_lidar_scan(self, t: float, revolution: int, scan):
        """Разобранный полный оборот [(угол_рад, дист_м), ...]."""
        self._put(('E', {'t': float(t), 'kind': 'lidar_scan', 'rev': int(revolution),
                         'scan': [[round(float(a), 5), round(float(d), 4)] for a, d in scan]}))

    def record_ultrasonic(self, t: float, dist_m: float):
        self._put(('E', {'t': float(t), 'kind': 'ultrasonic', 'dist_m': float(dist_m)}))

    def record_command(self, t: float, v: float, omega: float, dt: float):
        self._put(('E', {'t': float(t), 'kind': 'cmd',
                         'v': float(v), 'omega': float(omega), 'dt': float(dt)}))

    def record_pose(self, t: float, x: float, y: float, theta: float):
        self._put(('E', {'t': float(t), 'kind': 'pose',
                         'x': float(x), 'y': float(y), 'theta': float(theta)}))

    def record_camera(self, t: float, frame_path: str = None, depth_path: str = None):
        """Лёгкая отметка кадра камеры: ссылки на файлы в segmentation/."""
        self._put(('E', {'t': float(t), 'kind': 'camera',
                         'frame': os.path.basename(frame_path) if frame_path else None,
                         'depth': os.path.basename(depth_path) if depth_path else None}))

    # --- фоновый писатель ---
    def _open(self):
        os.makedirs(self.out_dir, exist_ok=True)
        self._raw_f = open(os.path.join(self.out_dir, 'lidar_raw.bin'), 'ab')
        self._ev_f = open(os.path.join(self.out_dir, 'events.jsonl'), 'a', encoding='utf-8')

    def _run(self):
        last_flush = time.time()
        while True:
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    break
                continue
            if self._raw_f is None:
                self._open()  # лениво: только когда реально есть что писать
            try:
                if item[0] == 'R':
                    _, t, chunk = item
                    self._raw_f.write(struct.pack('<dI', t, len(chunk)))
                    self._raw_f.write(chunk)
                else:
                    self._ev_f.write(json.dumps(item[1], ensure_ascii=False) + '\n')
                self.written += 1
            except Exception:
                pass  # запись логов не должна ронять робота
            now = time.time()
            if now - last_flush > 1.0:
                self._flush()
                last_flush = now
        self._flush()
        for f in (self._raw_f, self._ev_f):
            try:
                if f:
                    f.close()
            except Exception:
                pass

    def _flush(self):
        for f in (self._raw_f, self._ev_f):
            try:
                if f:
                    f.flush()
            except Exception:
                pass

    def stop(self, timeout: float = 3.0):
        """Слить очередь, дописать, закрыть файлы."""
        if not self.enabled or self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=timeout)


def iter_lidar_raw(path: str):
    """Генератор (timestamp, chunk_bytes) из lidar_raw.bin — для реплея/проверки."""
    with open(path, 'rb') as f:
        head = f.read(12)
        while len(head) == 12:
            t, n = struct.unpack('<dI', head)
            chunk = f.read(n)
            if len(chunk) < n:
                break
            yield t, chunk
            head = f.read(12)
