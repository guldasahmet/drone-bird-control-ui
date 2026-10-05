"""Button-driven MJPEG recording of the two overlaid UI video panels.

Frames come from the GTK appsinks (boxes and aim already drawn). A bounded
queue drops frames instead of slowing the camera/Hailo path.
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Full, Queue
import shutil
from threading import Lock, Thread
from time import time_ns

import cv2
import numpy as np


VIDEO_NAMES = ("cam0_global_shutter.avi", "cam1_hq.avi")
MIN_FREE_BYTES = 1024 ** 3


@dataclass(frozen=True)
class VideoStatus:
    active: bool
    directory: Path | None
    saved: tuple[int, int]
    dropped: tuple[int, int]
    error: str | None


class VideoRecorder:
    def __init__(self, root, *, width=640, height=640, fps=20, quality=80):
        if width <= 0 or height <= 0 or fps <= 0 or not 1 <= quality <= 100:
            raise ValueError("Geçersiz video kayıt ayarı")
        self.root = Path(root)
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.lock = Lock()
        self.queue = Queue(maxsize=4)
        self.active = False
        self.directory = None
        self.saved = [0, 0]
        self.dropped = [0, 0]
        self.error = None
        self.worker = None

    def start(self):
        with self.lock:
            if self.active:
                raise RuntimeError("Video kaydı zaten açık")
            if self.worker is not None and self.worker.is_alive():
                raise RuntimeError("Önceki video kaydı hâlâ kapanıyor")
        self.root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.root).free < MIN_FREE_BYTES:
            raise OSError("Video kaydı için en az 1 GB boş alan gerekli")

        directory = self.root / datetime.now().strftime("%Y%m%d_%H%M%S")
        directory.mkdir(parents=True, exist_ok=False)
        writers = []
        for name in VIDEO_NAMES:
            writer = cv2.VideoWriter(
                str(directory / name), cv2.VideoWriter_fourcc(*"MJPG"),
                self.fps, (self.width, self.height),
            )
            if not writer.isOpened():
                for opened in writers:
                    opened.release()
                raise RuntimeError(f"Video dosyası açılamadı: {name}")
            writer.set(cv2.VIDEOWRITER_PROP_QUALITY, self.quality)
            writers.append(writer)
        # Wall-clock time per written frame, to line videos up with trace CSVs.
        times = (directory / "frames.csv").open("w", buffering=1)
        times.write("camera,frame,time_ns\n")

        with self.lock:
            self.directory = directory
            self.active = True
            self.error = None
            self.saved = [0, 0]
            self.dropped = [0, 0]
            self.queue = Queue(maxsize=4)
            self.worker = Thread(target=self._write, args=(writers, times),
                                 name="video-mjpeg", daemon=True)
            self.worker.start()
        return directory

    def offer(self, slot, frame):
        """Queue one RGB panel frame (bytes); drop it if the writer lags."""
        with self.lock:
            if not self.active:
                return False
            try:
                self.queue.put_nowait((slot, time_ns(), frame))
                return True
            except Full:
                self.dropped[slot] += 1
                return False

    def _write(self, writers, times):
        try:
            while True:
                with self.lock:
                    active = self.active
                if not active and self.queue.empty():
                    return
                try:
                    slot, stamp, frame = self.queue.get(timeout=0.1)
                except Empty:
                    continue
                image = np.frombuffer(frame, dtype=np.uint8).reshape(
                    self.height, self.width, 3
                )
                writers[slot].write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                with self.lock:
                    self.saved[slot] += 1
                    number = self.saved[slot]
                times.write(f"{slot},{number},{stamp}\n")
                if number % 200 == 0 and shutil.disk_usage(self.root).free < MIN_FREE_BYTES:
                    raise OSError("Boş alan 1 GB altına düştü; video kaydı durduruldu")
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.active = False
        finally:
            for writer in writers:
                writer.release()
            times.close()

    def stop(self):
        with self.lock:
            self.active = False
            worker = self.worker
        if worker is not None:
            worker.join(timeout=3.0)
            if worker.is_alive():
                with self.lock:
                    self.error = "Video yazıcısı zamanında durmadı"

    def status(self):
        with self.lock:
            return VideoStatus(self.active, self.directory, tuple(self.saved),
                               tuple(self.dropped), self.error)
