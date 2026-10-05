"""Bounded, non-blocking JPEG capture for the two live camera streams."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from queue import Empty, Full, Queue
import shutil
from threading import Lock, Thread
from time import monotonic, time_ns

import cv2
import numpy as np


CAMERA_DIRS = ("cam0_global_shutter", "cam1_hq")
MIN_FREE_BYTES = 1024 ** 3


@dataclass(frozen=True)
class RecordingStatus:
    active: bool
    directory: Path | None
    saved: tuple[int, int]
    dropped: tuple[int, int]
    error: str | None


class DatasetRecorder:
    def __init__(self, root, *, width=640, height=640, fps=2, quality=92,
                 profile_name=None, inference_vertical_flip=False):
        if width <= 0 or height <= 0 or fps <= 0 or not 1 <= quality <= 100:
            raise ValueError("Geçersiz dataset kayıt ayarı")
        self.root = Path(root)
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.profile_name = profile_name
        self.inference_vertical_flip = bool(inference_vertical_flip)
        self.queue = Queue(maxsize=8)
        self.lock = Lock()
        self.active = False
        self.directory = None
        self.saved = [0, 0]
        self.dropped = [0, 0]
        self.error = None
        self.last_sample_at = [float("-inf"), float("-inf")]
        self.sequence = [0, 0]
        self.worker = None

    def start(self):
        with self.lock:
            if self.active:
                raise RuntimeError("Dataset kaydı zaten açık")
            if self.worker is not None and self.worker.is_alive():
                raise RuntimeError("Önceki dataset kaydı hâlâ kapanıyor")
        if shutil.disk_usage(self.root.parent).free < MIN_FREE_BYTES:
            raise OSError("Dataset kaydı için en az 1 GB boş alan gerekli")

        session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        directory = self.root / session
        for camera_dir in CAMERA_DIRS:
            (directory / camera_dir).mkdir(parents=True, exist_ok=False)
        (directory / "session.json").write_text(
            json.dumps(
                {
                    "created_utc": session,
                    "camera_directories": CAMERA_DIRS,
                    "width": self.width,
                    "height": self.height,
                    "format": "Standard RGB JPEG of the frames the model sees "
                              "(before Hailo overlay)",
                    "jpeg_color": "standard",
                    "sample_fps_per_camera": self.fps,
                    "jpeg_quality": self.quality,
                    "model_profile": self.profile_name,
                    "jpeg_orientation": ("inference" if self.inference_vertical_flip
                                         else "camera_native"),
                    "inference_vertical_flip": self.inference_vertical_flip,
                },
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )
        with self.lock:
            self.directory = directory
            self.active = True
            self.error = None
            self.saved = [0, 0]
            self.dropped = [0, 0]
            self.last_sample_at = [float("-inf"), float("-inf")]
            self.sequence = [0, 0]
            self.queue = Queue(maxsize=8)
            self.worker = Thread(target=self._write, name="dataset-jpeg", daemon=True)
            self.worker.start()
        return directory

    def offer(self, slot, raw, now=None):
        """Queue an existing immutable camera buffer without delaying capture."""
        if slot not in (0, 1):
            raise ValueError(f"Bilinmeyen kamera: {slot}")
        now = monotonic() if now is None else now
        with self.lock:
            if not self.active or now - self.last_sample_at[slot] < 1 / self.fps:
                return False
            self.last_sample_at[slot] = now
            self.sequence[slot] += 1
            sequence = self.sequence[slot]
            try:
                self.queue.put_nowait((slot, sequence, time_ns(), raw))
                return True
            except Full:
                self.dropped[slot] += 1
                return False

    def _write(self):
        while True:
            with self.lock:
                active = self.active
            if not active and self.queue.empty():
                return
            try:
                slot, sequence, timestamp_ns, raw = self.queue.get(timeout=0.1)
            except Empty:
                continue

            partial = None
            try:
                frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                    self.height, self.width, 3
                )
                # Camera buffers are RGB; OpenCV encodes BGR. Save upright, as
                # the model sees it after the inference flip.
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if self.inference_vertical_flip:
                    frame = np.flipud(frame)
                ok, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
                )
                if not ok:
                    raise RuntimeError("JPEG sıkıştırması başarısız")
                destination = self.directory / CAMERA_DIRS[slot] / (
                    f"{sequence:06d}_{timestamp_ns}.jpg"
                )
                partial = destination.with_suffix(".jpg.part")
                with partial.open("wb") as output:
                    output.write(encoded.tobytes())
                os.replace(partial, destination)
                with self.lock:
                    self.saved[slot] += 1
                    total = sum(self.saved)
                if total % 20 == 0 and shutil.disk_usage(self.root).free < MIN_FREE_BYTES:
                    raise OSError("Boş alan 1 GB altına düştü; dataset kaydı durduruldu")
            except Exception as exc:
                if partial is not None:
                    partial.unlink(missing_ok=True)
                with self.lock:
                    self.error = str(exc)
                    self.active = False
                while True:
                    try:
                        pending_slot, *_ = self.queue.get_nowait()
                    except Empty:
                        break
                    with self.lock:
                        self.dropped[pending_slot] += 1
                    self.queue.task_done()
                self.queue.task_done()
                return
            self.queue.task_done()

    def stop(self):
        with self.lock:
            self.active = False
            worker = self.worker
        if worker is not None:
            worker.join(timeout=3.0)
            if worker.is_alive():
                with self.lock:
                    self.error = "JPEG yazıcısı zamanında durmadı"

    def status(self):
        with self.lock:
            return RecordingStatus(
                self.active,
                self.directory,
                tuple(self.saved),
                tuple(self.dropped),
                self.error,
            )
