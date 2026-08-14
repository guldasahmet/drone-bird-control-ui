"""Hailo ByteTrack hedef takibi ve STM32 UART haberleşmesi.

UART paket yapısı (5 byte, little-endian)::

    <Bhh

    Byte 0..0 : 0xFF = takip ediliyor, 0xFE = hedef merkezde
    Byte 1..2 : hata_x, signed int16
    Byte 3..4 : hata_y, signed int16

Hata işaretleri::

    hata_x < 0 : hedef görüntü merkezinin solunda
    hata_x > 0 : hedef görüntü merkezinin sağında
    hata_y < 0 : hedef görüntü merkezinin üstünde
    hata_y > 0 : hedef görüntü merkezinin altında

X eksenine göre aynalama, görüntünün yukarı-aşağı çevrilmesidir. Gerçek
görüntü karesine erişilen yerde ``mirror_frame_x_axis(frame)`` çağrılmalıdır.
GStreamer kullanılıyorsa aynı işlem modelden önce
``videoflip method=vertical-flip`` elemanı eklenerek yapılabilir.
"""

from dataclasses import dataclass
from math import hypot
from threading import RLock
from time import monotonic, sleep
from types import SimpleNamespace
import struct

import hailo
import numpy as np
import serial

from hailo_apps.python.core.tracker.basetrack import BaseTrack
from hailo_apps.python.core.tracker.byte_tracker import BYTETracker


CLASS_LABELS = ("CELL PHONE",)

PACKET_TRACKING = 0xFF
PACKET_LOCKED = 0xFE
UART_INVERT_X = True


@dataclass(frozen=True)
class TargetTelemetry:
    track_id: int
    label: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    center_x: float
    center_y: float
    dx_px: float
    dy_px: float
    error_px: float
    dx_norm: float
    dy_norm: float
    error_norm: float


@dataclass(frozen=True)
class TrackingResult:
    targets: tuple[TargetTelemetry, ...]
    active_id: int | None
    state: str
    lock_seconds: float
    raw_count: int


def mirror_frame_x_axis(frame: np.ndarray) -> np.ndarray:
    """Görüntüyü x eksenine göre aynalar (yukarı-aşağı çevirir).

    Bu fonksiyon, görüntü Hailo modeline gönderilmeden önce çağrılmalıdır.
    Sağ-sol ayna görüntüsü istenirse ``np.flip(frame, axis=1)`` kullanılmalıdır.
    """

    if frame is None or frame.ndim < 2:
        raise ValueError("Aynalanacak geçerli bir görüntü karesi gerekli")

    # ascontiguousarray, negatif stride nedeniyle GStreamer/OpenCV tarafında
    # oluşabilecek uyumsuzlukları engeller.
    return np.ascontiguousarray(np.flip(frame, axis=0))


def _detection_box(detection, mirror_x_axis=False):
    bbox = detection.get_bbox()

    x1 = max(0.0, min(1.0, float(bbox.xmin())))
    y1 = max(0.0, min(1.0, float(bbox.ymin())))
    x2 = max(0.0, min(1.0, x1 + float(bbox.width())))
    y2 = max(0.0, min(1.0, y1 + float(bbox.height())))

    if x2 <= x1 or y2 <= y1:
        return None

    if mirror_x_axis:
        # Görüntü yukarı-aşağı çevrildiğinde kutunun yeni Y sınırları.
        y1, y2 = 1.0 - y2, 1.0 - y1

    return np.asarray([x1, y1, x2, y2], dtype=np.float64)


def _iou(box_a, box_b):
    left = max(float(box_a[0]), float(box_b[0]))
    top = max(float(box_a[1]), float(box_b[1]))
    right = min(float(box_a[2]), float(box_b[2]))
    bottom = min(float(box_a[3]), float(box_b[3]))

    intersection = max(0.0, right - left) * max(0.0, bottom - top)

    area_a = max(0.0, float(box_a[2] - box_a[0])) * max(
        0.0,
        float(box_a[3] - box_a[1]),
    )
    area_b = max(0.0, float(box_b[2] - box_b[0])) * max(
        0.0,
        float(box_b[3] - box_b[1]),
    )

    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def _greedy_matches(tracks, candidates, minimum_iou):
    possible = []

    for track_index, track in enumerate(tracks):
        for candidate_index, candidate in enumerate(candidates):
            score = _iou(track.tlbr, candidate[1])

            if score >= minimum_iou:
                possible.append((score, track_index, candidate_index))

    possible.sort(reverse=True)

    used_tracks = set()
    used_candidates = set()
    matches = []

    for _score, track_index, candidate_index in possible:
        if track_index in used_tracks or candidate_index in used_candidates:
            continue

        used_tracks.add(track_index)
        used_candidates.add(candidate_index)
        matches.append((tracks[track_index], candidates[candidate_index]))

    return matches


def _deduplicate_candidates(candidates, maximum_iou=0.70):
    """Birbirine çok yakın kutulardan güveni en yüksek olanı korur."""

    kept = []

    for candidate in sorted(
        candidates,
        key=lambda item: item[2],
        reverse=True,
    ):
        if any(
            _iou(candidate[1], previous[1]) >= maximum_iou
            for previous in kept
        ):
            continue

        kept.append(candidate)

    return kept


def _target_telemetry(track_id, label, confidence, box, width, height):
    if width <= 0 or height <= 0:
        raise ValueError("Görüntü genişliği ve yüksekliği pozitif olmalı")

    # Hailo kutusu normalize koordinatlardadır; önce hedef merkezini gerçek
    # piksel koordinatına çeviriyoruz.
    center_x = float((box[0] + box[2]) / 2.0)
    center_y = float((box[1] + box[3]) / 2.0)
    target_x_px = center_x * float(width)
    target_y_px = center_y * float(height)
    frame_center_x_px = float(width) / 2.0
    frame_center_y_px = float(height) / 2.0

    # İstenen işaretli hata hesabı:
    # hata_x = hedefin merkez X koordinatı - görüntünün merkez X koordinatı
    # hata_y = hedefin merkez Y koordinatı - görüntünün merkez Y koordinatı
    dx_px = target_x_px - frame_center_x_px
    dy_px = target_y_px - frame_center_y_px

    # Her ekseni kendi yarım görüntü boyutuna göre [-1, +1] ölçeğine getir.
    dx_norm = dx_px / frame_center_x_px
    dy_norm = dy_px / frame_center_y_px

    return TargetTelemetry(
        track_id=track_id,
        label=label,
        confidence=confidence,
        x1=float(box[0]),
        y1=float(box[1]),
        x2=float(box[2]),
        y2=float(box[3]),
        center_x=center_x,
        center_y=center_y,
        dx_px=dx_px,
        dy_px=dy_px,
        error_px=hypot(dx_px, dy_px),
        dx_norm=dx_norm,
        dy_norm=dy_norm,
        error_norm=hypot(dx_norm, dy_norm),
    )


class ClassAwareByteTracker:
    """Seçilen sınıfları takip eder ve aktif hedefi UART ile gönderir."""

    def __init__(
        self,
        frame_rate=40,
        low_threshold=0.10,
        high_threshold=0.25,
        new_track_threshold=0.35,
        display_threshold=0.30,
        match_threshold=0.85,
        track_buffer=2,
        min_confirmed_hits=2,
        display_match_iou=0.10,
        lock_tolerance_px=25,
        class_labels=CLASS_LABELS,
        serial_port="/dev/ttyACM0",
        baudrate=115200,
        serial_enabled=True,
    ):
        self._lock = RLock()

        self.frame_rate = frame_rate
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.new_track_threshold = new_track_threshold
        self.display_threshold = display_threshold
        self.match_threshold = match_threshold
        self.track_buffer = track_buffer
        self.min_confirmed_hits = min_confirmed_hits
        self.display_match_iou = display_match_iou
        self.lock_tolerance_px = max(0, int(lock_tolerance_px))
        self.class_labels = tuple(label.upper() for label in class_labels)

        if not self.class_labels:
            raise ValueError("En az bir takip sınıfı gerekli")

        self.trackers = {}
        self.confirmed_ids = set()
        self.active_id = None
        self.manual_id = None
        self.lock_started_at = None

        self.serial_port = serial_port
        self.baudrate = int(baudrate)
        self.serial_enabled = bool(serial_enabled)
        self.ser = None

        if self.serial_enabled:
            self._open_serial()

        self.reset()

    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0,
                write_timeout=0.1,
            )

            print(
                "Seri bağlantı başarılı: "
                f"{self.serial_port} @ {self.baudrate}"
            )

            sleep(0.3)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        except (OSError, serial.SerialException) as error:
            self.ser = None
            raise RuntimeError(
                f"Seri port açılamadı ({self.serial_port}): {error}"
            ) from error

    def reset(self):
        with self._lock:
            BaseTrack._count = 0

            args = SimpleNamespace(
                track_thresh=self.high_threshold,
                track_buffer=self.track_buffer,
                match_thresh=self.match_threshold,
                mot20=False,
            )

            self.trackers = {}

            for label in self.class_labels:
                tracker = BYTETracker(args, frame_rate=self.frame_rate)
                tracker.det_thresh = self.new_track_threshold
                self.trackers[label] = tracker

            self.confirmed_ids.clear()
            self.active_id = None
            self.manual_id = None
            self.lock_started_at = None

    def set_display_threshold(self, value):
        with self._lock:
            self.display_threshold = max(
                self.high_threshold,
                min(0.90, float(value)),
            )

    def select_target(self, track_id):
        with self._lock:
            self.manual_id = int(track_id) if track_id is not None else None
            self.active_id = self.manual_id
            self.lock_started_at = None

    @staticmethod
    def _int16_limit(value):
        # Referans OpenCV kodundaki int(hata_x) / int(hata_y) davranışı:
        # kesirli piksel değeri yuvarlanmaz, sıfıra doğru kesilir.
        return max(-32768, min(32767, int(value)))

    def _send_packet(self, header, hata_x, hata_y):
        if self.ser is None or not self.ser.is_open:
            return

        hata_x = self._int16_limit(hata_x)
        hata_y = self._int16_limit(hata_y)
        packet = struct.pack("<Bhh", int(header), hata_x, hata_y)

        try:
            self.ser.write(packet)
        except (OSError, serial.SerialException) as error:
            print(f"UART gönderme hatası: {error}")

    def koordinat_gonder(self, hata_x, hata_y):
        self._send_packet(PACKET_TRACKING, hata_x, hata_y)

    def hedefe_varildi_gonder(self, hata_x, hata_y):
        self._send_packet(PACKET_LOCKED, hata_x, hata_y)

    def _read_serial_message(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting

            if waiting <= 0:
                return

            message = self.ser.read(waiting).decode(
                "utf-8",
                errors="ignore",
            ).strip()

            if message:
                print(f"STM32: {message}")

        except (OSError, serial.SerialException) as error:
            print(f"UART okuma hatası: {error}")

    def _send_active_target(self, targets):
        active = next(
            (
                target
                for target in targets
                if target.track_id == self.active_id
            ),
            None,
        )

        if active is None:
            # Hedef yok. 0xFF başlığı, merkezde kilitli bir hedef olmadığını
            # 0xFE paketinden ayırır.
            self.koordinat_gonder(0, 0)
            self._read_serial_message()
            return

        hata_x = self._int16_limit(active.dx_px)
        hata_y = self._int16_limit(active.dy_px)

        kilit_x = abs(hata_x) <= self.lock_tolerance_px
        kilit_y = abs(hata_y) <= self.lock_tolerance_px
        tam_kilit = kilit_x and kilit_y

        if tam_kilit:
            self.hedefe_varildi_gonder(
                -hata_x if UART_INVERT_X else hata_x,
                hata_y,
            )
        else:
            self.koordinat_gonder(
                -hata_x if UART_INVERT_X else hata_x,
                hata_y,
            )

        self._read_serial_message()

    def process(self, roi, width, height, frame=None):
        """Bir Hailo ROI'sini işler ve aktif hedefi UART ile gönderir.

        ``frame`` verilirse görüntü x eksenine göre yerinde aynalanır. Bu
        durumda algılama kutularının Y koordinatları da çevrilerek görüntüyle
        aynı konumda tutulur. Hailo/GStreamer hattında görüntü zaten
        ``videoflip method=vertical-flip`` ile çevriliyorsa ``frame``
        verilmemelidir; aksi hâlde görüntü iki kez çevrilir.
        """

        with self._lock:
            mirror_boxes = frame is not None

            if frame is not None:
                mirrored_frame = mirror_frame_x_axis(frame)

                if mirrored_frame.shape != frame.shape:
                    raise ValueError("Aynalanan kare boyutu değişti")

                # Çağıran taraftaki aynı NumPy dizisini günceller.
                frame[...] = mirrored_frame

            raw_detections = list(
                roi.get_objects_typed(hailo.HAILO_DETECTION)
            )

            for detection in raw_detections:
                roi.remove_object(detection)

            grouped = {label: [] for label in self.class_labels}

            for detection in raw_detections:
                label = detection.get_label().upper()

                if label not in grouped:
                    continue

                confidence = float(detection.get_confidence())
                box = _detection_box(
                    detection,
                    mirror_x_axis=mirror_boxes,
                )

                if box is None or confidence < self.low_threshold:
                    continue

                grouped[label].append((detection, box, confidence))

            for label in self.class_labels:
                grouped[label] = _deduplicate_candidates(grouped[label])

            raw_count = sum(
                len(candidates) for candidates in grouped.values()
            )

            targets = []

            for label in self.class_labels:
                candidates = grouped[label]

                if candidates:
                    tracker_input = np.asarray(
                        [
                            [box[0], box[1], box[2], box[3], confidence]
                            for _detection, box, confidence in candidates
                        ],
                        dtype=np.float64,
                    ).reshape(-1, 5)
                else:
                    tracker_input = np.empty((0, 5), dtype=np.float64)

                online_tracks = self.trackers[label].update(tracker_input)

                visible_candidates = [
                    candidate
                    for candidate in candidates
                    if candidate[2] >= self.display_threshold
                ]
                visible_tracks = [
                    track
                    for track in online_tracks
                    if float(track.score) >= self.display_threshold
                ]

                for track, candidate in _greedy_matches(
                    visible_tracks,
                    visible_candidates,
                    self.display_match_iou,
                ):
                    track_id = int(track.track_id)
                    confirmed = track_id in self.confirmed_ids

                    if (
                        not confirmed
                        and track.tracklet_len + 1 >= self.min_confirmed_hits
                    ):
                        self.confirmed_ids.add(track_id)
                        confirmed = True

                    if not confirmed:
                        continue

                    targets.append(
                        _target_telemetry(
                            track_id,
                            label,
                            candidate[2],
                            candidate[1],
                            width,
                            height,
                        )
                    )

            targets.sort(key=lambda target: target.track_id)
            self._select_active_target(targets)
            state, lock_seconds = self._lock_state(targets, raw_count)

            # Her işlenen karede yalnızca aktif hedefin hata değerleri gönderilir.
            self._send_active_target(targets)

            return TrackingResult(
                targets=tuple(targets),
                active_id=self.active_id,
                state=state,
                lock_seconds=lock_seconds,
                raw_count=raw_count,
            )

    def _select_active_target(self, targets):
        visible_ids = {target.track_id for target in targets}

        if self.manual_id is not None:
            if self.manual_id in visible_ids:
                self.active_id = self.manual_id
                return

            self.manual_id = None

        if self.active_id in visible_ids:
            return

        previous_id = self.active_id
        self.active_id = (
            min(targets, key=lambda target: target.error_norm).track_id
            if targets
            else None
        )

        if self.active_id != previous_id:
            self.lock_started_at = None

    def _lock_state(self, targets, raw_count):
        active = next(
            (
                target
                for target in targets
                if target.track_id == self.active_id
            ),
            None,
        )

        if active is None:
            self.lock_started_at = None
            return ("ACQUIRING" if raw_count else "SEARCHING"), 0.0

        centered = (
            abs(active.dx_px) <= self.lock_tolerance_px
            and abs(active.dy_px) <= self.lock_tolerance_px
        )

        if not centered:
            self.lock_started_at = None
            return "TRACKING", 0.0

        now = monotonic()

        if self.lock_started_at is None:
            self.lock_started_at = now

        return "LOCKED", now - self.lock_started_at

    def close(self):
        with self._lock:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                print("Seri port kapatıldı.")

            self.ser = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False
