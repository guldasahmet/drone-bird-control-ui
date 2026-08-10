"""Filtered ByteTrack and resolution-independent targeting telemetry."""

from dataclasses import dataclass
from math import hypot
from threading import RLock
from time import monotonic
from types import SimpleNamespace

import hailo
import numpy as np

from hailo_apps.python.core.tracker.basetrack import BaseTrack
from hailo_apps.python.core.tracker.byte_tracker import BYTETracker


CLASS_LABELS = ("CELL PHONE",)


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


def _detection_box(detection):
    bbox = detection.get_bbox()
    x1 = max(0.0, min(1.0, float(bbox.xmin())))
    y1 = max(0.0, min(1.0, float(bbox.ymin())))
    x2 = max(0.0, min(1.0, x1 + float(bbox.width())))
    y2 = max(0.0, min(1.0, y1 + float(bbox.height())))
    if x2 <= x1 or y2 <= y1:
        return None
    return np.asarray([x1, y1, x2, y2], dtype=np.float64)


def _iou(box_a, box_b):
    left = max(float(box_a[0]), float(box_b[0]))
    top = max(float(box_a[1]), float(box_b[1]))
    right = min(float(box_a[2]), float(box_b[2]))
    bottom = min(float(box_a[3]), float(box_b[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(0.0, float(box_a[2] - box_a[0])) * max(
        0.0, float(box_a[3] - box_a[1])
    )
    area_b = max(0.0, float(box_b[2] - box_b[0])) * max(
        0.0, float(box_b[3] - box_b[1])
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
    for score, track_index, candidate_index in possible:
        if track_index in used_tracks or candidate_index in used_candidates:
            continue
        used_tracks.add(track_index)
        used_candidates.add(candidate_index)
        matches.append((tracks[track_index], candidates[candidate_index]))
    return matches


def _deduplicate_candidates(candidates, maximum_iou=0.70):
    """Keep the strongest observation for near-identical same-class boxes."""
    kept = []
    for candidate in sorted(candidates, key=lambda item: item[2], reverse=True):
        if any(_iou(candidate[1], previous[1]) >= maximum_iou for previous in kept):
            continue
        kept.append(candidate)
    return kept


def _target_telemetry(track_id, label, confidence, box, width, height):
    center_x = float((box[0] + box[2]) / 2.0)
    center_y = float((box[1] + box[3]) / 2.0)
    dx_norm = 2.0 * center_x - 1.0
    dy_norm = 2.0 * center_y - 1.0
    dx_px = dx_norm * width / 2.0
    dy_px = dy_norm * height / 2.0
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
    """Track only selected detection labels and expose one active target."""

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
        lock_deadband=0.05,
        class_labels=CLASS_LABELS,
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
        self.lock_deadband = lock_deadband
        self.class_labels = tuple(label.upper() for label in class_labels)
        if not self.class_labels:
            raise ValueError("En az bir takip sınıfı gerekli")
        self.trackers = {}
        self.confirmed_ids = set()
        self.active_id = None
        self.manual_id = None
        self.lock_started_at = None
        self.reset()

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
            self.display_threshold = max(self.high_threshold, min(0.90, float(value)))

    def select_target(self, track_id):
        with self._lock:
            self.manual_id = int(track_id) if track_id is not None else None
            self.active_id = self.manual_id
            self.lock_started_at = None

    def process(self, roi, width, height):
        with self._lock:
            raw_detections = list(roi.get_objects_typed(hailo.HAILO_DETECTION))
            for detection in raw_detections:
                roi.remove_object(detection)

            grouped = {label: [] for label in self.class_labels}
            for detection in raw_detections:
                label = detection.get_label().upper()
                if label not in grouped:
                    continue
                confidence = float(detection.get_confidence())
                box = _detection_box(detection)
                if box is None or confidence < self.low_threshold:
                    continue
                grouped[label].append((detection, box, confidence))

            for label in self.class_labels:
                grouped[label] = _deduplicate_candidates(grouped[label])
            raw_count = sum(len(candidates) for candidates in grouped.values())

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
            (target for target in targets if target.track_id == self.active_id),
            None,
        )
        if active is None:
            self.lock_started_at = None
            return ("ACQUIRING" if raw_count else "SEARCHING"), 0.0

        centered = (
            abs(active.dx_norm) <= self.lock_deadband
            and abs(active.dy_norm) <= self.lock_deadband
        )
        if not centered:
            self.lock_started_at = None
            return "TRACKING", 0.0
        now = monotonic()
        if self.lock_started_at is None:
            self.lock_started_at = now
        return "LOCKED", now - self.lock_started_at
