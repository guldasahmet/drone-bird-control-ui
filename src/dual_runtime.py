"""Two live cameras, one Hailo, independent phone tracks and HQ/GS control."""

from dataclasses import dataclass
import json
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic

import hailo
from hailo_platform import HEF

from dual_camera_pipeline import DualCameraPipeline, HEIGHT, WIDTH
from tracking import ClassAwareByteTracker
from uart import TargetUart


DEFAULT_HEF = Path("/usr/share/hailo-models/yolov8s_h8.hef")
DEFAULT_POSTPROCESS = Path(
    "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
)
PHONE_LABELS = Path(__file__).resolve().parents[1] / "config" / "phone_labels.json"
CAMERA_NAMES = ("GLOBAL SHUTTER", "HQ")


def target_pixel_errors(target):
    """Use the same pixel-center rounding as drone-bird-control and denme."""
    return (
        int(target.center_x * WIDTH) - WIDTH // 2,
        int(target.center_y * HEIGHT) - HEIGHT // 2,
    )


def add_overlay_objects(roi, width, height, tracking):
    """Keep target boxes and the active target vector with their Hailo frame."""
    red_index = 0
    active = None
    for target in tracking.targets:
        bbox = hailo.HailoBBox(
            target.x1,
            target.y1,
            target.x2 - target.x1,
            target.y2 - target.y1,
        )
        roi.add_object(
            hailo.HailoDetection(bbox, red_index, "", target.confidence)
        )
        if target.track_id == tracking.active_id:
            active = target
            pad_x = 1.0 / width
            pad_y = 1.0 / height
            x1 = max(0.0, target.x1 - pad_x)
            y1 = max(0.0, target.y1 - pad_y)
            x2 = min(1.0, target.x2 + pad_x)
            y2 = min(1.0, target.y2 + pad_y)
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(x1, y1, x2 - x1, y2 - y1),
                    red_index, "", target.confidence,
                )
            )

    if active is not None:
        roi.add_object(
            hailo.HailoLandmarks(
                "active_target_aim",
                [
                    hailo.HailoPoint(0.5, 0.5, 1.0),
                    hailo.HailoPoint(
                        active.center_x, active.center_y, active.confidence
                    ),
                ],
                0.0,
                [(0, 1)],
            )
        )


class HandoffController:
    """HQ controls until GS is confirmed; five GS misses return control to HQ."""

    def __init__(self, lost_frames=5, max_age_seconds=0.3, gs_confirm_frames=2):
        self.lock = Lock()
        self.active_slot = 1
        self.lost_frames = lost_frames
        self.max_age_seconds = max_age_seconds
        self.gs_confirm_frames = gs_confirm_frames
        self.gs_confirm_count = 0
        self.gs_candidate_id = None
        self.gs_misses = 0
        self.latest = {0: (None, 0.0), 1: (None, 0.0)}

    def observe(self, slot, target, now=None):
        now = monotonic() if now is None else now
        transition = None
        with self.lock:
            previous_gs_seen_at = self.latest[0][1]
            self.latest[slot] = (target, now)
            if slot == 0:
                if target is not None:
                    self.gs_misses = 0
                    if self.active_slot != 0:
                        candidate_id = getattr(target, "track_id", None)
                        if (now - previous_gs_seen_at > self.max_age_seconds
                                or candidate_id != self.gs_candidate_id):
                            self.gs_confirm_count = 0
                        self.gs_candidate_id = candidate_id
                        self.gs_confirm_count += 1
                        if self.gs_confirm_count >= self.gs_confirm_frames:
                            self.active_slot = 0
                            self.gs_confirm_count = 0
                            self.gs_candidate_id = None
                            transition = "HQ → GLOBAL SHUTTER"
                else:
                    self.gs_confirm_count = 0
                    self.gs_candidate_id = None
                    if self.active_slot == 0:
                        self.gs_misses += 1
                        if self.gs_misses >= self.lost_frames:
                            self.active_slot = 1
                            self.gs_misses = 0
                            transition = "GLOBAL SHUTTER → HQ"
        return transition

    def confirmation_progress(self, now=None):
        now = monotonic() if now is None else now
        with self.lock:
            if now - self.latest[0][1] > self.max_age_seconds:
                return 0, self.gs_confirm_frames
            return self.gs_confirm_count, self.gs_confirm_frames

    def snapshot(self, now=None):
        now = monotonic() if now is None else now
        with self.lock:
            active = self.active_slot
            latest = self.latest.copy()
        target, seen_at = latest[active]
        if now - seen_at > self.max_age_seconds:
            target = None
        return active, target, latest


@dataclass(frozen=True)
class DualSnapshot:
    status: str
    message: str
    active_slot: int
    active_target: object | None
    results: tuple
    camera_fps: tuple[float, float]
    infer_fps: float
    display_fps: tuple[float, float]
    uart_enabled: bool
    transition: str
    gs_confirm_count: int
    gs_confirm_frames: int
    branch_fps: tuple[float, float]


class DualVisionRuntime:
    def __init__(self, widget_handler, *, model_path=DEFAULT_HEF,
                 postprocess_path=DEFAULT_POSTPROCESS, fps=30,
                 confidence=0.15, uart_enabled=False,
                 uart_port="/dev/ttyACM0", baudrate=115200,
                 invert_x=True, invert_y=True, lock_tolerance=50,
                 display_backend="gtk"):
        self.widget_handler = widget_handler
        self.model_path = Path(model_path)
        self.postprocess_path = Path(postprocess_path)
        self.fps = fps
        self.confidence = confidence
        self.uart_enabled = uart_enabled
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.lock_tolerance = lock_tolerance
        self.display_backend = display_backend
        with PHONE_LABELS.open(encoding="utf-8") as handle:
            labels = json.load(handle).get("target_labels", [])
        self.target_labels = {str(label).strip().casefold() for label in labels}
        if self.target_labels != {"cell phone"}:
            raise ValueError("phone_labels.json hedefi cell phone olmalı")
        self.pipeline = None
        self.uart = None
        self.output_stop = Event()
        self.output_thread = None
        self.controller = HandoffController()
        self.trackers = [self._new_tracker(), self._new_tracker()]
        self.results = [None, None]
        self.metrics = ((0.0, 0.0), 0.0, (0.0, 0.0))
        self.lock = Lock()
        self.status = "STOPPED"
        self.message = "Hazır"
        self.transition = ""
        self.started_at = 0.0

    def _new_tracker(self):
        return ClassAwareByteTracker(
            frame_rate=self.fps,
            class_labels=("CELL PHONE",),
            priority_labels=("CELL PHONE",),
            sticky_labels=("CELL PHONE",),
            low_threshold=0.10,
            high_threshold=0.15,
            new_track_threshold=0.15,
            display_threshold=self.confidence,
            min_confirmed_hits=2,
            lock_tolerance_px=self.lock_tolerance,
        )

    def set_confidence(self, value):
        self.confidence = max(0.15, min(0.90, float(value)))
        for tracker in self.trackers:
            tracker.set_display_threshold(self.confidence)

    def select_target(self, slot, track_id):
        if slot in (0, 1):
            self.trackers[slot].select_target(track_id)

    def _filter(self, buffer):
        roi = hailo.get_roi_from_buffer(buffer)
        for detection in list(roi.get_objects_typed(hailo.HAILO_DETECTION)):
            if (detection.get_label().strip().casefold() not in self.target_labels
                    or detection.get_confidence() < self.confidence):
                roi.remove_object(detection)

    def _detect(self, slot, buffer):
        roi = hailo.get_roi_from_buffer(buffer)
        result = self.trackers[slot].process(roi, WIDTH, HEIGHT)
        add_overlay_objects(roi, WIDTH, HEIGHT, result)
        active = next(
            (item for item in result.targets if item.track_id == result.active_id),
            None,
        )
        transition = self.controller.observe(slot, active)
        with self.lock:
            self.results[slot] = result
            if transition:
                self.transition = transition
                print(f"KONTROL DEVRİ: {transition}", flush=True)

    def _output_worker(self):
        while not self.output_stop.wait(0.05):
            _, target, _ = self.controller.snapshot()
            try:
                if target is None:
                    self.uart.send_no_target()
                else:
                    error_x, error_y = target_pixel_errors(target)
                    locked = (abs(error_x) <= self.lock_tolerance
                              and abs(error_y) <= self.lock_tolerance)
                    self.uart.send_target(error_x, error_y, locked=locked)
                self.uart.read_message()
            except Exception as error:
                with self.lock:
                    self.message = f"UART hatası: {error}"
                    self.status = "ERROR"
                self.output_stop.set()

    def start(self):
        if self.pipeline is not None:
            return
        self.status = "STARTING"
        self.message = "Kameralar ve Hailo hazırlanıyor"
        self.controller = HandoffController()
        self.trackers = [self._new_tracker(), self._new_tracker()]
        self.results = [None, None]
        self.metrics = ((0.0, 0.0), 0.0, (0.0, 0.0))
        self.transition = ""
        self.started_at = monotonic()
        self.output_stop.clear()
        try:
            hef = HEF(str(self.model_path.expanduser().resolve()))
            outputs = hef.get_output_vstream_infos()
            if len(outputs) != 1 or int(outputs[0].shape[0]) != 80:
                raise ValueError("Telefon profili 80 sınıflı COCO HEF gerektirir")
            self.uart = TargetUart(
                self.uart_enabled, self.uart_port, self.baudrate,
                invert_x=self.invert_x,
                invert_y=self.invert_y,
            )
            self.uart.open()
            self.pipeline = DualCameraPipeline(
                fps=self.fps,
                hef_path=self.model_path,
                post_so=self.postprocess_path,
                on_filter=self._filter,
                on_detection=self._detect,
                widget_handler=self.widget_handler,
                display_backend=self.display_backend,
            )
            self.pipeline.start()
            if self.uart_enabled:
                self.output_thread = Thread(
                    target=self._output_worker, daemon=True,
                    name="stm32-output",
                )
                self.output_thread.start()
            self.status = "STARTING"
            self.message = "İki kameranın ilk görüntüsü bekleniyor"
        except Exception:
            self.stop()
            raise

    def update_metrics(self):
        if self.pipeline is None:
            return
        if self.pipeline.error:
            with self.lock:
                self.status = "ERROR"
                self.message = self.pipeline.error
            return
        self.metrics = self.pipeline.fps_snapshot()
        if self.status == "STARTING":
            if self.pipeline.first_frames_ready():
                self.status = "RUNNING"
                self.message = "İki kameranın görüntüsü arayüze ulaştı"
            elif monotonic() - self.started_at > 8.0:
                levels, sink_levels, branches = self.pipeline.frame_diagnostics()
                self.status = "ERROR"
                self.message = (
                    "Görüntü başlatılamadı: "
                    f"CAM0 hat={branches[0]:.0f} FPS, ekran={self.metrics[2][0]:.0f} FPS; "
                    f"CAM1 hat={branches[1]:.0f} FPS, ekran={self.metrics[2][1]:.0f} FPS; "
                    f"kamera parlaklığı={levels[0]:.1f}/{levels[1]:.1f}; "
                    f"sink parlaklığı={sink_levels[0]:.1f}/{sink_levels[1]:.1f}"
                )
                return
        active, target, latest = self.controller.snapshot()
        gs_confirm_count, gs_confirm_frames = self.controller.confirmation_progress()
        levels, sink_levels, branches = self.pipeline.frame_diagnostics()
        for slot in (0, 1):
            item, seen_at = latest[slot]
            target_text = (
                "hedef yok" if item is None or monotonic() - seen_at > 0.3
                else f"{item.label} %{item.confidence * 100:.0f} "
                     f"hata=({item.dx_px:+.0f},{item.dy_px:+.0f})"
            )
            print(
                f"[src_{slot}] {CAMERA_NAMES[slot]} | "
                f"kamera={self.metrics[0][slot]:.1f} FPS | "
                f"hat={branches[slot]:.1f} FPS | "
                f"pencere={self.metrics[2][slot]:.1f} FPS | "
                f"parlaklık={levels[slot]:.1f}"
                + (f"→{sink_levels[slot]:.1f}" if self.display_backend == "gtk"
                   else "")
                + f" | {target_text}",
                flush=True,
            )
        control_text = (
            f" | GS doğrulama={gs_confirm_count}/{gs_confirm_frames}"
            if active == 1 and gs_confirm_count else ""
        )
        wire_text = ""
        if self.uart_enabled and self.uart is not None:
            wire_x, wire_y = (
                (0, 0) if target is None
                else self.uart.wire_errors(*target_pixel_errors(target))
            )
            wire_text = f" | STM hata=({wire_x:+d},{wire_y:+d})"
        print(
            f"HAILO={self.metrics[1]:.1f} FPS | "
            f"KONTROL={CAMERA_NAMES[active]} | "
            f"UART={'açık' if self.uart_enabled else 'kapalı'}"
            f"{control_text}{wire_text}",
            flush=True,
        )

    def snapshot(self):
        active, target, _ = self.controller.snapshot()
        gs_confirm_count, gs_confirm_frames = self.controller.confirmation_progress()
        with self.lock:
            return DualSnapshot(
                self.status, self.message, active, target,
                tuple(self.results), tuple(self.metrics[0]),
                self.metrics[1], tuple(self.metrics[2]),
                self.uart_enabled, self.transition,
                gs_confirm_count, gs_confirm_frames,
                tuple(self.pipeline.branch_fps) if self.pipeline else (0.0, 0.0),
            )

    def stop(self):
        self.output_stop.set()
        if self.output_thread and self.output_thread.is_alive():
            self.output_thread.join(timeout=1.0)
        self.output_thread = None
        if self.uart is not None:
            if self.uart.connected:
                self.uart.send_no_target()
            self.uart.close()
            self.uart = None
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
        self.status = "STOPPED"
        self.message = "Hazır"
