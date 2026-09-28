"""Two live cameras, one Hailo, independent phone tracks and HQ/GS control."""

from dataclasses import dataclass
import json
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic

import hailo
from hailo_platform import HEF

from dual_camera_pipeline import DualCameraPipeline, HEIGHT, WIDTH
from runtime import VisionRuntime
from tracking import ClassAwareByteTracker
from uart import TargetUart


DEFAULT_HEF = Path("/usr/share/hailo-models/yolov8s_h8.hef")
DEFAULT_POSTPROCESS = Path(
    "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
)
PHONE_LABELS = Path(__file__).resolve().parents[1] / "config" / "phone_labels.json"
CAMERA_NAMES = ("GLOBAL SHUTTER", "HQ")


class HandoffController:
    """Same five-miss and stale-target policy as drone-bird-control."""

    def __init__(self, lost_frames=5, max_age_seconds=0.3):
        self.lock = Lock()
        self.active_slot = 1
        self.lost_frames = lost_frames
        self.max_age_seconds = max_age_seconds
        self.gs_misses = 0
        self.latest = {0: (None, 0.0), 1: (None, 0.0)}

    def observe(self, slot, target, now=None):
        now = monotonic() if now is None else now
        transition = None
        with self.lock:
            self.latest[slot] = (target, now)
            if slot == 0:
                if target is not None:
                    self.gs_misses = 0
                    if self.active_slot != 0:
                        self.active_slot = 0
                        transition = "HQ → GLOBAL SHUTTER"
                elif self.active_slot == 0:
                    self.gs_misses += 1
                    if self.gs_misses >= self.lost_frames:
                        self.active_slot = 1
                        self.gs_misses = 0
                        transition = "GLOBAL SHUTTER → HQ"
        return transition

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


class DualVisionRuntime:
    def __init__(self, widget_handler, *, model_path=DEFAULT_HEF,
                 postprocess_path=DEFAULT_POSTPROCESS, fps=30,
                 confidence=0.15, uart_enabled=False,
                 uart_port="/dev/ttyACM0", baudrate=115200,
                 invert_x=False, lock_tolerance=50):
        self.widget_handler = widget_handler
        self.model_path = Path(model_path)
        self.postprocess_path = Path(postprocess_path)
        self.fps = fps
        self.confidence = confidence
        self.uart_enabled = uart_enabled
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.invert_x = invert_x
        self.lock_tolerance = lock_tolerance
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
        VisionRuntime._add_overlay_objects(roi, WIDTH, HEIGHT, result)
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
                    locked = (abs(target.dx_px) <= self.lock_tolerance
                              and abs(target.dy_px) <= self.lock_tolerance)
                    self.uart.send_target(target.dx_px, target.dy_px,
                                          locked=locked)
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
        self.output_stop.clear()
        try:
            hef = HEF(str(self.model_path.expanduser().resolve()))
            outputs = hef.get_output_vstream_infos()
            if len(outputs) != 1 or int(outputs[0].shape[0]) != 80:
                raise ValueError("Telefon profili 80 sınıflı COCO HEF gerektirir")
            self.uart = TargetUart(
                self.uart_enabled, self.uart_port, self.baudrate,
                invert_x=self.invert_x,
            )
            self.uart.open()
            self.pipeline = DualCameraPipeline(
                fps=self.fps,
                hef_path=self.model_path,
                post_so=self.postprocess_path,
                on_filter=self._filter,
                on_detection=self._detect,
                widget_handler=self.widget_handler,
            )
            self.pipeline.start()
            if self.uart_enabled:
                self.output_thread = Thread(
                    target=self._output_worker, daemon=True,
                    name="stm32-output",
                )
                self.output_thread.start()
            self.status = "RUNNING"
            self.message = "İki kamera ve tek Hailo çalışıyor"
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
        active, target, latest = self.controller.snapshot()
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
                f"pencere={self.metrics[2][slot]:.1f} FPS | {target_text}",
                flush=True,
            )
        print(
            f"HAILO={self.metrics[1]:.1f} FPS | "
            f"KONTROL={CAMERA_NAMES[active]} | "
            f"UART={'açık' if self.uart_enabled else 'kapalı'}",
            flush=True,
        )

    def snapshot(self):
        active, target, _ = self.controller.snapshot()
        with self.lock:
            return DualSnapshot(
                self.status, self.message, active, target,
                tuple(self.results), tuple(self.metrics[0]),
                self.metrics[1], tuple(self.metrics[2]),
                self.uart_enabled, self.transition,
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
