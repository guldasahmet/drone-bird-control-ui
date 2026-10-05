"""Two live cameras, one Hailo, independent target tracks and HQ/GS control."""

import csv
from dataclasses import dataclass
from datetime import datetime
from threading import Event, Lock, Thread
from time import monotonic

import hailo
from hailo_platform import HEF

from dual_camera_pipeline import DualCameraPipeline, HEIGHT, WIDTH
from model_profile import load_active_profile
from tracking import ClassAwareByteTracker
from uart import TargetUart


CAMERA_NAMES = ("GLOBAL SHUTTER", "HQ")


def target_pixel_errors(target, *, flip_vertical=False):
    """Recover camera-native coordinates before reference UART rounding."""
    camera_y = 1.0 - target.center_y if flip_vertical else target.center_y
    return (
        int(target.center_x * WIDTH) - WIDTH // 2,
        int(camera_y * HEIGHT) - HEIGHT // 2,
    )


def limit_step(previous, target, max_step):
    """Move previous toward target by at most max_step (0 disables the limit)."""
    if max_step <= 0:
        return target
    return previous + max(-max_step, min(max_step, target - previous))


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


class TraceLog:
    """Per-frame detections, sent UART commands and STM32 replies as CSV."""

    FIELDS = ("t", "kind", "camera", "control", "candidates", "label",
              "confidence", "track_id", "error_x", "error_y",
              "wire_x", "wire_y", "locked", "stm", "age_ms")

    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.lock = Lock()
        self.started_at = monotonic()
        self.file = open(path, "w", newline="", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.FIELDS)

    def write(self, kind, **values):
        row = [f"{monotonic() - self.started_at:.3f}", kind]
        row += [values.get(name, "") for name in self.FIELDS[2:]]
        with self.lock:
            if self.file is not None:
                self.writer.writerow(row)

    def close(self):
        with self.lock:
            if self.file is not None:
                self.file.close()
                self.file = None


class HandoffController:
    """HQ controls until GS is confirmed; five GS misses return control to HQ.

    With hold_hq, GS is still observed but never takes control.
    """

    def __init__(self, lost_frames=5, max_age_seconds=0.3, gs_confirm_frames=2,
                 hold_hq=False):
        self.lock = Lock()
        self.hold_hq = hold_hq
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
            if slot == 0 and not self.hold_hq:
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
    def __init__(self, widget_handler, *, profile=None, fps=None,
                 uart_enabled=False,
                 uart_port="/dev/ttyACM0", baudrate=115200,
                 invert_x=True, invert_y=True, lock_tolerance=25,
                 display_backend="gtk", hold_hq=False, trace_dir=None,
                 max_step_px=30, queue_depth=1, batch_size=None):
        self.widget_handler = widget_handler
        self.profile = profile or load_active_profile()
        self.model_path = self.profile.hef
        self.postprocess_path = self.profile.postprocess_so
        self.fps = fps if fps is not None else self.profile.fps
        self.queue_depth = max(1, int(queue_depth))
        # Multi-context HEFs amortise context switches over a batch; with two
        # cameras, batch 2 takes one frame from each.
        self.batch_size = (batch_size if batch_size is not None
                           else self.profile.batch_size)
        self.ages = [None, None]
        self.confidence = self.profile.confidence
        self.uart_enabled = uart_enabled
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.lock_tolerance = lock_tolerance
        self.display_backend = display_backend
        self.hold_hq = hold_hq
        self.trace_dir = trace_dir
        self.trace = None
        # Per-packet (50 ms) limit on the commanded error change; softens the
        # STM32 start kick (D term on a reset error) when a target appears.
        self.max_step_px = max(0, int(max_step_px))
        self.command = (0, 0)
        self.target_ids = frozenset(self.profile.labels_by_id)
        self.target_labels = {label.casefold() for label in self.profile.target_names}
        self.pipeline = None
        self.uart = None
        self.output_stop = Event()
        self.output_thread = None
        self.controller = HandoffController(hold_hq=self.hold_hq)
        self.trackers = [self._new_tracker(), self._new_tracker()]
        self.results = [None, None]
        self.metrics = ((0.0, 0.0), 0.0, (0.0, 0.0))
        self.lock = Lock()
        self.status = "STOPPED"
        self.message = "Hazır"
        self.transition = ""
        self.started_at = 0.0
        self.last_recording_status = None

    def _new_tracker(self):
        return ClassAwareByteTracker(
            frame_rate=self.fps,
            class_labels=self.profile.target_names,
            priority_labels=self.profile.priority,
            sticky_labels=self.profile.sticky,
            labels_by_id=(self.profile.labels_by_id
                          if self.profile.match_by == "id" else None),
            low_threshold=self.profile.low_threshold,
            high_threshold=self.profile.high_threshold,
            new_track_threshold=self.profile.new_track_threshold,
            display_threshold=self.confidence,
            min_confirmed_hits=2,
            lock_tolerance_px=self.lock_tolerance,
        )

    def set_confidence(self, value):
        self.confidence = max(self.profile.high_threshold,
                              min(0.90, float(value)))
        for tracker in self.trackers:
            tracker.set_display_threshold(self.confidence)

    def select_target(self, slot, track_id):
        if slot in (0, 1):
            self.trackers[slot].select_target(track_id)

    def _age_text(self, slot):
        age = self.ages[slot]
        return "-" if age is None else f"{age:.0f} ms"

    def _target_fields(self, target):
        if target is None:
            return {}
        error_x, error_y = target_pixel_errors(
            target, flip_vertical=self.profile.camera_flip_vertical
        )
        return {
            "label": target.label,
            "confidence": f"{target.confidence:.2f}",
            "track_id": target.track_id,
            "error_x": error_x,
            "error_y": error_y,
        }

    def _filter(self, buffer):
        roi = hailo.get_roi_from_buffer(buffer)
        for detection in list(roi.get_objects_typed(hailo.HAILO_DETECTION)):
            accepted = (
                detection.get_label().strip().casefold() in self.target_labels
                if self.profile.match_by == "label"
                else detection.get_class_id() in self.target_ids
            )
            if not accepted or detection.get_confidence() < self.confidence:
                roi.remove_object(detection)

    def _detect(self, slot, buffer):
        pipeline = self.pipeline
        age = pipeline.buffer_age_ms(buffer) if pipeline is not None else None
        self.ages[slot] = age
        roi = hailo.get_roi_from_buffer(buffer)
        result = self.trackers[slot].process(roi, WIDTH, HEIGHT)
        add_overlay_objects(roi, WIDTH, HEIGHT, result)
        active = next(
            (item for item in result.targets if item.track_id == result.active_id),
            None,
        )
        transition = self.controller.observe(slot, active)
        trace = self.trace
        if trace is not None:
            trace.write(
                "det", camera=CAMERA_NAMES[slot],
                control=CAMERA_NAMES[self.controller.active_slot],
                candidates=result.raw_count, **self._target_fields(active),
                age_ms="" if age is None else f"{age:.0f}",
            )
        with self.lock:
            self.results[slot] = result
            if transition:
                self.transition = transition
                print(f"KONTROL DEVRİ: {transition}", flush=True)

    def _output_worker(self):
        while not self.output_stop.wait(0.05):
            active_slot, target, _ = self.controller.snapshot()
            wire_x, wire_y, locked = 0, 0, False
            try:
                if target is None:
                    # Stop at once on loss; the next target ramps up from zero.
                    self.command = (0, 0)
                    self.uart.send_no_target()
                else:
                    error_x, error_y = target_pixel_errors(
                        target, flip_vertical=self.profile.camera_flip_vertical
                    )
                    # Match STM32: lock only while both errors are strictly < tolerance.
                    locked = (abs(error_x) < self.lock_tolerance
                              and abs(error_y) < self.lock_tolerance)
                    if locked:
                        # STM32 stops both motors on lock; restart from zero.
                        self.command = (0, 0)
                        command_x, command_y = error_x, error_y
                    else:
                        command_x = limit_step(self.command[0], error_x,
                                               self.max_step_px)
                        command_y = limit_step(self.command[1], error_y,
                                               self.max_step_px)
                        self.command = (command_x, command_y)
                    wire_x, wire_y = self.uart.send_target(
                        command_x, command_y, locked=locked
                    )
                message = self.uart.read_message()
                trace = self.trace
                if trace is not None:
                    trace.write(
                        "tx", control=CAMERA_NAMES[active_slot],
                        wire_x=wire_x, wire_y=wire_y, locked=int(locked),
                        **self._target_fields(target),
                    )
                    if message:
                        trace.write("stm", stm=message)
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
        self.controller = HandoffController(hold_hq=self.hold_hq)
        self.trackers = [self._new_tracker(), self._new_tracker()]
        self.results = [None, None]
        self.metrics = ((0.0, 0.0), 0.0, (0.0, 0.0))
        self.transition = ""
        self.started_at = monotonic()
        self.last_recording_status = None
        self.command = (0, 0)
        self.ages = [None, None]
        self.output_stop.clear()
        try:
            if self.trace_dir is not None:
                self.trace = TraceLog(
                    self.trace_dir / f"trace_{datetime.now():%Y%m%d_%H%M%S}.csv"
                )
                print(f"İz kaydı: {self.trace.path}", flush=True)
            print(f"Kamera FPS={self.fps}, kuyruk derinliği={self.queue_depth}, "
                  f"batch={self.batch_size}", flush=True)
            self.profile.validate_hef(HEF)
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
                post_function=self.profile.postprocess_function,
                post_config_data=(self.profile.postprocess.as_dict()
                                  if self.profile.postprocess else None),
                profile_name=self.profile.name,
                nms_score_threshold=self.profile.nms_score_threshold,
                nms_iou_threshold=self.profile.nms_iou_threshold,
                flip_vertical=self.profile.camera_flip_vertical,
                lock_tolerance=self.lock_tolerance,
                queue_depth=self.queue_depth,
                batch_size=self.batch_size,
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
            result = self.results[slot]
            candidate_count = result.raw_count if result is not None else 0
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
                + f" | gecikme={self._age_text(slot)}"
                + f" | aday={candidate_count} | {target_text}",
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
                else self.uart.wire_errors(*target_pixel_errors(
                    target, flip_vertical=self.profile.camera_flip_vertical
                ))
            )
            wire_text = f" | STM hata=({wire_x:+d},{wire_y:+d})"
        print(
            f"HAILO={self.metrics[1]:.1f} FPS | "
            f"KONTROL={CAMERA_NAMES[active]} | "
            f"UART={'açık' if self.uart_enabled else 'kapalı'}"
            f"{' | HQ SABİT' if self.hold_hq else ''}"
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

    def start_recording(self):
        if self.pipeline is None or self.status != "RUNNING":
            raise RuntimeError("Dataset kaydı için önce iki kamerayı başlatın")
        return self.pipeline.start_recording()

    def stop_recording(self):
        if self.pipeline is not None:
            self.pipeline.stop_recording()

    def recording_status(self):
        if self.pipeline is not None:
            return self.pipeline.recording_status()
        return self.last_recording_status

    def stop(self):
        self.output_stop.set()
        if self.output_thread and self.output_thread.is_alive():
            self.output_thread.join(timeout=1.0)
        self.output_thread = None
        if self.trace is not None:
            self.trace.close()
            self.trace = None
        if self.uart is not None:
            if self.uart.connected:
                self.uart.send_no_target()
            self.uart.close()
            self.uart = None
        if self.pipeline is not None:
            self.pipeline.stop()
            self.last_recording_status = self.pipeline.recording_status()
            self.pipeline = None
        self.status = "STOPPED"
        self.message = "Hazır"
