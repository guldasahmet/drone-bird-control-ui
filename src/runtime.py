"""Low-latency Hailo/GStreamer runtime used by the GTK control interface."""

from collections import deque
from dataclasses import dataclass, replace
import json
from pathlib import Path
from threading import Event, Lock, RLock, Thread
from time import monotonic, perf_counter

import gi
import hailo

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst

from hailo_platform import HEF
from picamera2 import Picamera2

from hailo_apps.python.core.common.core import get_resource_path
from hailo_apps.python.core.common.defines import (
    DETECTION_PIPELINE,
    DETECTION_POSTPROCESS_FUNCTION,
    DETECTION_POSTPROCESS_SO_FILENAME,
    HAILO_RGB_VIDEO_FORMAT,
    RESOURCES_SO_DIR_NAME,
)
from hailo_apps.python.core.gstreamer.gstreamer_common import disable_qos
from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    QUEUE,
    SOURCE_PIPELINE,
    get_camera_resolution,
)

from settings import AppSettings
from tracking import ClassAwareByteTracker, TargetTelemetry, TrackingResult
from uart import TargetUart

PROJECT_ROOT = Path(__file__).resolve().parents[1]
NATIVE_PLUGIN_DIR = PROJECT_ROOT / "native" / "build"


@dataclass(frozen=True)
class ModelInfo:
    path: str
    network: str
    input_shape: tuple[int, ...]
    output_shape: tuple[int, ...]
    classes: int


@dataclass(frozen=True)
class RuntimeConfig:
    settings: AppSettings
    model_path: str
    source: str = "camera"
    video_path: str | None = None
    display_threshold: float = 0.30
    exposure_us: int | None = None
    analogue_gain: float = 1.0
    terminal_log_every: int = 10
    uart_enabled: bool | None = None

    @property
    def width(self):
        return self.settings.video.width

    @property
    def height(self):
        return self.settings.video.height

    @property
    def frame_rate(self):
        return self.settings.video.camera_fps if self.source == "camera" else 30

    @property
    def labels_path(self):
        return str(self.settings.model.labels)


@dataclass(frozen=True)
class RuntimeSnapshot:
    status: str = "STOPPED"
    message: str = "Ready"
    frame_index: int = 0
    fps: float = 0.0
    dropped: int = 0
    latency_ms: float = 0.0
    tracking_ms: float = 0.0
    targets: tuple[TargetTelemetry, ...] = ()
    active_id: int | None = None
    tracking_state: str = "SEARCHING"
    lock_seconds: float = 0.0


class SnapshotStore:
    """Small thread-safe telemetry store; it never retains video frames."""

    def __init__(self):
        self._lock = Lock()
        self._snapshot = RuntimeSnapshot()

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def set_status(self, status, message):
        with self._lock:
            self._snapshot = replace(self._snapshot, status=status, message=message)

    def publish(self, snapshot):
        with self._lock:
            self._snapshot = snapshot

    def reset(self, status="STOPPED", message="Ready"):
        with self._lock:
            self._snapshot = RuntimeSnapshot(status=status, message=message)


class VisionRuntime:
    """Own one Hailo pipeline and expose control-safe telemetry snapshots."""

    def __init__(self):
        Gst.init(None)
        Gst.Registry.get().scan_path(str(NATIVE_PLUGIN_DIR))
        if Gst.ElementFactory.find("bdtargetoverlay") is None:
            raise RuntimeError(
                "Native overlay bulunamadı; önce native/build.sh çalıştırın"
            )
        self.store = SnapshotStore()
        self.config = None
        self.model_info = None
        self.tracker = None
        self.uart = None
        self.pipeline = None
        self.video_sink = None
        self.bus = None
        self.camera_thread = None
        self.camera_stop = Event()
        self.camera_push = Event()
        self.widget_handler = None
        self.video_widget = None
        self.frame_index = 0
        self.frame_times = deque(maxlen=120)
        self.queue_drops = 0
        self._drop_lock = Lock()
        # start() may need to call stop() while unwinding a partially built
        # pipeline, so this lock must be re-entrant.
        self._lifecycle_lock = RLock()

    @staticmethod
    def validate_model(path, expected_classes=2):
        model_path = Path(path).expanduser().resolve()
        if model_path.suffix.lower() != ".hef" or not model_path.is_file():
            raise ValueError(f"Valid bir .hef dosyası seçin: {model_path}")
        hef = HEF(str(model_path))
        inputs = hef.get_input_vstream_infos()
        outputs = hef.get_output_vstream_infos()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Arayüz tek giriş ve tek NMS çıkışlı HEF bekliyor")
        input_shape = tuple(inputs[0].shape)
        output_shape = tuple(outputs[0].shape)
        if len(output_shape) != 3 or output_shape[1] != 5:
            raise ValueError(f"Çıkış Hailo NMS biçiminde değil: {output_shape}")
        classes = int(output_shape[0])
        if classes != int(expected_classes):
            raise ValueError(
                f"DRONE/BIRD modeli {expected_classes} sınıflı olmalı; "
                f"seçilen model={classes} sınıf"
            )
        return ModelInfo(
            path=str(model_path),
            network=hef.get_network_group_names()[0],
            input_shape=input_shape,
            output_shape=output_shape,
            classes=classes,
        )

    @staticmethod
    def validate_labels(path, expected_labels):
        labels_path = Path(path).expanduser().resolve()
        if not labels_path.is_file():
            raise FileNotFoundError(f"Labels JSON bulunamadı: {labels_path}")
        with labels_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        labels = tuple(str(label).upper() for label in payload.get("labels", []))
        if labels != tuple(expected_labels):
            raise ValueError(
                f"Labels sırası {tuple(expected_labels)} olmalı; bulunan={labels}"
            )
        return labels_path

    def set_widget_handler(self, handler):
        self.widget_handler = handler

    def set_display_threshold(self, value):
        if self.tracker is not None:
            self.tracker.set_display_threshold(value)

    def select_target(self, track_id):
        if self.tracker is not None:
            self.tracker.select_target(track_id)

    def start(self, config):
        with self._lifecycle_lock:
            if self.pipeline is not None:
                raise RuntimeError("Pipeline zaten çalışıyor")
            self.store.set_status("STARTING", "Model ve pipeline hazırlanıyor")
            self.config = config
            settings = config.settings
            self.model_info = self.validate_model(
                config.model_path,
                settings.model.expected_classes,
            )
            self.validate_labels(config.labels_path, settings.tracking.classes)
            if config.source == "video" and not Path(config.video_path or "").is_file():
                raise FileNotFoundError(f"Video bulunamadı: {config.video_path}")

            self.tracker = ClassAwareByteTracker(
                frame_rate=config.frame_rate,
                low_threshold=settings.tracking.low_threshold,
                high_threshold=settings.tracking.high_threshold,
                new_track_threshold=settings.tracking.new_track_threshold,
                display_threshold=config.display_threshold,
                match_threshold=settings.tracking.match_threshold,
                track_buffer=settings.tracking.track_buffer,
                min_confirmed_hits=settings.tracking.min_confirmed_hits,
                display_match_iou=settings.tracking.display_match_iou,
                lock_tolerance_px=settings.tracking.lock_tolerance_px,
                class_labels=settings.tracking.classes,
                priority_labels=settings.tracking.priority,
                sticky_labels=settings.tracking.sticky_labels,
            )
            uart_enabled = (
                settings.uart.enabled
                if config.uart_enabled is None
                else config.uart_enabled
            )
            self.uart = TargetUart(
                enabled=uart_enabled,
                port=settings.uart.port,
                baudrate=settings.uart.baudrate,
                invert_x=settings.uart.invert_x,
            )
            self.uart.open()
            self.frame_index = 0
            self.frame_times.clear()
            with self._drop_lock:
                self.queue_drops = 0
            self.pipeline = Gst.parse_launch(self._pipeline_string(config))
            self._configure_low_latency_elements()
            self.video_sink = self.pipeline.get_by_name("ui_video_sink")
            callback = self.pipeline.get_by_name("identity_callback")
            if self.video_sink is None or callback is None:
                self.stop()
                raise RuntimeError("Pipeline UI elemanları oluşturulamadı")

            callback.set_property("signal-handoffs", True)
            callback.connect("handoff", self._on_frame)
            self.video_widget = self.video_sink.get_property("widget")
            if self.widget_handler is not None:
                self.widget_handler(self.video_widget)

            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect("message", self._on_bus_message)
            disable_qos(self.pipeline)
            self.pipeline.set_latency(50 * Gst.MSECOND)
            self.camera_stop.clear()
            self.camera_push.clear()
            if config.source == "camera":
                self.camera_thread = Thread(
                    target=self._camera_worker,
                    name="imx296-capture",
                    daemon=True,
                )
                self.camera_thread.start()
            result = self.pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                self.stop()
                raise RuntimeError("GStreamer PLAYING durumuna geçemedi")
            if config.source == "camera":
                # The camera may configure in parallel, but it must not push
                # startup frames until hailonet has accepted PLAYING.
                self.camera_push.set()
            self.store.set_status("RUNNING", "Inference aktif")

    def _configure_low_latency_elements(self):
        """Bound linear-stage buffering while preserving crop/agg pairing."""
        one_frame_queues = (
            "source_queue_decode",
            "source_scale_q",
            "source_convert_q",
            "inference_wrapper_input_q",
            "inference_scale_q",
            "inference_convert_q",
            "inference_hailonet_q",
            "inference_hailofilter_q",
            "inference_output_q",
            "inference_wrapper_output_q",
        )
        for name in one_frame_queues:
            queue = self.pipeline.get_by_name(name)
            if queue is not None:
                queue.set_property("max-size-buffers", 1)
                queue.set_property("max-size-bytes", 0)
                queue.set_property("max-size-time", 0)
                if name in (
                    "source_scale_q",
                    "source_convert_q",
                    "inference_wrapper_input_q",
                ):
                    # Drop the oldest not-yet-inferred frame, never the newest
                    # camera observation used by pan/tilt control.
                    queue.set_property("leaky", 2)  # downstream / drop oldest
                    # Count at one choke point only; summing overrun signals
                    # from consecutive leaky queues would double-count a
                    # single stale-frame discard.
                    if name == "inference_wrapper_input_q":
                        queue.connect("overrun", self._on_queue_overrun)
        appsrc = self.pipeline.get_by_name("app_source")
        if appsrc is not None:
            appsrc.set_property("max-buffers", 1)

    def _on_queue_overrun(self, _queue):
        with self._drop_lock:
            self.queue_drops += 1

    def stop(self, final_status="STOPPED", final_message="Ready"):
        with self._lifecycle_lock:
            pipeline = self.pipeline
            if pipeline is None:
                if self.uart is not None:
                    self.uart.close()
                    self.uart = None
                self.tracker = None
                self.store.reset(final_status, final_message)
                return
            self.store.set_status("STOPPING", "Pipeline durduruluyor")
            self.camera_stop.set()
            self.camera_push.set()
            if self.camera_thread is not None and self.camera_thread.is_alive():
                self.camera_thread.join(timeout=3.0)
            # The producer must release its wrapped camera buffer before the
            # GL sink tears down display memory.
            pipeline.set_state(Gst.State.NULL)
            pipeline.get_state(3 * Gst.SECOND)
            if self.bus is not None:
                self.bus.remove_signal_watch()
            if self.widget_handler is not None:
                self.widget_handler(None)
            self.camera_thread = None
            self.bus = None
            self.video_sink = None
            self.video_widget = None
            self.pipeline = None
            self.tracker = None
            if self.uart is not None:
                self.uart.close()
                self.uart = None
            self.store.reset(final_status, final_message)

    def _pipeline_string(self, config):
        source_value = "rpi" if config.source == "camera" else config.video_path
        sync = "false" if config.source == "camera" else "true"
        source = SOURCE_PIPELINE(
            video_source=source_value,
            video_width=config.width,
            video_height=config.height,
            frame_rate=config.frame_rate,
            sync=sync,
            video_format=HAILO_RGB_VIDEO_FORMAT,
        )
        postprocess = get_resource_path(
            DETECTION_PIPELINE,
            RESOURCES_SO_DIR_NAME,
            "hailo8",
            DETECTION_POSTPROCESS_SO_FILENAME,
        )
        if postprocess is None or not Path(postprocess).is_file():
            raise FileNotFoundError(f"Hailo post-process bulunamadı: {postprocess}")
        thresholds = (
            f"nms-score-threshold={config.settings.tracking.low_threshold:.2f} "
            "nms-iou-threshold=0.45 "
            "output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )
        inference = INFERENCE_PIPELINE(
            hef_path=self.model_info.path,
            post_process_so=postprocess,
            post_function_name=DETECTION_POSTPROCESS_FUNCTION,
            batch_size=1,
            config_json=config.labels_path,
            additional_params=thresholds,
        )
        wrapper = INFERENCE_PIPELINE_WRAPPER(inference, bypass_max_size_buffers=2)
        # File decoders otherwise run ahead of presentation because the live
        # low-latency queues below are intentionally leaky. Pace timestamped
        # media before inference; never clock-throttle the live camera path.
        source_clock = (
            " ! identity name=ui_file_clock sync=true"
            if config.source == "video"
            else ""
        )
        # Mirror across the X axis before inference (top <-> bottom). Since
        # inference sees the mirrored pixels, boxes, the active-target line
        # and signed Y error all share the displayed coordinate system.
        x_axis_mirror = ""
        if config.settings.video.mirror_x_axis:
            x_axis_mirror = (
                " ! videoflip name=ui_x_axis_mirror "
                "video-direction=vert qos=false"
            )
        # Never let stale frames queue up behind inference or presentation.
        # This is essential for pan/tilt control: a fresh frame is preferable
        # to displaying an old frame later.
        output_leaky = "downstream"
        callback = (
            f"{QUEUE(name='ui_callback_q', max_size_buffers=1, leaky=output_leaky)} ! "
            "identity name=identity_callback"
        )
        display = (
            "bdtargetoverlay name=ui_aim_overlay "
            f"lock-tolerance={config.settings.tracking.lock_tolerance_px} ! "
            f"{QUEUE(name='ui_hailo_overlay_q', max_size_buffers=1, leaky=output_leaky)} ! "
            "hailooverlay name=target_overlay line-thickness=1 "
            "show-confidence=false qos=false ! "
            f"{QUEUE(name='ui_display_q', max_size_buffers=1, leaky=output_leaky)} ! "
            f"gtkwaylandsink name=ui_video_sink sync={sync} qos=false "
            "enable-last-sample=false"
        )
        return (
            f"{source}{source_clock}{x_axis_mirror} ! "
            f"{wrapper} ! {callback} ! {display}"
        )

    def _on_frame(self, element, buffer):
        started = perf_counter()
        pad = element.get_static_pad("src")
        caps = pad.get_current_caps()
        if caps is None:
            return
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        roi = hailo.get_roi_from_buffer(buffer)
        tracking = self.tracker.process(roi, width, height)
        self._add_overlay_objects(roi, width, height, tracking)

        active = next(
            (
                target
                for target in tracking.targets
                if target.track_id == tracking.active_id
            ),
            None,
        )
        wire_x = 0
        wire_y = 0
        if self.uart is not None:
            if active is None:
                self.uart.send_no_target()
            else:
                wire_x, wire_y = self.uart.send_target(
                    active.dx_px,
                    active.dy_px,
                    locked=tracking.state == "LOCKED",
                )
            self.uart.read_message()

        self.frame_index += 1
        now = monotonic()
        self.frame_times.append(now)
        fps = 0.0
        if len(self.frame_times) > 1:
            duration = self.frame_times[-1] - self.frame_times[0]
            fps = (len(self.frame_times) - 1) / duration if duration > 0 else 0.0
        tracking_ms = (perf_counter() - started) * 1000.0
        latency_ms = self._buffer_latency_ms(buffer)
        dropped = self._sink_dropped_frames()
        snapshot = RuntimeSnapshot(
            status="RUNNING",
            message="Inference aktif",
            frame_index=self.frame_index,
            fps=fps,
            dropped=dropped,
            latency_ms=latency_ms,
            tracking_ms=tracking_ms,
            targets=tracking.targets,
            active_id=tracking.active_id,
            tracking_state=tracking.state,
            lock_seconds=tracking.lock_seconds,
        )
        self.store.publish(snapshot)
        if (
            tracking.active_id is not None
            and self.frame_index % max(1, self.config.terminal_log_every) == 0
        ):
            half_width = width / 2.0
            half_height = height / 2.0
            uart_dx_norm = wire_x / half_width if half_width else 0.0
            uart_dy_norm = wire_y / half_height if half_height else 0.0
            print(
                f"HEDEF frame={self.frame_index} "
                f"{active.label} ID={active.track_id} | "
                f"GORUNTU_DX={active.dx_px:+.1f}px "
                f"GORUNTU_DY={active.dy_px:+.1f}px | "
                f"UART_HATA_X={wire_x:+d}px "
                f"UART_HATA_Y={wire_y:+d}px | "
                f"UART_X_NORM={uart_dx_norm:+.4f} "
                f"UART_Y_NORM={uart_dy_norm:+.4f} | "
                f"TOPLAM_HATA={active.error_px:.1f}px",
                flush=True,
            )

    @staticmethod
    def _add_overlay_objects(roi, width, height, tracking):
        """Build all overlay primitives in Hailo metadata, without frame copies."""
        red_index = 0

        active = None
        for target in tracking.targets:
            bbox = hailo.HailoBBox(
                target.x1,
                target.y1,
                target.x2 - target.x1,
                target.y2 - target.y1,
            )
            # On this RGB pipeline Hailo Tools 5.1 palette index 0 is red.
            # Empty labels keep class/confidence text out of the live video.
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
                        red_index,
                        "",
                        target.confidence,
                    )
                )

        if active is not None:
            # One metadata object is consumed by the native RGB filter. This
            # keeps the vector tied to this exact frame without allocating
            # hundreds of fake detections or copying the frame into Python.
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



    def _buffer_latency_ms(self, buffer):
        if self.pipeline is None or buffer.pts == Gst.CLOCK_TIME_NONE:
            return 0.0
        clock = self.pipeline.get_clock()
        if clock is None:
            return 0.0
        running_time = clock.get_time() - self.pipeline.get_base_time()
        return max(0.0, (running_time - buffer.pts) / Gst.MSECOND)

    def _sink_dropped_frames(self):
        with self._drop_lock:
            queue_drops = self.queue_drops
        if self.video_sink is None:
            return queue_drops
        try:
            stats = self.video_sink.get_property("stats")
            sink_drops = int(stats.get_value("dropped")) if stats else 0
            return queue_drops + sink_drops
        except Exception:
            return queue_drops

    def _on_bus_message(self, _bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            detail = f"{error}: {debug or ''}"
            self.store.set_status("ERROR", detail)
            GLib.idle_add(self.stop, "ERROR", detail)
        elif message.type == Gst.MessageType.EOS:
            self.store.set_status("EOS", "Video tamamlandı")
            GLib.idle_add(self.stop, "EOS", "Video tamamlandı")
        return True

    def _camera_worker(self):
        try:
            appsrc = self.pipeline.get_by_name("app_source")
            appsrc.set_property("is-live", True)
            appsrc.set_property("format", Gst.Format.TIME)
            config = self.config
            with Picamera2() as camera:
                main_width, main_height = get_camera_resolution(
                    config.width, config.height
                )
                controls = {"FrameRate": config.frame_rate}
                if config.exposure_us is not None:
                    controls.update(
                        {
                            "AeEnable": False,
                            "ExposureTime": config.exposure_us,
                            "AnalogueGain": config.analogue_gain,
                        }
                    )
                # Picamera2's BGR888 naming maps to RGB byte order. Feeding it
                # as RGB removes the per-frame cvtColor allocation/copy.
                main = {"size": (main_width, main_height), "format": "BGR888"}
                if (main_width, main_height) == (config.width, config.height):
                    camera_config = camera.create_preview_configuration(
                        main=main, controls=controls
                    )
                    stream = "main"
                else:
                    camera_config = camera.create_preview_configuration(
                        main=main,
                        lores={
                            "size": (config.width, config.height),
                            "format": "BGR888",
                        },
                        controls=controls,
                    )
                    stream = "lores"
                camera.configure(camera_config)
                stream_config = camera_config.get(stream, camera_config["main"])
                width, height = stream_config["size"]
                appsrc.set_property(
                    "caps",
                    Gst.Caps.from_string(
                        f"video/x-raw,format=RGB,width={width},height={height},"
                        f"framerate={config.frame_rate}/1,pixel-aspect-ratio=1/1"
                    ),
                )
                duration = Gst.util_uint64_scale_int(
                    1, Gst.SECOND, config.frame_rate
                )
                camera.start()
                self.camera_push.wait(timeout=10.0)
                if self.camera_stop.is_set():
                    return
                while not self.camera_stop.is_set():
                    frame_data = camera.capture_array(stream)
                    if frame_data is None:
                        break
                    buffer = Gst.Buffer.new_wrapped(frame_data.tobytes())
                    # Timestamp at capture completion using pipeline running
                    # time. This stays truthful if IMX296 cadence varies by a
                    # fraction of a frame and excludes camera/Hailo startup.
                    clock = self.pipeline.get_clock()
                    buffer.pts = (
                        max(0, clock.get_time() - self.pipeline.get_base_time())
                        if clock is not None
                        else Gst.CLOCK_TIME_NONE
                    )
                    buffer.dts = buffer.pts
                    buffer.duration = duration
                    result = appsrc.emit("push-buffer", buffer)
                    if result != Gst.FlowReturn.OK:
                        break
        except Exception as error:
            detail = f"Kamera hatası: {error}"
            self.store.set_status("ERROR", detail)
            GLib.idle_add(self.stop, "ERROR", detail)
