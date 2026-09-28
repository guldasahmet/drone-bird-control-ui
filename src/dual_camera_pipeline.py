"""Proven two-camera Picamera2 -> one Hailo -> two-window pipeline."""

from pathlib import Path
from collections import deque
from threading import Event, Lock, Thread
from time import monotonic

import gi
import numpy as np
from picamera2 import Picamera2

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst


WIDTH = 640
HEIGHT = 640


class DualCameraPipeline:
    def __init__(self, *, fps, hef_path, post_so, on_filter, on_detection,
                 widget_handler=None):
        Gst.init(None)
        self.fps = fps
        self.on_filter = on_filter
        self.on_detection = on_detection
        self.widget_handler = widget_handler
        self.running = Event()
        self.running.set()
        self.mainloop = GLib.MainLoop()
        self.error = None
        self.cameras = []
        self.threads = []
        self.count_lock = Lock()
        self.camera_counts = [0, 0]
        self.inference_count = 0
        self.display_fps = [0.0, 0.0]
        self.display_times = [deque(maxlen=90), deque(maxlen=90)]
        self.started = False
        self.stopped = False
        self.last_report_at = monotonic()
        self.last_camera_counts = [0, 0]
        self.last_inference_count = 0

        hef_path = Path(hef_path).expanduser().resolve()
        post_so = Path(post_so).expanduser().resolve()
        if not hef_path.is_file():
            raise FileNotFoundError(f"HEF modeli bulunamadı: {hef_path}")
        if not post_so.is_file():
            raise FileNotFoundError(f"YOLO postprocess bulunamadı: {post_so}")

        # Keep the camera, timestamp, queue and display settings of the
        # 30+30 FPS reference. The identity elements observe each routed
        # stream; neither camera is gated or restarted during handoff.
        display_0 = (
            "gtkwaylandsink name=ui_video_sink_0 sync=false qos=false "
            "enable-last-sample=false"
        ) if widget_handler else "autovideosink sync=false"
        display_1 = (
            "gtkwaylandsink name=ui_video_sink_1 sync=false qos=false "
            "enable-last-sample=false"
        ) if widget_handler else "autovideosink sync=false"
        description = f"""
hailoroundrobin name=rr mode=0 !
    queue name=preinfer_q max-size-buffers=3 !
    hailonet name=infer hef-path="{hef_path}" batch-size=1 force-writable=true !
    queue name=post_q max-size-buffers=3 !
    hailofilter name=post so-path="{post_so}" function-name=filter qos=false !
    queue name=overlay_q max-size-buffers=3 !
    identity name=target_filter signal-handoffs=true !
    queue name=router_q max-size-buffers=3 !
    hailostreamrouter name=router
        src_0::input-streams="<sink_0>"
        src_1::input-streams="<sink_1>"

appsrc name=cam0_src is-live=true do-timestamp=true format=time
    block=false leaky-type=downstream max-buffers=3
    caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={fps}/1 !
    queue name=cam0_in_q leaky=downstream max-size-buffers=3 ! rr.sink_0

appsrc name=cam1_src is-live=true do-timestamp=true format=time
    block=false leaky-type=downstream max-buffers=3
    caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={fps}/1 !
    queue name=cam1_in_q leaky=downstream max-size-buffers=3 ! rr.sink_1

router.src_0 !
    queue name=cam0_display_q leaky=downstream max-size-buffers=2 !
    identity name=detect0 signal-handoffs=true !
    bdtargetoverlay name=aim_0 lock-tolerance=50 !
    hailooverlay name=overlay_0 line-thickness=1 show-confidence=false qos=false !
    videoconvert n-threads=2 qos=false !
    identity name=display_count_0 signal-handoffs=true !
    {display_0}

router.src_1 !
    queue name=cam1_display_q leaky=downstream max-size-buffers=2 !
    identity name=detect1 signal-handoffs=true !
    bdtargetoverlay name=aim_1 lock-tolerance=50 !
    hailooverlay name=overlay_1 line-thickness=1 show-confidence=false qos=false !
    videoconvert n-threads=2 qos=false !
    identity name=display_count_1 signal-handoffs=true !
    {display_1}
"""
        self.pipeline = Gst.parse_launch(description)
        self.sources = [
            self.pipeline.get_by_name("cam0_src"),
            self.pipeline.get_by_name("cam1_src"),
        ]
        infer = self.pipeline.get_by_name("infer")
        infer.get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, self._count_inference
        )
        self.pipeline.get_by_name("target_filter").connect(
            "handoff", self._filter_buffer
        )
        for slot in (0, 1):
            self.pipeline.get_by_name(f"detect{slot}").connect(
                "handoff", self._on_buffer, slot
            )
            self.pipeline.get_by_name(f"display_count_{slot}").connect(
                "handoff", self._on_display_buffer, slot
            )
        if widget_handler:
            for slot in (0, 1):
                sink_element = self.pipeline.get_by_name(f"ui_video_sink_{slot}")
                widget_handler(slot, sink_element.get_property("widget"))
        bus = self.pipeline.get_bus()
        self.bus = bus
        bus.add_signal_watch()
        bus.connect("message", self._on_bus)

    def _count_inference(self, _pad, info):
        if info.get_buffer() is not None:
            with self.count_lock:
                self.inference_count += 1
        return Gst.PadProbeReturn.OK

    def _on_display_buffer(self, _identity, _buffer, slot):
        with self.count_lock:
            self.display_times[slot].append(monotonic())

    def _filter_buffer(self, _identity, buffer):
        try:
            self.on_filter(buffer)
        except Exception as exc:
            self.fail(f"Hedef filtresi hatası: {exc}")

    def _on_buffer(self, _identity, buffer, slot):
        try:
            self.on_detection(slot, buffer)
        except Exception as exc:
            self.fail(f"CAM{slot} tespit hatası: {exc}")

    def _on_bus(self, _bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            if "Output window was closed" in error.message:
                self.request_stop()
                return
            self.fail(f"GStreamer ({message.src.get_name()}): {error}; {debug or ''}")
        elif message.type == Gst.MessageType.EOS:
            self.request_stop()

    def fail(self, message):
        if self.error is None:
            self.error = message
        self.request_stop()

    def request_stop(self):
        self.running.clear()
        GLib.idle_add(self.mainloop.quit)

    def _make_camera(self, index):
        camera_info = Picamera2.global_camera_info()
        expected = "imx296" if index == 0 else "imx477"
        if index >= len(camera_info) or expected not in str(
            camera_info[index].get("Model", "")
        ).lower():
            raise RuntimeError(
                f"CAM{index} {expected} bekleniyor; bulunan: {camera_info}. "
                "Kamera sırasını doğrulayın."
            )
        camera = Picamera2(index)
        try:
            config = camera.create_video_configuration(
                main={"size": (WIDTH, HEIGHT), "format": "BGR888"},
                controls={"FrameRate": self.fps},
                buffer_count=4,
            )
            camera.configure(config)
        except Exception:
            camera.close()
            raise
        return camera

    def _push_camera(self, slot):
        camera = self.cameras[slot]
        source = self.sources[slot]
        while self.running.is_set():
            try:
                frame = camera.capture_array("main")
                frame = np.asarray(frame, dtype=np.uint8)
                frame = np.ascontiguousarray(frame[:, :, :3])
                if frame.shape != (HEIGHT, WIDTH, 3):
                    raise ValueError(f"Beklenmeyen kare boyutu: {frame.shape}")

                raw = frame.tobytes()
                buffer = Gst.Buffer.new_allocate(None, len(raw), None)
                buffer.fill(0, raw)
                # appsrc timestamps live buffers; do not supply manual PTS.
                buffer.pts = Gst.CLOCK_TIME_NONE
                buffer.dts = Gst.CLOCK_TIME_NONE
                buffer.duration = Gst.SECOND // self.fps
                flow = source.emit("push-buffer", buffer)
                if flow in (Gst.FlowReturn.FLUSHING, Gst.FlowReturn.EOS):
                    break
                if flow != Gst.FlowReturn.OK:
                    raise RuntimeError(f"push-buffer sonucu: {flow}")
                with self.count_lock:
                    self.camera_counts[slot] += 1
            except Exception as exc:
                if self.running.is_set():
                    self.fail(f"CAM{slot} yakalama hatası: {exc}")
                break

    def fps_snapshot(self):
        now = monotonic()
        with self.count_lock:
            camera_counts = self.camera_counts.copy()
            inference_count = self.inference_count
            display_fps = []
            for times in self.display_times:
                if len(times) < 2:
                    display_fps.append(0.0)
                else:
                    period = times[-1] - times[0]
                    display_fps.append((len(times) - 1) / period if period > 0 else 0.0)
        elapsed = max(now - self.last_report_at, 0.001)
        camera_fps = [
            (camera_counts[i] - self.last_camera_counts[i]) / elapsed
            for i in (0, 1)
        ]
        inference_fps = (
            inference_count - self.last_inference_count
        ) / elapsed
        self.last_report_at = now
        self.last_camera_counts = camera_counts
        self.last_inference_count = inference_count
        return camera_fps, inference_fps, display_fps

    def start(self):
        try:
            for index in (0, 1):
                self.cameras.append(self._make_camera(index))
            for camera in self.cameras:
                camera.start()
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer PLAYING durumuna geçemedi")
            for slot in (0, 1):
                thread = Thread(
                    target=self._push_camera, args=(slot,), daemon=True,
                    name=f"camera-{slot}",
                )
                self.threads.append(thread)
                thread.start()
            self.started = True
        except Exception:
            self.stop()
            raise

    def run(self):
        try:
            self.start()
            self.mainloop.run()
            if self.error:
                raise RuntimeError(self.error)
        finally:
            self.stop()

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.running.clear()
        for source in self.sources:
            try:
                source.emit("end-of-stream")
            except Exception:
                pass
        self.pipeline.set_state(Gst.State.NULL)
        self.bus.remove_signal_watch()
        for camera in self.cameras:
            try:
                camera.stop()
            except Exception:
                pass
            try:
                camera.close()
            except Exception:
                pass
        for thread in self.threads:
            thread.join(timeout=2.0)
        if self.widget_handler:
            for slot in (0, 1):
                self.widget_handler(slot, None)
