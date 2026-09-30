"""Two-camera Hailo inference for IMX477 and IMX296."""

from argparse import ArgumentParser
from collections import deque
import json
import os
from pathlib import Path
from queue import Empty, Queue as ThreadQueue
import signal
from threading import Event, Thread
from time import monotonic

import gi
import hailo

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from hailo_platform import HEF
from picamera2 import Picamera2

from hailo_apps.python.core.common.core import get_resource_path
from hailo_apps.python.core.common.defines import (
    DETECTION_PIPELINE,
    DETECTION_POSTPROCESS_FUNCTION,
    DETECTION_POSTPROCESS_SO_FILENAME,
    RESOURCES_SO_DIR_NAME,
    TAPPAS_POSTPROC_PATH_KEY,
    TAPPAS_STREAM_ID_TOOL_SO_FILENAME,
)
from hailo_apps.python.core.gstreamer.gstreamer_common import disable_qos
from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    QUEUE,
)

from settings import load_settings
from tracking import ClassAwareByteTracker


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CAMERA_NAMES = {0: "IMX477", 1: "IMX296"}


def parse_args():
    settings = load_settings()
    parser = ArgumentParser(
        description="İki Raspberry Pi kamerasını tek Hailo-8 üzerinde çalıştırır."
    )
    parser.add_argument("--camera0", type=int, default=0, help="Birinci kamera indeksi")
    parser.add_argument("--camera1", type=int, default=1, help="İkinci kamera indeksi")
    parser.add_argument("--width", type=int, default=settings.video.width)
    parser.add_argument("--height", type=int, default=settings.video.height)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--confidence",
        type=float,
        default=settings.tracking.confidence,
        help="Terminalde gösterilecek en düşük güven değeri",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=10,
        help="Her kamera için kaç çıkarımda bir durum yazılacağı",
    )
    parser.add_argument(
        "--flip-camera0",
        action="store_true",
        help="Kamera 0 görüntüsünü yukarı-aşağı çevirir",
    )
    parser.add_argument(
        "--flip-camera1",
        action="store_true",
        help="Kamera 1 görüntüsünü yukarı-aşağı çevirir",
    )
    parser.add_argument(
        "--display",
        action="store_true",
        help="İki kamerayı tespit kutularıyla tek pencerede yan yana gösterir",
    )
    args = parser.parse_args()

    if args.camera0 == args.camera1:
        parser.error("camera0 ve camera1 farklı indeksler olmalı")
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        parser.error("width, height ve fps pozitif olmalı")
    if not 0.0 <= args.confidence <= 0.90:
        parser.error("confidence 0.0 ile 0.90 arasında olmalı")
    if args.log_every <= 0:
        parser.error("log-every pozitif olmalı")
    return args, settings


class DualCameraHeadless:
    def __init__(self, args, settings):
        Gst.init(None)
        self.args = args
        self.settings = settings
        self.pipeline = None
        self.stop_event = Event()
        self.push_event = Event()
        self.camera_errors = ThreadQueue()
        self.camera_threads = []
        self.frame_counts = {0: 0, 1: 0}
        self.frame_times = {0: deque(maxlen=90), 1: deque(maxlen=90)}
        self.camera_indices = {0: args.camera0, 1: args.camera1}
        self.camera_names = {
            slot: CAMERA_NAMES.get(index, f"CAMERA-{index}")
            for slot, index in self.camera_indices.items()
        }
        self.stream_slots = {"src_0": 0, "src_1": 1}

        self.model_path = settings.model.path.resolve()
        self.labels_path = settings.model.labels.resolve()
        self._validate_model_and_labels()
        self.postprocess_path = self._resolve_postprocess()
        self.stream_id_path = self._resolve_stream_id_filter()
        self.trackers = {
            slot: self._new_tracker() for slot in self.camera_indices
        }

    def _validate_model_and_labels(self):
        if not self.model_path.is_file():
            raise FileNotFoundError(f"HEF modeli bulunamadı: {self.model_path}")
        if not self.labels_path.is_file():
            raise FileNotFoundError(f"Labels dosyası bulunamadı: {self.labels_path}")

        hef = HEF(str(self.model_path))
        inputs = hef.get_input_vstream_infos()
        outputs = hef.get_output_vstream_infos()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Model tek giriş ve tek Hailo NMS çıkışına sahip olmalı")
        output_shape = tuple(outputs[0].shape)
        if len(output_shape) != 3 or output_shape[1] != 5:
            raise ValueError(f"Model çıkışı Hailo NMS biçiminde değil: {output_shape}")
        if int(output_shape[0]) != self.settings.model.expected_classes:
            raise ValueError(
                "Model sınıf sayısı config/app.toml ile uyuşmuyor: "
                f"model={output_shape[0]} config={self.settings.model.expected_classes}"
            )

        with self.labels_path.open("r", encoding="utf-8") as handle:
            labels = tuple(
                str(label).upper() for label in json.load(handle).get("labels", [])
            )
        if labels != tuple(self.settings.tracking.classes):
            raise ValueError(
                f"Labels sırası hatalı: beklenen={self.settings.tracking.classes} "
                f"bulunan={labels}"
            )

    @staticmethod
    def _resolve_postprocess():
        path = get_resource_path(
            DETECTION_PIPELINE,
            RESOURCES_SO_DIR_NAME,
            "hailo8",
            DETECTION_POSTPROCESS_SO_FILENAME,
        )
        if path is None or not Path(path).is_file():
            raise FileNotFoundError(f"Hailo detection post-process bulunamadı: {path}")
        return Path(path).resolve()

    @staticmethod
    def _resolve_stream_id_filter():
        postprocess_dir = os.environ.get(TAPPAS_POSTPROC_PATH_KEY, "")
        path = Path(postprocess_dir) / TAPPAS_STREAM_ID_TOOL_SO_FILENAME
        if not path.is_file():
            raise FileNotFoundError(
                "Hailo stream-id filtresi bulunamadı. Önce hailo-apps/setup_env.sh "
                f"yüklenmeli. Aranan dosya: {path}"
            )
        return path.resolve()

    def _new_tracker(self):
        tracking = self.settings.tracking
        return ClassAwareByteTracker(
            frame_rate=self.args.fps,
            low_threshold=tracking.low_threshold,
            high_threshold=tracking.high_threshold,
            new_track_threshold=tracking.new_track_threshold,
            display_threshold=self.args.confidence,
            match_threshold=tracking.match_threshold,
            track_buffer=tracking.track_buffer,
            min_confirmed_hits=tracking.min_confirmed_hits,
            display_match_iou=tracking.display_match_iou,
            lock_tolerance_px=tracking.lock_tolerance_px,
            class_labels=tracking.classes,
            priority_labels=tracking.priority,
            sticky_labels=tracking.sticky_labels,
        )

    def _source_branch(self, slot, flip):
        flip_element = ""
        if flip:
            flip_element = (
                f"videoflip name=camera_{slot}_flip video-direction=vert qos=false ! "
            )
        return (
            f"appsrc name=camera_src_{slot} is-live=true format=time block=false "
            "max-buffers=1 leaky-type=downstream ! "
            f"video/x-raw,format=RGB,width={self.args.width},"
            f"height={self.args.height},framerate={self.args.fps}/1,"
            "pixel-aspect-ratio=1/1 ! "
            f"{QUEUE(name=f'camera_{slot}_q', max_size_buffers=1, leaky='downstream')} ! "
            f"{flip_element}"
            f"hailofilter name=set_src_{slot} so-path={self.stream_id_path} "
            f"config-path=src_{slot} qos=false ! "
            f"{QUEUE(name=f'stream_{slot}_q', max_size_buffers=1, leaky='downstream')} ! "
            f"robin.sink_{slot} "
        )

    def _pipeline_string(self):
        thresholds = (
            f"nms-score-threshold={self.settings.tracking.low_threshold:.2f} "
            "nms-iou-threshold=0.45 "
            "output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )
        inference = INFERENCE_PIPELINE(
            hef_path=str(self.model_path),
            post_process_so=str(self.postprocess_path),
            post_function_name=DETECTION_POSTPROCESS_FUNCTION,
            batch_size=1,
            config_json=str(self.labels_path),
            additional_params=thresholds,
            name="dual_inference",
        )
        inference_wrapper = INFERENCE_PIPELINE_WRAPPER(
            inference,
            bypass_max_size_buffers=4,
            name="dual_inference_wrapper",
        )
        sources = self._source_branch(0, self.args.flip_camera0)
        sources += self._source_branch(1, self.args.flip_camera1)
        sink = self._display_pipeline() if self.args.display else self._headless_sink()
        output = (
            "hailoroundrobin mode=1 name=robin ! "
            f"{inference_wrapper} ! "
            f"{QUEUE(name='headless_callback_q', max_size_buffers=1, leaky='downstream')} ! "
            "identity name=headless_callback signal-handoffs=true ! "
            f"{sink}"
        )
        return sources + output

    @staticmethod
    def _headless_sink():
        return (
            "fakesink name=headless_sink sync=false qos=false "
            "enable-last-sample=false"
        )

    def _display_branch(self, slot):
        label = f"CAM{slot} {self.camera_names[slot]}"
        return (
            f"router.src_{slot} ! "
            f"{QUEUE(name=f'display_{slot}_overlay_q', max_size_buffers=1, leaky='downstream')} ! "
            f"hailooverlay name=display_{slot}_overlay line-thickness=2 "
            "show-confidence=true qos=false ! "
            f"{QUEUE(name=f'display_{slot}_convert_q', max_size_buffers=1, leaky='downstream')} ! "
            f"videoconvert name=display_{slot}_convert n-threads=2 qos=false ! "
            f'textoverlay name=display_{slot}_label text="{label}" '
            "valignment=top halignment=left shaded-background=true "
            'font-desc="Sans Bold 18" ! '
            f"mix.sink_{slot} "
        )

    def _display_pipeline(self):
        combined_width = self.args.width * 2
        router = (
            "hailostreamrouter name=router "
            'src_0::input-streams="<sink_0>" '
            'src_1::input-streams="<sink_1>" '
        )
        branches = self._display_branch(0) + self._display_branch(1)
        compositor = (
            "compositor name=mix background=black "
            "sink_0::xpos=0 sink_0::ypos=0 "
            f"sink_1::xpos={self.args.width} sink_1::ypos=0 ! "
            f"video/x-raw,width={combined_width},height={self.args.height},"
            f"framerate={self.args.fps}/1 ! "
            f"{QUEUE(name='combined_display_q', max_size_buffers=1, leaky='downstream')} ! "
            "videoconvert name=combined_display_convert n-threads=2 qos=false ! "
            "autovideosink name=combined_display_sink sync=false"
        )
        return router + branches + compositor

    def _camera_worker(self, slot):
        camera_index = self.camera_indices[slot]
        source = self.pipeline.get_by_name(f"camera_src_{slot}")
        try:
            source.set_property("is-live", True)
            source.set_property("format", Gst.Format.TIME)
            duration = Gst.util_uint64_scale_int(1, Gst.SECOND, self.args.fps)
            with Picamera2(camera_index) as camera:
                camera_config = camera.create_preview_configuration(
                    main={
                        "size": (self.args.width, self.args.height),
                        "format": "BGR888",
                    },
                    controls={"FrameRate": self.args.fps},
                    buffer_count=4,
                )
                camera.configure(camera_config)
                camera.start()
                self.push_event.wait(timeout=15.0)
                if self.stop_event.is_set():
                    return

                print(
                    f"CAM{slot} {self.camera_names[slot]} hazır "
                    f"(libcamera index={camera_index})",
                    flush=True,
                )
                while not self.stop_event.is_set():
                    frame = camera.capture_array("main")
                    if frame is None:
                        raise RuntimeError("Picamera2 boş kare döndürdü")
                    buffer = Gst.Buffer.new_wrapped(frame.tobytes())
                    clock = self.pipeline.get_clock()
                    if clock is not None:
                        buffer.pts = max(
                            0, clock.get_time() - self.pipeline.get_base_time()
                        )
                    else:
                        buffer.pts = Gst.CLOCK_TIME_NONE
                    buffer.dts = buffer.pts
                    buffer.duration = duration
                    flow = source.emit("push-buffer", buffer)
                    if flow not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                        raise RuntimeError(f"appsrc push-buffer sonucu: {flow}")
                    if flow == Gst.FlowReturn.FLUSHING:
                        break
        except Exception as error:
            if not self.stop_event.is_set():
                self.camera_errors.put((slot, error))
                self.stop_event.set()

    @staticmethod
    def _stream_key(roi):
        return str(roi.get_stream_id()).replace("'", "")

    def _on_frame(self, _element, buffer):
        try:
            roi = hailo.get_roi_from_buffer(buffer)
            stream_key = self._stream_key(roi)
            slot = self.stream_slots.get(stream_key)
            if slot is None:
                raise RuntimeError(f"Bilinmeyen Hailo stream-id: {stream_key!r}")

            result = self.trackers[slot].process(
                roi,
                self.args.width,
                self.args.height,
            )
            if self.args.display:
                self._add_overlay_objects(roi, result)
            self.frame_counts[slot] += 1
            self.frame_times[slot].append(monotonic())
            if self.frame_counts[slot] % self.args.log_every == 0:
                self._print_result(slot, result)
        except Exception as error:
            if not self.stop_event.is_set():
                self.camera_errors.put((-1, error))
                self.stop_event.set()

    @staticmethod
    def _add_overlay_objects(roi, result):
        for target in result.targets:
            bbox = hailo.HailoBBox(
                target.x1,
                target.y1,
                target.x2 - target.x1,
                target.y2 - target.y1,
            )
            active = target.track_id == result.active_id
            label = (
                f"AKTIF {target.label} ID:{target.track_id}"
                if active
                else f"{target.label} ID:{target.track_id}"
            )
            roi.add_object(hailo.HailoDetection(bbox, 0, label, target.confidence))

    def _fps(self, slot):
        times = self.frame_times[slot]
        if len(times) < 2:
            return 0.0
        elapsed = times[-1] - times[0]
        return (len(times) - 1) / elapsed if elapsed > 0 else 0.0

    def _print_result(self, slot, result):
        prefix = (
            f"CAM{slot} {self.camera_names[slot]} | "
            f"FPS={self._fps(slot):4.1f} | {result.state}"
        )
        if not result.targets:
            print(f"{prefix} | HEDEF YOK", flush=True)
            return

        active = next(
            (target for target in result.targets if target.track_id == result.active_id),
            None,
        )
        if active is None:
            print(f"{prefix} | {len(result.targets)} HEDEF | AKTİF YOK", flush=True)
            return
        print(
            f"{prefix} | {len(result.targets)} HEDEF | "
            f"{active.label} ID={active.track_id} "
            f"CONF={active.confidence:.2f} "
            f"HATA_X={active.dx_px:+.1f}px "
            f"HATA_Y={active.dy_px:+.1f}px",
            flush=True,
        )

    def run(self):
        self.pipeline = Gst.parse_launch(self._pipeline_string())
        callback = self.pipeline.get_by_name("headless_callback")
        if callback is None:
            raise RuntimeError("Hailo callback elemanı oluşturulamadı")
        callback.connect("handoff", self._on_frame)
        disable_qos(self.pipeline)

        for slot in self.camera_indices:
            thread = Thread(
                target=self._camera_worker,
                args=(slot,),
                name=f"camera-{slot}-capture",
                daemon=True,
            )
            self.camera_threads.append(thread)
            thread.start()

        state_result = self.pipeline.set_state(Gst.State.PLAYING)
        if state_result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer PLAYING durumuna geçemedi")
        self.push_event.set()

        print("Çift kamera Hailo tespiti başladı. Durdurmak için Ctrl+C.", flush=True)
        display_status = "açık" if self.args.display else "kapalı"
        print(f"UART kapalı, görüntü penceresi {display_status}.", flush=True)
        bus = self.pipeline.get_bus()
        while not self.stop_event.is_set():
            message = bus.timed_pop_filtered(
                250 * Gst.MSECOND,
                Gst.MessageType.ERROR | Gst.MessageType.EOS,
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                raise RuntimeError(f"GStreamer: {error}; {debug or ''}")
            if message.type == Gst.MessageType.EOS:
                break

        try:
            slot, error = self.camera_errors.get_nowait()
        except Empty:
            return
        source = "Hailo callback" if slot < 0 else f"CAM{slot}"
        raise RuntimeError(f"{source}: {error}")

    def stop(self):
        self.stop_event.set()
        self.push_event.set()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        for thread in self.camera_threads:
            if thread.is_alive():
                thread.join(timeout=3.0)


def main():
    args, settings = parse_args()
    app = DualCameraHeadless(args, settings)

    def request_stop(_signum, _frame):
        app.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        app.run()
    finally:
        app.stop()


if __name__ == "__main__":
    main()
