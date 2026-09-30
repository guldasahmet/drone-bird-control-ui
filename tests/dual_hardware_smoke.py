#!/usr/bin/env python3
"""Start the new GTK UI with UART off, sample both cameras, then close it."""

import os
from pathlib import Path
import sys

project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root / "src"))

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk

from dual_app import DualControlWindow


class Args:
    uart = False
    config = Path(os.environ.get("SMOKE_CONFIG", project_root / "config.json"))


def main():
    window = DualControlWindow(Args())
    errors = []

    def show_error(message, _title=None):
        errors.append(message)
        print(f"DUAL_UI_ERROR {message}", flush=True)

    window._show_error = show_error
    window.maximize()
    window.show_all()
    cycle_results = []
    final_results = []
    recording_results = []

    def start():
        window._start(None)
        return False

    def finish():
        if os.environ.get("SMOKE_DIAG") == "1" and window.runtime.pipeline:
            pipeline = window.runtime.pipeline
            print(
                "DISPLAY_DIAG "
                f"generated={pipeline.frame_generation} "
                f"drawn={pipeline.drawn_generation} "
                f"pending={pipeline.pending_updates} "
                f"window_mapped={window.get_mapped()} "
                f"widgets_mapped={[widget.get_mapped() for widget in pipeline.video_widgets]} ",
                flush=True,
            )
            for slot, pixbuf in enumerate(pipeline.pixbufs):
                if pixbuf is not None:
                    pixbuf.savev(f"/tmp/drone-ui-smoke-cam{slot}.png", "png", [], [])
        if os.environ.get("SMOKE_RECORD") == "1":
            window.runtime.stop_recording()
            recording_results.append(window.runtime.recording_status())
        sample = window.runtime.snapshot()
        final_results.append(sample)
        window.close()
        print(
            "DUAL_UI_SMOKE "
            f"status={sample.status} message={sample.message!r} "
            f"camera={sample.camera_fps} display={sample.display_fps} "
            f"hailo={sample.infer_fps:.1f} uart={sample.uart_enabled}",
            flush=True,
        )
        if recording_results:
            recording = recording_results[-1]
            print(
                f"DATASET_SMOKE saved={recording.saved} "
                f"dropped={recording.dropped} error={recording.error!r} "
                f"directory={recording.directory}",
                flush=True,
            )
        return False

    def start_recording():
        window._toggle_recording(None)
        return False

    def sample_and_stop(cycle):
        sample = window.runtime.snapshot()
        cycle_results.append(sample)
        print(
            f"DUAL_UI_CYCLE {cycle} status={sample.status} "
            f"camera={sample.camera_fps} display={sample.display_fps} "
            f"hailo={sample.infer_fps:.1f} uart={sample.uart_enabled}",
            flush=True,
        )
        window._stop()
        return False

    cycles = int(os.environ.get("SMOKE_CYCLES", "0"))
    if cycles:
        for index in range(cycles):
            GLib.timeout_add(500 + 5000 * index, start)
            GLib.timeout_add(4500 + 5000 * index,
                             sample_and_stop, index + 1)
        GLib.timeout_add(5000 * cycles + 500, finish)
    else:
        GLib.timeout_add(500, start)
        if os.environ.get("SMOKE_RECORD") == "1":
            GLib.timeout_add(3000, start_recording)
        if os.environ.get("SMOKE_RESTART") == "1":
            GLib.timeout_add_seconds(4, lambda: window._stop() or False)
            GLib.timeout_add_seconds(5, start)
        GLib.timeout_add_seconds(int(os.environ.get("SMOKE_SECONDS", "8")), finish)
    Gtk.main()
    if errors or not final_results or final_results[-1].status != "RUNNING":
        raise SystemExit("Çift kamera başlatma testi başarısız")
    if os.environ.get("SMOKE_RECORD") == "1" and (
        not recording_results
        or recording_results[-1] is None
        or recording_results[-1].error
        or min(recording_results[-1].saved) < 2
    ):
        raise SystemExit("Dataset kayıt testi başarısız")
    if cycles and (
        len(cycle_results) != cycles
        or any(sample.status != "RUNNING"
               or min(sample.display_fps) < 20
               or sample.infer_fps < 40
               for sample in cycle_results)
    ):
        raise SystemExit("Çift kamera yeniden başlatma testi başarısız")


if __name__ == "__main__":
    main()
