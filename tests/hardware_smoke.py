#!/usr/bin/env python3
"""IMX296/Hailo/Wayland başlangıç ve temiz kapanış smoke testi."""

import os
import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root / "src"))

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk

from app import ControlWindow


def main():
    run_seconds = int(os.environ.get("SMOKE_SECONDS", "8"))
    os.environ.setdefault("DRONE_BIRD_DISABLE_UART", "1")
    window = ControlWindow()
    window.maximize()
    window.show_all()
    window.video_placeholder.show()

    def start_source():
        smoke_video = os.environ.get("SMOKE_VIDEO")
        if smoke_video:
            window.source_combo.set_active_id("video")
            window.video_path = Path(smoke_video).resolve()
            window.video_status.set_text(window.video_path.name)
        window._start(None)
        return False

    def close_window():
        snapshot = window.runtime.store.snapshot()
        window.close()
        print(
            "GUI_SHUTDOWN_OK "
            f"status={snapshot.status} message={snapshot.message!r} "
            f"frames={snapshot.frame_index} fps={snapshot.fps:.2f} "
            f"drops={snapshot.dropped} latency_ms={snapshot.latency_ms:.2f}"
        )
        return False

    GLib.timeout_add(500, start_source)
    GLib.timeout_add_seconds(run_seconds, close_window)
    Gtk.main()


if __name__ == "__main__":
    main()
