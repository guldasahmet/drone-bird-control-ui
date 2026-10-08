#!/usr/bin/env python3
"""GTK view for the two-camera, one-Hailo tracking pipeline."""

import argparse
from datetime import datetime
from pathlib import Path

import gi
import psutil

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk

from dual_runtime import CAMERA_NAMES, DualVisionRuntime
from model_profile import DEFAULT_CONFIG, load_active_profile


ROOT = Path(__file__).resolve().parents[1]


def label(text="", style=None):
    item = Gtk.Label(label=text, xalign=0)
    if style:
        item.get_style_context().add_class(style)
    return item


def panel(title):
    frame = Gtk.Frame()
    frame.get_style_context().add_class("card")
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    box.set_border_width(12)
    box.pack_start(label(title, "section-title"), False, False, 0)
    frame.add(box)
    return frame, box


class DualControlWindow(Gtk.Window):
    def __init__(self, args):
        super().__init__(title="DRONE–BIRD CONTROL | ÇİFT KAMERA")
        self.set_default_size(1600, 930)
        self.set_size_request(1080, 650)
        self.connect("delete-event", self._on_close)
        self.connect("key-press-event", self._on_key)
        self._closing = False
        self._fullscreen = False
        self._table_signature = None
        self.profile = load_active_profile(getattr(args, "config", DEFAULT_CONFIG))
        self.runtime = DualVisionRuntime(
            self._mount_widget,
            profile=self.profile,
            uart_enabled=args.uart,
            display_backend=getattr(args, "display_backend", "gtk"),
            hold_hq=getattr(args, "hold_hq", False),
            trace_dir=ROOT / "logs" if getattr(args, "trace", False) else None,
            max_step_px=getattr(args, "max_step", 30),
            unlock_tolerance=getattr(args, "unlock_px", 50),
            fps=getattr(args, "fps", None),
            queue_depth=getattr(args, "queue_depth", 1),
            batch_size=getattr(args, "batch_size", None),
        )
        self._load_theme()
        self._build()
        GLib.timeout_add(100, self._refresh)
        GLib.timeout_add_seconds(1, self._metrics)

    def _load_theme(self):
        provider = Gtk.CssProvider()
        provider.load_from_path(str(ROOT / "src" / "theme.css"))
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION,
        )

    def _build(self):
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(root)

        top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
        top.get_style_context().add_class("topbar")
        top.set_border_width(14)
        brand = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        brand.pack_start(label("DRONE–BIRD CONTROL", "brand"), False, False, 0)
        brand.pack_start(label(
            f"İKİ KAMERA • TEK HAILO • {self.profile.display_name.upper()} HEDEFİ",
            "subtitle"),
                         False, False, 0)
        top.pack_start(brand, True, True, 0)
        self.header_status = label("● HAZIR", "status-ready")
        top.pack_end(self.header_status, False, False, 0)
        root.pack_start(top, False, False, 0)

        body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        body.set_border_width(10)
        root.pack_start(body, True, True, 0)
        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        body.pack_start(left, True, True, 0)

        videos = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        left.pack_start(videos, True, True, 0)
        self.video_hosts = []
        self.video_placeholders = []
        self.camera_labels = []
        for slot in (0, 1):
            frame, box = panel(f"CAM{slot} · {CAMERA_NAMES[slot]}")
            frame.get_style_context().add_class("video-card")
            camera_status = label("0.0 FPS | hedef yok", "hint")
            box.pack_start(camera_status, False, False, 0)
            self.camera_labels.append(camera_status)
            overlay = Gtk.Overlay()
            host = Gtk.Box()
            host.set_size_request(320, 320)
            overlay.add(host)
            placeholder = label("KAMERA BEKLENİYOR", "video-placeholder")
            placeholder.set_halign(Gtk.Align.CENTER)
            placeholder.set_valign(Gtk.Align.CENTER)
            overlay.add_overlay(placeholder)
            box.pack_start(overlay, True, True, 0)
            videos.pack_start(frame, True, True, 0)
            self.video_hosts.append(host)
            self.video_placeholders.append(placeholder)

        frame, box = panel(
            f"{self.profile.display_name.upper()} HEDEFLERİ · "
            "AKTİF KAMERADAKİ HEDEF SEÇİLEBİLİR"
        )
        self.target_store = Gtk.ListStore(int, int, str, str, str, str, str, str)
        view = Gtk.TreeView(model=self.target_store)
        for title, column in (("KAMERA", 2), ("ID", 3), ("SINIF", 4),
                              ("GÜVEN", 5), ("dx px", 6), ("dy px", 7)):
            cell = Gtk.CellRendererText()
            view.append_column(Gtk.TreeViewColumn(title, cell, text=column))
        view.get_selection().connect("changed", self._select_target)
        scroll = Gtk.ScrolledWindow()
        scroll.set_size_request(-1, 130)
        scroll.add(view)
        box.pack_start(scroll, True, True, 0)
        left.pack_start(frame, False, False, 0)

        sidebar = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        sidebar.set_size_request(295, -1)
        body.pack_end(sidebar, False, False, 0)

        frame, box = panel("ÇALIŞTIRMA")
        model = label(
            f"PROFİL: {self.profile.display_name} ({self.profile.name})\n"
            f"MODEL: {self.runtime.model_path.name}", "hint"
        )
        model.set_line_wrap(True)
        box.pack_start(model, False, False, 0)
        box.pack_start(label(
            f"640×640 · 30 + 30 FPS · {', '.join(self.profile.target_names)}",
            "hint"),
                       False, False, 0)
        box.pack_start(label("CONFIDENCE", "field-label"), False, False, 0)
        self.threshold_text = label(f"{self.profile.confidence:.2f}", "metric-cyan")
        box.pack_start(self.threshold_text, False, False, 0)
        self.threshold = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL,
                                                  self.profile.high_threshold,
                                                  0.90, 0.01)
        self.threshold.set_value(self.profile.confidence)
        self.threshold.set_draw_value(False)
        self.threshold.connect("value-changed", self._change_threshold)
        box.pack_start(self.threshold, False, False, 0)
        self.start_button = Gtk.Button(label="▶ BAŞLAT")
        self.start_button.get_style_context().add_class("primary-button")
        self.start_button.connect("clicked", self._start)
        box.pack_start(self.start_button, False, False, 0)
        self.stop_button = Gtk.Button(label="■ DURDUR")
        self.stop_button.get_style_context().add_class("danger-button")
        self.stop_button.set_sensitive(False)
        self.stop_button.connect("clicked", self._stop)
        box.pack_start(self.stop_button, False, False, 0)
        self.record_button = Gtk.Button(label="● DATASET KAYDINI BAŞLAT")
        self.record_button.set_sensitive(False)
        self.record_button.connect("clicked", self._toggle_recording)
        box.pack_start(self.record_button, False, False, 0)
        self.recording_label = label("JPEG · kamera başına 2 FPS · kalite 92", "hint")
        self.recording_label.set_line_wrap(True)
        box.pack_start(self.recording_label, False, False, 0)
        self.video_button = Gtk.Button(label="● VİDEO KAYDINI BAŞLAT")
        self.video_button.set_sensitive(False)
        self.video_button.connect("clicked", self._toggle_video)
        box.pack_start(self.video_button, False, False, 0)
        self.video_label = label("MJPEG · kutular çizili · videos/", "hint")
        self.video_label.set_line_wrap(True)
        box.pack_start(self.video_label, False, False, 0)
        sidebar.pack_start(frame, False, False, 0)

        frame, box = panel("KONTROL DEVRİ")
        self.control_label = label("HQ", "active-class")
        box.pack_start(self.control_label, False, False, 0)
        self.transition_label = label("Başlangıç kontrolü HQ", "hint")
        box.pack_start(self.transition_label, False, False, 0)
        self.uart_label = label("UART: KAPALI", "status-ready")
        box.pack_start(self.uart_label, False, False, 0)
        sidebar.pack_start(frame, False, False, 0)

        frame, box = panel("AKTİF HEDEF")
        self.target_label = label("HEDEF YOK", "active-class")
        self.error_label = label("dx —   dy —", "metric-mono")
        self.direction_label = label("MERKEZ BEKLENİYOR", "direction")
        box.pack_start(self.target_label, False, False, 0)
        box.pack_start(self.error_label, False, False, 0)
        box.pack_start(self.direction_label, False, False, 0)
        sidebar.pack_start(frame, False, False, 0)

        frame, box = panel("PERFORMANS")
        self.performance_label = label("HAILO — | CAM0 — | CAM1 —", "metric-mono")
        self.performance_label.set_line_wrap(True)
        box.pack_start(self.performance_label, False, False, 0)
        self.system_label = label("", "hint")
        box.pack_start(self.system_label, False, False, 0)
        sidebar.pack_start(frame, False, False, 0)

        footer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        footer.get_style_context().add_class("footer")
        footer.set_border_width(9)
        self.message_label = label("Hazır", "footer-value")
        footer.pack_start(self.message_label, True, True, 0)
        self.clock_label = label("", "clock")
        footer.pack_end(self.clock_label, False, False, 0)
        root.pack_end(footer, False, False, 0)

    def _mount_widget(self, slot, widget):
        host = self.video_hosts[slot]
        for child in host.get_children():
            host.remove(child)
        if widget is not None:
            host.pack_start(widget, True, True, 0)
            widget.show()
            self.video_placeholders[slot].hide()
        else:
            self.video_placeholders[slot].show()

    def _change_threshold(self, scale):
        value = scale.get_value()
        self.threshold_text.set_text(f"{value:.2f}")
        self.runtime.set_confidence(value)

    def _start(self, _button):
        try:
            self.runtime.start()
        except Exception as error:
            self._show_error(str(error))
            return
        self.start_button.set_sensitive(False)
        self.stop_button.set_sensitive(True)

    def _stop(self, _button=None):
        self.runtime.stop()
        self.start_button.set_sensitive(True)
        self.stop_button.set_sensitive(False)

    def _toggle_recording(self, _button):
        try:
            recording = self.runtime.recording_status()
            if recording is not None and recording.active:
                self.runtime.stop_recording()
            else:
                self.runtime.start_recording()
        except Exception as error:
            self._show_error(str(error), "Dataset kaydı başlatılamadı")

    def _toggle_video(self, _button):
        try:
            video = self.runtime.video_status()
            if video is not None and video.active:
                self.runtime.stop_video()
            else:
                self.runtime.start_video()
        except Exception as error:
            self._show_error(str(error), "Video kaydı başlatılamadı")

    def _show_error(self, message, title="Çift kamera başlatılamadı"):
        dialog = Gtk.MessageDialog(
            transient_for=self,
            message_type=Gtk.MessageType.ERROR,
            buttons=Gtk.ButtonsType.CLOSE,
            text=title,
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _select_target(self, selection):
        model, row = selection.get_selected()
        if row is None:
            return
        slot = model.get_value(row, 0)
        if slot == self.runtime.controller.snapshot()[0]:
            self.runtime.select_target(slot, model.get_value(row, 1))

    def _refresh(self):
        snapshot = self.runtime.snapshot()
        if snapshot.status == "ERROR":
            error = snapshot.message
            self._stop()
            self._show_error(error)
            snapshot = self.runtime.snapshot()
        self.header_status.set_text(f"● {snapshot.status}")
        self.message_label.set_text(snapshot.message)
        self.control_label.set_text(CAMERA_NAMES[snapshot.active_slot])
        if snapshot.active_slot == 1 and snapshot.gs_confirm_count:
            self.transition_label.set_text(
                f"GS doğrulanıyor {snapshot.gs_confirm_count}/"
                f"{snapshot.gs_confirm_frames}"
            )
        elif snapshot.transition:
            self.transition_label.set_text(snapshot.transition)
        else:
            self.transition_label.set_text("Başlangıç kontrolü HQ")
        self.uart_label.set_text(
            "UART: AÇIK" if snapshot.uart_enabled and snapshot.status == "RUNNING"
            else "UART: KAPALI"
        )
        recording = self.runtime.recording_status()
        self.record_button.set_sensitive(snapshot.status == "RUNNING")
        self.record_button.set_label(
            "■ DATASET KAYDINI DURDUR"
            if recording is not None and recording.active
            else "● DATASET KAYDINI BAŞLAT"
        )
        if recording is not None and recording.error:
            self.recording_label.set_text(f"Kayıt hatası: {recording.error}")
        elif recording is not None and recording.directory is not None:
            state = "KAYIT AÇIK" if recording.active else "Kayıt kapalı"
            self.recording_label.set_text(
                f"{state} · {recording.directory.name}\n"
                f"CAM0 {recording.saved[0]} · CAM1 {recording.saved[1]} JPEG · "
                f"düşen {sum(recording.dropped)}"
            )
        else:
            self.recording_label.set_text("JPEG · kamera başına 2 FPS · kalite 92")
        video = self.runtime.video_status()
        self.video_button.set_sensitive(snapshot.status == "RUNNING")
        self.video_button.set_label(
            "■ VİDEO KAYDINI DURDUR" if video is not None and video.active
            else "● VİDEO KAYDINI BAŞLAT"
        )
        if video is not None and video.error:
            self.video_label.set_text(f"Video hatası: {video.error}")
        elif video is not None and video.directory is not None:
            state = "VİDEO AÇIK" if video.active else "Video kapalı"
            self.video_label.set_text(
                f"{state} · {video.directory.name}\n"
                f"CAM0 {video.saved[0]} · CAM1 {video.saved[1]} kare · "
                f"düşen {sum(video.dropped)}"
            )
        target = snapshot.active_target
        if target is None:
            self.target_label.set_text("HEDEF YOK")
            self.error_label.set_text("dx —   dy —")
            self.direction_label.set_text("MERKEZ BEKLENİYOR")
        else:
            self.target_label.set_text(f"{target.label} · %{target.confidence * 100:.0f}")
            self.error_label.set_text(
                f"dx {target.dx_px:+.0f} px   dy {target.dy_px:+.0f} px"
            )
            self.direction_label.set_text(
                "MERKEZDE" if abs(target.dx_px) <= 50 and abs(target.dy_px) <= 50
                else ("SAĞ" if target.dx_px > 0 else "SOL") + " / "
                     + ("AŞAĞI" if target.dy_px > 0 else "YUKARI")
            )
        table_rows = []
        for slot, result in enumerate(snapshot.results):
            if result is None:
                continue
            for item in result.targets:
                table_rows.append((slot, item.track_id, CAMERA_NAMES[slot],
                                   str(item.track_id), item.label,
                                   f"%{item.confidence * 100:.0f}",
                                   f"{item.dx_px:+.0f}", f"{item.dy_px:+.0f}"))
        signature = tuple(table_rows)
        if signature != self._table_signature:
            self._table_signature = signature
            self.target_store.clear()
            for row in table_rows:
                self.target_store.append(row)
        for slot in (0, 1):
            result = snapshot.results[slot]
            item = None if result is None else next(
                (value for value in result.targets if value.track_id == result.active_id),
                None,
            )
            target_text = "hedef yok" if item is None else f"{item.label} %{item.confidence * 100:.0f}"
            control = " · KONTROL" if slot == snapshot.active_slot else ""
            self.camera_labels[slot].set_text(
                f"{snapshot.display_fps[slot]:.1f} FPS | {target_text}{control}"
            )
        self.performance_label.set_text(
            f"HAILO {snapshot.infer_fps:.1f} FPS\n"
            f"CAM0 kamera {snapshot.camera_fps[0]:.1f} · "
            f"hat {snapshot.branch_fps[0]:.1f} · "
            f"ekran {snapshot.display_fps[0]:.1f}\n"
            f"CAM1 kamera {snapshot.camera_fps[1]:.1f} · "
            f"hat {snapshot.branch_fps[1]:.1f} · "
            f"ekran {snapshot.display_fps[1]:.1f} FPS"
        )
        return True

    def _metrics(self):
        self.runtime.update_metrics()
        self.system_label.set_text(
            f"CPU %{psutil.cpu_percent():.0f} · RAM %{psutil.virtual_memory().percent:.0f}"
        )
        self.clock_label.set_text(datetime.now().strftime("%d.%m.%Y %H:%M:%S"))
        return True

    def _on_key(self, _widget, event):
        if event.keyval == Gdk.KEY_F11:
            self.unfullscreen() if self._fullscreen else self.fullscreen()
            self._fullscreen = not self._fullscreen
            return True
        return False

    def _on_close(self, _widget, _event):
        if not self._closing:
            self._closing = True
            self.runtime.stop()
        Gtk.main_quit()
        return False


def main():
    parser = argparse.ArgumentParser(description="İki kamera, tek Hailo kontrol arayüzü")
    parser.add_argument("--uart", action="store_true", help="STM32 UART çıkışını aç")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="Aktif model profilinin bulunduğu config.json")
    parser.add_argument(
        "--display-backend", choices=("gtk", "wayland"), default="gtk",
        help="gtk: VNC uyumlu arayüz çizimi; wayland: eski doğrudan sink",
    )
    parser.add_argument("--hold-hq", action="store_true",
                        help="Kontrolü HQ'da sabitle; GS'ye kamera devri yapma")
    parser.add_argument("--trace", action="store_true",
                        help="Kare kare tespit/UART iz kaydını logs/ altına CSV yaz")
    parser.add_argument("--max-step", type=int, default=30,
                        help="UART hata komutunun 50 ms'de en fazla değişimi (px); 0 kapalı")
    parser.add_argument("--fps", type=int, default=None,
                        help="Kamera FPS (varsayılan: profildeki fps, yoksa 30)")
    parser.add_argument("--queue-depth", type=int, default=1,
                        help="Hailo öncesi kuyruk derinliği (eski değer 3)")
    parser.add_argument("--batch-size", type=int, default=None,
                        help="Hailo batch boyutu (varsayılan: profildeki batch_size, yoksa 1)")
    parser.add_argument("--unlock-px", type=int, default=50,
                        help="Kilidin bırakıldığı hata (px); kilitlenme 25 px altında")
    args = parser.parse_args()
    try:
        window = DualControlWindow(args)
    except (ValueError, FileNotFoundError) as error:
        parser.error(str(error))
    window.maximize()
    window.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
