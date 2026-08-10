#!/usr/bin/env python3
"""GTK control station for COCO cell-phone detection and tracking."""

import os
from datetime import datetime
from pathlib import Path

import gi
import psutil

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk

from runtime import RuntimeConfig, VisionRuntime


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
DEFAULT_MODEL = Path("/usr/share/hailo-models/yolov8s_h8.hef")
DEFAULT_LABELS = PROJECT_ROOT / "config" / "coco_labels.json"


def css_class(widget, name):
    widget.get_style_context().add_class(name)
    return widget


def label(text="", style=None, xalign=0.0):
    widget = Gtk.Label(label=text, xalign=xalign)
    if style:
        css_class(widget, style)
    return widget


class ControlWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="CELL PHONE TRACKING CONTROL")
        self.set_default_size(1600, 900)
        self.set_size_request(1080, 650)
        self.connect("delete-event", self._on_delete_event)
        self.connect("destroy", self._on_destroy)
        self.connect("key-press-event", self._on_key_press)

        self.runtime = VisionRuntime()
        self.model_path = DEFAULT_MODEL
        self.video_path = None
        self._fullscreen = False
        self._closing = False
        self._last_table_signature = None
        self._load_css()
        self._build_ui()
        self.runtime.set_widget_handler(self._mount_video_widget)

        GLib.timeout_add(100, self._refresh_runtime)
        GLib.timeout_add_seconds(1, self._refresh_system)
        GLib.timeout_add_seconds(1, self._refresh_clock)
        GLib.idle_add(self._validate_default_model)

    def _load_css(self):
        provider = Gtk.CssProvider()
        provider.load_from_path(str(PROJECT_ROOT / "src" / "theme.css"))
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION,
        )

    def _build_ui(self):
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(root)
        root.pack_start(self._build_header(), False, False, 0)

        body = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        body.set_wide_handle(True)
        body.set_position(1190)
        body.pack1(self._build_left_area(), resize=True, shrink=False)
        body.pack2(self._build_sidebar(), resize=False, shrink=False)
        root.pack_start(body, True, True, 0)
        root.pack_end(self._build_footer(), False, False, 0)

    def _build_header(self):
        box = css_class(Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL), "topbar")
        box.set_border_width(14)

        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        title_box.pack_start(label("CELL PHONE TRACKING", "brand"), False, False, 0)
        title_box.pack_start(
            label("COCO-80 FILTERED VISION & PAN-TILT STATION", "subtitle"),
            False,
            False,
            0,
        )
        box.pack_start(title_box, False, False, 0)

        spacer = Gtk.Box()
        box.pack_start(spacer, True, True, 0)
        self.header_status = label("●  HAZIR", "status-ready", 1.0)
        box.pack_start(self.header_status, False, False, 18)
        self.clock_label = label("", "clock", 1.0)
        box.pack_end(self.clock_label, False, False, 0)
        return box

    def _build_left_area(self):
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        outer.set_border_width(12)

        video_card = css_class(Gtk.Frame(), "video-card")
        self.video_overlay = Gtk.Overlay()
        video_card.add(self.video_overlay)
        self.video_host = Gtk.Box()
        self.video_host.set_size_request(720, 405)
        self.video_overlay.add(self.video_host)

        self.video_placeholder = label(
            "KAMERA / VİDEO BEKLENİYOR\n\nHEF modelini ve kaynağı seçip BAŞLAT'a basın",
            "video-placeholder",
            0.5,
        )
        self.video_placeholder.set_justify(Gtk.Justification.CENTER)
        self.video_overlay.add_overlay(self.video_placeholder)
        outer.pack_start(video_card, True, True, 0)
        outer.pack_start(self._build_target_table(), False, False, 0)
        return outer

    def _build_target_table(self):
        section = css_class(Gtk.Box(orientation=Gtk.Orientation.VERTICAL), "section")
        heading = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        heading.pack_start(label("DOĞRULANMIŞ TELEFONLAR", "section-title"), True, True, 0)
        heading.pack_end(
            label("Satıra tıkla: aktif hedef seç", "hint", 1.0),
            False,
            False,
            0,
        )
        section.pack_start(heading, False, False, 8)

        self.target_store = Gtk.ListStore(
            int, str, str, str, str, str, str, str, str
        )
        self.target_view = Gtk.TreeView(model=self.target_store)
        self.target_view.set_headers_visible(True)
        self.target_view.set_grid_lines(Gtk.TreeViewGridLines.HORIZONTAL)
        self.target_view.connect("row-activated", self._on_target_activated)
        self.target_view.get_selection().connect("changed", self._on_target_selected)
        columns = (
            ("ID", 0),
            ("SINIF", 1),
            ("GÜVEN", 2),
            ("dx px", 3),
            ("dy px", 4),
            ("HATA px", 5),
            ("dx norm", 6),
            ("dy norm", 7),
            ("DURUM", 8),
        )
        for title, index in columns:
            renderer = Gtk.CellRendererText()
            renderer.set_property("xalign", 0.5)
            column = Gtk.TreeViewColumn(title, renderer, text=index)
            column.set_alignment(0.5)
            column.set_expand(title == "DURUM")
            self.target_view.append_column(column)
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        scroll.set_size_request(-1, 140)
        scroll.add(self.target_view)
        section.pack_start(scroll, True, True, 0)
        return section

    def _build_sidebar(self):
        sidebar = css_class(
            Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7), "sidebar"
        )
        sidebar.set_size_request(370, -1)
        sidebar.set_border_width(8)

        sidebar.pack_start(self._build_setup_card(), False, False, 0)
        sidebar.pack_start(self._build_active_card(), False, False, 0)
        sidebar.pack_start(self._build_error_card(), False, False, 0)
        sidebar.pack_start(self._build_control_card(), False, False, 0)
        return sidebar

    def _card(self, title):
        frame = css_class(Gtk.Frame(), "card")
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        box.set_border_width(10)
        box.pack_start(label(title, "section-title"), False, False, 0)
        frame.add(box)
        return frame, box

    def _build_setup_card(self):
        frame, box = self._card("MODEL & KAYNAK")
        box.pack_start(label("HEF MODELİ", "field-label"), False, False, 0)
        model_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=7)
        self.model_entry = Gtk.Entry()
        self.model_entry.set_editable(False)
        self.model_entry.set_text(str(self.model_path))
        model_button = Gtk.Button(label="SEÇ")
        model_button.connect("clicked", self._choose_model)
        model_row.pack_start(self.model_entry, True, True, 0)
        model_row.pack_end(model_button, False, False, 0)
        box.pack_start(model_row, False, False, 0)
        self.model_status = label("Model doğrulanıyor…", "hint")
        self.model_status.set_line_wrap(True)
        box.pack_start(self.model_status, False, False, 0)

        box.pack_start(label("GÖRÜNTÜ KAYNAĞI", "field-label"), False, False, 2)
        self.source_combo = Gtk.ComboBoxText()
        self.source_combo.append("camera", "RPI GLOBAL SHUTTER KAMERA")
        self.source_combo.append("video", "VİDEO DOSYASI")
        self.source_combo.set_active_id("camera")
        self.source_combo.connect("changed", self._source_changed)
        self.video_button = Gtk.Button(label="VİDEO SEÇ")
        self.video_button.set_sensitive(False)
        self.video_button.connect("clicked", self._choose_video)
        source_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=7)
        source_row.pack_start(self.source_combo, True, True, 0)
        source_row.pack_end(self.video_button, False, False, 0)
        box.pack_start(source_row, False, False, 0)
        self.video_status = label("1280×720 @ 40 FPS", "hint")
        self.video_status.set_ellipsize(3)
        box.pack_start(self.video_status, False, False, 0)

        threshold_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        threshold_row.pack_start(label("CONFIDENCE", "field-label"), True, True, 0)
        self.threshold_value = label("0.30", "metric-cyan", 1.0)
        threshold_row.pack_end(self.threshold_value, False, False, 0)
        box.pack_start(threshold_row, False, False, 2)
        adjustment = Gtk.Adjustment(
            value=0.30,
            lower=0.25,
            upper=0.90,
            step_increment=0.01,
            page_increment=0.05,
            page_size=0.0,
        )
        self.threshold_scale = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL, adjustment=adjustment
        )
        self.threshold_scale.set_draw_value(False)
        self.threshold_scale.connect("value-changed", self._threshold_changed)
        box.pack_start(self.threshold_scale, False, False, 0)

        action_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.start_button = css_class(Gtk.Button(label="▶  BAŞLAT"), "primary-button")
        self.start_button.connect("clicked", self._start)
        self.stop_button = css_class(Gtk.Button(label="■  DURDUR"), "danger-button")
        self.stop_button.connect("clicked", self._stop)
        self.stop_button.set_sensitive(False)
        action_row.pack_start(self.start_button, True, True, 0)
        action_row.pack_start(self.stop_button, True, True, 0)
        box.pack_start(action_row, False, False, 4)
        return frame

    def _build_active_card(self):
        frame, box = self._card("AKTİF HEDEF")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        self.active_class = label("HEDEF YOK", "active-class")
        self.active_confidence = label("—", "active-confidence", 1.0)
        row.pack_start(self.active_class, True, True, 0)
        row.pack_end(self.active_confidence, False, False, 0)
        box.pack_start(row, False, False, 2)
        self.active_id = label("ID —", "metric")
        state_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        state_row.pack_start(self.active_id, True, True, 0)
        self.active_state = label("SEARCHING", "state-searching", 1.0)
        state_row.pack_end(self.active_state, False, False, 0)
        box.pack_start(state_row, False, False, 0)
        duration_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        duration_row.pack_start(label("KİLİT SÜRESİ", "field-label"), True, True, 0)
        self.lock_duration = label("0.0 s", "metric", 1.0)
        duration_row.pack_end(self.lock_duration, False, False, 0)
        box.pack_start(duration_row, False, False, 0)
        return frame

    def _metric_row(self, title):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        row.pack_start(label(title, "field-label"), True, True, 0)
        value = label("—", "metric-mono", 1.0)
        row.pack_end(value, False, False, 0)
        return row, value

    def _build_error_card(self):
        frame, box = self._card("HEDEF HATASI")
        for title, attr in (
            ("PIXEL X / Y", "pixel_xy"),
            ("PIXEL TOPLAM", "pixel_total"),
            ("NORMALİZE X / Y", "norm_xy"),
            ("NORMALİZE TOPLAM", "norm_total"),
        ):
            row, value = self._metric_row(title)
            setattr(self, attr, value)
            box.pack_start(row, False, False, 0)
        note = label("X: sağ + / sol −    Y: aşağı + / yukarı −", "hint")
        box.pack_start(note, False, False, 3)
        return frame

    def _build_control_card(self):
        frame, box = self._card("PAN / TILT YÖNLENDİRME")
        self.direction = label("MERKEZ BEKLENİYOR", "direction", 0.5)
        self.direction.set_justify(Gtk.Justification.CENTER)
        box.pack_start(self.direction, False, False, 8)
        self.stm_note = label(
            "STM çıkışı bu sürümde kapalıdır. Arayüz, bağlanacak denetleyici için dx/dy normalize telemetrisini hazır tutar.",
            "hint",
        )
        self.stm_note.set_line_wrap(True)
        box.pack_start(self.stm_note, False, False, 0)
        return frame

    def _build_footer(self):
        footer = css_class(Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=18), "footer")
        footer.set_border_width(9)
        self.system_labels = {}
        for key, title in (
            ("cpu", "CPU"),
            ("ram", "RAM"),
            ("temp", "SICAKLIK"),
            ("hailo", "HAILO-8"),
            ("camera", "KAMERA"),
            ("fps", "FPS"),
            ("drop", "DROP"),
            ("latency", "GECİKME"),
        ):
            item = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
            item.pack_start(label(title, "footer-title"), False, False, 0)
            value = label("—", "footer-value")
            item.pack_start(value, False, False, 0)
            footer.pack_start(item, key in ("hailo", "camera"), key in ("hailo", "camera"), 0)
            self.system_labels[key] = value
        return footer

    def _mount_video_widget(self, widget):
        for child in self.video_host.get_children():
            self.video_host.remove(child)
        if widget is not None:
            self.video_host.pack_start(widget, True, True, 0)
            widget.show()

    def _validate_default_model(self):
        self._validate_model(self.model_path)
        return False

    def _validate_model(self, path):
        try:
            info = self.runtime.validate_model(path)
            self.model_path = Path(info.path)
            self.model_entry.set_text(str(self.model_path))
            shape = "×".join(str(value) for value in info.input_shape)
            self.model_status.set_text(
                f"✓ COCO-80 • yalnız CELL PHONE • giriş {shape} • {info.network}"
            )
            self.model_status.get_style_context().add_class("valid")
            return True
        except Exception as error:
            self.model_status.set_text(f"✕ {error}")
            self.model_status.get_style_context().remove_class("valid")
            return False

    def _choose_model(self, _button):
        dialog = Gtk.FileChooserDialog(
            title="COCO-80 Hailo-8 HEF modelini seç",
            parent=self,
            action=Gtk.FileChooserAction.OPEN,
        )
        dialog.add_buttons(
            "İPTAL", Gtk.ResponseType.CANCEL, "SEÇ", Gtk.ResponseType.OK
        )
        model_filter = Gtk.FileFilter()
        model_filter.set_name("Hailo modeli (*.hef)")
        model_filter.add_pattern("*.hef")
        dialog.add_filter(model_filter)
        dialog.set_current_folder(str(self.model_path.parent))
        if dialog.run() == Gtk.ResponseType.OK:
            self._validate_model(Path(dialog.get_filename()))
        dialog.destroy()

    def _choose_video(self, _button):
        dialog = Gtk.FileChooserDialog(
            title="Test videosunu seç",
            parent=self,
            action=Gtk.FileChooserAction.OPEN,
        )
        dialog.add_buttons(
            "İPTAL", Gtk.ResponseType.CANCEL, "SEÇ", Gtk.ResponseType.OK
        )
        video_filter = Gtk.FileFilter()
        video_filter.set_name("Video (*.mp4, *.webm, *.mkv, *.avi)")
        for pattern in ("*.mp4", "*.webm", "*.mkv", "*.avi"):
            video_filter.add_pattern(pattern)
        dialog.add_filter(video_filter)
        dialog.set_current_folder(str(WORKSPACE_ROOT))
        if dialog.run() == Gtk.ResponseType.OK:
            self.video_path = Path(dialog.get_filename())
            self.video_status.set_text(self.video_path.name)
        dialog.destroy()

    def _source_changed(self, combo):
        video = combo.get_active_id() == "video"
        self.video_button.set_sensitive(video)
        if video:
            self.video_status.set_text(
                self.video_path.name if self.video_path else "Video seçilmedi"
            )
        else:
            self.video_status.set_text("1280×720 @ 40 FPS")

    def _threshold_changed(self, scale):
        value = scale.get_value()
        self.threshold_value.set_text(f"{value:.2f}")
        self.runtime.set_display_threshold(value)

    def _start(self, _button):
        if not self._validate_model(self.model_path):
            return
        source = self.source_combo.get_active_id()
        if source == "video" and self.video_path is None:
            self._show_error("Önce bir video dosyası seçin.")
            return
        config = RuntimeConfig(
            model_path=str(self.model_path),
            labels_path=str(DEFAULT_LABELS),
            source=source,
            video_path=str(self.video_path) if self.video_path else None,
            width=1280,
            height=720,
            frame_rate=40 if source == "camera" else 30,
            display_threshold=self.threshold_scale.get_value(),
        )
        try:
            self.runtime.start(config)
        except Exception as error:
            self.runtime.stop("ERROR", str(error))
            self._show_error(str(error))
            return
        self.video_placeholder.hide()
        self.start_button.set_sensitive(False)
        self.stop_button.set_sensitive(True)
        self.source_combo.set_sensitive(False)

    def _stop(self, _button=None):
        self.runtime.stop()
        self.video_placeholder.show()
        self.start_button.set_sensitive(True)
        self.stop_button.set_sensitive(False)
        self.source_combo.set_sensitive(True)

    def _refresh_runtime(self):
        snapshot = self.runtime.store.snapshot()
        status_map = {
            "RUNNING": ("●  HAILO AKTİF", "status-running"),
            "STARTING": ("●  BAŞLATILIYOR", "status-ready"),
            "STOPPING": ("●  DURDURULUYOR", "status-ready"),
            "ERROR": ("●  HATA", "status-error"),
            "EOS": ("●  VİDEO BİTTİ", "status-ready"),
            "STOPPED": ("●  HAZIR", "status-ready"),
        }
        text, style = status_map.get(snapshot.status, (snapshot.status, "status-ready"))
        self.header_status.set_text(text)
        context = self.header_status.get_style_context()
        for class_name in ("status-running", "status-ready", "status-error"):
            context.remove_class(class_name)
        context.add_class(style)

        active = next(
            (item for item in snapshot.targets if item.track_id == snapshot.active_id),
            None,
        )
        self._show_active(active, snapshot.tracking_state, snapshot.lock_seconds)
        self._update_table(snapshot.targets, snapshot.active_id)
        self.system_labels["fps"].set_text(f"{snapshot.fps:4.1f}")
        self.system_labels["drop"].set_text(str(snapshot.dropped))
        self.system_labels["latency"].set_text(f"{snapshot.latency_ms:4.1f} ms")

        running = snapshot.status in ("RUNNING", "STARTING", "STOPPING")
        if not running and not self.start_button.get_sensitive():
            self.video_placeholder.show()
            self.start_button.set_sensitive(True)
            self.stop_button.set_sensitive(False)
            self.source_combo.set_sensitive(True)
        return True

    def _show_active(self, target, state, lock_seconds):
        self.active_state.set_text(state)
        state_context = self.active_state.get_style_context()
        for class_name in (
            "state-searching",
            "state-acquiring",
            "state-tracking",
            "state-locked",
        ):
            state_context.remove_class(class_name)
        state_context.add_class(f"state-{state.lower()}")
        self.lock_duration.set_text(f"{lock_seconds:.1f} s")
        if target is None:
            self.active_class.set_text("HEDEF YOK")
            self.active_confidence.set_text("—")
            self.active_id.set_text("ID —")
            for widget in (self.pixel_xy, self.pixel_total, self.norm_xy, self.norm_total):
                widget.set_text("—")
            self.direction.set_text("MERKEZ BEKLENİYOR")
            return

        self.active_class.set_text(target.label)
        self.active_confidence.set_text(f"%{target.confidence * 100:.0f}")
        self.active_id.set_text(f"ID {target.track_id}")
        self.pixel_xy.set_text(f"{target.dx_px:+.1f} / {target.dy_px:+.1f}")
        self.pixel_total.set_text(f"{target.error_px:.1f} px")
        self.norm_xy.set_text(f"{target.dx_norm:+.4f} / {target.dy_norm:+.4f}")
        self.norm_total.set_text(f"{target.error_norm:.4f}")
        self.direction.set_text(self._direction_text(target.dx_norm, target.dy_norm))

    @staticmethod
    def _direction_text(dx, dy):
        deadband = 0.02
        horizontal = "" if abs(dx) <= deadband else ("SAĞ" if dx > 0 else "SOL")
        vertical = "" if abs(dy) <= deadband else ("AŞAĞI" if dy > 0 else "YUKARI")
        if not horizontal and not vertical:
            return "● MERKEZDE"
        arrow_x = "→" if dx > deadband else ("←" if dx < -deadband else "")
        arrow_y = "↓" if dy > deadband else ("↑" if dy < -deadband else "")
        return f"{arrow_x}{arrow_y}  {' + '.join(filter(None, (horizontal, vertical)))}"

    def _update_table(self, targets, active_id):
        signature = tuple(
            (
                target.track_id,
                target.label,
                round(target.confidence, 3),
                round(target.dx_px, 1),
                round(target.dy_px, 1),
                active_id,
            )
            for target in targets
        )
        if signature == self._last_table_signature:
            return
        self._last_table_signature = signature
        self.target_store.clear()
        for target in targets:
            self.target_store.append(
                [
                    target.track_id,
                    target.label,
                    f"%{target.confidence * 100:.0f}",
                    f"{target.dx_px:+.1f}",
                    f"{target.dy_px:+.1f}",
                    f"{target.error_px:.1f}",
                    f"{target.dx_norm:+.4f}",
                    f"{target.dy_norm:+.4f}",
                    "AKTİF" if target.track_id == active_id else "İZLENİYOR",
                ]
            )

    def _on_target_activated(self, view, path, _column):
        tree_iter = view.get_model().get_iter(path)
        self.runtime.select_target(view.get_model().get_value(tree_iter, 0))

    def _on_target_selected(self, selection):
        model, tree_iter = selection.get_selected()
        if tree_iter is not None:
            self.runtime.select_target(model.get_value(tree_iter, 0))

    def _refresh_system(self):
        self.system_labels["cpu"].set_text(f"%{psutil.cpu_percent():.0f}")
        self.system_labels["ram"].set_text(f"%{psutil.virtual_memory().percent:.0f}")
        self.system_labels["temp"].set_text(self._temperature())
        hailo_ok = Path("/dev/hailo0").exists()
        camera_ok = Path("/dev/video0").exists() or Path("/dev/video1").exists()
        self.system_labels["hailo"].set_text("BAĞLI" if hailo_ok else "YOK")
        self.system_labels["camera"].set_text("IMX296" if camera_ok else "YOK")
        return True

    @staticmethod
    def _temperature():
        try:
            raw = Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()
            return f"{int(raw) / 1000:.1f}°C"
        except (OSError, ValueError):
            return "—"

    def _refresh_clock(self):
        self.clock_label.set_text(datetime.now().strftime("%d.%m.%Y  %H:%M:%S"))
        return True

    def _show_error(self, message):
        dialog = Gtk.MessageDialog(
            transient_for=self,
            flags=0,
            message_type=Gtk.MessageType.ERROR,
            buttons=Gtk.ButtonsType.CLOSE,
            text="İşlem başlatılamadı",
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _on_key_press(self, _widget, event):
        if event.keyval == Gdk.KEY_F11:
            if self._fullscreen:
                self.unfullscreen()
            else:
                self.fullscreen()
            self._fullscreen = not self._fullscreen
            return True
        if event.keyval == Gdk.KEY_Escape:
            self.unfullscreen()
            self._fullscreen = False
        return False

    def _on_destroy(self, _widget):
        if not self._closing:
            self.runtime.stop()
        Gtk.main_quit()

    def _on_delete_event(self, _widget, _event):
        # Tear the sink down while the DrawingArea/XID still exists. Waiting
        # for "destroy" lets GstGL race the native window destruction.
        if not self._closing:
            self._closing = True
            self.runtime.stop()
        return False


def main():
    window = ControlWindow()
    window.maximize()
    window.show_all()
    window.video_placeholder.show()
    try:
        Gtk.main()
    except KeyboardInterrupt:
        window.runtime.stop()


if __name__ == "__main__":
    main()
