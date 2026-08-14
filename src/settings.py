"""TOML tabanlı Drone/Bird uygulama ayarları."""

from dataclasses import dataclass
from pathlib import Path
import tomllib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "app.toml"


@dataclass(frozen=True)
class ModelSettings:
    path: Path
    labels: Path
    expected_classes: int


@dataclass(frozen=True)
class VideoSettings:
    width: int
    height: int
    camera_fps: int
    mirror_x_axis: bool


@dataclass(frozen=True)
class TrackingSettings:
    classes: tuple[str, ...]
    priority: tuple[str, ...]
    sticky_labels: tuple[str, ...]
    confidence: float
    low_threshold: float
    high_threshold: float
    new_track_threshold: float
    match_threshold: float
    display_match_iou: float
    track_buffer: int
    min_confirmed_hits: int
    lock_tolerance_px: int


@dataclass(frozen=True)
class UartSettings:
    enabled: bool
    port: str
    baudrate: int
    invert_x: bool


@dataclass(frozen=True)
class AppSettings:
    model: ModelSettings
    video: VideoSettings
    tracking: TrackingSettings
    uart: UartSettings


def _project_path(value):
    path = Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (PROJECT_ROOT / path).resolve()


def _section(data, name):
    section = data.get(name)
    if not isinstance(section, dict):
        raise ValueError(f"Config bölümü eksik veya hatalı: [{name}]")
    return section


def _labels(values, field):
    if not isinstance(values, list) or not values:
        raise ValueError(f"{field} boş olmayan bir liste olmalı")
    labels = tuple(str(value).strip().upper() for value in values)
    if any(not label for label in labels) or len(set(labels)) != len(labels):
        raise ValueError(f"{field} boş veya tekrarlanan sınıf içeriyor")
    return labels


def load_settings(path=DEFAULT_CONFIG_PATH):
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Config bulunamadı: {config_path}")

    with config_path.open("rb") as handle:
        data = tomllib.load(handle)

    model_data = _section(data, "model")
    video_data = _section(data, "video")
    tracking_data = _section(data, "tracking")
    uart_data = _section(data, "uart")

    classes = _labels(tracking_data.get("classes"), "tracking.classes")
    priority = _labels(tracking_data.get("priority"), "tracking.priority")
    sticky = tuple(
        str(value).strip().upper()
        for value in tracking_data.get("sticky_labels", [])
    )
    if set(priority) != set(classes):
        raise ValueError("tracking.priority bütün sınıfları bir kez içermeli")
    if not set(sticky).issubset(classes):
        raise ValueError("tracking.sticky_labels takip sınıflarında bulunmalı")

    expected_classes = int(model_data.get("expected_classes", 0))
    width = int(video_data.get("width", 0))
    height = int(video_data.get("height", 0))
    camera_fps = int(video_data.get("camera_fps", 0))
    confidence = float(tracking_data.get("confidence", 0.0))
    low = float(tracking_data.get("low_threshold", 0.0))
    high = float(tracking_data.get("high_threshold", 0.0))
    new_track = float(tracking_data.get("new_track_threshold", 0.0))

    if expected_classes != len(classes):
        raise ValueError("Model sınıf sayısı ile tracking.classes uyuşmuyor")
    if width <= 0 or height <= 0 or camera_fps <= 0:
        raise ValueError("Video genişliği, yüksekliği ve FPS pozitif olmalı")
    if not 0.0 <= low <= high <= confidence <= 0.90:
        raise ValueError("Confidence eşikleri 0 <= low <= high <= confidence <= 0.90 olmalı")
    if not high <= new_track <= 0.90:
        raise ValueError("new_track_threshold high_threshold değerinden küçük olamaz")

    return AppSettings(
        model=ModelSettings(
            path=_project_path(model_data.get("path", "")),
            labels=_project_path(model_data.get("labels", "")),
            expected_classes=expected_classes,
        ),
        video=VideoSettings(
            width=width,
            height=height,
            camera_fps=camera_fps,
            mirror_x_axis=bool(video_data.get("mirror_x_axis", True)),
        ),
        tracking=TrackingSettings(
            classes=classes,
            priority=priority,
            sticky_labels=sticky,
            confidence=confidence,
            low_threshold=low,
            high_threshold=high,
            new_track_threshold=new_track,
            match_threshold=float(tracking_data.get("match_threshold", 0.85)),
            display_match_iou=float(tracking_data.get("display_match_iou", 0.10)),
            track_buffer=max(1, int(tracking_data.get("track_buffer", 2))),
            min_confirmed_hits=max(
                1, int(tracking_data.get("min_confirmed_hits", 2))
            ),
            lock_tolerance_px=max(
                0, int(tracking_data.get("lock_tolerance_px", 25))
            ),
        ),
        uart=UartSettings(
            enabled=bool(uart_data.get("enabled", True)),
            port=str(uart_data.get("port", "/dev/ttyACM0")),
            baudrate=int(uart_data.get("baudrate", 115200)),
            invert_x=bool(uart_data.get("invert_x", True)),
        ),
    )
