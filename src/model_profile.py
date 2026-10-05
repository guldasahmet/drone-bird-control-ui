"""Load one active detection model profile from the project configuration."""

from dataclasses import dataclass
import json
from pathlib import Path
import re


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config.json"


@dataclass(frozen=True)
class PostprocessConfig:
    detection_threshold: float
    max_boxes: int
    labels: tuple[str, ...]

    def as_dict(self):
        return {
            "detection_threshold": self.detection_threshold,
            "max_boxes": self.max_boxes,
            "labels": list(self.labels),
        }


@dataclass(frozen=True)
class ModelProfile:
    name: str
    display_name: str
    hef: Path
    postprocess_so: Path
    postprocess_function: str
    postprocess: PostprocessConfig | None
    match_by: str
    expected_classes: int
    targets: tuple[tuple[int, str], ...]
    priority: tuple[str, ...]
    sticky: tuple[str, ...]
    confidence: float
    low_threshold: float
    high_threshold: float
    new_track_threshold: float
    nms_score_threshold: float | None
    nms_iou_threshold: float | None
    camera_flip_vertical: bool
    fps: int = 30
    batch_size: int = 1

    @property
    def labels_by_id(self):
        return dict(self.targets)

    @property
    def target_names(self):
        return tuple(label for _, label in self.targets)

    def validate_hef(self, hef_class):
        """Reject HEFs that cannot feed this 640x640 Hailo NMS pipeline."""
        hef = hef_class(str(self.hef))
        inputs = hef.get_input_vstream_infos()
        outputs = hef.get_output_vstream_infos()
        if len(inputs) != 1 or tuple(inputs[0].shape) != (640, 640, 3):
            raise ValueError(f"{self.name}: HEF girişi 640×640×3 olmalı")
        if (len(outputs) != 1 or len(outputs[0].shape) != 3
                or int(outputs[0].shape[0]) != self.expected_classes
                or int(outputs[0].shape[1]) != 5):
            raise ValueError(
                f"{self.name}: HEF çıkışı {self.expected_classes} sınıflı "
                "Hailo NMS [sınıf, 5, kutu] olmalı"
            )


def load_active_profile(config_path=DEFAULT_CONFIG):
    path = Path(config_path).expanduser().resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("profiles"), dict):
        raise ValueError("config.json içinde profiles nesnesi gerekli")
    name = data.get("active_profile")
    if not isinstance(name, str) or name not in data["profiles"]:
        raise ValueError("active_profile, profiles içindeki bir profil olmalı")
    item = data["profiles"][name]
    if not isinstance(item, dict):
        raise ValueError(f"{name}: profil nesnesi gerekli")
    camera = data.get("camera", {})
    if not isinstance(camera, dict) or type(camera.get("flip_vertical", False)) is not bool:
        raise ValueError("camera.flip_vertical true veya false olmalı")
    flip_vertical = camera.get("flip_vertical", False)

    def file_path(key, required=True):
        value = item.get(key)
        if value is None and not required:
            return None
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name}: {key} dosya yolu gerekli")
        target = Path(value).expanduser()
        if not target.is_absolute():
            target = path.parent / target
        target = target.resolve()
        if not target.is_file():
            raise FileNotFoundError(f"{name}: {key} bulunamadı: {target}")
        return target

    display_name = item.get("display_name")
    if not isinstance(display_name, str) or not display_name.strip():
        raise ValueError(f"{name}: display_name gerekli")
    function = item.get("postprocess_function", "filter")
    if not isinstance(function, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", function):
        raise ValueError(f"{name}: geçersiz postprocess_function")
    match_by = item.get("match_by", "label")
    if match_by not in ("label", "id"):
        raise ValueError(f"{name}: match_by 'label' veya 'id' olmalı")
    count = item.get("expected_classes")
    if type(count) is not int or count <= 0:
        raise ValueError(f"{name}: expected_classes pozitif tam sayı olmalı")
    raw_targets = item.get("targets")
    if not isinstance(raw_targets, list) or not raw_targets:
        raise ValueError(f"{name}: en az bir hedef gerekli")
    targets = []
    for target in raw_targets:
        if not isinstance(target, dict):
            raise ValueError(f"{name}: hedef nesnesi gerekli")
        class_id, label = target.get("id"), target.get("label")
        if (type(class_id) is not int or not 0 <= class_id < count
                or not isinstance(label, str) or not label.strip()):
            raise ValueError(f"{name}: hedef id/label geçersiz")
        targets.append((class_id, label.strip().upper()))
    ids = [class_id for class_id, _ in targets]
    labels = [label for _, label in targets]
    if len(ids) != len(set(ids)) or len(labels) != len(set(labels)):
        raise ValueError(f"{name}: hedef id ve adları benzersiz olmalı")

    postprocess_data = item.get("postprocess")
    postprocess = None
    if postprocess_data is not None:
        if not isinstance(postprocess_data, dict):
            raise ValueError(f"{name}: postprocess nesnesi gerekli")
        post_labels = postprocess_data.get("labels")
        if (not isinstance(post_labels, list) or len(post_labels) != count
                or any(not isinstance(label, str) or not label.strip()
                       for label in post_labels)):
            raise ValueError(f"{name}: postprocess.labels {count} etiket içermeli")
        detection_threshold = postprocess_data.get("detection_threshold")
        max_boxes = postprocess_data.get("max_boxes")
        if (type(detection_threshold) not in (float, int)
                or not 0 < detection_threshold <= 1
                or type(max_boxes) is not int or max_boxes <= 0):
            raise ValueError(f"{name}: postprocess eşik veya max_boxes geçersiz")
        if match_by == "label" and any(
            post_labels[class_id].strip().upper() != label
            for class_id, label in targets
        ):
            raise ValueError(f"{name}: postprocess etiketleri hedef id/label ile uyuşmuyor")
        postprocess = PostprocessConfig(
            detection_threshold=float(detection_threshold),
            max_boxes=max_boxes,
            labels=tuple(label.strip() for label in post_labels),
        )

    def label_list(key, default):
        values = item.get(key, default)
        if not isinstance(values, list) or any(
            not isinstance(value, str) for value in values
        ):
            raise ValueError(f"{name}: {key} liste olmalı")
        return tuple(value.strip().upper() for value in values)

    priority = label_list("priority", labels)
    sticky = label_list("sticky", labels)
    if len(priority) != len(labels) or set(priority) != set(labels):
        raise ValueError(f"{name}: priority bütün hedefleri bir kez içermeli")
    if len(sticky) != len(set(sticky)) or not set(sticky).issubset(labels):
        raise ValueError(f"{name}: sticky yalnız hedef sınıflarını içermeli")

    def threshold(key, default=None, maximum=0.90):
        value = item.get(key, default)
        if type(value) not in (float, int) or not 0 < value <= maximum:
            raise ValueError(f"{name}: {key} 0 ile {maximum} arasında olmalı")
        return float(value)

    low = threshold("low_threshold")
    high = threshold("high_threshold")
    new = threshold("new_track_threshold")
    confidence = threshold("confidence")
    if not (low <= high <= new and high <= confidence):
        raise ValueError(f"{name}: eşikler low ≤ high ≤ new ve confidence ≥ high olmalı")
    nms_score = (threshold("nms_score_threshold")
                 if "nms_score_threshold" in item else None)
    nms_iou = (threshold("nms_iou_threshold", maximum=1.0)
               if "nms_iou_threshold" in item else None)
    if nms_score is not None and nms_score > low:
        raise ValueError(f"{name}: nms_score_threshold low_threshold değerini aşmamalı")
    fps = item.get("fps", 30)
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError(f"{name}: fps 1 ile 60 arasında tam sayı olmalı")
    batch_size = item.get("batch_size", 1)
    if type(batch_size) is not int or not 1 <= batch_size <= 8:
        raise ValueError(f"{name}: batch_size 1 ile 8 arasında tam sayı olmalı")

    return ModelProfile(
        name=name, display_name=display_name.strip(),
        hef=file_path("hef"), postprocess_so=file_path("postprocess_so"),
        postprocess_function=function,
        postprocess=postprocess,
        match_by=match_by,
        expected_classes=count, targets=tuple(targets),
        priority=priority, sticky=sticky, confidence=confidence,
        low_threshold=low, high_threshold=high, new_track_threshold=new,
        nms_score_threshold=nms_score, nms_iou_threshold=nms_iou,
        camera_flip_vertical=flip_vertical,
        fps=fps,
        batch_size=batch_size,
    )
