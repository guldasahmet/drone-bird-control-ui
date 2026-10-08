import json
import tempfile
import unittest
from pathlib import Path

from model_profile import load_active_profile


class ModelProfileTests(unittest.TestCase):
    def test_active_profile_controls_classes_and_paths(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "custom.hef").touch()
            (root / "post.so").touch()
            data = {
                "active_profile": "bird",
                "profiles": {
                    "bird": {
                        "display_name": "Kuş",
                        "hef": "custom.hef",
                        "postprocess_so": "post.so",
                        "postprocess": {
                            "detection_threshold": 0.10,
                            "max_boxes": 100,
                            "labels": ["BIRD", "DRONE"],
                        },
                        "expected_classes": 2,
                        "targets": [
                            {"id": 0, "label": "bird"},
                            {"id": 1, "label": "drone"},
                        ],
                        "priority": ["drone", "bird"],
                        "sticky": ["drone"],
                        "confidence": 0.35,
                        "low_threshold": 0.10,
                        "high_threshold": 0.20,
                        "new_track_threshold": 0.25,
                    },
                    "unused": {"hef": "missing.hef"},
                },
            }
            config = root / "config.json"
            config.write_text(json.dumps(data), encoding="utf-8")
            profile = load_active_profile(config)
            self.assertEqual(profile.name, "bird")
            self.assertEqual(profile.labels_by_id, {0: "BIRD", 1: "DRONE"})
            self.assertEqual(profile.priority, ("DRONE", "BIRD"))
            self.assertEqual(profile.hef, root / "custom.hef")
            self.assertEqual(profile.postprocess.as_dict(), {
                "detection_threshold": 0.10,
                "max_boxes": 100,
                "labels": ["BIRD", "DRONE"],
            })
            self.assertFalse(profile.camera_flip_vertical)

            data["camera"] = {"flip_vertical": True}
            config.write_text(json.dumps(data), encoding="utf-8")
            self.assertTrue(load_active_profile(config).camera_flip_vertical)

            self.assertEqual(load_active_profile(config).fps, 30)
            data["profiles"]["bird"]["fps"] = 20
            config.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(load_active_profile(config).fps, 20)
            data["profiles"]["bird"]["fps"] = 0
            config.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "fps"):
                load_active_profile(config)
            del data["profiles"]["bird"]["fps"]
            config.write_text(json.dumps(data), encoding="utf-8")

            self.assertEqual(load_active_profile(config).camera_error_scale,
                             ((1.0, 1.0), (1.0, 1.0)))
            data["camera"]["error_scale"] = {"global_shutter": [0.5, 0.62]}
            config.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(load_active_profile(config).camera_error_scale,
                             ((0.5, 0.62), (1.0, 1.0)))
            data["camera"]["error_scale"] = {"hq": [0, 1]}
            config.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "error_scale"):
                load_active_profile(config)
            del data["camera"]["error_scale"]
            config.write_text(json.dumps(data), encoding="utf-8")

            self.assertEqual(load_active_profile(config).batch_size, 1)
            data["profiles"]["bird"]["batch_size"] = 2
            config.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(load_active_profile(config).batch_size, 2)
            data["profiles"]["bird"]["batch_size"] = 0
            config.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "batch_size"):
                load_active_profile(config)
            del data["profiles"]["bird"]["batch_size"]

            data["profiles"]["bird"]["postprocess"]["labels"][0] = "PHONE"
            config.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "postprocess etiketleri"):
                load_active_profile(config)
            data["profiles"]["bird"]["postprocess"]["labels"][0] = "BIRD"

            data["profiles"]["bird"]["targets"][1]["id"] = 2
            config.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "hedef id/label"):
                load_active_profile(config)


if __name__ == "__main__":
    unittest.main()
