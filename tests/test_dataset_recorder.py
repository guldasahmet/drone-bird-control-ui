import tempfile
import unittest
import json
from pathlib import Path

import cv2
import numpy as np

from dataset_recorder import DatasetRecorder


class DatasetRecorderTests(unittest.TestCase):
    def test_saves_unannotated_samples_from_both_cameras(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = DatasetRecorder(Path(temp) / "dataset", profile_name="phone",
                                       inference_vertical_flip=True)
            directory = recorder.start()
            frame = np.full((640, 640, 3), (10, 80, 170), dtype=np.uint8)
            raw = frame.tobytes()

            self.assertTrue(recorder.offer(0, raw, now=10.0))
            self.assertFalse(recorder.offer(0, raw, now=10.1))
            self.assertTrue(recorder.offer(1, raw, now=10.0))
            recorder.stop()

            status = recorder.status()
            self.assertFalse(status.active)
            self.assertIsNone(status.error)
            self.assertEqual(status.saved, (1, 1))
            self.assertEqual(status.dropped, (0, 0))
            for camera in ("cam0_global_shutter", "cam1_hq"):
                images = list((directory / camera).glob("*.jpg"))
                self.assertEqual(len(images), 1)
                self.assertEqual(cv2.imread(str(images[0])).shape, (640, 640, 3))
            self.assertTrue((directory / "session.json").is_file())
            metadata = json.loads((directory / "session.json").read_text())
            self.assertEqual(metadata["model_profile"], "phone")
            self.assertEqual(metadata["jpeg_orientation"], "camera_native")
            self.assertTrue(metadata["inference_vertical_flip"])

            next_directory = recorder.start()
            self.assertNotEqual(next_directory, directory)
            self.assertTrue(recorder.offer(0, raw, now=20.0))
            recorder.stop()
            self.assertEqual(recorder.status().saved, (1, 0))


if __name__ == "__main__":
    unittest.main()
