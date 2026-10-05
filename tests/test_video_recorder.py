import csv
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from video_recorder import VideoRecorder


class VideoRecorderTests(unittest.TestCase):
    def test_records_both_panels_as_mjpeg_with_frame_times(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = VideoRecorder(Path(temp) / "videos", fps=20)
            frame = np.full((640, 640, 3), (10, 80, 170), dtype=np.uint8)  # RGB
            self.assertFalse(recorder.offer(0, frame.tobytes()))  # not started

            directory = recorder.start()
            for _ in range(3):
                while not recorder.offer(0, frame.tobytes()):
                    pass
                while not recorder.offer(1, frame.tobytes()):
                    pass
            recorder.stop()

            status = recorder.status()
            self.assertFalse(status.active)
            self.assertIsNone(status.error)
            self.assertEqual(status.saved, (3, 3))
            for name in ("cam0_global_shutter.avi", "cam1_hq.avi"):
                video = cv2.VideoCapture(str(directory / name))
                self.assertEqual(int(video.get(cv2.CAP_PROP_FRAME_COUNT)), 3)
                ok, image = video.read()
                video.release()
                self.assertTrue(ok)
                # Standard colours: RGB (10, 80, 170) reads back as BGR.
                np.testing.assert_allclose(image[320, 320], (170, 80, 10), atol=8)
            with open(directory / "frames.csv", newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 6)
            self.assertEqual({row["camera"] for row in rows}, {"0", "1"})


if __name__ == "__main__":
    unittest.main()
