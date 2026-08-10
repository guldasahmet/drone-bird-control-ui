import unittest

import hailo

from tracking import ClassAwareByteTracker, _target_telemetry


class TrackingPolicyTests(unittest.TestCase):
    def setUp(self):
        self.tracker = ClassAwareByteTracker()

    @staticmethod
    def target(track_id, label, x, y, confidence=0.8):
        half = 0.02
        return _target_telemetry(
            track_id,
            label,
            confidence,
            [x - half, y - half, x + half, y + half],
            1280,
            720,
        )

    def test_nearest_phone_is_selected(self):
        far = self.target(1, "CELL PHONE", 0.90, 0.90)
        near = self.target(2, "CELL PHONE", 0.52, 0.51)
        self.tracker._select_active_target([far, near])
        self.assertEqual(self.tracker.active_id, 2)

    def test_visible_active_id_is_retained(self):
        retained = self.target(4, "CELL PHONE", 0.90, 0.90)
        new_closer = self.target(5, "CELL PHONE", 0.51, 0.51)
        self.tracker.active_id = 4
        self.tracker._select_active_target([retained, new_closer])
        self.assertEqual(self.tracker.active_id, 4)

    def test_manual_target_falls_back_after_loss(self):
        first = self.target(1, "CELL PHONE", 0.70, 0.50)
        second = self.target(2, "CELL PHONE", 0.50, 0.50)
        self.tracker.select_target(2)
        self.tracker._select_active_target([first, second])
        self.assertEqual(self.tracker.active_id, 2)
        self.tracker._select_active_target([first])
        self.assertEqual(self.tracker.active_id, 1)
        self.assertIsNone(self.tracker.manual_id)

    def test_normalized_axes_use_half_frame(self):
        target = self.target(1, "CELL PHONE", 0.75, 0.25)
        self.assertAlmostEqual(target.dx_px, 320.0)
        self.assertAlmostEqual(target.dy_px, -180.0)
        self.assertAlmostEqual(target.dx_norm, 0.5)
        self.assertAlmostEqual(target.dy_norm, -0.5)

    def test_only_cell_phone_reaches_tracker(self):
        def frame():
            roi = hailo.HailoROI(hailo.HailoBBox(0.0, 0.0, 1.0, 1.0))
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.1, 0.1, 0.2, 0.3),
                    0,
                    "person",
                    0.95,
                )
            )
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.4, 0.3, 0.1, 0.2),
                    68,
                    "cell phone",
                    0.85,
                )
            )
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.402, 0.302, 0.1, 0.2),
                    68,
                    "cell phone",
                    0.40,
                )
            )
            return roi

        first = self.tracker.process(frame(), 1280, 720)
        second = self.tracker.process(frame(), 1280, 720)
        self.assertEqual(first.raw_count, 1)
        self.assertEqual(second.raw_count, 1)
        self.assertEqual(len(second.targets), 1)
        self.assertEqual(second.targets[0].label, "CELL PHONE")


if __name__ == "__main__":
    unittest.main()
