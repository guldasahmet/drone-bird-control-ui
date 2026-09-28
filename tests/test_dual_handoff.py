import unittest

import hailo

from dual_runtime import DualVisionRuntime, HandoffController


class HandoffTests(unittest.TestCase):
    def test_hq_to_gs_and_back_without_stopping_either_camera(self):
        controller = HandoffController(lost_frames=5, max_age_seconds=0.3)
        self.assertEqual(controller.snapshot(now=1.0)[0], 1)
        phone = object()
        controller.observe(1, phone, now=1.0)
        self.assertIs(controller.snapshot(now=1.1)[1], phone)
        self.assertEqual(controller.observe(0, phone, now=1.1),
                         "HQ → GLOBAL SHUTTER")
        for frame in range(4):
            self.assertIsNone(controller.observe(0, None, now=1.12 + frame * 0.02))
            self.assertEqual(controller.snapshot(now=1.2)[0], 0)
        self.assertEqual(controller.observe(0, None, now=1.2),
                         "GLOBAL SHUTTER → HQ")
        self.assertEqual(controller.snapshot(now=1.2)[0], 1)
        self.assertIsNone(controller.snapshot(now=1.4)[1])

    def test_phone_track_is_selected_per_camera(self):
        runtime = DualVisionRuntime(lambda _slot, _widget: None)

        def frame():
            roi = hailo.HailoROI(hailo.HailoBBox(0, 0, 1, 1))
            roi.add_object(hailo.HailoDetection(
                hailo.HailoBBox(0.1, 0.1, 0.2, 0.2), 0, "person", 0.95
            ))
            roi.add_object(hailo.HailoDetection(
                hailo.HailoBBox(0.6, 0.3, 0.2, 0.2), 67, "cell phone", 0.85
            ))
            return roi

        for slot in (0, 1):
            runtime.trackers[slot].process(frame(), 640, 640)
            result = runtime.trackers[slot].process(frame(), 640, 640)
            self.assertEqual(len(result.targets), 1)
            self.assertEqual(result.targets[0].label, "CELL PHONE")
            self.assertEqual(result.active_id, result.targets[0].track_id)


if __name__ == "__main__":
    unittest.main()
