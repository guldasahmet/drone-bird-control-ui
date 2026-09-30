import struct
import unittest
from types import SimpleNamespace

import hailo

from dual_runtime import DualVisionRuntime, HandoffController, target_pixel_errors
from uart import TargetUart


class HandoffTests(unittest.TestCase):
    def test_hq_to_gs_and_back_without_stopping_either_camera(self):
        controller = HandoffController(lost_frames=5, max_age_seconds=0.3)
        self.assertEqual(controller.snapshot(now=1.0)[0], 1)
        phone = object()
        controller.observe(1, phone, now=1.0)
        self.assertIs(controller.snapshot(now=1.1)[1], phone)
        self.assertIsNone(controller.observe(0, phone, now=1.1))
        self.assertEqual(controller.confirmation_progress(now=1.1), (1, 2))
        self.assertEqual(controller.observe(0, phone, now=1.12),
                         "HQ → GLOBAL SHUTTER")
        for frame in range(4):
            self.assertIsNone(controller.observe(0, None, now=1.14 + frame * 0.02))
            self.assertEqual(controller.snapshot(now=1.2)[0], 0)
        self.assertEqual(controller.observe(0, None, now=1.22),
                         "GLOBAL SHUTTER → HQ")
        self.assertEqual(controller.snapshot(now=1.22)[0], 1)
        self.assertIsNone(controller.snapshot(now=1.4)[1])

    def test_gs_confirmation_resets_after_miss_and_stale_gap(self):
        controller = HandoffController(max_age_seconds=0.3)
        phone = object()
        self.assertIsNone(controller.observe(0, phone, now=1.0))
        self.assertIsNone(controller.observe(0, None, now=1.02))
        self.assertEqual(controller.confirmation_progress(now=1.02), (0, 2))
        self.assertIsNone(controller.observe(0, phone, now=1.04))
        self.assertEqual(controller.confirmation_progress(now=1.4), (0, 2))
        self.assertIsNone(controller.observe(0, phone, now=1.4))
        self.assertEqual(controller.observe(0, phone, now=1.42),
                         "HQ → GLOBAL SHUTTER")

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

    def test_phone_profile_reverses_both_uart_axes(self):
        runtime = DualVisionRuntime(lambda _slot, _widget: None)

        class FakeSerial:
            is_open = True

            def __init__(self):
                self.packets = []

            def write(self, packet):
                self.packets.append(packet)

        link = TargetUart(
            True, "/dev/null", 115200,
            invert_x=runtime.invert_x, invert_y=runtime.invert_y,
        )
        link.serial = FakeSerial()
        self.assertEqual(link.send_target(80, 60), (-80, -60))
        self.assertEqual(link.serial.packets[-1], struct.pack("<Bhh", 0xFF, -80, -60))
        self.assertEqual(link.send_target(10, -20, locked=True), (-10, 20))
        self.assertEqual(link.serial.packets[-1], struct.pack("<Bhh", 0xFE, -10, 20))
        link.send_no_target()
        self.assertEqual(link.serial.packets[-1], struct.pack("<Bhh", 0xFF, 0, 0))

    def test_phone_uart_matches_denme_pixel_center_rounding(self):
        target = SimpleNamespace(center_x=0.49, center_y=0.51)
        self.assertEqual(target_pixel_errors(target), (-7, 6))


if __name__ == "__main__":
    unittest.main()
