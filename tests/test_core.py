import struct
import unittest

import hailo

from runtime import VisionRuntime
from settings import load_settings
from tracking import ClassAwareByteTracker, _target_telemetry
from uart import PACKET_LOCKED, PACKET_TRACKING, TargetUart, pack_target_packet


class FakeSerial:
    is_open = True
    in_waiting = 0

    def __init__(self):
        self.packets = []

    def write(self, packet):
        self.packets.append(packet)

    def close(self):
        self.is_open = False


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_settings()
        tracking = self.settings.tracking
        self.tracker = ClassAwareByteTracker(
            class_labels=tracking.classes,
            priority_labels=tracking.priority,
            sticky_labels=tracking.sticky_labels,
            min_confirmed_hits=tracking.min_confirmed_hits,
            lock_tolerance_px=tracking.lock_tolerance_px,
        )

    @staticmethod
    def target(track_id, label, x, y, confidence=0.8):
        half = 0.02
        return _target_telemetry(
            track_id,
            label,
            confidence,
            [x - half, y - half, x + half, y + half],
            640,
            480,
        )

    def test_settings_and_model_match_two_class_project(self):
        self.assertEqual(self.settings.tracking.classes, ("DRONE", "BIRD"))
        self.assertEqual(self.settings.video.width, 640)
        self.assertEqual(self.settings.video.height, 480)
        self.assertTrue(self.settings.model.path.is_file())
        info = VisionRuntime.validate_model(
            self.settings.model.path,
            self.settings.model.expected_classes,
        )
        self.assertEqual(info.classes, 2)
        self.assertEqual(info.input_shape, (640, 640, 3))
        with self.assertRaises(ValueError):
            VisionRuntime.validate_model(self.settings.model.path, 3)
        VisionRuntime.validate_labels(
            self.settings.model.labels,
            self.settings.tracking.classes,
        )

    def test_drone_has_priority_over_bird(self):
        bird = self.target(1, "BIRD", 0.51, 0.50)
        drone = self.target(2, "DRONE", 0.90, 0.90)
        self.tracker._select_active_target([bird, drone])
        self.assertEqual(self.tracker.active_id, 2)

    def test_visible_active_drone_id_is_retained(self):
        retained = self.target(4, "DRONE", 0.90, 0.90)
        closer = self.target(5, "DRONE", 0.51, 0.51)
        self.tracker.active_id = 4
        self.tracker._select_active_target([retained, closer])
        self.assertEqual(self.tracker.active_id, 4)

    def test_nearest_bird_is_fallback_when_no_drone_exists(self):
        far = self.target(6, "BIRD", 0.90, 0.90)
        near = self.target(7, "BIRD", 0.52, 0.51)
        self.tracker._select_active_target([far, near])
        self.assertEqual(self.tracker.active_id, 7)

    def test_manual_visible_target_overrides_class_priority(self):
        drone = self.target(8, "DRONE", 0.50, 0.50)
        bird = self.target(9, "BIRD", 0.70, 0.50)
        self.tracker.select_target(9)
        self.tracker._select_active_target([drone, bird])
        self.assertEqual(self.tracker.active_id, 9)

    def test_pixel_and_normalized_error_use_640_by_480_center(self):
        target = self.target(1, "DRONE", 0.75, 0.25)
        self.assertAlmostEqual(target.dx_px, 160.0)
        self.assertAlmostEqual(target.dy_px, -120.0)
        self.assertAlmostEqual(target.dx_norm, 0.5)
        self.assertAlmostEqual(target.dy_norm, -0.5)

    def test_only_drone_and_bird_reach_independent_trackers(self):
        def frame():
            roi = hailo.HailoROI(hailo.HailoBBox(0.0, 0.0, 1.0, 1.0))
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.1, 0.1, 0.2, 0.3), 9, "person", 0.95
                )
            )
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.40, 0.30, 0.10, 0.20), 0, "drone", 0.85
                )
            )
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.402, 0.302, 0.10, 0.20), 0, "drone", 0.40
                )
            )
            roi.add_object(
                hailo.HailoDetection(
                    hailo.HailoBBox(0.70, 0.20, 0.12, 0.16), 1, "bird", 0.80
                )
            )
            return roi

        first = self.tracker.process(frame(), 640, 480)
        second = self.tracker.process(frame(), 640, 480)
        self.assertEqual(first.raw_count, 2)
        self.assertEqual(second.raw_count, 2)
        self.assertEqual({target.label for target in second.targets}, {"DRONE", "BIRD"})
        self.assertEqual(second.active_id, next(
            target.track_id for target in second.targets if target.label == "DRONE"
        ))

    def test_uart_packets_and_x_inversion(self):
        self.assertEqual(
            pack_target_packet(PACKET_TRACKING, 120, -30),
            struct.pack("<Bhh", 0xFF, 120, -30),
        )
        fake = FakeSerial()
        link = TargetUart(True, "/dev/null", 115200, invert_x=True)
        link.serial = fake
        wire = link.send_target(120.9, -30.9, locked=False)
        self.assertEqual(wire, (-120, -30))
        self.assertEqual(fake.packets[-1], struct.pack("<Bhh", 0xFF, -120, -30))
        link.send_target(10.9, -20.9, locked=True)
        self.assertEqual(fake.packets[-1], struct.pack("<Bhh", 0xFE, -10, -20))
        link.send_no_target()
        self.assertEqual(fake.packets[-1], struct.pack("<Bhh", 0xFF, 0, 0))
        self.assertEqual(PACKET_LOCKED, 0xFE)


if __name__ == "__main__":
    unittest.main()
