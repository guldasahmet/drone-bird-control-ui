import csv
import struct
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import hailo

from dual_runtime import (
    DualVisionRuntime,
    HandoffController,
    TraceLog,
    limit_step,
    lock_with_hysteresis,
    add_overlay_objects,
    target_pixel_errors,
)
from uart import TargetUart
from dual_camera_pipeline import hailortpp_config
from model_profile import load_active_profile


class HandoffTests(unittest.TestCase):
    def test_active_phone_overlay_contains_boxes_and_aim(self):
        roi = hailo.HailoROI(hailo.HailoBBox(0, 0, 1, 1))
        phone = SimpleNamespace(
            track_id=7,
            x1=0.1, y1=0.2, x2=0.3, y2=0.4,
            center_x=0.2, center_y=0.3, confidence=0.8,
        )
        add_overlay_objects(
            roi, 640, 640, SimpleNamespace(targets=(phone,), active_id=7)
        )
        self.assertEqual(len(roi.get_objects_typed(hailo.HAILO_DETECTION)), 2)
        self.assertEqual(len(roi.get_objects_typed(hailo.HAILO_LANDMARKS)), 1)

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

    def test_hailortpp_labels_get_background_placeholder(self):
        config = hailortpp_config(
            {"detection_threshold": 0.1, "max_boxes": 100, "labels": ["DRONE", "BIRD"]}
        )
        # Model class i -> class_id i + 1 -> labels[i + 1].
        self.assertEqual(config["labels"], ["unlabeled", "DRONE", "BIRD"])
        self.assertEqual(config["max_boxes"], 100)

    def test_gs_errors_are_scaled_to_hq_pixels(self):
        runtime = DualVisionRuntime(lambda _slot, _widget: None)
        runtime.profile = replace(runtime.profile, camera_flip_vertical=False,
                                  camera_error_scale=((0.5, 0.62), (1.0, 1.0)))
        target = SimpleNamespace(center_x=0.75, center_y=0.25)  # (+160, -160) px
        self.assertEqual(runtime._control_errors(0, target), (80, -99))
        self.assertEqual(runtime._control_errors(1, target), (160, -160))

    def test_lock_hysteresis_enters_at_25_and_releases_at_50(self):
        self.assertFalse(lock_with_hysteresis(30, 0, False, 25, 50))
        self.assertTrue(lock_with_hysteresis(24, -24, False, 25, 50))
        self.assertTrue(lock_with_hysteresis(40, -49, True, 25, 50))
        self.assertFalse(lock_with_hysteresis(50, 0, True, 25, 50))
        self.assertFalse(lock_with_hysteresis(0, -60, True, 25, 50))

    def test_limit_step_ramps_toward_target_and_can_be_disabled(self):
        self.assertEqual(limit_step(0, 250, 30), 30)
        self.assertEqual(limit_step(240, 250, 30), 250)
        self.assertEqual(limit_step(0, -100, 30), -30)
        self.assertEqual(limit_step(0, 250, 0), 250)

    def test_trace_log_writes_header_and_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace = TraceLog(Path(tmp) / "logs" / "trace.csv")
            trace.write("tx", control="HQ", wire_x=-12, wire_y=30, locked=0)
            trace.close()
            trace.write("tx")  # ignored after close
            with open(trace.path, newline="") as file:
                rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["kind"], "tx")
        self.assertEqual(rows[0]["wire_x"], "-12")
        self.assertEqual(rows[0]["camera"], "")

    def test_hold_hq_never_hands_control_to_gs(self):
        controller = HandoffController(hold_hq=True)
        phone = object()
        for frame in range(5):
            self.assertIsNone(controller.observe(0, phone, now=1.0 + frame * 0.02))
        self.assertEqual(controller.confirmation_progress(now=1.08), (0, 2))
        self.assertEqual(controller.snapshot(now=1.08)[0], 1)
        controller.observe(1, phone, now=1.1)
        self.assertIs(controller.snapshot(now=1.1)[1], phone)

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

    def test_custom_profile_uses_class_id_and_configured_label(self):
        profile = replace(load_active_profile(), name="bird",
                          display_name="Kuş", targets=((0, "BIRD"),),
                          priority=("BIRD",), sticky=("BIRD",),
                          match_by="id")
        runtime = DualVisionRuntime(lambda _slot, _widget: None, profile=profile)

        def frame():
            roi = hailo.HailoROI(hailo.HailoBBox(0, 0, 1, 1))
            roi.add_object(hailo.HailoDetection(
                hailo.HailoBBox(0.2, 0.2, 0.2, 0.2), 0, "other label", 0.85
            ))
            roi.add_object(hailo.HailoDetection(
                hailo.HailoBBox(0.6, 0.6, 0.2, 0.2), 67, "cell phone", 0.99
            ))
            return roi

        runtime.trackers[0].process(frame(), 640, 640)
        result = runtime.trackers[0].process(frame(), 640, 640)
        self.assertEqual(len(result.targets), 1)
        self.assertEqual(result.targets[0].label, "BIRD")

    def test_phone_profile_uses_the_working_postprocess_label(self):
        runtime = DualVisionRuntime(lambda _slot, _widget: None)

        def frame():
            roi = hailo.HailoROI(hailo.HailoBBox(0, 0, 1, 1))
            roi.add_object(hailo.HailoDetection(
                hailo.HailoBBox(0.2, 0.2, 0.2, 0.2), 0, "cell phone", 0.85
            ))
            return roi

        runtime.trackers[0].process(frame(), 640, 640)
        result = runtime.trackers[0].process(frame(), 640, 640)
        self.assertEqual(len(result.targets), 1)
        self.assertEqual(result.targets[0].label, "CELL PHONE")

    def test_vertical_display_flip_preserves_the_previous_stm_direction(self):
        runtime = DualVisionRuntime(lambda _slot, _widget: None)
        self.assertTrue(runtime.profile.camera_flip_vertical)
        self.assertTrue(runtime.invert_y)

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
        # Recover original camera coordinates before the reference sign flip.
        raw_target = SimpleNamespace(center_x=0.625, center_y=0.59375)
        flipped_target = SimpleNamespace(center_x=0.625, center_y=0.40625)
        self.assertEqual(
            target_pixel_errors(flipped_target, flip_vertical=True),
            target_pixel_errors(raw_target),
        )
        self.assertEqual(link.send_target(*target_pixel_errors(
            flipped_target, flip_vertical=True)), (-80, -60))
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
