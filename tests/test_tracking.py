import unittest

import hailo

from tracking import ClassAwareByteTracker


def make_tracker(**kwargs):
    return ClassAwareByteTracker(
        frame_rate=30,
        class_labels=("DRONE", "BIRD"),
        priority_labels=("DRONE", "BIRD"),
        sticky_labels=(),
        track_buffer=15,
        **kwargs,
    )


def frame(*objects):
    """objects: (label, x, y, confidence) with a 0.04 x 0.04 box at (x, y)."""
    roi = hailo.HailoROI(hailo.HailoBBox(0, 0, 1, 1))
    for label, x, y, confidence in objects:
        roi.add_object(hailo.HailoDetection(
            hailo.HailoBBox(x, y, 0.04, 0.04), 0, label, confidence
        ))
    return roi


class ClassVotingTrackerTests(unittest.TestCase):
    def test_single_drone_frame_keeps_bird_track_and_label(self):
        tracker = make_tracker()
        labels = ["bird"] * 10 + ["drone"] + ["bird"] * 3
        results = [tracker.process(frame((label, 0.40, 0.40, 0.70)), 640, 640)
                   for label in labels]

        ids = {result.targets[0].track_id for result in results[1:]}
        self.assertEqual(len(ids), 1)
        flipped = results[10].targets[0]
        self.assertEqual(flipped.raw_label, "DRONE")
        self.assertEqual(flipped.label, "BIRD")

    def test_drone_needs_ten_votes_before_it_is_called_drone(self):
        tracker = make_tracker()
        results = [tracker.process(frame(("drone", 0.40, 0.40, 0.70)), 640, 640)
                   for _ in range(12)]

        self.assertEqual(results[8].targets[0].label, "BIRD")  # 9 votes
        self.assertEqual(results[9].targets[0].label, "DRONE")  # 10 votes
        self.assertEqual(results[11].targets[0].label, "DRONE")

    def test_short_gap_keeps_the_same_track_id(self):
        tracker = make_tracker()
        for _ in range(5):
            before = tracker.process(frame(("bird", 0.40, 0.40, 0.70)), 640, 640)
        for _ in range(5):
            tracker.process(frame(), 640, 640)
        after = tracker.process(frame(("bird", 0.40, 0.40, 0.70)), 640, 640)

        self.assertEqual(after.targets[0].track_id, before.targets[0].track_id)

    def test_confirmed_drone_is_preferred_over_bird(self):
        tracker = make_tracker()
        for _ in range(12):
            result = tracker.process(frame(
                ("drone", 0.70, 0.70, 0.70),  # far from centre
                ("bird", 0.48, 0.48, 0.70),   # near centre
            ), 640, 640)

        active = next(t for t in result.targets if t.track_id == result.active_id)
        self.assertEqual(active.label, "DRONE")

    def test_unconfirmed_drone_does_not_pull_focus_from_centre_bird(self):
        tracker = make_tracker()
        for _ in range(4):
            result = tracker.process(frame(
                ("drone", 0.70, 0.70, 0.70),
                ("bird", 0.48, 0.48, 0.70),
            ), 640, 640)

        active = next(t for t in result.targets if t.track_id == result.active_id)
        self.assertEqual(active.raw_label, "BIRD")


if __name__ == "__main__":
    unittest.main()
