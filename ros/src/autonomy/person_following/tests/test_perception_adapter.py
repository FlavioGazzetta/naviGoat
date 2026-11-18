"""Tracking JSON and adapter-to-state-machine checks without ROS."""

import json
import math
import unittest

from perception_adapter import COORDINATE_CONVENTION, PerceptionAdapter, PerceptionError
from state_machine import FollowingState, FollowingStateMachine


def payload(stamp=1000.0, tracks=None):
    # Same fields emitted by Enid's asdict(TrackView) publisher.
    return {
        "stamp_sec": stamp,
        "coordinate_convention": COORDINATE_CONVENTION,
        "tracks": [dict(id=7, x=2.0, y=0.3, vx=0.1, vy=0.0, hits=3,
                        age_since_seen_s=0.0, observed=True, confirmed=True)]
        if tracks is None else tracks,
    }


class PerceptionAdapterTests(unittest.TestCase):
    def parse(self, data, adapter=None, source_now=1000.15, monotonic_now=10.0):
        adapter = adapter if adapter is not None else PerceptionAdapter()
        return adapter.parse(json.dumps(data), source_now_s=source_now,
                             monotonic_now_s=monotonic_now)

    def test_actual_publisher_schema_and_clock_mapping(self):
        result = self.parse(payload())
        self.assertAlmostEqual(result.timestamp_s, 9.85)
        self.assertEqual(result.source_timestamp_s, 1000.0)
        person = result.people[0]
        self.assertEqual(person.track_id, 7)
        self.assertEqual((person.forward_m, person.lateral_m), (2.0, 0.3))
        self.assertTrue(person.observed)
        self.assertTrue(person.confirmed)

    def test_latency_is_preserved_instead_of_using_arrival_time(self):
        result = self.parse(payload(), source_now=1000.4)
        self.assertAlmostEqual(10.0 - result.timestamp_s, 0.4)

    def test_empty_frame_and_predictions_remain_distinct(self):
        self.assertEqual(self.parse(payload(tracks=[])).people, ())
        data = payload()
        data["tracks"][0].update(observed=False, age_since_seen_s=0.3)
        self.assertFalse(self.parse(data).people[0].observed)

    def test_unconfirmed_track_does_not_become_confirmed(self):
        data = payload()
        data["tracks"][0]["confirmed"] = False
        self.assertFalse(self.parse(data).people[0].confirmed)

    def test_duplicate_or_out_of_order_frame_is_not_a_new_sighting(self):
        adapter = PerceptionAdapter()
        self.parse(payload(), adapter)
        self.assertIsNone(self.parse(payload(), adapter, source_now=1000.25, monotonic_now=10.1))
        self.assertIsNone(self.parse(payload(999.9), adapter, source_now=1000.25, monotonic_now=10.1))

    def test_rejects_bad_json_and_top_level_values(self):
        for text in ("{", "null", "[]", '"text"'):
            with self.subTest(text=text), self.assertRaises(PerceptionError):
                PerceptionAdapter().parse(text, source_now_s=1000.15, monotonic_now_s=10)

    def test_coordinate_convention_must_match(self):
        for convention in (None, "map", "x_right,y_forward"):
            data = payload()
            data["coordinate_convention"] = convention
            with self.subTest(convention=convention), self.assertRaises(PerceptionError):
                self.parse(data)

    def test_rejects_missing_fields_and_wrong_types(self):
        for field, value in (("stamp_sec", None), ("stamp_sec", True),
                             ("stamp_sec", "1000"), ("tracks", {}), ("tracks", [None])):
            data = payload()
            data[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(PerceptionError):
                self.parse(data)

    def test_rejects_invalid_track_fields(self):
        for field, value in (("id", 0), ("id", True), ("id", "7"),
                             ("observed", "true"), ("confirmed", None),
                             ("x", 0), ("x", -1), ("x", math.nan),
                             ("x", 10**500), ("y", math.inf), ("y", "0.1"),
                             ("age_since_seen_s", -1), ("age_since_seen_s", 0.2)):
            data = payload()
            data["tracks"][0][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(PerceptionError):
                self.parse(data)

    def test_duplicate_ids_are_rejected(self):
        data = payload()
        data["tracks"].append(dict(data["tracks"][0]))
        with self.assertRaises(PerceptionError):
            self.parse(data)

    def test_stale_future_and_nonfinite_timestamps_rejected(self):
        for stamp in (999.0, 1001.0, -1, math.inf, math.nan):
            with self.subTest(stamp=stamp), self.assertRaises(PerceptionError):
                self.parse(payload(stamp))

    def test_invalid_frame_does_not_poison_next_valid_frame(self):
        adapter = PerceptionAdapter()
        data = payload()
        data["tracks"][0]["x"] = math.nan
        with self.assertRaises(PerceptionError):
            self.parse(data, adapter)
        self.assertIsNotNone(self.parse(payload(), adapter))

    def test_backward_monotonic_mapping_is_rejected(self):
        adapter = PerceptionAdapter()
        self.parse(payload(), adapter)
        with self.assertRaises(PerceptionError):
            self.parse(payload(1000.1), adapter, source_now=1000.3, monotonic_now=9.9)

    def test_invalid_clock_and_age_configuration_rejected(self):
        for max_age in (0, -1, math.inf, True):
            with self.subTest(max_age=max_age), self.assertRaises(PerceptionError):
                PerceptionAdapter(max_age)
        with self.assertRaises(PerceptionError):
            self.parse(payload(), source_now=math.nan)


class ConnectedPipelineTests(unittest.TestCase):
    def setUp(self):
        self.adapter = PerceptionAdapter()
        self.machine = FollowingStateMachine()
        self.machine.enable()

    def send(self, elapsed, *, predicted=False, empty=False, track_id=7):
        data = payload(1000.0 + elapsed)
        data["tracks"][0]["id"] = track_id
        if predicted:
            data["tracks"][0].update(observed=False, age_since_seen_s=elapsed - 0.5)
        if empty:
            data["tracks"] = []
        # 150 ms processing latency preserved all the way into the state machine.
        now = 10.0 + elapsed + 0.15
        frame = self.adapter.parse(json.dumps(data), source_now_s=1000.0 + elapsed + 0.15,
                                   monotonic_now_s=now)
        return self.machine.update(now, frame)

    def test_real_schema_flows_through_acquisition_loss_and_timeout(self):
        for elapsed in (0.0, 0.25, 0.5):
            following = self.send(elapsed)
        self.assertEqual(following.state, FollowingState.FOLLOWING)
        self.assertEqual(following.target_id, 7)
        lost = self.send(0.75, predicted=True)
        self.assertEqual(lost.state, FollowingState.TEMPORARILY_LOST)
        still_missing = self.send(1.0, predicted=True)
        self.assertEqual(still_missing.state, FollowingState.TEMPORARILY_LOST)
        self.assertEqual(still_missing.target_id, 7)
        empty = self.send(1.25, empty=True)
        self.assertEqual(empty.state, FollowingState.TEMPORARILY_LOST)
        for elapsed in (1.5, 1.75, 2.0):
            self.send(elapsed, empty=True)
        timeout = self.send(2.5, empty=True)
        self.assertEqual(timeout.event, "target_lost")
        self.assertEqual(self.send(2.75).state, FollowingState.IDLE)

    def test_different_id_does_not_resume_and_absent_stream_triggers_watchdog(self):
        self.send(0)
        self.send(0.25)
        result = self.send(0.5, track_id=8)
        self.assertEqual(result.target_id, 7)
        self.assertEqual(result.state, FollowingState.TEMPORARILY_LOST)
        result = self.machine.update(11.1)
        self.assertEqual(result.event, "input_stale")
        self.assertEqual(result.state, FollowingState.IDLE)


if __name__ == "__main__":
    unittest.main()
