"""Exercise the actual middleman-to-following protocol without ROS or hardware."""

import json
from pathlib import Path
import sys
import unittest

# The repo runs core scripts directly; import the same implementations here.
CORE_DIR = Path(__file__).resolve().parents[3] / "core"
sys.path.insert(0, str(CORE_DIR))
from middleman import Middleman, Mode
from middleman_node import mode_payload

from mode_bridge import FollowingModeBridge, ModeCommandError
from state_machine import (FollowingState, FollowingStateMachine,
                           PerceptionFrame, PersonObservation)


class VoiceFollowingTests(unittest.TestCase):
    def setUp(self):
        self.middleman = Middleman()
        self.machine = FollowingStateMachine()
        self.bridge = FollowingModeBridge(self.machine)

    def speech(self, text):
        decision = self.middleman.handle(text)
        self.assertIsNotNone(decision)
        payload = mode_payload(decision.mode.value, decision.target, decision.intent,
                               decision.stop, decision.request_id)
        return self.bridge.handle_command(json.dumps(payload)), payload

    def observe(self, stamp):
        people = (PersonObservation(8, 3, 0), PersonObservation(7, 2, 0.2))
        return self.machine.update(stamp, PerceptionFrame(stamp, people))

    def start(self):
        output, command = self.speech("pete start following")
        self.assertEqual(output.state, FollowingState.ACQUIRING)
        for stamp in (0.0, 0.25, 0.5):
            output = self.observe(stamp)
        self.assertEqual(output.state, FollowingState.FOLLOWING)
        return output, command

    def test_starts_idle_and_voice_selects_nearest(self):
        self.assertEqual(self.machine.state, FollowingState.IDLE)
        output, command = self.start()
        status = self.bridge.status_payload(output)
        self.assertEqual(status["request_id"], command["request_id"])
        self.assertEqual(status["target_id"], 7)
        self.assertEqual(status["state"], "FOLLOWING")

    def test_stop_command_is_immediate_and_does_not_say_arrived(self):
        self.start()
        output, command = self.speech("pete stop following")
        self.assertTrue(command["stop"])
        self.assertEqual(command["mode"], "stop")
        self.assertEqual(output.state, FollowingState.IDLE)
        self.assertIsNone(output.target_id)
        self.assertEqual(self.middleman.mode, Mode.IDLE)

    def test_mode_switch_disables_following(self):
        for text, mode in (("pete manual mode", "manual"),
                           ("pete take me to the kitchen", "location"),
                           ("pete help", "help")):
            with self.subTest(text=text):
                self.setUp()
                self.start()
                output, command = self.speech(text)
                self.assertEqual(command["mode"], mode)
                self.assertEqual(output.state, FollowingState.IDLE)
                self.assertIsNone(output.target_id)

    def test_target_loss_round_trip_sets_middleman_idle_and_returns_idle_command(self):
        self.start()
        for stamp in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5):
            output = self.machine.update(stamp, PerceptionFrame(stamp))
        self.assertEqual(output.event, "target_lost")
        status = json.loads(json.dumps(self.bridge.status_payload(output)))
        decision = self.middleman.following_status(status)
        self.assertEqual(self.middleman.mode, Mode.IDLE)
        self.assertIn("lost the person", decision.reply)
        self.assertNotIn("Arrived", decision.reply)
        idle = mode_payload(decision.mode.value, request_id=decision.request_id)
        self.assertEqual(self.bridge.handle_command(json.dumps(idle)).state, FollowingState.IDLE)
        self.assertIsNone(self.middleman.following_status(status))

    def test_perception_fault_and_stale_input_update_middleman(self):
        for healthy, now, event in ((False, 0.6, "input_fault"), (True, 1.1, "input_stale")):
            with self.subTest(event=event):
                self.setUp()
                self.start()
                output = self.machine.update(now, input_healthy=healthy)
                self.assertEqual(output.event, event)
                decision = self.middleman.following_status(self.bridge.status_payload(output))
                self.assertEqual(decision.mode, Mode.IDLE)
                self.assertIn("Following stopped", decision.reply)

    def test_nav2_and_transform_failures_are_correlated_and_spoken(self):
        for event in ("navigation_unavailable", "navigation_failed", "transform_unavailable",
                      "navigation_cancel_failed"):
            with self.subTest(event=event):
                self.setUp()
                _, command = self.start()
                status = dict(request_id=command["request_id"], state="IDLE", event=event)
                decision = self.middleman.following_status(status)
                self.assertEqual(decision.mode, Mode.IDLE)
                self.assertNotIn("Arrived", decision.reply)

    def test_old_failure_cannot_cancel_new_following_request(self):
        _, old = self.start()
        stale_status = dict(request_id=old["request_id"], state="IDLE", event="target_lost")
        self.speech("pete stop")
        _, new = self.speech("pete start following")
        self.assertNotEqual(old["request_id"], new["request_id"])
        self.assertIsNone(self.middleman.following_status(stale_status))
        self.assertEqual(self.middleman.mode, Mode.FOLLOW)

    def test_late_failure_cannot_cancel_another_mode(self):
        _, old = self.start()
        self.speech("pete manual mode")
        status = dict(request_id=old["request_id"], state="IDLE", event="target_lost")
        self.assertIsNone(self.middleman.following_status(status))
        self.assertEqual(self.middleman.mode, Mode.MANUAL)

    def test_duplicate_command_does_not_restart_after_timeout_or_stop(self):
        _, command = self.start()
        for stamp in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5):
            self.machine.update(stamp, PerceptionFrame(stamp))
        duplicate = self.bridge.handle_command(json.dumps(command))
        self.assertEqual(duplicate.state, FollowingState.IDLE)
        self.assertIsNone(duplicate.event)
        self.speech("pete stop")
        self.assertEqual(self.bridge.handle_command(json.dumps(command)).state, FollowingState.IDLE)

    def test_new_follow_command_while_active_retains_selected_id(self):
        self.start()
        result, _ = self.speech("pete start following")
        self.assertEqual(result.target_id, 7)
        self.assertEqual(result.state, FollowingState.FOLLOWING)

    def test_named_contact_is_rejected_instead_of_following_nearest(self):
        output, _ = self.speech("pete follow alice")
        self.assertEqual(output.event, "unsupported_target")
        self.assertEqual(output.state, FollowingState.IDLE)
        decision = self.middleman.following_status(self.bridge.status_payload(output))
        self.assertEqual(decision.mode, Mode.IDLE)
        self.assertIn("named contact", decision.reply)

    def test_manual_session_status_does_not_change_middleman(self):
        self.middleman.handle("pete start following")
        output = self.bridge.set_enabled(True)
        self.assertIsNone(self.bridge.status_payload(output)["request_id"])
        self.assertIsNone(self.middleman.following_status(self.bridge.status_payload(output)))

    def test_manual_disable_of_voice_session_updates_middleman(self):
        self.start()
        output = self.bridge.set_enabled(False)
        self.assertEqual(self.middleman.following_status(self.bridge.status_payload(output)).mode, Mode.IDLE)

    def test_malformed_command_disables_and_reports_correlated_failure(self):
        bad_commands = ("{", "[]", '{"mode":"follow","stop":"false"}',
                        '{"mode":"follow","stop":false,"target":""}')
        for text in bad_commands:
            with self.subTest(text=text):
                self.setUp()
                self.start()
                with self.assertRaises(ModeCommandError):
                    self.bridge.handle_command(text)
                output = self.bridge.reject_command()
                self.assertEqual(output.state, FollowingState.IDLE)
                self.assertEqual(output.event, "command_invalid")
                self.assertEqual(self.middleman.following_status(self.bridge.status_payload(output)).mode, Mode.IDLE)

    def test_progress_and_malformed_status_do_not_end_following(self):
        _, command = self.start()
        for status in (None, [], {}, dict(request_id=None, state="IDLE", event="target_lost"),
                       dict(request_id=command["request_id"], state="FOLLOWING", event="target_acquired"),
                       dict(request_id=command["request_id"], state="IDLE", event=[])):
            with self.subTest(status=status):
                self.assertIsNone(self.middleman.following_status(status))
                self.assertEqual(self.middleman.mode, Mode.FOLLOW)


if __name__ == "__main__":
    unittest.main()
