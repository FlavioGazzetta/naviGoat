"""Action lifecycle checks with real futures and a deterministic fake transport."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest

from nav2_goal_client import Nav2GoalClient


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.status = 2
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return self.cancel_future

    def acknowledge_cancel(self, accepted=True):
        self.cancel_future.set_result(SimpleNamespace(return_code=0 if accepted else 1,
                                                       goals_canceling=[object()] if accepted else []))

    def finish(self, status):
        self.status = status
        self.result_future.set_result(SimpleNamespace(status=status))


class Transport:
    def __init__(self):
        self.ready = True
        self.sent = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, message):
        future = Future()
        self.sent.append((message, future))
        return future

    def accept(self, index=0, accepted=True):
        handle = Handle(accepted)
        self.sent[index][1].set_result(handle)
        return handle


class Nav2GoalClientTests(unittest.TestCase):
    def setUp(self):
        self.transport = Transport()
        self.client = Nav2GoalClient(self.transport, lambda goal: ("pose", goal))

    def start(self):
        self.client.request_goal("first", "session-1", 0)
        return self.transport.accept()

    def test_sends_through_transport_and_success_does_not_end_following(self):
        handle = self.start()
        self.assertEqual(self.transport.sent[0][0], ("pose", "first"))
        handle.finish(4)
        self.assertEqual(self.client.events, [("session-1", "goal_reached")])
        self.assertFalse(self.client.busy)
        self.client.tick(0.5)
        self.assertEqual(len(self.transport.sent), 1)

    def test_new_goal_preempts_and_old_cancelled_result_is_ignored(self):
        old = self.start()
        self.client.request_goal("second", "session-1", 0.5)
        new = self.transport.accept(1)
        old.finish(5)
        self.assertEqual(self.client.events, [])
        new.finish(4)
        self.assertEqual(self.client.events, [("session-1", "goal_reached")])

    def test_pending_updates_coalesce_to_latest_destination(self):
        self.client.request_goal("first", "session-1", 0)
        self.client.request_goal("second", "session-1", 0.1)
        self.client.request_goal("latest", "session-1", 0.2)
        self.assertEqual(len(self.transport.sent), 1)
        self.transport.accept()
        self.client.tick(0.3)
        self.assertEqual(self.transport.sent[1][0], ("pose", "latest"))

    def test_stop_before_acceptance_cancels_late_handle(self):
        self.client.request_goal("first", "session-1", 0)
        self.client.cancel(0.1)
        handle = self.transport.accept()
        self.assertEqual(handle.cancels, 1)
        handle.acknowledge_cancel()
        self.assertTrue(self.client.busy)  # acknowledgement alone is not a terminal result
        handle.finish(5)
        self.assertFalse(self.client.busy)
        self.assertEqual(self.client.events, [])

    def test_new_session_waits_for_previous_cancellation_to_complete(self):
        old = self.start()
        self.client.request_goal("new-session", "session-2", 0.1)
        self.assertEqual(old.cancels, 1)
        self.assertEqual(len(self.transport.sent), 1)
        old.acknowledge_cancel()
        self.client.tick(0.2)
        self.assertEqual(len(self.transport.sent), 1)
        old.finish(5)
        self.client.tick(0.3)
        self.assertEqual(self.transport.sent[1][0], ("pose", "new-session"))

    def test_stop_cancels_all_owned_goals_once_and_sends_no_queued_goal(self):
        old = self.start()
        self.client.request_goal("second", "session-1", 0.2)
        new = self.transport.accept(1)
        self.client.cancel(0.3)
        self.client.cancel(0.4)
        self.client.tick(0.5)
        self.assertEqual((old.cancels, new.cancels), (1, 1))
        self.assertEqual(len(self.transport.sent), 2)

    def test_unavailable_or_rejected_navigation_reports_failure(self):
        self.transport.ready = False
        self.client.request_goal("first", "session-1", 0)
        self.assertEqual(self.client.events, [("session-1", "navigation_unavailable")])
        self.setUp()
        self.client.request_goal("first", "session-1", 0)
        self.transport.accept(accepted=False)
        self.assertEqual(self.client.events, [("session-1", "navigation_failed")])

    def test_aborted_goal_stops_session(self):
        handle = self.start()
        handle.finish(6)
        self.assertEqual(self.client.events, [("session-1", "navigation_failed")])

    def test_goal_response_timeout_still_cancels_late_acceptance(self):
        self.client.request_goal("first", "session-1", 0)
        self.client.tick(2.1)
        self.assertEqual(self.client.events, [("session-1", "navigation_failed")])
        handle = self.transport.accept()
        self.assertEqual(handle.cancels, 1)

    def test_cancellation_rejection_blocks_new_session_and_reports_it(self):
        old = self.start()
        self.client.request_goal("new", "session-2", 0.1)
        old.acknowledge_cancel(accepted=False)
        self.assertEqual(self.client.events, [("session-2", "navigation_cancel_failed")])
        self.client.tick(0.2)
        self.assertEqual(len(self.transport.sent), 1)

    def test_accepted_cancellation_without_terminal_result_times_out(self):
        handle = self.start()
        self.client.cancel(0.1)
        handle.acknowledge_cancel()
        self.client.tick(2.2)
        self.assertEqual(self.client.events, [("session-1", "navigation_cancel_failed")])

    def test_send_exception_is_a_failure(self):
        def broken(_message):
            raise RuntimeError("transport down")
        self.transport.send_goal_async = broken
        self.client.request_goal("first", "session-1", 0)
        self.assertEqual(self.client.events, [("session-1", "navigation_failed")])

    def test_delayed_acceptance_does_not_burst_goal_publications(self):
        self.client = Nav2GoalClient(self.transport, lambda goal: goal, min_interval_s=0.5)
        self.client.request_goal("first", "session-1", 0)
        self.client.request_goal("second", "session-1", 0.1)
        self.transport.accept()
        self.client.tick(0.2)
        self.assertEqual(len(self.transport.sent), 1)
        self.client.tick(0.5)
        self.assertEqual(len(self.transport.sent), 2)

    def test_new_session_does_not_wait_forever_for_old_lost_response(self):
        self.client.request_goal("first", "session-1", 0)
        self.client.tick(2.1)
        self.client.events.clear()
        self.client.request_goal("new", "session-2", 2.2)
        self.client.tick(4.3)
        self.assertEqual(self.client.events, [("session-2", "navigation_cancel_failed")])

    def test_invalid_timeouts_are_rejected(self):
        with self.assertRaises(ValueError):
            Nav2GoalClient(self.transport, lambda goal: goal, timeout_s=0)


if __name__ == "__main__":
    unittest.main()
