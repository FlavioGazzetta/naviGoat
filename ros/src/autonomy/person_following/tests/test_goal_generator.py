import math
import unittest

from goal_generator import FollowingGoal, GoalConfig, GoalGenerator, camera_point_on_map


class GoalGeneratorTests(unittest.TestCase):
    def test_stands_one_metre_short_of_person_and_faces_them(self):
        action, goal = GoalGenerator().update((0, 0), (3, 0), 0)
        self.assertEqual(action, "send")
        self.assertEqual(goal, FollowingGoal(2, 0, 0))

    def test_standoff_with_offset_robot_and_diagonal_target(self):
        _, goal = GoalGenerator().update((2, 3), (5, 7), 0)
        self.assertAlmostEqual(math.hypot(goal.x - 5, goal.y - 7), 1)
        self.assertAlmostEqual(goal.yaw, math.atan2(4, 3))

    def test_nearby_person_requests_hold_without_retreat(self):
        for target in ((0, 0), (0.5, 0), (1.15, 0), (0, 1)):
            with self.subTest(target=target):
                self.assertEqual(GoalGenerator().update((0, 0), target, 0), ("hold", None))

    def test_goal_updates_are_limited_to_two_hz(self):
        planner = GoalGenerator()
        planner.update((0, 0), (3, 0), 0)
        self.assertEqual(planner.update((0, 0), (4, 0), 0.49), ("wait", None))
        self.assertEqual(planner.update((0, 0), (4, 0), 0.5)[0], "send")

    def test_small_jitter_and_unchanged_world_goal_do_not_reissue(self):
        planner = GoalGenerator()
        planner.update((0, 0), (3, 0), 0)
        self.assertEqual(planner.update((0.5, 0), (3.01, 0), 0.5), ("wait", None))
        self.assertEqual(planner.update((1, 0), (3, 0), 1), ("wait", None))

    def test_heading_change_can_trigger_an_update(self):
        planner = GoalGenerator(GoalConfig(movement_threshold_m=100))
        planner.update((0, 0), (3, 0), 0)
        self.assertEqual(planner.update((0, 0), (0, 3), 0.5)[0], "send")

    def test_heading_wrap_does_not_count_as_full_rotation(self):
        planner = GoalGenerator()
        planner.update((0, 0), (-3, 0.01), 0)
        self.assertEqual(planner.update((0, 0), (-3, -0.01), 0.5), ("wait", None))

    def test_hold_then_return_can_issue_same_goal_again(self):
        planner = GoalGenerator()
        planner.update((0, 0), (3, 0), 0)
        planner.update((2.1, 0), (3, 0), 0.5)
        self.assertEqual(planner.update((0, 0), (3, 0), 1)[0], "send")

    def test_reset_allows_new_session_to_send_immediately(self):
        planner = GoalGenerator()
        planner.update((0, 0), (3, 0), 0)
        planner.reset()
        self.assertEqual(planner.update((0, 0), (3, 0), 0.1)[0], "send")

    def test_invalid_positions_times_and_configuration_are_rejected(self):
        planner = GoalGenerator()
        planner.update((0, 0), (3, 0), 1)
        for now in (0, math.nan, math.inf):
            with self.subTest(now=now), self.assertRaises(ValueError):
                planner.update((0, 0), (3, 0), now)
        with self.assertRaises(ValueError):
            GoalGenerator().update((math.nan, 0), (3, 0), 0)
        for settings in ({"distance_m": 0}, {"update_hz": math.nan}, {"tolerance_m": 1}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                GoalConfig(**settings)

    def test_camera_mount_translation_and_yaw_are_applied(self):
        q = math.sqrt(0.5)
        x, y = camera_point_on_map(2, 0.3, (1, 4, 0, 0, q, q))
        self.assertAlmostEqual(x, 0.7)
        self.assertAlmostEqual(y, 6)

    def test_missing_vertical_measurement_rejects_tilted_camera(self):
        with self.assertRaises(ValueError):
            camera_point_on_map(2, 0, (0, 0, 0, math.sin(0.1), 0, math.cos(0.1)))
        with self.assertRaises(ValueError):
            camera_point_on_map(2, 0, (0, 0, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
