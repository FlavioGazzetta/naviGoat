"""No-camera checks: run `python -m unittest -v test_tracker.py`."""
import unittest
from tracker2d import Tracker2D


class TrackerTests(unittest.TestCase):
    def test_persistent_id_and_velocity(self):
        tr = Tracker2D(gate_m=1., max_age_s=1., min_hits=2)
        first_id, _ = tr.update([(2., 0.)], 0.)
        for step in range(1, 10):
            ids, tracks = tr.update([(2. + 0.05 * step, 0.)], 0.1 * step)
            self.assertEqual(ids, first_id)
        self.assertTrue(tracks[0].confirmed)
        self.assertGreater(tracks[0].vx, 0.25)
        self.assertLess(abs(tracks[0].vy), 0.1)

    def test_short_miss_reassociation(self):
        tr = Tracker2D(gate_m=1., max_age_s=0.8, min_hits=2)
        ids0, _ = tr.update([(2., 0.)], 0.)
        tr.update([(2.1, 0.)], 0.2)
        _, missed = tr.update([], 0.3)
        self.assertFalse(missed[0].observed)
        ids1, _ = tr.update([(2.25, 0.)], 0.5)
        self.assertEqual(ids0, ids1)

    def test_long_absence_gets_new_id(self):
        tr = Tracker2D(gate_m=1., max_age_s=0.5)
        ids0, _ = tr.update([(2., 0.)], 0.)
        tr.update([], 0.6)
        ids1, _ = tr.update([(2., 0.)], 0.7)
        self.assertNotEqual(ids0, ids1)

    def test_two_people(self):
        tr = Tracker2D(gate_m=1.)
        ids0, _ = tr.update([(2., 1.), (4., -1.)], 0.)
        ids1, _ = tr.update([(2.1, 1.), (3.9, -1.)], 0.2)
        self.assertEqual(ids0, ids1)

    def test_out_of_order_rejected(self):
        tr = Tracker2D()
        tr.update([(2., 0.)], 3.)
        ids, _ = tr.update([(3., 0.)], 2.)
        self.assertEqual(ids, [None])
        self.assertEqual(len(tr.tracks), 1)


if __name__ == "__main__":
    unittest.main()
