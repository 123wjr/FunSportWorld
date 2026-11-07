"""Offline geometry invariants without temporary directories or network."""
import math
import unittest
from funsport.track.route_metrics import EARTH_RADIUS_M, route_overlap

def points(*xy):
    scale = EARTH_RADIUS_M * math.pi / 180
    return [(y / scale, x / scale) for x, y in xy]

class RouteOverlapTests(unittest.TestCase):
    def test_first_pass_and_crossings(self):
        for path in (points((0, 0), (100, 0)),
                     points((-10, 0), (10, 0), (0, -10), (0, 10))):
            self.assertAlmostEqual(route_overlap(path)["repeated_m"], 0)

    def test_reverse_and_three_passes(self):
        twice = route_overlap(points((0, 0), (100, 0), (0, 0)))
        thrice = route_overlap(points((0, 0), (100, 0), (0, 0), (100, 0)))
        self.assertAlmostEqual(twice["repeat_ratio"], .5)
        self.assertAlmostEqual(thrice["repeat_ratio"], 2 / 3)

    def test_partial_overlap_and_union(self):
        result = route_overlap(points((0, 0), (100, 0), (40, 0), (80, 0)))
        self.assertAlmostEqual(result["length_m"], 200)
        self.assertAlmostEqual(result["repeated_m"], 100)

    def test_collinear_segmentation_and_reversal(self):
        sparse = points((0, 0), (100, 0), (40, 0), (80, 0))
        dense = points((0, 0), (20, 0), (100, 0), (70, 0), (40, 0), (80, 0))
        for path in (dense, list(reversed(sparse))):
            for key, value in route_overlap(sparse).items():
                self.assertAlmostEqual(route_overlap(path)[key], value)

    def test_parallel_roads_and_tolerance_sensitivity(self):
        path = points((0, 0), (100, 0), (100, 2), (0, 2))
        self.assertAlmostEqual(route_overlap(path)["repeated_m"], 0)
        close = points((0, 0), (100, 0), (100, .5), (0, .5))
        self.assertAlmostEqual(route_overlap(close)["repeated_m"], 100)
        self.assertAlmostEqual(route_overlap(close, tolerance=.1)["repeated_m"], 0)

    def test_empty_and_degenerate(self):
        for path in ([], [(0, 0)], [(0, 0)] * 4):
            self.assertEqual(route_overlap(path), dict(length_m=0, repeated_m=0, repeat_ratio=0))

    def test_invalid_tolerance(self):
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                route_overlap([], tolerance=value)

    def test_near_collinear_subdivision_reversal_and_start_rotation(self):
        sparse = points((0, 0), (100, 0), (0, 3), (0, 0))
        dense = points(*([(x, 0) for x in range(0, 101, 10)] + [(0, 3), (0, 0)]))
        expected = route_overlap(sparse)
        self.assertGreater(expected["repeated_m"], 30)
        for path in (dense, list(reversed(dense)), dense[4:-1] + dense[:5]):
            for key, value in expected.items():
                self.assertAlmostEqual(route_overlap(path)[key], value, places=6)

    def test_nonfinite_coordinates_are_rejected(self):
        with self.assertRaises(ValueError):
            route_overlap([(0, 0), (float("nan"), 0)])

