"""Synthetic safeguards for bounded road-corner smoothing."""
import unittest

from funsport.coordinates import distance_m
from tools.road_joint_eval import _point_to_polyline_m
from tools.road_smoothing import rounded_leg


class RoadSmoothingTests(unittest.TestCase):
    def test_preserves_endpoints_and_caps_spatial_offset(self):
        road = [(30, 104), (30, 104.0002), (30.0002, 104.0002)]
        rounded = rounded_leg(road, radius_m=8, max_offset_m=5)
        self.assertEqual((rounded[0], rounded[-1]), (road[0], road[-1]))
        self.assertGreater(len(rounded), len(road))
        self.assertLessEqual(max(_point_to_polyline_m(p, road) for p in rounded), 5.01)
        self.assertGreater(distance_m(rounded[1], road[0]), 0)

    def test_zero_radius_and_tight_corridor_are_identity(self):
        road = [(30, 104), (30, 104.0002), (30.0002, 104.0002)]
        self.assertEqual(rounded_leg(road, 0), road)
        self.assertEqual(rounded_leg(road, 8, 0), road)

    def test_invalid_parameter_fails_closed(self):
        for bad in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                rounded_leg([(30, 104)] * 3, radius_m=bad)


if __name__ == "__main__":
    unittest.main()
