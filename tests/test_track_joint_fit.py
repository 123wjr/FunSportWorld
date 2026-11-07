"""Offline path, gait, and altitude consistency tests with synthetic routes."""
import math
import unittest
from unittest.mock import patch

from funsport.track import generator, realism
from funsport.track.geom import make_point_ring, ring_point_at, meters_per_deg_lat, meters_per_deg_lng


class TrackJointFitTests(unittest.TestCase):
    @staticmethod
    def route():
        return [(39.90, 119.55), (39.901, 119.55),
                (39.901, 119.552), (39.90, 119.552), (39.90, 119.55)]

    def make_track(self, cadence=168, elevations=None, sample_period_s=2):
        route = self.route()
        _, arcs, _ = make_point_ring(route, ordered=True)
        distance = arcs[-1] * 2.5
        with patch.object(generator.log, "disabled", True):
            track = generator.build(distance, round(distance / 3), 7, 1000, route,
                                    ordered_path=True, cadence_target=cadence,
                                    elevations_m=elevations, sample_period_s=sample_period_s)
        return track, arcs[-1]

    def test_speed_steps_and_gait_share_the_path_distance(self):
        track, _ = self.make_track()
        points = track["locations"]
        self.assertEqual(track["elevationSource"], "synthetic_baseline")
        self.assertAlmostEqual(track["totalDistance"], points[-1]["totalDis"], places=2)
        self.assertEqual(track["totalSteps"], points[-1]["steps"])
        self.assertAlmostEqual(60 * track["totalSteps"] / track["totalTime"], 168, delta=1.0)
        for earlier, later in zip(points, points[1:]):
            dt = later["totalTime"] - earlier["totalTime"]
            dd = later["totalDis"] - earlier["totalDis"]
            ds = later["steps"] - earlier["steps"]
            self.assertGreater(dt, 0)
            self.assertGreaterEqual(dd, 0)
            self.assertGreaterEqual(ds, 0)
            if ds > 0:
                v = dd / dt
                c = 60 * ds / dt
                stride = dd / ds
                self.assertAlmostEqual(v, c * stride / 60, places=8)

    def test_route_elevation_repeats_with_distance_not_time(self):
        elevations = [10.0, 15.0, 15.0, 10.0, 10.0]
        track, lap = self.make_track(elevations=elevations)
        self.assertEqual(track["elevationSource"], "route_vertices")
        points = track["locations"]
        heights = [p["bdA"] for p in points]
        self.assertGreater(max(heights) - min(heights), 4)
        for p in points[1::20]:
            s = p["totalDis"] % lap
            _, arcs, _ = make_point_ring(self.route(), ordered=True)
            expected = generator._elevation_at(arcs, elevations, s)
            self.assertAlmostEqual(p["bdA"], expected, delta=0.11)
        self.assertAlmostEqual(points[0]["bdA"], 10, delta=0.11)

    def test_inserted_start_uses_route_origin_elevation(self):
        track, _ = self.make_track(elevations=[10.0, 30.0, 30.0, 10.0, 10.0])
        start, first_sample = track["locations"][:2]
        self.assertEqual(start["totalTime"], 0)
        self.assertAlmostEqual(start["bdA"], 10.0, places=2)
        self.assertGreater(first_sample["bdA"], start["bdA"])

    def test_grade_changes_speed_allocation_along_the_same_route(self):
        flat, lap = self.make_track(elevations=[10.0] * 5)
        sloped, _ = self.make_track(elevations=[10.0, 15.0, 15.0, 10.0, 10.0])
        self.assertAlmostEqual(flat["totalDistance"], sloped["totalDistance"], delta=0.01)
        self.assertEqual(flat["totalTime"], sloped["totalTime"])

        def leg_speeds(track, low, high):
            points = track["locations"]
            return [(b["totalDis"] - a["totalDis"]) / (b["totalTime"] - a["totalTime"])
                    for a, b in zip(points, points[1:])
                    if low < a["totalDis"] % lap < high and
                    b["totalDis"] // lap == a["totalDis"] // lap]

        _, arcs, _ = make_point_ring(self.route(), ordered=True)
        ascending = (10, arcs[1] - 10)
        descending = (arcs[2] + 10, arcs[3] - 10)
        uphill = sum(leg_speeds(sloped, *ascending)) / len(leg_speeds(sloped, *ascending))
        uphill_flat = sum(leg_speeds(flat, *ascending)) / len(leg_speeds(flat, *ascending))
        downhill = sum(leg_speeds(sloped, *descending)) / len(leg_speeds(sloped, *descending))
        downhill_flat = sum(leg_speeds(flat, *descending)) / len(leg_speeds(flat, *descending))
        self.assertLess(uphill, uphill_flat)
        self.assertGreater(downhill, downhill_flat)

    def test_coordinates_follow_same_arclength_as_distance(self):
        track, lap = self.make_track()
        dense, arcs, center = make_point_ring(self.route(), ordered=True)
        for p in track["locations"][1::25]:
            x, y = ring_point_at(dense, arcs, p["totalDis"] % lap)
            actual_x = (p["gLng"] - center[1]) * meters_per_deg_lng(center[0])
            actual_y = (p["gLat"] - center[0]) * meters_per_deg_lat(center[0])
            self.assertLess(math.hypot(actual_x - x, actual_y - y), 6.0)

    def test_one_second_sampling_is_run_level_and_consistent(self):
        track, _ = self.make_track(sample_period_s=1, elevations=[10, 15, 15, 10, 10])
        gaps = [b["totalTime"] - a["totalTime"]
                for a, b in zip(track["locations"], track["locations"][1:])]
        self.assertTrue(all(dt == 1 for dt in gaps))
        self.assertAlmostEqual(track["totalDistance"], track["locations"][-1]["totalDis"], delta=0.01)
        self.assertAlmostEqual(60 * track["totalSteps"] / track["totalTime"], 168, delta=1.1)
        with self.assertRaises(ValueError):
            self.make_track(sample_period_s=3)

    def test_one_second_sampling_many_seeds_keep_distance_and_gait(self):
        route = self.route()
        _, arcs, _ = make_point_ring(route, ordered=True)
        distance = arcs[-1] * 2.5
        duration = round(distance / 3)
        for seed in range(50):
            with self.subTest(seed=seed), patch.object(generator.log, "disabled", True):
                track = generator.build(distance, duration, seed, 1000, route,
                                        ordered_path=True, cadence_target=168,
                                        sample_period_s=1)
            self.assertAlmostEqual(track["totalDistance"], distance, delta=0.01)
            self.assertAlmostEqual(60 * track["totalSteps"] / duration, 168, delta=1.1)
            points = track["locations"]
            self.assertTrue(all(b["totalTime"] - a["totalTime"] == 1
                                for a, b in zip(points, points[1:])))

    def test_cadence_targets_anchor_full_run_average(self):
        for cadence in (60, 70, 130, 155, 168, 190, 200, 220):
            with self.subTest(cadence=cadence):
                track, _ = self.make_track(cadence=cadence)
                realized = 60 * track["totalSteps"] / track["totalTime"]
                self.assertAlmostEqual(realized, cadence, delta=1.1)

    def test_clipped_cadence_still_matches_weighted_target(self):
        for cadence in (60, 70, 130, 168, 200, 220):
            for seed in range(10):
                with self.subTest(cadence=cadence, seed=seed), patch.object(generator.log, "disabled", True):
                    track = generator.build(1600, 500, seed, 1000, self.route(),
                                            ordered_path=True, cadence_target=cadence)
                self.assertAlmostEqual(60 * track["totalSteps"] / track["totalTime"],
                                       cadence, delta=0.25)

    def test_many_seeds_keep_distance_and_cadence_targets(self):
        route = self.route()
        _, arcs, _ = make_point_ring(route, ordered=True)
        distance = arcs[-1] * 2.5
        duration = round(distance / 3)
        for seed in range(100):
            with self.subTest(seed=seed), patch.object(generator.log, "disabled", True):
                track = generator.build(distance, duration, seed, 1000, route,
                                        ordered_path=True, cadence_target=168,
                                        elevations_m=[10, 15, 15, 10, 10])
            self.assertAlmostEqual(track["totalDistance"], distance, delta=0.01)
            self.assertAlmostEqual(60 * track["totalSteps"] / duration, 168, delta=1.1)
            self.assertEqual(track["totalSteps"], track["locations"][-1]["steps"])

    def test_invalid_elevation_input_rejected(self):
        with self.assertRaises(ValueError):
            self.make_track(elevations=[10.0, 11.0])
        with self.assertRaises(ValueError):
            self.make_track(elevations=[10, 11, math.nan, 10, 10])
        with self.assertRaises(ValueError):
            self.make_track(elevations=[10, 11, 11, 10, 20])
        with self.assertRaises(ValueError):
            self.make_track(elevations=[10, 11, "12", 10, 10])

    def test_invalid_cadence_target_rejected(self):
        for cadence in (-1, 1, 59, 221, math.nan, math.inf, "168"):
            with self.subTest(cadence=cadence), self.assertRaises(ValueError):
                self.make_track(cadence=cadence)

    def test_cadence_model_preserves_speed_stride_identity(self):
        for v in (1.0, 1.75, 2.1, 2.5, 3.0, 4.5):
            self.assertAlmostEqual(realism.cadence_for_speed(v) *
                                   realism.stride_for_speed(v) / 60, v)
        self.assertEqual(realism.cadence_for_speed(3.0), realism.CADENCE_AT_REF)


if __name__ == "__main__":
    unittest.main()
