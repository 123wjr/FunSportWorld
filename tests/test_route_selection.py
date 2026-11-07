"""Scored route search tests with no network or account data."""
import math
import random
import unittest
from unittest.mock import patch
from funsport.api import campus_loop as loop
from funsport.track.route_metrics import EARTH_RADIUS_M, route_overlap

def square_legs():
    sy = EARTH_RADIUS_M * math.pi / 180
    sx = sy * math.cos(math.radians(30))
    a, b, c, d = [(30 + y / sy, 104 + x / sx) for x, y in ((0, 0), (100, 0), (100, 100), (0, 100))]
    def option(path):
        return dict(points=path, distance=1000, source="v5")
    return [[option([a, b, a, b]), option([a, b])], [option([b, c])], [option([c, d])], [option([d, a])]], [loop.gcj02_to_bd09(*p) for p in (a, b, c, d)]

class ScoredRouteTests(unittest.TestCase):
    def setUp(self):
        quiet = patch.object(loop.log, "disabled", True)
        quiet.start()
        self.addCleanup(quiet.stop)

    def test_less_retrace_wins_for_all_seeds_and_reports_returned_geometry(self):
        legs, checkpoints = square_legs()
        for seed in (0, 1, 42, -1):
            ring, info = loop._select_loop(legs, checkpoints, seed)
            self.assertEqual(info["selected_variant"], 2)
            self.assertEqual(info["evaluated_combinations"], 2)
            self.assertEqual(info["near_optimal_combinations"], 1)
            actual = route_overlap(ring)
            self.assertAlmostEqual(info["route_length_m"], actual["length_m"], places=3)
            self.assertAlmostEqual(info["repeated_distance_m"], actual["repeated_m"], places=3)
            self.assertLess(info["repeated_distance_m"], 1)
            self.assertEqual(ring[0], ring[-1])

    def test_invalid_cheapest_combination_is_never_selected(self):
        legs, checkpoints = square_legs()
        legs[0][1]["points"][-1] = (31, 105)
        _, info = loop._select_loop(legs, checkpoints, 1)
        self.assertEqual(info["selected_variant"], 1)
        self.assertEqual(info["valid_combinations"], 1)

    def test_all_243_combinations_scored_including_last(self):
        legs = [[{"points": []}] * 3 for _ in range(5)]
        seen = []
        def assemble(legs, choices, quiet=False):
            seen.append(tuple(choices))
            return [10] if choices == [2] * 5 else [100]
        def metric(ring):
            return dict(length_m=ring[0], repeated_m=0, repeat_ratio=0)
        with patch.object(loop, "_assemble_loop", side_effect=assemble), patch.object(loop, "_validated_ring", side_effect=lambda ring, points: ring), patch.object(loop, "route_overlap", side_effect=metric):
            _, info = loop._select_loop(legs, [], 0)
        self.assertEqual(len(set(seen)), 243)
        self.assertEqual(info["selected_variant"], 243)
        self.assertEqual(info["evaluated_combinations"], 243)
        self.assertTrue(info["search_exhaustive"])

    def test_bounded_sample_has_no_duplicates_and_preserves_global_rng(self):
        legs = [[{}] * 3 for _ in range(6)]
        seen = []
        def assemble(legs, choices, quiet=False):
            seen.append(tuple(choices))
            return None
        state = random.getstate()
        with patch.object(loop, "_assemble_loop", side_effect=assemble), self.assertRaises(RuntimeError):
            loop._select_loop(legs, [], 42)
        self.assertEqual(random.getstate(), state)
        self.assertEqual(len(seen), 512)
        self.assertEqual(len(set(seen)), 512)

    def test_near_optimal_seed_cycle_and_reproducibility(self):
        legs = [[{}] * 3]
        def assemble(legs, choices, quiet=False):
            return [100 + choices[0] * 4]
        def metric(ring):
            return dict(length_m=ring[0], repeated_m=0, repeat_ratio=0)
        with patch.object(loop, "_assemble_loop", side_effect=assemble), patch.object(loop, "_validated_ring", side_effect=lambda ring, points: ring), patch.object(loop, "route_overlap", side_effect=metric):
            outputs = [loop._select_loop(legs, [], seed) for seed in range(4)]
            self.assertEqual(outputs[1], loop._select_loop(legs, [], 1))
        self.assertEqual([info["selected_variant"] for _, info in outputs], [1, 2, 1, 2])
        for _, info in outputs:
            self.assertLessEqual(info["selection_score"], info["best_selection_score"] * 1.05)

    def test_missing_options_and_invalid_seeds(self):
        for legs in ([], [[]]):
            with self.assertRaises(RuntimeError):
                loop._select_loop(legs, [], 0)
        for seed in (True, None, "1", 1.5):
            with self.assertRaises(ValueError):
                loop._select_loop([], [], seed)

    def test_huge_combination_space_is_bounded_unique_and_reproducible(self):
        legs = [[{}] * 3 for _ in range(40)]
        def sample():
            seen = []
            def assemble(legs, choices, quiet=False):
                seen.append(tuple(choices))
                return None
            with patch.object(loop, "_assemble_loop", side_effect=assemble), self.assertRaisesRegex(RuntimeError, "512/12157665459056928801"):
                loop._select_loop(legs, [], 42)
            return seen
        state = random.getstate()
        first = sample()
        self.assertEqual(first, sample())
        self.assertEqual(random.getstate(), state)
        self.assertEqual(len(first), 512)
        self.assertEqual(len(set(first)), 512)

    def test_only_selected_route_emits_cleaning_log(self):
        legs, checkpoints = square_legs()
        with patch.object(loop, "dim") as emit:
            loop._select_loop(legs, checkpoints, 0)
        self.assertEqual(emit.call_count, 1)
        self.assertIn("清洗:", emit.call_args.args[0])


class CoverageGridTests(unittest.TestCase):
    @staticmethod
    def destination(point, metres, bearing):
        lat, lon = map(math.radians, point)
        angle = metres / EARTH_RADIUS_M
        target_lat = math.asin(math.sin(lat) * math.cos(angle) +
                               math.cos(lat) * math.sin(angle) * math.cos(bearing))
        target_lon = lon + math.atan2(math.sin(bearing) * math.sin(angle) * math.cos(lat),
                                      math.cos(angle) - math.sin(lat) * math.sin(target_lat))
        return math.degrees(target_lat), (math.degrees(target_lon) + 180) % 360 - 180

    def test_coverage_matches_brute_force_at_threshold_poles_and_dateline(self):
        rng = random.Random(73)
        for centre in ((30, 104), (0, 179.9999), (89.9999, -179.9), (-89.9999, 30)):
            ring = [self.destination(centre, rng.uniform(0, 2000), rng.uniform(0, math.tau))
                    for _ in range(80)]
            for index in range(40):
                origin = ring[index]
                metres = (99.999999, 100, 100.000001, 250)[index % 4]
                point = self.destination(origin, metres, rng.uniform(0, math.tau))
                expected = any(loop.dist_m(point, vertex) <= loop.ENDPOINT_TOLERANCE_M for vertex in ring)
                self.assertEqual(loop._covers_checkpoints(ring, [point]), expected)
            self.assertTrue(loop._covers_checkpoints(ring, ring[::10]))
            self.assertTrue(loop._covers_checkpoints(ring, []))

    def test_sparse_coverage_prunes_distance_evaluations(self):
        ring = [(30 + i * .003, 104) for i in range(1000)]
        checkpoints = ring[::25]
        with patch.object(loop, "dist_m", wraps=loop.dist_m) as distance:
            self.assertTrue(loop._covers_checkpoints(ring, checkpoints))
        self.assertLess(distance.call_count, 200)

