import math
from dataclasses import replace
import unittest
from unittest.mock import patch

from funsport.track import generator
from funsport.track.geom import (make_point_ring, ring_point_at, to_bd,
                                 meters_per_deg_lat, meters_per_deg_lng)
from funsport.track.joint_controls import (JointControls, LateralField, offset_route,
                                             controls_from_training_stats)
from funsport.coordinates import bd09_to_gcj02, distance_m
from tools.road_joint_controls_eval import evaluate, summarize_track


class JointControlsTests(unittest.TestCase):
    def setUp(self):
        self.route = [(39.90, 119.55), (39.901, 119.55),
                      (39.901, 119.552), (39.90, 119.552), (39.90, 119.55)]
        _, arcs, _ = make_point_ring(self.route, ordered=True, open_path=True)
        self.distance = arcs[-1] * 0.45
        self.duration = round(self.distance / 3.0)

    def build(self, controls=None, seed=7):
        with patch.object(generator.log, "disabled", True):
            return generator.build(self.distance, self.duration, seed, 1000, self.route,
                                   ordered_path=True, open_path=True, distance_mode="sampled_gcj",
                                   cadence_target=170, joint_controls=controls)

    def test_none_and_zero_are_field_and_rng_compatible(self):
        self.assertEqual(self.build(None), self.build(JointControls()))

    def test_speed_and_cadence_controls_are_identifiable_and_closed(self):
        control = JointControls(speed_sigma=0.12, speed_tau_s=45, start_transient=0.12,
                                finish_transient=0.10, cadence_slope_spm_per_mps=15,
                                cadence_tau_s=8, cadence_residual_spm=2)
        track = self.build(control)
        base = self.build(None)
        self.assertNotEqual([p["gLat"] for p in track["locations"]],
                            [p["gLat"] for p in base["locations"]])
        self.assertEqual(track["totalSteps"], track["locations"][-1]["steps"])
        for a, b in zip(track["locations"], track["locations"][1:]):
            dt = b["totalTime"] - a["totalTime"]
            dd = b["totalDis"] - a["totalDis"]
            ds = b["steps"] - a["steps"]
            if dt > 0 and ds > 0:
                self.assertAlmostEqual(dd / dt, (60 * ds / dt) * (dd / ds) / 60, places=8)

    def test_lateral_signal_anchors_origin_and_changes_side_smoothly(self):
        dense, arcs, _ = make_point_ring(self.route, ordered=True, open_path=True)
        controls = JointControls(lateral_mean_m=1.2, lateral_amplitude_m=0.7, lateral_tau_s=90)
        shifted = offset_route(dense, arcs, controls)
        self.assertEqual(shifted[0], dense[0])
        field = LateralField(dense, arcs, controls)
        for i in range(1, len(dense) - 1):
            _, normal = field.frame(arcs[i])
            signed_normal_m = sum((a - b) * n for a, b, n in zip(shifted[i], dense[i], normal))
            expected = (controls.lateral_mean_m * (1 - math.exp(-arcs[i] / controls.lateral_tau_s)) +
                        controls.lateral_amplitude_m * math.sin(math.tau * arcs[i] / controls.lateral_tau_s))
            self.assertAlmostEqual(signed_normal_m, expected, places=6)
        moves = [math.hypot(a[0]-b[0], a[1]-b[1]) for a,b in zip(dense, shifted)]
        self.assertLessEqual(max(moves), 2.0)
        self.assertGreater(max(moves), 0.2)
        track = self.build(controls)
        self.assertAlmostEqual(track["locations"][0]["gLat"], self.route[0][0], places=6)
        self.assertAlmostEqual(track["locations"][0]["gLng"], self.route[0][1], places=6)
        self.assertLessEqual(track["distanceAudit"]["max_road_gcj_speed_mps"], 6.6)
        fixes = track["locations"]
        for a,b in zip(fixes[1:], fixes[2:]):
            dt = b["totalTime"] - a["totalTime"]
            if dt > 0:
                self.assertLessEqual(distance_m(bd09_to_gcj02(a["gLat"], a["gLng"]),
                                               bd09_to_gcj02(b["gLat"], b["gLng"])) / dt, 6.6)

    @staticmethod
    def straight(step=1000):
        return [to_bd(x, 0, 30.0, 110.0) for x in range(0, 1001, step)]

    def synthetic_build(self, route, controls=None, seed=7, distance=300, duration=100):
        with patch.object(generator.log, "disabled", True):
            return generator.build(distance, duration, seed, 1000, route, ordered_path=True,
                                   open_path=True, distance_mode="sampled_gcj",
                                   cadence_target=170, joint_controls=controls)

    def test_same_geometry_sparse_dense_field_and_sparse_build_effect(self):
        controls = JointControls(lateral_amplitude_m=8, lateral_tau_s=100)
        fields = []
        for step in (1000, 25):
            dense, arcs, center = make_point_ring(self.straight(step), ordered=True, open_path=True)
            original = (list(dense), list(arcs))
            field = LateralField(dense, arcs, controls)
            fields.append(field)
            for s in (0, 1, 25, 37.5, 100, 399.9, 999):
                source = ring_point_at(dense, arcs, s, open_path=True)
                self.assertAlmostEqual(math.dist(field.point(s), source),
                                       abs(8 * math.sin(math.tau * s / 100)), places=7)
            self.assertEqual((dense, arcs), original)
            self.assertAlmostEqual(math.dist(field.point(25), dense[0]) ** 2, 25**2 + 8**2, places=5)
        for s in (0, 1, 25, 37.5, 100, 399.9, 999):
            # Both representations have the same center; no artificial route vertices are added.
            self.assertLess(math.dist(fields[0].point(s), fields[1].point(s)), 1e-7)
        route = self.straight()
        self.assertNotEqual(self.synthetic_build(route, controls), self.synthetic_build(route))

    def test_geographic_asymmetric_collinear_subdivision_field_tolerance(self):
        controls = JointControls(lateral_mean_m=1.2, lateral_amplitude_m=.7)
        coarse = [to_bd(x,y,30.,110.) for x,y in
                  [(0,0),(100,0),(100,100),(0,100),(0,0),(500,0)]]
        fine = [coarse[0]]
        for count, a, b in zip((3,17,8,31,5), coarse, coarse[1:]):
            fine.extend([(a[0]+(b[0]-a[0])*j/count, a[1]+(b[1]-a[1])*j/count)
                         for j in range(1, count+1)])
        fields = []
        for route in (coarse, fine):
            dense, arcs, center = make_point_ring(route, ordered=True, open_path=True)
            self.assertEqual(len(dense), len(route))
            fields.append((LateralField(dense, arcs, controls), center))
        # Existing source projection uses vertex-mean latitude; changing that mean
        # produces sub-millimetre field drift on this 800 m artificial route.
        for i in range(1601):
            samples = [to_bd(*field.point(i*.5), *center) for field, center in fields]
            self.assertLess(distance_m(*samples), .002)

    def test_corner_frame_subdivision_revisit_and_continuity(self):
        controls = JointControls(lateral_mean_m=2, lateral_amplitude_m=1, lateral_tau_s=90)
        # The last segment naturally revisits the first segment in the opposite direction.
        coarse = [(0., 0.), (100., 0.), (100., 100.), (0., 100.), (0., 0.), (100., 0.), (0., 0.)]
        fine = [coarse[0]]
        for a, b in zip(coarse, coarse[1:]):
            fine.extend([(a[0] + (b[0]-a[0])*i/10, a[1] + (b[1]-a[1])*i/10) for i in range(1, 11)])
        def field(points):
            arcs = [0.]
            for a, b in zip(points, points[1:]):
                arcs.append(arcs[-1] + math.dist(a, b))
            return LateralField(points, arcs, controls)
        sparse, dense = field(coarse), field(fine)
        for s in (0., 95., 100., 105., 200., 400., 499.9999, 500., 500.0001, 599.):
            self.assertLess(math.dist(sparse.point(s), dense.point(s)), 1e-9)
            left, right = sparse.point(max(0., s-1e-5)), sparse.point(min(600., s+1e-5))
            self.assertLess(math.dist(left, right), 1e-4)
        tangent, normal = sparse.frame(500.)
        self.assertAlmostEqual(tangent[0], 0., places=8)
        self.assertAlmostEqual(tangent[1], 1., places=8)  # Exact U-turn chooses left.
        self.assertAlmostEqual(sum(a*b for a,b in zip(tangent, normal)), 0., places=12)

    def assert_native_and_road_guards(self, track, route, target):
        points = track["locations"]
        gcj = [bd09_to_gcj02(p["gLat"], p["gLng"]) for p in points]
        chords = [distance_m(a, b) for a,b in zip(gcj, gcj[1:])]
        dts = [b["totalTime"] - a["totalTime"] for a,b in zip(points, points[1:])]
        dense, arcs, center = make_point_ring(route, ordered=True, open_path=True)
        road = generator._GcjRoadTraversal(dense, arcs, center)
        for i, (a, b, dd, dt) in enumerate(zip(points, points[1:], chords, dts)):
            self.assertGreater(dt, 0)
            self.assertGreater(b["pathProgressM"], a["pathProgressM"])
            self.assertLessEqual(dd/dt, 6.6 + 1e-9)
            traversed = road.distance(a["pathProgressM"], b["pathProgressM"])
            self.assertLessEqual(traversed/dt, 6.6 + 1e-9)
            self.assertAlmostEqual(b["totalDis"] - a["totalDis"], dd, delta=.002)
            ds = b["steps"] - a["steps"]
            self.assertGreater(ds, 0)
            self.assertAlmostEqual(dd/dt, (60*ds/dt)*(dd/ds)/60, places=10)
        self.assertLessEqual(abs(sum(chords) - target), .25)
        self.assertAlmostEqual(sum(chords), track["totalDistance"], delta=.00051)
        self.assertEqual(track["totalSteps"], points[-1]["steps"])
        self.assertEqual(points[-1]["totalTime"], track["totalTime"])
        self.assertEqual((points[0]["gLat"], points[0]["gLng"]), tuple(round(x,6) for x in route[0]))
        self.assertLessEqual(points[-1]["pathProgressM"], arcs[-1])

    def test_final_quantized_envelope_mean8_tau5_seed5(self):
        # Same class of counterexample as the independently reported 8.75749 m breach.
        route = self.straight(5)
        controls = JointControls(lateral_mean_m=8, lateral_tau_s=5)
        track = self.synthetic_build(route, controls, seed=5)
        dense, arcs, center = make_point_ring(route, ordered=True, open_path=True)
        field = LateralField(dense, arcs, controls)
        for p in track["locations"]:
            s = p["pathProgressM"]
            source = ring_point_at(dense, arcs, s, open_path=True)
            source_bd = to_bd(*source, *center)
            stored = (p["gLat"], p["gLng"])
            local = ((stored[1]-source_bd[1])*meters_per_deg_lng(center[0]),
                     (stored[0]-source_bd[0])*meters_per_deg_lat(center[0]))
            tangent, normal = field.frame(s)
            signed = sum(a*b for a,b in zip(local, normal))
            tangential = sum(a*b for a,b in zip(local, tangent))
            self.assertLessEqual(math.hypot(signed, tangential), 8 + 1e-8)
            self.assertLessEqual(distance_m(stored, source_bd), 8 + 1e-8)
            self.assertLessEqual(distance_m(bd09_to_gcj02(*stored), bd09_to_gcj02(*source_bd)), 8 + 1e-8)
        self.assert_native_and_road_guards(track, route, 300)

    def test_quantized_envelope_both_sides_tiny_grid_failure(self):
        dense, arcs, center = make_point_ring(self.straight(5), ordered=True, open_path=True)
        for sign in (-1, 1):
            field = LateralField(dense, arcs, JointControls(lateral_mean_m=sign*8, lateral_tau_s=5))
            for s in (0, .5, 5, 25.123456, 100, 500):
                _, _, x, y, _ = field.sample(s, (4., sign*4.), center)
                stored = tuple(round(v, 6) for v in to_bd(x, y, *center))
                source = to_bd(*ring_point_at(dense, arcs, s, open_path=True), *center)
                self.assertLessEqual(distance_m(stored, source), 8 + 1e-8)
        tiny = LateralField(dense, arcs, JointControls(lateral_mean_m=.000001))
        with self.assertRaisesRegex(ValueError, "no feasible quantized candidate"):
            tiny.sample(1.234567, (0., 0.), center)

    def test_natural_revisit_build_seed_and_native_guards(self):
        route = [to_bd(x, y, 30., 110.) for x,y in
                 [(0,0),(50,0),(50,50),(0,50),(0,0),(500,0)]]
        controls = JointControls(lateral_mean_m=1.2, lateral_amplitude_m=.7)
        for seed in (1, 5, 7):
            track = self.synthetic_build(route, controls, seed)
            self.assertEqual(track, self.synthetic_build(route, controls, seed))
            self.assert_native_and_road_guards(track, route, 300)
            self.assertGreater(track["locations"][-1]["pathProgressM"], 200)
        a = self.synthetic_build(route, controls, 1)
        b = self.synthetic_build(route, controls, 5)
        self.assertNotEqual(a, b)

    def test_sharp_revisit_reports_guard_failure_without_repairs(self):
        route = [to_bd(x,y,30.,110.) for x,y in [(0,0),(80,0),(0,0),(500,0)]]
        controls = JointControls(lateral_mean_m=1.2, lateral_amplitude_m=.7)
        original = list(route)
        for seed in (1,7):
            with self.assertRaisesRegex(ValueError, "GCJ road traversal speed budget"):
                self.synthetic_build(route, controls, seed)
        self.assertEqual(route, original)
        self.assert_native_and_road_guards(self.synthetic_build(route, controls, 5), route, 300)

    def test_cadence_only_parameters_leave_position_process_and_legacy_rng_unchanged(self):
        base = JointControls(speed_sigma=.08, lateral_mean_m=1.2, lateral_amplitude_m=.7)
        variants = (base, replace(base, cadence_residual_spm=2),
                    replace(base, cadence_slope_spm_per_mps=18), replace(base, cadence_tau_s=3))
        original_rng = generator.Rng
        original_plan = generator._sampled_gcj_plan
        captures = []
        for control in variants:
            streams, process = [], []
            def rng(seed):
                result = original_rng(seed)
                streams.append((seed, result))
                return result
            def plan(dense, arcs, center, speeds, dts, x, y, target, **kwargs):
                process.append((list(speeds), list(dts), list(x), list(y)))
                return original_plan(dense, arcs, center, speeds, dts, x, y, target, **kwargs)
            with patch.object(generator, "Rng", side_effect=rng), patch.object(generator, "_sampled_gcj_plan", side_effect=plan):
                track = self.synthetic_build(self.straight(), control)
            primary = [r.r.getstate() for seed,r in streams if seed == 7]
            self.assertEqual(len(primary), 1)
            geometry = [tuple(p[k] for k in ("gLat", "gLng", "totalTime", "gainTime", "pathProgressM", "totalDis", "speed", "avgSpeed")) for p in track["locations"]]
            captures.append((process, primary, geometry, [p["steps"] for p in track["locations"]]))
        for process, state, geometry, steps in captures[1:]:
            self.assertEqual(process, captures[0][0])
            self.assertEqual(state, captures[0][1])
            self.assertEqual(geometry, captures[0][2])
        self.assertNotEqual(captures[0][3], captures[1][3])

    def test_evaluator_pace_units_explicit_target_and_failure_denominator(self):
        track = {"locations": [{"gLat":30.,"gLng":110.,"totalTime":0,"steps":0},
                               {"gLat":30.,"gLng":110.001,"totalTime":54,"steps":153}],
                 "totalTime":54, "totalDistance":160.544, "totalSteps":153,
                 "distanceAudit":{"max_road_gcj_speed_mps":3.}}
        with patch("tools.road_joint_controls_eval.distance_m", return_value=160.544):
            summary = summarize_track(track, 160.5, 54, 170)
        self.assertAlmostEqual(summary["pace_min_km"], 5.605939805910404)
        self.assertEqual(summary["cadence_mean_spm"], 170)
        self.assertEqual(summary["duration_s"], 54)
        self.assertAlmostEqual(summary["stride_mean_m"], 160.544/153)
        calls = []
        real_build = generator.build
        def build(*args, **kwargs):
            calls.append(kwargs["cadence_target"])
            if args[2] == 2 and kwargs["joint_controls"] is not None:
                raise ValueError("synthetic controlled failure")
            return real_build(*args, **kwargs)
        with patch.object(generator, "build", side_effect=build):
            result = evaluate({"speed_sigma":.08}, [1,2,3], "bend")
        self.assertEqual(calls, [170.] * 6)
        self.assertEqual((result["attempts"], result["baseline_success"], result["candidate_success"]), (3,3,2))
        self.assertFalse(result["rows"][1]["candidate"]["ok"])
        for pair in result["rows"]:
            for label in ("baseline", "candidate"):
                if pair[label]["ok"]:
                    row = pair[label]
                    self.assertEqual(row["cadence_target_spm"], 170.)
                    self.assertLessEqual(abs(row["cadence_mean_spm"]-170), 60/row["duration_s"])
                    self.assertTrue(4 <= row["pace_min_km"] <= 10)
                    self.assertLessEqual(abs(row["native_minus_reported_m"]), .00051)

    def test_aggregate_statistics_fit_scalar_controls_only(self):
        fitted = controls_from_training_stats({"speed_log_sigma": 0.08,
            "speed_correlation_s": 40, "cadence_speed_slope": 18})
        self.assertEqual(fitted.speed_sigma, 0.08)
        self.assertEqual(fitted.speed_tau_s, 40)
        self.assertEqual(fitted.cadence_slope_spm_per_mps, 18)
        with self.assertRaisesRegex(ValueError, "unknown aggregate training statistic"):
            controls_from_training_stats({"locations": [(1, 2)]})

    def test_invalid_and_non_sampled_controls_rejected(self):
        with self.assertRaisesRegex(ValueError, "lateral offset"):
            JointControls(lateral_mean_m=8, lateral_amplitude_m=1).validate()
        with self.assertRaisesRegex(ValueError, "joint controls require"):
            generator.build(1500, 500, 1, 1000, self.route, ordered_path=True, joint_controls=JointControls())


if __name__ == "__main__":
    unittest.main()
