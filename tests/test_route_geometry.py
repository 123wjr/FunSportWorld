"""闭环覆盖、上传坐标及App历史连线规则的离线回归。"""
import base64
import gzip
import json
from unittest.mock import patch

from tests.support import IsolatedCase
from tests.plan_fixture import sample_ring
from funsport import map_preview
from funsport.api.flow import _ring_length
from funsport.coordinates import bd09_to_gcj02, distance_m
from funsport.track import generator, wire
from funsport.track.geom import make_point_ring, ring_point_at, to_bd


class RouteGeometryTests(IsolatedCase):
    """使用人工闭环验证几何完整性，不调用地图或运动服务。"""

    def build(self, distance, seed=42):
        """沿固定闭环生成已知里程的有序测试轨迹。"""
        with patch.object(generator.log, "disabled", True):
            return generator.build(distance, round(distance / 2.5), seed, 1000,
                                   sample_ring(), ordered_path=True, cadence_target=160)

    def typed_track(self, types, headings=None):
        """创建便于人工核对抽样索引的少量轨迹点。"""
        headings = headings or [0.0] * len(types)
        return {"locations": [{"gLat": 30.6 + index * 0.001, "gLng": 104.1,
                               "type": point_type, "bdD": heading}
                              for index, (point_type, heading) in enumerate(zip(types, headings))]}

    def test_thirty_seeds_complete_requested_lap(self):
        """此前19次不足一圈的30个种子，现在均达到目标且不伪造业务事件。"""
        length = _ring_length(sample_ring())
        for seed in range(30):
            with self.subTest(seed=seed):
                track = self.build(length + 50, seed)
                self.assertGreaterEqual(track["totalDistance"], length)
                self.assertAlmostEqual(track["totalDistance"], length + 50, places=2)
                points = track["locations"]
                self.assertEqual(points[0]["type"], 5)
                self.assertEqual(points[-1]["type"], 6)
                self.assertTrue(all(point["type"] == 0 for point in points[1:-1]))

    def test_one_lap_returns_to_start_on_the_path(self):
        """目标为一整圈时真实上传采样首尾相接，不只在画布上补线。"""
        track = self.build(_ring_length(sample_ring()))
        geometry = map_preview.history_geometry(track)
        self.assertLess(distance_m(geometry["start"], geometry["end"]), 0.03)
        self.assertEqual(geometry["segments"][0][0], geometry["start"])
        self.assertEqual(geometry["segments"][-1][1], geometry["end"])

    def test_partial_extra_lap_does_not_teleport_to_start(self):
        """一圈加部分里程可覆盖闭环，但不强行把终点改回起点。"""
        length = _ring_length(sample_ring())
        track = self.build(length * 1.25)
        geometry = map_preview.history_geometry(track)
        self.assertGreater(distance_m(geometry["start"], geometry["end"]), 100)
        self.assertAlmostEqual(track["totalDistance"], length * 1.25, places=2)

    def test_start_time_distance_and_identifiers_are_consistent(self):
        """起点为t=0，后续时间与里程递增，终点数据和轨迹汇总相同。"""
        track = self.build(_ring_length(sample_ring()) + 50)
        points = track["locations"]
        self.assertEqual((points[0]["totalTime"], points[0]["totalDis"], points[0]["steps"]), (0, 0, 0))
        self.assertEqual((points[0]["gLat"], points[0]["gLng"]), sample_ring()[0])
        self.assertEqual(points[0]["gainTimeMs"], track["startTime"])
        self.assertEqual(points[-1]["totalTime"], track["totalTime"])
        self.assertAlmostEqual(points[-1]["totalDis"], track["totalDistance"], places=2)
        self.assertEqual(points[-1]["steps"], track["totalSteps"])
        self.assertEqual([point["id"] for point in points], list(range(1, len(points) + 1)))
        for previous, current in zip(points, points[1:]):
            self.assertLess(previous["totalTime"], current["totalTime"])
            self.assertLessEqual(previous["totalDis"], current["totalDis"])

    def test_infeasible_speed_does_not_make_terminal_jump(self):
        """无法在速度约束内覆盖里程时失败，不用最后一点瞬移凑距离。"""
        with patch.object(generator.log, "disabled", True), self.assertRaises(ValueError):
            generator.build(2000, 10, 42, 0, sample_ring(), ordered_path=True)

    def test_invalid_samples_are_skipped_not_line_breaks(self):
        """起终点之间的type=-1像App一样跳过，不导致整段线消失。"""
        track = self.typed_track([5, -1, 6])
        coords = map_preview.track_coordinates(track)
        geometry = map_preview.history_geometry(track)
        self.assertIsNone(coords[1])
        self.assertEqual(geometry["segments"], [(coords[0], coords[2], "#00c18b")])

    def test_same_type_and_heading_retains_every_other_point(self):
        """同类型且方向差不超过10度时，对照APK保留第0、2、4点。"""
        track = self.typed_track([0] * 5)
        coords = map_preview.track_coordinates(track)
        self.assertEqual(map_preview.history_geometry(track)["segments"],
                         [(coords[0], coords[2], "#00c18b"), (coords[2], coords[4], "#00c18b")])

    def test_type_and_heading_changes_preserve_points(self):
        """类型变化或方向差超过10度会保留点，边界10度本身不会触发。"""
        track = self.typed_track([0, 0, 0, 5, 6], [0, 10, 21, 21, 21])
        coords = map_preview.track_coordinates(track)
        geometry = map_preview.history_geometry(track)
        self.assertEqual([segment[1] for segment in geometry["segments"]], coords[2:])

    def test_app_colors_and_explicit_markers(self):
        """起终点来自type5/6，灰色和红色业务点不再全部画成蓝线。"""
        track = self.typed_track([0, 5, 3, 4, 7, 10, 6])
        coords = map_preview.track_coordinates(track)
        geometry = map_preview.history_geometry(track)
        self.assertEqual(geometry["start"], coords[1])
        self.assertEqual(geometry["end"], coords[-1])
        self.assertEqual([segment[2] for segment in geometry["segments"]],
                         ["#00c18b", "#b5b5b5", "#b5b5b5", "#ff0000", "#ff0000", "#00c18b"])

    def test_preview_coordinates_equal_decoded_obs(self):
        """预览逐点使用与实际OBS解压后完全相同的七位GCJ坐标。"""
        track = self.build(_ring_length(sample_ring()))
        obs = wire.build_obs_object(track, 123, "fixture", 1, [])
        run_wrapper = json.loads(gzip.decompress(base64.b64decode(obs["run_data"])))
        encoded_points = json.loads(run_wrapper["allLocJson"])
        self.assertEqual(map_preview.track_coordinates(track),
                         [(point["gLat"], point["gLng"]) for point in encoded_points])
        self.assertEqual([point["type"] for point in track["locations"]],
                         [point["type"] for point in encoded_points])

    def test_open_path_never_closes_or_wraps(self):
        route = sample_ring()[:-1]
        dense, arcs, center = make_point_ring(route, ordered=True, open_path=True)
        self.assertNotEqual(dense[0], dense[-1])
        self.assertEqual(len(dense), len(route))
        self.assertEqual(ring_point_at(dense, arcs, arcs[-1], open_path=True), dense[-1])
        self.assertEqual(ring_point_at(dense, arcs, arcs[-1] + 100, open_path=True), dense[-1])
        self.assertEqual(ring_point_at(dense, arcs, -100, open_path=True), dense[0])
        closed, closed_arcs, _ = make_point_ring(route, ordered=True)
        self.assertEqual(closed[0], closed[-1])
        self.assertGreater(closed_arcs[-1], arcs[-1])

    def test_open_path_generator_stays_on_mapped_road(self):
        route = sample_ring()[:-1]
        dense, arcs, center = make_point_ring(route, ordered=True, open_path=True)
        distance = arcs[-1] - 50
        with patch.object(generator.log, "disabled", True):
            track = generator.build(distance, round(distance / 2.5), 42, 1000,
                                    route, ordered_path=True, open_path=True,
                                    cadence_target=160)
        end = track["locations"][-1]
        x, y = ring_point_at(dense, arcs, distance, open_path=True)
        self.assertLess(distance_m((end["gLat"], end["gLng"]), to_bd(x, y, *center)), 0.12)
        self.assertEqual((track["locations"][0]["gLat"], track["locations"][0]["gLng"]), route[0])
        self.assertAlmostEqual(track["totalDistance"], distance, places=1)

    def test_open_path_can_stop_exactly_at_last_mapped_vertex(self):
        route = sample_ring()[:-1]
        _, arcs, _ = make_point_ring(route, ordered=True, open_path=True)
        with patch.object(generator.log, "disabled", True):
            track = generator.build(arcs[-1], round(arcs[-1] / 2.5), 42, 1000,
                                    route, ordered_path=True, open_path=True,
                                    cadence_target=160)
        end = track["locations"][-1]
        self.assertLess(distance_m((end["gLat"], end["gLng"]), route[-1]), .12)

    def test_open_path_rejects_distance_requiring_unmapped_connector(self):
        route = sample_ring()[:-1]
        _, arcs, _ = make_point_ring(route, ordered=True, open_path=True)
        with patch.object(generator.log, "disabled", True), self.assertRaisesRegex(ValueError, "道路折线不足"):
            generator.build(arcs[-1] + 1, round(arcs[-1] / 2.5), 42, 1000,
                            route, ordered_path=True, open_path=True)

    @staticmethod
    def sampled_road(shape, length=6000):
        """Synthetic road substitutes, never a private/reference GPS skeleton."""
        import math
        from funsport.coordinates import gcj02_to_bd09
        coords = []
        for i in range(int(length / 3) + 1):
            x = i * 3.0
            if shape == "straight":
                y = 0.0
            elif shape == "turns":
                x = min(i * 3.0, 500.0) + max(0.0, i * 3.0 - 1000.0)
                y = min(500.0, max(0.0, i * 3.0 - 500.0))
            else:
                y = (i % 4 if i // 4 % 2 == 0 else 4 - i % 4) * 3.0
            coords.append(gcj02_to_bd09(30 + y / 111195.0802335329,
                                       110 + x / (111195.0802335329 * math.cos(math.radians(30)))))
        return coords

    @staticmethod
    def road_from_xy(points):
        import math
        from funsport.coordinates import gcj02_to_bd09
        return [gcj02_to_bd09(30 + y / 111195.0802335329,
                             110 + x / (111195.0802335329 * math.cos(math.radians(30))))
                for x, y in points]

    @staticmethod
    def independent_road_distance(dense, arcs, center, start, end):
        # Deliberately enumerate original knots instead of using the sampler's
        # prefix cache; count every traversal of a naturally repeated edge.
        marks = [start] + [s for s in arcs if start < s < end] + [end]
        coords = [bd09_to_gcj02(*to_bd(*ring_point_at(dense, arcs, s, open_path=True), *center))
                  for s in marks]
        return sum(distance_m(a, b) for a, b in zip(coords, coords[1:]))

    def test_sampled_distance_matches_output_matrix_and_physical_accounting(self):
        for shape in ("straight", "turns", "zigzag"):
            road = self.sampled_road(shape)
            dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
            for period in (1, 2):
                for seed in (1, 42, 20261206):
                    for requested in (300, 2001):
                        duration = round(requested * .36)
                        if (shape, period, seed, requested) == ("zigzag", 2, 1, 2001):
                            # 2026-11-07 的新速度包络（起步惯性/疲劳/方差归一）把
                            # 该固定抽样的输出弦长从 10.205m 降到 9.506m，13.2m
                            # 道路预算内已可合法建成，"此处必须报错"的前提失效。
                            # 速度守卫本身没改，改用同一 draw 的更短时长（配速
                            # 3.13 m/s 而非 2.78）：每 2s 需要更大弦长，仍必须被拒。
                            duration = 640
                        with self.subTest(shape=shape, period=period, seed=seed, distance=requested), \
                             patch.object(generator.log, "disabled", True), \
                             patch.object(generator, "_sampled_gcj_plan", wraps=generator._sampled_gcj_plan) as planner:
                            if (shape, period, seed, requested) == ("zigzag", 2, 1, 2001):
                                # Fixed draw #88 at 2.78 m/s needed a 10.205m chord
                                # while only 9.590m exists in its 13.2m road budget;
                                # at 3.13 m/s (duration 640) it still overruns, so the
                                # same fixed plan must be rejected without regeneration.
                                with self.assertRaisesRegex(ValueError, "GCJ road traversal speed budget"):
                                    generator.build(requested, duration, seed, 0, road,
                                                    ordered_path=True, open_path=True,
                                                    cadence_target=170, sample_period_s=period,
                                                    distance_mode="sampled_gcj")
                                self.assertEqual(planner.call_count, 1)
                                continue
                            track = generator.build(requested, duration, seed, 0, road,
                                                    ordered_path=True, open_path=True,
                                                    cadence_target=170, sample_period_s=period,
                                                    distance_mode="sampled_gcj")
                            points = track["locations"]
                            coords = [bd09_to_gcj02(p["gLat"], p["gLng"]) for p in points]
                            measured = traversed = 0.0
                            for i, (a, b) in enumerate(zip(points, points[1:])):
                                dd = distance_m(coords[i], coords[i + 1])
                                measured += dd
                                self.assertAlmostEqual(b["totalDis"], measured, delta=.0001)
                                self.assertGreater(b["pathProgressM"], a["pathProgressM"])
                                self.assertGreater(b["totalTime"], a["totalTime"])
                                self.assertGreaterEqual(b["steps"], a["steps"])
                                self.assertLessEqual(dd, 6.6 * (b["totalTime"] - a["totalTime"]))
                                along = self.independent_road_distance(
                                    dense, arcs, center, a["pathProgressM"], b["pathProgressM"])
                                traversed += along
                                self.assertLessEqual(along, 6.6 * (b["totalTime"] - a["totalTime"]) + 1e-7)
                            self.assertAlmostEqual(measured, requested, delta=.25)
                            self.assertAlmostEqual(track["totalDistance"], measured, delta=.001)
                            self.assertEqual(track["totalTime"], duration)
                            self.assertEqual(points[-1]["steps"], track["totalSteps"])
                            self.assertAlmostEqual(track["totalSteps"] / duration * 60, 170, delta=.6)
                            audit = track["distanceAudit"]
                            self.assertAlmostEqual(audit["road_gcj_m"], traversed, delta=1e-6)
                            self.assertAlmostEqual(audit["road_minus_sampled_gcj_m"],
                                                   traversed - measured, delta=1e-6)
                            self.assertEqual(audit["kinematics_distance_basis"], "sampled_gcj_chords")
                            self.assertLessEqual(audit["max_road_gcj_speed_mps"], 6.6)
                            self.assertEqual(planner.call_count, 1)
                            dts = planner.call_args.args[4]
                            self.assertEqual(len(points), len(dts) + 1)
                            self.assertEqual([b["totalTime"] - a["totalTime"]
                                              for a, b in zip(points, points[1:])], dts)
                            self.assertLessEqual(audit["path_progress_m"], audit["available_bd_arc_m"])
                            self.assertLessEqual(audit["coordinate_evaluations"], (8192 + 25) * (len(points) - 1))
                            self.assertAlmostEqual(sum(w["time"] for w in track["speedPerTenSec"]), duration)

    def test_legacy_arc_chord_loss_reproduces_without_jitter_and_defaults_unchanged(self):
        from funsport.coordinates import bd09_to_gcj02
        road = self.sampled_road("zigzag")
        kwargs = dict(ordered_path=True, open_path=True, cadence_target=170)
        with patch.object(generator.log, "disabled", True), \
             patch.object(generator.realism, "jitter_series", side_effect=lambda n, rng: [0.0] * n):
            legacy = generator.build(2001, 720, 20261206, 0, road, **kwargs)
            explicit = generator.build(2001, 720, 20261206, 0, road, distance_mode="arc", **kwargs)
            corrected = generator.build(2001, 720, 20261206, 0, road, distance_mode="sampled_gcj", **kwargs)
        def measured(track):
            coords = [bd09_to_gcj02(p["gLat"], p["gLng"]) for p in track["locations"]]
            return sum(distance_m(a, b) for a, b in zip(coords, coords[1:]))
        self.assertEqual(legacy, explicit)
        self.assertLess(measured(legacy), 1950)
        self.assertAlmostEqual(measured(corrected), 2001, delta=.25)
        self.assertGreater(corrected["distanceAudit"]["path_progress_m"], 2001)

    def test_sampled_distance_fails_on_short_road_without_inventing_extension(self):
        road = self.sampled_road("zigzag", length=1500)
        with patch.object(generator.log, "disabled", True), \
             self.assertRaisesRegex(ValueError, "more mapped forward road|道路折线不足|GCJ road traversal speed budget"):
            generator.build(2001, 720, 20261206, 0, road, ordered_path=True,
                            open_path=True, distance_mode="sampled_gcj")

    def test_sampled_distance_is_deterministic_and_rejects_ambiguous_modes(self):
        road = self.sampled_road("turns")
        kwargs = dict(ordered_path=True, open_path=True, distance_mode="sampled_gcj")
        with patch.object(generator.log, "disabled", True):
            a = generator.build(300, 108, 42, 0, road, **kwargs)
            b = generator.build(300, 108, 42, 0, road, **kwargs)
            self.assertEqual(a, b)
            for changes in ({"distance_mode": "other"}, {"open_path": False},
                            {"checkpoints_bd": road[:1]}, {"dur": 108.5}):
                options = dict(kwargs, **changes)
                duration = options.pop("dur", 108)
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    generator.build(300, duration, 42, 0, road, **options)

    def test_sampled_hairpin_and_tight_snake_cannot_hide_fast_road_traversal(self):
        hairpin = [(0, 0), (5, 0), (5, 1), (0, 1), (0, 2), (30, 2)]
        snake = [(0, 0)]
        for row in range(20):
            x = 5 if row % 2 == 0 else 0
            snake.append((x, row * .5))
            if row < 19:
                snake.append((x, (row + 1) * .5))
        self.assertEqual(len(snake), len(set(snake)))
        for shape in (hairpin, snake):
            road = self.road_from_xy(shape)
            dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
            # Before this fix both returned a ~6m output chord after >13.2m
            # of original GCJ road in two seconds, without any jitter.
            with self.subTest(shape=shape[:4]), self.assertRaisesRegex(
                    ValueError, "GCJ road traversal speed budget"):
                generator._sampled_gcj_plan(dense, arcs, center, [3.0], [2], [0.0], [0.0], 6.0)
            # A feasible prefix of the same shape remains usable. Rejection is
            # caused by this fixed interval budget, not by banning bends.
            plan, _, traversals = generator._sampled_gcj_plan(
                dense, arcs, center, [2.0], [2], [0.0], [0.0], 4.0)
            self.assertLess(plan[0][0], arcs[1])
            self.assertLessEqual(traversals[0], 13.2)
            self.assertAlmostEqual(plan[0][-1], 4.0, delta=.25)

    def test_sampled_nonmonotone_chord_keeps_feasible_early_peak(self):
        # The first corner peaks between the old 0.5m search probes. A coarse
        # endpoint-only bracket misses it and can jump to a later road visit.
        road = self.road_from_xy([(0, 0), (4.2, 0), (0, .1), (30, .1)])
        dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
        start_bd = tuple(generator.round_to(v, 6) for v in to_bd(*dense[0], *center))
        corner_bd = tuple(generator.round_to(v, 6) for v in to_bd(*dense[1], *center))
        target = distance_m(bd09_to_gcj02(*start_bd), bd09_to_gcj02(*corner_bd)) - .01
        plan, _, traversals = generator._sampled_gcj_plan(
            dense, arcs, center, [target / 2], [2], [0.0], [0.0], target)
        self.assertLessEqual(plan[0][0], arcs[1] + 1e-6)
        self.assertLessEqual(traversals[0], 13.2)
        self.assertAlmostEqual(plan[0][-1], target, delta=.25)

    def test_gcj_traversal_counts_partial_segments_turns_and_repeated_edges(self):
        road = self.road_from_xy([(0, 0), (5, 0), (5, 2), (0, 2), (0, 0), (5, 0), (20, 0)])
        dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
        traversal = generator._GcjRoadTraversal(dense, arcs, center)
        for start, end in ((.2, 1.7), (.2, arcs[1] + .4),
                           (arcs[1], arcs[4]), (.3, arcs[5] + .7)):
            with self.subTest(start=start, end=end):
                expected = self.independent_road_distance(dense, arcs, center, start, end)
                self.assertAlmostEqual(traversal.distance(start, end), expected, delta=1e-7)
                limit = traversal.limit(start, expected)
                self.assertAlmostEqual(limit, end, delta=1e-5)
        repeated = traversal.distance(0, arcs[5])
        self.assertGreater(repeated, 18)
        self.assertGreater(repeated, distance_m(traversal.vertices[0], traversal.vertices[5]) + 10)
        # BD progress is a location coordinate, not the GCJ distance metric.
        self.assertGreater(abs(traversal.distance(0, arcs[-1]) - arcs[-1]), .01)

    def test_sampled_natural_road_revisit_is_legal_and_counted(self):
        road = self.road_from_xy([(0, 0), (30, 0), (30, 30), (0, 30), (0, 0), (30, 0), (60, 0)])
        dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
        speeds, dts = [3.0] * 28, [2] * 28
        plan, _, traversals = generator._sampled_gcj_plan(
            dense, arcs, center, speeds, dts, [0.0] * 28, [0.0] * 28, 168.0)
        self.assertGreater(plan[-1][0], arcs[5])
        previous = 0.0
        for row, along in zip(plan, traversals):
            self.assertGreater(row[0], previous)
            self.assertLessEqual(along, 13.2)
            self.assertAlmostEqual(along, self.independent_road_distance(
                dense, arcs, center, previous, row[0]), delta=1e-7)
            previous = row[0]
        self.assertGreater(sum(traversals), 168)

    def test_sampled_quantization_at_speed_guard_and_one_second_tail(self):
        road = self.road_from_xy([(0, 0), (100, 0)])
        dense, arcs, center = make_point_ring(road, ordered=True, open_path=True)
        for dts in ([1], [2], [2, 1]):
            speeds = [6.6] * len(dts)
            target = sum(v * dt for v, dt in zip(speeds, dts))
            with self.subTest(dts=dts):
                plan, _, traversals = generator._sampled_gcj_plan(
                    dense, arcs, center, speeds, dts, [0.0] * len(dts), [0.0] * len(dts), target)
                self.assertEqual(len(plan), len(dts))
                for row, along, dt in zip(plan, traversals, dts):
                    self.assertLessEqual(row[-1], 6.6 * dt)
                    self.assertLessEqual(along, 6.6 * dt)
                self.assertAlmostEqual(sum(row[-1] for row in plan), target, delta=.25)
        short = self.road_from_xy([(0, 0), (2, 0)])
        dense, arcs, center = make_point_ring(short, ordered=True, open_path=True)
        with self.assertRaisesRegex(ValueError, "mapped forward road"):
            generator._sampled_gcj_plan(dense, arcs, center, [3.0], [2], [0.0], [0.0], 6.0)

    def test_sampled_two_second_output_preserves_odd_duration_tail(self):
        # Force only the synthetic clock to exercise a one-second last dt.
        # Production 2s mode also permits its existing 1/3/4/6s gaps.
        with patch.object(generator.log, "disabled", True), \
             patch.object(generator.realism, "sample_interval_series", return_value=list(range(0, 109, 2))):
            track = generator.build(300, 109, 42, 0, self.sampled_road("turns"),
                                    ordered_path=True, open_path=True, sample_period_s=2,
                                    distance_mode="sampled_gcj")
        points = track["locations"]
        self.assertEqual(track["totalTime"], 109)
        self.assertEqual(points[-1]["totalTime"] - points[-2]["totalTime"], 1)
        self.assertEqual(len(points), 56)
        self.assertLessEqual(track["distanceAudit"]["max_road_gcj_speed_mps"], 6.6)
        self.assertAlmostEqual(track["distanceAudit"]["sampled_gcj_m"], 300, delta=.25)

    def test_empty_track_has_no_invented_line_or_markers(self):
        """无有效点时没有轨迹或起终点，不补造位置。"""
        self.assertEqual(map_preview.history_geometry(self.typed_track([-1, -1])),
                         {"segments": [], "start": None, "end": None})

    def test_endpoint_matches_arc_progress_after_snapping(self):
        """逐点吸附完成后，终点仍由真实弧长重算，不被附近顶点覆盖。"""
        dense, arcs, center = make_point_ring(sample_ring(), ordered=True)
        for fraction in (0.003, 0.1, 0.25, 0.99):
            with self.subTest(fraction=fraction):
                distance = arcs[-1] * (1 + fraction)
                track = self.build(distance)
                end = track["locations"][-1]
                x, y = ring_point_at(dense, arcs, distance)
                expected = to_bd(x, y, *center)
                # 坐标按真实设备的 6 位小数落盘，量化台阶在纬度方向约 0.11m、
                # 经度方向约 0.10m，半台阶合计可达 0.08m；吸附错位是米级，
                # 因此容差取 0.12m 仍能区分「按弧长重算」和「被附近顶点覆盖」。
                self.assertLess(distance_m((end["gLat"], end["gLng"]), expected), 0.12)

    def test_distance_windows_use_corrected_progress(self):
        """制造0.75m拟合残差，窗口里程仍跟随校正后的实际累计值。"""
        original_fit = generator.fit_speeds

        def undershoot(speeds, durations, target, floor=None, ceil=None):
            """只在测试中制造允许范围内的拟合偏差，检验末段校正。"""
            original_fit(speeds, durations, target, floor=floor, ceil=ceil)
            factor = (target - 0.75) / sum(speed * duration for speed, duration in zip(speeds, durations))
            for index in range(len(speeds)):
                speeds[index] *= factor

        with patch.object(generator, "fit_speeds", side_effect=undershoot):
            track = self.build(_ring_length(sample_ring()) + 50)
        windows = track["speedPerTenSec"]
        self.assertAlmostEqual(sum(window["time"] for window in windows), track["totalTime"])
        self.assertLess(abs(sum(window["value"] for window in windows) - track["totalDistance"]),
                        0.005 * len(windows) + 0.01)

    def test_one_second_tail_is_not_dropped_or_extrapolated(self):
        """不足十秒的最后一秒保留真实里程和时长，不扩大成十秒。"""
        track = self.build(1002.5)
        self.assertEqual(track["totalTime"], 401)
        windows = track["speedPerTenSec"]
        self.assertEqual(len(windows), 41)
        self.assertEqual(windows[-1]["time"], 1)
        self.assertEqual(sum(window["time"] for window in windows), 401)
        self.assertLess(abs(sum(window["value"] for window in windows) - track["totalDistance"]), 0.22)
