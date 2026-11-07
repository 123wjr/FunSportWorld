"""链路核心 funsport/api/chain.py 的离线单测（不打网络）。"""
import unittest
from unittest.mock import patch

from funsport.api import chain
from funsport.coordinates import gcj02_to_bd09


class ParseTests(unittest.TestCase):
    def test_accepts_both_orders(self):
        self.assertEqual(chain.parse_lnglat("119.5,39.9"), (39.9, 119.5))
        self.assertEqual(chain.parse_lnglat("39.9,119.5"), (39.9, 119.5))
        self.assertEqual(chain.parse_lnglat(" 119.5 , 39.9 "), (39.9, 119.5))

    def test_rejects_bad_input(self):
        for text in ("119.5", "a,b", "1,2,3", ""):
            with self.assertRaises(ValueError):
                chain.parse_lnglat(text)


class MileageTests(unittest.TestCase):
    def test_ring_is_taken_out_of_laps_not_padded(self):
        plan = chain.plan_mileage([100.0, 200.0, 100.0], 1000.0, 1500.0)
        self.assertEqual(plan["laps"], 2)
        self.assertFalse(plan["raised"])
        self.assertEqual(plan["visit_all_points_m"], 300.0)

    def test_target_shorter_than_visits_is_raised(self):
        plan = chain.plan_mileage([100.0, 200.0, 100.0], 1000.0, 120.0)
        self.assertTrue(plan["raised"])
        self.assertEqual(plan["target_m"], 300.0)
        self.assertEqual(plan["requested_m"], 120.0)

    def test_degenerate_inputs(self):
        with self.assertRaises(ValueError):
            chain.plan_mileage([100.0, 100.0], 0.0, 500.0)
        with self.assertRaises(ValueError):
            chain.plan_mileage([], 500.0, 0.0)


def square(start_lat=39.925, start_lng=119.547, side_lat=0.0009, side_lng=0.00104):
    """约 100 m 边长的小方环（GCJ-02）。"""
    return [
        (start_lat, start_lng),
        (start_lat + side_lat, start_lng),
        (start_lat + side_lat, start_lng + side_lng),
        (start_lat, start_lng + side_lng),
        (start_lat, start_lng),
    ]


class FetchChainTests(unittest.TestCase):
    def test_degenerate_first_leg_is_skipped(self):
        """起点就是第一个点位时不发 origin==destination 的腿，leg_dists 仍对齐。"""
        start = (39.925, 119.547)
        p1 = (39.926, 119.548)
        calls = []

        def fake(a, b, key):
            calls.append((a, b))
            mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            return [a, mid, b], 100.0, "v5"

        with patch.object(chain.api_loop, "_amap_walking", side_effect=fake):
            route, legs = chain.fetch_chain(start, [start, p1], key="k")
        self.assertEqual(len(calls), 2)
        self.assertEqual(legs, [0.0, 100.0, 100.0])
        self.assertGreater(len(route), 3)


class RouteTests(unittest.TestCase):
    def test_ring_length_is_positive(self):
        ring_bd = [gcj02_to_bd09(lat, lng) for lat, lng in square()]
        self.assertGreater(chain.ring_length(ring_bd), 300.0)

    def test_track_points_carry_gait_and_gcj(self):
        track = {
            "totalTime": 4.0, "totalDistance": 10.0,
            "locations": [
                {"totalTime": 0.0, "totalDis": 0.0, "steps": 0,
                 "gLat": 39.925, "gLng": 119.547},
                {"totalTime": 2.0, "totalDis": 5.0, "steps": 3,
                 "gLat": 39.9251, "gLng": 119.5471},
                {"totalTime": 4.0, "totalDis": 10.0, "steps": 6,
                 "gLat": 39.9252, "gLng": 119.5472},
            ],
        }
        pts, total = chain.track_to_points(track)
        self.assertEqual(total, 10.0)
        self.assertEqual(len(pts), 2)
        self.assertAlmostEqual(pts[0]["v"], 2.5, places=4)
        self.assertAlmostEqual(pts[0]["cadence"], 90.0, places=4)
        self.assertAlmostEqual(pts[0]["stride_m"], 60.0 * 2.5 / 90.0, places=4)
        # 落盘 BD -> 出图 GCJ，坐标必须明显不同（本点相差 ~500 m 量级）
        self.assertLess(abs(pts[0]["lat"] - 39.9251), 0.02)
        self.assertNotAlmostEqual(pts[0]["lat"], 39.9251, places=6)
        self.assertEqual(pts[0]["corner_radius_m"], None)
        self.assertFalse(pts[0]["u_turn"])

    def test_summarize_flags_short_slow_run(self):
        pts = [{"v": 3.0, "cadence": 168.0, "stride_m": 1.0, "u_turn": False}
               for _ in range(8)]
        pts[3]["v"] = 1.0
        pts[4]["v"] = 1.0
        stat = chain.summarize(pts)
        self.assertEqual(stat["max_low_speed_run"], 2)
        self.assertLess(stat["v_min"], stat["v_mean"])

    def test_overlay_document_from_track(self):
        track = {
            "totalTime": 4.0, "totalDistance": 10.0,
            "locations": [
                {"totalTime": 0.0, "totalDis": 0.0, "steps": 0,
                 "gLat": 39.925, "gLng": 119.547},
                {"totalTime": 2.0, "totalDis": 5.0, "steps": 3,
                 "gLat": 39.9251, "gLng": 119.5471},
            ],
        }
        doc = chain.overlay_document_from_track(track, name="demo")
        self.assertEqual(doc["name"], "demo")
        self.assertEqual(doc["meta"]["schema"], chain.SCHEMA)
        self.assertEqual(len(doc["points"]), 1)
        self.assertEqual(doc["route"]["track"], [])

    def test_build_chain_route_from_cache(self):
        pts_gcj = square()
        chain_gcj = pts_gcj
        leg_dists = [100.0, 300.0, 100.0]
        res = chain.build_chain_route(
            (39.925, 119.547), pts_gcj[:3], target_m=400.0, pace_s=300.0,
            seed=7, start_ms=1762502400000, chain_gcj=chain_gcj,
            leg_dists=leg_dists)
        self.assertAlmostEqual(res["dist_m"], 400.0, delta=1.0)
        self.assertGreater(len(res["points"]), 10)
        self.assertGreaterEqual(res["chain"]["laps"], 1)
        self.assertFalse(res["chain"]["raised_to_visit"])
        doc = chain.overlay_document(res)
        self.assertEqual(doc["meta"]["schema"], chain.SCHEMA)
        self.assertGreater(len(doc["route"]["track"]), 3)
        self.assertTrue(doc["points"][0]["t"] > 0)


if __name__ == "__main__":
    unittest.main()
