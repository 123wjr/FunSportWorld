"""链路规则：目标里程不足「走完所有点位」时按后者生成；坐标口径统一走 checkpoint_coordinates。"""

import unittest

from funsport.api import flow


class ChainRuleTests(unittest.TestCase):
    def test_visit_all_points_stops_at_last_point(self):
        # 边长 100 m 的正方形环（首尾闭合），最后一个点位在第 300 m 处
        ring = [(30.0, 120.0), (30.0, 120.00104), (30.0009, 120.00104), (30.0009, 120.0), (30.0, 120.0)]
        points = [(30.0, 120.0), (30.0, 120.00104), (30.0009, 120.00104)]
        visited = flow._visit_all_points_m(ring, points)
        self.assertGreater(visited, 150.0)
        self.assertLess(visited, 480.0)

    def test_visit_all_points_zero_when_nothing_to_walk(self):
        ring = [(30.0, 120.0), (31.0, 121.0), (30.0, 120.0)]
        self.assertEqual(flow._visit_all_points_m(ring, [(30.0, 120.0)]), 0.0)
        self.assertEqual(flow._visit_all_points_m(ring[:1], [(30.0, 120.0)]), 0.0)

    def test_visit_ignores_return_leg(self):
        ring = [(30.0, 120.0), (30.0, 120.00104), (30.0, 120.0)]
        points = [(30.0, 120.0), (30.0, 120.00104)]
        visited = flow._visit_all_points_m(ring, points)
        self.assertGreater(visited, 100.0)
        self.assertLess(visited, 220.0)

    def test_as_latlon_accepts_tuple_dict_and_gcj(self):
        self.assertEqual(flow._as_latlon((30.5, 120.25)), (30.5, 120.25))
        self.assertEqual(flow._as_latlon({"lat": 30.5, "lon": 120.25}), (30.5, 120.25))
        bd = flow._as_latlon({"glat": 39.925029, "glon": 119.547939})
        self.assertNotAlmostEqual(bd[0], 39.925029, places=4)
        self.assertNotAlmostEqual(bd[1], 119.547939, places=4)

    def test_visit_matches_visited_points_after_conversion(self):
        # 用 GCJ 点位换算出的 BD 点位应能命中 BD 环
        from funsport.coordinates import gcj02_to_bd09
        ring_gcj = [(30.0, 120.0), (30.0, 120.00104), (30.0009, 120.00104), (30.0009, 120.0), (30.0, 120.0)]
        ring = [gcj02_to_bd09(*p) for p in ring_gcj]
        points = [{"glat": 30.0, "glon": 120.0}, {"glat": 30.0, "glon": 120.00104}]
        visited = flow._visit_all_points_m(ring, points)
        self.assertGreater(visited, 12.0)


if __name__ == "__main__":
    unittest.main()