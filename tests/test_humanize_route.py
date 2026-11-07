#!/usr/bin/env python3
"""funsport/track/humanize.py 的回归测试（unittest，零依赖、不联网、不读 .funsport/）。

只断言可用合成几何验证的性质：转角被圆滑、环闭合、偏离有界、掉头成弧、同种子可复现。
"""
from __future__ import annotations

import math
import unittest

from funsport.track import humanize as hz

ORIGIN = (30.6, 104.1)
M_PER_DEG_LAT = 111132.0


def bd_from_xy(x, y):
    """把以米为单位的局部平面坐标放回 BD09 经纬度（只用于合成用例）。"""
    m_lng = 111320.0 * math.cos(math.radians(ORIGIN[0]))
    return (ORIGIN[0] + y / M_PER_DEG_LAT, ORIGIN[1] + x / m_lng)


def square_ring(side=200.0):
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    return [bd_from_xy(x, y) for x, y in corners]


def retrace_ring(length=300.0, width=40.0):
    """沿同一条走廊原路往返的环：去程与回程完全重合，含一个 180° 掉头。"""
    return [bd_from_xy(0.0, 0.0), bd_from_xy(length, 0.0),
            bd_from_xy(0.0, 0.0), bd_from_xy(0.0, width)]


class HumanizeRouteTests(unittest.TestCase):
    def test_rounded_corners_remove_sharp_turns(self):
        """90° 的方角环经过拟人化后不应再出现任何大于 45° 的单点转角。"""
        road = square_ring(200.0)
        before = hz.analyze_route(road)
        out, info = hz.humanize_route(road, seed=7)
        after = hz.analyze_route(out)
        self.assertGreaterEqual(info["corners_rounded"], 4)
        self.assertEqual(before["n_turn_gt_45"], 4)
        self.assertEqual(after["n_turn_gt_45"], 0)
        self.assertLess(after["turn_max_deg"], 45.0)
        self.assertGreater(after["min_radius_m"], 2.0)

    def test_length_and_closure_are_preserved(self):
        """圆滑只切角、不改变里程量级，且输出仍是一个闭合环。"""
        road = square_ring(200.0)
        out, _ = hz.humanize_route(road, seed=1)
        stats = hz.analyze_route(out)
        self.assertAlmostEqual(stats["length_m"], 800.0, delta=40.0)
        # 循环点列的隐式闭合边不应显著长于普通相邻点间距。
        self.assertLessEqual(stats["seam_ratio"], 2.0)
        self.assertTrue(stats["closure_ok"])

    def test_deviation_from_road_is_bounded(self):
        """直线摆动与靠右偏移叠加后，仍应贴着原本的道路（校园场景允许大胆漂移，但有界）。"""
        road = square_ring(200.0)
        out, info = hz.humanize_route(road, seed=3)
        stats = hz.analyze_route(out, reference_bd=road)
        self.assertLess(stats["deviation_m"]["p95"], 8.0)
        self.assertLess(stats["deviation_m"]["max"], 15.0)
        self.assertLessEqual(info["offset_max_m"], 6.0)

    def test_retrace_corridor_is_separated_by_lane_offset(self):
        """原路往返的重合走廊应被靠右行驶分成两条平行车道，掉头处形成一个半径合理的弧。"""
        road = retrace_ring()
        before = hz.analyze_route(road)
        out, info = hz.humanize_route(road, seed=7)
        after = hz.analyze_route(out)
        self.assertGreater(before["repeat"]["ratio"], 0.3)
        self.assertLess(after["repeat"]["ratio"], 0.05)
        self.assertGreaterEqual(len(info["uturns"]), 1)
        for item in info["uturns"]:
            self.assertGreater(item["turn_deg"], 150.0)
            self.assertAlmostEqual(item["radius_m"], hz.DEFAULTS["u_turn_radius_m"], delta=1e-6)
        self.assertGreater(after["min_radius_m"], 0.6)

    def test_seed_is_reproducible_and_matters(self):
        """同种子必须逐点一致（可复现），不同种子必须给出不同形状（不是伪随机装饰）。"""
        road = square_ring(200.0)
        a, _ = hz.humanize_route(road, seed=7)
        b, _ = hz.humanize_route(road, seed=7)
        c, _ = hz.humanize_route(road, seed=8)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)

    def test_degenerate_input_does_not_crash(self):
        """少于 4 个点、或三点共线的退化输入必须原样/安全返回，不能抛异常。"""
        two = [bd_from_xy(0.0, 0.0), bd_from_xy(10.0, 0.0)]
        out, info = hz.humanize_route(two, seed=1)
        self.assertIsInstance(info, dict)
        self.assertGreaterEqual(len(out), 2)
        self.assertTrue(all(math.isfinite(v) for p in out for v in p))
        # 起点不应被甩离原位置太远：偏移上限约为 lane_offset + 摆动幅度。
        self.assertLess(math.hypot((out[0][0] - two[0][0]) * M_PER_DEG_LAT,
                                   (out[0][1] - two[0][1]) * 111320.0 * math.cos(math.radians(two[0][0]))), 10.0)
        collinear = [bd_from_xy(0.0, 0.0), bd_from_xy(10.0, 0.0), bd_from_xy(20.0, 0.0)]
        out2, _ = hz.humanize_route(collinear, seed=1)
        self.assertGreaterEqual(len(out2), 3)


if __name__ == "__main__":
    unittest.main()
