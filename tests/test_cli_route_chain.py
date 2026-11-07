"""funsport route 子命令接链路规则的离线单测：不联网、只读临时目录。"""
import io
import json
import unittest
from contextlib import redirect_stderr, redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch

from tests.support import IsolatedCase
from funsport.api import chain
from funsport.cli import cmds
from funsport.main import main

# 虚构点位（GCJ-02 的 (lat, lng)），测试不读真实点位缓存。
CACHE_POINTS = [(30.601000, 104.101000), (30.602000, 104.102000)]
POINT_ARGS = ["104.101000,30.601000", "104.102000,30.602000"]


def minimal_result():
    """build_chain_route 的最小替身：字段与 funsport/api/chain.py 的返回值同名。"""
    return {
        "chain": {"lap_len_m": 1000.0, "humanized_len_m": 1010.0,
                  "visit_all_points_m": 600.0, "requested_m": 2000.0,
                  "target_m": 2000.0, "laps": 2, "raised_to_visit": False,
                  "start_gcj": "30.600000,104.100000", "end_offset_m": 3.5},
        "points": [{"t": 2.0, "dist_m": 5.0, "lat": 30.6001, "lng": 104.1001,
                    "v": 2.8, "cadence": 165.0, "stride_m": 1.02,
                    "u_turn": False, "corner_radius_m": None}],
        "dist_m": 1999.0, "dur_s": 720.0,
        "stats": {"v_mean": 2.78, "v_cv": 0.05, "cadence_mean": 165.0,
                  "stride_mean": 1.02, "max_low_speed_run": 0,
                  "cadence_elasticity": None},
    }


class RouteCliCase(IsolatedCase):
    """为 route 子命令准备独立输出目录，并收集命令行参数。"""

    def args(self, **overrides):
        """构造 argparse 命名空间，默认只写 JSON 到临时目录。"""
        values = {"start": None, "point": None, "dist": 0.0, "target": None,
                  "pace": 360.0, "cadence": 0.0, "seed": 7, "start_ms": 0,
                  "json": str(self.data / "cli_route.json"),
                  "html": str(self.data / "cli_route.html"),
                  "no_html": True, "title": None}
        values.update(overrides)
        return SimpleNamespace(**values)

    def invoke(self, args):
        """执行 cmd_route 并捕获标准输出，返回 (退出码, 输出)。"""
        stream = io.StringIO()
        with redirect_stdout(stream):
            code = cmds.cmd_route(args)
        return code, stream.getvalue()

    def build_with_stub(self, **overrides):
        """替换实际生成入口，返回 (退出码, 输出, 收到的参数)。"""
        captured = {}

        def fake(start, points, **options):
            captured["start"] = start
            captured["points"] = list(points)
            captured.update(options)
            return minimal_result()

        with patch.object(chain, "build_chain_route", side_effect=fake) as build:
            code, output = self.invoke(self.args(**overrides))
        captured["called"] = build.call_count
        return code, output, captured


class TargetTests(RouteCliCase):
    def test_dist_km_and_meter_mean_the_same_target(self):
        """--dist 口径与 run 一致：<100 按 km，>=100 按米。"""
        for dist in (2.0, 2000.0):
            with self.subTest(dist=dist), \
                    patch.object(chain, "points_from_cache_gcj",
                                 return_value=list(CACHE_POINTS)):
                code, _output, captured = self.build_with_stub(dist=dist)
            self.assertEqual(code, 0)
            self.assertAlmostEqual(captured["target_m"], 2000.0)

    def test_target_argument_overrides_dist(self):
        """--target 的单位是米，优先级高于 --dist。"""
        with patch.object(chain, "points_from_cache_gcj",
                          return_value=list(CACHE_POINTS)):
            code, _output, captured = self.build_with_stub(dist=2.0, target=1500.0)
        self.assertEqual(code, 0)
        self.assertAlmostEqual(captured["target_m"], 1500.0)


class PointSourceTests(RouteCliCase):
    def test_point_arguments_skip_cache_and_set_default_start(self):
        """给了 --point 就不读缓存；缺省起点是第一个点位。"""
        with patch.object(chain, "points_from_cache_gcj",
                          side_effect=AssertionError("给了 --point 不能读缓存")):
            code, _output, captured = self.build_with_stub(point=list(POINT_ARGS))
        self.assertEqual(code, 0)
        self.assertEqual(captured["points"], [(30.6010, 104.1010), (30.6020, 104.1020)])
        self.assertEqual(captured["start"], (30.6010, 104.1010))

    def test_without_point_arguments_reads_cache(self):
        """不给 --point 时退回点位缓存，起点仍是第一个点位。"""
        with patch.object(chain, "points_from_cache_gcj",
                          return_value=list(CACHE_POINTS)) as cache:
            code, _output, captured = self.build_with_stub()
        self.assertEqual(code, 0)
        cache.assert_called_once_with()
        self.assertEqual(captured["points"], list(CACHE_POINTS))
        self.assertEqual(captured["start"], CACHE_POINTS[0])

    def test_start_argument_overrides_first_point(self):
        """--start 是 GCJ-02 lng,lat，接受反序。"""
        with patch.object(chain, "points_from_cache_gcj",
                          return_value=list(CACHE_POINTS)):
            code, _output, captured = self.build_with_stub(start="104.109,30.609")
        self.assertEqual(code, 0)
        self.assertEqual(captured["start"], (30.609, 104.109))


class WalkthroughTests(RouteCliCase):
    def test_route_writes_json_and_reports_chain(self):
        """跑通一次：打印链路/走完点位，--no-html 时只落 JSON。"""
        json_path = self.data / "out" / "route.json"
        html_path = self.data / "out" / "route.html"
        with patch.object(chain, "build_chain_route", return_value=minimal_result()):
            code, output = self.invoke(self.args(point=list(POINT_ARGS),
                                                 json=str(json_path), html=str(html_path),
                                                 no_html=True))
        self.assertEqual(code, 0)
        self.assertIn("走完点位", output)
        self.assertIn("链路", output)
        self.assertTrue(json_path.exists())
        self.assertFalse(html_path.exists())
        payload = json.loads(json_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["series"][0]["meta"]["schema"], chain.SCHEMA)

    def test_html_path_is_passed_unless_no_html(self):
        """默认把 HTML 路径交给写盘函数；--no-html 时才传 None。"""
        json_path = self.data / "cli.json"
        html_path = self.data / "cli.html"
        with patch.object(chain, "build_chain_route", return_value=minimal_result()), \
                patch.object(chain, "write_overlay",
                             return_value=(json_path, html_path)) as writer:
            code, _output = self.invoke(self.args(point=list(POINT_ARGS),
                                                  json=str(json_path), html=str(html_path),
                                                  no_html=False))
        self.assertEqual(code, 0)
        self.assertEqual(writer.call_args.args[1], str(json_path))
        self.assertEqual(writer.call_args.args[2], str(html_path))

    def test_raised_target_is_announced(self):
        """目标短于走完点位时按后者生成，并把这句话打出来。"""
        result = minimal_result()
        result["chain"].update(raised_to_visit=True, requested_m=500.0,
                               target_m=600.0, visit_all_points_m=600.0)
        with patch.object(chain, "build_chain_route", return_value=result):
            code, output = self.invoke(self.args(point=list(POINT_ARGS)))
        self.assertEqual(code, 0)
        self.assertIn("目标 500 m 短于走完所有点位所需的 600 m，按 600 m 生成", output)


class ErrorTests(RouteCliCase):
    def test_pace_out_of_range_returns_1(self):
        """配速越界在生成前就被拒绝。"""
        with patch.object(chain, "build_chain_route") as build:
            code, output = self.invoke(self.args(pace=100.0))
        self.assertEqual(code, 1)
        self.assertIn("配速", output)
        build.assert_not_called()

    def test_missing_points_returns_1_without_raising(self):
        """没有点位也没有缓存：cmd_route 返回 1，不抛未捕获异常。"""
        with patch.object(chain, "points_from_cache_gcj",
                          side_effect=RuntimeError("没有点位缓存")):
            code, output = self.invoke(self.args())
        self.assertEqual(code, 1)
        self.assertIn("没有点位缓存", output)

    def test_cli_exits_1_without_traceback_when_no_points(self):
        """整条 CLI 路径同样退出码 1，且不落到 traceback 处理器。"""
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(chain, "points_from_cache_gcj",
                          side_effect=RuntimeError("没有点位缓存")), \
                redirect_stdout(stdout), redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                main(["route"])
        self.assertEqual(raised.exception.code, 1)
        self.assertNotIn("Traceback", stdout.getvalue() + stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
