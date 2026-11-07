"""真实化采样空档与里程自洽的离线回归（不联网、不读账号数据）。"""
import math
from unittest.mock import patch

from tests.plan_fixture import sample_ring
from tests.support import IsolatedCase
from funsport.api import campus_loop as loop, flow
from funsport.api.flow import _ring_length
from funsport.coordinates import bd09_to_gcj02, distance_m
from funsport.track import generator, realism
from funsport.track.geom import SPEED_CEIL, Rng


def circle_ring(radius_m=100.0, count=180):
    """按米制半径生成近似正圆的闭环，便于精确核查里程与圈数。"""
    points = []
    for index in range(count):
        angle = math.tau * index / count
        points.append((30.6 + radius_m * math.sin(angle) / 111132.0,
                       104.1 + radius_m * math.cos(angle) / 86600.0))
    points.append(points[0])
    return points


class RealismSamplingTests(IsolatedCase):
    """定位丢失与末段不得制造长时间采样空档。"""

    def build(self, distance, duration, ring, seed=42):
        """静音日志后按有序模式生成轨迹。"""
        with patch.object(generator.log, "disabled", True):
            return generator.build(distance, duration, seed, 1000, ring,
                                   ordered_path=True, cadence_target=168)

    @staticmethod
    def gaps(track):
        """逐点间隔秒数。"""
        points = track["locations"]
        return [later["totalTime"] - earlier["totalTime"]
                for earlier, later in zip(points, points[1:])]

    @staticmethod
    def coordinate_length(track):
        """按落盘坐标累计里程（与请求里程同一 CRS）。"""
        points = [(point["gLat"], point["gLng"]) for point in track["locations"]
                  if point["type"] != -1]
        return sum(distance_m(earlier, later) for earlier, later in zip(points, points[1:]))

    def test_sampler_never_returns_a_gap_longer_than_the_cap(self):
        """丢失定位只漏一个采样点，不产生 25~150s 的真空，也不丢末段。"""
        for duration in (60, 245, 586, 1200, 3600):
            for seed in range(60):
                with self.subTest(duration=duration, seed=seed):
                    times = realism.sample_interval_series(duration, Rng(seed))
                    self.assertEqual(times[0], 0.0)
                    self.assertEqual(times, sorted(times))
                    self.assertLessEqual(times[-1], duration)
                    self.assertLessEqual(duration - times[-1], realism.MAX_SAMPLE_GAP)
                    for earlier, later in zip(times, times[1:]):
                        self.assertGreater(later - earlier, 0.0)
                        self.assertLessEqual(later - earlier, realism.MAX_SAMPLE_GAP)

    def test_long_run_has_no_teleport_and_keeps_requested_distance(self):
        """多圈长跑逐点间隔不超过上限，坐标累计里程贴近请求值。"""
        ring = circle_ring()
        length = _ring_length(ring)
        for fraction in (1.0, 2.2, 3.7):
            distance = length * fraction + 50
            duration = int(round(distance / 2.5))
            with self.subTest(fraction=fraction):
                track = self.build(distance, duration, ring)
                self.assertAlmostEqual(track["totalDistance"], distance, places=2)
                self.assertEqual(track["totalTime"], duration)
                self.assertLessEqual(max(self.gaps(track)), realism.MAX_SAMPLE_GAP + 0.5)
                ratio = self.coordinate_length(track) / distance
                self.assertLess(abs(ratio - 1.0), 0.03)

    def test_short_run_and_turns_stay_within_physical_envelope(self):
        """短程与 90 度折角闭环同样满足间隔、速度与里程一致性。"""
        corner = [(30.6, 104.1), (30.6015, 104.1), (30.6015, 104.1022), (30.6, 104.1022), (30.6, 104.1)]
        for ring, distance in ((sample_ring(), _ring_length(sample_ring()) * 0.35 + 50),
                               (corner, _ring_length(corner) * 2.4 + 50)):
            duration = int(round(distance / 2.5))
            with self.subTest(distance=distance):
                track = self.build(distance, duration, ring)
                points = track["locations"][1:]
                self.assertLessEqual(max(self.gaps(track)), realism.MAX_SAMPLE_GAP + 0.5)
                for earlier, later in zip(points, points[1:]):
                    speed = distance_m((earlier["gLat"], earlier["gLng"]),
                                       (later["gLat"], later["gLng"])) / (
                        later["totalTime"] - earlier["totalTime"])
                    self.assertLessEqual(speed, SPEED_CEIL + 0.3)
                self.assertLess(abs(self.coordinate_length(track) / distance - 1.0), 0.03)

    def test_non_integral_tail_keeps_time_distance_and_gain_time_in_sync(self):
        """末段不足一秒也保留：总时长不凑整，逐点 gainTimeMs 与 totalTime 一致。"""
        ring = sample_ring()
        distance = _ring_length(ring) + 50
        track = self.build(distance, 401, ring, seed=7)
        self.assertEqual(track["totalTime"], 401)
        points = track["locations"]
        for point in points:
            self.assertEqual(point["gainTimeMs"], track["startTime"] + point["totalTime"] * 1000)
        self.assertEqual(points[-1]["totalTime"], track["totalTime"])
        self.assertAlmostEqual(points[-1]["totalDis"], track["totalDistance"], places=2)
        self.assertAlmostEqual(sum(self.gaps(track)), track["totalTime"], places=6)

    def test_declared_distance_matches_speed_windows_and_point_clock(self):
        """声明里程、十秒速度窗与逐点时钟三者自洽，不靠改总时长凑整。"""
        ring = circle_ring()
        distance = _ring_length(ring) * 2.2 + 50
        track = self.build(distance, 602, ring, seed=11)
        windows = track["speedPerTenSec"]
        # value 落盘保留两位小数，n 个窗最多引入 0.005*n 的舍入差。
        self.assertAlmostEqual(sum(window["value"] for window in windows),
                               track["totalDistance"], delta=0.005 * len(windows) + 0.01)
        self.assertEqual(sum(window["time"] for window in windows), track["totalTime"])
        self.assertEqual(track["totalTime"], 602)
        self.assertAlmostEqual(track["totalDistance"], distance, places=2)
        self.assertLessEqual(track["validDistance"], track["totalDistance"])
        self.assertLessEqual(track["validTime"], track["totalTime"])

    def test_same_seed_reproduces_identical_geometry(self):
        """收缩抖动不重新抽样，因此同种子逐点坐标仍完全一致。"""
        ring = circle_ring()
        length = _ring_length(ring)
        first = self.build(length * 2.2 + 50, 602, ring, seed=3)
        second = self.build(length * 2.2 + 50, 602, ring, seed=3)
        self.assertEqual([(point["gLat"], point["gLng"]) for point in first["locations"]],
                         [(point["gLat"], point["gLng"]) for point in second["locations"]])

    def test_speed_cap_holds_on_the_landed_coordinates(self):
        """速度上限必须按**落盘坐标**成立，不能只在浮点位置上通过。

        验收链路是「6 位小数 BD 坐标 → BD→GCJ02 → 球面距离 / 逐点间隔」，它与
        生成侧内部的平面近似系统地差 0.2%~0.5%。历史上只在浮点位置上钳 6.6，
        10 例 × 30 seed 里有 4 行落盘后算成 6.605~6.621。
        """
        ring = circle_ring()
        length = _ring_length(ring)
        distance = length * 2.2 + 50
        duration = int(round(distance / 2.5))
        for seed in (1, 18, 19, 22):
            track = self.build(distance, duration, ring, seed=seed)
            points = track["locations"]
            for earlier, later in zip(points, points[1:]):
                dt = later["totalTime"] - earlier["totalTime"]
                if dt <= 0:
                    continue
                speed = distance_m((earlier["gLat"], earlier["gLng"]),
                                   (later["gLat"], later["gLng"])) / dt
                with self.subTest(seed=seed, point=later["id"]):
                    self.assertLessEqual(speed, realism.SPEED_CEIL_POINT)
            self.assertLess(abs(sum(distance_m((a["gLat"], a["gLng"]), (b["gLat"], b["gLng"]))
                                    for a, b in zip(points, points[1:])) / track["totalDistance"] - 1.0), 0.03)

    def test_acceptance_clock_and_crs_hold_on_non_integral_durations(self):
        """落盘时钟是整秒、验收还多做一次 BD→GCJ02 取 6 位，限速必须在两者上都成立。

        历史上钳制用的是采样器的浮点 dt 与 BD 坐标：dur=401 的折角闭环里有两条
        采样间隔落到 1s（浮点 1.9995s），按浮点判 6.6 通过、按落盘整秒算成 6.604；
        换成验收 CRS 又叠一层 0.2%~0.5% 的偏差。
        """
        corner = [(30.6, 104.1), (30.6015, 104.1), (30.6015, 104.1022),
                  (30.6, 104.1022), (30.6, 104.1)]
        distance = _ring_length(corner) * 2.4 + 50
        for duration in (401, 599):
            for seed in range(1, 31):
                with self.subTest(duration=duration, seed=seed):
                    track = self.build(distance, duration, corner, seed=seed)
                    points = track["locations"]
                    self.assertEqual(track["totalTime"], duration)
                    self.assertEqual(points[-1]["totalTime"], duration)
                    self.assertLessEqual(max(self.gaps(track)), realism.MAX_SAMPLE_GAP)
                    landed = [(point["gLat"], point["gLng"]) for point in points]
                    # 验收链路：落盘 BD → BD09→GCJ02 → 6 位小数 → 球面距离。
                    accepted = [(round(coords[0], realism.COORD_DECIMALS),
                                 round(coords[1], realism.COORD_DECIMALS))
                                for coords in (bd09_to_gcj02(*pair) for pair in landed)]
                    for index in range(1, len(points)):
                        dt = points[index]["totalTime"] - points[index - 1]["totalTime"]
                        if dt <= 0:
                            continue
                        speed = distance_m(accepted[index - 1], accepted[index]) / dt
                        self.assertLessEqual(speed, realism.SPEED_CEIL_POINT,
                                             f"duration={duration} seed={seed} "
                                             f"id={points[index]['id']}")
                    traveled = sum(distance_m(earlier, later)
                                   for earlier, later in zip(accepted, accepted[1:]))
                    self.assertLess(abs(traveled / track["totalDistance"] - 1.0), 0.03)

    def test_legacy_fit_mode_does_not_fabricate_a_stop_gap(self):
        """旧拟合模式（非有序）不再注入 12~16s 的「跑者停下」尾段空档。

        该后处理过去在末个有效点上做 totalTime += randint(6, 14)，把采样间隔
        拉到 12~16s 并让 Σdt 超过总时长，与生产链路同一条验收线冲突。
        速度用 gainTimeMs 时钟度量（验收口径），覆盖两类旧缺陷：
        首点即无效定位时坐标被甩到全局原点、无效点后一瞬间挪回整段位移。
        """
        ring = circle_ring()
        length = _ring_length(ring)
        distance = length * 2.0 + 50
        duration = int(round(distance / 2.5))
        for seed in range(1, 31):
            with patch.object(generator.log, "disabled", True):
                track = generator.build(distance, duration, seed, 1000, ring,
                                        ordered_path=False, cadence_target=168)
            points = track["locations"]
            clock = [(point["gainTimeMs"] - track["startTime"]) / 1000.0 for point in points]
            with self.subTest(seed=seed):
                self.assertLessEqual(max(self.gaps(track)), realism.MAX_SAMPLE_GAP)
                self.assertEqual(points[-1]["totalTime"], track["totalTime"])
                self.assertEqual(track["totalTime"], duration)
                for index in range(1, len(points)):
                    dt = clock[index] - clock[index - 1]
                    if dt <= 0:
                        continue
                    speed = distance_m((points[index - 1]["gLat"], points[index - 1]["gLng"]),
                                       (points[index]["gLat"], points[index]["gLng"])) / dt
                    self.assertLessEqual(speed, realism.SPEED_CEIL_POINT, f"seed={seed} id={points[index]['id']}")

class CachedRouteGenerationTests(IsolatedCase):
    """缓存选路后的实际生成路径同样不得出现长空档或里程漂移。"""

    @staticmethod
    def checkpoints():
        """六个虚构打卡点构成的闭环，不含真实校园坐标。"""
        points = []
        for index in range(6):
            angle = math.tau * index / 6
            points.append((30.6 + 120.0 * math.sin(angle) / 111132.0,
                           104.1 + 120.0 * math.cos(angle) / 86600.0))
        return points

    def payload(self):
        """按官方点位结构包装打卡点。"""
        return [{"id": index + 1, "position": index, "isPass": False,
                 "glat": lat, "glon": lon, "pointName": "测试点" + str(index)}
                for index, (lat, lon) in enumerate(self.checkpoints())]

    def legs(self):
        """每段给一条带折点的候选道路，模拟高德返回的折线。"""
        checkpoints = self.checkpoints()
        legs = []
        for index, origin in enumerate(checkpoints):
            destination = checkpoints[(index + 1) % len(checkpoints)]
            middle = ((origin[0] + destination[0]) / 2, (origin[1] + destination[1]) / 2)
            legs.append([{"points": [origin, middle, destination],
                          "distance": int(distance_m(origin, destination)), "source": "v5"}])
        return legs

    def test_cache_hit_route_keeps_sampling_and_distance_consistent(self):
        """候选缓存命中时不再请求Key，生成结果仍无长空档且里程自洽。"""
        payload = self.payload()
        with patch.object(loop, "get_amap_key", return_value="fixture"), \
                patch.object(loop, "_fetch_route_legs", return_value=self.legs()) as fetch, \
                patch.object(loop.log, "disabled", True):
            ring = loop.get_campus_loop(payload, route_seed=1)
        self.assertEqual(fetch.call_count, 1)
        self.assertTrue((self.data / "campus_loop_bd.json").exists())
        length = _ring_length(ring)
        with patch.object(loop, "get_amap_key", side_effect=AssertionError("应命中缓存")), \
                patch.object(loop.log, "disabled", True), \
                patch.object(flow.log, "disabled", True):
            for fraction in (1.0, 2.35):
                distance = length * fraction + 50
                with self.subTest(fraction=fraction):
                    track, metadata = flow._build_amap_track(payload, distance, 400, 168, 1, 1000)
                    # 时长严格取「里程 / 配速」的整数秒，不向采样节奏凑整。
                    self.assertEqual(track["totalTime"], int(distance / 1000 * 400))
                    self.assertAlmostEqual(track["totalDistance"], distance, places=2)
                    self.assertLessEqual(max(RealismSamplingTests.gaps(track)),
                                         realism.MAX_SAMPLE_GAP + 0.5)
                    ratio = RealismSamplingTests.coordinate_length(track) / distance
                    self.assertLess(abs(ratio - 1.0), 0.03)
                    self.assertEqual(metadata["provider"], "amap_walking")
                    self.assertAlmostEqual(metadata["ring_length_m"], round(length, 2), places=2)
