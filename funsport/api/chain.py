"""链路里程规则：起点 -> 点位顺序 -> 返程 -> 重复，满目标里程就地停止。

产品口径（2026-11-07 定稿）：
  1. 从起点出发，按点位顺序依次走高德步行路线（起点 -> P1 -> ... -> Pn）；
  2. 走完点位若还没到目标里程，就往回生成到出发点（Pn -> 起点 的返程腿）；
  3. 还不够，就按点位顺序再跑一遍（多圈，每圈带一点横向漂移，免得逐点重合）；
  4. 一到目标里程立即停止，不补完整圈、不额外闭合；
  5. 反方向同样成立：目标短于「走完所有点位」所需里程时，按后者生成。

所以环只是「跑完点位后的返程 + 重复」的副产品，不是指标：轨迹的最后一秒
停在哪里就是哪里。本模块把这条规则做成 CLI / GUI 共用的实现，
tools/run_chain_route.py 仍然可以单独使用。

坐标口径：
  * 对外的 start / points 参数是 GCJ-02（高德、Keep、地图同源）；
  * generator 内部与落盘 gLat / gLng 是 BD-09；
  * 返回的 overlay 文档统一转回 GCJ-02，否则整张图会偏 ~500 m，和 Keep 叠不上。
"""
from __future__ import annotations

import bisect
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from ..config import get_amap_key, load_points_cache
from ..coordinates import bd09_to_gcj02, distance_m, gcj02_to_bd09
from ..track import generator as gen
from ..track import geom
from ..track import humanize as hz
from ..track import realism
from . import campus_loop as api_loop

DEFAULT_TARGET_M = 2000.0
DEFAULT_PACE_S_PER_KM = 360.0
DEFAULT_SEED = 7
JOIN_TOL_M = 5.0
U_TURN_RADIUS_M = 1.5
SCHEMA = "funsport.overlay/1"
ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------- 输入
def parse_lnglat(text):
    """接受 lng,lat 或 lat,lng，统一返回 (lat, lng)。"""
    parts = [p for p in str(text).replace(" ", "").split(",") if p != ""]
    if len(parts) != 2:
        raise ValueError("坐标请写成 lng,lat：%s" % text)
    a, b = float(parts[0]), float(parts[1])
    if abs(a) > 90 and abs(b) <= 90:
        return (b, a)
    if abs(a) <= 90 and abs(b) > 90:
        return (a, b)
    raise ValueError("坐标请写成 lng,lat：%s" % text)


def points_from_cache_gcj():
    """读取点位缓存并统一转成 GCJ-02 的 (lat, lng) 列表。

    缓存里点位同时带 lat/lon（BD-09）与 glat/glon（原生 GCJ-02），优先用后者，
    没有原生坐标时才从 BD-09 反算。
    """
    cache = load_points_cache(with_metadata=True)
    if not cache or not cache.get("points"):
        raise RuntimeError("没有点位缓存；要么先跑一次 funsport points，"
                           "要么用 --point 显式给点位")
    from ..coordinates import checkpoint_coordinates
    out = []
    for point in cache["points"]:
        _bd, gcj, _source, _delta = checkpoint_coordinates(point)
        out.append((float(gcj[0]), float(gcj[1])))
    return out


# ---------------------------------------------------------------- 几何
def ring_length(points_bd):
    """按 generator 自己用的口径量环长（ordered=True，顶点直用）。"""
    _dense, arcs, _c = geom.make_point_ring(points_bd, ordered=True)
    return arcs[-1] if arcs else 0.0


def fetch_chain(start_gcj, points_gcj, key=None, progress=None):
    """起点 -> 各点位 -> 起点，逐段走高德步行路线。

    返回 (chain_gcj, leg_dists)；leg_dists 含返程腿，所以「走完点位」的里程
    是 leg_dists[:-1] 之和。链路首尾若相差不足 JOIN_TOL_M 就吸附成同一点，
    否则按高德实际返程点收尾（高德会把起点吸附到最近可步行道路）。
    """
    say = progress or (lambda *_a, **_k: None)
    if not points_gcj:
        raise ValueError("至少需要一个点位：规则是「依次经过各个点位」")
    key = key or get_amap_key()
    waypoints = [start_gcj] + list(points_gcj) + [start_gcj]
    chain, leg_dists = [], []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        # 起点就是第一个点位时，第一段是 origin==destination。高德对这种退化腿
        # 的行为未经验证（可能直接拒绝），所以不请求，按 0 m 记入 leg_dists，
        # 保持「leg_dists 与腿一一对应、[:-1] 之和 = 走完点位里程」不变。
        if distance_m(a, b) < 1.0:
            leg_dists.append(0.0)
            say("leg %.6f,%.6f -> 同一点，跳过请求" % (a[0], a[1]))
            continue
        pts, dist, ver = api_loop._amap_walking(a, b, key)
        leg_dists.append(float(dist))
        say("leg %.6f,%.6f -> %.6f,%.6f  %4.0f m  %d 点 (%s)"
            % (a[0], a[1], b[0], b[1], dist, len(pts), ver))
        for pt in pts:
            if chain and distance_m(chain[-1], pt) < 1.0:
                continue
            chain.append((float(pt[0]), float(pt[1])))
    if distance_m(chain[-1], chain[0]) < JOIN_TOL_M:
        chain[-1] = chain[0]
    else:
        say("返程终点离起点 %.1f m（高德吸附所致），按实际返程点收尾"
            % distance_m(chain[-1], chain[0]))
    return chain, leg_dists


def plan_mileage(leg_dists, lap_len_m, target_m):
    """把「目标里程」换算成圈数，并把「走完点位」当下界。

    返回 dict：requested_m（原目标）、target_m（可能被抬高）、laps、
    visit_all_points_m、raised（是否因点位下界抬高）。
    """
    lap_len_m = float(lap_len_m)
    requested = float(target_m)
    visit = float(sum(leg_dists[:-1])) if len(leg_dists) > 1 else 0.0
    target = requested
    raised = False
    if visit and target < visit:
        target = visit
        raised = True
    if target <= 0:
        raise ValueError("目标里程必须大于 0")
    if lap_len_m <= 0:
        raise ValueError("链路长度为 0，无法生成轨迹")
    laps = max(1, int(math.ceil(target / lap_len_m)))
    return {"requested_m": requested, "target_m": float(target), "laps": laps,
            "visit_all_points_m": visit, "lap_len_m": lap_len_m, "raised": raised}


# ---------------------------------------------------------------- 逐点还原
def _plane(points_bd):
    """与 generator ordered 模式同一口径的局部平面投影。"""
    n = len(points_bd)
    lat0 = sum(p[0] for p in points_bd) / n
    lng0 = sum(p[1] for p in points_bd) / n
    m_lat = geom.meters_per_deg_lat(lat0)
    m_lng = geom.meters_per_deg_lng(lat0)
    return [((p[1] - lng0) * m_lng, (p[0] - lat0) * m_lat) for p in points_bd]


def _menger(a, b, c):
    """三点外接圆半径（米）；近似共线时返回 inf。"""
    ab = math.hypot(b[0] - a[0], b[1] - a[1])
    bc = math.hypot(c[0] - b[0], c[1] - b[1])
    ca = math.hypot(a[0] - c[0], a[1] - c[1])
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if cross < 1e-9:
        return float("inf")
    return ab * bc * ca / (2.0 * cross)


class _RouteIndex:
    """按里程查路线顶点，用于还原逐点局部曲率半径。"""

    def __init__(self, points_bd):
        self.P = _plane(points_bd)
        self.s = [0.0]
        for i in range(len(self.P) - 1):
            a, b = self.P[i], self.P[i + 1]
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.total = self.s[-1]
        self.unique = max(1, len(self.P) - 1)

    def radius(self, dist_m):
        if self.unique < 3:
            return float("inf")
        s = max(0.0, min(float(dist_m), self.total))
        i = bisect.bisect_right(self.s, s) - 1
        i = min(i, len(self.P) - 2)
        a = self.P[(i - 1) % self.unique]
        b = self.P[i % self.unique]
        c = self.P[(i + 1) % self.unique]
        return _menger(a, b, c)


def track_to_points(track, route_bd=None):
    """把 generator 的逐点输出折成 funsport.overlay/1 的 points 数组（GCJ-02）。"""
    index = _RouteIndex(route_bd) if route_bd else None
    loc = track["locations"]
    pts = []
    for i in range(1, len(loc)):
        a, b = loc[i - 1], loc[i]
        dt = b["totalTime"] - a["totalTime"]
        if dt <= 0:
            continue
        dd = b["totalDis"] - a["totalDis"]
        ds = b["steps"] - a["steps"]
        v = dd / dt
        cad = ds / dt * 60.0
        stride = (v / cad * 60.0) if cad > 0 else 0.0
        r = index.radius(b["totalDis"]) if index else float("inf")
        lat, lng = bd09_to_gcj02(b["gLat"], b["gLng"])
        pts.append({
            "t": round(float(b["totalTime"]), 3),
            "dist_m": round(float(b["totalDis"]), 3),
            "lat": round(lat, 7), "lng": round(lng, 7),
            "v": round(v, 5),
            "cadence": round(cad, 4),
            "stride_m": round(stride, 5),
            "emp_cadence": round(realism.cadence_for_speed(v), 3),
            "emp_stride_m": round(realism.stride_power_for_speed(v), 4),
            "corner_radius_m": (round(r, 2) if math.isfinite(r) else None),
            "u_turn": bool(math.isfinite(r) and r <= U_TURN_RADIUS_M),
        })
    return pts, float(track.get("totalDistance") or 0.0)


def summarize(pts):
    """逐点统计：速度均值/变异系数/最低速连段/步频弹性。"""
    if not pts:
        return {}
    vs = [p["v"] for p in pts]
    cads = [p["cadence"] for p in pts]
    strs = [p["stride_m"] for p in pts]
    mean = sum(vs) / len(vs)
    var = sum((v - mean) ** 2 for v in vs) / len(vs)
    gate = 0.6 * mean
    best = cur = 0
    for v in vs:
        cur = cur + 1 if v < gate else 0
        best = max(best, cur)
    lv = [math.log(v) for v in vs if v > 1e-6]
    lc = [math.log(c) for c in cads if c > 1e-6]
    elast = None
    if len(lv) == len(lc) and len(lv) > 2:
        mv, mc = sum(lv) / len(lv), sum(lc) / len(lc)
        den = sum((x - mv) ** 2 for x in lv)
        num = sum((x - mv) * (y - mc) for x, y in zip(lv, lc))
        elast = num / den if den else None
    return {
        "samples": len(pts), "v_mean": round(mean, 4),
        "v_cv": round(math.sqrt(var) / mean, 4) if mean else None,
        "v_min": round(min(vs), 3), "v_max": round(max(vs), 3),
        "cadence_mean": round(sum(cads) / len(cads), 2),
        "stride_mean": round(sum(strs) / len(strs), 4),
        "max_low_speed_run": best,
        "cadence_elasticity": (round(elast, 4) if elast is not None else None),
        "u_turn_points": sum(1 for p in pts if p["u_turn"]),
    }


# ---------------------------------------------------------------- 主入口
def build_chain_route(start_gcj, points_gcj, target_m=DEFAULT_TARGET_M,
                      pace_s=DEFAULT_PACE_S_PER_KM, cadence=0, seed=DEFAULT_SEED,
                      start_ms=0, key=None, chain_gcj=None, leg_dists=None,
                      progress=None):
    """生成一条符合链路规则的轨迹（不联网时可用 chain_gcj/leg_dists 复用缓存）。

    返回 dict：track / points / stats / route_bd / route_gcj / road_bd /
    road_gcj / chain（链路统计）/ dist_m / dur_s。
    """
    start_gcj = (float(start_gcj[0]), float(start_gcj[1]))
    points_gcj = [(float(p[0]), float(p[1])) for p in points_gcj]
    if pace_s < 160 or pace_s > 520:
        raise ValueError("配速需在 160~520 秒每公里之间（当前 %.0f）" % pace_s)
    if chain_gcj is None:
        chain_gcj, leg_dists = fetch_chain(start_gcj, points_gcj, key=key,
                                           progress=progress)
    else:
        chain_gcj = [(float(p[0]), float(p[1])) for p in chain_gcj]
        leg_dists = list(leg_dists or [])
    road_bd = [gcj02_to_bd09(lat, lng) for lat, lng in chain_gcj]
    lap_len = ring_length(road_bd)
    plan = plan_mileage(leg_dists, lap_len, target_m)
    dist = plan["target_m"]
    laps = plan["laps"]
    route_bd, hinfo = hz.humanize_laps(road_bd, seed=seed, laps=laps)
    route_bd = list(route_bd)
    human_len = ring_length(route_bd)
    # 拟人化后单圈会短几米；若目标里程刚好卡在整圈边界，必须再补一圈，
    # 否则生成器会回绕到第 0 圈、与第一圈逐点重合。
    while human_len < dist and laps < 64:
        laps += 1
        route_bd, hinfo = hz.humanize_laps(road_bd, seed=seed, laps=laps)
        route_bd = list(route_bd)
        human_len = ring_length(route_bd)
    if len(route_bd) > 1 and route_bd[0] != route_bd[-1]:
        route_bd.append(route_bd[0])
    dur = max(1, int(round(dist / 1000.0 * pace_s)))
    if not start_ms:
        start_ms = int(time.time() * 1000)
    track = gen.build(dist, dur, seed, start_ms, route_bd, ordered_path=True,
                      cadence_target=cadence, sample_period_s=2,
                      distance_mode="arc")
    pts, total = track_to_points(track, route_bd)
    stats = summarize(pts)
    road_gcj = [(float(p[0]), float(p[1])) for p in chain_gcj]
    route_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in route_bd]
    end_offset = 0.0
    if pts:
        end_offset = distance_m((pts[-1]["lat"], pts[-1]["lng"]), start_gcj)
    chain_meta = {
        "lap_len_m": round(lap_len, 2),
        "humanized_len_m": round(human_len, 2),
        "visit_all_points_m": round(plan["visit_all_points_m"], 2),
        "requested_m": round(plan["requested_m"], 2),
        "target_m": round(plan["target_m"], 2),
        "laps": laps,
        "raised_to_visit": plan["raised"],
        "humanize": hinfo,
        "start_gcj": "%.6f,%.6f" % start_gcj,
        "end_offset_m": round(end_offset, 2),
    }
    return {
        "track": track, "points": pts, "stats": stats,
        "road_bd": road_bd, "road_gcj": road_gcj,
        "route_bd": route_bd, "route_gcj": route_gcj,
        "chain": chain_meta,
        "dist_m": round(total, 3), "dur_s": round(float(track.get("totalTime") or 0.0), 3),
        "laps": laps, "seed": seed,
    }


def overlay_document(result, name=None, extra_meta=None):
    """把 build_chain_route 的结果包成 funsport.overlay/1 文档（GCJ-02）。"""
    chain_meta = result.get("chain") or {}
    meta = {
        "schema": SCHEMA,
        "laps": result.get("laps"),
        "ring_length_m": chain_meta.get("lap_len_m"),
        "humanized_len_m": chain_meta.get("humanized_len_m"),
        "dist_m": result.get("dist_m"),
        "dur_s": result.get("dur_s"),
        "seed": result.get("seed"),
        "cadence_target": 0,
        "samples": len(result.get("points") or ()),
        "target_dist_m": chain_meta.get("target_m"),
        "requested_dist_m": chain_meta.get("requested_m"),
        "visit_all_points_m": chain_meta.get("visit_all_points_m"),
        "raised_to_visit": 1 if chain_meta.get("raised_to_visit") else 0,
        "start_gcj": chain_meta.get("start_gcj", ""),
        "end_offset_m": chain_meta.get("end_offset_m"),
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    for k, v in (extra_meta or {}).items():
        if isinstance(v, (int, float, str)) and not isinstance(v, bool):
            meta[k] = v
    return {
        "name": name or ("%.2f km 链路" % (float(result.get("dist_m") or 0.0) / 1000.0)),
        "meta": meta,
        "route": {
            "road": [[p[0], p[1]] for p in (result.get("road_gcj") or ())],
            "track": [[p[0], p[1]] for p in (result.get("route_gcj") or ())],
        },
        "points": result.get("points") or [],
    }


def overlay_document_from_track(track, route_bd=None, name=None, extra_meta=None,
                                seed=None):
    """从已有的 generator 轨迹生成 overlay 文档（GUI 导出用，不需要重新规划路线）。"""
    pts, total = track_to_points(track, route_bd)
    lat0 = (pts[0]["lat"], pts[0]["lng"]) if pts else (0.0, 0.0)
    meta = {
        "schema": SCHEMA, "samples": len(pts),
        "dist_m": round(total, 3),
        "dur_s": round(float(track.get("totalTime") or 0.0), 3),
        "seed": seed,
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    for k, v in (extra_meta or {}).items():
        if isinstance(v, (int, float, str)) and not isinstance(v, bool):
            meta[k] = v
    doc = {"name": name or "轨迹预览", "meta": meta,
           "route": {"road": [], "track": []}, "points": pts}
    if lat0[0]:
        doc["meta"].setdefault("start_lat", lat0[0])
    return doc


def write_document(doc, json_path, html_path=None, title=None, progress=None):
    """写出任意 overlay 文档（GUI 预览导出用），可选生成可交互 HTML。"""
    say = progress or (lambda *_a, **_k: None)
    json_path = Path(json_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps({"series": [doc]}, ensure_ascii=False),
                         encoding="utf-8")
    say("-> %s" % json_path)
    if not html_path:
        return json_path, None
    html_path = Path(html_path)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    tool = ROOT / "tools" / "run_overlay_charts.py"
    if not tool.exists():
        raise RuntimeError("找不到图表工具：%s" % tool)
    subprocess.run([sys.executable, "-B", str(tool), "--input", str(json_path),
                    "--out", str(html_path), "--title", title or doc.get("name", "轨迹")],
                   check=True)
    say("-> %s" % html_path)
    return json_path, html_path


def write_overlay(result, json_path, html_path=None, title=None, progress=None):
    """写出 overlay JSON；给了 html_path 就再调用图表模板生成可交互 HTML。"""
    say = progress or (lambda *_a, **_k: None)
    json_path = Path(json_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    doc = overlay_document(result, extra_meta={"title": title or ""})
    json_path.write_text(json.dumps({"series": [doc]}, ensure_ascii=False),
                         encoding="utf-8")
    say("-> %s" % json_path)
    if not html_path:
        return json_path, None
    html_path = Path(html_path)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    tool = ROOT / "tools" / "run_overlay_charts.py"
    if not tool.exists():
        raise RuntimeError("找不到图表工具：%s" % tool)
    subprocess.run([sys.executable, "-B", str(tool), "--input", str(json_path),
                    "--out", str(html_path),
                    "--title", title or doc["name"]], check=True)
    say("-> %s" % html_path)
    return json_path, html_path
