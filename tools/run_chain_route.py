#!/usr/bin/env python3
"""从指定起点出发，按点位顺序跑，满目标里程就地停止。

规则（按产品口径）：
  1. 起点 -> P1 -> P2 -> ... -> Pn 依次走高德步行路线；
  2. 走完点位若还没到目标里程，就**往回生成到出发点**（Pn -> 起点 的返程腿）；
  3. 还不够，就**再按点位顺序跑一遍**（多圈，每圈带一点横向漂移，免得逐点重合）；
  4. 一到目标里程立即停止，不补完整圈、不额外闭合。

所以环只是「跑完点位后的返程 + 重复」的副产品，不是指标。

用法：
    python -B tools/run_chain_route.py --start 119.547725,39.925210 \
        --point 119.552100,39.925800 --point 119.550900,39.928400 --target 2000
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from funsport import config                                     # noqa: E402
from funsport.api import campus_loop as api_loop                # noqa: E402
from funsport.coordinates import (bd09_to_gcj02, distance_m,   # noqa: E402
                                  gcj02_to_bd09)
from funsport.track import generator as gen                     # noqa: E402
from funsport.track import humanize as hz                       # noqa: E402
import run_humanized_loop as rhl                                # noqa: E402

DEFAULT_START_MS = rhl.DEFAULT_START_MS
JOIN_TOL_M = 5.0


def parse_lnglat(text):
    """接受 lng,lat 或 lat,lng；统一返回 (lat, lng)。"""
    a, b = [float(x) for x in text.replace(" ", "").split(",")]
    if abs(a) > 90 and abs(b) <= 90:
        return (b, a)
    if abs(a) <= 90 and abs(b) > 90:
        return (a, b)
    raise SystemExit("坐标请写成 lng,lat：%s" % text)


def amap_key():
    cfg = config.load_config()
    key = cfg.get("amap_key") if isinstance(cfg, dict) else getattr(cfg, "amap_key", None)
    if not key:
        raise SystemExit("未配置高德 Web 服务 Key（.funsport/config.json 的 amap_key）")
    return key


def fetch_chain(start_gcj, points_gcj, key):
    """起点 -> 各点位 -> 起点；返回 (点列, 每段距离)。"""
    waypoints = [start_gcj] + list(points_gcj) + [start_gcj]
    chain, leg_dists = [], []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        pts, dist, ver = api_loop._amap_walking(a, b, key)
        leg_dists.append(dist)
        print("  leg %.6f,%.6f -> %.6f,%.6f  %4d m  %d 点 (%s)"
              % (a[0], a[1], b[0], b[1], dist, len(pts), ver))
        for pt in pts:
            if chain and distance_m(chain[-1], pt) < 1.0:
                continue
            chain.append(pt)
    if distance_m(chain[-1], chain[0]) < JOIN_TOL_M:
        chain[-1] = chain[0]
    else:
        print("  ⚠ 返程终点离起点 %.1f m（高德吸附所致），已按实际返程点收尾" % distance_m(chain[-1], chain[0]))
    return chain, leg_dists


def main(argv=None):
    ap = argparse.ArgumentParser(description="起点 -> 点位顺序 -> 返程 -> 重复，满目标里程就地停止")
    ap.add_argument("--start", default="119.547725,39.925210", help="GCJ-02 lng,lat")
    ap.add_argument("--point", action="append", default=[], help="点位 GCJ-02 lng,lat，可重复")
    ap.add_argument("--target", type=float, default=2000.0, help="目标里程（米），达到即停")
    ap.add_argument("--pace", type=float, default=360.0, help="秒/公里")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-ms", type=int, default=DEFAULT_START_MS)
    ap.add_argument("--chain-cache", default="", help="复用已抓好的链路 JSON")
    ap.add_argument("--save-chain", default="", help="保存链路 JSON")
    ap.add_argument("--out-dir", default=str(ROOT / "artifacts" / "humanized"))
    ap.add_argument("--name", default="chain_route")
    ap.add_argument("--title", default="")
    ap.add_argument("--no-charts", action="store_true")
    args = ap.parse_args(argv)

    start = parse_lnglat(args.start)
    points = [parse_lnglat(p) for p in args.point]

    if args.chain_cache:
        cached = json.loads(Path(args.chain_cache).read_text(encoding="utf-8"))
        road_bd = [tuple(p) for p in cached["road_bd"]]
        leg_dists = cached.get("leg_dists", [])
        print("链路来自缓存：%s" % args.chain_cache)
    else:
        print("起点 GCJ %.6f,%.6f，点位 %d 个" % (start[0], start[1], len(points)))
        chain_gcj, leg_dists = fetch_chain(start, points, amap_key())
        road_bd = [gcj02_to_bd09(lat, lng) for lat, lng in chain_gcj]
        if args.save_chain:
            Path(args.save_chain).parent.mkdir(parents=True, exist_ok=True)
            Path(args.save_chain).write_text(json.dumps(
                {"road_bd": road_bd, "leg_dists": leg_dists}, ensure_ascii=False), encoding="utf-8")

    lap_len = rhl.ring_length(road_bd)
    laps = max(1, int(math.ceil(args.target / max(lap_len, 1e-6))))
    print("链路（起点->点位->返程）%.1f m；目标 %.0f m 需要 %d 圈" % (lap_len, args.target, laps))

    route_bd, info = hz.humanize_laps(road_bd, seed=args.seed, laps=laps)
    route_bd = list(route_bd)
    if route_bd[0] != route_bd[-1]:
        route_bd.append(route_bd[0])
    human_len = rhl.ring_length(route_bd)
    print("拟人化后 %d 圈共 %.1f m（漂移 %s m）" % (info["laps"],
          human_len, [round(x, 2) for x in info.get("lap_offsets_m", [])]))

    dur = args.target / 1000.0 * args.pace
    track = gen.build(args.target, round(dur), args.seed, args.start_ms, route_bd,
                      ordered_path=True, cadence_target=0, sample_period_s=2,
                      distance_mode="arc")
    pts, total = rhl.track_to_points(track, route_bd)
    s = rhl.summarize(pts)
    first, last = pts[0], pts[-1]
    from funsport.coordinates import bd09_to_gcj02
    g0 = bd09_to_gcj02(first["lat"], first["lng"])
    g1 = bd09_to_gcj02(last["lat"], last["lng"])
    print("生成 %.1f m / %.0f s / %d 点" % (total, dur, len(pts)))
    print("起点 GCJ %.6f,%.6f（请求 %.6f,%.6f，差 %.0f m）"
          % (g0[0], g0[1], start[0], start[1], distance_m((g0[0], g0[1]), start)))
    print("终点 GCJ %.6f,%.6f（离开起点 %.0f m）"
          % (g1[0], g1[1], distance_m((g1[0], g1[1]), start)))
    print("v %.2f cv %.3f 步频 %.1f 步长 %.3f 最长低速连段 %s 弹性 %.3f"
          % (s["v_mean"], s["v_cv"], s["cadence_mean"], s["stride_mean"],
             s["max_low_speed_run"], s["cadence_elasticity"]))

    # 出图统一用 GCJ-02：Keep/高德/地图都是 GCJ，内部 BD 在这里显式转回来，
    # 否则图上整体偏 ~500 m，和 Keep 记录叠不上。
    for p in pts:
        g = bd09_to_gcj02(p["lat"], p["lng"])
        p["lat"], p["lng"] = round(g[0], 7), round(g[1], 7)
    road_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in road_bd]
    route_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in route_bd]
    doc = rhl.overlay_doc("%.2f km" % (args.target / 1000.0), pts, road_gcj, route_gcj,
                          laps, human_len, args.target, dur, args.seed, 0,
                          {"target_dist_m": args.target, "laps_needed": laps,
                           "chain_len_m": round(lap_len, 1),
                           "start_gcj": "%.6f,%.6f" % start})
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_path = out_dir / ("%s.json" % args.name)
    doc_path.write_text(json.dumps({"series": [doc]}, ensure_ascii=False), encoding="utf-8")
    print("-> %s" % doc_path)
    if not args.no_charts:
        html = out_dir / ("%s.html" % args.name)
        title = args.title or ("%.2f km · 起点 %.6f,%.6f" % (args.target / 1000.0, start[0], start[1]))
        subprocess.run([sys.executable, "-B", str(ROOT / "tools" / "run_overlay_charts.py"),
                        "--input", str(doc_path), "--out", str(html), "--title", title], check=True)
        print("-> %s" % html)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
