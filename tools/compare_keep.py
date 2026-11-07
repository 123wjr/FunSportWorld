#!/usr/bin/env python3
"""把一条 Keep 记录和我们的生成轨迹画在同一张可叠加图上。

两份数据都统一到 **GCJ-02**（Keep 的 geoPoints 本来就是 GCJ-02；生成器内部的
BD-09 在这里显式转回来），所以两张图叠在一起位置才对得上。

「同一组点位」：从 Keep 自己的轨迹上按里程取几个点作为点位，我们的引擎按
起点 -> 点位顺序 -> 返程 -> 满目标里程即停 的规则走同一组点，因此两条线覆盖
同一段路，可以直接比形状与速度/步频/步长。

用法：
    python -B tools/compare_keep.py --keep data/keep/9223370261200580990.json
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
from funsport.coordinates import (bd09_to_gcj02, distance_m,    # noqa: E402
                                  gcj02_to_bd09)
from funsport.track import generator as gen                     # noqa: E402
from funsport.track import humanize as hz                       # noqa: E402
import run_humanized_loop as rhl                                # noqa: E402


def load_keep(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pts = data["decodedSeries"]["geoPoints"]
    detail = data.get("keepDetail", {})
    return pts, detail


def keep_series(pts):
    """Keep 逐点序列 -> overlay/1 的 points（t 秒、dist 米、v m/s、步频、步长）。"""
    out = []
    t0 = pts[0]["unixTimestamp"]
    for a, b in zip(pts[:-1], pts[1:]):
        dt = (b["unixTimestamp"] - a["unixTimestamp"]) / 1000.0
        if dt <= 0:
            continue
        dd = float(b.get("currentTotalDistance", 0.0)) - float(a.get("currentTotalDistance", 0.0))
        ds = float(b.get("currentTotalSteps", 0)) - float(a.get("currentTotalSteps", 0))
        v = dd / dt
        cad = ds / dt * 60.0
        out.append({
            "t": round((b["unixTimestamp"] - t0) / 1000.0, 3),
            "dist_m": round(float(b.get("currentTotalDistance", 0.0)), 3),
            "lat": b["latitude"], "lng": b["longitude"],
            "v": round(v, 5),
            "cadence": round(cad, 4),
            "stride_m": round(v / cad * 60.0, 5) if cad > 0 else 0.0,
        })
    return out


def resample_by_distance(pts, fracs):
    """按累计里程取点位（返回 GCJ (lat, lng)），沿用 Keep 自己的走法。"""
    acc = [0.0]
    for a, b in zip(pts[:-1], pts[1:]):
        acc.append(acc[-1] + distance_m((a["latitude"], a["longitude"]),
                                        (b["latitude"], b["longitude"])))
    total = acc[-1]
    out = []
    for f in fracs:
        target = total * f
        i = min(range(len(acc)), key=lambda k: abs(acc[k] - target))
        out.append((pts[i]["latitude"], pts[i]["longitude"]))
    return out, total


def amap_key():
    cfg = config.load_config()
    key = cfg.get("amap_key") if isinstance(cfg, dict) else getattr(cfg, "amap_key", None)
    if not key:
        raise SystemExit("未配置高德 Web 服务 Key")
    return key


def fetch_chain(start, points, key):
    waypoints = [start] + list(points) + [start]
    chain = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        pts, dist, ver = api_loop._amap_walking(a, b, key)
        print("  leg %8.1f m  %d 点 (%s)" % (dist, len(pts), ver))
        for pt in pts:
            if chain and distance_m(chain[-1], pt) < 1.0:
                continue
            chain.append(pt)
    if distance_m(chain[-1], chain[0]) < 5.0:
        chain[-1] = chain[0]
    return chain


def main(argv=None):
    ap = argparse.ArgumentParser(description="Keep 记录 vs 生成轨迹（同为 GCJ-02，同一组点位）")
    ap.add_argument("--keep", default="data/keep/9223370261200580990.json")
    ap.add_argument("--fracs", default="0.33,0.66,1.0", help="按 Keep 里程取点位的比例")
    ap.add_argument("--target", type=float, default=0.0, help="目标里程，默认=Keep 里程")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--chain-cache", default="")
    ap.add_argument("--save-chain", default=".test-tmp/lead/keep_chain.json")
    ap.add_argument("--out-dir", default=str(ROOT / "artifacts" / "humanized"))
    ap.add_argument("--name", default="keep_vs_generated")
    ap.add_argument("--title", default="")
    ap.add_argument("--no-charts", action="store_true")
    args = ap.parse_args(argv)

    pts, detail = load_keep(args.keep)
    kseries = keep_series(pts)
    keep_dist = float(pts[-1].get("currentTotalDistance", 0.0))
    keep_dur = float(detail.get("duration", 0)) or (pts[-1]["unixTimestamp"] - pts[0]["unixTimestamp"]) / 1000.0
    print("Keep：%.0f m / %.0f s / %d 点，起点 %.6f,%.6f"
          % (keep_dist, keep_dur, len(pts), pts[0]["latitude"], pts[0]["longitude"]))

    start = (pts[0]["latitude"], pts[0]["longitude"])
    fracs = [float(x) for x in args.fracs.split(",") if x.strip()]
    waypoints, track_len = resample_by_distance(pts, fracs)
    print("点位（取自 Keep 轨迹，按里程 %s）：%s" % (fracs, ["%.6f,%.6f" % p for p in waypoints]))

    if args.chain_cache and Path(args.chain_cache).exists():
        cached = json.loads(Path(args.chain_cache).read_text(encoding="utf-8"))
        chain_gcj = [tuple(p) for p in cached["chain_gcj"]]
        print("链路来自缓存")
    else:
        print("抓取高德链路…")
        chain_gcj = fetch_chain(start, waypoints, amap_key())
        if args.save_chain:
            Path(args.save_chain).parent.mkdir(parents=True, exist_ok=True)
            Path(args.save_chain).write_text(json.dumps(
                {"chain_gcj": chain_gcj, "waypoints_gcj": waypoints}, ensure_ascii=False), encoding="utf-8")

    target = args.target or keep_dist
    chain_bd = [gcj02_to_bd09(lat, lng) for lat, lng in chain_gcj]
    lap_len = rhl.ring_length(chain_bd)
    laps = max(1, int(math.ceil(target / max(lap_len, 1e-6))))
    print("链路 %.1f m（起点->点位->返程）；目标 %.0f m，%d 圈" % (lap_len, target, laps))

    route_bd, info = hz.humanize_laps(chain_bd, seed=args.seed, laps=laps)
    route_bd = list(route_bd)
    if route_bd[0] != route_bd[-1]:
        route_bd.append(route_bd[0])
    route_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in route_bd]
    human_len = rhl.ring_length(route_bd)

    pace = keep_dur / max(keep_dist / 1000.0, 1e-6)
    dur = target / 1000.0 * pace
    track = gen.build(target, round(dur), args.seed, rhl.DEFAULT_START_MS, route_bd,
                      ordered_path=True, cadence_target=0, sample_period_s=2,
                      distance_mode="arc")
    gpts, total = rhl.track_to_points(track, route_bd)
    for p in gpts:
        g = bd09_to_gcj02(p["lat"], p["lng"])
        p["lat"], p["lng"] = round(g[0], 7), round(g[1], 7)
    s = rhl.summarize(gpts)
    print("生成 %.0f m / %.0f s（配速 %.0f s/km 取自 Keep）/ %d 点" % (total, dur, pace, len(gpts)))
    print("v %.2f cv %.3f 步频 %.1f 步长 %.3f 最长低速连段 %s 弹性 %.3f"
          % (s["v_mean"], s["v_cv"], s["cadence_mean"], s["stride_mean"],
             s["max_low_speed_run"], s["cadence_elasticity"]))

    keep_doc = rhl.overlay_doc("Keep 实测", kseries, [(p["latitude"], p["longitude"]) for p in pts],
                               [(p["latitude"], p["longitude"]) for p in pts], 1, keep_dist,
                               keep_dist, keep_dur, 0, 0, {"source": "keep", "pace_s_per_km": round(pace, 1)})
    gen_doc = rhl.overlay_doc("生成轨迹", gpts, chain_gcj, route_gcj, laps, human_len,
                              target, dur, args.seed, 0,
                              {"source": "generated", "pace_s_per_km": round(pace, 1),
                               "human_laps": info.get("laps")})
    doc = {"series": [keep_doc, gen_doc]}

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_path = out_dir / ("%s.json" % args.name)
    doc_path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
    print("-> %s" % doc_path)
    if not args.no_charts:
        html = out_dir / ("%s.html" % args.name)
        title = args.title or ("Keep %.2f km vs 生成 %.2f km · 同一组点位" % (keep_dist / 1000, target / 1000))
        subprocess.run([sys.executable, "-B", str(ROOT / "tools" / "run_overlay_charts.py"),
                        "--input", str(doc_path), "--out", str(html), "--title", title], check=True)
        print("-> %s" % html)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
