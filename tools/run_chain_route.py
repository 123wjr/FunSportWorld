#!/usr/bin/env python3
"""从指定起点出发，按点位顺序跑，满目标里程就地停止（单机脚本入口）。

规则（与 funsport/api/chain.py 同源，CLI 的 route 子命令走同一实现）：
  1. 起点 -> P1 -> P2 -> ... -> Pn 依次走高德步行路线；
  2. 走完点位若还没到目标里程，就**往回生成到出发点**（Pn -> 起点 的返程腿）；
  3. 还不够，就**再按点位顺序跑一遍**（多圈，每圈带一点横向漂移，免得逐点重合）；
  4. 一到目标里程立即停止，不补完整圈、不额外闭合。
  5. 反方向也成立：目标里程短于「走完所有点位」所需里程时，按后者生成。

所以环只是「跑完点位后的返程 + 重复」的副产品，不是指标。

用法：
    python -B tools/run_chain_route.py --start 119.547725,39.925210 \
        --point 119.552100,39.925800 --point 119.550900,39.928400 --target 2000
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from funsport.api import chain                                    # noqa: E402
from funsport.coordinates import bd09_to_gcj02, distance_m        # noqa: E402

DEFAULT_START_MS = 1762502400000


def main(argv=None):
    ap = argparse.ArgumentParser(description="起点 -> 点位顺序 -> 返程 -> 重复，满目标里程就地停止")
    ap.add_argument("--start", default="119.547725,39.925210",
                    help="GCJ-02 lng,lat；默认即用户指定起点")
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

    start = chain.parse_lnglat(args.start)
    points = [chain.parse_lnglat(p) for p in args.point]
    if not points:
        raise SystemExit("至少需要一个 --point：规则是「依次经过各个点位」，没有点位就没有链路")

    cached_gcj = None
    leg_dists = None
    if args.chain_cache:
        cached = json.loads(Path(args.chain_cache).read_text(encoding="utf-8"))
        road_bd = [tuple(p) for p in cached["road_bd"]]
        leg_dists = cached.get("leg_dists", [])
        cached_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in road_bd]
        print("链路来自缓存：%s" % args.chain_cache)
    else:
        print("起点 GCJ %.6f,%.6f，点位 %d 个" % (start[0], start[1], len(points)))
        cached_gcj, leg_dists = chain.fetch_chain(start, points, progress=print)
        if args.save_chain:
            path = Path(args.save_chain)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({
                "road_bd": [list(p) for p in
                            ((lat, lng) for lat, lng in cached_gcj)],
                "leg_dists": leg_dists}, ensure_ascii=False), encoding="utf-8")

    result = chain.build_chain_route(
        start, points, target_m=args.target, pace_s=args.pace, seed=args.seed,
        start_ms=args.start_ms, chain_gcj=cached_gcj, leg_dists=leg_dists,
        progress=print)

    meta = result["chain"]
    print("链路（起点->点位->返程）%.1f m，走完点位 %.1f m；目标 %.0f m 需要 %d 圈"
          % (meta["lap_len_m"], meta["visit_all_points_m"], meta["target_m"], meta["laps"]))
    if meta["raised_to_visit"]:
        print("⚠ 目标 %.0f m 短于走完所有点位所需的 %.0f m，按 %.0f m 生成"
              % (meta["requested_m"], meta["visit_all_points_m"], meta["target_m"]))
    hz_meta = meta.get("humanize") or {}
    print("拟人化后 %d 圈共 %.1f m（各圈长 %s m，接缝 %.2f m）"
          % (result["laps"], meta["humanized_len_m"],
             [round(x, 2) for x in hz_meta.get("lap_lengths_m", [])],
             hz_meta.get("seam_shift_max_m", 0.0)))

    pts = result["points"]
    s = result["stats"]
    first, last = pts[0], pts[-1]
    print("生成 %.1f m / %.0f s / %d 点" % (result["dist_m"], result["dur_s"], len(pts)))
    print("起点 GCJ %.6f,%.6f（请求 %.6f,%.6f，差 %.0f m）"
          % (first["lat"], first["lng"], start[0], start[1],
             distance_m((first["lat"], first["lng"]), start)))
    print("终点 GCJ %.6f,%.6f（离开起点 %.0f m）"
          % (last["lat"], last["lng"], distance_m((last["lat"], last["lng"]), start)))
    print("v %.2f cv %.3f 步频 %.1f 步长 %.3f 最长低速连段 %s 弹性 %.3f"
          % (s["v_mean"], s["v_cv"], s["cadence_mean"], s["stride_mean"],
             s["max_low_speed_run"], s["cadence_elasticity"] or 0.0))

    out_dir = Path(args.out_dir)
    doc_path = out_dir / ("%s.json" % args.name)
    html = None if args.no_charts else out_dir / ("%s.html" % args.name)
    title = args.title or ("%.2f km · 起点 %.6f,%.6f"
                           % (args.target / 1000.0, start[0], start[1]))
    chain.write_overlay(result, doc_path, html, title=title, progress=print)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
