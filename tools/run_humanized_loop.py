#!/usr/bin/env python
# -*- coding: utf-8 -*-
r"""拟人化路线的一条命令入口：高德候选缓存 -> humanize -> generator -> 可叠加交互图表。

一句话：
    缓存里的高德环线  ──humanize──▶  拟人化环线  ──generator.build──▶  逐点轨迹
                                                                        │
                                       tools/run_overlay_charts.py  ◀────┘
                                                     │
                                       单个离线 HTML（路线图 + 速度/步频/步长）

默认全程离线（只读 .funsport/campus_loop_bd.json，不请求高德、不联网）。

用法：
  .venv\Scripts\python.exe -B tools\run_humanized_loop.py
  .venv\Scripts\python.exe -B tools\run_humanized_loop.py --laps 1,2,3 --pace 360 --cadence 168
  .venv\Scripts\python.exe -B tools\run_humanized_loop.py --laps 2 --no-charts --out-json out.json

产物：
  artifacts/humanized/track_laps<N>.json   逐圈轨迹（funsport.overlay/1）
  artifacts/humanized/overlay_all.json     多圈合并（{"series":[...]}）
  artifacts/humanized/route_overlay.html   最终可叠加交互图（自包含、无 CDN、非图片）

重要口径说明（避免误读）：
  * lat/lng 一律是 **BD09**（与 generator 输出 gLat/gLng 同源），不是 GCJ/WGS84。
  * 逐点速度 v = ΔtotalDis / ΔtotalTime（米/秒）；逐点步频 = Δsteps / Δt × 60。
    generator 落盘的 "speed"/"avgSpeed" 字段恒为 0.0，不得直接采信。
  * 步频在 2 s 采样 + 整数步数下被量化成 30 spm 一档（只出现 120/150/180/210），
    因此逐点步频图必然是阶梯；经验式线只用于对照均值与此例下的期望值。
  * 选路复刻 funsport/api/campus_loop.py:_select_loop 的打分与近优轮换，
    但省掉打卡点覆盖校验（缓存里没有打卡点坐标），因此选出的组合可能与
    生产选路差一个种子档位。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from funsport.api import campus_loop as cl                      # noqa: E402
from funsport.coordinates import bd09_to_gcj02                  # noqa: E402
from funsport.track import generator as gen                     # noqa: E402
from funsport.track import geom                                 # noqa: E402
from funsport.track import humanize as hz                       # noqa: E402
from funsport.track import realism                              # noqa: E402
from funsport.track.route_metrics import route_overlap          # noqa: E402

CACHE = os.path.join(ROOT, ".funsport", "campus_loop_bd.json")
OUT_DIR = os.path.join(ROOT, "artifacts", "humanized")
CHARTS = os.path.join(HERE, "run_overlay_charts.py")

# 固定起点时刻，保证同 seed 同参数的结果逐字节可复现。
DEFAULT_START_MS = 1762502400000          # 2025-11-07 08:00:00 +0800
SCHEMA = "funsport.overlay/1"
U_TURN_RADIUS_M = 1.5                     # 局部曲率半径 <= 此值标为掉头点


# ------------------------------------------------------------------ 选路
def select_ring(legs, route_seed):
    """复刻 _select_loop：全组合打分 -> 近优集 -> near[seed % len(near)]。"""
    counts = [len(options) for options in legs]
    total = 1
    for c in counts:
        total *= c
    if not total:
        raise SystemExit("缓存里没有可用候选组合")
    if total <= cl.MAX_COMBINATION_TRIES:
        variants = list(range(total))
    else:
        import random
        rng = random.Random(route_seed)
        variants, seen = [], set()
        for upper in range(total - cl.MAX_COMBINATION_TRIES, total):
            draw = rng.randrange(upper + 1)
            variants.append(upper if draw in seen else draw)
            seen.add(variants[-1])
    valid = []
    for variant in variants:
        remainder, choices = variant, []
        for count in counts:
            choices.append(remainder % count)
            remainder //= count
        ring = cl._assemble_loop(legs, choices, quiet=True)
        if ring is None:
            continue
        metrics = route_overlap(ring)
        score = metrics["length_m"] + cl.REPEAT_PENALTY * metrics["repeated_m"]
        valid.append((score, variant, choices, metrics))
    if not valid:
        raise SystemExit("已检查 %d/%d 个组合，未找到可用环线" % (len(variants), total))
    best = min(item[0] for item in valid)
    near = sorted((it for it in valid if it[0] <= best * (1 + cl.NEAR_OPTIMAL_FRACTION) + 1e-8),
                  key=lambda it: it[1])
    score, variant, choices, metrics = near[route_seed % len(near)]
    ring = cl._assemble_loop(legs, choices, quiet=False)
    if ring is None:
        raise SystemExit("入选道路重建校验失败")
    info = {"segment_candidates": counts, "candidate_combinations": total,
            "selected_variant": variant + 1, "near_optimal_combinations": len(near),
            "valid_combinations": len(valid), "road_length_m": round(metrics["length_m"], 3),
            "repeated_distance_m": round(metrics["repeated_m"], 3),
            "geometric_repeat_ratio": round(metrics["repeat_ratio"], 6),
            "selection_score": round(score, 3), "best_selection_score": round(best, 3),
            "repeat_penalty": cl.REPEAT_PENALTY,
            "near_optimal_fraction": cl.NEAR_OPTIMAL_FRACTION}
    return ring, info


def load_cache():
    if not os.path.exists(CACHE):
        raise SystemExit("找不到候选缓存 %s（本脚本不联网，请先让生产流程生成缓存）" % CACHE)
    with open(CACHE, encoding="utf-8") as fh:
        doc = json.load(fh)
    legs = doc.get("legs") if isinstance(doc, dict) else None
    if not legs:
        raise SystemExit("缓存格式不符：缺少 legs")
    meta = {"schema": doc.get("schema"), "provider": doc.get("provider"),
            "fingerprint": doc.get("fingerprint")}
    return legs, meta


# ------------------------------------------------------------------ 几何
def plane_of(points_bd):
    """与 humanize/generator 同口径的局部平面投影。"""
    return hz._to_plane(points_bd)


def track_arcs(points_bd):
    P, _center, _scale = plane_of(points_bd)
    n = len(P)
    arcs = [0.0]
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        arcs.append(arcs[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return P, arcs


def radius_at(P, i):
    """第 i 个顶点处的局部曲率半径（环上取相邻两点，Menger 式近似）。"""
    n = len(P)
    a, b, c = P[(i - 1) % n], P[i], P[(i + 1) % n]
    u1 = (b[0] - a[0], b[1] - a[1])
    u2 = (c[0] - b[0], c[1] - b[1])
    l1 = math.hypot(*u1)
    l2 = math.hypot(*u2)
    if l1 <= 1e-9 or l2 <= 1e-9:
        return float("inf")
    cosang = max(-1.0, min(1.0, (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)))
    dtheta = math.acos(cosang)
    ds = 0.5 * (l1 + l2)
    kappa = dtheta / ds if ds > 1e-9 else 0.0
    return (1.0 / kappa) if kappa > 1e-9 else float("inf")


def radius_lookup(P, arcs):
    radii = [radius_at(P, i) for i in range(len(P))]
    total = arcs[-1]

    def at(s):
        s = s % total if total > 0 else 0.0
        lo, hi = 0, len(P) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if arcs[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return radii[lo % len(P)]

    return at, total


# ------------------------------------------------------------------ 轨迹
def ring_length(points_bd):
    """按 generator 自己用的口径量环长（ordered=True，顶点直用）。"""
    dense, arcs, _c = geom.make_point_ring(points_bd, ordered=True)
    return arcs[-1] if arcs else 0.0


def build_track(route_bd, pace_s_per_km, seed, cadence_target, start_ms,
                sample_period_s=2.0):
    """沿一条多圈点列跑到它的末尾（不重复同一圈）。"""
    ring_len = ring_length(route_bd)
    dist = ring_len
    dur = dist / 1000.0 * pace_s_per_km
    # 与生产同口径：flow.py / chain.py 都把场地高程场喂给 generator，本工具必须一致，
    # 否则验收产物（artifacts/humanized/*.json）与线上跑的就不是同一条轨迹。
    elevations = realism.elevations_for_path(route_bd)
    tr = gen.build(dist, round(dur), seed, start_ms, route_bd, ordered_path=True,
                   cadence_target=cadence_target, sample_period_s=sample_period_s,
                   distance_mode="arc", elevations_m=elevations)
    return tr, ring_len, dist, dur


def track_to_points(tr, points_bd):
    """把 generator 的逐点输出折成 funsport.overlay/1 的 points 数组。"""
    P, arcs = track_arcs(points_bd)
    rad_at, total = radius_lookup(P, arcs)
    loc = tr["locations"]
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
        r = rad_at(b["totalDis"])
        pts.append({
            "t": round(float(b["totalTime"]), 3),
            "dist_m": round(float(b["totalDis"]), 3),
            "lat": b["gLat"], "lng": b["gLng"],
            "v": round(v, 5),
            "cadence": round(cad, 4),
            "stride_m": round(stride, 5),
            "emp_cadence": round(realism.cadence_for_speed(v), 3),
            "emp_stride_m": round(realism.stride_power_for_speed(v), 4),
            "elevation_m": (round(float(b["bdA"]), 2) if isinstance(b.get("bdA"), (int, float)) else None),
            "corner_radius_m": (round(r, 2) if math.isfinite(r) else None),
            "u_turn": bool(math.isfinite(r) and r <= U_TURN_RADIUS_M),
        })
    return pts, total


def overlay_doc(name, pts, road_bd, track_bd, laps, ring_len, dist, dur, seed,
                cadence_target, extra_meta=None):
    meta = {"schema": SCHEMA, "laps": laps, "ring_length_m": round(ring_len, 3),
            "dist_m": round(dist, 3), "dur_s": round(dur, 3), "seed": seed,
            "cadence_target": cadence_target, "samples": len(pts),
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    if extra_meta:
        for k, v in extra_meta.items():
            if isinstance(v, (int, float, str)) and not isinstance(v, bool):
                meta[k] = v
            elif (isinstance(v, (list, tuple))
                  and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)):
                # 纯数字列表（如 lap_lengths_m）必须落盘：曾经的标量白名单把它**静默丢掉**，
                # 下游只能靠猜圈边界，直接导致一次圈间距测量错到 0.02 m。
                meta[k] = list(v)
    return {
        "name": name,
        "meta": meta,
        "route": {"road": [[p[0], p[1]] for p in road_bd],
                  "track": [[p[0], p[1]] for p in track_bd]},
        "points": pts,
    }


# ------------------------------------------------------------------ 统计
def summarize(pts):
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


# ------------------------------------------------------------------ 主流程
def run(laps, seed, pace, cadence_target, out_dir, want_charts, title, start_ms,
        sample_period_s):
    legs, cache_meta = load_cache()
    ring, selection = select_ring(legs, seed)
    road_len = route_overlap(ring)["length_m"]

    pts_bd, hinfo = hz.humanize_route(ring, seed=seed)
    hstats = hz.analyze_route(pts_bd, reference_bd=ring)
    rstats = hz.analyze_route(ring)

    os.makedirs(out_dir, exist_ok=True)
    docs, lines = [], []
    lines.append("高德缓存指纹 %s（%s）" % (cache_meta.get("fingerprint"), cache_meta.get("provider")))
    lines.append("选路：组合 %d/%d，近优 %d 个，几何长 %.1f m，重复 %.1f m（%.1f%%）"
                 % (selection["selected_variant"], selection["candidate_combinations"],
                    selection["near_optimal_combinations"], selection["road_length_m"],
                    selection["repeated_distance_m"],
                    selection["geometric_repeat_ratio"] * 100))
    lines.append("拟人化：转角 p50 %.2f / p90 %.2f / max %.2f deg（原路 p50 %.2f / max %.2f），"
                 ">45deg %d 个，minR %.3f m，偏离 p50 %.2f / p95 %.2f / max %.2f m"
                 % (hstats["turn_p50_deg"], hstats["turn_p90_deg"], hstats["turn_max_deg"],
                    rstats["turn_p50_deg"], rstats["turn_max_deg"],
                    hstats["n_turn_gt_45"], hstats["min_radius_m"],
                    hstats["deviation_m"]["p50"], hstats["deviation_m"]["p95"],
                    hstats["deviation_m"]["max"]))
    lines.append("高德原始环长 %.1f m，拟人化后 %.1f m"
                 % (road_len, hstats["length_m"]))

    for lap in laps:
        # 多圈不是把同一圈交给生成器（那会逐点重合）：humanize_laps 每圈用
        # 独立子种子重新拟人化，再平滑拼接，让生成器跑到这条路径的末尾。
        route_bd, linfo = hz.humanize_laps(ring, seed=seed, laps=lap)
        route_bd = list(route_bd)
        if route_bd and route_bd[0] != route_bd[-1]:
            route_bd.append(route_bd[0])
        lapstats = hz.analyze_route(route_bd, reference_bd=ring)
        tr, ring_len, dist, dur = build_track(route_bd, pace, seed, cadence_target,
                                              start_ms, sample_period_s)
        lines.append("laps=%d：各圈长 %s m（接缝 %.2f m），路径长 %.1f m，"
                     "转角 p90 %.2f / max %.2f deg，minR %.3f m"
                     % (lap, [round(v, 2) for v in linfo["lap_lengths_m"]],
                        linfo["seam_shift_max_m"], lapstats["length_m"],
                        lapstats["turn_p90_deg"], lapstats["turn_max_deg"],
                        lapstats["min_radius_m"]))
        pts, tlen = track_to_points(tr, route_bd)
        # 出图统一 GCJ-02：generator 落盘的 gLat/gLng 是 BD-09，Keep / 高德 / 地图
        # 都是 GCJ-02，不转会整体偏 ~870 m（本起点实测）。
        for p in pts:
            g = bd09_to_gcj02(p["lat"], p["lng"])
            p["lat"], p["lng"] = round(g[0], 7), round(g[1], 7)
        road_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in ring]
        route_gcj = [bd09_to_gcj02(lat, lng) for lat, lng in route_bd]
        doc = overlay_doc("laps=%d" % lap, pts, road_gcj, route_gcj, lap, ring_len, dist, dur,
                          seed, cadence_target,
                          {"selection_variant": selection["selected_variant"],
                           "humanize_min_radius_m": round(lapstats["min_radius_m"], 3),
                           "lap_lengths_m": [round(v, 3) for v in linfo["lap_lengths_m"]],
                           "seam_shift_max_m": round(linfo["seam_shift_max_m"], 3),
                           "road_length_m": round(road_len, 3),
                           "pace_s_per_km": pace,
                           "elevationSource": tr.get("elevationSource", "unknown")})
        docs.append(doc)
        path = os.path.join(out_dir, "track_laps%d.json" % lap)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, ensure_ascii=False)
            fh.write("\n")
        s = summarize(pts)
        lines.append("laps=%d  %.1f m / %.0f s（%.1f s/km）  采样 %d 点  环长 %.1f m  "
                     "v mean %.2f cv %.3f [%.2f..%.2f]  步频 %.1f  步长 %.3f m  "
                     "最长低速连段 %d  弹性 %s  掉头点 %d"
                     % (lap, dist, dur, pace, s["samples"], ring_len, s["v_mean"], s["v_cv"],
                        s["v_min"], s["v_max"], s["cadence_mean"], s["stride_mean"],
                        s["max_low_speed_run"], s["cadence_elasticity"],
                        s["u_turn_points"]))
        lines.append("    -> %s" % path)

    html_path = None
    if want_charts:
        if len(docs) > 1:
            merged = os.path.join(out_dir, "overlay_all.json")
            with open(merged, "w", encoding="utf-8") as fh:
                json.dump({"series": docs}, fh, ensure_ascii=False)
                fh.write("\n")
            inputs = [merged]
        else:
            inputs = [os.path.join(out_dir, "track_laps%d.json" % docs[0]["meta"]["laps"])]
        html_path = os.path.join(out_dir, "route_overlay.html")
        cmd = [sys.executable, "-B", CHARTS, "--input", *inputs, "--out", html_path]
        if title:
            cmd += ["--title", title]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            lines.append("图表生成失败（exit %d）：%s" % (proc.returncode, proc.stderr.strip()[:400]))
            html_path = None
        else:
            for ln in (proc.stdout or "").strip().splitlines():
                lines.append("   charts: %s" % ln)
            lines.append("    -> %s" % html_path)

    print("\n".join(lines))
    return {"selection": selection, "humanize": hstats, "docs": docs, "html": html_path}


def main(argv=None):
    ap = argparse.ArgumentParser(description="humanized route closed loop + overlay charts")
    ap.add_argument("--laps", default="1", help="圈数，可用逗号分隔：1 / 1,2,3")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--pace", type=float, default=360.0, help="配速 秒/公里（默认 360 = 6:00/km）")
    ap.add_argument("--cadence", type=float, default=0.0, help="目标步频 spm，0 表示不强拟合")
    ap.add_argument("--start-ms", type=int, default=DEFAULT_START_MS)
    ap.add_argument("--sample-period-s", type=float, default=2.0)
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--title", default=None)
    ap.add_argument("--no-charts", action="store_true")
    ap.add_argument("--json-only", action="store_true", help="等同 --no-charts，并抑制 stdout 摘要")
    args = ap.parse_args(argv)

    laps = [int(x) for x in str(args.laps).replace(" ", "").split(",") if x]
    if any(l < 1 for l in laps):
        ap.error("--laps 必须是正整数")
    res = run(laps, args.seed, args.pace, args.cadence, args.out_dir,
              not (args.no_charts or args.json_only), args.title, args.start_ms,
              args.sample_period_s)
    if args.json_only:
        print(json.dumps({"html": res["html"], "laps": laps}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
