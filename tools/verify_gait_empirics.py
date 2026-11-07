"""经验步态核验：步频/步长 vs 经验方程、速度波动与低速持续性、疲劳单调性。

用法：
  python -B tools/verify_gait_empirics.py [--dist 1000,3000,8000] [--dur 334,1000,2700]
                                          [--seed 1,7,42] [--cadence 0,168]
                                          [--out .test-tmp/motion/gait_empirics.json]
自包含：只用标准库（不引 numpy/pandas/matplotlib/generator 之外的东西）。
before/after：用 git show ec49382 把旧 realism/generator 读进独立模块做纯函数对比。
输出 JSON + 人读摘要；数值全部由本次运行实测。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import funsport.track.generator as generator          # noqa: E402
from funsport.track import realism                     # noqa: E402

generator.log.disabled = True

CAD_REF_SPM = realism.CADENCE_POWER_REF_SPM
CAD_REF_V = realism.CADENCE_POWER_REF_SPEED
CAD_EXP = realism.CADENCE_POWER_EXP
STR_REF_M = realism.STRIDE_POWER_REF_M
STR_EXP = realism.STRIDE_POWER_EXP
LOW_RATIO = realism.LOW_SPEED_RATIO
LEGACY_REF = "ec49382"
TMP_DIR = os.path.join(ROOT, ".test-tmp", "motion")


def empirical_cadence(v):
    return CAD_REF_SPM * (v / CAD_REF_V) ** CAD_EXP


def empirical_stride(v):
    return STR_REF_M * (v / CAD_REF_V) ** STR_EXP


# ── 路线：campus_loop_bd.json 每 leg 取 distance 最小的候选，拼一条有序折线 ──
def campus_points():
    with open(os.path.join(ROOT, ".funsport", "campus_loop_bd.json"), encoding="utf-8") as fh:
        data = json.load(fh)
    pts = []
    for leg in data["legs"]:
        seg = sorted(leg, key=lambda c: c.get("distance", 0))[0]["points"]
        if pts and pts[-1:] == seg[0:1]:
            seg = seg[1:]
        pts.extend(seg)
    return pts


def legacy_modules():
    """旧实现（HEAD=ec49382）读进独立模块；git 不可用则返回 (None, None)。"""
    try:
        realism_src = subprocess.check_output(
            ["git", "show", LEGACY_REF + ":funsport/track/realism.py"],
            cwd=ROOT, stderr=subprocess.DEVNULL).decode("utf-8")
        gen_src = subprocess.check_output(
            ["git", "show", LEGACY_REF + ":funsport/track/generator.py"],
            cwd=ROOT, stderr=subprocess.DEVNULL).decode("utf-8")
    except Exception as exc:                                    # pragma: no cover
        print("legacy 不可用：%s" % exc)
        return None, None
    old_realism = types.ModuleType("funsport.track.realism")
    old_realism.__package__ = "funsport.track"
    exec(compile(realism_src, "legacy:realism.py", "exec"), old_realism.__dict__)
    saved = sys.modules["funsport.track.realism"]
    sys.modules["funsport.track.realism"] = old_realism
    try:
        old_gen = types.ModuleType("funsport.track._legacy_gen")
        old_gen.__package__ = "funsport.track"
        exec(compile(gen_src, "legacy:generator.py", "exec"), old_gen.__dict__)
    finally:
        sys.modules["funsport.track.realism"] = saved
    old_gen.__dict__["realism"] = old_realism
    old_gen.log.disabled = True
    return old_realism, old_gen


# ── 基本统计 ─────────────────────────────────────────────────
def corr(a, b):
    n = len(a)
    if n < 2:
        return float("nan")
    ma, mb = sum(a) / n, sum(b) / n
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((x - mb) ** 2 for x in b)
    if va <= 0 or vb <= 0:
        return float("nan")
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(va * vb)


def elasticity(speeds, cadences):
    """d ln(cad) / d ln(v)：最小二乘斜率（唯一的经验弹性定义）。"""
    pairs = [(math.log(v), math.log(c)) for v, c in zip(speeds, cadences) if v > 0 and c > 0]
    if len(pairs) < 2:
        return float("nan")
    lv = [p[0] for p in pairs]
    lc = [p[1] for p in pairs]
    mv, mc = sum(lv) / len(lv), sum(lc) / len(lc)
    den = sum((x - mv) ** 2 for x in lv)
    if den <= 0:
        return float("nan")
    return sum((x - mv) * (y - mc) for x, y in zip(lv, lc)) / den


def percentile(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    pos = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[pos]


def landed_series(locations):
    """落盘序列：逐采样点速度/步频/时刻/里程（全部由 output 坐标差分得到）。"""
    v, c, t, d = [], [], [], []
    for i in range(1, len(locations)):
        dt = locations[i]["totalTime"] - locations[i - 1]["totalTime"]
        if dt <= 0:
            continue
        v.append((locations[i]["totalDis"] - locations[i - 1]["totalDis"]) / dt)
        c.append((locations[i]["steps"] - locations[i - 1]["steps"]) * 60.0 / dt)
        t.append(locations[i]["totalTime"])
        d.append(locations[i]["totalDis"])
    return v, c, t, d


def windowed(locations, target_s):
    """按 target_s 秒聚合窗口速度/步频。"""
    ov, oc = [], []
    acc_t = acc_d = acc_s = 0.0
    cur = 0.0
    for i in range(1, len(locations)):
        dt = locations[i]["totalTime"] - locations[i - 1]["totalTime"]
        if dt <= 0:
            continue
        acc_t += dt
        acc_d += locations[i]["totalDis"] - locations[i - 1]["totalDis"]
        acc_s += locations[i]["steps"] - locations[i - 1]["steps"]
        cur += dt
        if cur >= target_s - 1e-9:
            ov.append(acc_d / acc_t)
            oc.append(acc_s * 60.0 / acc_t)
            acc_t = acc_d = acc_s = 0.0
            cur = 0.0
    return ov, oc


def low_runs(speeds, gate):
    runs, k = [], 0
    while k < len(speeds):
        if speeds[k] >= gate:
            k += 1
            continue
        end = k
        while end < len(speeds) and speeds[end] < gate:
            end += 1
        runs.append({"start": k, "length": end - k})
        k = end
    return runs


def km_buckets(speeds, dists):
    """按累计里程每 km 一桶的平均速度与桶内点数。"""
    buckets, idx = [], 0
    total = int(dists[-1] // 1000)
    for km in range(total + 1):
        lo, hi = km * 1000.0, (km + 1) * 1000.0
        sel = [s for s, dd in zip(speeds, dists) if lo <= dd < hi]
        if sel:
            buckets.append({"km": km, "n": len(sel), "mean": sum(sel) / len(sel)})
    return buckets


def metrics(locations):
    v, c, t, d = landed_series(locations)
    if not v:
        return None
    mean = sum(v) / len(v)
    gate = LOW_RATIO * mean
    runs = low_runs(v, gate)
    cad_res = [x - empirical_cadence(s) for x, s in zip(c, v)]
    str_res = [60.0 * s / x - empirical_stride(s) for x, s in zip(c, v) if x > 0]
    w10_v, w10_c = windowed(locations, 10)
    w30_v, w30_c = windowed(locations, 30)
    w10_res = [x - empirical_cadence(s) for x, s in zip(w10_c, w10_v)]
    w30_res = [x - empirical_cadence(s) for x, s in zip(w30_c, w30_v)]
    total = d[-1]
    buckets = km_buckets(v, d)
    slope = None
    if len(buckets) >= 2:
        slope = buckets[-1]["mean"] - buckets[-2]["mean"]
    start_points = [s for s, dd in zip(v, d) if dd <= 600.0]
    end_points = [s for s, dd in zip(v, d) if dd > total - 100.0]
    return {
        "n": len(v),
        "speed": {
            "mean": mean, "cv": statistics.pstdev(v) / mean,
            "p1": percentile(v, .01), "p5": percentile(v, .05), "p50": percentile(v, .50),
            "p95": percentile(v, .95), "p99": percentile(v, .99),
            "min": min(v), "max": max(v),
        },
        "cadence_vs_empirical": {
            "mean": sum(cad_res) / len(cad_res), "median": statistics.median(cad_res),
            "p5": percentile(cad_res, .05), "p95": percentile(cad_res, .95),
            "in_window_10s_mean": (sum(w10_res) / len(w10_res)) if w10_res else None,
            "in_window_30s_mean": (sum(w30_res) / len(w30_res)) if w30_res else None,
        },
        "stride_vs_empirical": {
            "mean": sum(str_res) / len(str_res), "median": statistics.median(str_res),
            "p5": percentile(str_res, .05), "p95": percentile(str_res, .95),
        },
        "elasticity": {
            "point": elasticity(v, c),
            "window_10s": elasticity(w10_v, w10_c),
            "window_30s": elasticity(w30_v, w30_c),
            "target": CAD_EXP,
        },
        "corr_speed_cadence": {
            "point": corr(v, c), "window_10s": corr(w10_v, w10_c),
            "window_30s": corr(w30_v, w30_c),
        },
        "low_speed": {
            "gate": gate, "count": len(runs),
            "longest": max([r["length"] for r in runs], default=0),
            "length_hist": {str(n): sum(1 for r in runs if r["length"] == n)
                            for n in sorted({r["length"] for r in runs})},
        },
        "start_peak_600m": max(start_points) if start_points else None,
        "last_100m_mean": (sum(end_points) / len(end_points)) if end_points else None,
        "fatigue_buckets": buckets,
        "last_two_bucket_slope": slope,
    }


def build(mod, dist, dur, seed, cadence_target, points):
    return mod.build(float(dist), float(dur), seed, 1000, points,
                     ordered_path=True, cadence_target=cadence_target)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", default="1000,3000,8000")
    ap.add_argument("--dur", default="334,1000,2700")
    ap.add_argument("--seed", default="1,7,42")
    ap.add_argument("--cadence", default="0,168")
    ap.add_argument("--out", default=os.path.join(TMP_DIR, "gait_empirics.json"))
    args = ap.parse_args(argv)
    dists = [float(x) for x in args.dist.split(",")]
    durs = [float(x) for x in args.dur.split(",")]
    if len(durs) == 1:
        durs = durs * len(dists)
    seeds = [int(x) for x in args.seed.split(",")]
    cads = [float(x) for x in args.cadence.split(",")]
    points = campus_points()
    old_realism, old_gen = legacy_modules()

    cases = []
    for i, dist in enumerate(dists):
        dur = durs[min(i, len(durs) - 1)]
        for seed in seeds:
            for cad in cads:
                entry = {"dist": dist, "dur": dur, "seed": seed, "cadence_target": cad}
                try:
                    track = build(generator, dist, dur, seed, cad, points)
                    entry["after"] = metrics(track["locations"])
                except ValueError as exc:
                    entry["after"] = {"error": str(exc)}
                if old_gen is not None:
                    try:
                        old_track = build(old_gen, dist, dur, seed, cad, points)
                        entry["before"] = metrics(old_track["locations"])
                    except ValueError as exc:
                        entry["before"] = {"error": str(exc)}
                cases.append(entry)

    # 纯函数对比：同一条 v 网格上的旧/新步频曲线（旧实现是本分段的平坦截断）
    grid = [round(0.25 * k, 2) for k in range(4, 25)]
    curve = {
        "v": grid,
        "new": [realism.cadence_for_speed(v) for v in grid],
        "old": ([old_realism.cadence_for_speed(v) for v in grid]
                if old_realism is not None else None),
        "empirical": [empirical_cadence(v) for v in grid],
    }
    report = {
        "constants": {
            "cadence_empirical": "%.4f*(v/%.1f)^%.2f" % (CAD_REF_SPM, CAD_REF_V, CAD_EXP),
            "stride_empirical": "%.2f*(v/%.1f)^%.2f" % (STR_REF_M, CAD_REF_V, STR_EXP),
            "cadence_floor_spm": realism.CADENCE_FLOOR_SPM,
            "cadence_floor_speed": realism.CADENCE_FLOOR_SPEED,
            "low_ratio": LOW_RATIO, "low_run_max": realism.LOW_SPEED_RUN_MAX,
            "speed_cv_target": realism.SPEED_CV_TARGET,
        },
        "cases": cases,
        "cadence_curve": curve,
        "legacy_ref": LEGACY_REF if old_realism is not None else None,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1, sort_keys=True)
    print_summary(report)
    print("JSON -> %s" % args.out)
    return 0


def print_summary(report):
    print("经验式：cadence = %s   stride = %s" % (
        report["constants"]["cadence_empirical"], report["constants"]["stride_empirical"]))
    head = ("case", "cv", "p1", "p5", "p95", "corrP", "corr10", "corr30",
            "elaP", "ela10", "cadRes", "strRes", "run", "startPk", "last100", "kmSlope")
    print("%-20s" % head[0] + "".join("%9s" % h for h in head[1:]))
    for case in report["cases"]:
        for arm in ("before", "after"):
            m = case.get(arm)
            if m is None:
                continue
            tag = "%s/%s" % ("%g/%g/s%d/c%g" % (case["dist"], case["dur"], case["seed"],
                                               case["cadence_target"]), arm[:3])
            if "error" in m:
                print("%-20s ERROR %s" % (tag, m["error"][:60]))
                continue
            print("%-20s%9.3f%9.2f%9.2f%9.2f%9.3f%9.3f%9.3f%9.3f%9.3f%9.2f%9.3f%9d%9.2f%9.2f%9s" % (
                tag, m["speed"]["cv"], m["speed"]["p1"], m["speed"]["p5"], m["speed"]["p95"],
                m["corr_speed_cadence"]["point"], m["corr_speed_cadence"]["window_10s"],
                m["corr_speed_cadence"]["window_30s"], m["elasticity"]["point"],
                m["elasticity"]["window_10s"], m["cadence_vs_empirical"]["mean"],
                m["stride_vs_empirical"]["mean"], m["low_speed"]["longest"],
                m["start_peak_600m"] or float("nan"), m["last_100m_mean"] or float("nan"),
                ("%.3f" % m["last_two_bucket_slope"]) if m["last_two_bucket_slope"] is not None else "-"))
    curve = report["cadence_curve"]
    if curve.get("old"):
        print("步频曲线 v=%.2f..%.2f: old %s" % (curve["v"][0], curve["v"][-1],
                                                [round(x, 1) for x in curve["old"][::4]]))
        print("                        new %s" % [round(x, 1) for x in curve["new"][::4]])
        print("                        emp %s" % [round(x, 1) for x in curve["empirical"][::4]])


if __name__ == "__main__":
    raise SystemExit(main())
