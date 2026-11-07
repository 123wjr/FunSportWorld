# -*- coding: utf-8 -*-
"""Independent verifier for the humanized route + motion model (task-6).

Read-only w.r.t. the repo: only reads .funsport/campus_loop_bd.json and imports
funsport.track.{humanize,generator,realism}. Writes nothing except the JSON dump
under .test-tmp/verify/ and stdout.

Usage:
  python -B tools/verify_humanized.py              # full run (blocked-safe)
  python -B tools/verify_humanized.py --before-only
  python -B tools/verify_humanized.py --no-motion
"""
import argparse
import importlib
import json
import math
import os
import statistics as st
import sys
import time
import traceback

sys.stdout.reconfigure(encoding="utf-8")
try:
    import numpy as np
except Exception:                                    # pragma: no cover
    np = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
DATA = os.path.join(ROOT, ".funsport", "campus_loop_bd.json")
OUTDIR = os.path.join(ROOT, ".test-tmp", "verify")
OUT_JSON = os.path.join(OUTDIR, "metrics.json")

# ─────────────────────────── projection ───────────────────────────
# Local equirectangular metric frame; adequate (<0.1% error) over a 1.5 km loop.
_ORIGIN = None
_MLAT = _MLNG = None


def set_origin(lat, lng):
    global _ORIGIN, _MLAT, _MLNG
    p = math.radians(lat)
    _ORIGIN = (lat, lng)
    _MLAT = 111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    _MLNG = 111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)


def xy(p):
    return ((p[1] - _ORIGIN[1]) * _MLNG, (p[0] - _ORIGIN[0]) * _MLAT)


def ll(q):
    return (_ORIGIN[0] + q[1] / _MLAT, _ORIGIN[1] + q[0] / _MLNG)


def seg_len(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


# ─────────────────────────── route loading ───────────────────────────
def load_route():
    """Min-length candidate per leg, concatenated into a closed ring (BD-09)."""
    with open(DATA, encoding="utf-8") as fh:
        d = json.load(fh)
    legs = d["legs"]
    first = sorted(legs[0], key=lambda c: c.get("distance", 0))[0]["points"][0]
    set_origin(float(first[0]), float(first[1]))
    pts = []
    for leg in legs:
        cand = sorted(leg, key=lambda c: c.get("distance", 0))[0]["points"]
        seg = [[float(p[0]), float(p[1])] for p in cand]
        if pts and seg and math.hypot((pts[-1][0] - seg[0][0]) * _MLAT,
                                      (pts[-1][1] - seg[0][1]) * _MLNG) < 2.0:
            seg = seg[1:]
        pts.extend(seg)
    if pts[0] != pts[-1]:
        pts.append(list(pts[0]))
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot((out[-1][0] - p[0]) * _MLAT, (out[-1][1] - p[1]) * _MLNG) >= 0.5:
            out.append(p)
    if out[-1] != out[0]:
        if math.hypot((out[-1][0] - out[0][0]) * _MLAT,
                      (out[-1][1] - out[0][1]) * _MLNG) < 0.5:
            out[-1] = list(out[0])
        else:
            out.append(list(out[0]))
    return out, len(legs)


def resample(pts, step):
    """Uniform arc-length resample of a closed polyline. Returns (xy list, step used)."""
    P = [xy(p) for p in pts]
    cum = [0.0]
    for i in range(len(P) - 1):
        cum.append(cum[-1] + seg_len(P[i], P[i + 1]))
    total = cum[-1]
    n = max(8, int(round(total / step)))
    h = total / n
    out, j = [], 0
    for k in range(n):
        s = k * h
        while j + 2 < len(cum) and cum[j + 1] < s:
            j += 1
        a, b = P[j], P[j + 1]
        sl = cum[j + 1] - cum[j]
        t = 0.0 if sl <= 0 else (s - cum[j]) / sl
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out, h, total


# ─────────────────────────── stats helpers ───────────────────────────
def pct(vals, q):
    v = sorted(vals)
    if not v:
        return float("nan")
    k = (len(v) - 1) * q
    f = int(math.floor(k))
    c = min(f + 1, len(v) - 1)
    return v[f] + (v[c] - v[f]) * (k - f)


def stats(vals, prefix):
    if not vals:
        return {}
    return {prefix + k: v for k, v in (
        ("p50", pct(vals, .5)), ("p90", pct(vals, .9)), ("p95", pct(vals, .95)),
        ("max", max(vals)), ("min", min(vals)), ("mean", sum(vals) / len(vals)))}


def corr(a, b):
    n = min(len(a), len(b))
    if n < 3:
        return float("nan")
    a, b = list(a[:n]), list(b[:n])
    ma, mb = sum(a) / n, sum(b) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((y - mb) ** 2 for y in b))
    return num / (da * db) if da and db else float("nan")


# ─────────────────────────── A: turns / curvature ───────────────────────────
def vertex_turns(pts):
    P = [xy(p) for p in pts]
    ang = []
    for i in range(1, len(P) - 1):
        v1 = (P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1])
        v2 = (P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 < 0.05 or n2 < 0.05:      # sub-5cm segments are numeric noise
            continue
        c = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        ang.append(math.degrees(math.acos(c)))
    return ang


def menger_curvature(Q, h):
    """Discrete curvature of a uniformly sampled polyline (1/m)."""
    k = []
    for i in range(1, len(Q) - 1):
        a = seg_len(Q[i - 1], Q[i])
        b = seg_len(Q[i], Q[i + 1])
        c = seg_len(Q[i - 1], Q[i + 1])
        if a < 1e-9 or b < 1e-9 or c < 1e-9:
            k.append(0.0)
            continue
        ux, uy = Q[i][0] - Q[i - 1][0], Q[i][1] - Q[i - 1][1]
        vx, vy = Q[i + 1][0] - Q[i][0], Q[i + 1][1] - Q[i][1]
        cross = abs(ux * vy - uy * vx)
        k.append(2.0 * cross / (a * b * c))
    return k


def curvature_stats(pts, step=1.0):
    Q, h, total = resample(pts, step)
    k = menger_curvature(Q, h)
    dk = [abs(k[i + 1] - k[i]) for i in range(len(k) - 1)]
    kmax = max(k) if k else 0.0
    out = {}
    out.update({f"curv_{a}": b for a, b in stats(k, "").items()})
    out.update({"dk_" + a: b for a, b in stats(dk, "").items()})
    out["curv_occ_frac"] = (sum(1 for x in k if x > 0.01) / len(k)) if k else 0.0
    nz = [x for x in k if x > 0.01]
    out["curv_nz_p50"] = pct(nz, .5) if nz else 0.0
    out["curv_nz_p95"] = pct(nz, .95) if nz else 0.0
    out["curv_nz_max"] = max(nz) if nz else 0.0
    out["minR_m"] = (1.0 / kmax) if kmax > 0 else float("inf")
    out["n_samples"] = len(Q)
    out["step_m"] = h
    return out, Q, h, k


# ─────────────────────────── B: straight segments ───────────────────────────
def rdp(P, eps):
    if len(P) < 3:
        return list(range(len(P)))
    keep = [0, len(P) - 1]
    stack = [(0, len(P) - 1)]
    while stack:
        i, j = stack.pop()
        a, b = P[i], P[j]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        best, bi = -1.0, -1
        for t in range(i + 1, j):
            p = P[t]
            if L < 1e-9:
                dd = math.hypot(p[0] - a[0], p[1] - a[1])
            else:
                dd = abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / L
            if dd > best:
                best, bi = dd, t
        if best > eps and bi > 0:
            keep.append(bi)
            stack.append((i, bi))
            stack.append((bi, j))
    return sorted(set(keep))


def straight_segments(pts, eps=0.4, min_len=30.0, max_turn=5.0):
    """Arc intervals [s0,s1] of the reference polyline that are long and near-straight."""
    P = [xy(p) for p in pts]
    cum = [0.0]
    for i in range(len(P) - 1):
        cum.append(cum[-1] + seg_len(P[i], P[i + 1]))
    idx = rdp(P, eps)
    segs = []
    for a, b in zip(idx[:-1], idx[1:]):
        L = cum[b] - cum[a]
        if L < min_len:
            continue
        # interior original vertices must not turn more than max_turn
        mx = 0.0
        for i in range(a + 1, b):
            v1 = (P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1])
            v2 = (P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
            n1, n2 = math.hypot(*v1), math.hypot(*v2)
            if n1 < 1e-6 or n2 < 1e-6:
                continue
            c = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            mx = max(mx, math.degrees(math.acos(c)))
        if mx <= max_turn:
            segs.append({"s0": cum[a], "s1": cum[b], "len": L, "A": P[a], "B": P[b],
                         "max_turn": mx})
    return segs, cum[-1]


def line_residual(Q, A, B):
    """RMS + max perpendicular deviation of Q against the chord A→B (metres)."""
    dx, dy = B[0] - A[0], B[1] - A[1]
    L = math.hypot(dx, dy)
    if L < 1e-9 or not Q:
        return float("nan"), float("nan"), []
    ux, uy = dx / L, dy / L
    r = []
    for p in Q:
        rx, ry = p[0] - A[0], p[1] - A[1]
        t = rx * ux + ry * uy
        if t < 0 or t > L:
            continue                                    # outside the chord span
        r.append(-rx * uy + ry * ux)                    # signed left-positive
    if not r:
        return float("nan"), float("nan"), []
    rms = math.sqrt(sum(x * x for x in r) / len(r))
    return rms, max(abs(x) for x in r), r


# ─────────────────────────── alignment (after → before arc) ───────────────────────────
def polyline_segments(pts):
    """(P0, P1, arc0, seglen, total) of the raw polyline in the metric frame."""
    P = [xy(p) for p in pts]
    P0, P1, arcs, lens = [], [], [], []
    a = 0.0
    for i in range(len(P) - 1):
        L = seg_len(P[i], P[i + 1])
        P0.append(P[i]); P1.append(P[i + 1]); arcs.append(a); lens.append(L)
        a += L
    return P0, P1, arcs, lens, a


def proj_to_polyline(pts, seg):
    """Exact nearest point of each pts[i] onto the polyline `seg`.

    Brute force over segments; the reference loop has ~35 segments so this is
    exact and cheap.  Returns (dist, arc_at_nearest, seg_index, t).
    """
    P0, P1, arcs, lens, total = seg
    out_d, out_arc, out_i, out_t = [], [], [], []
    for q in pts:
        best, bj, bt = 1e18, 0, 0.0
        for j in range(len(P0)):
            ax, ay = P0[j]
            dx = P1[j][0] - ax
            dy = P1[j][1] - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 <= 1e-12 else ((q[0] - ax) * dx + (q[1] - ay) * dy) / L2
            t = 0.0 if t < 0 else (1.0 if t > 1 else t)
            d = math.hypot(q[0] - (ax + t * dx), q[1] - (ay + t * dy))
            if d < best:
                best, bj, bt = d, j, t
        out_d.append(best)
        out_arc.append(arcs[bj] + bt * lens[bj])
        out_i.append(bj)
        out_t.append(bt)
    return out_d, out_arc, out_i, out_t


def nearest_to_stretch(pts, seg, arc_lo, arc_hi):
    """Distance from each pts[i] to the polyline stretch spanning arcs [arc_lo, arc_hi]."""
    P0, P1, arcs, lens, total = seg
    out = []
    for q in pts:
        best = 1e18
        for j in range(len(P0)):
            s0, s1 = arcs[j], arcs[j] + lens[j]
            if s1 <= arc_lo or s0 >= arc_hi or lens[j] <= 1e-12:
                continue
            t0 = max(0.0, (arc_lo - s0) / lens[j])
            t1 = min(1.0, (arc_hi - s0) / lens[j])
            if t1 < t0:
                continue
            ax, ay = P0[j]
            dx = P1[j][0] - ax
            dy = P1[j][1] - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 <= 1e-12 else ((q[0] - ax) * dx + (q[1] - ay) * dy) / L2
            t = max(t0, min(t1, t))
            d = math.hypot(q[0] - (ax + t * dx), q[1] - (ay + t * dy))
            if d < best:
                best = d
        out.append(best)
    return out

def densify_before(pts, step=0.25):
    Q, h, total = resample(pts, step)
    s = [i * h for i in range(len(Q))]
    return Q, s, total


def align_to_before(after_pts, before_pts, win_back=3.0, win_fwd=8.0):
    """Monotone windowed nearest-point alignment. Returns (dist[], s[])."""
    BQ, Bs, Btot = densify_before(before_pts)
    AQ, Ah, Atot = resample(after_pts, 0.5)
    n = len(BQ)
    dists, prog = [], []
    idx = 0
    for p in AQ:
        lo_f = ((prog[-1] if prog else 0.0) - win_back) / (Btot / n)
        hi_f = ((prog[-1] if prog else 0.0) + win_fwd) / (Btot / n)
        lo = max(0, int(math.floor(lo_f)))
        hi = min(n - 1, int(math.ceil(hi_f)))
        if hi <= lo:
            lo, hi = 0, n - 1
        best, bi = 1e18, lo
        for i in range(lo, hi + 1):
            d = (BQ[i][0] - p[0]) ** 2 + (BQ[i][1] - p[1]) ** 2
            if d < best:
                best, bi = d, i
        dists.append(math.sqrt(best))
        prog.append(Bs[bi])
    return dists, prog, AQ, Ah, Atot


# ─────────────────────────── C: repeat corridors ───────────────────────────
def repeat_corridors(pts, cell=5.0, sep=25.0, tol=2.0):
    Q, h, total = resample(pts, 0.5)
    n = len(Q)
    def key(p):
        return (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)))
    grid = {}
    for i, p in enumerate(Q):
        grid.setdefault(key(p), []).append(i)
    pair = {}
    for i, p in enumerate(Q):
        K = key(p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((K[0] + dx, K[1] + dy), ()):
                    if j <= i:
                        continue
                    ds = abs(j - i) * h
                    ds = min(ds, total - ds)
                    if ds < sep:
                        continue
                    d = seg_len(p, Q[j])
                    if d < tol:
                        pair[(i, j)] = d
    if not pair:
        return [], total, Q, h, n
    # contiguous runs on the lower index
    idx = sorted({k[0] for k in pair})
    runs, s0, prev = [], idx[0], idx[0]
    for x in idx[1:]:
        if x - prev > 6:
            runs.append((s0, prev))
            s0 = x
        prev = x
    runs.append((s0, prev))
    out = []
    for a, b in runs:
        if (b - a) * h < 10.0:
            continue
        # median partner index for each i in the run: robust against a corridor whose
        # partner set is polluted by a distant stretch that happens to be nearby
        js = {}
        for (i, j) in pair:
            if a <= i <= b:
                js.setdefault(i, []).append(j)
        if len(js) < 5:
            continue
        med = {i: sorted(v)[len(v) // 2] for i, v in js.items()}
        ii = sorted(med)
        j0, j1 = med[ii[0]], med[ii[-1]]
        ds = [pair[k] for k in pair if a <= k[0] <= b]
        out.append({"sA0": ii[0] * h, "sA1": ii[-1] * h, "sB0": min(j0, j1) * h,
                    "sB1": max(j0, j1) * h, "lenA": (ii[-1] - ii[0]) * h,
                    "lenB": abs(j1 - j0) * h, "min_d": min(ds),
                    "p50_d": sorted(ds)[len(ds) // 2], "max_d": max(ds),
                    "dir": 1 if j1 > j0 else -1,
                    "i0": ii[0], "i1": ii[-1], "j0": min(j0, j1), "j1": max(j0, j1)})
    return out, total, Q, h, n


def pass_sep(Q, c):
    """Half-width between one corridor's two passes, measured on polyline Q itself.

    Pass A is the index window [i0,i1]; pass B is [j0,j1].  For each sample of A we
    take the exact distance to the B window (its own polyline), and vice versa, so
    the number reported is the perpendicular gap actually left between the two
    times the runner covered this stretch.
    """
    out = []
    trim = max(1, int(round(5.0 / 0.5)))       # skip 5 m at each end where passes merge
    for (a0, a1, b0, b1) in ((c["i0"] + trim, c["i1"] - trim, c["j0"] + trim, c["j1"] - trim),
                             (c["j0"] + trim, c["j1"] - trim, c["i0"] + trim, c["i1"] - trim)):
        A = Q[a0:a1 + 1]
        if len(A) < 3 or b1 <= b0:
            continue
        B = Q[b0:b1 + 1]
        step = max(1, len(A) // 400)
        for q in A[::step]:
            best = 1e18
            for k in range(len(B) - 1):
                ax, ay = B[k]
                dx, dy = B[k + 1][0] - ax, B[k + 1][1] - ay
                L2 = dx * dx + dy * dy
                t = 0.0 if L2 <= 1e-12 else ((q[0] - ax) * dx + (q[1] - ay) * dy) / L2
                t = 0.0 if t < 0 else (1.0 if t > 1 else t)
                d = math.hypot(q[0] - (ax + t * dx), q[1] - (ay + t * dy))
                if d < best:
                    best = d
            out.append(best)
    return out


def arc_map(after_pts, before_total):
    """Monotone correspondence after-sample -> before-arc, by route fraction.

    humanize preserves arc length (ratio ~1.00), so fraction-of-loop is the only
    correspondence that stays monotone across the repeated stretches where two
    passes of the SAME ground are ~1 m apart (nearest-point picking would flip
    between them and destroy any arc-parameterised statistic).
    """
    AQ, Ah, Atot = resample(after_pts, 0.5)
    m = before_total / Atot
    return AQ, Ah, Atot, [i * Ah * m for i in range(len(AQ))], before_total

def scale_win(s0, s1, before_total, n_after):
    """Map before-side arc bounds onto after-side sample indices."""
    k = (n_after - 1) / float(before_total)
    a = max(0, int(math.floor(s0 * k)))
    b = min(n_after - 1, int(math.ceil(s1 * k)))
    return a, b

def pos_at_progress(AQ, Ah, prog, target):
    """Interpolate a position along the after polyline at a given before-arc target."""
    for i in range(1, len(prog)):
        if prog[i] >= target:
            p0, p1 = prog[i - 1], prog[i]
            t = 0.0 if p1 <= p0 else (target - p0) / (p1 - p0)
            a, b = AQ[i - 1], AQ[i]
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    return AQ[-1]


def pos_at_arc(Q, h, target):
    """Position at a given arc length along a uniformly sampled polyline."""
    total = (len(Q)) * h
    if target <= 0:
        return Q[0]
    if target >= total:
        return Q[-1]
    i = int(target / h)
    i = max(0, min(len(Q) - 2, i))
    f = (target - i * h) / h
    a, b = Q[i], Q[i + 1]
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)


def sep_between_passes(AQ, segRef, corr, step_pts=None):
    """Separation between the two passes of one repeat corridor.

    For every sample of pass A, take its offset to the nearest point of pass B
    (measured against the *reference* polyline restricted to B's arc window).
    This is the perpendicular gap the runner actually leaves between the two
    times they ride this stretch; arc-parameter interpolation is wrong here
    because both passes are far apart along the loop even though they are close
    on the ground.
    """
    arc_lo, arc_hi = min(corr["sB0"], corr["sB1"]), max(corr["sB0"], corr["sB1"])
    a_lo, a_hi = min(corr["sA0"], corr["sA1"]), max(corr["sA0"], corr["sA1"])
    Pa = [p for p, arc in AQ if a_lo <= arc <= a_hi]
    if len(Pa) < 5:
        Pa = [p for p, arc in AQ]
    return nearest_to_stretch([p for p in Pa], segRef, arc_lo, arc_hi)


def corridor_separation(corr, AQ, prog, before_total):
    """Lateral separation between the two passes of one repeat corridor."""
    lens = min(corr["lenA"], corr["lenB"])
    k = max(5, int(lens / 2.0))
    seps = []
    for t in range(k + 1):
        f = t / k
        pA = corr["sA0"] + f * corr["lenA"]
        pB = corr["sB0"] + f * corr["lenB"]
        a = pos_at_progress(AQ, None, prog, pA)
        b = pos_at_progress(AQ, None, prog, pB)
        seps.append(seg_len(a, b))
    return seps


# ─────────────────────────── D: U-turns ───────────────────────────
def u_turns(pts, thresh=150.0):
    P = [xy(p) for p in pts]
    cum = [0.0]
    for i in range(len(P) - 1):
        cum.append(cum[-1] + seg_len(P[i], P[i + 1]))
    out = []
    for i in range(1, len(P) - 1):
        v1 = (P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1])
        v2 = (P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 < 0.05 or n2 < 0.05:
            continue
        c = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        ang = math.degrees(math.acos(c))
        if ang > thresh:
            out.append({"i": i, "s": cum[i], "turn": ang, "p": P[i]})
    return out


def self_intersections(Q, min_arc=5.0, h=1.0):
    """Count self-crossings among segments whose arc separation exceeds min_arc."""
    n = len(Q)
    hits = 0
    worst = 1e18
    def seg_int(p1, p2, p3, p4):
        def d(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        d1 = d(p3, p4, p1); d2 = d(p3, p4, p2)
        d3 = d(p1, p2, p3); d4 = d(p1, p2, p4)
        return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))
    if n > 1200:                       # keep the O(n²) bounded for long loops
        return None, None
    for i in range(n - 1):
        for j in range(i + 4, n - 1):
            if min(abs(i - j), n - abs(i - j)) * h < min_arc:
                continue
            if seg_int(Q[i], Q[i + 1], Q[j], Q[j + 1]):
                hits += 1
    return hits, worst


# ─────────────────────────── geometry bundle ───────────────────────────
def geom_metrics(pts, tag):
    Q, h, total = resample(pts, 0.5)
    closed_err = seg_len(xy(pts[0]), xy(pts[-1]))
    turns = vertex_turns(pts)
    cstats, CQ, ch, ck = curvature_stats(pts, 1.0)
    segs, ref_total = straight_segments(pts)
    # residual of THIS polyline against its own RDP chords (before is the reference)
    res_all = []
    for sg in segs:
        mask = [p for i, p in enumerate(Q) if sg["s0"] <= i * h <= sg["s1"]]
        rms, mx, r = line_residual(mask, sg["A"], sg["B"])
        sg["self_rms"] = rms
        sg["self_max"] = mx
        res_all.extend(r)
    lengths = [seg_len(xy(pts[i]), xy(pts[i + 1])) for i in range(len(pts) - 1)]
    return {
        "tag": tag,
        "n_vertices": len(pts),
        "length_m": ref_total,
        "closure_err_m": closed_err,
        "turn_n": len(turns),
        "turn_p50": pct(turns, .5), "turn_p90": pct(turns, .9),
        "turn_max": max(turns) if turns else float("nan"),
        "turn_gt45": sum(1 for x in turns if x > 45),
        "turn_gt90": sum(1 for x in turns if x > 90),
        "turn_gt135": sum(1 for x in turns if x > 135),
        "turn_gt150": sum(1 for x in turns if x > 150),
        "spacing_min": min(lengths), "spacing_p50": pct(lengths, .5),
        "spacing_max": max(lengths),
        **cstats,
        "straight_n": len(segs),
        "straight_total_m": sum(s["len"] for s in segs),
        "straight_rms_max": max([s["self_rms"] for s in segs], default=float("nan")),
        "straight_rms_mean": (sum(s["self_rms"] for s in segs) / len(segs)) if segs else float("nan"),
        "_segments": segs,
        "_residuals": res_all,
    }


# ─────────────────────────── motion model ───────────────────────────
def run_motion(points_bd, tag, seed=7, target_v=2.8, sample_period=2):
    from funsport.track import generator, realism
    Q, h, total = resample(points_bd, 0.5)
    dist = round(total)
    dur = int(round(total / target_v))
    t0 = time.time()
    tr = generator.build(dist, dur, seed, 20261107000000, points_bd,
                         ordered_path=True, sample_period_s=sample_period,
                         open_path=False, distance_mode="arc")
    elapsed = time.time() - t0
    locs = tr["locations"]
    v, cad, gaps, stride = [], [], [], []
    for i in range(1, len(locs)):
        dt = locs[i]["totalTime"] - locs[i - 1]["totalTime"]
        dd = locs[i]["totalDis"] - locs[i - 1]["totalDis"]
        dstep = locs[i]["steps"] - locs[i - 1]["steps"]
        gaps.append(dt)
        if dt > 0:
            v.append(dd / dt)
            cad.append(dstep * 60.0 / dt)
            stride.append(dd / dstep if dstep > 0 else float("nan"))
    mean_v = sum(v) / len(v)
    cv = st.pstdev(v) / mean_v if len(v) > 1 else float("nan")
    # longest run of v < 0.6*mean
    runs, cur = [], 0
    for x in v:
        if x < 0.6 * mean_v:
            cur += 1
            runs.append(cur)
        else:
            cur = 0
    # empirical laws
    emp_cad = [171.43 * (x / 3.0) ** 0.28 for x in v]
    emp_str = [1.05 * (x / 3.0) ** 0.72 for x in v]
    res_cad = [c - e for c, e in zip(cad, emp_cad)]
    res_str = [s - e for s, e in zip(stride, emp_str) if s == s]
    eps = [math.log(max(x, 1e-6)) for x in v]
    logc = [math.log(max(c, 1e-6)) for c in cad]
    # dln cad / dln v via least squares slope
    me, mc = sum(eps) / len(eps), sum(logc) / len(logc)
    den = sum((x - me) ** 2 for x in eps)
    slope = sum((x - me) * (y - mc) for x, y in zip(eps, logc)) / den if den > 0 else float("nan")
    # 10 s / 30 s windows
    def windows(win):
        wv, wc = [], []
        i = 0
        n = len(locs)
        while i < n - 1:
            j = i
            while j + 1 < n and locs[j + 1]["totalTime"] - locs[i]["totalTime"] <= win:
                j += 1
            if j > i:
                dt = locs[j]["totalTime"] - locs[i]["totalTime"]
                if dt >= win * 0.8:
                    wv.append((locs[j]["totalDis"] - locs[i]["totalDis"]) / dt)
                    wc.append((locs[j]["steps"] - locs[i]["steps"]) * 60.0 / dt)
            i = j
        return wv, wc
    w10v, w10c = windows(10)
    w30v, w30c = windows(30)
    out = {
        "tag": tag, "dist_m": dist, "dur_s": dur, "n_points": len(locs),
        "elapsed_s": round(elapsed, 2),
        "v_mean": mean_v, "v_cv": cv,
        "v_p1": pct(v, .01), "v_p5": pct(v, .05), "v_p50": pct(v, .5),
        "v_p95": pct(v, .95), "v_p99": pct(v, .99),
        "v_min": min(v), "v_max": max(v),
        "low_run_max": max(runs) if runs else 0,
        "low_n": sum(1 for x in v if x < 0.6 * mean_v),
        "cad_p50": pct(cad, .5), "cad_mean": sum(cad) / len(cad),
        "cad_min": min(cad), "cad_max": max(cad),
        "res_cad_p50": pct(res_cad, .5), "res_cad_p95": pct([abs(x) for x in res_cad], .95),
        "res_str_p50": pct(res_str, .5), "res_str_p95": pct([abs(x) for x in res_str], .95),
        "elasticity": slope,
        "corr_v_cad_point": corr(v, cad),
        "corr_v_cad_10s": corr(w10v, w10c),
        "corr_v_cad_30s": corr(w30v, w30c),
        "gap_hist": dict(sorted({g: gaps.count(g) for g in set(gaps)}.items())[:8]),
        "gap_p50": pct(gaps, .5), "gap_dominant_share": max(gaps.count(g) for g in set(gaps)) / len(gaps),
        "total_distance": tr["totalDistance"],
        "sim_dis_err": tr["totalDistance"] - dist,
        "avg_speed_field": (tr["totalDistance"] / tr["totalTime"]) if tr["totalTime"] else float("nan"),
        "steps_total": tr["totalSteps"],
    }
    return out, tr


# ─────────────────────────── humanize import ───────────────────────────
def try_humanize():
    info = {"available": False}
    try:
        mod = importlib.import_module("funsport.track.humanize")
    except Exception as exc:
        info["error"] = f"{type(exc).__name__}: {exc}"
        return None, info
    # accept the documented name first, then whatever the module actually exposes
    fn = None
    for cand in ("humanize_loop", "humanize_route", "humanize"):
        fn = getattr(mod, cand, None)
        if callable(fn):
            info["entry"] = cand
            break
        fn = None
    if fn is None:
        info["error"] = "module present but no humanize entry point"
        info["names"] = [n for n in dir(mod) if not n.startswith("_")]
        return None, info
    info["defaults"] = {k: v for k, v in (getattr(mod, "DEFAULTS", {}) or {}).items()}
    fn_knobs = dict(getattr(mod, "DEFAULTS", {}) or {})
    import inspect
    try:
        info["signature"] = str(inspect.signature(fn))
        info["params"] = [p for p in inspect.signature(fn).parameters]
    except Exception:
        pass
    info["has_laps"] = callable(getattr(mod, "humanize_laps", None))
    info["available"] = True
    info["module_names"] = [n for n in dir(mod) if not n.startswith("_")]
    return fn, info


def fmt(x, nd=2):
    if x is None:
        return "na"
    if isinstance(x, str):
        return x
    if isinstance(x, float) and (x != x):
        return "nan"
    if isinstance(x, float) and math.isinf(x):
        return "inf"
    return f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before-only", action="store_true")
    ap.add_argument("--no-motion", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--json", default=OUT_JSON)
    args = ap.parse_args()

    os.makedirs(OUTDIR, exist_ok=True)
    raw, nlegs = load_route()
    set_origin(raw[0][0], raw[0][1])
    before = raw
    res = {"legs": nlegs, "raw_vertices": len(before)}

    gb = geom_metrics(before, "before")
    res["before"] = {k: v for k, v in gb.items() if not k.startswith("_")}
    print("== BEFORE (raw AMap polyline, min-length candidates) ==")
    print(f"  vertices={gb['n_vertices']} length={fmt(gb['length_m'],1)}m closure={fmt(gb['closure_err_m'],3)}m")
    print(f"  turns n={gb['turn_n']} p50={fmt(gb['turn_p50'],1)} p90={fmt(gb['turn_p90'],1)} "
          f"max={fmt(gb['turn_max'],1)} >45={gb['turn_gt45']} >90={gb['turn_gt90']} >135={gb['turn_gt135']}")
    print(f"  curv p50={fmt(gb['curv_p50'],4)} p95={fmt(gb['curv_p95'],4)} max={fmt(gb['curv_max'],4)} minR={fmt(gb['minR_m'],3)}m")
    print(f"  |dk| p50={fmt(gb['dk_p50'],4)} p95={fmt(gb['dk_p95'],4)} max={fmt(gb['dk_max'],4)}")
    print(f"  straight segs={gb['straight_n']} total={fmt(gb['straight_total_m'],1)}m "
          f"rms_max={fmt(gb['straight_rms_max'],3)}m")

    corrs, btot, BQ, Bh, bn = repeat_corridors(before)
    res["before"]["repeat_corridors"] = [{k: round(v, 2) if isinstance(v, float) else v
                                          for k, v in c.items()} for c in corrs]
    res["before"]["repeat_total_m"] = sum(c["lenA"] for c in corrs)
    res["before"]["repeat_ratio"] = sum(c["lenA"] for c in corrs) / btot
    print(f"  repeat corridors={len(corrs)} total={fmt(sum(c['lenA'] for c in corrs),1)}m "
          f"({fmt(100*sum(c['lenA'] for c in corrs)/btot,1)}% of loop)")

    b_seps = [pass_sep(BQ, c) for c in corrs]
    res["before"]["corridor_sep"] = [
        {"lenA": c["lenA"], "min": (min(sp) if sp else float("nan")),
         "p50": (pct(sp, .5) if sp else float("nan")),
         "max": (max(sp) if sp else float("nan")), "n": len(sp)}
        for c, sp in zip(corrs, b_seps)]
    res["before"]["corridor_sep_min"] = min([x["min"] for x in res["before"]["corridor_sep"]],
                                            default=float("nan"))
    res["before"]["corridor_sep_p50"] = pct([x["p50"] for x in res["before"]["corridor_sep"]], .5)
    for c, sp in zip(corrs, b_seps):
        print(f"    corridor A {fmt(c['sA0'],0)}-{fmt(c['sA1'],0)}m  B {fmt(c['sB0'],0)}-{fmt(c['sB1'],0)}m"
              f"  lenA={fmt(c['lenA'],0)}m dir={c['dir']}  pair d min/p50/max="
              f"{fmt(c['min_d'],2)}/{fmt(c['p50_d'],2)}/{fmt(c['max_d'],2)}m"
              f"  pass-sep min/p50/max={fmt(min(sp) if sp else float('nan'),2)}/"
              f"{fmt(pct(sp,.5) if sp else float('nan'),2)}/"
              f"{fmt(max(sp) if sp else float('nan'),2)}m")

    uts = u_turns(before)
    res["before"]["u_turns"] = [{"s": round(u["s"], 1), "turn": round(u["turn"], 1)} for u in uts]
    print(f"  U-turns (>150 deg): {len(uts)} at arcs {[round(u['s'],0) for u in uts]}")

    fn, hinfo = try_humanize()
    res["humanize"] = hinfo
    if fn is None:
        print("\n!! humanize module NOT available:", hinfo.get("error"))
        print("   -> geometry after-side and motion model: BLOCKED")
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(res, fh, ensure_ascii=False, indent=1, default=str)
        print("wrote", args.json)
        return 2

    print(f"\n== {hinfo.get('entry')}: {hinfo.get('signature')}")
    print("   knobs:", hinfo.get("defaults"))
    t0 = time.time()
    after, info7 = fn(before, seed=args.seed)
    t_h = time.time() - t0
    after = [[float(p[0]), float(p[1])] for p in after]
    res["humanize"]["info_keys"] = sorted(info7.keys()) if isinstance(info7, dict) else str(type(info7))
    res["humanize"]["info"] = info7 if isinstance(info7, (int, float, str)) else {
        k: (v if isinstance(v, (int, float, str, bool, list, dict)) and not isinstance(v, list) else str(v))
        for k, v in (info7 or {}).items()}
    res["humanize"]["elapsed_s"] = round(t_h, 3)

    # determinism
    after2, _ = fn(before, seed=args.seed)
    after3, _ = fn(before, seed=args.seed + 1)
    same = [list(map(float, p)) for p in after2] == after
    diff_seed = [list(map(float, p)) for p in after3] != after
    res["determinism"] = {"same_seed_identical": same, "other_seed_differs": diff_seed}

    ga = geom_metrics(after, "after")
    res["after"] = {k: v for k, v in ga.items() if not k.startswith("_")}

    segB = polyline_segments(before)
    AQ, Ah, Atot = arc_map(after, btot)[0], arc_map(after, btot)[1], arc_map(after, btot)[2]
    # exact perpendicular distance from every after sample to the BEFORE polyline
    dists, d_arc, d_seg, d_t = proj_to_polyline(AQ, segB)
    # correspondence by fraction of loop: monotone, and the only thing that stays
    # stable across the repeated stretches where both passes are <2 m apart
    prog = [i * Ah * (btot / Atot) for i in range(len(AQ))]
    res["after"]["dev_p50"] = pct(dists, .5)
    res["after"]["dev_p90"] = pct(dists, .9)
    res["after"]["dev_p95"] = pct(dists, .95)
    res["after"]["dev_max"] = max(dists)
    res["after"]["dev_frac_gt8"] = sum(1 for d in dists if d > 8.0) / len(dists)
    res["after"]["dev_frac_gt20"] = sum(1 for d in dists if d > 20.0) / len(dists)

    # straight-segment lateral wobble on the AFTER polyline, against the original chords
    wob = []
    per_seg = []
    for sg in gb["_segments"]:
        q = [AQ[i] for i in range(len(AQ)) if sg["s0"] <= prog[i] <= sg["s1"]]
        rms, mx, r = line_residual(q, sg["A"], sg["B"])
        per_seg.append({"len": sg["len"], "rms": rms, "max": mx, "n": len(q)})
        wob.extend(r)
    res["after"]["straight_per_seg"] = per_seg
    res["after"]["wobble_abs_p50"] = pct([abs(x) for x in wob], .5) if wob else float("nan")
    res["after"]["wobble_abs_p95"] = pct([abs(x) for x in wob], .95) if wob else float("nan")
    res["after"]["wobble_abs_max"] = max([abs(x) for x in wob], default=float("nan"))
    res["after"]["wobble_abs_mean"] = (sum(abs(x) for x in wob) / len(wob)) if wob else float("nan")

    # second-difference (curvature proxy) restricted to the straight parts
    d2 = []
    for i in range(1, len(AQ) - 1):
        if not any(sg["s0"] <= prog[i] <= sg["s1"] for sg in gb["_segments"]):
            continue
        p0, p1, p2 = AQ[i - 1], AQ[i], AQ[i + 1]
        d2.append(math.hypot(p2[0] - 2 * p1[0] + p0[0], p2[1] - 2 * p1[1] + p0[1]) / (Ah * Ah))
    res["after"]["straight_d2_rms"] = math.sqrt(sum(x * x for x in d2) / len(d2)) if d2 else float("nan")
    res["after"]["n_straight_samples"] = len(d2)

    # dominant wobble wavelength: autocorrelation of the signed offset on the longest
    # straight segment (more robust than an FFT peak on a short, tapered window)
    if wob:
        longest = max(gb["_segments"], key=lambda g: g["len"])
        sel = [i for i in range(len(AQ)) if longest["s0"] <= prog[i] <= longest["s1"]]
        _, _, rr = line_residual([AQ[i] for i in sel], longest["A"], longest["B"])
        ss = [prog[i] for i in sel]
        if len(rr) > 64:
            grid = np.linspace(ss[0], ss[-1], len(rr))
            yy = np.interp(grid, ss, rr)
            yy = yy - yy.mean()
            dy = float(grid[1] - grid[0])
            ac = np.correlate(yy, yy, mode="full")[len(yy) - 1:]
            ac = ac / (ac[0] + 1e-12)
            # first local maximum after the zero crossing = dominant period
            best_lag, best_val = 0, -1.0
            for lag in range(int(10.0 / dy), min(len(ac) - 1, int(400.0 / dy))):
                if ac[lag] > ac[lag - 1] and ac[lag] >= ac[lag + 1] and ac[lag] > best_val:
                    best_val, best_lag = ac[lag], lag
            res["after"]["wobble_wavelength_m"] = best_lag * dy if best_lag else float("nan")
            res["after"]["wobble_ac_peak"] = float(best_val)
            f = np.fft.rfftfreq(len(yy), d=dy)
            amp = np.abs(np.fft.rfft(yy))
            if len(amp) > 2:
                kk = int(np.argmax(amp[1:]) + 1)
                res["after"]["wobble_fft_wavelength_m"] = (1.0 / f[kk]) if f[kk] > 0 else float("nan")

    # after-side repeat detection + separation between the two passes
    a_corrs, atot, ABQ, ABh, abn = repeat_corridors(after)
    res["after"]["repeat_corridors"] = len(a_corrs)
    res["after"]["repeat_total_m"] = sum(c["lenA"] for c in a_corrs)
    res["after"]["repeat_ratio"] = sum(c["lenA"] for c in a_corrs) / atot
    seps = []
    for c in corrs:                       # BEFORE corridors are the ground truth
        a0, a1 = scale_win(c["sA0"], c["sA1"], btot, len(AQ))
        b0, b1 = scale_win(c["sB0"], c["sB1"], btot, len(AQ))
        sp = pass_sep(AQ, {"i0": a0, "i1": a1, "j0": b0, "j1": b1})
        seps.append({"lenA": c["lenA"], "min": min(sp) if sp else float("nan"),
                     "p50": pct(sp, .5) if sp else float("nan"),
                     "max": max(sp) if sp else float("nan"), "n": len(sp)})
    res["after"]["corridor_sep"] = seps
    res["after"]["corridor_sep_min"] = min([x["min"] for x in seps], default=float("nan"))
    res["after"]["corridor_sep_p50"] = pct([x["p50"] for x in seps], .5) if seps else float("nan")
    res["after"]["corridor_sep_p05"] = pct([x["p50"] for x in seps], .05) if seps else float("nan")

    # U-turn handling
    ut_out = []
    Qc, hc, tc = resample(after, 1.0)
    kc = menger_curvature(Qc, hc)
    for u in uts:
        lo, hi = u["s"] - 30.0, u["s"] + 30.0
        ks = [kc[i] for i in range(len(kc)) if lo <= (i + 1) * hc <= hi]
        kmax = max(ks) if ks else 0.0
        dks = [abs(ks[i + 1] - ks[i]) for i in range(len(ks) - 1)]
        # lateral displacement before vs after the turn
        seg = {"s": u["s"], "turn": u["turn"], "curv_max": kmax,
               "minR_m": (1.0 / kmax) if kmax > 0 else float("inf"),
               "dk_max": max(dks) if dks else 0.0}
        try:
            # deviation at +/- 15 m around the apex is a fair "how far did it swing" proxy
            dA = [dists[i] for i in range(len(prog)) if abs(prog[i] - (u["s"] - 15)) < 1.0]
            dB = [dists[i] for i in range(len(prog)) if abs(prog[i] - (u["s"] + 15)) < 1.0]
            seg["dev_-15m"] = dA[0] if dA else float("nan")
            seg["dev_+15m"] = dB[0] if dB else float("nan")
            seg["dev_local_max"] = max([dists[i] for i in range(len(prog))
                                        if abs(prog[i] - u["s"]) <= 25.0], default=float("nan"))
        except Exception:
            pass
        ut_out.append(seg)
    res["after"]["u_turn_detail"] = ut_out

    # self-intersections of the after polyline (sampled at 1 m)
    hits, _ = self_intersections(Qc, min_arc=5.0, h=hc)
    res["after"]["self_intersections"] = hits
    if hits:                       # locate them so a U-turn can be blamed or cleared
    # arc positions of crossings
        def _cross(p1, p2, p3, p4):
            def d(a, b, c):
                return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            return (((d(p3, p4, p1) > 0) != (d(p3, p4, p2) > 0))
                    and ((d(p1, p2, p3) > 0) != (d(p1, p2, p4) > 0)))
        locs_x = []
        if len(Qc) <= 1200:
            for i in range(len(Qc) - 1):
                for j in range(i + 4, len(Qc) - 1):
                    if min(abs(i - j), len(Qc) - abs(i - j)) * hc < 5.0:
                        continue
                    if _cross(Qc[i], Qc[i + 1], Qc[j], Qc[j + 1]):
                        locs_x.append((round(i * hc, 1), round(j * hc, 1)))
        res["after"]["self_intersection_arcs"] = locs_x[:40]
        for seg in ut_out:
            seg["self_int_near"] = sum(1 for a, b in locs_x
                                       if abs(((a + b) / 2.0) - seg["s"]) <= 40.0)

    print("\n== AFTER (humanize) ==")
    print(f"  vertices={ga['n_vertices']} length={fmt(ga['length_m'],1)}m "
          f"ratio={fmt(ga['length_m']/gb['length_m'],4)} closure={fmt(ga['closure_err_m'],3)}m")
    print(f"  spacing min/p50/max = {fmt(ga['spacing_min'],2)}/{fmt(ga['spacing_p50'],2)}/{fmt(ga['spacing_max'],2)} m")
    print(f"  turns n={ga['turn_n']} p50={fmt(ga['turn_p50'],1)} p90={fmt(ga['turn_p90'],1)} "
          f"max={fmt(ga['turn_max'],1)} >45={ga['turn_gt45']} >90={ga['turn_gt90']} >135={ga['turn_gt135']}")
    print(f"  curv p50={fmt(ga['curv_p50'],4)} p95={fmt(ga['curv_p95'],4)} max={fmt(ga['curv_max'],4)} "
          f"minR={fmt(ga['minR_m'],3)}m")
    print(f"  |dk| p50={fmt(ga['dk_p50'],4)} p95={fmt(ga['dk_p95'],4)} max={fmt(ga['dk_max'],4)}")
    print(f"  deviation p50={fmt(res['after']['dev_p50'],2)} p95={fmt(res['after']['dev_p95'],2)} "
          f"max={fmt(res['after']['dev_max'],2)} m")
    print(f"  straight wobble |off| mean={fmt(res['after']['wobble_abs_mean'],2)} "
          f"p95={fmt(res['after']['wobble_abs_p95'],2)} max={fmt(res['after']['wobble_abs_max'],2)} m  "
          f"d2rms={fmt(res['after']['straight_d2_rms'],4)} 1/m  wl={fmt(res['after'].get('wobble_wavelength_m'),1)}m")
    print(f"  corridor sep min={fmt(res['after']['corridor_sep_min'],2)} "
          f"p50={fmt(res['after']['corridor_sep_p50'],2)} m   self-int={res['after']['self_intersections']}")
    for s_c in seps:
        print(f"    corridor lenA={fmt(s_c['lenA'],0)}m sep min/p50/max = "
              f"{fmt(s_c['min'],2)}/{fmt(s_c['p50'],2)}/{fmt(s_c['max'],2)}")
    print(f"  determinism same-seed={same} other-seed-differs={diff_seed}  humanize {fmt(t_h,2)}s")

    if not args.no_motion:
        print("\n== MOTION MODEL (generator.build, ordered_path=True) ==")
        try:
            mb, _ = run_motion(before, "before", seed=args.seed)
            res["motion_before"] = mb
            print(f"  BEFORE  v mean={fmt(mb['v_mean'],3)} cv={fmt(mb['v_cv'],3)} "
                  f"p1={fmt(mb['v_p1'],2)} p99={fmt(mb['v_p99'],2)} min={fmt(mb['v_min'],2)} "
                  f"max={fmt(mb['v_max'],2)}  lowrun={mb['low_run_max']} ({mb['low_n']} pts)")
            print(f"          res_cad p50/p95={fmt(mb['res_cad_p50'],2)}/{fmt(mb['res_cad_p95'],2)} "
                  f"res_str p50/p95={fmt(mb['res_str_p50'],3)}/{fmt(mb['res_str_p95'],3)} "
                  f"elast={fmt(mb['elasticity'],3)} corr={fmt(mb['corr_v_cad_point'],3)}/"
                  f"{fmt(mb['corr_v_cad_10s'],3)}/{fmt(mb['corr_v_cad_30s'],3)}")
        except Exception as exc:
            res["motion_before"] = {"error": f"{type(exc).__name__}: {exc}"}
            print("  BEFORE motion failed:", exc)
            traceback.print_exc()
        try:
            ma, tr = run_motion(after, "after", seed=args.seed)
            res["motion_after"] = ma
            print(f"  AFTER   v mean={fmt(ma['v_mean'],3)} cv={fmt(ma['v_cv'],3)} "
                  f"p1={fmt(ma['v_p1'],2)} p99={fmt(ma['v_p99'],2)} min={fmt(ma['v_min'],2)} "
                  f"max={fmt(ma['v_max'],2)}  lowrun={ma['low_run_max']} ({ma['low_n']} pts)")
            print(f"          res_cad p50/p95={fmt(ma['res_cad_p50'],2)}/{fmt(ma['res_cad_p95'],2)} "
                  f"res_str p50/p95={fmt(ma['res_str_p50'],3)}/{fmt(ma['res_str_p95'],3)} "
                  f"elast={fmt(ma['elasticity'],3)} corr={fmt(ma['corr_v_cad_point'],3)}/"
                  f"{fmt(ma['corr_v_cad_10s'],3)}/{fmt(ma['corr_v_cad_30s'],3)}")
            print(f"          gaps={ma['gap_hist']} dominant_share={fmt(ma['gap_dominant_share'],2)} "
                  f"dis_err={fmt(ma['sim_dis_err'],2)}m")
        except Exception as exc:
            res["motion_after"] = {"error": f"{type(exc).__name__}: {exc}"}
            print("  AFTER motion failed:", exc)
            traceback.print_exc()

    with open(args.json, "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1, default=str)
    print("\nwrote", args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
