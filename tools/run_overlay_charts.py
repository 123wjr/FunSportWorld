#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Build a single offline, self-contained interactive HTML that overlays
route map + speed / cadence / stride time series on one shared time cursor.

Usage:
  python tools/run_overlay_charts.py --demo
  python tools/run_overlay_charts.py --input A.json [B.json ...] [--out OUT.html]
  python tools/run_overlay_charts.py --input A.json --out OUT.html --selftest

No network, no CDN, no PNG. All data and JS are inlined into one file.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TEMPLATE = os.path.join(HERE, "overlay_charts_template.html")
DEFAULT_OUT = os.path.join(ROOT, ".test-tmp", "charts", "overlay.html")
PALETTE = ["#1f6f8b", "#c0603a", "#3f7a3a", "#7a5aa8", "#a8862a"]
CORR_COLORS = ["#8a8f8c", "#1f6f8b", "#c0603a", "#3f7a3a", "#7a5aa8"]
APP_SENTINEL = 'JSON.parse(document.getElementById("payload").textContent)'

CAD_A, CAD_B = 171.43, 0.28
STR_A, STR_B = 1.05, 0.72


class Bad(Exception):
    pass


def _f(x, d=None):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return d
    return v if math.isfinite(v) else d


def emp_cadence(v):
    return CAD_A * (max(0.0, v) / 3.0) ** CAD_B


def emp_stride(v):
    return STR_A * (max(0.0, v) / 3.0) ** STR_B


# ---------------------------------------------------------------- validation
def norm_series(obj, name, color):
    """Validate + normalise one track dict to {name,color,meta,route,points,derived}."""
    if not isinstance(obj, dict):
        raise Bad("series entry is not an object")
    pts_in = obj.get("points")
    if not isinstance(pts_in, list) or not pts_in:
        raise Bad("%s: points must be a non-empty list" % name)
    pts = []
    last_t = -1e18
    for i, p in enumerate(pts_in):
        if not isinstance(p, dict):
            raise Bad("%s: points[%d] is not an object" % (name, i))
        t = _f(p.get("t"))
        if t is None:
            raise Bad("%s: points[%d].t missing/not a number" % (name, i))
        if t < last_t - 1e-9:
            raise Bad("%s: points[%d].t is not non-decreasing (%.3f < %.3f)" % (name, i, t, last_t))
        last_t = t
        v = max(0.0, _f(p.get("v"), 0.0))
        cad = max(0.0, _f(p.get("cadence"), 0.0))
        st = max(0.0, _f(p.get("stride_m"), 0.0))
        ec = _f(p.get("emp_cadence"))
        es = _f(p.get("emp_stride_m"))
        cr = _f(p.get("corner_radius_m"))
        q = {
            "t": round(t, 3),
            "dist_m": round(_f(p.get("dist_m"), 0.0), 2),
            "lat": _f(p.get("lat")),
            "lng": _f(p.get("lng")),
            "v": round(v, 4),
            "cadence": round(cad, 3),
            "stride_m": round(st, 4),
            "emp_cadence": round(ec, 3) if ec is not None else round(emp_cadence(v), 3),
            "emp_stride_m": round(es, 4) if es is not None else round(emp_stride(v), 4),
            "corner_radius_m": round(cr, 1) if cr is not None else None,
            "u_turn": bool(p.get("u_turn")),
        }
        el = _f(p.get("elevation_m"))
        q["elevation_m"] = round(el, 2) if el is not None else None
        if "emp_cadence" not in p:
            q["emp_cadence"] = round(emp_cadence(v), 3)
        pts.append(q)

    route_in = obj.get("route") or {}
    route = {"road": [], "track": []}
    if isinstance(route_in, dict):
        for key in ("road", "track"):
            seq = route_in.get(key)
            if not isinstance(seq, list):
                continue
            out = []
            for pair in seq:
                if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                    la, ln = _f(pair[0]), _f(pair[1])
                    if la is not None and ln is not None:
                        out.append([round(la, 7), round(ln, 7)])
            route[key] = out
    if not route["track"]:
        route["track"] = _track_from_points(pts)

    der_in = obj.get("derived") or {}
    derived = {}
    for key in ("speed_10s", "cadence_10s", "stride_10s"):
        seq = der_in.get(key) if isinstance(der_in, dict) else None
        clean = []
        if isinstance(seq, list):
            for e in seq:
                if not isinstance(e, dict):
                    continue
                et = _f(e.get("t", e.get("time")))
                ev = _f(e.get("v", e.get("value")))
                if et is None or ev is None:
                    continue
                clean.append({"t": round(et, 3), "v": round(ev, 4)})
        derived[key] = clean
    if any(len(derived[k]) < 2 for k in ("speed_10s", "cadence_10s", "stride_10s")):
        alt = _derive_rows(pts)
        for key in ("speed_10s", "cadence_10s", "stride_10s"):
            if len(derived[key]) < 2:
                derived[key] = alt[key]

    meta_in = obj.get("meta") or {}
    meta = {}
    if isinstance(meta_in, dict):
        for k, v in meta_in.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                meta[k] = v
            elif isinstance(v, str):
                meta[k] = v[:120]
    return {
        "name": str(obj.get("name") or name)[:60],
        "color": color,
        "meta": meta,
        "route": route,
        "points": pts,
        "derived": derived,
    }


def expand(doc, base_name):
    """Accept: single track | {"series":[...]} | [track, track, ...]."""
    out = []
    if isinstance(doc, list):
        for i, d in enumerate(doc):
            out.extend(expand(d, "%s#%d" % (base_name, i + 1)))
        return out
    if isinstance(doc, dict) and isinstance(doc.get("series"), list):
        for i, d in enumerate(doc["series"]):
            out.extend(expand(d, "%s#%d" % (base_name, i + 1)))
        return out
    out.append(norm_series(doc, base_name, PALETTE[0]))
    return out


# ------------------------------------------------------------------- demo
def _demo_path():
    """Closed-ish demo route: straights + 90 deg corners + two hairpins.

    Returns list of (x_m, y_m, radius_m_or_None, is_uturn) vertices, spaced ~2 m.
    """
    V = []
    st = {"x": 0.0, "y": 0.0, "hd": 0.0}

    def push(x, y, rad, ut):
        V.append((x, y, rad, ut))

    def straight(length, step=2.0):
        n = max(1, int(round(length / step)))
        for _ in range(n):
            st["x"] += math.cos(st["hd"]) * length / n
            st["y"] += math.sin(st["hd"]) * length / n
            push(st["x"], st["y"], None, False)

    def turn(radius, sign_deg, ut=False):
        deg = abs(sign_deg)
        sign = 1.0 if sign_deg > 0 else -1.0
        n = max(3, int(round(deg / 1.5)))
        dtheta = math.radians(deg) / n
        chord = 2.0 * radius * math.sin(dtheta / 2.0)
        for _ in range(n):
            mid = st["hd"] + sign * dtheta / 2.0
            st["x"] += math.cos(mid) * chord
            st["y"] += math.sin(mid) * chord
            st["hd"] += sign * dtheta
            push(st["x"], st["y"], radius if radius < 60 else None, ut)

    push(0.0, 0.0, None, False)
    straight(360.0)
    turn(9.0, 90.0)
    straight(220.0)
    turn(3.5, 180.0, True)   # hairpin / u-turn
    straight(240.0)
    turn(11.0, -90.0)
    straight(300.0)
    turn(9.0, 90.0)
    straight(200.0)
    turn(3.2, -180.0, True)  # second hairpin
    straight(260.0)
    turn(12.0, 90.0)
    straight(280.0)
    turn(9.0, 90.0)
    straight(150.0)
    turn(14.0, -90.0)
    straight(120.0)
    return V


def _path_cum(V):
    cum = [0.0]
    for i in range(1, len(V)):
        cum.append(cum[-1] + math.hypot(V[i][0] - V[i - 1][0], V[i][1] - V[i - 1][1]))
    return cum


def _sample_path(V, cum, s):
    """Return (x, y, heading, radius, is_uturn) at arclength s (clamped)."""
    s = max(0.0, min(cum[-1] - 1e-6, s))
    lo, hi = 0, len(cum) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    x0, y0 = V[lo][0], V[lo][1]
    x1, y1 = V[hi][0], V[hi][1]
    seg = cum[hi] - cum[lo]
    f = (s - cum[lo]) / seg if seg > 1e-12 else 0.0
    hd = math.atan2(y1 - y0, x1 - x0) if seg > 1e-12 else 0.0
    rad = V[hi][2]
    ut = V[hi][3]
    return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, hd, rad, ut


def _path_len(V):
    return _path_cum(V)[-1]


def _scale_path(V, k):
    return [(x * k, y * k, r, u) for (x, y, r, u) in V]


def build_demo(seed=7, dur_s=1500.0, dt=2.0, lat0=30.2741, lng0=120.1551):
    V = _demo_path()
    total = _path_len(V)
    # make sure the route is long enough for the whole run without wrapping twice
    need = dur_s * 3.6
    if total < need:
        V = _scale_path(V, need / total)
        total = _path_len(V)
    cum = _path_cum(V)
    n = int(dur_s / dt) + 1
    phase = seed * 0.37
    ar = 0.0
    dist = 0.0
    rows = []
    for i in range(n):
        t = i * dt
        x, y, hd, rad, ut = _sample_path(V, cum, dist)
        tight = 0.0 if rad is None else max(0.0, min(1.0, (14.0 - rad) / 11.0))
        ramp = min(1.0, 0.30 + i / 20.0)
        base = 2.90 + 0.32 * math.sin(t / 47.0 + phase) + 0.14 * math.sin(t / 11.3 + phase * 2.1)
        ar = 0.80 * ar + 0.55 * math.sin(t / 6.7 + phase * 3.3) + 0.22 * math.cos(t / 3.1 + phase)
        v = max(0.30, (base + 0.16 * ar) * ramp * (1.0 - 0.72 * tight))
        if i == 0:
            v = 0.0
        dist += v * dt
        if dist >= cum[-1] - 1.0:
            dist = 1.0  # loop the demo route
        cad = 0.0 if v < 0.05 else emp_cadence(v) * (1.0 + 0.035 * math.sin(t / 8.3 + phase) + 0.015 * math.cos(t / 2.7))
        cad = max(0.0, cad)
        stride = (v * 60.0 / cad) if cad > 1.0 else 0.0
        rows.append({
            "t": round(t, 2),
            "dist_m": round(dist, 2),
            "lat": round(lat0 + y / 111320.0, 7),
            "lng": round(lng0 + x / (111320.0 * math.cos(math.radians(lat0))), 7),
            "v": round(v, 4),
            "cadence": round(cad, 2),
            "stride_m": round(stride, 4),
            "emp_cadence": round(emp_cadence(v), 2),
            "emp_stride_m": round(emp_stride(v), 4),
            "corner_radius_m": round(rad, 1) if rad is not None else None,
            "u_turn": bool(ut),
        })
    road = []
    for i in range(0, len(V), max(1, len(V) // 240)):
        x, y = V[i][0], V[i][1]
        road.append([round(lat0 + (y + 8.0) / 111320.0, 7),
                     round(lng0 + x / (111320.0 * math.cos(math.radians(lat0))), 7)])
    return {
        "meta": {"dist_m": round(rows[-1]["dist_m"], 1), "dur_s": round(rows[-1]["t"], 1), "seed": seed,
                 "cadence_target": 0, "samples": len(rows), "sample_dt_s": dt,
                 "route_len_m": round(cum[-1], 1),
                 "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")},
        "route": {"road": road, "track": [[r["lat"], r["lng"]] for r in rows]},
        "points": rows,
        "derived": _derive(rows),
    }


def _track_from_points(pts):
    """Per-sample [lat,lng] with carry-forward/back-fill so len(track) == len(points)."""
    n = len(pts)
    filled = [None] * n
    last = None
    for i, p in enumerate(pts):
        if p["lat"] is not None and p["lng"] is not None:
            last = [p["lat"], p["lng"]]
        filled[i] = last
    nxt = None
    for i in range(n - 1, -1, -1):
        if filled[i] is None:
            filled[i] = nxt
        else:
            nxt = filled[i]
    if any(v is None for v in filled):
        return []
    return [list(v) for v in filled]


def _derive_rows(rows, win=10.0):
    """Centred moving average evaluated at every sample time (fallback for short tracks)."""
    out = {"speed_10s": [], "cadence_10s": [], "stride_10s": []}
    if not rows:
        return out
    keys = {"speed_10s": "v", "cadence_10s": "cadence", "stride_10s": "stride_m"}
    n = len(rows)
    half = win / 2.0
    lo = 0
    hi = 0
    for i in range(n):
        while lo < n and rows[lo]["t"] < rows[i]["t"] - half:
            lo += 1
        if hi < i + 1:
            hi = i + 1
        while hi < n and rows[hi]["t"] <= rows[i]["t"] + half:
            hi += 1
        if lo > i:
            lo = i
        for dk, pk in keys.items():
            s = sum(rows[j][pk] for j in range(lo, hi))
            out[dk].append({"t": round(rows[i]["t"], 3), "v": round(s / (hi - lo), 4)})
    return out


def _derive(rows, win=10.0):
    out = {"speed_10s": [], "cadence_10s": [], "stride_10s": []}
    if not rows:
        return out
    keys = {"speed_10s": "v", "cadence_10s": "cadence", "stride_10s": "stride_m"}
    t_end = rows[-1]["t"]
    t = 0.0
    lo = 0
    while t <= t_end + 1e-9:
        while lo < len(rows) and rows[lo]["t"] < t - win:
            lo += 1
        hi = lo
        while hi < len(rows) and rows[hi]["t"] <= t:
            hi += 1
        if hi > lo:
            for dk, pk in keys.items():
                s = sum(rows[i][pk] for i in range(lo, hi))
                out[dk].append({"t": round(t, 1), "v": round(s / (hi - lo), 4)})
        t += win
    return out


# ------------------------------------------------------------------- build
# ------------------------------------------------------------------ extras
# Three extra views (task-14): elevation profile, multi-lap corridor zoom, wander A/B.
# Everything is computed here at build time and inlined, so the HTML stays offline
# and the browser never needs numpy / funsport / network.

def _merc_xy(pts, lat0):
    """[(lat, lng), ...] -> local equirectangular metres, origin at pts[0]."""
    kx = 111320.0 * math.cos(math.radians(lat0))
    ky = 111132.0
    la0, ln0 = pts[0]
    return [((ln - ln0) * kx, (la - la0) * ky) for la, ln in pts]


def _arcs(xy):
    out = [0.0]
    for i in range(1, len(xy)):
        out.append(out[-1] + math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]))
    return out


def _project(q, xy, arc):
    """Nearest point on the polyline: (arc_m, signed lateral offset_m, distance_m).

    Sign follows the cross product of the local tangent and the offset vector, so
    + is one consistent side of the ring for the whole track.
    """
    best_d, best_s, best_o = 1e18, 0.0, 0.0
    for i in range(len(xy) - 1):
        ax, ay = xy[i]
        dx = xy[i + 1][0] - ax
        dy = xy[i + 1][1] - ay
        seg = math.hypot(dx, dy)
        if seg <= 1e-9:
            continue
        ux, uy = dx / seg, dy / seg
        t = (q[0] - ax) * ux + (q[1] - ay) * uy
        t = 0.0 if t < 0.0 else (seg if t > seg else t)
        px, py = ax + ux * t, ay + uy * t
        d = math.hypot(q[0] - px, q[1] - py)
        if d < best_d:
            best_d, best_s = d, arc[i] + t
            best_o = ux * (q[1] - ay) - uy * (q[0] - ax)
    return best_s, best_o, best_d


def _resample_xy(xy, step):
    """Uniform arc-length resampling: ([(x, y), ...], total_length_m)."""
    arc = _arcs(xy)
    total = arc[-1]
    n = max(2, int(total / step) + 1)
    out = []
    j = 0
    for k in range(n):
        s = min(total, k * step)
        while j + 2 < len(arc) and arc[j + 1] < s:
            j += 1
        span = arc[j + 1] - arc[j]
        f = 0.0 if span <= 1e-12 else (s - arc[j]) / span
        ax, ay = xy[j]
        bx, by = xy[j + 1]
        out.append((ax + (bx - ax) * f, ay + (by - ay) * f))
    return out, total


def _offset_vs_ring(qs, rp, step, period, win=15.0):
    """Signed lateral offset of every path point against a resampled ring.

    The campus loop doubles back on itself, so a global nearest-point projection is
    ambiguous within ~1e-14 of a tie and jumps between the two parallel legs (that
    is what produced 331 m arc jumps and 2.8 m phantom amplitudes). Instead the
    path's OWN cumulative arc is the prior: only ring points within +-win metres of
    that arc position are considered, which is monotone and local.
    Returns [(ring_arc_m, signed_offset_m, distance_m), ...].
    """
    m = len(rp) - 1                      # rp[-1] duplicates rp[0]
    if m < 2 or period <= 0.0:
        return []
    cum = _arcs(qs)
    kwin = max(1, int(win / step))
    out = []
    for (qx, qy), u in zip(qs, cum):
        j0 = int(round((u % period) / step))
        best_d, best_o, best_s = 1e18, 0.0, u % period
        for k in range(-kwin, kwin + 1):
            j = (j0 + k) % m
            ax, ay = rp[j]
            bx, by = rp[(j + 1) % m]
            dx = bx - ax
            dy = by - ay
            seg = math.hypot(dx, dy)
            if seg <= 1e-12:
                continue
            ux, uy = dx / seg, dy / seg
            t = (qx - ax) * ux + (qy - ay) * uy
            t = 0.0 if t < 0.0 else (seg if t > seg else t)
            px, py = ax + ux * t, ay + uy * t
            d = math.hypot(qx - px, qy - py)
            if d < best_d:
                best_d, best_s = d, j * step + t
                best_o = ux * (qy - ay) - uy * (qx - ax)
        out.append((best_s, best_o, best_d))
    return out


def _highpass(grid, y, half_m):
    """Remove the slow trend: y minus its centred moving average over +-half_m metres."""
    if len(grid) < 3:
        return list(y)
    step = grid[1] - grid[0]
    k = max(1, int(round(half_m / step)))
    pre = [0.0]
    for v in y:
        pre.append(pre[-1] + v)
    n = len(y)
    out = []
    for i in range(n):
        lo = max(0, i - k)
        hi = min(n - 1, i + k)
        out.append(y[i] - (pre[hi + 1] - pre[lo]) / float(hi - lo + 1))
    return out


def _interp_periodic(grid, s, y, period):
    """Sample y(s) onto grid; s/y are one lap (closes back to s[0] + period)."""
    idx = sorted(range(len(s)), key=lambda i: s[i])
    ss = [s[i] for i in idx]
    yy = [y[i] for i in idx]
    ss.append(ss[0] + period)
    yy.append(yy[0])
    out = []
    j = 0
    n = len(ss)
    for g in grid:
        while j + 1 < n and ss[j + 1] < g:
            j += 1
        if j + 1 >= n:
            out.append(yy[-1])
            continue
        span = ss[j + 1] - ss[j]
        f = 0.0 if span <= 1e-12 else (g - ss[j]) / span
        out.append(yy[j] * (1.0 - f) + yy[j + 1] * f)
    return out


def _band_amp(grid, y, period, cycles):
    """Two-sided Fourier amplitude (m) of the integer-cycle component."""
    n = len(grid)
    if n < 8:
        return 0.0
    re = im = 0.0
    for i in range(n):
        ph = 2.0 * math.pi * cycles * grid[i] / period
        re += y[i] * math.cos(ph)
        im -= y[i] * math.sin(ph)
    return 2.0 * math.hypot(re, im) / n


def _thin(xs, ys, cap=520):
    """Stride-sample a polyline so the inline payload stays small."""
    n = len(xs)
    if n <= cap:
        return xs, ys
    step = (n - 1) / float(cap - 1)
    out_x, out_y = [], []
    for k in range(cap):
        i = min(n - 1, int(round(k * step)))
        out_x.append(xs[i])
        out_y.append(ys[i])
    return out_x, out_y


def _extras_elev(tracks):
    """Distance-elevation profile per series, plus cumulative gain / span."""
    rows = []
    for tr in tracks:
        pts = tr["points"]
        el = [p.get("elevation_m") for p in pts]
        if any(e is None for e in el):
            continue
        ds = [float(p.get("dist_m") or 0.0) for p in pts]
        gain = loss = 0.0
        for i in range(1, len(el)):
            d = el[i] - el[i - 1]
            if d > 0:
                gain += d
            else:
                loss -= d
        xs, ys = _thin(ds, el)
        rows.append({
            "name": tr["name"], "color": tr["color"],
            "s": [round(v, 1) for v in xs], "e": [round(v, 2) for v in ys],
            "gain_m": round(gain, 2), "loss_m": round(loss, 2),
            "lo_m": round(min(el), 2), "hi_m": round(max(el), 2),
            "src": str(tr["meta"].get("elevationSource") or "未标注"),
        })
    if not rows:
        return None
    return {"series": rows, "src": rows[0]["src"]}


def _arc_of(xy):
    out = [0.0]
    for i in range(1, len(xy)):
        out.append(out[-1] + math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]))
    return out


def _lap_lengths(doc):
    """Per-lap lengths for one track: meta.lap_lengths_m, else re-run humanize.

    The loop runner writes lap_lengths_m from humanize_laps; older artifacts do
    not carry it, and guessing the boundaries is exactly what made an earlier
    version of this panel report that laps 2 and 3 sat 0.02 m apart. Without the
    real lengths the ring can be re-humanised with the same seed and lap count
    (the wander panel relies on the same reproducibility).
    """
    meta = doc.get("meta") or {}
    laps = int(meta.get("laps") or 1)
    if laps < 2:
        return None
    lpl = meta.get("lap_lengths_m")
    if isinstance(lpl, (list, tuple)) and len(lpl) == laps and all(float(v) > 0 for v in lpl):
        return [float(v) for v in lpl]
    road = (doc.get("route") or {}).get("road")
    if not road or len(road) < 8:
        return None
    try:
        if ROOT not in sys.path:
            sys.path.insert(0, ROOT)
        from funsport.coordinates import gcj02_to_bd09
        from funsport.track import humanize as hz
        _bd, info = hz.humanize_laps([gcj02_to_bd09(la, ln) for la, ln in road],
                                     seed=int(meta.get("seed") or 0), laps=laps)
    except Exception:  # noqa: BLE001 - funsport is optional for this chart tool
        return None
    got = (info or {}).get("lap_lengths_m")
    if isinstance(got, (list, tuple)) and len(got) == laps and all(float(v) > 0 for v in got):
        return [float(v) for v in got]
    return None


def _lap_bounds(xy, arc, laps, lengths=None):
    """Lap boundaries expressed as arc positions along one track."""
    total = arc[-1]
    if laps < 2 or total <= 0:
        return [0.0, total]
    if lengths:
        cum, acc = [0.0], 0.0
        for v in lengths:
            acc += v
            cum.append(acc)
        return [total * v / acc for v in cum]
    out = [0.0]
    for j in range(1, laps):
        naive = total * j / laps
        best = None
        for qq, aa in zip(xy, arc):
            if abs(aa - naive) > 45.0:
                continue
            d0 = math.hypot(qq[0] - xy[0][0], qq[1] - xy[0][1])
            if best is None or d0 < best[0]:
                best = (d0, aa)
        out.append(best[1] if best else naive)
    out.append(total)
    return out


def _extras_corr(tracks, win_m=200.0):
    """Corridor view: laps of ONE run, each matched to that run's lap 1 by arc.

    Matching is by normalised arc position, never by nearest point: this ring
    doubles back on itself, so a nearest-point search snaps onto the parallel
    leg and reports ~0.9 m (or ~0.02 m) where the laps really sit metres apart.
    That bug, plus using the 5.6 m-sampled points[] instead of the 1.5 m
    route.track, produced the earlier 0.87/0.89/0.02 m numbers.

    Laps of the same run carry the lane pattern (0, +1, -1); mixing lap 2 of
    different runs instead puts those laps on the same side and makes them look
    coincident.
    """
    src = None
    for tr in tracks:
        n_l = int((tr.get("meta") or {}).get("laps") or 1)
        if n_l > 1 and (src is None or n_l > int((src.get("meta") or {}).get("laps") or 1)):
            src = tr
    if src is None:
        return None
    meta = src.get("meta") or {}
    nlaps = int(meta.get("laps") or 1)
    road_pts = [p for p in ((src.get("route") or {}).get("track") or [])
                if isinstance(p, (list, tuple)) and len(p) >= 2]
    use_track = len(road_pts) >= max(16, 3 * nlaps)
    if use_track:
        geo = [(float(p[0]), float(p[1])) for p in road_pts]
    else:
        gp = [p for p in src["points"] if p.get("lat") is not None and p.get("lng") is not None]
        geo = [(float(p["lat"]), float(p["lng"])) for p in gp]
    if nlaps < 2 or len(geo) < 16:
        return None
    lat0 = geo[0][0]
    xy = _merc_xy(geo, lat0)
    arc = _arc_of(xy)
    if arc[-1] <= win_m * 1.5:
        return None
    lengths = _lap_lengths(src)
    bnd = _lap_bounds(xy, arc, nlaps, lengths)
    if len(bnd) != nlaps + 1:
        return None

    def lap_slice(k):
        return [(q, a) for q, a in zip(xy, arc) if bnd[k] <= a <= bnd[k + 1]]

    ref = lap_slice(0)
    if len(ref) < 8:
        return None
    rc = _arc_of([q for q, _a in ref])
    L1 = rc[-1]
    if L1 <= 0:
        return None

    def at(u):
        u = min(max(u, 0.0), L1)
        j = 0
        while j + 2 < len(rc) and rc[j + 1] < u:
            j += 1
        span = rc[j + 1] - rc[j]
        f = 0.0 if span <= 1e-9 else (u - rc[j]) / span
        ax, ay = ref[j][0]
        bx, by = ref[j + 1][0]
        sl = math.hypot(bx - ax, by - ay) or 1e-9
        return (ax + (bx - ax) * f, ay + (by - ay) * f), ((bx - ax) / sl, (by - ay) / sl)

    rows = []
    lap_stats = []
    for k in range(nlaps):
        seg = lap_slice(k)
        if len(seg) < 8:
            continue
        sac = _arc_of([q for q, _a in seg])
        span = sac[-1] or 1.0
        rw = []
        dists = []
        for i, (q, _a) in enumerate(seg):
            base, tan = at(sac[i] / span * L1)
            dx, dy = q[0] - base[0], q[1] - base[1]
            rw.append((sac[i], tan[0] * dy - tan[1] * dx))
            dists.append(math.hypot(dx, dy))
        rows.append(rw)
        d = sorted(dists)
        n = len(d)
        lap_stats.append({"lap": k + 1,
                          "len_m": round(bnd[k + 1] - bnd[k], 1),
                          "p50": round(d[n // 2], 2),
                          "p95": round(d[int(0.95 * (n - 1))], 2),
                          "max": round(d[-1], 2)})
    if len(rows) < 2:
        return None

    win = min(win_m, L1 / 3.0)
    best = None
    s0 = 0.0
    step = max(10.0, win / 20.0)
    while s0 + win <= L1:
        meds, spreads, ok = [], [], True
        for rr in rows:
            o = sorted(off for u, off in rr if s0 <= u <= s0 + win)
            if len(o) < 5:
                ok = False
                break
            meds.append(o[len(o) // 2])
            spreads.append(o[int(len(o) * 0.9)] - o[int(len(o) * 0.1)])
        if ok:
            seps = [abs(meds[i] - meds[j]) for i in range(len(meds)) for j in range(i + 1, len(meds))]
            score = min(seps) - 0.15 * max(spreads)     # every pair must look separate
            if best is None or score > best[0]:
                best = (score, s0, meds, spreads, seps)
        s0 += step
    if best is None:
        return None
    _, s0, meds, spreads, seps = best

    ser = []
    for idx2, rr in enumerate(rows):
        xs, ys = [], []
        for u, off in rr:
            if s0 <= u <= s0 + win:
                xs.append(round(u - s0, 1))
                ys.append(round(off, 2))
        if len(xs) < 2:
            continue
        if len(xs) > 260:
            keep = int(math.ceil(len(xs) / 260.0))
            xs, ys = xs[::keep], ys[::keep]
        ser.append({"name": "第%d圈" % (idx2 + 1),
                    "color": CORR_COLORS[idx2 % len(CORR_COLORS)],
                    "s": xs, "off": ys})
    if len(ser) < 2:
        return None
    return {
        "s0": round(s0, 1), "win": round(win, 1), "series": ser,
        "track": src.get("name") or "",
        "median_off": [round(v, 2) for v in meds],
        "spread_p10_p90": [round(v, 2) for v in spreads],
        "min_sep": round(min(seps), 2),
        "pair_sep": [round(v, 2) for v in seps],
        "lap_p50": [r["p50"] for r in lap_stats],
        "lap_p95": [r["p95"] for r in lap_stats],
        "lap_stats": lap_stats,
        "geom": "route.track" if use_track else "points",
        "lap_lengths": [round(v, 2) for v in lengths] if lengths else None,
        "design_m": round(float(meta.get("lap_lane_m") or 2.4), 2),
        "align": "arc_norm",
    }


def _extras_wander(road, seed=7, grid_n=None, hp_m=60.0):
    """Re-run the humanised path twice (new vs old wander defaults) and diff it.

    Both runs share the same seed, the same ring and the same first three random
    phases, so this is a controlled A/B of the added short wave only.  The lateral
    offset is measured against the ring with the arc prior, then high-passed
    (minus a +-hp_m moving average) so that 10-100 m ripple is not buried under the
    corner-cutting excursions.
    """
    if not road or len(road) < 8:
        return None
    try:
        if ROOT not in sys.path:
            sys.path.insert(0, ROOT)
        from funsport.coordinates import gcj02_to_bd09, bd09_to_gcj02
        from funsport.track import humanize as hz
    except Exception:  # noqa: BLE001 - funsport is optional for this chart tool
        return None
    defaults = getattr(hz, "DEFAULTS", None)
    if isinstance(defaults, dict) and defaults.get("wander_waves"):
        waves = tuple(defaults["wander_waves"])
        amps = tuple(defaults.get("wander_amp_m") or [])
    else:
        waves, amps = (8, 16, 30, 55), (1.20, 0.80, 0.52, 0.26)
    if len(waves) < 4 or len(amps) < 4:
        return None
    new_opt = {"wander_waves": waves, "wander_amp_m": amps}
    old_opt = {"wander_waves": waves[:-1], "wander_amp_m": amps[:-1]}

    lat0 = sum(p[0] for p in road) / len(road)
    rxy = _merc_xy(road + [road[0]], lat0)
    rp, period = _resample_xy(rxy, 0.5)
    if period < 100.0:
        return None

    def offsets(opts):
        try:
            bd, _info = hz.humanize_laps([gcj02_to_bd09(la, ln) for la, ln in road],
                                         seed=int(seed), laps=1, **opts)
        except Exception:  # noqa: BLE001
            return None
        pts = [bd09_to_gcj02(la, ln) for la, ln in list(bd)]
        qxy = _merc_xy(pts, lat0)
        rows = _offset_vs_ring(qxy, rp, 0.5, period)
        return [r[0] for r in rows], [r[1] for r in rows]

    got_new = offsets(new_opt)
    got_old = offsets(old_opt)
    if not got_new or not got_old:
        return None
    n = int(grid_n or max(120, min(760, int(period / 2.0) + 1)))
    grid = [period * i / float(n - 1) for i in range(n)]
    y_new = _interp_periodic(grid, got_new[0], got_new[1], period)
    y_old = _interp_periodic(grid, got_old[0], got_old[1], period)
    r_new = _highpass(grid, y_new, hp_m)
    r_old = _highpass(grid, y_old, hp_m)
    delta = [a - b for a, b in zip(r_new, r_old)]
    d_abs = sorted(abs(v) for v in delta)
    rms = math.sqrt(sum(v * v for v in delta) / len(delta))
    cyc = list(waves)
    return {
        "ring_m": round(period, 1),
        "grid": [round(v, 1) for v in grid],
        "new": [round(v, 2) for v in r_new],
        "old": [round(v, 2) for v in r_old],
        "cycles": cyc,
        "amp_new": [round(_band_amp(grid, r_new, period, c), 3) for c in cyc],
        "amp_old": [round(_band_amp(grid, r_old, period, c), 3) for c in cyc],
        "short_lam_m": round(period / float(cyc[-1]), 1),
        "hp_m": hp_m,
        "mean_new": round(sum(y_new) / len(y_new), 3),
        "mean_old": round(sum(y_old) / len(y_old), 3),
        "d_rms": round(rms, 3),
        "d_p95": round(d_abs[int(0.95 * (len(d_abs) - 1))], 3),
        "d_max": round(d_abs[-1], 3),
        "seed": int(seed),
    }


def build_extras(tracks, with_wander=True):
    ex = {}
    try:
        ex["elev"] = _extras_elev(tracks)
    except Exception as exc:  # noqa: BLE001
        print("extras: elevation skipped (%r)" % (exc,), file=sys.stderr)
    try:
        ex["corr"] = _extras_corr(tracks)
    except Exception as exc:  # noqa: BLE001
        print("extras: corridor skipped (%r)" % (exc,), file=sys.stderr)
    if with_wander:
        try:
            seed = int((tracks[0].get("meta") or {}).get("seed") or 7)
            ex["wander"] = _extras_wander((tracks[0].get("route") or {}).get("road") or [], seed=seed)
        except Exception as exc:  # noqa: BLE001
            print("extras: wander skipped (%r)" % (exc,), file=sys.stderr)
    return {k: v for k, v in ex.items() if v}

def build_html(tracks, title, extras=None):
    with open(TEMPLATE, "r", encoding="utf-8") as fh:
        tpl = fh.read()
    for ph in ("__TITLE__", "__PAYLOAD__"):
        if ph not in tpl:
            raise SystemExit("template %s is missing placeholder %s" % (TEMPLATE, ph))
    if APP_SENTINEL not in tpl:
        raise SystemExit("template %s is missing the inlined app script (sentinel %r)"
                         % (TEMPLATE, APP_SENTINEL))
    payload = {
        "schema": "funsport.overlay/1",
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "title": title,
        "series": tracks,
        "extras": extras or {},
    }
    raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    # never let a scheme prefix survive inline (offline guarantee)
    for bad in ("http://", "https://"):
        raw = raw.replace(bad, "")
    if "</script" in raw.lower():
        raw = raw.replace("</script", "<\\/script")
    html = tpl.replace("__TITLE__", title).replace("__PAYLOAD__", raw)
    if "__PAYLOAD__" in html or "__TITLE__" in html:
        raise SystemExit("template placeholders were not substituted")
    return html, len(raw)


# ------------------------------------------------------------------ selftest
def _bbox(seq):
    """(lat0, lng0, lat1, lng1) for [[lat, lng], ...]; None when nothing usable."""
    lo_lat = lo_lng = float("inf")
    hi_lat = hi_lng = float("-inf")
    n = 0
    for p in seq:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            continue
        try:
            lat = float(p[0])
            lng = float(p[1])
        except (TypeError, ValueError):
            continue
        if abs(lat) > 90.0 or abs(lng) > 180.0:
            continue
        lo_lat = min(lo_lat, lat); hi_lat = max(hi_lat, lat)
        lo_lng = min(lo_lng, lng); hi_lng = max(hi_lng, lng)
        n += 1
    return (lo_lat, lo_lng, hi_lat, hi_lng) if n else None


def _span(box):
    return max(box[2] - box[0], box[3] - box[1])


def _bbox_ok(a, b, pad_frac=0.10):
    """True when the two boxes overlap once padded by pad_frac of the larger span."""
    if a is None or b is None:
        return False
    pad = pad_frac * max(_span(a), _span(b), 1e-9)
    return not (a[2] + pad < b[0] or b[2] + pad < a[0]
                or a[3] + pad < b[1] or b[3] + pad < a[1])


BAD_PAT = ("http://", "https://", "//cdn", "cdnjs", "unpkg", "jsdelivr", "googleapis",
           "cdn.", "<link", "src='http", 'src="http')


def selftest(path, min_bytes=30 * 1024):
    fails = []
    if not os.path.exists(path):
        return ["output missing: %s" % path], {}
    size = os.path.getsize(path)
    with open(path, "r", encoding="utf-8") as fh:
        html = fh.read()
    info = {"size": size, "chars": len(html)}
    for pat in BAD_PAT:
        if pat in html:
            i = html.find(pat)
            fails.append("external-ref pattern %r at char %d: %s" % (pat, i, html[max(0, i - 40):i + 40].replace("\n", " ")))
    if size <= min_bytes:
        fails.append("file is %d bytes, expected > %d" % (size, min_bytes))
    for ph in ("__PAYLOAD__", "__TITLE__"):
        if ph in html:
            fails.append("unfilled placeholder %s" % ph)
    m0 = html.find('<script id="payload" type="application/json">')
    m1 = html.find("</script>", m0)
    if m0 < 0 or m1 < 0:
        fails.append("payload script block not found")
        return fails, info
    blob = html[m0 + len('<script id="payload" type="application/json">'):m1]
    info["payload_bytes"] = len(blob)
    try:
        doc = json.loads(blob)
    except Exception as exc:  # noqa: BLE001
        fails.append("payload JSON does not parse: %r" % (exc,))
        return fails, info
    series = doc.get("series") or []
    info["series_count"] = len(series)
    if not series:
        fails.append("payload has no series")
    for si, s in enumerate(series):
        pts = s.get("points") or []
        if len(pts) < 2:
            fails.append("series[%d] has %d points" % (si, len(pts)))
            continue
        need = ("t", "dist_m", "v", "cadence", "stride_m", "emp_cadence", "emp_stride_m", "u_turn")
        for f in need:
            if f not in pts[0]:
                fails.append("series[%d].points[0] missing field %s" % (si, f))
        # route.track is the whole humanised polyline, points are 2 s time samples, so the
        # lengths legitimately differ: only require >= 2 vertices whose bbox is compatible.
        trk = (s.get("route") or {}).get("track") or []
        info["route_%d" % si] = len(trk)
        bb_t = _bbox(trk)
        bb_p = _bbox([[p.get("lat"), p.get("lng")] for p in pts
                      if p.get("lat") is not None and p.get("lng") is not None])
        if len(trk) < 2:
            fails.append("series[%d] route.track has %d point(s), need >= 2" % (si, len(trk)))
        elif bb_t is None:
            fails.append("series[%d] route.track has no usable lat/lng" % si)
        elif bb_p is None:
            info["bbox_%d" % si] = "skipped"
        elif _span(bb_t) < 1e-9 or _span(bb_p) < 1e-9:
            info["bbox_%d" % si] = "degenerate"
        elif not _bbox_ok(bb_t, bb_p):
            info["bbox_%d" % si] = "disjoint"
            fails.append("series[%d] route.track bbox %s does not overlap points bbox %s"
                         % (si, tuple(round(v, 6) for v in bb_t), tuple(round(v, 6) for v in bb_p)))
        else:
            info["bbox_%d" % si] = "ok"
        ts = [p["t"] for p in pts]
        if any(ts[i] < ts[i - 1] for i in range(1, len(ts))):
            fails.append("series[%d] t not non-decreasing" % si)
        dv = s.get("derived") or {}
        for k in ("speed_10s", "cadence_10s", "stride_10s"):
            arr = dv.get(k)
            if not isinstance(arr, list) or len(arr) < 2:
                fails.append("series[%d].derived.%s missing or too short" % (si, k))
                continue
            if any(("t" not in e) or ("v" not in e) for e in arr):
                fails.append("series[%d].derived.%s entries need t/v" % (si, k))
        info["points_%d" % si] = len(pts)
    ex = doc.get("extras") or {}
    info["extras"] = ",".join(sorted(ex)) or "none"
    for i, row in enumerate((ex.get("elev") or {}).get("series") or []):
        if len(row.get("s") or []) != len(row.get("e") or []):
            fails.append("extras.elev.series[%d] s/e length mismatch" % i)
    co = ex.get("corr") or {}
    for i, row in enumerate(co.get("series") or []):
        if len(row.get("s") or []) != len(row.get("off") or []):
            fails.append("extras.corr.series[%d] s/off length mismatch" % i)
    if co:
        if len(co.get("series") or []) < 2:
            fails.append("extras.corr has fewer than 2 lap series")
        lps = co.get("lap_p50") or []
        if len(lps) != len(co.get("lap_p95") or []):
            fails.append("extras.corr lap_p50/lap_p95 length mismatch")
        # the earlier bug reported 0.02-0.9 m; laps of one run sit metres apart
        for i, v in enumerate(lps[1:], start=2):
            if not (1.0 <= float(v) <= 30.0):
                fails.append("extras.corr lap %d p50 %.2f m outside 1-30 m" % (i, float(v)))
        info["lap_p50"] = "/".join("%.2f" % v for v in lps[1:]) or "-"
        info["corr_geom"] = co.get("geom")
    wa = ex.get("wander") or {}
    if wa:
        n = len(wa.get("grid") or [])
        for k in ("new", "old"):
            if len(wa.get(k) or []) != n:
                fails.append("extras.wander.%s length %d != grid %d" % (k, len(wa.get(k) or []), n))
        if len(wa.get("amp_new") or []) != len(wa.get("amp_old") or []):
            fails.append("extras.wander amp arrays differ in length")
        info["wander_amp55"] = "%s -> %s" % ((wa.get("amp_old") or ["?"])[-1], (wa.get("amp_new") or ["?"])[-1])
    if (ex.get("elev") or {}).get("series"):
        info["elev_gain"] = "/".join("%.2f" % r.get("gain_m", 0.0) for r in ex["elev"]["series"])
        info["elev_src"] = ex["elev"].get("src")
    for needle in ("Web Mercator", "步频", "步长", "segMode", "segNorm", "segX", "按各自时长",
                   "海拔剖面", "多圈走廊", "摆动对比"):
        if needle not in html:
            fails.append("html missing expected marker %r" % needle)
    return fails, info


# -------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description="offline overlay charts (route + speed/cadence/stride)")
    ap.add_argument("--input", nargs="+", default=None, metavar="JSON", help="one or more track JSON files")
    ap.add_argument("--out", default=None, help="output html (default %s)" % DEFAULT_OUT)
    ap.add_argument("--title", default=None, help="page title")
    ap.add_argument("--demo", action="store_true", help="use built-in synthetic data")
    ap.add_argument("--selftest", action="store_true", help="static checks on the generated html")
    ap.add_argument("--no-extras", action="store_true",
                    help="skip the elevation / corridor / wander panels (faster, smaller)")
    ap.add_argument("--emit-sample", default=None, metavar="JSON", help="write a demo JSON sample and exit")
    args = ap.parse_args(argv)

    if args.emit_sample:
        doc = build_demo()
        doc["points"] = doc["points"][:60]
        doc["route"]["track"] = doc["route"]["track"][:60]
        doc["derived"] = _derive(doc["points"])
        pts = doc["points"]
        doc["meta"]["dur_s"] = round(pts[-1]["t"], 3)
        doc["meta"]["dist_m"] = round(pts[-1]["dist_m"], 2)
        doc["meta"]["samples"] = len(pts)
        doc["meta"]["note"] = "minimal sample: first 60 points of the built-in demo path"
        d = os.path.dirname(os.path.abspath(args.emit_sample))
        if d:
            os.makedirs(d, exist_ok=True)
        with open(args.emit_sample, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1)
            fh.write("\n")
        print("wrote sample %s (%d points)" % (args.emit_sample, len(doc["points"])))
        return 0

    if not args.demo and not args.input:
        ap.error("need --input FILE [FILE...] or --demo")

    tracks = []
    if args.demo:
        demo = build_demo()
        tracks.append(norm_series(demo, "demo", PALETTE[0]))
        tracks[0]["name"] = demo_name()
    else:
        for i, p in enumerate(args.input):
            if not os.path.exists(p):
                print("input not found: %s" % p, file=sys.stderr)
                return 2
            with open(p, "r", encoding="utf-8") as fh:
                try:
                    doc = json.load(fh)
                except Exception as exc:  # noqa: BLE001
                    print("input %s is not valid JSON: %r" % (p, exc), file=sys.stderr)
                    return 2
            try:
                got = expand(doc, os.path.splitext(os.path.basename(p))[0])
            except Bad as exc:
                print("input %s failed schema check: %s" % (p, exc), file=sys.stderr)
                return 2
            for g in got:
                g["color"] = PALETTE[(len(tracks)) % len(PALETTE)]
                if len(got) > 1 or len(args.input) > 1:
                    g["name"] = "%s [%s]" % (g["name"], os.path.basename(p))
                tracks.append(g)
    if not tracks:
        print("no usable tracks", file=sys.stderr)
        return 2

    out = args.out or DEFAULT_OUT
    d = os.path.dirname(os.path.abspath(out))
    if d:
        os.makedirs(d, exist_ok=True)
    title = args.title or default_title(tracks)
    extras = {} if args.no_extras else build_extras(tracks)
    html, plen = build_html(tracks, title, extras)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(html)
    print("wrote %s" % out)
    print("  tracks: %d, points: %s, inline payload %.1f KB, file %.1f KB"
          % (len(tracks), "/".join(str(len(t["points"])) for t in tracks), plen / 1024.0,
             os.path.getsize(out) / 1024.0))
    if extras:
        bits = []
        if extras.get("elev"):
            e0 = extras["elev"]["series"][0]
            bits.append("海拔 %.2f–%.2f m / 爬升 %s m / 源 %s"
                        % (e0["lo_m"], e0["hi_m"],
                           "/".join("%.2f" % r["gain_m"] for r in extras["elev"]["series"]), e0["src"]))
        if extras.get("corr"):
            c0 = extras["corr"]
            bits.append("走廊 s0=%.0f m 中位间距 %s m / 整圈 p50 %s m(%s)"
                        % (c0["s0"], "/".join("%.2f" % v for v in c0["pair_sep"]),
                           "/".join("%.2f" % v for v in c0["lap_p50"][1:]),
                           "/".join("L%d" % (i + 2) for i in range(len(c0["lap_p50"]) - 1))))
        if extras.get("wander"):
            w0 = extras["wander"]
            bits.append("摆动 %s m 分量 %s→%s m (Δrms %s)"
                        % (w0["short_lam_m"], w0["amp_old"][-1], w0["amp_new"][-1], w0["d_rms"]))
        print("  extras: " + " | ".join(bits))
    if args.selftest:
        fails, info = selftest(out)
        print("selftest: %s" % ("PASS" if not fails else "FAIL"))
        for k in sorted(info):
            print("  %s = %s" % (k, info[k]))
        for f in fails:
            print("  FAIL %s" % f)
        return 0 if not fails else 1
    return 0


def demo_name():
    return "合成 demo 轨迹"


def default_title(tracks):
    m = tracks[0].get("meta") or {}
    bits = ["轨迹叠加图"]
    if m.get("dist_m"):
        bits.append("%.2f km" % (float(m["dist_m"]) / 1000.0))
    if m.get("dur_s"):
        bits.append("%d s" % int(float(m["dur_s"])))
    if len(tracks) > 1:
        bits.append("%d 条轨迹对比" % len(tracks))
    return " · ".join(bits)


if __name__ == "__main__":
    sys.exit(main())