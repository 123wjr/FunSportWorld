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
def build_html(tracks, title):
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
    for needle in ("Web Mercator", "步频", "步长", "segMode", "segNorm", "segX", "按各自时长"):
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
    html, plen = build_html(tracks, title)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(html)
    print("wrote %s" % out)
    print("  tracks: %d, points: %s, inline payload %.1f KB, file %.1f KB"
          % (len(tracks), "/".join(str(len(t["points"])) for t in tracks), plen / 1024.0,
             os.path.getsize(out) / 1024.0))
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