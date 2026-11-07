"""Symmetric geometric retracing proxy for local polylines."""
import math

EARTH_RADIUS_M = 6371008.8
OVERLAP_ANGLE_DEG = 5.0


def route_overlap(points, tolerance=1.0):
    """Return length, repeated metres and ratio in a fixed local projection.

    Integrate k/(k+1) along every segment where k other traversals overlap.
    Exact coincident traversals give (passes-1)/passes regardless of direction,
    vertex subdivision or traversal order. Near-collinear segments use an
    angular gate and a clipped distance band; this remains a geometry proxy,
    not road identity. Parallel roads within tolerance can be conflated.
    O(n squared) in segment count; intended for sparse map polylines.
    """
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("Overlap tolerance must be finite and nonnegative")
    if not points:
        return dict(length_m=0.0, repeated_m=0.0, repeat_ratio=0.0)
    if any(not math.isfinite(p[0]) or not math.isfinite(p[1]) for p in points):
        raise ValueError("Route coordinates must be finite")
    # Bounding-box centre is invariant under collinear subdivision/reversal.
    lat0 = (min(p[0] for p in points) + max(p[0] for p in points)) / 2
    lon0 = (min(p[1] for p in points) + max(p[1] for p in points)) / 2
    sy = EARTH_RADIUS_M * math.pi / 180
    sx = sy * math.cos(math.radians(lat0))
    plane = [((p[1] - lon0) * sx, (p[0] - lat0) * sy) for p in points]
    segments = []
    for a, b in zip(plane, plane[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length > 1e-9:
            segments.append((a, b, dx / length, dy / length, length))
    # Bbox grid restricts comparisons to spatial neighbours without changing
    # the polyline. Very long segments fall back to scanning to bound memory.
    cell_size = max(50.0, tolerance * 2)
    grid = {}
    broad = set()
    tiles = []
    for index, (a, b, _, _, _) in enumerate(segments):
        x0 = math.floor((min(a[0], b[0]) - tolerance) / cell_size)
        x1 = math.floor((max(a[0], b[0]) + tolerance) / cell_size)
        y0 = math.floor((min(a[1], b[1]) - tolerance) / cell_size)
        y1 = math.floor((max(a[1], b[1]) + tolerance) / cell_size)
        if (x1 - x0 + 1) * (y1 - y0 + 1) > 4096:
            broad.add(index)
            tiles.append(None)
            continue
        keys = [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
        tiles.append(keys)
        for key in keys:
            grid.setdefault(key, set()).add(index)
    total = sum(s[4] for s in segments)
    repeated = 0.0
    cos_limit = math.cos(math.radians(OVERLAP_ANGLE_DEG))
    for index, (a, b, ux, uy, length) in enumerate(segments):
        events = []
        candidates = set(broad)
        if tiles[index] is None:
            candidates.update(range(len(segments)))
        else:
            for key in tiles[index]:
                candidates.update(grid[key])
        for other in sorted(candidates):
            c, d, vx, vy, _ = segments[other]
            if other == index or abs(ux * vx + uy * vy) < cos_limit:
                continue
            if (max(a[0], b[0]) + tolerance < min(c[0], d[0]) or
                    max(c[0], d[0]) + tolerance < min(a[0], b[0]) or
                    max(a[1], b[1]) + tolerance < min(c[1], d[1]) or
                    max(c[1], d[1]) + tolerance < min(a[1], b[1])):
                continue
            x0 = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy
            x1 = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
            y0 = (c[0] - a[0]) * uy - (c[1] - a[1]) * ux
            y1 = (d[0] - a[0]) * uy - (d[1] - a[1]) * ux
            # Clip the OTHER segment to the normal-distance band first.
            # Whole-endpoint rejection would depend on how it is subdivided.
            t0, t1 = 0.0, 1.0
            delta = y1 - y0
            if abs(delta) < 1e-12:
                if abs(y0) > tolerance + 1e-8:
                    continue
            else:
                low, high = sorted(((-tolerance - y0) / delta, (tolerance - y0) / delta))
                t0, t1 = max(t0, low), min(t1, high)
                if t1 <= t0:
                    continue
            lo, hi = sorted((x0 + (x1 - x0) * t0, x0 + (x1 - x0) * t1))
            lo, hi = max(0.0, lo), min(length, hi)
            if hi > lo + 1e-9:
                events.extend(((lo, 1), (hi, -1)))
        previous = 0.0
        count = 0
        for position, change in sorted(events):
            if count:
                repeated += (position - previous) * count / (count + 1)
            count += change
            previous = position
    return dict(length_m=total, repeated_m=repeated,
                repeat_ratio=repeated / total if total else 0.0)
