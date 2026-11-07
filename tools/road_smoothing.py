"""Conservative, road-bounded rounding of mapped walking-route corners."""
import math

from funsport.track.geom import meters_per_deg_lat, meters_per_deg_lng


def rounded_leg(points, radius_m=8.0, max_offset_m=5.0):
    """Round interior corners; retain endpoints and stay within road corridor.

    A vertex is replaced only if every Bezier sample stays close to the two
    adjacent original road segments. No observed GPS points are inputs.
    """
    if not (math.isfinite(radius_m) and radius_m >= 0 and
            math.isfinite(max_offset_m) and max_offset_m >= 0):
        raise ValueError("Corner rounding parameters must be nonnegative and finite")
    if len(points) < 3 or not radius_m:
        return list(points)
    lat0 = points[0][0]
    sx, sy = meters_per_deg_lng(lat0), meters_per_deg_lat(lat0)
    x0, y0 = points[0][1], points[0][0]
    xy = [((lon - x0) * sx, (lat - y0) * sy) for lat, lon in points]
    result = [points[0]]
    for index in range(1, len(xy) - 1):
        a, b, c = xy[index - 1:index + 2]
        ab, bc = math.dist(a, b), math.dist(b, c)
        if ab < 1 or bc < 1:
            result.append(points[index])
            continue
        cut = min(radius_m, .24 * ab, .24 * bc)
        if cut < .5:
            result.append(points[index])
            continue
        before = (b[0] + (a[0] - b[0]) * cut / ab,
                  b[1] + (a[1] - b[1]) * cut / ab)
        after = (b[0] + (c[0] - b[0]) * cut / bc,
                 b[1] + (c[1] - b[1]) * cut / bc)
        samples = []
        for t in (0, .25, .5, .75, 1):
            x = (1 - t) ** 2 * before[0] + 2 * t * (1 - t) * b[0] + t ** 2 * after[0]
            y = (1 - t) ** 2 * before[1] + 2 * t * (1 - t) * b[1] + t ** 2 * after[1]
            samples.append((x, y))
        # Every curve point is bounded by its distance to the original vertex
        # pair's corridor; neighboring route segments remain unchanged.
        def segment_distance(p, start, end):
            vx, vy = end[0] - start[0], end[1] - start[1]
            u = max(0, min(1, ((p[0] - start[0]) * vx + (p[1] - start[1]) * vy) /
                            (vx * vx + vy * vy)))
            return math.dist(p, (start[0] + u * vx, start[1] + u * vy))
        if any(min(segment_distance(p, a, b), segment_distance(p, b, c)) > max_offset_m
               for p in samples):
            result.append(points[index])
            continue
        result.extend((y / sy + y0, x / sx + x0) for x, y in samples)
    result.append(points[-1])
    return result
