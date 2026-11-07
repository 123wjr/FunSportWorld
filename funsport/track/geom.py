"""几何与随机工具。"""
import math
import random
from datetime import datetime

# 纬度方向每度的米数近似恒定（111132m），但**经度方向随纬度按 cos(lat) 变化**：
# 赤道 111320m/度，北纬 40° 只有 85275m/度。这里保留原来的常数作为默认值，
# 同时提供按纬度计算的函数，由 make_point_ring 依据实际中心纬度选用。
# 固定 86600 隐含纬度 arccos(86600/111132)=38.8°：在该纬度附近误差很小，
# 但在北纬 30.6° 会把东西向距离**少算 10.6%**，导致坐标反算里程比声明里程
# 短 2%（实测 1933.6 vs 1974）——这是纯几何错误，与动力学无关。
MET_PER_DEG_LAT = 111132.0
MET_PER_DEG_LNG = 86600.0


def meters_per_deg_lng(lat_deg):
    """该纬度处 1 度经度的真实地表距离（WGS84 椭球，误差 <0.1%）。"""
    a, b = 6378137.0, 6356752.314245
    phi = math.radians(lat_deg)
    sin2 = math.sin(phi) ** 2
    n = a / math.sqrt(1.0 - 0.00669437999014 * sin2)
    return math.pi / 180.0 * n * math.cos(phi)


def meters_per_deg_lat(lat_deg):
    """该纬度处 1 度纬度的真实地表距离（子午圈曲率半径）。"""
    a, b = 6378137.0, 6356752.314245
    phi = math.radians(lat_deg)
    sin2 = math.sin(phi) ** 2
    k = 0.00669437999014
    m = a * (1.0 - k) / (1.0 - k * sin2) ** 1.5
    return math.pi / 180.0 * m

SPEED_FLOOR = 1.90
SPEED_CEIL = 6.30


def round_to(x, n):
    return float(f"{x:.{n}f}")


class Rng:
    def __init__(self, seed):
        self.r = random.Random(seed)

    def random(self):           return self.r.random()
    def uniform(self, a, b):    return self.r.uniform(a, b)
    def randint(self, a, b):    return self.r.randint(a, b)
    def gauss(self, mu, sigma): return self.r.gauss(mu, sigma)
    def choice(self, items):    return self.r.choice(items)

    def weighted(self, items):
        total = sum(w for _, w in items)
        u = self.r.uniform(0, total)
        for v, w in items:
            u -= w
            if u < 0:
                return v
        return items[-1][0]


def make_point_ring(bd_points, ordered=False, open_path=False):
    """打卡点 → 平面路径 + 弧长表 + 中心。

    ordered=False：稀疏打卡点，按极角排序 + Catmull-Rom 插值（原逻辑）
    ordered=True： 已是有序密集路径（如高德 API 结果），直接使用，不再排序/插值
    open_path=True：有序道路只走到末端，不补闭合段或回绕
    """
    n = len(bd_points)
    if n == 0:
        return [], [0.0], (0.0, 0.0)

    cx = sum(p[0] for p in bd_points) / n
    cy = sum(p[1] for p in bd_points) / n
    # 经度米数按实际纬度取，而不是固定 86600：否则跨纬度的路线会被系统性
    # 压缩/拉伸，坐标反算里程与声明里程对不上（北纬 30.6° 处差 10.6%）。
    m_lng = meters_per_deg_lng(cx)
    m_lat = meters_per_deg_lat(cx)

    if open_path and not ordered:
        raise ValueError("开放路径必须是有序道路折线")
    if ordered:
        # 直接转平面，保持原顺序
        dense = [((p[1] - cy) * m_lng, (p[0] - cx) * m_lat)
                 for p in bd_points]
        # 闭合（若首尾不重合则补一个首点）
        if len(dense) >= 2 and not open_path:
            dx = dense[0][0] - dense[-1][0]
            dy = dense[0][1] - dense[-1][1]
            if (dx * dx + dy * dy) > 1e-6:
                dense.append(dense[0])
    else:
        # 原逻辑：极角排序 + Catmull-Rom
        ordered_pts = sorted(bd_points, key=lambda p: math.atan2(p[0] - cx, p[1] - cy))
        plane = [((p[1] - cy) * MET_PER_DEG_LNG, (p[0] - cx) * MET_PER_DEG_LAT)
                 for p in ordered_pts]
        samples = 18
        dense = []
        m = len(plane)
        for i in range(m):
            p0 = plane[(i - 1) % m]
            p1 = plane[i]
            p2 = plane[(i + 1) % m]
            p3 = plane[(i + 2) % m]
            for j in range(samples):
                t = j / samples
                t2, t3 = t * t, t * t * t
                x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t
                           + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                           + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
                y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t
                           + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                           + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                dense.append((x, y))

    # 弧长表（两分支共用）
    arcs = [0.0]
    for i in range(1, len(dense)):
        a = dense[i - 1]
        b = dense[i]
        arcs.append(arcs[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    return dense, arcs, (cx, cy)


def ring_point_at(dense, arcs, s, open_path=False):
    total = arcs[-1] if arcs else 1.0
    if open_path and s <= 0:
        return dense[0]
    if open_path and s >= total:
        return dense[-1]
    s = s if open_path else s % total
    lo, hi = 0, len(arcs)
    while lo < hi:
        mid = (lo + hi) // 2
        if arcs[mid] < s:
            lo = mid + 1
        else:
            hi = mid
    i = max(lo, 1)
    a = dense[(i - 1) % len(dense)]
    b = dense[i % len(dense)]
    seg = arcs[i] - arcs[i - 1]
    t = (s - arcs[i - 1]) / seg if seg > 0 else 0.0
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def to_bd(x, y, c_lat, c_lng):
    """平面坐标 → BD 经纬度。经度米数按目标纬度取，与 make_point_ring 保持一致。"""
    return (c_lat + y / meters_per_deg_lat(c_lat),
            c_lng + x / meters_per_deg_lng(c_lat))


def fmt_gain_time(ms):
    return datetime.fromtimestamp(ms / 1000).strftime("%Y-%m-%d %H:%M:%S")