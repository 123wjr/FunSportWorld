# -*- coding: utf-8 -*-
"""高德路线拟人化（humanize）。

高德 API 返回的 polyline 是**道路中心线**：顶点密、转角尖、长直段笔直、
同一条路被走两次时两次轨迹逐点重合。直接喂给 generator.build 会产出
「机器画的路线」。本模块把它改成人跑出来的轨迹。

处理链（全部在局部等距平面内完成，最后回 BD-09；平面投影口径与
funsport.track.geom.make_point_ring 的 ordered 分支一致）：

1. clean    合并 <0.5m 的重复顶点
2. collapse 再合并 <2m 的短段。实测环里有 8 段 <1m，最短 0.069m（腿缝），
            任何横向偏移构造在短段上都会退化——早先的 miter 版本就在
            0.55m 短段上生成 168.8° 假尖点与三连重合点，kmax 冲到 3.28 1/m。
3. round    5°..150° 的尖角 → **精确圆弧**（半径被相邻段长 45% 夹紧）。
            不用 5 点贝塞尔：tools/road_smoothing.rounded_leg 每角只采 5 个
            t 值，交付折线曲率被网格化放大 9.8×（kmax 7.80，minR 0.128m）。
4. resample 按弧长重采样（默认 0.6m）
5. lane     稠密点列上沿**局部右法向**偏移 h（靠右行驶 ⇒ 反向重复路段
            自然分开 2h）；曲率半径 <6m 的地方收窄，避免偏移线自交。
6. uturn    >150° 的掉头无法圆角（切线长 r·tan(φ/2)→∞）。该顶点换成以 V
            为圆心、半径 hu 的半圆：V+hu·n_in → V+hu·u_in → V+hu·n_out，
            跑者冲过 V 再绕回。hu=h ⇒ 与前后偏移直线精确相切，无需斜坡。
7. wander   低频横向摆动（波长取**环路长的整数分之一**，保证首尾闭合），
            用曲率半径做 taper，掉头附近不摆动。
8. relane   同向重复路段按通过次序加车道偏移（反向重复已由第 5 步的
            「靠右」自然分开 2h，同向重复不会）。
9. laps     多圈时每圈一个常值横向偏移，接缝处用 30m 斜坡过渡。

对外 API::

    humanize_route(points_bd, seed=0, **opt) -> (points_bd, info)
    humanize_laps(points_bd, seed=0, laps=3, **opt) -> (points_bd, info)
    analyze_route(points_bd, reference_bd=None) -> dict
"""

import math
import random

from .geom import meters_per_deg_lat, meters_per_deg_lng

# 7 位 ≈ 1.1 cm。6 位（≈11 cm）在 0.6 m 采样下会引入约 10° 的航向噪声，
# 实测把 |dθ| 总量从 1214°/km 抬到 3896°/km —— 这是量化伪影，不是几何。
COORD_DECIMALS = 7

# 摆动波长下限。再短就低于「左右各让一步」的尺度，而且在 0.6 m 采样下会退化成
# 锯齿：100 m 的小环上 55 个波 = 1.8 m 波长、振幅 0.26 m，单点转角 40°+。
WANDER_MIN_WAVE_M = 20.0

# 每圈扇出的保护阈值：只有曲率半径小于 LAP_TAPER_HI_M 的掉头弧才把扇出收回 0，
# 普通圆角（corner_radius_m=8）保持全幅。
LAP_TAPER_HI_M = 2.6
LAP_RAMP_M = 14.0

DEFAULTS = {
    # 2. 转角圆滑
    "corner_radius_m": 8.0,        # 目标圆角半径（校园场景可以大胆）
    "corner_min_deg": 5.0,         # 小于此角度不算拐角
    "corner_max_deg": 150.0,       # 大于此角度走掉头分支
    "corner_max_len_frac": 0.45,   # 切线长最多吃掉的相邻段比例
    "arc_sample_m": 0.5,           # 圆弧采样间距
    # 3. 短段合并
    "collapse_min_seg_m": 2.0,     # 比它短的段直接并掉（腿缝/重复点）
    # 4. 重采样
    "resample_m": 0.6,
    # 5. 靠右行驶
    "lane_offset_m": 2.0,          # 全程向右偏移；掉头两腿因此分开 2h
                                   # 实测：h=1.2 时重复走廊仍剩 156 m(10.7%)，
                                   # h=2.0 → 0~61 m(0~4.2%)，偏离 p95 仍 <4.1 m。
    "lane_taper_lo_m": 2.0,        # 曲率半径小于此值 → 偏移收窄到 0
    "lane_taper_hi_m": 6.0,        # 大于此值 → 全幅偏移
    # 6. 掉头
    "u_turn_min_deg": 150.0,
    # 标称 1.2 m，但 0.6 m 采样 + 7 位 BD 量化（1 cm）把实测三点圆半径压到 1.14~1.18 m，
    # 低于"掉头半径 >= 1.2 m"的硬约束；抬到 1.3 m 后实测 1.267 m（laps=1/3）。
    "u_turn_radius_m": 1.3,
    "u_turn_ramp_m": 12.0,         # hu≠h 时在顶点前这么长内平滑过渡
    # 6. 直线摆动
    # 振幅按真实跑步数据定标：data/huawei 80 条记录 / 179 km，取"直线段"（自身
    # 50 m 中轴曲率半径 >150 m）后用居中滑动平均剥出横向摆动，生成侧用同一条
    # 合成 1480 m 直路做同样测量（见 docs/.route-shape-empirics.md）。结论是不必
    # 整体放大：10~100 m 各带通 RMS 差 <15%，只有 20~50 m 一档偏小 15~20%，
    # 因此保留前三条、加第 4 个 ~27 m 短波分量。
    "wander_amp_m": (1.20, 0.80, 0.52, 0.26),   # 对应下面 4 个波数
    "wander_waves": (8, 16, 30, 55),           # 波长 = 环长 / 波数（下限 20 m）
    "wander_taper_lo_m": 4.0,              # 曲率半径小于此值不摆动
    "wander_taper_hi_m": 6.0,              # 大于此值全幅摆动
    "offset_ramp_m": 10.0,                 # 高曲率区到全幅偏移的过渡长度
    "offset_slope_max": 0.20,              # 横向偏移斜率上限 ≈ 11° 附加航向
    # 7. 同向重复分离
    "repeat_tol_m": 1.0,
    "repeat_min_gap_m": 20.0,
    "repeat_angle_cos": 0.90,
    "repeat_lane_m": 1.6,          # 同向重复段的额外分道（1.6 实测把残留再压一档）
    "repeat_smooth_m": 10.0,
    # 1b. 针尖毛刺（高德折线在路口的小尖刺，2~6 m 往返）
    "spur_max_m": 6.0,
    "spur_close_m": 10.0,
    # 8. 多圈
    # 每圈用独立子种子重新走一遍完整拟人化链（摆动相位、圆角半径都重新取），
    # 而不是把第一圈刚性复制若干份 —— 后者会让每圈的走线、圆角、摆动完全一样。
    "lap_corner_jitter": 0.15,     # 每圈圆角半径相对目标的随机抖动（±15%）
    "lap_ramp_m": 30.0,            # 接缝处平滑长度（把上一圈终点接到本圈起点）
    # 每圈相对第 0 圈的横向扇出，倍数取 (0, +1, -1, +2, -2)（见 _LAP_LANE_PATTERN）。
    # 定标依据：真实同向重访的横向间距 p50 1.04 / p90 2.63 m、反向 p50 2.70 /
    # p90 5.95 m；2.0 m 落在真实分布内，又大于 route_metrics 判定的 1.5 m 重合容差。
    # 实测多圈重复走廊 32.4%（laps=3）→ 扇出后接近 0，见 docs/.route-shape-report.md。
    "lap_lane_m": 2.4,
    # 各圈的摆动相位相对第 0 圈的抖动幅度（弧度）。每圈完全独立取相位时，相邻
    # 两圈的摆动差可达 ±2.4 m，会把 lap_lane_m 的扇出整个吃掉（实测 laps=3 重合
    # 仍剩 16%）；把相位限制在基相位附近，各圈的"人味"不同、但整体走线成车道，
    # 重合才真的降下来。
    "lap_phase_jitter": 0.3,
    "clean_min_seg_m": 0.5,
    # 9. 输出抽稀
    # BD 只留 COORD_DECIMALS 位小数（7 位 ≈ 1.1 cm）。若输出点间距只有
    # 0.6 m，量化误差在差商里被放大成 ~10° 的假航向噪声：实测同一折线
    # 6 位 → 3896°/km、7 位 + 0.6 m 间距 → 1874°/km、7 位 + 1.5 m 间距 →
    # 1320°/km，而平面真值是 1290°/km。所以直线段抽稀到 out_spacing_m，
    # 弯道（半径 < out_keep_radius_m，即圆角弧与掉头弧）一个点都不丢。
    "out_spacing_m": 1.5,
    "out_keep_radius_m": 20.0,
}


# ---------------------------------------------------------------- 基础工具

def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _unit(dx, dy):
    L = math.hypot(dx, dy)
    if L < 1e-12:
        return (0.0, 0.0)
    return (dx / L, dy / L)


def _right(u):
    """行进方向 u 的右手法向（数学平面，y 轴向上：右转 = 顺时针）。"""
    return (u[1], -u[0])


def _smoothstep(x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * (3.0 - 2.0 * x)


def _cyclic(points):
    """去掉与首点重合的末点，返回纯环点列。"""
    pts = list(points)
    while len(pts) > 1 and _dist(pts[0], pts[-1]) < 1e-9:
        pts.pop()
    return pts


def _seg_info(P):
    """环上每段的方向与长度。"""
    n = len(P)
    dirs, lens = [], []
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        L = _dist(a, b)
        lens.append(L)
        dirs.append(_unit(b[0] - a[0], b[1] - a[1]))
    return dirs, lens


def _turn_deg(u_in, u_out):
    dot = max(-1.0, min(1.0, u_in[0] * u_out[0] + u_in[1] * u_out[1]))
    return math.degrees(math.acos(dot))


def _intersect(p1, u1, p2, u2):
    den = u1[0] * u2[1] - u1[1] * u2[0]
    if abs(den) < 1e-9:
        return None
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    t = (dx * u2[1] - dy * u2[0]) / den
    return (p1[0] + t * u1[0], p1[1] + t * u1[1])


# ---------------------------------------------------------------- 投影

def _to_plane(points_bd):
    n = len(points_bd)
    cx = sum(p[0] for p in points_bd) / n
    cy = sum(p[1] for p in points_bd) / n
    m_lng = meters_per_deg_lng(cx)
    m_lat = meters_per_deg_lat(cx)
    P = [((p[1] - cy) * m_lng, (p[0] - cx) * m_lat) for p in points_bd]
    return P, (cx, cy), (m_lng, m_lat)


def _to_bd(P, center, scale):
    cx, cy = center
    m_lng, m_lat = scale
    out = []
    for x, y in P:
        out.append((round(y / m_lat + cx, COORD_DECIMALS),
                    round(x / m_lng + cy, COORD_DECIMALS)))
    return out


# ---------------------------------------------------------------- 1. clean

def _snip_spurs(P, opt):
    """剪掉地图数据里的「针尖」：来回各走 2~6 m 又原路返回的毛刺。

    高德折线在路口/掉头点偶尔会出现 2 m 的尖刺（相邻两条腿几乎反向、端点
    却只隔几米）。这种地方既不是真实掉头（真实掉头两条腿有几十米长），又会让
    后面的靠右偏移把两条腿推向彼此、半径 1.2 m 的两个半圆互相穿插，交付折线
    于是留下 70~110 度的假折角。判据：转角 > 150 度、且相邻某条腿短于
    spur_max_m、且前后端点相距 < spur_close_m —— 直接删掉顶点，让它变成通路。
    """
    spur_max = float(opt["spur_max_m"])
    close_max = float(opt["spur_close_m"])
    out = list(P)
    changed = True
    while changed and len(out) > 3:
        changed = False
        n = len(out)
        dirs, lens = _seg_info(out)
        for i in range(n):
            turn = _turn_deg(dirs[i - 1], dirs[i])
            if turn <= 150.0:
                continue
            if lens[i - 1] >= spur_max and lens[i] >= spur_max:
                continue
            if _dist(out[i - 1], out[(i + 1) % n]) > close_max:
                continue
            del out[i]
            changed = True
            break
    return out


def _clean(P, min_seg_m):
    n = len(P)
    out = []
    for i in range(n):
        p = P[i]
        if not out or _dist(out[-1], p) >= min_seg_m:
            out.append(p)
    while len(out) > 2 and _dist(out[0], out[-1]) < min_seg_m:
        out.pop()
    return out


# ---------------------------------------------------------------- 2. round

def _round_corners(P, opt):
    r_target = opt["corner_radius_m"]
    lo = opt["corner_min_deg"]
    hi = opt["corner_max_deg"]
    frac = opt["corner_max_len_frac"]
    ds = opt["arc_sample_m"]
    dirs, lens = _seg_info(P)
    n = len(P)
    out = []
    rounded = 0
    for i in range(n):
        V = P[i]
        u_in, u_out = dirs[i - 1], dirs[i]
        Lp, Ln = lens[i - 1], lens[i]
        if Lp < 1e-9 or Ln < 1e-9:
            out.append(V)
            continue
        turn = _turn_deg(u_in, u_out)
        if turn < lo or turn > hi:
            out.append(V)
            continue
        phi = math.radians(turn)
        tan_half = math.tan(phi * 0.5)
        if tan_half < 1e-9:
            out.append(V)
            continue
        t = min(r_target * tan_half, frac * Lp, frac * Ln)
        r = t / tan_half
        if r < 0.30 or t < 0.10:
            out.append(V)
            continue
        mx, my = u_out[0] - u_in[0], u_out[1] - u_in[1]
        ml = math.hypot(mx, my)
        if ml < 1e-9:
            out.append(V)
            continue
        # 圆心到 V 的距离用**切线长 t** 算：O 在 (u_out-u_in) 方向、距离 t/sin(φ/2)。
        # 用 r/sin(φ/2) 是错的（只有 tan(φ/2)=1 的 90° 角碰巧对），会让圆弧
        # 不经过 A/B，实测把 1427m 的环撑到 1932m。
        d = t / math.sin(phi * 0.5)
        O = (V[0] + d * mx / ml, V[1] + d * my / ml)
        A = (V[0] - t * u_in[0], V[1] - t * u_in[1])
        B = (V[0] + t * u_out[0], V[1] + t * u_out[1])
        a0 = math.atan2(A[1] - O[1], A[0] - O[0])
        a1 = math.atan2(B[1] - O[1], B[0] - O[0])
        sw = a1 - a0
        while sw > math.pi:
            sw -= 2.0 * math.pi
        while sw < -math.pi:
            sw += 2.0 * math.pi
        steps = max(2, int(abs(sw) * r / ds) + 1)
        for k in range(steps + 1):
            ang = a0 + sw * k / steps
            out.append((O[0] + r * math.cos(ang), O[1] + r * math.sin(ang)))
        rounded += 1
    return out, rounded


# ------------------------------------------------------- 3+4. lane & uturn

def _lane_and_uturn(P, opt):
    """稠密点列上的靠右偏移 + 掉头半圆。

    P 必须是**已重采样**的稠密环（间距 ≈ arc_sample_m）。横向场沿法向逐点
    施加，因此没有 miter、没有短段退化；掉头顶点换成半径 hu 的半圆，hu 与
    偏移量一致时进出腿与半圆精确相切。

    返回 (新点列, 掉头信息列表, 施加了偏移的点数)。
    """
    h = opt["lane_offset_m"]
    hu = opt["u_turn_radius_m"]
    hi = opt["u_turn_min_deg"]
    ds = max(opt.get("_step_m") or opt["arc_sample_m"], 1e-6)
    lo = opt["lane_taper_lo_m"]
    hie = opt["lane_taper_hi_m"]
    ramp = max(opt["u_turn_ramp_m"], ds)

    n = len(P)
    dirs, _lens = _seg_info(P)
    turns = [_turn_deg(dirs[i - 1], dirs[i]) for i in range(n)]
    t = _tangents(P, 2)

    # 曲率 taper：半径太小时收窄偏移，否则内侧偏移线自交（r - h < 0）
    kap = _curvature(P, ds)
    raw = []
    for i in range(n):
        r = 1.0 / kap[i] if kap[i] > 1e-9 else 1e9
        raw.append(h * _smoothstep((r - lo) / max(hie - lo, 1e-9)))
    field = _cycle_smooth(raw, max(1, int(round(3.0 / ds))))

    ut_idx = [i for i in range(n) if turns[i] >= hi]
    if ut_idx:
        reach = int(math.ceil(ramp / ds))
        near = [1e18] * n
        for i in ut_idx:
            for k in range(-reach, reach + 1):
                j = (i + k) % n
                d = abs(k) * ds
                if d < near[j]:
                    near[j] = d
        for i in range(n):
            if near[i] < 1e17:
                w = _smoothstep(near[i] / ramp)      # 0 = 贴顶点，1 = 已出 ramp
                field[i] = hu * (1.0 - w) + field[i] * w

    ut_set = set(ut_idx)
    out = []
    uturns = []
    for i in range(n):
        V = P[i]
        if i in ut_set:
            # 掉头：以 V 为圆心、半径 hu 的半圆，前凸方向 = u_in
            u_in, u_out = dirs[i - 1], dirs[i]
            n_in, n_out = _right(u_in), _right(u_out)
            a0 = math.atan2(n_in[1], n_in[0])
            a1 = math.atan2(n_out[1], n_out[0])
            at = math.atan2(u_in[1], u_in[0])
            sweep = (a1 - a0) % (2.0 * math.pi)
            if (at - a0) % (2.0 * math.pi) > sweep:
                sweep -= 2.0 * math.pi
            steps = max(4, int(abs(sweep) * hu / ds) + 1)
            for k in range(steps + 1):
                ang = a0 + sweep * k / steps
                out.append((V[0] + hu * math.cos(ang), V[1] + hu * math.sin(ang)))
            uturns.append({"vertex": i, "turn_deg": round(turns[i], 2),
                           "radius_m": hu, "arc_len_m": abs(sweep) * hu})
            continue
        nr = _right(t[i])
        out.append((V[0] + field[i] * nr[0], V[1] + field[i] * nr[1]))
    return out, uturns, len(out)


# ---------------------------------------------------------------- 5. resample

def _resample(P, ds):
    n = len(P)
    segs = []
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        segs.append((a, b, _dist(a, b)))
    total = sum(s[2] for s in segs)
    if total <= 1e-9:
        return list(P), 0.0, ds
    m = max(4, int(round(total / ds)))
    step = total / m
    cum = [0.0]
    for _, _, L in segs:
        cum.append(cum[-1] + L)
    out = []
    k = 0
    for j in range(m):
        s = j * step
        while k + 1 < len(segs) and cum[k + 1] < s:
            k += 1
        L = segs[k][2]
        t = 0.0 if L <= 1e-12 else min(1.0, max(0.0, (s - cum[k]) / L))
        a, b = segs[k][0], segs[k][1]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out, total, step


def _tangents(P, win):
    n = len(P)
    out = []
    for i in range(n):
        a, b = P[(i - win) % n], P[(i + win) % n]
        out.append(_unit(b[0] - a[0], b[1] - a[1]))
    return out


def _curvature(P, step):
    """逐点曲率 |dθ/ds|（±1 点中心差分切向）。"""
    n = len(P)
    t = _tangents(P, 2)
    k = []
    for i in range(n):
        a, b = t[i - 1], t[i]
        d = math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        k.append(abs(d) / step if step > 1e-12 else 0.0)
    return k


def _cycle_smooth(values, win):
    """环上滑动平均（win 为半径点数），用于把横向场抹平。"""
    n = len(values)
    if n == 0 or win <= 0:
        return list(values)
    win = min(win, max(1, n // 4))
    out = []
    for i in range(n):
        acc = 0.0
        for d in range(-win, win + 1):
            acc += values[(i + d) % n]
        out.append(acc / (2 * win + 1))
    return out


# ---------------------------------------------------------------- 6. wander

def _wander_raw(P, step, total, opt):
    """几条低频正弦叠加出的横向摆动目标（未乘包络）。波长为环长整数分之一，
    因此首尾天然闭合。

    相位由 humanize_* 的 seed 决定（opt["_phases"]），这样不同 seed 给出不同的
    摆动形状；只随机相位、不随机波数，是因为非整数波数会让环的首尾对不上。

    波数按环长封顶（波长 >= WANDER_MIN_WAVE_M）：短环上同一个波数会把波长压到
    1~2 m，振幅不变就成了锯齿而不是摆动。
    """
    n = len(P)
    amps = opt["wander_amp_m"]
    wcap = max(1, int(total / WANDER_MIN_WAVE_M))
    waves = [max(1, min(int(w), wcap)) for w in opt["wander_waves"]]
    phases = opt.get("_phases") or [0.37 * (j + 1) for j in range(len(waves))]
    out = []
    for i in range(n):
        s = i * step / max(total, 1e-9)
        v = 0.0
        for A, w, ph in zip(amps, waves, phases):
            v += A * math.sin(2.0 * math.pi * w * s + ph)
        out.append(v)
    return out


def _offset_envelope(P, step, opt):
    """摆动包络：高曲率区（圆角弧、掉头弧）附近收紧到 0，直线段放开到 1。

    早期版本按**点曲率**直接判定 0/1，弧与直线交界处横向偏移在 0.6 m 内从
    2.2 m 掉到 0，等于凭空造出一个 70° 折角（实测 turn_sum 从 1170°/km
    涨到 8050°/km）。这里改成两步都连续：
      1) 算出"到最近高曲率点的弧长距离"，做 0..1 的斜坡；
      2) 再对这个斜坡做长窗（默认 12 m）环上滑动平均，交界处自然过渡。
    """
    n = len(P)
    hi = opt["wander_taper_hi_m"]
    ramp = max(opt["offset_ramp_m"], step)
    kap = _curvature(P, step)
    blocked = [i for i in range(n) if kap[i] > 1e-9 and 1.0 / kap[i] < hi]
    if not blocked:
        return [1.0] * n
    reach = int(math.ceil(ramp / step))
    near = [1e18] * n
    for i in blocked:
        for k in range(-reach, reach + 1):
            j = (i + k) % n
            d = abs(k) * step
            if d < near[j]:
                near[j] = d
    raw = []
    for i in range(n):
        # near 是"到最近高曲率点的弧长距离"：贴着弧 -> 0（不摆动），
        # 超出 ramp -> 1（全幅摆动）。早期版本把 far 写成 0.0，等于把整条
        # 直线的摆动全部关掉（实测 env 峰值只有 0.32），这里修正。
        raw.append(1.0 if near[i] >= 1e17 else _smoothstep(near[i] / ramp))
    # 用**滑动最小值**（腐蚀）而不是滑动平均来收口：掉头弧只有 ~3.8 m 长，
    # 而 ramp 窗口有 ~17 个点，平均会把保护值从 0 平均回 0.46（实测 env[掉头]
    # = 0.463），等于保护失效、掉头半径被切到 0.5 m。取最小值等价于把保护
    # 半径外扩 ramp，仍然连续，只是不再是"半开"。最后 3 点平滑去台阶。
    out = list(raw)
    for i in range(n):
        b = raw[i]
        for k in range(-reach, reach + 1):
            v = raw[(i + k) % n]
            if v < b:
                b = v
        out[i] = b
    return _cycle_smooth(out, 2)


def _limit_slope(field, step, slope_max):
    """反复环上平滑，直到横向偏移的斜率 |df/ds| <= slope_max。

    偏移场沿法向施加，附加航向角 ≈ atan(df/ds)，斜率 0.2 只对应 11°，而
    斜率 3.7 会造出 75° 假折角。平滑到斜率上限后，转角总量由建筑几何决定。
    """
    f = list(field)
    for _ in range(400):
        worst = 0.0
        n = len(f)
        for i in range(n):
            worst = max(worst, abs(f[(i + 1) % n] - f[i]) / max(step, 1e-9))
        if worst <= slope_max:
            break
        f = _cycle_smooth(f, 2)
    return f


def _decimate(P, step, opt):
    """输出抽稀：弯道全保留，直线按 out_spacing_m 抽稀（见 DEFAULTS 里的量化噪声说明）。"""
    n = len(P)
    spacing = max(float(opt.get("out_spacing_m", 1.5)), step)
    r_keep = float(opt.get("out_keep_radius_m", 20.0))
    kap = _curvature(P, step)
    out = []
    acc = spacing
    for i in range(n):
        r = (1.0 / kap[i]) if kap[i] > 1e-12 else 1e18
        if r < r_keep:
            out.append(P[i])
            acc = 0.0
        else:
            acc += step
            if acc >= spacing:
                out.append(P[i])
                acc = 0.0
    if len(out) < 4:
        return list(P)
    return out


# ---------------------------------------------------------------- 7. relane

def _repeat_pass(P, step, opt):
    tol = opt["repeat_tol_m"]
    gap = opt["repeat_min_gap_m"]
    cos_min = opt["repeat_angle_cos"]
    n = len(P)
    cell = max(3.0, tol * 2.0)
    grid = {}
    t = _tangents(P, 2)
    pass_idx = [0] * n
    for i in range(n):
        x, y = P[i]
        gx, gy = int(math.floor(x / cell)), int(math.floor(y / cell))
        best = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), ()):
                    arc_gap = min(i - j, n - (i - j)) * step
                    if arc_gap < gap:
                        continue
                    if (P[j][0] - x) ** 2 + (P[j][1] - y) ** 2 > tol * tol:
                        continue
                    if t[i][0] * t[j][0] + t[i][1] * t[j][1] < cos_min:
                        continue
                    if best is None or j < best:
                        best = j
        if best is not None:
            pass_idx[i] = pass_idx[best] + 1
        grid.setdefault((gx, gy), []).append(i)
    return pass_idx


def _relane_field(P, step, opt):
    n = len(P)
    pat = (0.0, 1.0, -1.0, 2.0, -2.0)
    idx = _repeat_pass(P, step, opt)
    raw = [pat[min(k, len(pat) - 1)] * opt["repeat_lane_m"] for k in idx]
    win = max(1, int(round(opt["repeat_smooth_m"] / max(step, 1e-9))))
    return _cycle_smooth(raw, win), idx


# ---------------------------------------------------------------- 主流程

def _humanize_plane(P, opt):
    """平面内完整处理链。P 为纯环点列；多圈由 humanize_laps 逐圈独立调用。"""
    P = _clean(P, opt["clean_min_seg_m"])
    P = _snip_spurs(P, opt)
    P = _clean(P, opt["collapse_min_seg_m"])
    P, n_rounded = _round_corners(P, opt)
    P = _clean(P, 1e-6)
    P, _total0, step = _resample(P, opt["resample_m"])
    opt = dict(opt)
    opt["_step_m"] = step
    P, uturns, n_shifted = _lane_and_uturn(P, opt)
    total = sum(_dist(P[i], P[(i + 1) % len(P)]) for i in range(len(P)))
    t = _tangents(P, 2)
    n = len(P)

    env = _offset_envelope(P, step, opt)
    want = _wander_raw(P, step, total, opt)
    relane, pass_idx = _relane_field(P, step, opt)

    # 包络必须作用在**合成后的总偏移**上，不能只乘摆动项：掉头弧半径只有
    # u_turn_radius_m（1.2 m），若在弧上还叠一个 ±0.8 m 的分道偏移，半径会被
    # 削到 0.4 m（实测 seed=0 掉头处 minR=0.52、单点转角 61°）。把 relane 一起
    # 收进包络后，弧上偏移归零、半径还原，而直线段的分道能力不受影响。
    # 另外横向偏移场必须先限斜率再施加，否则场里的台阶会被几何放大成假折角。
    # 每圈的横向扇出也并进总额偏移：多圈跑同一条路时，第 k 圈整体相对第 0 圈
    # 平移 lap_lane_m 的整数倍，这样相邻两圈的走廊真正分开（而不是靠随机相位
    # 偶尔错开）。同样过包络 —— 圆角/掉头弧上它归零，掉头半径不被撑大。
    # 每圈的横向扇出用**自己的一套包络**：只让掉头弧（半径 < LAP_TAPER_HI_M）把
    # 它收回 0，半径 8 m 的普通圆角保持全幅。用摆动那套（6 m）会把 16 个圆角
    # 也一起抹掉，而圆角加圆弧过渡约占环长 1/4 —— 实测那正是各圈重新叠到一起
    # 的主因（laps=3 重合 32%）。掉头弧必须归零：弧半径只有 1.2 m，弧上叠一个
    # 常值偏移会把两条腿的间距吃掉（实测 minR 掉到 0.32 m、单点转角 114°）。
    lap_lane = float(opt.get("_lap_lane_m") or 0.0)
    lap_env = (_offset_envelope(P, step, dict(opt, wander_taper_hi_m=LAP_TAPER_HI_M,
                                              offset_ramp_m=LAP_RAMP_M))
               if lap_lane else [0.0] * n)
    raw_off = [env[i] * (want[i] + relane[i]) + lap_lane * lap_env[i] for i in range(n)]
    off = _limit_slope(raw_off, step, opt["offset_slope_max"])

    out = []
    for i in range(n):
        nr = _right(t[i])
        out.append((P[i][0] + off[i] * nr[0], P[i][1] + off[i] * nr[1]))
    n_dense = len(out)
    out = _decimate(out, step, opt)
    info = {
        "n_points_in": len(P),
        "corners_rounded": n_rounded,
        "shifted_points": n_shifted,
        "n_points_out": len(out),
        "n_points_dense": n_dense,
        "uturns": uturns,
        "total_length_m": total,
        "sample_step_m": step,
        "wander_max_m": max(abs(v) for v in want) if want else 0.0,
        "offset_max_m": max(abs(v) for v in off) if off else 0.0,
        "offset_slope_max": max((abs(off[(i + 1) % n] - off[i]) / step for i in range(n)), default=0.0),
        "relane_max_m": max(abs(v) for v in relane) if relane else 0.0,
        "lap_lane_m": lap_lane,
        "pass_index_max": max(pass_idx) if pass_idx else 0,
    }
    return out, info


def _seed_phases(seed, opt):
    """把 seed 变成摆动相位。只随机相位，不动波数（见 _wander_raw）。"""
    import random

    rng = random.Random(seed)
    return [rng.uniform(0.0, 2.0 * math.pi) for _ in opt["wander_waves"]]


def humanize_route(points_bd, seed=0, **opt):
    """单圈拟人化。返回 (新的 BD 点列, info)。"""
    o = dict(DEFAULTS)
    o.update(opt)
    o.setdefault("_phases", _seed_phases(seed, o))
    P, center, scale = _to_plane(_cyclic(points_bd))
    out, info = _humanize_plane(P, o)
    pts = _to_bd(out, center, scale)
    info["laps"] = 1
    return pts, info


def _arc_positions(P):
    """环上逐点弧长与整环长度（最后一段回到首点）。"""
    cum = [0.0]
    for i in range(1, len(P)):
        cum.append(cum[-1] + _dist(P[i - 1], P[i]))
    total = cum[-1] + _dist(P[-1], P[0]) if len(P) > 1 else 0.0
    return cum, total


# 第 k 圈相对第 0 圈的横向扇出倍数。第 0 圈必须为 0：laps=1 与 humanize_route
# 要逐点一致（tests/test_humanize_route.py::test_single_lap_matches_route_entry）。
_LAP_LANE_PATTERN = (0.0, 1.0, -1.0, 2.0, -2.0)


def _lap_seed(seed, k):
    """第 k 圈的子种子：k=0 用原种子（单圈与 humanize_route 完全一致）。"""
    if k == 0:
        return int(seed)
    return (int(seed) * 7919 + k * 104729) & 0x7FFFFFFF


def humanize_laps(points_bd, seed=0, laps=1, **opt):
    """多圈拟人化：每圈按新路线独立重跑完整链，再在接缝处平滑拼接。

    与「拟人化一圈后刚性平移复制」不同，这里第 k 圈用独立子种子
    （_lap_seed）重新生成：摆动相位、圆角半径（±lap_corner_jitter）都重新取，
    所以每一圈的走线、圆角、摆动都不一样，拼起来的轨迹不会看出重复。

    拼接方式：第 k≥1 圈整体先不移动，圈首 lap_ramp_m 内把它平滑接到上一圈
    的终点（位移量 = 上一圈末点 − 本圈首点，按 smoothstep 衰减到 0），这样
    接缝处位置连续，而本圈其余部分保持自己独立的几何。第 0 圈用原种子，
    因此 laps=1 与 humanize_route(seed) 结果一致。

    返回点列长度为各圈弧长之和（不含闭合段）；调用方按需补闭合点。若调用方
    只跑其中一段里程（生成器按目标里程截断），拼接方式不会影响那段几何。
    """
    import random

    o = dict(DEFAULTS)
    o.update(opt)
    laps = max(1, int(laps))
    P, center, scale = _to_plane(_cyclic(points_bd))
    jitter = max(0.0, float(o["lap_corner_jitter"]))
    base_radius = float(o["corner_radius_m"])
    lap_lane = max(0.0, float(o["lap_lane_m"]))
    pj = max(0.0, float(o["lap_phase_jitter"]))

    base_phases = _seed_phases(seed, o)
    singles = []
    infos = []
    lap_seeds = []
    lap_radii = []
    for k in range(laps):
        lap_seed = _lap_seed(seed, k)
        lap_seeds.append(lap_seed)
        ok = dict(o)
        if k == 0 or pj <= 0.0:
            ok["_phases"] = base_phases if k == 0 else _seed_phases(lap_seed, ok)
        else:
            rng_p = random.Random(lap_seed ^ 0x9E3779B9)
            ok["_phases"] = [ph + rng_p.uniform(-pj, pj) for ph in base_phases]
        ok["_lap_lane_m"] = lap_lane * _LAP_LANE_PATTERN[min(k, len(_LAP_LANE_PATTERN) - 1)]
        if k > 0 and jitter > 0.0:
            rng = random.Random(lap_seed ^ 0x5DEECE66D)
            ok["corner_radius_m"] = base_radius * rng.uniform(1.0 - jitter, 1.0 + jitter)
        lap_radii.append(ok["corner_radius_m"])
        single, info = _humanize_plane(P, ok)
        singles.append(single)
        infos.append(info)

    out = list(singles[0])
    seams = []
    ramp = max(1.0, float(o["lap_ramp_m"]))
    for k in range(1, laps):
        nxt = singles[k]
        cum, lap_len = _arc_positions(nxt)
        rr = max(1.0, min(ramp, lap_len * 0.2))
        dx = out[-1][0] - nxt[0][0]
        dy = out[-1][1] - nxt[0][1]
        seams.append(math.hypot(dx, dy))
        # 首点与上一圈末点重合，跳过它避免零长段；其余点在 rr 内平滑衰减位移。
        for i in range(1, len(nxt)):
            w = 1.0 - _smoothstep(min(cum[i] / rr, 1.0)) if cum[i] < rr else 0.0
            out.append((nxt[i][0] + dx * w, nxt[i][1] + dy * w))

    # 收尾也要接回第 0 圈的起点：每圈的横向扇出让最后一圈的末点离起点差
    # 2*lap_lane_m 左右，直接闭合会在环的接缝处留下一个 40~50° 的折角
    # （实测 200 m 方角环 laps=3、lap_lane_m=2.5 时闭合段单点转角 49.7°）。
    # 这里把最后一段的位移按 smoothstep 衰减回 0，闭合差摊到 lap_ramp_m 上。
    if laps > 1 and out:
        cum, lap_len = _arc_positions(out)
        rr2 = max(1.0, min(ramp, lap_len * 0.2))
        dx2 = out[0][0] - out[-1][0]
        dy2 = out[0][1] - out[-1][1]
        if math.hypot(dx2, dy2) > 1e-9:
            for i in range(len(out) - 1, -1, -1):
                d = cum[-1] - cum[i]
                if d >= rr2:
                    break
                w = _smoothstep(1.0 - d / rr2)
                out[i] = (out[i][0] + dx2 * w, out[i][1] + dy2 * w)

    total = sum(_dist(out[i], out[i + 1]) for i in range(len(out) - 1))
    lap_lengths = [_arc_positions(s)[1] for s in singles]
    uturns = []
    for k, info in enumerate(infos):
        for u in info.get("uturns") or ():
            u = dict(u)
            u["lap"] = k
            uturns.append(u)
    info = dict(infos[0])
    info.update({
        "laps": laps,
        "lap_seeds": lap_seeds,
        "lap_lengths_m": lap_lengths,
        "single_lap_length_m": lap_lengths[0],
        "total_length_m": total,
        "seam_shift_max_m": max(seams) if seams else 0.0,
        "offset_max_m": max(i.get("offset_max_m") or 0.0 for i in infos),
        "corners_rounded": sum(int(i.get("corners_rounded") or 0) for i in infos),
        "uturns": uturns,
        "n_points_out": len(out),
        "seam_ramp_m": ramp,
        "lap_corner_radius_m": lap_radii,
        "lap_lane_m": [lap_lane * _LAP_LANE_PATTERN[min(k, len(_LAP_LANE_PATTERN) - 1)]
                       for k in range(laps)],
    })
    return _to_bd(out, center, scale), info


# ---------------------------------------------------------------- 度量

def analyze_route(points_bd, reference_bd=None):
    """路线几何体检：转角、最小半径、曲率跳变、闭合、重复率、偏离。"""
    P0 = _cyclic(points_bd)
    Pp0, _c, _s = _to_plane(P0)
    P, total, step = _resample(Pp0, 0.5)
    n = len(P)
    out = {"n_vertices": len(P0), "length_m": total}

    dirs, lens = _seg_info(P)
    turns = [_turn_deg(dirs[i - 1], dirs[i]) for i in range(n)]
    st = sorted(turns)
    out["turn_p50_deg"] = st[n // 2]
    out["turn_p90_deg"] = st[int(n * 0.90)]
    out["turn_max_deg"] = st[-1]
    out["turn_sum_per_km_deg"] = sum(turns) / max(total / 1000.0, 1e-9)
    out["n_turn_gt_45"] = sum(1 for x in turns if x > 45.0)
    out["n_turn_gt_90"] = sum(1 for x in turns if x > 90.0)
    out["n_turn_gt_135"] = sum(1 for x in turns if x > 135.0)

    kap = _curvature(P, step)
    kmax = max(kap) if kap else 0.0
    out["kmax_1_per_m"] = kmax
    out["min_radius_m"] = (1.0 / kmax) if kmax > 1e-9 else float("inf")
    jumps = sorted(abs(kap[i] - kap[i - 1]) for i in range(n))
    out["curv_jump_p50"] = jumps[n // 2]
    out["curv_jump_max"] = jumps[-1]

    sd = []
    for i in range(n):
        ax, ay = P[(i - 1) % n]
        bx, by = P[i]
        cx, cy = P[(i + 1) % n]
        ddx = (cx - 2 * bx + ax) / (step * step)
        ddy = (cy - 2 * by + ay) / (step * step)
        sd.append(math.hypot(ddx, ddy))
    sd.sort()
    out["straight_2nd_diff_p50"] = sd[n // 2]
    out["straight_2nd_diff_p90"] = sd[int(n * 0.9)]

    # 闭合口径（两种"缝隙"必须分清，否则会把正常的一步误判成误差）：
    #   1) 输入列表若**显式**把首点重复在末尾（高德原路、make_point_ring 的闭合形式），
    #      首尾距离恒为 0，闭合误差 = closure_error_m = 0；
    #   2) 若输入是**循环点列**（首尾不重复，humanize 的输出就是这种），最后一点回到第一点
    #      的那条弦就是环的最后一条边，它**不是**误差。判据是 seam_ratio = 弦长 / 相邻间距
    #      中位数：<= 2 视为正常的一步，远大于 2 才是真没接上。
    _raw = list(points_bd)
    if len(_raw) > 1:
        _rawP, _rc, _rs = _to_plane(_raw)
        out["closure_error_m"] = math.dist(_rawP[0], _rawP[-1])
    else:
        out["closure_error_m"] = 0.0
    _gaps = sorted(math.dist(Pp0[i], Pp0[(i + 1) % len(Pp0)]) for i in range(len(Pp0))) if len(Pp0) > 1 else []
    _med = _gaps[len(_gaps) // 2] if _gaps else 0.0
    _seam = math.dist(Pp0[0], Pp0[-1]) if len(Pp0) > 1 else 0.0
    out["seam_edge_m"] = _seam
    out["seam_gap_m"] = _seam          # 兼容旧调用方；单位已统一为**米**（旧版误把 BD 度当米）
    out["seam_median_seg_m"] = _med
    out["seam_ratio"] = (_seam / _med) if _med > 1e-9 else 0.0
    out["closure_ok"] = bool(out["closure_error_m"] <= 0.05 or (out["seam_ratio"] <= 2.0))

    try:
        from .route_metrics import route_overlap
        ov = route_overlap(list(points_bd), tolerance=1.5)
        out["repeat"] = {"length_m": ov["length_m"], "repeated_m": ov["repeated_m"],
                         "ratio": ov["repeat_ratio"]}
    except Exception as exc:                                   # pragma: no cover
        out["repeat"] = {"error": repr(exc)}

    if reference_bd is not None:
        # 口径：点到**参考折线段**的最短距离（不是到最近顶点），与"漂移出道路多远"同义。
        R = _cyclic(reference_bd)
        lat0 = R[0][0]
        m_lng = meters_per_deg_lng(lat0)
        m_lat = meters_per_deg_lat(lat0)
        Rm = [(q[0] * m_lat, q[1] * m_lng) for q in R]
        segs = []
        for i in range(len(Rm)):
            ax, ay = Rm[i]
            bx, by = Rm[(i + 1) % len(Rm)]
            ux, uy = bx - ax, by - ay
            L2 = ux * ux + uy * uy
            segs.append((ax, ay, ux, uy, L2))
        devs = []
        for p in P0:
            px, py = p[0] * m_lat, p[1] * m_lng
            best = 1e18
            for ax, ay, ux, uy, L2 in segs:
                if L2 <= 1e-12:
                    d = math.hypot(px - ax, py - ay)
                else:
                    t = ((px - ax) * ux + (py - ay) * uy) / L2
                    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                    d = math.hypot(px - (ax + t * ux), py - (ay + t * uy))
                if d < best:
                    best = d
            devs.append(best)
        devs.sort()
        m = len(devs)
        out["deviation_m"] = {"p50": devs[m // 2], "p95": devs[int(m * 0.95)],
                              "max": devs[-1]}
    return out
