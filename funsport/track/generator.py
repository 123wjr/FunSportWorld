"""自然轨迹生成器。"""
import bisect
import math

from . import realism
from .joint_controls import JointControls, LateralField
from .geom import (round_to, Rng, make_point_ring, ring_point_at, to_bd,
                   MET_PER_DEG_LAT, MET_PER_DEG_LNG, fmt_gain_time,
                   meters_per_deg_lat, meters_per_deg_lng,
                   SPEED_FLOOR, SPEED_CEIL)
from .pace import apply_location_paces
from ..coordinates import bd09_to_gcj02, distance_m
from ..logger import log, ok, dim


def fit_speeds(w, dts, target, floor=None, ceil=None):
    """在速度上下限内迭代匹配目标里程，原地更新每段速度。

    floor 默认是 SPEED_FLOOR(1.90)，但那是「平均配速合法性」的下限，不是
    单点的物理下限：真实数据里 p1=0.80 m/s、p5=1.99 m/s，确实存在走路和
    路口等待的采样点。用 1.90 逐点截断会把这类低速点全部抬回去，导致速度
    分布左尾消失（真实偏度 −0.73、生成 +0.19）。因此允许调用方传入更低的
    逐点下限，平均值仍由 SPEED_FLOOR/SPEED_CEIL 校验。
    """
    lo = SPEED_FLOOR if floor is None else floor
    hi = SPEED_CEIL if ceil is None else ceil
    # 收敛判据用相对误差而不是绝对 1m：采样点变密后每条 dt 更小，同样的
    # 绝对容差会留下 0.2m 级残差，与「totalDistance == 目标里程」的契约冲突。
    tol = max(1e-4, target * 1e-6)
    for _ in range(60):
        cur = sum(x * dt for x, dt in zip(w, dts))
        if abs(cur - target) <= tol:
            break
        k = target / cur
        for i in range(len(w)):
            w[i] = min(max(w[i] * k, lo), hi)
    for i in range(len(w)):
        w[i] = min(max(w[i], lo), hi)


def _fit_cadence(base, dts, target):
    """限定 60–220 spm，并按目标均值施加**对数域整体偏移**。

    target=0（默认）不做任何均值拟合：把时间加权均值强行拉回某个数会把
    幂律步频拉平，正是旧实现 corr(瞬时速度, 步频) 只有 ~0.23 的主因之一。

    target>0 时只施加一个指数偏移 exp(s)（等价于把整条幂律曲线在 log 域平移，
    即把参考步频 cad_ref 搬到 cad_ref·exp(s)）：单调、保序，
    dln(cad)/dln(v) 恒等于 realism.CADENCE_POWER_EXP 不被缩放破坏。

    旧实现在**夹紧之后**做标量缩放，且自己的下限写死 60：模型里 120spm 的
    慢行地板被乘成 90spm 后仍落在 60–220 内合法存活，于是出现「经验式地板被
    悄悄下移」的边界伪影。现在地板随偏移一起搬动
    （realism.CADENCE_FLOOR_SPM·exp(s)，并夹回 [60,220]）：偏移≈0 时模型地板
    原样保留，只有用户明确要求整体慢步频时地板才跟着下移。
    """
    if target == 0:
        return [realism.clip(c, 60.0, 220.0) for c in base]
    if target in (60, 220):
        return [float(target)] * len(base)
    duration = sum(dts)

    def mean_at(shift):
        floor = realism.clip(realism.CADENCE_FLOOR_SPM * math.exp(shift),
                             60.0, 220.0)
        k = math.exp(shift)
        return sum(realism.clip(c * k, floor, 220.0) * dt
                   for c, dt in zip(base, dts)) / duration

    lo, hi = -1.0, 1.0
    while mean_at(hi) < target:
        hi += 1.0
    while mean_at(lo) > target:
        lo -= 1.0
    for _ in range(48):
        mid = (lo + hi) / 2
        if mean_at(mid) < target:
            lo = mid
        else:
            hi = mid
    k = math.exp(hi)
    floor = realism.clip(realism.CADENCE_FLOOR_SPM * k, 60.0, 220.0)
    return [realism.clip(c * k, floor, 220.0) for c in base]


def _measure_coord(px, py, c_lat, c_lng):
    """平面坐标 → (落盘 BD 坐标, 验收口径的 GCJ-02 坐标)。

    速度约束必须和验收用同一把尺子，否则「浮点判过、落盘判不过」：
      · 落盘经纬度只有 6 位小数（realism.COORD_DECIMALS），一格 ~0.11m，
        落在几米量程的短采样段上就是百分级抬升；
      · 验收把落盘 BD 反算成 GCJ-02、四舍五入到 6 位，再用球面 Haversine
        算距离，而生成侧内部是「固定 m/度」的平面近似，两者系统差 0.2%~0.5%
        （实测同一段平面 13.150m / 验收 13.223m，差 0.55%，恰好把 6.60 顶到
        6.61）。所以度量一律走这条链：先量化 BD，再转 GCJ、再取 6 位。
    """
    lat, lng = to_bd(px, py, c_lat, c_lng)
    bd = (round_to(lat, realism.COORD_DECIMALS), round_to(lng, realism.COORD_DECIMALS))
    glat, glng = bd09_to_gcj02(*bd)
    return bd, (round_to(glat, realism.COORD_DECIMALS), round_to(glng, realism.COORD_DECIMALS))


class _GcjRoadTraversal:
    """GCJ length of the original ordered road, including partial/repeated edges.

    Road vertices and partial endpoints are unjittered and unquantized. BD
    progress only locates an endpoint; it is never substituted for GCJ metres.
    """

    def __init__(self, dense, arcs, center):
        self.dense, self.arcs, self.center = dense, arcs, center
        self.vertices = [bd09_to_gcj02(*to_bd(*point, *center)) for point in dense]
        self.lengths = [0.0]
        for a, b in zip(self.vertices, self.vertices[1:]):
            self.lengths.append(self.lengths[-1] + distance_m(a, b))

    def point(self, progress):
        xy = ring_point_at(self.dense, self.arcs, progress, open_path=True)
        return bd09_to_gcj02(*to_bd(*xy, *self.center))

    def distance(self, start, end):
        if end <= start:
            return 0.0
        first = min(bisect.bisect_right(self.arcs, start), len(self.arcs) - 1)
        last = min(bisect.bisect_left(self.arcs, end), len(self.arcs) - 1)
        a, b = self.point(start), self.point(end)
        if first >= last:
            return distance_m(a, b)
        return (distance_m(a, self.vertices[first]) +
                self.lengths[last - 1] - self.lengths[first] +
                distance_m(self.vertices[last - 1], b))

    def limit(self, start, budget):
        if self.distance(start, self.arcs[-1]) <= budget:
            return self.arcs[-1]
        low, high = start, self.arcs[-1]
        for _ in range(40):
            middle = (low + high) / 2.0
            if self.distance(start, middle) <= budget:
                low = middle
            else:
                high = middle
        return low


def _sampled_gcj_brackets(dense, arcs, center, start, end, jitter):
    """Visit every stored-BD quantization cell in original traversal order.

    Chord length need not be monotone across a bend (or a quantization corner).
    Splitting at every source vertex and BD grid boundary avoids skipping an
    interior maximum just because both ends of a coarse bracket are shorter.
    These are search evaluations, never additional output samples.
    """
    low = start
    scale = 10 ** realism.COORD_DECIMALS
    stops = [s for s in arcs[bisect.bisect_right(arcs, start):] if s < end] + [end]
    for high in stops:
        if high <= low:
            continue
        a, b = [to_bd(x + jitter[0], y + jitter[1], *center)
                for x, y in (ring_point_at(dense, arcs, low, open_path=True),
                             ring_point_at(dense, arcs, high, open_path=True))]
        boundaries = {high}
        for left, right in zip(a, b):
            left, right = left * scale, right * scale
            if right == left:
                continue
            first = math.floor(min(left, right) - 0.5) + 1
            last = math.ceil(max(left, right) - 0.5)
            if last - first > 8192:
                raise ValueError("Sampled GCJ quantized forward search exceeded its bounded budget")
            for cell in range(first, last):
                fraction = (cell + 0.5 - left) / (right - left)
                if 0.0 < fraction < 1.0:
                    boundaries.add(low + (high - low) * fraction)
        previous = low
        for boundary in sorted(boundaries):
            yield (previous + boundary) / 2.0
            yield boundary
            previous = boundary
        low = high


def _sampled_gcj_plan(dense, arcs, center, speeds, dts, jit_x, jit_y, target_m, lateral_field=None):
    """Fixed draws; bound both stored-fix chords and original GCJ road traversal.

    Seek cumulative output-chord targets only within this interval's existing
    internal speed guard. Visit at most 8192 quantization/vertex candidates
    per interval. No regeneration, wrap, added fixes, or invented road.
    Quantization prevents exact equality; final error must be <= 0.25 m.
    This guard is not a user local-pace acceptance criterion.
    """
    road = _GcjRoadTraversal(dense, arcs, center)
    road_steps = []
    progress = measured = requested = 0.0
    first_bd = tuple(round_to(v, realism.COORD_DECIMALS) for v in to_bd(*dense[0], *center))
    previous = bd09_to_gcj02(*first_bd)
    if lateral_field is not None:
        # The immutable origin anchor must also fit the final quantized envelope.
        # Infeasible sub-grid envelopes fail; the anchor is never relocated.
        lateral_field.sample(0.0, (0.0, 0.0), center)
    plan = []
    evaluations = 0
    for index, (speed, dt) in enumerate(zip(speeds, dts)):
        requested += speed * dt
        want = requested - measured
        jitter = (0.0, 0.0) if index == len(dts) - 1 else (jit_x[index], jit_y[index])
        # A fix must advance along the road, rather than spending its distance
        # budget only on a fresh jitter draw. Bound both adjacent fix offsets
        # by 10% of this interval (look ahead for the next interval as well).
        next_step = speeds[index + 1] * dts[index + 1] if index + 1 < len(dts) else speed * dt
        radius = 0.1 * min(speed * dt, next_step)
        magnitude = math.hypot(*jitter)
        if magnitude > radius:
            jitter = tuple(value * radius / magnitude for value in jitter)

        def sample(s):
            nonlocal evaluations
            evaluations += 1
            # The audit persists six-decimal BD progress; guard that exact
            # progress too, rather than a nearby higher-precision endpoint.
            s = round_to(s, 6)
            if lateral_field is None:
                bx, by = ring_point_at(dense, arcs, s, open_path=True)
                px, py = bx + jitter[0], by + jitter[1]
                bd, _ = _measure_coord(px, py, *center)
                gcj = bd09_to_gcj02(*bd)
            else:
                bx, by, px, py, gcj = lateral_field.sample(s, jitter, center)
            return (s, bx, by, px, py, gcj, distance_m(previous, gcj))

        budget = realism.SPEED_CEIL_POINT * dt
        road_end = road.limit(progress, budget)
        end = math.floor(road_end * 1e6) / 1e6
        if sample(progress)[-1] >= want:
            raise ValueError("Sampled GCJ jitter exhausted the forward progress budget")
        found = best = None
        # Every candidate stays within the *GCJ road* budget, not a BD-progress
        # proxy. Examine quantized cells in traversal order: chord distance can
        # decrease across a bend, including a legitimate source-road revisit.
        brackets = (_sampled_gcj_brackets(dense, arcs, center, progress, end, jitter)
                    if lateral_field is None else lateral_field.brackets(progress, end, center))
        for count, high in enumerate(brackets):
            if count >= 8192:
                raise ValueError("Sampled GCJ quantized forward search exceeded its bounded budget")
            candidate = sample(high)
            if candidate[0] <= progress or candidate[0] > end or candidate[-1] > budget:
                continue
            if best is None or candidate[-1] > best[-1]:
                best = candidate
            if candidate[-1] >= want:
                found = candidate
                break
        # A quantization cell may straddle either speed guard. Carry only a
        # small output-chord residual; never borrow traversal time or road.
        if found is None and best is not None and abs(best[-1] - want) <= 0.25:
            found = best
        if found is None:
            reason = ("more mapped forward road" if road_end == arcs[-1] else
                      "a feasible forward sample within the GCJ road traversal speed budget")
            raise ValueError("Sampled GCJ distance requires " + reason)
        traversed = road.distance(progress, found[0])
        if (found[0] <= progress or found[-1] > budget or traversed > budget):
            raise ValueError("Sampled GCJ progress exceeds the interval speed bound")
        road_steps.append(traversed)
        progress, _, _, _, _, previous, step = found
        measured += step
        plan.append(found)
    if abs(measured - target_m) > 0.25:
        raise ValueError("Sampled GCJ distance did not meet the quantized tolerance")
    return plan, evaluations, road_steps


def _make_dips(rng, dur):
    """生成速度曲线的缓慢降速区间，不代表App暂停事件。"""
    dips = []
    for _ in range(rng.choice([1, 1, 2])):
        dips.append((rng.uniform(0.15, 0.75) * dur,
                     rng.uniform(25, 55), rng.uniform(0.08, 0.16)))
    return dips


def _dip_factor(tt, dips):
    """计算指定时刻在平滑降速区间中的速度倍率。"""
    f = 1.0
    for c, hw, d in dips:
        if abs(tt - c) < hw:
            f *= 1.0 - d * 0.5 * (1 + math.cos(math.pi * (tt - c) / hw))
    return f


def _elevation_at(arcs, heights, distance, open_path=False):
    """按路径弧长插值；重复绕圈返回相同地点的高程。"""
    s = min(max(distance, 0.0), arcs[-1]) if open_path else distance % arcs[-1]
    j = min(max(bisect.bisect_right(arcs, s), 1), len(arcs) - 1)
    span = arcs[j] - arcs[j - 1]
    f = (s - arcs[j - 1]) / span if span else 0.0
    return heights[j - 1] + f * (heights[j] - heights[j - 1])


def build(dist, dur, seed, start_ms, points_bd,
          ordered_path=False, cadence_target=0, checkpoints_bd=None, elevations_m=None,
          sample_period_s=2, open_path=False, distance_mode="arc", joint_controls=None):
    """沿路径生成轨迹；有序规划不随机伪造无效、越界或打卡点类型。

    cadence_target: 目标步频 spm（0 = 自动按速度推算）
    checkpoints_bd: 需要精确命中的打卡点（BD 坐标）；只对这些点做吸附
    elevations_m: 与有序路径顶点一一对应的可信高程（m）；二维路线不推测坡度
    sample_period_s: 设备逐点记录的主导采样周期（1s 或 2s；默认旧版 2s）
    open_path: 有序道路只走到末端；超过可用弧长时拒绝生成
    distance_mode: arc 保留旧弧长契约；sampled_gcj 按落盘 BD 转 GCJ 后的
        相邻折线累计校正开放道路进度（误差 <= 0.25m），需要整秒时长。
    """
    if distance_mode not in ("arc", "sampled_gcj"):
        raise ValueError("Unknown path distance mode")
    sampled_gcj = distance_mode == "sampled_gcj"
    if sampled_gcj and (not isinstance(dur, (int, float)) or not math.isfinite(dur)
                        or dur != round(dur)):
        raise ValueError("Sampled GCJ distance requires an integer-second duration")
    if sampled_gcj and (not open_path or not ordered_path or checkpoints_bd):
        raise ValueError("Sampled GCJ distance requires an unsnapped ordered open road")
    if open_path and not ordered_path:
        raise ValueError("开放路径必须使用有序道路折线")
    if (not isinstance(cadence_target, (int, float)) or
            not math.isfinite(cadence_target) or
            (cadence_target != 0 and not 60 <= cadence_target <= 220)):
        raise ValueError("目标步频须为 60–220 spm，或 0 表示自动估算")
    if elevations_m is not None and (not ordered_path or len(elevations_m) != len(points_bd)
                                  or not all(isinstance(h, (int, float)) and math.isfinite(h)
                                             for h in elevations_m)):
        raise ValueError("路线高程须与有序路径顶点一一对应，且为有限值")
    if joint_controls is not None:
        if not isinstance(joint_controls, JointControls):
            raise ValueError("joint_controls must be a JointControls object")
        joint_controls.validate()
        if not ordered_path or not sampled_gcj:
            raise ValueError("joint controls require ordered_path and sampled_gcj")
    rng = Rng(seed)
    # Versioned optional domains never draw a seed from the legacy stream.
    speed_rng = Rng(f"joint-controls:v1:speed:{seed}") if joint_controls is not None else None
    cadence_rng = Rng(f"joint-controls:v1:cadence:{seed}") if joint_controls is not None else None
    log.info(f"[track] 生成 {dist:.0f}m / {dur}s seed={seed} "
             f"点位={len(points_bd)} 模式={'真实路径' if ordered_path else '拟合环'}"
             f" 步频={cadence_target:.0f}spm" if cadence_target > 0 else "")

    dense, arcs, (c_lat, c_lng) = make_point_ring(points_bd, ordered=ordered_path, open_path=open_path)
    ring_len = arcs[-1] if arcs else 0.0
    if ring_len <= 0 or dur <= 0 or dist <= 0:
        raise ValueError("路线长度、目标里程和时长必须为正数")
    if open_path and dist > ring_len:
        raise ValueError("开放道路折线不足规划里程；须取得高德后续道路")
    heights = list(elevations_m) if elevations_m is not None else None
    if heights is not None and not open_path and len(arcs) == len(heights) + 1:
        heights.append(heights[0])
    if heights is not None and (len(heights) != len(arcs) or
                                (dense[0] == dense[-1] and
                                 abs(heights[0] - heights[-1]) > 0.1)):
        raise ValueError("闭环高程须与路径顶点对齐且起终点一致")

    if ordered_path:
        if not SPEED_FLOOR <= dist / dur <= SPEED_CEIL:
            raise ValueError("规划里程与时长超出生成器的速度范围")
        direction = 1.0
        s0 = 0.0
        s_limit = dist
        jitter_sigma = 0.22
        jitter_ar = 0.45
        log.info(f"[track] 环长 {ring_len:.0f}m，只跑前 {dist:.0f}m")
    else:
        direction = rng.choice([1.0, -1.0])
        s0 = rng.uniform(0, ring_len)
        s_limit = float("inf")
        jitter_sigma = 0.75
        jitter_ar = 0.72

    phase_v = rng.uniform(0, math.tau)
    phase_l = rng.uniform(0, math.tau)

    # 默认沿用旧华为子集的 2s 采样模式；Keep 的 1s 阶段尚未分层拟合。
    times = realism.sample_interval_series(dur, rng, sample_period_s=sample_period_s)
    n_max = len(times)

    dips = _make_dips(rng, dur)
    base = dist / dur

    # 速度过程用 AR(1) 对数速度，复现真实速度序列的自相关
    # （真实 dt=2s 时 lag-1 为 0.38~0.81，纯白噪声只能给出 0）。
    w = realism.ar1_log_speed(times, base, rng)
    finish = realism.finish_deceleration(times, dur)
    walk = realism.walk_events(n_max, rng)

    # 末段就是「最后一个采样点到 dur」的真实间隔：可能不足 1s（非整秒收尾），
    # 但不能像旧实现那样下限抬到 1s —— 那会让 Σdt > dur，把总时长篡改掉。
    dts = [times[i + 1] - times[i] for i in range(n_max - 1)]
    dts.append(max(0.0, dur - times[-1]))
    total_dt = sum(dts)

    # ── 速度包络：解析式 + 包络内归一化 ──────────────────────────
    # 起跑惯性 ramp(t) = R0 + (1−R0)·(1−exp(−t/τ))，τ≈4s（文献 §2.5 一阶惯性）
    # 疲劳     f(d)    = 1 − A·(1−exp(−d/D))，A=0.12、D=0.5·全程（文献 §2.2）
    # 两项都全程单调平滑、没有平台段。归一化（时间加权均值 = 1）必须发生在
    # 包络内部，不能靠 fit_speeds 事后把整体均值拉回来：旧实现的 ramp 最低只
    # 到 0.85、疲劳又在前 500m 抬到 1.03，fit_speeds 为补回被扣掉的里程把全段
    # 一起抬高，于是起步 600m 峰值被放大到 4.271m/s（1.41× 均速）；疲劳项
    # max(1.03−0.06·(km−0.5), 0.80) 在 4.3333km 撞底后完全平坦、撞底处斜率突变。
    envelope = []
    for i, tt in enumerate(times):
        inertia = (realism.START_INERTIA_RATIO +
                   (1.0 - realism.START_INERTIA_RATIO) *
                   (1.0 - math.exp(-tt / realism.START_INERTIA_TAU)))
        fatigue = 1.0 - realism.FATIGUE_AMP * (
            1.0 - math.exp(-tt / max(1e-9, dur * realism.FATIGUE_DIST_FRAC)))
        wave = (1.0
                + 0.010 * math.sin(math.tau * tt / 115.0 + phase_v)
                + 0.035 * math.sin(math.tau * tt / 47.0 + phase_v * 2.3)
                + 0.015 * math.sin(math.tau * tt / 19.0 + phase_v * 3.7)) * _dip_factor(tt, dips)
        envelope.append(inertia * fatigue * wave * finish[i] * walk[i])
    env_mean = sum(e * dt for e, dt in zip(envelope, dts)) / max(1e-9, total_dt)
    if env_mean > 0:
        w = [speed * env / env_mean for speed, env in zip(w, envelope)]

    # ── 波动幅度归一（log 域等比缩放）───────────────────────────
    # 真实 Keep 逐点 cv≈0.21（对数稳态 sd≈0.2077）。AR 过程本身给 0.187，
    # 再叠上正弦/降速/步行/末段后实测被抬到 cv 0.236、极差 0.93~4.64m/s。
    # 把合成后的对数偏差整体缩放到标定 sd：点的排序、lag 自相关、与步频的
    # 单调关系全部不变，只有幅度被压回经验值。可选 joint_controls 的残差在
    # 这一步之后才乘上去，因此不会吞掉用户显式要求的额外波动。
    logs = [math.log(max(1e-9, speed)) for speed in w]
    log_mean = sum(logs) / max(1, len(logs))
    log_sd = math.sqrt(sum((x - log_mean) ** 2 for x in logs) / max(1, len(logs)))
    if log_sd > 1e-9:
        gain = realism.clip(realism.SPEED_LOG_SD_TARGET / log_sd, 0.40, 1.60)
        w = [math.exp(log_mean + gain * (x - log_mean)) for x in logs]

    if joint_controls is not None:
        for i, tt in enumerate(times):
            transient = (joint_controls.start_transient * max(0.0, 1.0 - tt / max(1.0, dur * 0.2)) -
                         joint_controls.finish_transient * max(0.0, (tt / max(1.0, dur) - 0.8) / 0.2))
            w[i] *= 1.0 + transient

    grades = None
    if heights is not None:
        # 用预计路径位置处的坡度调节速度分配，之后重新满足目标总里程。
        # 系数来自 realism.grade_speed_factor = 1/PACF：上坡 Minetti 2002
        # Cr(i)/Cr(0)，下坡 Strava 二次式 1+15.14i²+2.71i。旧的 exp(-2.0·grade)
        # 没有任何出处，且强度只有文献的 38%（+1% 坡度只给 −1.98%，Minetti 是
        # −5.25%、Strava 表是 −2.8%）；+5% 由 −9.5% 改为 −23.2%。
        cumulative = 0.0
        rough_distance = sum(speed * dt for speed, dt in zip(w, dts))
        grades = [0.0] * len(dts)
        for i, dt in enumerate(dts):
            ds = dist * w[i] * dt / rough_distance
            if ds > 0:
                grade = (_elevation_at(arcs, heights, cumulative + ds, open_path=open_path) -
                         _elevation_at(arcs, heights, cumulative, open_path=open_path)) / ds
                grades[i] = grade
                w[i] *= realism.grade_speed_factor(grade)
            cumulative += ds
    fit_speeds(w, dts, dist, floor=realism.SPEED_FLOOR_POINT,
               ceil=realism.SPEED_CEIL_POINT)
    # ── 低速持续性约束 ─────────────────────────────────────────
    # 低于 LOW_SPEED_RATIO×当前均速的采样点最多连续 LOW_SPEED_RUN_MAX 个。真实数据
    # 里连续多点低速只来自「终点停表」和「定位丢失」；用户反馈的「异常的低值
    # 持续」正指这类段。超出的点从门槛值起在对数域线性抬升到段后第一个正常点，
    # 既不断档也不生成平台。
    # 必须放在 fit_speeds 之后：fit_speeds 会整体缩放，缩放前满足的门槛相对关系
    # 会被它重新打破（实测 1/2/3 圈分别出现 2/4/3 连段）。抬升后整体均匀缩回目标
    # 里程——均匀缩放同时缩放均速与门槛，所以约束在缩放后仍然成立。
    def _cap_low_runs(values, gate, max_run):
        out = list(values)
        k = 0
        while k < len(out):
            if out[k] >= gate:
                k += 1
                continue
            end = k
            while end < len(out) and out[end] < gate:
                end += 1
            if end - k > max_run:
                tail = k + max_run
                resume = out[end] if end < len(out) else gate * 1.10
                span = end - tail
                for m in range(tail, end):
                    frac = (m - tail + 1) / span
                    out[m] = gate * (resume / gate) ** frac
            k = end
        return out

    speed_mean = sum(x * dt for x, dt in zip(w, dts)) / max(1e-9, total_dt)
    cap_gate = (realism.LOW_SPEED_RATIO * realism.LOW_SPEED_GATE_MARGIN *
                speed_mean)
    w = _cap_low_runs(w, cap_gate, realism.LOW_SPEED_RUN_MAX)
    capped_total = sum(x * dt for x, dt in zip(w, dts))
    if capped_total > 0:
        restore = dist / capped_total
        w = [realism.clip(x * restore, realism.SPEED_FLOOR_POINT,
                          realism.SPEED_CEIL_POINT) for x in w]
    if ordered_path and abs(sum(speed * dt for speed, dt in zip(w, dts)) - dist) > 1.0:
        raise ValueError("速度拟合未达到规划里程，停止生成不完整路线")
    dim(f"速度拟合：{n_max} 点 平均 {dist/dur:.2f} m/s")
    # 步频用瞬时速度代入经验幂律 171.43·(v/3)^0.28（realism.cadence_for_speed）。
    # 经验式本身就是在同一采样区间上标定的短窗关系（Keep 的累计步数/距离短窗），
    # 所以这里直接对区间自身速度求值，不再套 15s（或任何一阶）滞后：实测一阶
    # 滞后会把 dln(cad)/dln(v) 从 0.28 压到 τ=3s 的 0.21、τ=15s 的 0.16，并让
    # 落盘步长系统性偏离 1.05·(v/3)^0.72。
    # 注意：cadence_noise 的 rng 抽取必须保持位置与次数不变——它是 legacy 流，
    # tests/test_track_joint_controls.py 会逐项比对 rng 状态（cadence-only 参数
    # 只允许影响 steps）。
    cadence_base = []
    control_speed = w[0]
    cadence_residual = 0.0
    cadence_noise = 0.0
    for _i, (v, dt) in enumerate(zip(w, dts)):
        cadence_noise = (0.75 * cadence_noise +
                         math.sqrt(1.0 - 0.75 ** 2) * rng.gauss(0.0, 3.0))
        response = realism.cadence_for_speed(v)
        if grades is not None:
            # 坡度→步频的额外耦合。realism.GRADE_CADENCE_EXTRA_PER_UNIT 默认 0，
            # 该因子恒等于 1.0（x*1.0 == x，逐点不变），即坡度只通过速度影响步频。
            # 保留接线是因为华为数据在固定速度下的偏斜率（−1.99%/1%）全部来自
            # 3/65 条陡记录，不足以支撑非零系数；一旦有更好的数据，改这个常量
            # 即可生效。此处不消耗 rng，legacy 抽取流不受影响。
            response *= realism.grade_cadence_factor(grades[_i])
        if joint_controls is not None:
            tau_c = joint_controls.cadence_tau_s
            c_alpha = 1.0 - math.exp(-dt / tau_c)
            control_speed += c_alpha * (v - control_speed)
            response += joint_controls.cadence_slope_spm_per_mps * (control_speed - base)
            if joint_controls.cadence_residual_spm:
                ar = math.exp(-dt / max(1.0, tau_c))
                cadence_residual = ar * cadence_residual + math.sqrt(1.0 - ar * ar) * cadence_rng.gauss(0.0, joint_controls.cadence_residual_spm)
            response += cadence_residual
        cadence_base.append(response + cadence_noise)
    cadence_series = _fit_cadence(cadence_base, dts, cadence_target)

    kinds = []
    for _ in range(n_max):
        if ordered_path:
            # 规划折线没有真实定位异常或打卡事件，普通点必须使用type=0。
            kinds.append((0, 1))
            continue
        u = rng.random()
        if u < 0.39:
            kinds.append((3, 1))
        elif u < 0.93:
            kinds.append((0, 1))
        elif u < 0.96:
            kinds.append(rng.weighted([((-1, 4), 54), ((-1, 1), 27),
                                       ((-1, 12), 13), ((-1, 5), 3),
                                       ((-1, 6), 2)]))
        else:
            kinds.append((rng.choice([1, 1, 1, 1, 2, 2]), 1))
    for i in range(1, n_max):
        if kinds[i][0] == -1 and kinds[i - 1][0] == -1:
            kinds[i] = (rng.choice([3, 0]), 1)

    # 坐标与弧长必须同口径：locs[k] 记录第 k 段间隔「开始时」的路径点，对应
    # totalDis[k] 也是这段开始时的累计里程。旧实现先推进 s 再取坐标，坐标因此
    # 比 totalDis 领先一个采样步：普通 2s 段只差 0.2%，看不出问题；但采样间隔
    # 被抖动/超速钳制压到 0.5~1.5s 时，一个完整步长的错位足以把坐标反算速度
    # 顶到 6.6 m/s 以上（实测 seed18 第 7 点 6.617，声明弧长只有 7.13m/2s）。
    locs = []
    s = s0
    bx, by = ring_point_at(dense, arcs, s, open_path=open_path)
    t_acc = dist_acc = steps_acc = 0.0
    jx = jy = 0.0
    alt = 6.0 + rng.uniform(-1, 1)
    if heights is not None:
        distance_marks = []
        progress = 0.0
        for v, dt in zip(w, dts):
            progress += v * dt
            distance_marks.append(min(progress, dist))
        source_elevation = [_elevation_at(arcs, heights, s, open_path=open_path) for s in distance_marks]
    else:
        source_elevation = [alt] * n_max
    elev_series = realism.quantize_elevation(source_elevation, rng)
    n_est = max(1, int(dur / 5))
    alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est)
    # 定位抖动：重尾分布（小基底 + 稀有跳点），不是单一高斯。见 realism 注释。
    jit_x = realism.jitter_series(n_max, rng)
    jit_y = realism.jitter_series(n_max, rng)
    lateral_field = (LateralField(dense, arcs, joint_controls)
                     if joint_controls is not None and
                     (joint_controls.lateral_mean_m or joint_controls.lateral_amplitude_m) else None)
    sampled_plan, distance_evaluations, road_steps = (
        _sampled_gcj_plan(dense, arcs, (c_lat, c_lng), w, dts, jit_x, jit_y, dist,
                          lateral_field=lateral_field)
        if sampled_gcj else (None, 0, None))
    if sampled_gcj and heights is not None:
        elev_series = realism.quantize_elevation(
            [_elevation_at(arcs, heights, row[0], open_path=True) for row in sampled_plan], rng)
    # 航向安装角：手机在臂包/腰包里的固定偏置（文献 5~15°），全程不变。
    mount_offset = rng.uniform(*realism.BDD_MOUNT_OFFSET) * rng.choice([1.0, -1.0])
    # bx/by 只在「有效点」分支里赋值；无效点（type=-1）沿用的是上一个有效点
    # 的位置。旧版非有序模式若第 0 个点就是无效点，此处需要有初值。
    bx = by = 0.0
    prev_bx = prev_by = None
    prev_px = prev_py = None
    prev_qxy = None
    prev_clock = None
    last_heading = 0.0
    prev_heading_for_gyro = None

    for i in range(n_max):
        dt = dts[i]
        typ, lt = kinds[i]
        t_acc += dt
        # 落盘时钟是整秒（totalTime = int(round(t_acc))），验收也用这个整秒差
        # 当分母。dts 里的间隔可以是小数（非整秒尾巴），若按浮点 dt 判速限就会
        # 「浮点 6.60 通过、落盘按整秒算成 6.604」——实测折角环 dur=401 的
        # seed 7/42 各有一处 6.604。因此限速一律用落盘时钟的整数间隔。
        tt_clock = int(round(t_acc))
        dt_clock = tt_clock - prev_clock if prev_clock is not None else 0
        d_step = 0.0
        if typ != -1:
            # 逐点里程只由速度×间隔给出：不再在末点把位移强行拉满到规划里程。
            # 旧实现在末点令 d_step = s_limit - (s - s0)，配合 25~150s 的采样
            # 空档，等于让最后一个点从几百米外瞬移到路径终点，坐标反算里程比
            # 声明里程短 7%~27%，且末点速度物理上不可达。里程由 fit_speeds 在
            # 全程内摊平，末点不需要特殊照顾。
            d_step = w[i] * dt
            if ordered_path and not sampled_gcj and (s - s0 + d_step) > s_limit:
                d_step = max(0.0, s_limit - (s - s0))
            jx, jy = jit_x[i], jit_y[i]
            # 末点不加抖动：ordered_path 下终点坐标随后会被重算成弧长对应的
            # 真实路径点，若这里带抖动就变成「末点从抖动位置弹回路径点」，
            # 在不足 1s 的末段上会算出十几 m/s 的伪速度。
            if ordered_path and i == n_max - 1:
                jx = jy = 0.0
            s += direction * d_step
            # locs[k] 的 totalDis[k] 与 gainTime 都是「本段走完」的量，坐标因此
            # 也取本段末的路径点，三者同口径。
            bx, by = ring_point_at(dense, arcs, s, open_path=open_path)
            px, py = bx + jx, by + jy
            if sampled_gcj:
                s, bx, by, px, py, _, d_step = sampled_plan[i]
            # 抖动叠在路径位移上会造出物理上不存在的瞬时速度：真实设备逐点最高
            # 6.31 m/s，米级跳点叠 2s 采样可以给出 7 m/s 以上（实测 30 例最大
            # 7.17）。超限时把本点沿「上一点→本点」方向按比例收回，只收缩不重新
            # 抽样，因此同 seed 仍可复现。
            #
            # 关键：判据必须用**验收同口径**（见 _measure_coord）：只在浮点位置上
            # 判 6.6 会同时漏掉 6 位小数量化和平面/Haversine 两处偏差，实测
            # 4/300 行落盘后算成 6.605~6.621。这里量完超限就按比例收到 limit
            # 以内，收回量级 ~0.1m，不构成会累积的位置滞后。
            # 收缩目标要留一个量化格的余量：落盘坐标是 6 位小数的格子点，
            # 只按 limit/used 收 0.01m 这种不足半格的量时，重新量化会四舍五入
            # 回同一个格子点，迭代 8 次仍原地不动（实测 20261009/seed22 收到
            # 6.605 就卡死）。留 0.15m 余量才保证换到更近的格子点。
            _, qxy = _measure_coord(px, py, c_lat, c_lng)
            if not sampled_gcj and prev_qxy is not None and dt_clock > 0:
                limit = realism.SPEED_CEIL_POINT * dt_clock
                target = limit - min(0.15, 0.25 * limit)
                for _ in range(8):
                    used = distance_m(prev_qxy, qxy)
                    if used <= limit:
                        break
                    k = max(0.0, min(1.0, target / used))
                    px = prev_px + (px - prev_px) * k
                    py = prev_py + (py - prev_py) * k
                    _, qxy = _measure_coord(px, py, c_lat, c_lng)
            prev_px, prev_py = px, py
            prev_qxy = qxy
            prev_clock = tt_clock
            dist_acc += d_step
        else:
            if prev_px is not None:
                # 无效点沿用上一个有效点的**本地**坐标。旧实现按落盘 BD 坐标
                # 反算回米制再重新落盘，往返换算把这个点挪出 7.5m（实测
                # 20260928/seed2 反算出 7.55 m/s）；首点即无效时更取全局原点，
                # 首点与第 1 个有效点相距 211.8m（105.9 m/s）。
                px, py = prev_px, prev_py
            else:
                px, py = ring_point_at(dense, arcs, s, open_path=open_path)
            # 无效点也参与验收的真实时钟：验收按相邻两点的坐标距离 / 时间差算
            # 速度，因此这里同步速度基准，避免把停滞后累积的步长算进一个间隔。
            _, qxy = _measure_coord(px, py, c_lat, c_lng)
            prev_px, prev_py = px, py
            prev_qxy = qxy
            prev_clock = tt_clock

        lat, lng = to_bd(px, py, c_lat, c_lng)
        # 高程与当前弧长同源；二维路线只能保留设备基线，不能声称有坡度。
        alt = elev_series[i]

        # sampled_gcj 的 d_step 是输出采样弦长；步长/速度字段沿用此口径，
        # 不声称同时匹配原道路遍历里程。累计步数仍由步频和时间积分。
        cad = cadence_series[i]
        if typ == -1:
            # 无效定位不等于停止迈步；旧拟合模式仍按经过的时间计步。
            d_steps = cad * dt / 60.0
        else:
            stride = realism.stride_for_speed(d_step / dt, cad) if dt else 0.0
            d_steps = d_step / stride if stride else 0.0
        steps_acc += d_steps

        if typ == -1:
            avg_sp = round_to(dist_acc / max(1, t_acc), 4)
        else:
            avg_sp = round_to(d_step / dt, 4) if dt > 0 else 0.0

        # ── 传感器字段由轨迹物理推导，而不是独立随机数 ──
        # bdD：设备航向 = 运动方向 + 安装偏置 + 噪声，由相邻两点位移算出。
        if prev_bx is not None and (bx != prev_bx or by != prev_by):
            heading = math.degrees(math.atan2(bx - prev_bx, by - prev_by)) % 360.0
        else:
            heading = last_heading
        last_heading = heading
        prev_bx, prev_by = bx, by
        bdd_value = round_to((heading + mount_offset + rng.gauss(0, realism.BDD_NOISE_SIGMA)) % 360, 2)
        # bdS：加速度计模长。静止/匀速时读到重力 9.81，跑步时叠加步态振荡
        # a_rms = A(2πf)²/√2，因此是 |(g, a_rms)| 而非速度的线性函数。
        cad_now = cad if typ != -1 else 0.0
        acc_rms = realism.accel_rms(max(0.0, avg_sp), cad_now)
        bds_value = round_to(math.hypot(realism.GRAVITY, acc_rms) + rng.gauss(0, 0.15), 3)
        # bdG：陀螺仪某一轴读数，转弯时角速度 ω = v/R 明显增大。
        # 用相邻两点航向变化率近似，符号跟随转向。
        if prev_heading_for_gyro is not None and dt > 0:
            dh = ((heading - prev_heading_for_gyro + 180.0) % 360.0) - 180.0
            omega = dh / dt
        else:
            omega = 0.0
        prev_heading_for_gyro = heading
        bdg_value = int(round_to(math.copysign(1.0, omega), 0)) if abs(omega) >= 3.0 \
            else rng.choice([1, 1, -1])

        locs.append({
            "id": i + 1, "flag": start_ms,
            "lat": -1.0, "lng": -1.0,
            "gLat": round_to(lat, realism.COORD_DECIMALS), "gLng": round_to(lng, realism.COORD_DECIMALS),
            "speed": 0.0, "avgSpeed": 0.0,
            "radius": round_to(rng.uniform(1.4, 5.1) if typ == 3 else rng.uniform(1.4, 2.4), 2),
            "accuracy": round_to(rng.uniform(1.4, 2.4), 2),
            "type": typ, "locType": lt,
            "hasAltitude": True,
            "totalTime": int(round(t_acc)),
            "totalDis": round_to(dist_acc, 4),
            "validDis": round_to(dist_acc, 4),
            "validTime": int(round(t_acc)),
            "steps": int(steps_acc),
            # 本段平均步长 = 段内位移 / 段内步数。旧实现恒写 0.0，等于把这个
            # 字段拆成"只能由外部反算"的空壳；步数与位移都已在上面算好，直接相除。
            "stepDistance": round_to(d_step / d_steps, 4) if d_steps > 0 else 0.0,
            "gainTime": fmt_gain_time(start_ms + int(t_acc * 1000)),
            "gainTimeMs": start_ms + int(t_acc * 1000),
            "queueNum": 0, "coorType": "gcj02",
            "bdA": round_to(alt, 2),
            "bdD": bdd_value,
            "bdS": bds_value,
            "bdG": bdg_value,
            "count": rng.randint(20, 88), "dtr": 0.0,
            "state": rng.weighted([(1, 145), (2, 256), (3, 151)]) if typ != 0
                     else rng.weighted([(1, 145), (2, 45), (3, 164)]),
            "locationId": "",
        })

        if sampled_gcj:
            locs[-1]["pathProgressM"] = round_to(s, 6)
        if ordered_path and not sampled_gcj and (s - s0) >= s_limit - 1e-6:
            break

    from .postfix import apply_post_fixes
    apply_post_fixes(locs, rng, start_ms, ordered_path=ordered_path)

    # ── 打卡点吸附：只吸附真正的打卡点 ──
    # 旧实现遍历 points_bd 的每一个顶点（ordered_path 下就是整条高德折线的
    # 全部顶点），把每个顶点最近的采样点直接改写过去。这会打乱采样间距：
    # 实测 120×80m 环上速度被放大到 1.48~6.49 m/s（声明 2.50），
    # |a| 峰值从 0.42 升到 4.57 m/s²；顶点越密越糟（5m 间距时 34/34 全被吸附）。
    # 采样点本来就落在 route 折线上，不需要再拉向顶点，因此只处理打卡点。
    if checkpoints_bd:
        snap_radius = 20.0 if ordered_path else 40.0
        for pl in checkpoints_bd:
            best_i, best_d = None, 1e18
            for i, q in enumerate(locs):
                dd = ((q["gLat"] - pl[0]) * MET_PER_DEG_LAT) ** 2 + \
                     ((q["gLng"] - pl[1]) * MET_PER_DEG_LNG) ** 2
                if dd < best_d:
                    best_d, best_i = dd, i
            if best_i is not None and best_d < snap_radius * snap_radius:
                locs[best_i]["gLat"] = round_to(pl[0], realism.COORD_DECIMALS)
                locs[best_i]["gLng"] = round_to(pl[1], realism.COORD_DECIMALS)

    speed_win, steps_win = [], []
    ten_t = ten_d = ten_st = 0.0
    n_used = len(locs)
    for i in range(n_used):
        ten_t += dts[i] if i < len(dts) else 0.0
        if ordered_path:
            ten_d += locs[i]["totalDis"] - (locs[i - 1]["totalDis"] if i else 0.0)
        else:
            ten_d += w[i] * (dts[i] if i < len(dts) else 0.0)
        ten_st += (locs[i]["steps"] - (locs[i - 1]["steps"] if i else 0))
        while ten_t >= 10.0:
            k = 10.0 / ten_t
            out_d, out_st = ten_d * k, ten_st * k
            speed_win.append({"time": 10, "value": round_to(out_d, 2)})
            steps_win.append({"time": 10, "value": round_to(out_st, 0)})
            ten_t -= 10.0
            ten_d -= out_d
            ten_st -= out_st
    if ordered_path and ten_t > 0:
        speed_win.append({"time": ten_t, "value": round_to(ten_d, 2)})
        steps_win.append({"time": ten_t, "value": round_to(ten_st, 0)})
    elif ten_t > 1.0:
        k = min(10.0 / ten_t, 1.4)
        speed_win.append({"time": 10, "value": round_to(ten_d * k, 2)})
        steps_win.append({"time": 10, "value": round_to(ten_st * k, 0)})

    total_dis_actual = dist_acc
    total_t_actual = int(round(t_acc))
    if ordered_path and locs:
        # 起点单独占用t=0；终点沿路径采样，不直接拉一条线回起点。
        end_x, end_y = ring_point_at(dense, arcs, s - s0, open_path=open_path)
        end_lat, end_lng = to_bd(end_x, end_y, c_lat, c_lng)
        if not sampled_gcj:
            locs[-1]["gLat"], locs[-1]["gLng"] = round_to(end_lat, realism.COORD_DECIMALS), round_to(end_lng, realism.COORD_DECIMALS)
        start_point = dict(locs[0])
        if heights is not None:
            start_point["bdA"] = round_to(_elevation_at(arcs, heights, 0.0, open_path=open_path), 2)
        start_point.update({"gLat": round_to(points_bd[0][0], realism.COORD_DECIMALS),
                            "gLng": round_to(points_bd[0][1], realism.COORD_DECIMALS),
                            "type": 5, "state": 1, "totalTime": 0, "validTime": 0,
                            "totalDis": 0.0, "validDis": 0.0, "steps": 0,
                            "speed": 0.0, "avgSpeed": 0.0, "bdS": 0.0,
                            "gainTime": fmt_gain_time(start_ms), "gainTimeMs": start_ms})
        if sampled_gcj:
            start_point["pathProgressM"] = 0.0
        locs.insert(0, start_point)
        for index, point in enumerate(locs, 1):
            point["id"] = index
    apply_location_paces(locs)
    track = {
        "totalTime": total_t_actual,
        "totalDistance": round_to(total_dis_actual, 3),
        "validDistance": round_to(total_dis_actual, 3),
        "validTime": total_t_actual,
        "startTime": start_ms,
        "startLatitude": locs[0]["gLat"] if locs else 0.0,
        "startLongitude": locs[0]["gLng"] if locs else 0.0,
        "totalSteps": int(steps_acc),
        "elevationSource": "route_vertices" if heights is not None else "synthetic_baseline",
        "locations": locs,
        "speedPerTenSec": speed_win,
        "stepsPerTenSec": steps_win,
    }
    if sampled_gcj:
        track["distanceAudit"] = {"mode": distance_mode, "requested_m": dist,
                                  "sampled_gcj_m": dist_acc, "path_progress_m": s,
                                  "available_bd_arc_m": ring_len, "tolerance_m": 0.25,
                                  "coordinate_evaluations": distance_evaluations,
                                  "road_gcj_m": sum(road_steps),
                                  "road_minus_sampled_gcj_m": sum(road_steps) - dist_acc,
                                  "max_road_gcj_speed_mps": max(d / dt for d, dt in zip(road_steps, dts)),
                                  "internal_speed_guard_mps": realism.SPEED_CEIL_POINT,
                                  "kinematics_distance_basis": "sampled_gcj_chords",
                                  "road_distance_basis": "unjittered_unquantized_gcj_source_polyline"}
    avg_cad = (track["totalSteps"] / total_t_actual * 60) if total_t_actual > 0 else 0
    ok(f"轨迹完成 {len(locs)} 点 totalDis={track['totalDistance']:.0f}m "
       f"steps={track['totalSteps']} 平均步频={avg_cad:.0f}spm")
    return track
