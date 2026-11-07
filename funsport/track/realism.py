"""真实数据标定的生成组件。

标定来源：原有采样、坐标和速度参数取自 ``data/`` 的华为运动记录；
``data/keep`` 的配对导出增加累计距离和步数，可用于估算短窗步频/步长。
Keep 的估算序列不是原生逐步传感器读数；相同活动的华为/Keep 文件不得
作为独立样本重复计数。海拔和坡度只有在路径具备可信高程输入时才可拟合。

* **数据近似标定**：采样节奏、坐标精度、速度与累计步数派生的步频关系。
* **文献/假设参数**：加速度计振幅、没有逐步观测支持的高频噪声模型。

每个常数都标注了来源，便于复核与改写。
"""
import math

# ── 采样节奏（数据标定）─────────────────────────────────────
# 以下比例取自较早的 2s 华为样本子集，不能代表完整 Keep 导出；
# Keep 记录同时存在以 1s 和 2s 为主的采样阶段，需要按来源另行分层。
SAMPLE_DT_MODE = 2.0
SAMPLE_DT_MODE_PROB = 0.976
SAMPLE_DT_OTHERS = ((1.0, 0.005), (3.0, 0.004), (4.0, 0.013), (6.0, 0.002))

# ── 坐标精度（数据标定）─────────────────────────────────────
# 真实文件 lat/lon 一律 6 位小数（≈0.11m 量化）。
COORD_DECIMALS = 6

# ── 海拔（数据标定）─────────────────────────────────────────
# 真实海拔几乎恒定：多数 run range ≤0.4m、唯一值只有 1~4 个；20260922 全程
# 只有 1.0 这一个值。说明手机对气压计做了量化与单调化，不是原始气压噪声。
ELE_QUANTUM = 0.1          # 量化步长（m），真实唯一值间隔就是 0.1
ELE_DRIFT_RANGE = 0.06     # 整场漂移幅度上界（m）：真实 range 中位仅 0.05m

# ── 速度过程（数据标定）─────────────────────────────────────
# 真实 dt=2s 时速度 lag-1 自相关 0.38~0.81（中位约 0.65），lag-2 约 0.30，
# 变异系数 cv≈0.21。AR(1) 在 dt=2s 下的 rho 直接等于 lag-1。
SPEED_AR1_RHO = 0.75       # lag-1 自相关目标
SPEED_AR2_RHO2 = 0.44      # lag-2 自相关目标（真实中位）
SPEED_LOG_SIGMA = 0.185    # 对数速度的稳态标准差（对齐 cv≈0.21）
# 末段减速：真实速度剖面的最后一个时间十分位中位 0.746×中位速度，且因为是
# 少数几点慢，p50 反而高于均值（真实 p50=3.43 > mean=3.22）。所以减速要
# 陡而短，不能摊开——摊开会把中位速度拉低，方向正好相反。
FINISH_DECEL = 0.35
FINISH_WINDOW_FRAC = 0.09  # 末段减速占全程时间的比例

# ── 逐点速度边界（数据标定）─────────────────────────────────
# geom.SPEED_FLOOR=1.90 是「平均配速合法性」的下限（配速 8'46"/km），不是逐点
# 的物理下限。真实数据逐点 p1=0.80 m/s、p5=1.99 m/s —— 起步、路口等待、避让
# 行人都会掉到 1 m/s 以下。用 1.90 逐点截断会把速度分布左尾整个切掉。
SPEED_FLOOR_POINT = 0.45   # 真实最低 0.62 m/s（4 处 GPS 空档处），留余量
SPEED_CEIL_POINT = 6.60    # 真实最高 6.31 m/s

# ── 步行/停顿（数据标定）───────────────────────────────────
# 真实速度分布是**左偏**的（偏度 −0.73，p1=0.80 m/s、p5=1.99 m/s），而生成
# 分布接近对称、p1 高达 1.46——说明真实跑步中夹着少量「降速走几步」的采样点
# （起跑热身、路口等待、避让行人）。这类点只占 1~3%，但决定了分布的左尾。
WALK_EVENT_PROB = 0.020       # 每个采样点成为「降速」起点的概率
WALK_EVENT_LEN = (1, 3)       # 持续采样点数
WALK_SPEED_RATIO = (0.20, 0.55)  # 降速到正常速度的比例
WALK_IN_POINTS = (1, 2)       # 起跑后必然出现的慢速点数（热身起步）

# ── 定位丢失（数据标定）─────────────────────────────────────
# 真实数据里有 27/52/87/159s 的超长间隔共 4 处（11 次跑步），是树荫或楼间
# 的信号丢失，期间位移照常累积、只是不产生采样点。这也是 dt_std 的真实来源。
DROPOUT_PROB = 0.36        # 每次跑步出现定位丢失的期望次数
DROPOUT_RANGE = (25.0, 150.0)
# 采样空档硬上限（本任务的验收约束，不是数据标定）：任何相邻采样点的间隔都
# 不得超过 6s。真实数据里 6s 本来就是一个合法间隔（9/3033），因此把一次定位
# 丢失压成「一个 6s 的漏采样间隔」，而不是 25~150s 的整段空洞 —— 后者会让
# 末点被迫从几百米外「跳」到路径终点，坐标反算里程比声明里程少 7%~27%。
MAX_SAMPLE_GAP = 6.0
# 末段最短时长（s）：短于它时提前收尾，使末点仍落在可测量的间隔上。
MIN_SAMPLE_TAIL = 0.5

# ── 定位抖动（数据标定 + 文献标定）─────────────────────────
# 真实 GPS 误差是**重尾**的：绝大多数点只有几十厘米抖动（所以 turn_p50 只有
# 3.1°，转向总量 1230°/km），但偶尔出现米级跳点（所以 accel_p99 高达 1.14
# m/s²，而生成器用纯高斯只能给 0.76）。纯高斯无论怎么调 sigma 都同时
# 得到「中位转角过大 + 极端加速度过小」，必须用「小基底 + 稀有脉冲」。
JITTER_SIGMA_BASE = 0.055     # 常规抖动（m）
JITTER_OUTLIER_PROB = 0.010   # 每点成为跳点的概率
JITTER_OUTLIER_SIGMA = 2.60   # 跳点幅度（m）
JITTER_OUTLIER_DECAY = 0.35   # 跳点后残余拖尾（越大拖得越久）

# ── 转向（数据标定）─────────────────────────────────────────
# 真实 2s 采样（≈6.6m）下转角中位 3.1°、p95 29°、转向总量 1230°/km。
TURN_MEDIAN_TARGET = 3.1
TURN_P95_TARGET = 29.0
TURN_SUM_PER_KM_TARGET = 1230.0

# ── 步频与步长（Keep 累计步数/距离短窗的近似标定）──────
# 常规 2.5~4m/s 时步频约 168spm，速度差异主要反映在估算步长；
# 慢行 1~2m/s 时步频下降。短窗值由累计计数推导，并非逐步传感器读数。
CADENCE_AT_REF = 168.0
CADENCE_REF_SPEED = 3.0
CADENCE_WALK = 120.0
CADENCE_WALK_SPEED = 1.75
CADENCE_RUN_SPEED = 2.5
STRIDE_AT_REF = CADENCE_REF_SPEED * 60.0 / CADENCE_AT_REF

# ── 传感器（文献标定）──────────────────────────────────────
# 垂直振荡周期 = 步频/120（每一步一次垂直起伏）。
GAIT_VERT_FREQ_DIV = 120.0
# 垂直振幅随速度线性增长，业余跑者 6~9cm、精英 4~6cm。
VERT_AMP_SLOPE = 0.020
VERT_AMP_INTERCEPT = 0.015
VERT_AMP_MIN = 0.040
VERT_AMP_MAX = 0.090
GRAVITY = 9.81
BDD_NOISE_SIGMA = 3.0      # 航向噪声（度）
BDD_MOUNT_OFFSET = (5.0, 15.0)   # 手机安装角随机范围（度）


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def cadence_for_speed(v):
    """慢行到稳定跑步的连续短窗步频模型（spm）。"""
    if v <= CADENCE_WALK_SPEED:
        return CADENCE_WALK
    if v >= CADENCE_RUN_SPEED:
        return CADENCE_AT_REF
    f = (v - CADENCE_WALK_SPEED) / (CADENCE_RUN_SPEED - CADENCE_WALK_SPEED)
    return CADENCE_WALK + (CADENCE_AT_REF - CADENCE_WALK) * f * f * (3 - 2 * f)


def stride_for_speed(v, cadence=None):
    """由同一段速度和步频推导步长（m），不独立抽样。"""
    if v <= 0:
        return 0.0
    return 60.0 * v / (cadence if cadence is not None else cadence_for_speed(v))


def vertical_amplitude(v):
    """垂直振幅（m），随速度线性增长并夹在生理区间内。"""
    return clip(VERT_AMP_SLOPE * v + VERT_AMP_INTERCEPT, VERT_AMP_MIN, VERT_AMP_MAX)


def accel_rms(v, cadence):
    """身体质心垂直加速度的 RMS（m/s²）。正弦振荡 a_rms = A(2πf)²/√2。"""
    f_v = cadence / GAIT_VERT_FREQ_DIV
    a_peak = vertical_amplitude(v) * (math.tau * f_v) ** 2
    return a_peak / math.sqrt(2.0)


def sample_interval_series(dur, rng, sample_period_s=2):
    """按活动的 1s/2s 采样模式生成采样时刻表。

    当前默认使用较早华为子集的 2s 采样模式及少量 1/3/4/6s 间隔；
    完整 Keep 导出另有 1s 主导记录，本函数尚未按活动分层拟合。
    定位丢失不再把间隔拉长成 25~150s 的空洞：那会让
    轨迹在最后一两百秒里「没有采样点、却在移动」，末点只能从几百米外跳回
    路径终点，坐标反算里程比声明里程少 7%~27%（实测 seed2 只有 73%）。
    丢失改为「漏掉一个采样点」，即一个 MAX_SAMPLE_GAP（6s）的间隔——6s 本身
    就是真实设备会出现的间隔（9/3033），既保留定位丢失这一事实，又不制造
    超过 6s 的采样空档。

    末段：网格上的最后一个间隔由调用方补成 dur - times[-1]，因此末尾可能是
    不足 1s 的非整秒间隔（真实设备不会正好在整秒处截止）。剩余时间小于
    MIN_SAMPLE_TAIL 时提前收尾，使末段落在 [MIN_SAMPLE_TAIL, MAX_SAMPLE_GAP]。
    """
    if sample_period_s not in (1, 2):
        raise ValueError("采样周期只支持 1s 或 2s")
    times = [0.0]
    t = 0.0
    # 1s 主导的 Keep 活动 4189/4193 个间隔恰为 1s；不沿用 2s 子集的漏采样机制。
    if sample_period_s == 1:
        while dur - t > 1.0:
            t += 1.0
            times.append(t)
        return times
    # 定位丢失发生的位置（占全程比例），每场 0~2 次
    n_drop = 0
    while rng.random() < DROPOUT_PROB and n_drop < 2:
        n_drop += 1
    drops = sorted(rng.uniform(0.15, 0.85) * dur for _ in range(n_drop))
    next_drop = drops.pop(0) if drops else float("inf")

    while True:
        remaining = dur - t
        # 剩下这一段已经可以作为一个合法末段（≤MAX_SAMPLE_GAP）时交给调用方，
        # 这里不再落点：否则会出现跨过 dur 的间隔，由调用方补尾巴多算时长。
        if remaining <= MAX_SAMPLE_GAP:
            break
        u = rng.random()
        if u < SAMPLE_DT_MODE_PROB:
            dt = SAMPLE_DT_MODE
        else:
            dt = SAMPLE_DT_MODE
            acc = SAMPLE_DT_MODE_PROB
            for value, weight in SAMPLE_DT_OTHERS:
                if u < acc + weight:
                    dt = value
                    break
                acc += weight
        # 到了丢失点：这次不出采样点，只漏掉一个采样周期，不产生长空档
        if t >= next_drop:
            dt = MAX_SAMPLE_GAP
            next_drop = drops.pop(0) if drops else float("inf")
        # 末段至少保留 MIN_SAMPLE_TAIL，必要时把当前间隔压短（仍 ≤MAX_SAMPLE_GAP）
        if dt > remaining - MIN_SAMPLE_TAIL:
            dt = remaining - MIN_SAMPLE_TAIL
        t += dt
        times.append(t)
    return times


def ar1_log_speed(times, base, rng, rho=None, sigma=None,
                  ref_dt=None):
    """AR(2) 对数速度过程，窗口长度可不等。

    真实验证：dt=2s 下速度 lag-1 自相关中位 0.72、lag-2 0.44、lag-3 0.29。
    AR(1) 只能匹配 lag-1，之后按 ρ^k 衰减——ρ=0.72 时 lag-2 会是 0.52，
    比真实的 0.44 偏高，这正是 ``speed_lag2/lag3`` 被判可分的原因。AR(2)
    多一个自由度，可以同时匹配 lag-1 与 lag-2：

        ρ1 = φ1/(1−φ2)、ρ2 = φ1·ρ1 + φ2

    由 ρ1=0.70、ρ2=0.44 解出 φ1≈0.769、φ2≈−0.098。负的 φ2 表示速度有一个
    「回弹」成分，符合跑步中配速围绕目标值来回调整的实际过程，也解释了真实
    数据里 lag-2 掉得比指数更快。对不等间隔，把系数按 Δt/ref_dt 幂次收缩。
    """
    # 默认值在调用时读取模块常量，使标定扫描可以直接改 realism.XXX
    rho = SPEED_AR1_RHO if rho is None else rho
    sigma = SPEED_LOG_SIGMA if sigma is None else sigma
    ref_dt = SAMPLE_DT_MODE if ref_dt is None else ref_dt
    phi1, phi2, rho1, rho2 = _ar2_coeffs(rho, SPEED_AR2_RHO2)
    out = []
    x1 = x2 = 0.0
    for i, t in enumerate(times):
        if i == 0:
            x = rng.gauss(0.0, sigma)
        else:
            dt = max(1e-6, t - times[i - 1])
            k = dt / ref_dt
            # φ2 可能为负，分数次幂要用「绝对值幂 + 保留符号」，否则得到复数
            a1 = math.copysign(abs(phi1) ** k, phi1)
            a2 = math.copysign(abs(phi2) ** k, phi2)
            # 创新项方差归一，使稳态标准差仍为 sigma
            innov = math.sqrt(max(0.0, 1.0 - a1 * rho1 - a2 * rho2))
            x = a1 * x1 + a2 * x2 + rng.gauss(0.0, sigma * innov)
        x2, x1 = x1, x
        # 对数正态的均值修正，使 E[exp(x)] = 1
        out.append(base * math.exp(x - 0.5 * sigma * sigma))
    return out


def _ar2_coeffs(rho1, rho2):
    """由 lag-1/lag-2 自相关反解 AR(2) 系数。"""
    phi2 = (rho2 - rho1 * rho1) / (1.0 - rho1 * rho1)
    phi1 = rho1 * (1.0 - phi2)
    return phi1, phi2, rho1, rho2


def finish_deceleration(times, dur):
    """末段减速倍率：跑完后人会走或停下，GPS 继续记录低速点。

    真实数据里最后一个时间十分位的中位是 0.75×，但中位速度仍高于均值——说明
    减速集中在最后3~4个点，不是整段缓降。
    """
    if dur <= 0:
        return [1.0] * len(times)
    start = dur * (1.0 - FINISH_WINDOW_FRAC)
    out = []
    for t in times:
        if t <= start:
            out.append(1.0)
        else:
            frac = (t - start) / max(1e-6, dur - start)
            out.append(1.0 - (1.0 - FINISH_DECEL) * frac)
    return out


def jitter_series(n, rng):
    """重尾定位抖动：小基底高斯 + 稀有米级跳点 + 跳点拖尾。

    为什么不能用单一高斯：真实数据中位转角只有 3.1°（对应约 0.1m 的横向抖动），
    但 accel_p99 达 1.14 m/s²（对应米级跳点）。σ=0.1 的高斯给出正确的中位
    转角却完全没有极端值；σ=0.5 的高斯给出极端值却让中位转角翻倍。
    重尾分布才能同时满足两端——这也与 GPS 误差实测服从 Student-t(ν≈3) 一致。
    """
    out = []
    decay = 0.0
    for _ in range(n):
        step = rng.gauss(0.0, JITTER_SIGMA_BASE)
        if rng.random() < JITTER_OUTLIER_PROB:
            decay = rng.gauss(0.0, JITTER_OUTLIER_SIGMA)
        else:
            decay *= JITTER_OUTLIER_DECAY
        out.append(step + decay)
    return out


def walk_events(n, rng):
    """降速倍率序列：起跑热身 + 途中偶发「走几步」。

    真实速度分布左偏（偏度 −0.73），p1 只有 0.80 m/s。生成器用纯对数正态
    只能给出对称分布、p1 偏高，所以必须显式注入低速事件。这些点的位移也按
    低速累计，因此不影响总里程与平均速度——它们只是把少量时间换成慢速。
    """
    out = [1.0] * n
    # 起跑热身：前 WALK_IN_POINTS 个点明显慢
    for i in range(min(n, rng.randint(*WALK_IN_POINTS))):
        out[i] = rng.uniform(0.45, 0.75)
    # 途中降速事件
    i = 2
    while i < n - 2:
        if rng.random() < WALK_EVENT_PROB:
            length = rng.randint(*WALK_EVENT_LEN)
            ratio = rng.uniform(*WALK_SPEED_RATIO)
            for j in range(i, min(n, i + length)):
                out[j] = ratio
            i += length + rng.randint(2, 6)
        else:
            i += 1
    return out


def quantize_elevation(values, rng, quantum=ELE_QUANTUM,
                       drift=ELE_DRIFT_RANGE):
    """保留路径逐点高程及很小的设备漂移，并按设备精度量化。"""
    if not values:
        return values
    amp = drift * rng.uniform(0.2, 0.5)
    phase = rng.uniform(0.0, math.tau)
    n = max(1, len(values) - 1)
    out = []
    for i, value in enumerate(values):
        frac = i / n
        drift_val = amp * math.sin(math.tau * frac * 0.5 + phase)
        out.append(round((value + drift_val) / quantum) * quantum)
    return out


def snap_precision(value, decimals=COORD_DECIMALS):
    """按真实设备的坐标精度截断（真实是 6 位小数，不是 7 位）。"""
    return float(f"{value:.{decimals}f}")
