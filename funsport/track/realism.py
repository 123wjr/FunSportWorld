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
# 末段减速：真实速度剖面的最后一个时间十分位中位 0.746×中位速度，p50 反而
# 高于均值（真实 p50=3.43 > mean=3.22）。这说明末段减速必须**陡而短**：
# 旧实现是 9% 时长的线性缓降，等于让末段十分之一的时间都以 0.65~0.82× 运行，
# 再叠上 AR 低值后实测末 100m 只剩 0.40×（步频 122.89、步长 59.13cm），
# 正是用户反馈的「异常的低值持续」。现在把窗口收窄到 6% 并改用 frac² 形状：
# 末段大部分时间仍是正常配速，减速集中在最后 2~4 秒（1~2 个采样点）。
FINISH_DECEL = 0.35
FINISH_WINDOW_FRAC = 0.06  # 末段减速占全程时间的比例
FINISH_SHAPE = 2.0         # 形状指数：1=线性（摊开全程），2=尾部集中（陡而短）

# ── 速度包络（文献 + 数据标定）───────────────────────────────
# 疲劳：docs/realism-state-of-the-art.md §2.2 的饱和指数
#     f(d) = 1 − A·(1 − exp(−d/D))，A≈0.10~0.13，D≈0.5·总距离
# 旧实现是 max(1.03 − 0.06·(km−0.5), 0.80)：先抬到 1.03（伪造「先快后慢」的
# 起步加速），4.3333km 撞底后**完全平坦**，末段再没有疲劳信号，撞底处还有
# 斜率突变。饱和指数全程单调平滑，d→∞ 渐近到 1−A，不存在平台段。
FATIGUE_AMP = 0.12
FATIGUE_DIST_FRAC = 0.5    # D = 0.5·总距离
# 起跑惯性：一阶惯性 v(t)=v_c·(1−exp(−t/τ))，τ≈3~5s（同节 §2.5）。归一化成
# 不归零的形式，起点即 R0（避免首点速度 0 造成 pace=∞ 与 0 位移）：
#     ramp(t) = R0 + (1−R0)·(1−exp(−t/τ))
START_INERTIA_RATIO = 0.60
START_INERTIA_TAU = 4.0
# 波动幅度目标：真实 Keep 的对数速度稳态 sd σ=0.185 对应 cv=√(e^σ²−1)=0.187，
# 但包络各项是各自独立乘上去的（起步/疲劳/三条正弦/末段/步行），实测 cv 被抬
# 到 0.236、极差 0.93~4.64 m/s、起步 600m 峰值 1.41×。这里把合成倍率的对数
# 偏差整体归一到目标 sd 再回乘：形状与相关性不变，只收窄幅度。目标取真实
# Keep 的 cv≈0.21。
SPEED_CV_TARGET = 0.21
# 落盘口径的标定增益：逐点速度是 2 位小数的 totalDis 差分/整数秒，加上
# fit_speeds 的边界夹紧，会把 cv 系统性压低约 15%。实测把合成倍率的对数 sd
# 归一到 target×1.18，落盘 cv 的中位才真正落在 0.21（1000/3000/8000m × 3 seed
# 六条轨迹：0.197~0.219，中位 0.212）。
SPEED_LOG_SD_GAIN = 1.18
SPEED_LOG_SD_TARGET = (math.sqrt(math.log(1.0 + SPEED_CV_TARGET ** 2))
                       * SPEED_LOG_SD_GAIN)
# 低速持续性：低于 LOW_SPEED_RATIO×平均速度的点最多连续 LOW_SPEED_RUN_MAX 个。
# 2s 采样下连 3 点就是 6s 以上的持续低速；真实数据里这只会出现在定位丢失
# （已压成单个 6s 间隔）与终点停表，用户反馈的「异常的低值持续」正指这类段。
# 这是保底约束，不是经验式：波动归一后绝大多数轨迹根本触发不到。
LOW_SPEED_RATIO = 0.60
LOW_SPEED_RUN_MAX = 2
# 门槛裕度：约束在 build 里施加于「拟合前的速度序列」，而校验用的是落盘坐标
# 差分出的速度，两者相差一次均匀缩放与 2 位小数取整（实测约 0.2%）。留 3% 裕度
# 后，落盘序列上不会再出现恰好压着 0.6×均速的多点连段（未加裕度时会出现
# 1.56 vs 门槛 1.5603 这类刀锋样本）。
LOW_SPEED_GATE_MARGIN = 1.03

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

# ── 步频与步长（经验方程）───────────────────────────────────
# 旧实现是「v≤1.75 恒 120spm / v≥2.5 恒 168spm / 中间 smoothstep」：跑步区间
# v≥2.5 完全不随速度变化（弹性 dln(cad)/dln(v)=0，经验值 0.28），过渡区弹性
# 反而高达 ~1.3。实测 v=1.75 给 120 而经验式要 146.2（−17.9%），v=4.00 给
# 168 而经验式要 184.3（−8.8%），corr(瞬时速度, 步频) 中位只有 0.2347。
#
# 现在改用文献经验方程对（docs/realism-state-of-the-art.md §2.1）：
#     步长  L(v)   = STRIDE_POWER_REF_M · (v/3)^0.72 = 1.05·(v/3)^0.72  m
#     步频  cad(v) = CADENCE_POWER_REF_SPM · (v/3)^0.28                spm
# 推导（两者是同一个式子的两种写法，不是两条独立标定）：
#     cad = 60·v / L = 60·v / (1.05·(v/3)^0.72)
#         = (60·3/1.05) · (v/3)^(1−0.72)
#         = 171.4286 · (v/3)^0.28
# 因此 CADENCE_POWER_EXP = 1 − STRIDE_POWER_EXP，参考点 v=3 处 L=1.05m 与
# cad=171.4286spm 严格互为 60v/L。**只把步频作为自变量、步长由 60v/cad 反推**
# 就自动同时满足两条经验式；反过来「步频和步长各自独立满足经验式」不可能，
# 因为 steps×stride 必须等于里程。
STRIDE_POWER_REF_M = 1.05
STRIDE_POWER_REF_SPEED = 3.0
STRIDE_POWER_EXP = 0.72
CADENCE_POWER_REF_SPEED = STRIDE_POWER_REF_SPEED
CADENCE_POWER_EXP = 1.0 - STRIDE_POWER_EXP                      # = 0.28
CADENCE_POWER_REF_SPM = 60.0 * STRIDE_POWER_REF_SPEED / STRIDE_POWER_REF_M  # 171.4286
# 兼容别名（tests/tools 里的旧名字，语义保持不变）：
#   CADENCE_AT_REF    = 参考速度处的步频（现在是幂律参考值 171.43，不再是 168）
#   CADENCE_REF_SPEED = 参考速度
#   STRIDE_AT_REF     = 参考速度处的步长 = 60·v_ref/cad_ref = 1.05 m
#   CADENCE_WALK / CADENCE_WALK_SPEED / CADENCE_RUN_SPEED 曾是分段模型的两端，
#   现保留为历史别名，不再参与计算。
CADENCE_AT_REF = CADENCE_POWER_REF_SPM
CADENCE_REF_SPEED = CADENCE_POWER_REF_SPEED
STRIDE_AT_REF = CADENCE_REF_SPEED * 60.0 / CADENCE_AT_REF
CADENCE_WALK = 120.0
CADENCE_WALK_SPEED = 1.75
CADENCE_RUN_SPEED = 2.5
# 边界钳制（**不是经验式本体**）：幂律在 v→0 时 cad→0，但那是外推失效区——
# 经验式只在跑步区间（约 1.6~6 m/s）有数据支撑，而慢走/原地踏步仍有
# 100~110spm。所以只做下限钳制，且它只在 v < CADENCE_FLOOR_SPEED（≈0.44m/s，
# 即逐点速度下限附近）才可能生效，正常跑步区间永远碰不到。
CADENCE_FLOOR_SPM = 100.0
CADENCE_FLOOR_SPEED = CADENCE_POWER_REF_SPEED * (CADENCE_FLOOR_SPM / CADENCE_POWER_REF_SPM) ** (1.0 / CADENCE_POWER_EXP)

# 步频对速度的一阶跟随时间常数（s）：真实跑者在 1~2 个步态周期内完成调整。
# 旧实现取 15s，步频整体滞后速度十几秒，是 corr(v, cadence) 只有 0.23 的主因。
CADENCE_RESPONSE_TAU_S = 3.0

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
    """经验幂律短窗步频（spm）：171.43·(v/3)^0.28，慢行处受下限钳制。

    弹性 dln(cad)/dln(v) ≡ CADENCE_POWER_EXP = 0.28（实测目标值）；旧实现
    v≥2.5 与 v≤1.75 两端的弹性都是 0，只有过渡区剧烈变化。
    v→0 时幂律会趋于 0，因此加 CADENCE_FLOOR_SPM 钳制——这是边界处理，
    不是经验式的一部分（见上方常量注释）。
    """
    if v <= 0:
        return CADENCE_FLOOR_SPM
    return max(CADENCE_FLOOR_SPM,
               CADENCE_POWER_REF_SPM
               * (v / CADENCE_POWER_REF_SPEED) ** CADENCE_POWER_EXP)


def stride_power_for_speed(v):
    """经验幂律步长（m）：1.05·(v/3)^0.72，仅供核验/诊断对照。"""
    if v <= 0:
        return 0.0
    return STRIDE_POWER_REF_M * (v / STRIDE_POWER_REF_SPEED) ** STRIDE_POWER_EXP


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
            # 形状指数 FINISH_SHAPE>1：前段几乎不减速，减速集中在最后几个点
            out.append(1.0 - (1.0 - FINISH_DECEL) * frac ** FINISH_SHAPE)
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
