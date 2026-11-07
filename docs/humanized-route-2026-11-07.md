# 高德路线拟人化 + 运动拟真改造（2026-11-07）

分支：feat/humanized-route（基线 ec49382，即 research/road-realism-comparison 的 HEAD）
产出：funsport/track/humanize.py（新增）、funsport/api/flow.py（接入生产）、
funsport/track/generator.py 与 realism.py（运动模型）、tools/run_humanized_loop.py（一键入口）、
tools/run_overlay_charts.py 与 overlay_charts_template.html（可叠加可视化）。

---

## 1. 结论速览

| 用户提出的问题 | 处理 | 实测（同一环 1478.9 m，seed=7） |
|---|---|---|
| 道路转角太尖锐 | 5°–150° 转角改精确圆弧（圆心距 t/sin(φ/2)），大于 150° 改真实半径 U 型弧 | 最大单点转角 173.7° → 25.0°，>45° 12 个 → 0 个，最小曲率半径 0.60 → 1.18 m |
| 直线太笔直 | 低频横向摆动（3 个整数波数正弦，波长约 180/90/49 m）+ 曲率包络，直线全幅、弯道收窄 | 二阶差分 RMS 0.002 1/m（旧 0.000），偏离高德折线 p50 1.86 / p95 3.86 / max 4.55 m |
| 相同道路轨迹太重合 | 全程靠右行驶 2.0 m（掉头两腿因此自然分开）+ 同向重复段分道 1.6 m + 跨圈漂移 | 重复走廊 298 m(20.1%) → 0.0~4.2%，反向平行段横向间距 p50 0.00 → 1.64 m |
| 掉头处理 | U 型弧半径 1.2 m，顶点替换为半圆，前后与偏移直线相切 | 三处掉头 Menger 半径 1.177 / 1.143 / 1.136 m，无自交 |
| 速度波动大、有异常低值持续 | 起步惯性 τ=4 s、疲劳饱和指数、方差归一 + log 域增益、末段二次减速、低速连段约束 | cv 0.283 → 0.217；最长连续低速点 6 → 2 |
| 步频与经验方程 | 幂律 171.43·(v/3)^0.28（由步长式 1.05·(v/3)^0.72 反推） | 弹性 dln(cad)/dln(v) 0.072 → 0.265~0.364；corr(v,cad) 0.206 → 0.60~0.71（2 s）/ 0.82~0.95（10 s） |

**可复跑**：一条命令从高德缓存到交互式 HTML，产物 artifacts/humanized/route_overlay.html。
---

## 2. 路线拟人化（funsport/track/humanize.py）

处理链（_humanize_plane）：

1. clean：合并短于 0.5 m 的段。
2. collapse：短于 2.0 m 的段并入相邻长段。这一步必须做——高德原始折线里有 0.069 m 的腿缝与 0.55 m 的退化段，它们会把任何沿法向偏移的算法逼出假尖点（实测未 collapse 时曲率跳变 max 2.589、最小半径 0.305 m、单点转角 173.7°）。
3. round：对 5°–150° 转角插精确圆弧，t = min(r·tan(φ/2), 0.45·Lp, 0.45·Ln)，圆心距 = t/sin(φ/2)。写成 r/sin 是错的：只有 90° 角碰巧成立，其余会把整条环撑大（实测 1427.6 m → 1932.0 m）。
4. resample：0.6 m 等距重采样，给后面的连续场一个稳定网格。
5. lane：按局部右法向偏移 h=2.0 m，曲率半径小于 6 m 处收窄（保护掉头弧）。
6. uturn：大于 150° 的顶点整点替换为半径 1.2 m 的半圆；h 与 hu 的差在 12 m 内平滑过渡。
7. wander：横向摆动 want = Σ a_j·sin(2π·w_j·s/L + φ_j)，振幅 1.20/0.80/0.50 m，波数 8/16/30（取整数才保证环首尾闭合）。
8. relane：同向重复段（切向点积 ≥0.90 且间隔 ≥20 m）按 ±0.8 / 1.6 m 轮换分道。
9. laps：多圈时每圈整体加 0.9~1.8 m 偏移，避免同一条轨迹压在一起。

包络：env = smoothstep(near/10 m)，near 是到最近高曲率点的弧长距离；远处 1.0（全幅摆动），近处 0.0（保护转弯）。收口用**滑动最小值（腐蚀）**而不是滑动平均——掉头弧只有 3.8 m 长，在 17 点的平均窗口里会被平均回 0.463，实测把掉头半径削到 0.52 m。包络必须乘在**合成后的总偏移**上（摆动 + 分道 + 跨圈）：只乘摆动项时掉头弧上仍叠着 0.8 m 分道偏移，半径照样被削。

输出：1.5 m 间距抽稀（弯道全保留），7 位 BD 小数。6 位小数在 0.6 m 采样上会产生约 10°/点的量化航向噪声，把 1214°/km 的真实几何抬到 3896°/km；7 位 + 1.5 m 抽稀后回到 1320°/km（平面真值 1290°/km）。

### 参数表（DEFAULTS）

corner_radius_m 8.0 / corner_min_deg 5 / corner_max_deg 150 / corner_max_len_frac 0.45 / arc_sample_m 0.5 / collapse_min_seg_m 2.0 / resample_m 0.6 / lane_offset_m 2.0 / lane_taper_lo_m 2.0 / lane_taper_hi_m 6.0 / u_turn_min_deg 150 / u_turn_radius_m 1.2 / u_turn_ramp_m 12.0 / wander_amp_m (1.20, 0.80, 0.50) / wander_waves (8, 16, 30) / repeat_tol_m 1.0 / repeat_min_gap_m 20 / repeat_angle_cos 0.90 / repeat_lane_m 1.6 / lap_drift_m 0.9 / lap_drift_max_m 1.8 / offset_ramp_m 10.0 / offset_slope_max 0.20 / out_spacing_m 1.5 / out_keep_radius_m 20.0 / COORD_DECIMALS 7。

### 重复走廊调参证据（seed=7）

h=0.6 → 180.0 m(12.3%)｜h=1.2 → 156.5 m(10.7%)｜h=1.6 → 113.5 m(7.7%)｜h=2.0 → 54.5 m(3.7%)｜h=2.4 → 54.5 m(3.7%)｜h=3.0 → 26.5 m(1.8%)。
五 seed（0/1/7/42/123）：0.0% / 0.8% / 4.2% / 3.2% / 1.6%。
主控旋钮是 lane_offset_m；relane 只处理同向重复，反向往返一律不检测（反向往返是靠靠右行驶解决的）。

### 接入生产

funsport/api/flow.py：HUMANIZE_ROUTE = True，_build_amap_track 里选路结果先过 humanize_route(ring, seed=seed)，再补一个闭合点到末尾（_ring_length 只累加相邻段），然后进 generator.build。对外 ring_length_m 仍按**高德原折线**算，口径不变（tests/test_realism_sampling.py 有断言）。任一步抛异常都 warn 后退回原折线——拟人化不能成为生成失败的新来源。
---

## 3. 运动模型（realism.py + generator.py）

### 3.1 两条经验式

cadence_for_speed(v) = max(100, 171.4286·(v/3)^0.28)
stride_power_for_speed(v) = 1.05·(v/3)^0.72

二者在 v=3 处严格互推（171.43 spm × 1.05 m = 180 m/min = 3 m/s）。落盘步长仍由 stride_for_speed(v, cad) = 60v/cad 反推，所以两条经验式同时成立。旧实现在 v ≥ 2.5 完全平坦（弹性 0），v=1.75 给 120 spm 而经验式要 146.2。

### 3.2 速度包络

w = ar1_log_speed × inertia × fatigue × wave × finish × walk

- inertia：0.60 + 0.40·(1 − e^(−t/4))，τ=4 s（旧实现是线性 0.85→1.03 抬升）
- fatigue：1 − 0.12·(1 − e^(−d/(0.5·总程)))（旧实现 max(1.03−0.06(km−0.5), 0.80)，在 4.3333 km 撞底后完全平坦）
- finish：最低 0.65×，占时 6%，二次型

先归一均值（env_mean）再压缩方差（log_sd → 目标 sqrt(ln(1+0.21²))·1.18 = 0.2451），然后 fit_speeds 回到规划里程；低速连段约束 _cap_low_runs（门槛 0.6×均速×1.03、最长 2 点）**放在 fit_speeds 之后**，放前面会被整体缩放重新打破。

### 3.3 端到端（humanized 环 1472.3 m，seed=7，pace 360 s/km）

| 圈数 | 点数 | v 均值 | cv | 最长低速连段 | 弹性 | 总时长 |
|---|---|---|---|---|---|---|
| 1 | 262 | 2.79 | 0.214 | 2 | 0.300 | 08:50 |
| 2 | 524 | 2.78 | 0.211 | 2 | 0.278 | 17:40 |
| 3 | 786 | 2.78 | 0.212 | 2 | 0.297 | 26:30 |

步频均值 167.0~167.7 spm，步长均值 0.993~0.997 m，cadence/emp 0.98~1.00，stride/emp 1.00~1.02。

### 3.4 已知限制（不靠调参掩盖）

- **2 s 整数量化导致 30 spm 阶梯**：2 s 采样下 steps 是整数，逐点步频只能是 120/150/180/210。时间加权均值与经验式吻合（残差 −2.7~+0.4 spm），但逐点曲线是方波，看步频请用 10 s 窗。
- stepDistance 原来恒为 0.0（generator.py:727），已改为本段平均步长 = d_step / d_steps。
- 步长在落盘侧仍是从 v 与 cad 反推（stride_for_speed = 60v/cad），不是独立建模量；它看起来对，是因为 v 与 cad 各自都被经验式约束住了。
- laps=3 曾观察到最长低速连段 4，无法在 build 内复现（18 例单圈 + 12 例多圈均为 2），未定位，保留为开放项。
- 传感器字段（bdS/bdA/bdG）与逐点步频噪声不在本轮范围。

---

## 4. 一键入口

    .venv\Scripts\python.exe -B tools\run_humanized_loop.py --laps 1,2,3 --seed 7 --pace 360

流程：读 .funsport/campus_loop_bd.json → 复刻生产选路打分（score = 长度 + 4×重复长度，近优集取 seed 对 3 取模）→ humanize_route → geom.make_point_ring(ordered=True) 取环长 → generator.build(dist=环长×laps, ordered_path=True, distance_mode=arc) → 折成 funsport.overlay/1 → 调 tools/run_overlay_charts.py 出单文件 HTML。

产物：artifacts/humanized/track_laps1.json、track_laps2.json、track_laps3.json、overlay_all.json、route_overlay.html。

常用参数：--laps 1,2,3 / --seed / --pace（秒每公里，默认 360）/ --cadence / --out-dir / --no-charts / --json-only。
---

## 5. 可叠加可视化

单文件 HTML，无外链、无 CDN，双击即开。内容：

- 路线图（Web Mercator 等比）：拟人化轨迹、原始高德折线（低饱和灰虚线参考）、掉头标记（白圈去重）、公里刻度；滚轮缩放 / 拖拽平移，与下方时间轴共享游标。
- 速度 / 步频 / 步长三面板：共享时间轴与游标，可「分面」或「叠加」；叠加支持 z-score 与 min-max 归一。
- 时间轴「绝对 / 按各自时长」开关：多圈共用绝对轴时 laps=1 只占左边一小段，归一后才能比形状。
- 经验式参考虚线：步频 171.43·(v/3)^0.28、步长 1.05·(v/3)^0.72。
- 10 s 平滑开关、播放 1×/4×/16×、滑块、游标数值面板、总览统计卡片。

数据 schema：funsport.overlay/1，最小字段 points[].{t, v, cadence, stride_m}；坐标、经验式列、10 s 派生序列都可缺省自动补。

命令行：

    .venv\Scripts\python.exe -B tools\run_overlay_charts.py --input artifacts/humanized/overlay_all.json
    .venv\Scripts\python.exe -B tools\run_overlay_charts.py --selftest      （7 条路径自检）

验收状态：headless Chrome 实际渲染通过（截图 .test-tmp/lead/final_shot.png、.test-tmp/charts/_shot_after.png）；
归一模式与 DOM 交互用 node DOM stub 覆盖 27/27。唯一未做的是真人手工点选，属观感验证。
---

## 6. 测试与仓库卫生

- 基线（ec49382 未改动）：Ran 810 / failures=2 / errors=14。验收跑（含本轮新增用例）：**Ran 817 / failures=2 / errors=14**，与基线逐条相同，没有新增失败。2 个失败与 14 个 error 全部与本次改动无关：error 清一色是 funsport/api/flow.py:318 的 SimpleNamespace 缺 freeze_run_time。
- 本轮新增 tests/test_humanize_route.py（6 例：方角环圆滑、里程与闭合、偏离有界、原路往返分道与掉头弧半径、种子可复现、退化输入不崩）。
- 本轮改了 3 处既有断言，都是**场景**而非阈值：
  - tests/test_route_geometry.py 的 zigzag/2/1/2001 改为 duration=640（配速 3.13 m/s）：新包络把该固定抽样的弦长从 10.205 m 降到 9.506 m，13.2 m 道路预算内已可建成；守卫不变，换到仍不可行的配速继续验证。
  - tests/test_road_joint_e0_diagnostic.py 的 fast 用例改为 dur=110（2.73 m/s），仍必须是 no_feasible_sample_in_gcj_road_budget。
  - tests/test_server_time_error.py 的预览断言：一个用例显式关掉 HUMANIZE_ROUTE 继续断言「传给生成器的是高德折线」，另新增一个用例断言拟人化开启后环被改写、首尾闭合、且每个顶点离高德道路不超过 25 m。
- 仓库卫生：artifacts/、.test-tmp/、data/keep/ 已加入 .gitignore（生成产物可复跑；data/keep 是真实用户导出，属私密数据）。
- 未提交：本分支的改动由 lead 统一提交；队友与 lead 的中间报告（docs/.status-*.md、.motion-report.md、.charts-report.md、.verify-humanized.md）是工作笔记，保持未跟踪。
