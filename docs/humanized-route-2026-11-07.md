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
2. snip_spurs：剪掉 2~6 m 的「针尖毛刺」——转角 >150°、某条腿短于 6 m、且两个端点相距不超过 10 m 时删掉该顶点（循环直到没有可剪的）。高德折线在往返腿末端常有 2 m 尖刺，靠右偏移会把往返两腿推到一起，实测残留 78° 假折角；剪后单圈最大转角 78.77° → 24.77°、>45° 0 个。
3. collapse：短于 2.0 m 的段并入相邻长段。这一步必须做——高德原始折线里有 0.069 m 的腿缝与 0.55 m 的退化段，它们会把任何沿法向偏移的算法逼出假尖点（实测未 collapse 时曲率跳变 max 2.589、最小半径 0.305 m、单点转角 173.7°）。
4. round：对 5°–150° 转角插精确圆弧，t = min(r·tan(φ/2), 0.45·Lp, 0.45·Ln)，圆心距 = t/sin(φ/2)。写成 r/sin 是错的：只有 90° 角碰巧成立，其余会把整条环撑大（实测 1427.6 m → 1932.0 m）。
5. resample：0.6 m 等距重采样，给后面的连续场一个稳定网格。
6. lane：按局部右法向偏移 h=2.0 m，曲率半径小于 6 m 处收窄（保护掉头弧）。
7. uturn：大于 150° 的顶点整点替换为半径 1.2 m 的半圆；h 与 hu 的差在 12 m 内平滑过渡。
8. wander：横向摆动 want = Σ a_j·sin(2π·w_j·s/L + φ_j)，振幅 1.20/0.80/0.50 m，波数 8/16/30（取整数才保证环首尾闭合）。
9. relane：同向重复段（切向点积 ≥0.90 且间隔 ≥20 m）按 ±0.8 / 1.6 m 轮换分道。
10. laps：多圈时**每圈独立重跑上面的完整链**，不是把第一圈复制/平移。第 k 圈用子种子 `_lap_seed`（k=0 用原 seed，k≥1 用 seed×7919+k×104729）重新生成摆动相位，圆角半径再乘 ±lap_corner_jitter（15%）随机抖动；第 k≥1 圈圈首 lap_ramp_m（30 m）内用 smoothstep 平滑接到上一圈末点。因此每圈的走线、圆角、摆动都不同（两圈同位置横向相差中位 1.2 m、最大 3.6 m），掉头半径与走廊宽度不受圈间拼接影响；laps=1 与 humanize_route(seed) 逐点一致。实测 laps 1/2/3 最大转角 24.96/25.27/26.85°、>45° 0 个、最小半径 1.17~1.18 m、接缝位移 0.6 m。

包络：env = smoothstep(near/10 m)，near 是到最近高曲率点的弧长距离；远处 1.0（全幅摆动），近处 0.0（保护转弯）。收口用**滑动最小值（腐蚀）**而不是滑动平均——掉头弧只有 3.8 m 长，在 17 点的平均窗口里会被平均回 0.463，实测把掉头半径削到 0.52 m。包络必须乘在**合成后的总偏移**上（摆动 + 分道 + 跨圈）：只乘摆动项时掉头弧上仍叠着 0.8 m 分道偏移，半径照样被削。

输出：1.5 m 间距抽稀（弯道全保留），7 位 BD 小数。6 位小数在 0.6 m 采样上会产生约 10°/点的量化航向噪声，把 1214°/km 的真实几何抬到 3896°/km；7 位 + 1.5 m 抽稀后回到 1320°/km（平面真值 1290°/km）。

### 参数表（DEFAULTS）

corner_radius_m 8.0 / corner_min_deg 5 / corner_max_deg 150 / corner_max_len_frac 0.45 / arc_sample_m 0.5 / spur_max_m 6.0 / spur_close_m 10.0 / collapse_min_seg_m 2.0 / resample_m 0.6 / lane_offset_m 2.0 / lane_taper_lo_m 2.0 / lane_taper_hi_m 6.0 / u_turn_min_deg 150 / u_turn_radius_m 1.2 / u_turn_ramp_m 12.0 / wander_amp_m (1.20, 0.80, 0.50) / wander_waves (8, 16, 30) / repeat_tol_m 1.0 / repeat_min_gap_m 20 / repeat_angle_cos 0.90 / repeat_lane_m 1.6 / lap_corner_jitter 0.15 / lap_ramp_m 30.0 / offset_ramp_m 10.0 / offset_slope_max 0.20 / out_spacing_m 1.5 / out_keep_radius_m 20.0 / COORD_DECIMALS 7。

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
- --auto 不再随机步频目标：_resolve_params 的 auto 分支把 cadence 置 0，交给经验式 171.43·(v/3)^0.28 由瞬时速度反推（实测 auto 得到 2538.9 m / 401.6 s/km / cadence 0）。手动 --cadence 仍是硬目标，_fit_cadence 在 log 域整体缩放使时间加权均值等于它。

先归一均值（env_mean）再压缩方差（log_sd → 目标 sqrt(ln(1+0.21²))·1.18 = 0.2451），然后 fit_speeds 回到规划里程；低速连段约束 _cap_low_runs（门槛 0.6×均速×1.03、最长 2 点）**放在 fit_speeds 之后**，放前面会被整体缩放重新打破。

### 3.3 端到端（humanized 环 1472.3 m，seed=7，pace 360 s/km）

| 圈数 | 点数 | v 均值 | cv | 最长低速连段 | 弹性 | 总时长 |
|---|---|---|---|---|---|---|
| 1 | 262 | 2.79 | 0.214 | 2 | 0.300 | 08:50 |
| 2 | 524 | 2.78 | 0.211 | 2 | 0.278 | 17:40 |
| 3 | 786 | 2.78 | 0.212 | 2 | 0.297 | 26:30 |

步频均值 167.0~167.7 spm，步长均值 0.993~0.997 m，cadence/emp 0.98~1.00，stride/emp 1.00~1.02。

多圈不是同一环逐点复制，也不是「拟人化一圈后整环平移」：每圈用独立子种子重新走完整拟人化链，摆动相位与圆角半径都重新取。seed=7 的两圈：第 2 圈到第 1 圈的最近距离中位 1.22 m、p90 2.88 m、最大 3.63 m，两圈长度 1472.31 / 1467.64 m；接缝处（圈首 30 m）平滑过渡，位移 0.60 m。三圈的各圈长 1472.31 / 1467.64 / 1471.92 m。因为每圈几何不同，生成器沿这条路径跑出来的速度也是分段独立的：artifacts 里 laps=2 的第 1/2 圈 v 均值 2.836 / 2.723，laps=3 的第 1/2/3 圈 2.930 / 2.737 / 2.693（laps=3 时末圈还含终点减速）。

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

### 4.1 里程规则：不是环，是链路

起点 → P1 → … → Pn → 返程回起点，一圈是一整条链路；目标里程到了就停，不补整圈、不额外闭合。环只在「点位走完了还没到目标」时用来继续跑，本身不是指标。

- **起点默认固定**：生产 funsport/api/flow.py 新增 START_GCJ = (39.925210, 119.547725)（GCJ-02，用户指定点），_build_amap_track 调 get_campus_loop(..., start_gcj=START_GCJ) 把它作为链路锚点插到点位序列最前，于是轨迹预览（GUI「生成轨迹预览」/ prepare_route_preview）与提交方案都从该点出发，末段再回到该点；不再是「点位缓存里的第一个点位」。tools/run_chain_route.py 的 --start 默认值同样是该点，--target 默认 2000、--pace 默认 360。高德会把起点吸附到最近可步行道路，本点实测偏移 **27 m**（吸附后 39.925029,119.547939），工具会打印这个差值。
- **不再为凑圈多跑**：生产 funsport/api/flow.py 原先在「目标小于环长」时强行跑满一整圈 +50 m；现在先算 _visit_all_points_m（沿环走到最后一个点位所需里程），目标不足时按该里程生成并 warn，绝不为了凑整圈多跑。
- **元数据**新增 visit_all_points_m、partial_lap_distance_m、returns_to_start，可判断这次是「走完点位」还是「提前停」、以及是否包含返程段。
- 示例（缓存链路 2856.9 m，走完点位 1672.0 m）：--target 1500 → 实际 1672 m、终点停在最后一个点位附近（离起点 536 m）；--target 2000 → 2000 m、终点离起点 613 m。

### 4.2 命令行：route 子命令

    .venv\Scripts\python.exe -B -m funsport.main route --start 119.547725,39.925210 \
        --point 119.551768,39.924663 --point 119.550203,39.929288 \
        --target 2000 --pace 360

- 只本地生成轨迹与 HTML，**不提交、不访问运动接口**；与 run 一样是独立子命令。
- 点位来源：给了 --point 就按给定顺序（不读缓存）；没给就读点位缓存（chain.points_from_cache_gcj）。起点缺省 = 第一个点位；两者重合时**跳过这条 0 m 的退化腿**，不向高德发 origin==destination 请求（leg_dists 仍按 0 m 计入，走完点位里程不受影响）。
- 目标口径与 run 一致：--target 单位米、优先级最高；--dist <100 按 km、>=100 按米；都没有则 2000 m。
- 打印：链路长度（起点→点位→返程）、走完点位里程、是否因目标过短而升档、圈数、实际里程/时长/点数、终点离起点距离、速度均值与 cv、步频/步长、最长低速连段、步频弹性、JSON/HTML 路径。
- 参数错误、生成失败、写盘失败都返回退出码 1 并打印一行错误，不抛 traceback。

### 4.3 图形界面

- 方案面板底部新增**链路诊断行**：读方案 route 元数据的 visit_all_points_m / complete_laps / partial_lap_distance_m / returns_to_start，显示「链路：走完点位 1672 m · 已走 1 圈 · 尾部 328 m · 已走返程」；旧方案没有这些字段时显示「未提供（旧方案）」，参数一改就回到「尚未生成方案」。
- 新增**「导出交互路线图」按钮**：只在有方案且不忙时可用；后台调用 gui_services.export_route_overlay，默认写 artifacts/humanized/gui_<plan_id>.json / .html，完成后状态栏与日志给出路径并交给系统浏览器打开。导出只读已生成轨迹，**不联网、不重新规划、不提交**。
- **轨迹预览起点也已固定**：「生成轨迹预览」走 funsport/api/flow.py:prepare_route_preview → _build_amap_track，与提交方案共用同一条链路，因此预览轨迹从 START_GCJ 出发（吸附后 39.925029,119.547939），不再从「点位缓存里的第一个点位」出发。
- 已知限制：方案快照不含路线折线，导出的 overlay 文档 route 为空，图里只有轨迹点、没有高德参考折线；需要参考折线时用 4.1 / 4.2 的入口（它们带链路折线）。

---

## 5. 可叠加可视化

单文件 HTML，无外链、无 CDN，双击即开。内容：

- 路线图（Web Mercator 等比）：拟人化轨迹、原始高德折线（低饱和灰虚线参考）、掉头标记（白圈去重）、公里刻度；滚轮缩放 / 拖拽平移，与下方时间轴共享游标。
- 速度 / 步频 / 步长三面板：共享时间轴与游标，可「分面」或「叠加」；叠加支持 z-score 与 min-max 归一。
- 时间轴「绝对 / 按各自时长」开关：多圈共用绝对轴时 laps=1 只占左边一小段，归一后才能比形状。
- 经验式参考虚线：步频 171.43·(v/3)^0.28、步长 1.05·(v/3)^0.72。
- 10 s 平滑开关、播放 1×/4×/16×、滑块、游标数值面板、总览统计卡片。

数据 schema：funsport.overlay/1，最小字段 points[].{t, v, cadence, stride_m}；坐标、经验式列、10 s 派生序列都可缺省自动补。

坐标系：generator 落盘的 gLat/gLng 是 **BD-09**（funsport/track/generator.py:669/717），而 Keep、高德、地图瓦片都是 **GCJ-02**。本起点两坐标系差 **868.5 m**，所以出图前必须用 funsport.coordinates.bd09_to_gcj02 转换，否则轨迹整体偏移、与 Keep 记录叠不上。tools/run_humanized_loop.py、tools/run_chain_route.py、tools/compare_keep.py 三个出图入口都已统一到 GCJ-02。

命令行：

    .venv\Scripts\python.exe -B tools\run_overlay_charts.py --input artifacts/humanized/overlay_all.json
    .venv\Scripts\python.exe -B tools\run_overlay_charts.py --selftest      （7 条路径自检）

验收状态：headless Chrome 实际渲染通过（截图 .test-tmp/lead/final_shot.png、.test-tmp/charts/_shot_after.png）；
归一模式与 DOM 交互用 node DOM stub 覆盖 27/27。唯一未做的是真人手工点选，属观感验证。
---

## 6. 测试与仓库卫生

- 基线（ec49382 未改动）：Ran 810 / failures=2 / errors=14。本轮最终验收（含全部新增用例）：**Ran 858 / failures=2 / errors=14**，即 810 + 48 条新增，16 条基线失败 ID 0 新增、0 消失（日志 .test-tmp/lead/ut_perlap.log）。14 个 error 清一色是 funsport/api/flow.py 的 SimpleNamespace 缺 freeze_run_time。此前一轮曾出现 Ran 857 / failures=3 / errors=19，多出的 1 failure + 5 error 来自未跟踪的 tests/test_road_joint_lifecycle.py 在特定发现顺序下的既有污染（单跑 26/26 全过）；本轮全量没有触发。
- 新增用例合计 48 条：tests/test_humanize_route.py 8（方角环圆滑、里程与闭合、偏离有界、原路往返分道与掉头弧半径、种子可复现、退化输入不崩、多圈逐圈重新拟人化、laps=1 与单圈入口一致）、tests/test_route_chain_rule.py 5（走完点位里程、0 里程、返程段不计入、_as_latlon 三种入参、GCJ→BD 命中环）、tests/test_chain_core.py 11（解析、圈数换算、抬高语义、轨迹折点、链路构建、退化腿跳过）、tests/test_cli_route_chain.py 11、tests/test_gui_route_export.py 10、tests/test_route_options.py 2（固定起点成为第 0 条腿与环原点、--auto 把步频交给经验式）、tests/test_server_time_error.py 1（预览开启拟人化后环被改写且仍贴高德道路）。
- 注意：未跟踪的研究模块 tests/test_road_joint_lifecycle.py 若被全量发现，会因 tools/road_joint_lifecycle.py 的源码指纹与导入污染制造额外 1 failure + 5 error（仓库既有问题，不计入本轮，也不纳入提交）。
- 本轮改了 3 处既有断言，都是**场景**而非阈值：
  - tests/test_route_geometry.py 的 zigzag/2/1/2001 改为 duration=640（配速 3.13 m/s）：新包络把该固定抽样的弦长从 10.205 m 降到 9.506 m，13.2 m 道路预算内已可建成；守卫不变，换到仍不可行的配速继续验证。
  - tests/test_road_joint_e0_diagnostic.py 的 fast 用例改为 dur=110（2.73 m/s），仍必须是 no_feasible_sample_in_gcj_road_budget。
  - tests/test_server_time_error.py 的预览断言：一个用例显式关掉 HUMANIZE_ROUTE 继续断言「传给生成器的是高德折线」，另新增一个用例断言拟人化开启后环被改写、首尾闭合、且每个顶点离高德道路不超过 25 m。
- 仓库卫生：artifacts/、.test-tmp/、data/ 整个目录已加入 .gitignore（生成产物可复跑；data/ 下是真实用户导出与本地轨迹，含 .gpx/.kml/.tcx，属私密数据）。
- 未提交：本分支的改动由 lead 统一提交；队友与 lead 的中间报告（docs/.status-*.md、.motion-report.md、.charts-report.md、.verify-humanized.md）是工作笔记，保持未跟踪。

---

## 7. 第二轮改造（2026-11-11）：摆动幅度、多圈分离、海拔接入

用户三条反馈（原话）：①「还是太直，中间的波动不够大，可以参考我自己的路线波动」；②「多次经过的路段还是太重叠，可以完全不重叠」；③「海拔可以纳入考量了……先看有没有经验公式，再根据我的实际数据微调」。

### 7.1 摆动：不整体放大，只补 20–50 m 一档

仪器 tools/measure_route_shape.py（新增），语料 = data/huawei 80 条 + data/keep 52 条，合计 179 347 m。做法：以窗口 W 的居中移动平均当中心线，取横向距离的标准差 rms_W，带通 = sqrt(rms_W2² − rms_W1²)；只统计自身 50 m 中轴曲率半径 >150 m 的「直路段」。

真实直路段带通 RMS（米）：10~15 0.12｜15~20 0.14｜**20~30 0.31｜30~40 0.39｜40~50 0.44**｜50~75 0.69｜75~100 1.05。

| 配置 | 20~30 | 30~40 | 40~50 | 50~75 | 75~100 |
|---|---|---|---|---|---|
| 旧默认 (1.20,0.80,0.50)/(8,16,30) | 0.245 | 0.325 | 0.400 | 0.807 | 0.898 |
| 零摆动地板（振幅全 0） | 0.226 | 0.279 | 0.323 | 0.628 | 0.723 |
| 新默认 (1.20,0.80,0.52,**0.26**)/(8,16,30,**55**) | 0.255 | 0.341 | 0.417 | 0.821 | 0.875 |
| 真实直路段 | 0.31 | 0.39 | 0.44 | 0.69 | 1.05 |

- 旧默认在 20–50 m 偏低 15–25%（这就是用户说的「中间波动不够大」）；新增一个 0.26 m / 27 m 波长的第 4 分量后收到 −18%/−13%/−5%。
- **零摆动地板说明 0.2–0.3 m 的细纹来自 lane/offset 场本身，不是摆动项**，所以不能靠整体加大振幅。
- 新增 WANDER_MIN_WAVE_M = 20.0：波长 = 环长/波数，短环上 55 波会退化成一米级锯齿，波数按 `wcap = max(1, int(total/20.0))` 封顶。
- 长波（100–400 m）不能靠摆动补：真实语料该档 RMS 13.2/27.7 m，而选出的路网环本身是 16.0/31.0 m，**已经比真人更弯** —— 那些频段测的是选路几何。

### 7.2 多圈分离：lap_lane_m = 2.4

先量真实：判据为两条轨迹点对弧长距离 >40 m、切向夹角 <25°（同向），71 条记录 767 个事件 —— **同向重访横向间距 p50 1.04 m / p90 2.63 m**（反向 p50 2.70 / p90 5.95）。也就是说真人多次经过同一路段，本来就只差 1~2.6 m。

实现：`_LAP_LANE_PATTERN = (0, +1, −1, +2, −2)` × `lap_lane_m`，第 0 圈恒为 0（保证 laps=1 与 humanize_route 逐点一致）。扇出挂在**掉头专用包络**上（LAP_TAPER_HI_M = 2.6、LAP_RAMP_M = 14）：不能乘主包络（转角处 env=0 会把扇出抹平），也不能脱离包络（斜率限幅会把 3 m 台阶糊到 4 m 掉头弧上，minR 掉到 0.32 m）。另加 lap_phase_jitter = 0.3 rad——两圈摆动差可达 ±2.4 m，会把扇出整个吃掉（实测 jitter 0 → 17.1%、0.3 → 9.7%、1.0 → 13.9%）。

**容差曲线（必须连容差一起读，funsport/track/route_metrics.py 默认 1.5 m）**：

| 扇出 | laps | 重复@1.5 m | @2.0 m | @3.0 m | 偏离 p50/p95 | minR | 最大转角 |
|---|---|---|---|---|---|---|---|
| 0.0（改前本质） | 3 | 66.45% | 66.62% | 67.88% | 1.82/3.90 | 1.265 | 26.20° |
| 2.0 | 3 | 11.55% | 36.68% | 56.76% | 1.91/4.96 | 1.260 | 26.27° |
| **2.4（采用）** | 2 | 4.58% | 7.36% | 47.94% | 2.82/5.72 | 1.266 | 24.77° |
| **2.4（采用）** | 3 | **9.92%** | **14.72%** | 56.37% | 1.91/5.33 | 1.261 | 26.01° |
| 2.6 | 3 | 9.92% | 11.15% | 54.55% | 1.95/5.52 | 1.263 | 26.31° |
| 3.0 | 3 | 9.37% | 10.27% | 38.56% | 2.08/5.90 | 1.255 | 25.34° |

取舍：选 2.4 m 而不是 2.6/3.0。真实同向 p90 只有 2.63 m，2.4 已落在真实分布内；再往上 tol=2.0 只从 14.72% 降到 11.15%（2.6）/10.27%（3.0），代价是偏离 p95 5.33→5.52→5.90 m、offset_max 4.46→5.02 m（轨迹肉眼离路），且 2.6 m 会撞已有契约 seam_shift_max_m < 5.0（实测 5.176）。**用 3 m 判「重合」比真人还严。**

两把尺子的差异要记住：route_metrics.route_overlap（项目自带，默认 1.5 m）与 tools/verify_humanized.repeat_corridors（走廊聚类，默认 2.0 m）在**同一份落盘轨迹**上给出 9.00% vs 16.54%（laps=3, 1.5 m）——报重复率必须带容差与仪器名。

### 7.3 minR 定论：抬到 1.3 m

最小半径顶点是 **180.0° 掉头圆弧**（laps=1 时 idx 496/983，邻接段 0.505/0.518 m = arc_sample_m 0.5）。标称 u_turn_radius_m = 1.2 经 0.6 m 重采样 + 7 位 BD 量化（≈1 cm）后，三点圆实测塌到 1.136–1.18 m，基线（1a1631d）就是 1.169–1.180，**不是本轮引入**。抬到 1.3 后实测 minR 1.267 / 1.266 / 1.261 m（laps=1/2/3），最大转角 26.12 / 24.77 / 26.01°、>45° 0 个、偏离不变。

扇出还引出一个必修：末圈留在起点 2×lap_lane 外，隐式闭合段在 200 m 方环上产生 49.7° 单点转角（原有测试 test_multi_lap_is_regenerated_per_lap 直接红）。已在 humanize_laps 尾部做反向 smoothstep 收敛，方环 turnmax 43.98° → 8.20°、minR 2.27 → 6.85。

### 7.4 海拔：第一次真正进入生成

改前：generator.build 早就支持 elevations_m，但 funsport/api/flow.py 与 chain.py **从不传**，生产海拔恒为 synthetic_baseline = 6.0 ± 1.0 m 常数；速度分配用 `w[i] *= exp(-2.0·grade)`，那个 −2.0 没有任何数据支撑。

实测（tools/measure_elevation_coupling.py，华为 80 条→71 条可用 / 9911 个 100 m 水平窗样本）：**真实场地几乎全平**——坡度 p5 = −0.10% / p50 = 0.00% / p95 = +0.17%，|grade| > 0.5% 仅 2.34%。记录内去均值 + 按记录聚类稳健 SE 的回归：ln(v) b = −5.67（t = −6.05，R²within = 0.0055）、ln(cad) t = −4.51、ln(stride) t = −2.94。**反证**：剔除最陡的 3 条记录后全部塌零（t = −0.08 / −0.23 / +0.09）；逐记录 ln(v) 只有 58% 为负。⇒ 结论是**弱耦合、数据不足以标定系数**，系数全部来自文献，未按实测放大（用户的直觉「耦合不多」被证实）。

方法学教训：坡度必须用**固定 100 m 水平距离窗口**的中央差分；早先用固定 ±15 s 时间窗，分母 Σv·dt 里含 v，会造出伪相关。

替换后的坡度律（realism.grade_speed_factor，Minetti 2002 上坡 Cr(i)/Cr(0) + Strava 下坡二次式，−10% 封顶）：

| 坡度 | 新（速度因子） | 旧 exp(−2·i) | 备注 |
|---|---|---|---|
| +1% | 0.9475（−5.25%） | 0.9802（−1.98%） | 旧只有文献的 38% |
| +2% | 0.8982（−10.2%） | 0.9608 | |
| +5% | 0.7684（−23.2%） | 0.9048（−9.5%） | |
| +10% | 0.6030（−39.7%） | 0.8187（−18.1%） | |
| −5% | 1.1082（+10.8%） | 1.1052 | |
| −10% | 1.1362（+13.6%） | 1.2214（+22.1%） | 官方 0.88 |
| ≤−10% | 封顶同 −10% | 1.4918（−20% 时） | 旧值离谱 |

量级标定（222.3 m/段，上坡半段 ÷ 平地半段做差比）：+2% 落盘 0.8970 vs 文献 0.8982（−0.1%）、+4% +0.9%、+6% +3.3% ⇒ 响应量级 == 文献。

接线：funsport/api/flow.py 与 chain.py 现在调用 realism.elevations_for_path(...)（闭环强制首尾一致，避开 generator.py 首尾高程差 >0.1 m 的 ValueError）。高程场由 `tools/measure_elevation_coupling.py --fit-site` 从真实 GPS 拟合，落盘 .funsport/site_elevation.json（.gitignore 已忽略 .funsport/），也支持环境变量 FUNSPORT_SITE_ELEVATION 注入；文件缺失时自动退回 synthetic_baseline。

端到端 before → after（3000 m / 3 圈 / seed 7）：elevationSource synthetic_baseline → route_vertices；累计爬升 0 → 4.8 m；逐点 |Δv| rms 0.0543 / max 0.1453；totalDistance 3000.0 不变；步数不变。

**必须如实告知用户的一点**：拟合出的场地高程场是**弱信号**——残差 rms 3.84 m，远大于环上真实高差 1.20 m，东向斜率 90% bootstrap 区间跨零。它的价值是「有依据、可复跑、非零」，不是「测出了这个场地的真实坡度」。

### 7.5 测试与验收

- 全量 `.venv\Scripts\python.exe -B -m unittest discover -s tests -t .`：**Ran 883 / failures=2 / errors=14**（上一轮基线 858/2/14，16 条失败 ID 逐条相同、0 新增 0 消失；新增 25 条用例来自 tests/test_humanize_shape.py 5 条与 tests/test_elevation_coupling.py 20 条）。日志 .test-tmp/lead/ut_final.log。
- 注意 tests/test_road_joint_control_lifecycle.py 的源码指纹在 import 时快照，**在自己改动 funsport/** 的同时跑全量，会额外报 implementation_changed_during_lifecycle**；判定回归必须「单跑通过 + 全量同 ID 复现」两条一起看。
- 端到端复跑：`.venv\Scripts\python.exe -B tools/run_humanized_loop.py --laps 1,2,3 --seed 7 --pace 360` → 环长 1478.9 → 拟人化 1473.6 m；laps 1/2/3 = 1473.6 / 2956.3 / 4416.5 m（接缝 0.62 m），转角 p90 2.02 / 1.70 / 1.80°、max 26.12 / 24.77 / 26.01°、>45° 0 个、minR 1.267 / 1.266 / 1.261 m、偏离 p50 1.86 / p95 3.88、v mean 2.79/2.78/2.78、cv 0.214/0.209/0.212、步频 167.9/167.1/166.9、步长 0.992/0.996/0.996 m。artifacts/humanized/track_laps{1,2,3}.json 与 route_overlay.html 已再生。
- 本轮改动文件：funsport/track/humanize.py（+82/−7）、funsport/track/realism.py、funsport/track/generator.py（海拔/坡度段）、funsport/api/flow.py、funsport/api/chain.py；新增 tools/measure_route_shape.py、tools/measure_elevation_coupling.py、tests/test_humanize_shape.py、tests/test_elevation_coupling.py、docs/.route-shape-empirics.md、docs/.route-shape-report.md、docs/.elevation-coupling.md。
- 仍未做（诚实清单）：真实手机 GPS 端到端误差无法复现；视觉观感需人工看图；3.0 m 以上扇出的路侧空间余量未测；步频随坡度的系数 grade_cadence_factor 目前**默认恒 1.0**（即只有速度受坡度影响，步频-坡度这条实测不支持，故未启用）。

### 7.7 圈与圈到底分开了多少：三把尺子 + 一个被推翻的中间结论（lead 实测）

设计值 lap_lane_m = 2.4 m，模式 (0, +2.4, −2.4)。**实测（.test-tmp/lead/lap_gap.py，直接调 humanize_laps 并用 linfo["lap_lengths_m"] 按弧长对齐）**，seed=7 / laps=3，lap_lengths_m = [1473.56, 1483.59, 1461.03]：

| 配对 | p05 | p50 | p95 | max |
|---|---|---|---|---|
| 第 2 圈 vs 第 1 圈 | 2.07 | **3.37** | 9.99 | 10.31 m |
| 第 3 圈 vs 第 1 圈 | 1.59 | **3.00** | 8.46 | 9.13 m |
| 第 2 圈 vs 第 3 圈 | — | 6.34 | 18.29 | — |

最差的 200 m 窗（s0=225 m）第 2 圈 vs 第 1 圈中位间距仍有 **2.43 m**。⇒ **分道是生效的**，实际横向间距中位 3.0~3.4 m，比设计值 2.4 m 还大一点（摆动相位不同又贡献了一部分）。

**中途出现过一次错误结论，必须记录在案**：验收图「多圈走廊」面板一度报出「三圈中位间距 0.87 / 0.89 / 0.02 m，第 2、3 圈几乎完全重合」。这是**测量错**，不是几何错，原因有两条：

1. artifacts/humanized/track_laps*.json 的 meta **没有 lap_lengths_m**（只有 laps / ring_length_m / dist_m / road_length_m / seam_shift_max_m / elevationSource），点里也没有圈号——面板只能自己猜圈边界；
2. 在自重叠环上用「全局最近点投影」会跳到另一支（该面板作者自己也记到过 331 m 假跳变），再叠上「按圈边界猜错」就得到 0.02 m。

修法：圈边界按长度比例切（[0, 1473.56, 2957.16, 4418.19] / 4418.19 × meta.dist_m），再用 dist_m 对齐取最近点。已开 task-15 让 chart-builder 重算或撤下该数字。

**「还有多重叠」的正确回答取决于尺子**（三把尺子量的是三件事，不能互相替代）：

| 尺子 | 量的东西 | laps=3 结果 |
|---|---|---|
| `route_overlap`（tol 2.0 m） | 近似共线（同向，角度门 5°）的重复里程占比 | 13.68% |
| `repeat_corridors`（tol 2.0 m） | 走廊聚类 | 21.77% |
| verifier 逐点最近邻（排除沿路 ±25 m） | 「有没有别的圈离我 <2 m」 | **38.0%**（<0.6 m 为 18.6%，即 822 m）；最近他圈距离 p5/p50/p95 = 0.174 / 2.17 / 2.71 m |

第三把尺子最保守，也最接近用户「会不会看成同一条线」的直觉。它的 p5 = 0.174 m 说明**确实存在贴在一起的地方**，且 verifier 定位出**残留重合的 49% 落在掉头 20 m 邻域**（该邻域只占全程 10.0%）——因为分道的包络在掉头锥形区被压回 0，三圈在掉头顶点必然汇合。这是当前的**已知热点**，不是随机分布。


### 7.6 重复路段的天花板：路网本身就没有零重叠环线（lead 实测）

用户诉求「多次经过路段完全不重叠」要分两层看，混在一起会得出错误结论：

**第一层：选路层（走哪几条路）**。对缓存里全部 24 个有效道路组合跑 `route_overlap`（tol 默认 1.0 m，脚本 .test-tmp/lead/sel_sweep.py）：

| 组合 | 环长 m | 自重叠加里程 m | 比例 |
|---|---|---|---|
| 6 | 1479.2 | 315.3 | **21.31%（最好）** |
| 13（seed 7 选中） | 1478.9 | 316.2 | 21.38% |
| 18 | 1507.9 | 329.5 | 21.85% |
| 8 / 15 / 20 | 1546~1575 | 421~449 | 27.2~28.5% |
| 12 | 1497.3 | 853.7 | 57.0% |
| 3（最差） | 1493.1 | 931.3 | 62.4% |

**24 个组合没有一个低于 21.3%**——这个校园路网里任何闭合环线都必须把某条干路走两遍（去/回）。所以「完全不重叠」在**道路身份**这一层不可能达成，只能把两次经过在**空间上分开**。原始环线在 tol 0.5/1.0/2.0/3.0 m 下都恰好是 316.16 m / 21.38%，说明重复段是**逐点精确重合**（同一条折线走两遍），不是近似平行。

**第二层：空间分离层（拟人化）**。同一把尺子（`route_overlap`，跑落盘 artifacts 的 GCJ 采样点，脚本 .test-tmp/lead/overlap_decomp.py）：

| tol | laps=1 | laps=2 | laps=3 |
|---|---|---|---|
| 0.5 m | 0.63% | 1.90% | 5.23% |
| 1.0 m | 1.05% | 2.96% | 8.28% |
| 2.0 m | 1.10% | 5.75% | 13.68% |
| 2.5 m | 2.40% | 29.34% | 38.55% |
| 3.0 m | 3.86% | 38.87% | 49.88% |

对照原始环线的 21.38%：拟人化把「另一次经过」推到 2 m 以外，laps=1 只剩 1.1%；但 **2.0→2.5 m 之间数字会跳一个台阶**，因为 lap_lane_m = 2.4 恰好卡在两档之间，圈间距就压在 2.4 m 附近。

**两条必须同时给出才诚实的注意事项**：

1. **仪器名 + 容差必须一起报**。同一批 artifacts 换 `tools/verify_humanized.repeat_corridors`（走廊聚类）更保守：tol 1.5/2.0/2.5 下 laps=1 = 3.12/3.63/6.24%、laps=2 = 7.13/10.10/38.21%、laps=3 = 16.54/21.77/43.75%。报「重复率」不写这两项，数字没有意义。
2. **`route_overlap` 对「反向重走」只认精确重合**（角度门 5°，只有逐点重合时方向无关）。原始环线因为是同一条折线走两遍所以全额计入；一旦被拟人化推开 1~2 m，反向那一遍就不再被计入。所以上表是**同向**重叠的可靠度量，不等于「看起来会不会像同一条线」——那要靠 repeat_corridors 或人眼。


