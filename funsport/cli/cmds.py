"""CLI 子命令实现。"""
import argparse
import json
import random
import subprocess
import sys
import time
from datetime import datetime, timedelta

from .mod import (
    make_client, logger, parse_flags, get, fmt_hms, now_ms, jstr, print_rows,
    parse_pace,
)
from ..api.client import ApiClient
from ..config import (
    load_identity, load_session, load_config, save_config,
    save_session, clear_session, get_amap_key, clear_loop_cache,
)
from ..logger import log, ok, err, warn, info


# ── init ────────────────────────────────────────────────────
def cmd_init(args):
    from . import init_cmd
    init_cmd.cmd_init(args)


# ── login / logout ─────────────────────────────────────────
def cmd_login(args):
    from ..api import login as api_login
    identity = load_identity()
    client = ApiClient(identity)

    # 优先用命令行参数，否则从 config.json 读取
    user = getattr(args, "user", None)
    pwd = getattr(args, "pass_", None)

    if not user or not pwd:
        cfg = load_config()
        cfg_user = (cfg.get("username") or "").strip()
        cfg_pass = (cfg.get("password") or "").strip()
        if not cfg_user or not cfg_pass:
            err("未提供账号密码，且 config.json 中没有保存的凭据")
            err("用法：python -m funsport login --user 手机号 --pass 密码")
            err("或先执行：python -m funsport init")
            sys.exit(1)
        user = user or cfg_user
        pwd = pwd or cfg_pass
        ok(f"从 config.json 读取账号：{user}")

    sess = api_login.login(client, user, pwd)

    if getattr(args, "remember", False):
        cfg = load_config()
        cfg["username"] = user
        cfg["password"] = pwd
        cfg["remember"] = True
        save_config(cfg)
        ok("凭据已保存")


def cmd_logout(args):
    """请求服务器退出并释放连接；失败保留会话并让 CLI 返回错误。"""
    from ..api import login as api_login
    sess = load_session()
    if not sess.get("uid") or not sess.get("token"):
        raise ValueError("没有可用会话，无法请求服务器退出；未执行本地清理")
    client = ApiClient(load_identity(), sess)
    try:
        result = api_login.logout(client)
        if not result["local_cleared"]:
            raise RuntimeError(result["message"])
    finally:
        client.http.close()


# ── points / policy ────────────────────────────────────────
def cmd_points(args):
    from ..api import points as api_points
    client = make_client()
    pts = api_points.fetch_points(client)
    log.info(f"共 {len(pts)} 个打卡点：")
    for i, p in enumerate(pts, 1):
        log.info(f"  {i:2}. {p.get('pointName',''):20s} "
                 f"BD=({p.get('lat')},{p.get('lon')})")


def cmd_policy(args):
    from ..api import policy as api_policy
    client = make_client()
    pol = api_policy.fetch_policy(client)
    print(f"policy       = {pol.policy}")
    print(f"min_distance = {pol.min_distance} m")
    print(f"valid_time   = {pol.valid_time}")
    print(f"timestamp    = {pol.timestamp}")


# ── 高德 ────────────────────────────────────────────────────
def cmd_lbs_amap(args):
    from ..config import set_amap_key
    set_amap_key(args.key)
    ok(f"高德 Key 已写入 config.json：{args.key[:8]}…")


def cmd_loop_rebuild(args):
    from ..api import points as api_points
    from ..api import campus_loop as api_loop
    client = make_client()
    clear_loop_cache()
    pts = api_points.fetch_points(client)
    ring = api_loop.get_campus_loop(pts, force_rebuild=True)
    ok(f"环已重建：{len(ring)} 点 → .funsport/campus_loop_bd.json")


# ── config ──────────────────────────────────────────────────
def cmd_config(args):
    cfg = load_config()
    if args.set:
        from ..config import set_config
        updates = {}
        for pair in args.set:
            if "=" not in pair:
                continue
            k, v = pair.split("=", 1)
            updates[k.strip()] = v.strip()
        n = set_config(**updates)
        ok(f"已更新 {n} 项")
        return
    for k in sorted(cfg.keys()):
        v = cfg[k]
        if k == "password" and v:
            v = v[:2] + "***"
        print(f"{k:24s} = {v}")


# ── run ─────────────────────────────────────────────────────
def cmd_run(args):
    """运行现有跑步链，并区分提交完成与详情、打卡点检查状态。"""
    from ..api import flow
    client = make_client()

    before = getattr(args, "before", 0) or 0
    allow_outside = getattr(args, "allow_outside", False)

    if args.time:
        h, m = args.time.split(":")
        base = datetime.now() - timedelta(days=args.days_ago)
        dt = base.replace(hour=int(h) % 24, minute=int(m) % 60,
                          second=0, microsecond=0)
        start_ms = int(dt.timestamp() * 1000)
    elif args.ago:
        start_ms = int(time.time() * 1000) - args.ago * 60_000
    else:
        start_ms = 0

    tag = " / AUTO" if args.auto else (" / 真实路径" if args.use_map else "")
    log.info(f"参数：{tag.strip() or '手动'}")

    result = flow.run_full_flow(
        client,
        dist=args.dist or 0,
        pace=args.pace or 0,
        cadence=args.cadence or 0,
        start_ms=start_ms,
        face_check=1 if args.face else 0,
        seed=args.seed,
        use_map=args.use_map,
        auto=args.auto,
        dist_min=args.dist_min,
        dist_max=args.dist_max,
        pace_min=args.pace_min,
        pace_max=args.pace_max,
        cadence_min=args.cadence_min,
        cadence_max=args.cadence_max,
        min_dist=args.min_dist,
        before=before,
        allow_outside_window=allow_outside,
    )
    print()
    ok(f"记录已提交 rrid={result['rrid']} uuid={result['uuid']}")
    ok(f"距离 {result['dist']:.0f}m / 时长 {result['dur']}s / "
       f"平均步频 {result['cadence_avg']:.0f}spm")
    info(f"OBS {result['obs_ok']}/2 · 详情读取 {'成功' if result['detail_ok'] else '失败'}")
    info("打卡点是否显示仍需以官方 App 为准，详情读取成功不代表打卡验证通过")


# ── route ───────────────────────────────────────────────────
DEFAULT_ROUTE_JSON = "artifacts/humanized/cli_route.json"
DEFAULT_ROUTE_HTML = "artifacts/humanized/cli_route.html"
DEFAULT_TARGET_M = 2000.0


def _route_target_m(args):
    """--target（米）优先；否则 --dist 按 run 的口径换算：<100 当 km，>=100 当米。"""
    target = getattr(args, "target", None)
    if target is not None and float(target) > 0:
        return float(target)
    dist = float(getattr(args, "dist", 0) or 0)
    if dist > 0:
        return dist * 1000.0 if dist < 100 else dist
    return DEFAULT_TARGET_M


def _fmt_num(value, digits=3):
    """统计量样本不足时是 None，打印占位符而不是在格式化时抛异常。"""
    return "-" if value is None else f"{float(value):.{digits}f}"


def cmd_route(args):
    """按链路规则生成轨迹并写出 overlay，仅本地生成，不提交、不访问运动接口。"""
    from ..api import chain

    try:
        pace = float(getattr(args, "pace", 0) or 0)
        if pace < 160 or pace > 520:
            print(f"错误：配速需在 160~520 秒每公里之间（当前 {pace:.0f}）")
            return 1
        point_texts = list(getattr(args, "point", None) or [])
        if point_texts:
            points = [chain.parse_lnglat(text) for text in point_texts]
        else:
            points = chain.points_from_cache_gcj()
        start_text = getattr(args, "start", None)
        start = chain.parse_lnglat(start_text) if start_text else points[0]
        target_m = _route_target_m(args)
        cadence = float(getattr(args, "cadence", 0) or 0)
        seed = int(getattr(args, "seed", 7))
        start_ms = int(getattr(args, "start_ms", 0) or 0)

        print(f"起点 GCJ {start[0]:.6f},{start[1]:.6f}，点位 {len(points)} 个，"
              f"目标 {target_m:.0f} m，配速 {pace:.0f} s/km")
        result = chain.build_chain_route(
            start, points, target_m=target_m, pace_s=pace, cadence=cadence,
            seed=seed, start_ms=start_ms, progress=print)
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"错误：{exc}")
        return 1

    meta = result.get("chain") or {}
    stats = result.get("stats") or {}
    requested = float(meta.get("requested_m") or target_m)
    target = float(meta.get("target_m") or target_m)
    raised = bool(meta.get("raised_to_visit"))
    laps = meta.get("laps") or result.get("laps") or 0

    print(f"链路长度（起点 -> 点位 -> 返程）：{float(meta.get('lap_len_m') or 0.0):.1f} m")
    print(f"走完点位里程：{float(meta.get('visit_all_points_m') or 0.0):.1f} m")
    if raised:
        visit = float(meta.get("visit_all_points_m") or target)
        print(f"目标 {requested:.0f} m 短于走完所有点位所需的 {visit:.0f} m，按 {visit:.0f} m 生成")
    else:
        print(f"目标里程：{target:.0f} m")
    print(f"规划圈数（链路重复遍数）：{laps}")
    print(f"实际里程 {float(result.get('dist_m') or 0.0):.1f} m / "
          f"时长 {float(result.get('dur_s') or 0.0):.0f} s / "
          f"点数 {len(result.get('points') or ())}")
    print(f"终点离起点距离：{float(meta.get('end_offset_m') or 0.0):.1f} m")
    print(f"速度均值 {_fmt_num(stats.get('v_mean'), 2)} m/s，"
          f"cv {_fmt_num(stats.get('v_cv'), 3)}")
    print(f"步频 {_fmt_num(stats.get('cadence_mean'), 1)} spm，"
          f"步长 {_fmt_num(stats.get('stride_mean'), 3)} m")
    print(f"最长低速连段 {stats.get('max_low_speed_run', '-')} 点，"
          f"步频弹性 {_fmt_num(stats.get('cadence_elasticity'), 3)}")

    json_path = getattr(args, "json", "") or DEFAULT_ROUTE_JSON
    html_path = None
    if not getattr(args, "no_html", False):
        html_path = getattr(args, "html", "") or DEFAULT_ROUTE_HTML
    title = getattr(args, "title", None) or None
    try:
        out_json, out_html = chain.write_overlay(result, json_path, html_path,
                                                 title=title, progress=print)
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"错误：{exc}")
        return 1
    print(f"JSON：{out_json}")
    if out_html:
        print(f"HTML：{out_html}")
    return 0


# ── AI ──────────────────────────────────────────────────────
def cmd_ai_list(args):
    from ..api import ai as api_ai
    client = make_client()
    lst = api_ai.fetch_list(client)
    for s in lst:
        print(f"id={s['id']:<5} {s['name']}")


def cmd_ai(args):
    from ..api import ai as api_ai
    client = make_client()
    if args.mode == "count":
        mode = api_ai.AiMode.count(args.score)
    else:
        mode = api_ai.AiMode.minutes(args.score)
    api_ai.upload(client, args.sport, mode)


def cmd_ai_records(args):
    from ..api import ai as api_ai
    client = make_client()
    page = api_ai.fetch_records(client, args.sport, 50)
    for g in page["groups"]:
        d = datetime.fromtimestamp(g["score_date"] / 1000).strftime("%Y-%m-%d")
        print(f"{d} ×{g['frequency']}：")
        for r in g["records"]:
            if r["type"] == 2:
                grade = f"{float(r['score'] or 0) / 1000:.1f} 秒"
            else:
                grade = f"{r['score']} 个"
            print(f"  {r['name']} {grade}")


def cmd_ai_info(args):
    from ..api import ai as api_ai
    client = make_client()
    d = api_ai.fetch_record_detail(client, args.id)
    print(json.dumps(d, ensure_ascii=False, indent=2))


# ── records ─────────────────────────────────────────────────
def cmd_records(args):
    from ..api import records as api_records
    client = make_client()
    rows = api_records.fetch_records(client)
    print_rows(rows)


def cmd_record_info(args):
    from ..api import records as api_records
    client = make_client()
    d = api_records.fetch_one_record(client, args.rrid)
    print(json.dumps(d, ensure_ascii=False, indent=2))


# ── semester / cheat / rank ─────────────────────────────────
def cmd_semester(args):
    from ..api import semester as api_semester
    client = make_client()
    r = api_semester.query(client)
    s = r["summary"]
    print(f"学期：{s.sname}")
    print(f"  有效次数：{s.semester_valid_count}/{s.semester_count}")
    print(f"  有效里程：{s.semester_valid_dis/1000:.2f} km")


def cmd_cheat(args):
    from ..api import cheat as api_cheat
    client = make_client()
    r = api_cheat.query(client, args.page)
    if r.is_clean():
        print("自查：干净")
    else:
        print(f"已被标记：{r.self_brief()}")
    print(f"全校违规 {len(r.list)} 条")
    for it in r.list[:20]:
        print(f"  {it.get('name','')} | {it.get('reason','')}")


def cmd_rank(args):
    from ..api import rank as api_rank
    client = make_client()
    if args.kind == "indoor":
        rows = api_rank.indoor_rank(client, args.range or 1)
    elif args.kind == "history":
        rows = api_rank.history_rank(client, args.sort or 1)
    else:
        rows = api_rank.main_rank(client, args.type or 1, args.sort or 1)
    for r in rows:
        print(f"{r['sort']:<4} {r['name']}  {r['length']/1000:.2f} km")
