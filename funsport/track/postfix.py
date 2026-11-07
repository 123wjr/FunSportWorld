"""轨迹后处理。"""
from .geom import round_to, fmt_gain_time


def apply_post_fixes(locs, rng, start_ms, ordered_path=False):
    """有序规划只标终点；旧拟合模式保留原后处理，不伪造规划段的事件。"""
    if ordered_path:
        if locs:
            locs[-1]["type"] = 6
        return
    # 末点之前插入一段「跑者停下」的停留时间会把采样空档拉长到 12~16s：
    # 旧实现在末个有效点上做 totalTime += randint(6,14)，这个点本来还带着
    # 自己的前一个间隔，追加后既超出 MAX_SAMPLE_GAP，也让 Σdt 大于 totalTime
    # （实测 30 例全部 Σdt 比总时长多 6~12s）。停下的语义只需要传感器读数
    # 静下来（bdS 掉到静止量级），不需要凭空占用时间轴，因此这里只改读数。
    if len(locs) >= 4:
        ci = len(locs) - 1
        while ci > 0 and locs[ci]["type"] == -1:
            ci -= 1
        if ci > 1:
            locs[ci]["bdS"] = round_to(rng.uniform(0.05, 0.15), 3)

    if len(locs) > 2:
        # 起点标记就是第 0 点：它的 totalTime 已经是 0。旧实现把第 1 点改写成
        # 起点标记并清零它的 totalTime，等于把第 0 个采样的时间抹掉，于是
        # 「标记(0s) → 第 2 点」之间凭空出现 8s 的间隔（实测 seed21 maxgap 8）。
        sp = locs[0]
        sp.update({"type": 5, "state": 1, "locType": 1,
                   "radius": round_to(rng.uniform(2.0, 3.5), 2),
                   "speed": 0.0, "avgSpeed": 0.0, "bdS": 0.0,
                   "totalDis": 0.0, "validDis": 0.0,
                   "totalTime": 0, "validTime": 0, "steps": 0,
                   "gainTime": fmt_gain_time(start_ms), "gainTimeMs": start_ms})
        last = len(locs) - 1
        locs[last].update({"type": 6, "locType": 1,
                           "radius": round_to(rng.uniform(1.5, 3.0), 2)})
        frm = int(len(locs) * 0.67)
        for i in range(frm, max(frm, len(locs) - 2)):
            if rng.random() < 0.06:
                locs[i]["type"] = rng.choice([1, 1, 2, 8])
                locs[i]["locType"] = 1

    if locs and locs[0]["type"] != 5:
        locs[0].update({"totalTime": 0, "validTime": 0, "state": 1,
                        "type": rng.choice([0, 0, 0, 7]), "locType": 1,
                        "radius": round_to(rng.uniform(1.5, 3.0), 2),
                        "gainTime": fmt_gain_time(start_ms), "gainTimeMs": start_ms})
