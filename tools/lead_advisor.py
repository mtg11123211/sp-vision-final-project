#!/usr/bin/env python3
"""lead_advisor.py — 把探针 CSV 变成「DSH_LEAD 该设多少」以及「设准到多少才打得中」。

背景（为什么需要它）：
    3 m 外一块 13.5 cm 宽的装甲板，张角只有 2.6°。靶机以 ω 旋转时，整块板从准星下
    通过的时间是 2.6°/ω：
        ω = 3 rad/s (172°/s)  →  15 ms
        ω = 5 rad/s (286°/s)  →   9 ms
        ω = 7 rad/s (401°/s)  →   6 ms
    也就是说 lead 只差几毫秒，弹着点就滑出板外。手调（0.00 / 0.05 / 0.10 / 0.15）的
    步长是 50 ms，在 5 rad/s 下相当于 14°，差了一个数量级 —— 所以「调了几档还是空」
    不是手法问题，是分辨率问题。

它算什么：
    lead_total = 视觉侧延迟 + 云台纯延迟 + 云台一阶时间常数
      视觉侧延迟  (send_ns - grab_ns)          ← 普通跑车数据就能给
      云台纯延迟  D   （阶跃后 fb 开始动）     ← 需要 DSH_STEP_DEG 那次的数据
      云台时间常数 τ  （fb 走到 63%）          ← 同上
    跟踪一条匀速斜坡时，一阶系统的稳态滞后正好是 τ，所以三项直接相加。

用法：
    py tools/lead_advisor.py logs/lat_run1.csv
    py tools/lead_advisor.py logs/lat_step.csv          # 阶跃那次，会同时给出 D 和 τ
    py tools/lead_advisor.py logs/lat_step.csv --dist 3

只依赖标准库。Windows 控制台先 chcp 65001 且 $env:PYTHONIOENCODING='utf-8'。
"""

import argparse
import csv
import math
import statistics
import sys

BOARD_SMALL_W = 0.135  # 小装甲板宽（米）
BOARD_LARGE_W = 0.225  # 大装甲板宽（米）

COLS = (
    "dev_ts", "grab_ns", "infer_ns", "send_ns",
    "cmd_yaw", "cmd_pitch", "fb_yaw", "fb_pitch", "fire", "dist",
)


def load(path):
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in COLS if c not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"不是探针 CSV：缺列 {missing}\n表头是 {reader.fieldnames}")
        rows = []
        for r in reader:
            try:
                rows.append({k: float(r[k]) for k in COLS})
            except (TypeError, ValueError):
                continue  # 半行/坏行直接丢
    if not rows:
        raise SystemExit(f"{path} 里没有可用数据行")
    return rows


def pct(xs, p):
    s = sorted(xs)
    if not s:
        return float("nan")
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def vision_latency(rows):
    grab = [r["grab_ns"] for r in rows]
    v = [(r["send_ns"] - r["grab_ns"]) / 1e6 for r in rows]
    inf = [(r["infer_ns"] - r["grab_ns"]) / 1e6 for r in rows]
    ser = [(r["send_ns"] - r["infer_ns"]) / 1e6 for r in rows]
    dts = [(grab[i] - grab[i - 1]) / 1e6 for i in range(1, len(grab)) if grab[i] > grab[i - 1]]
    return {
        "n": len(rows),
        "fps": (1000.0 / statistics.median(dts)) if dts else float("nan"),
        "total": v,
        "infer": inf,
        "serial": ser,
        "fires": sum(1 for r in rows if r["fire"] > 0.5),
        "dist": [r["dist"] for r in rows if r["dist"] > 0],
    }


def step_fit(rows):
    """从 ±degree 方波里量出云台纯延迟 D 与一阶时间常数 tau。

    按「找 5%/63% 交点」量会掉进一个坑：58 fps 下每帧 17 ms，D 和 tau 只能读到
    一帧的分辨率，而打中旋转靶需要毫秒级。所以这里改成对整段响应做最小二乘拟合
    p(t) = 1 - exp(-(t-t0)/tau)，t0 与 tau 在网格上搜，能给出亚帧分辨率。
    """
    ys = [r["cmd_yaw"] for r in rows]
    fbs = [r["fb_yaw"] for r in rows]
    ts = [r["grab_ns"] / 1e9 for r in rows]

    diffs = [abs(ys[i] - ys[i - 1]) for i in range(1, len(ys))]
    if not diffs:
        return None
    med = statistics.median(diffs)
    amp = max(ys) - min(ys)
    thr = max(6.0 * med, 0.25 * amp, 1e-4)

    edges = [i for i in range(1, len(ys)) if abs(ys[i] - ys[i - 1]) > thr]
    if len(edges) < 2:
        return {"edges": edges, "D": None, "tau": None, "t90": None, "overshoot": None, "rms": None}

    Ds, taus, rmses, overs = [], [], [], []
    for i in edges:
        A, B = ys[i - 1], ys[i]
        span = B - A
        if abs(span) < 1e-6:
            continue
        stop = len(ys)
        for k in range(i + 1, len(ys)):
            if abs(ys[k] - ys[k - 1]) > thr:
                stop = k
                break
        seg = [j for j in range(i, min(stop, i + 220))]
        if len(seg) < 5:
            continue
        pts = [((ts[j] - ts[i]), (fbs[j] - A) / span) for j in seg]
        overs.append(max(p for _, p in pts) - 1.0)

        best = None
        t0_grid = [d * 0.001 for d in range(-10, 121, 2)]
        tau_grid = [d * 0.001 for d in range(4, 151, 2)]
        for t0 in t0_grid:
            for tau in tau_grid:
                err = 0.0
                for dt_, p in pts:
                    if dt_ < t0:
                        model = 0.0
                    else:
                        model = 1.0 - math.exp(-(dt_ - t0) / tau)
                    err += (p - model) ** 2
                if best is None or err < best[0]:
                    best = (err, t0, tau)
        if best is None:
            continue
        err, t0, tau = best
        # 再在最优附近细化一格
        for t0b in (t0 - 0.0015 + 0.0005 * k for k in range(4)):
            for taub in (tau - 0.0015 + 0.0005 * k for k in range(4)):
                if taub <= 0:
                    continue
                e = 0.0
                for dt_, p in pts:
                    model = 0.0 if dt_ < t0b else 1.0 - math.exp(-(dt_ - t0b) / taub)
                    e += (p - model) ** 2
                if e < err:
                    err, t0, tau = e, t0b, taub
        Ds.append(max(0.0, t0))
        taus.append(tau)
        rmses.append(math.sqrt(err / max(1, len(pts))))

    if not Ds:
        return {"edges": edges, "D": None, "tau": None, "t90": None, "overshoot": None, "rms": None}
    d_med = statistics.median(Ds)
    tau_med = statistics.median(taus)
    return {
        "edges": edges,
        "D": d_med,
        "tau": tau_med,
        "t90": d_med + 2.302585 * tau_med,
        "overshoot": statistics.median(overs) if overs else None,
        "rms": statistics.median(rmses) if rmses else None,
    }


def fmt_ms(x):
    return "  n/a " if x is None or math.isnan(x) else f"{x * 1000:7.1f}"


def main():
    ap = argparse.ArgumentParser(description="从探针 CSV 推算 DSH_LEAD 与命中容差")
    ap.add_argument("csv", help="DSH_LAT_CSV 输出的那个文件")
    ap.add_argument("--dist", type=float, default=3.0, help="靶机距离（米），默认 3")
    ap.add_argument("--board", type=float, default=BOARD_SMALL_W, help="装甲板宽度（米），默认 0.135")
    args = ap.parse_args()

    rows = load(args.csv)
    vis = vision_latency(rows)
    st = step_fit(rows)

    print("=" * 66)
    print(f"文件      : {args.csv}")
    print(f"帧数      : {vis['n']}   帧率 ≈ {vis['fps']:.1f} fps   开火帧 {int(vis['fires'])}")
    if vis["dist"]:
        print(f"目标距离  : 中位数 {statistics.median(vis['dist']):.2f} m（解算值，非设定值）")

    print("\n[1] 视觉侧：取到帧 → 串口写完")
    for name, xs in (("取帧→解算完", vis["infer"]), ("解算完→串口", vis["serial"]), ("取帧→串口", vis["total"])):
        if xs:
            print(f"    {name:<10} p50 {pct(xs,50):6.1f} ms   p95 {pct(xs,95):6.1f} ms   max {max(xs):6.1f} ms")
    l_vis = pct(vis["total"], 50) / 1000.0 if vis["total"] else 0.0

    lead = l_vis
    frame = (1.0 / vis["fps"]) if vis["fps"] and not math.isnan(vis["fps"]) else 0.017
    unc = max(0.5 * frame, 0.010)
    print("\n[2] 云台侧：阶跃响应")
    if st is None:
        print("    没有 cmd 变化，无法判断")
    elif st["D"] is None:
        print(f"    只找到 {len(st['edges'])} 个沿（需要 ≥2），这段数据不是阶跃测试")
        print("    → 想分开「纯延迟」和「时间常数」，得跑一次 DSH_STEP_DEG=3")
    else:
        d_note = "（0 表示小于一帧，被 τ 吸收了，属正常）" if st["D"] < frame else ""
        print(f"    纯延迟 D   {fmt_ms(st['D'])} ms{d_note}")
        print(f"    时间常数 τ {fmt_ms(st['tau'])} ms   （一阶爬升到 63%，跟踪斜坡时稳态滞后 = τ）")
        print(f"    T90        {fmt_ms(st['t90'])} ms")
        print(f"    超调       {st['overshoot'] * 100:+.1f} %" if st["overshoot"] is not None else "    超调       n/a")
        if st["rms"] is not None:
            print(f"    拟合残差   {st['rms'] * 100:.1f} %（>15% 说明不是一阶响应，D+τ 这套模型不成立）")
        print(f"    分辨率     ±{unc * 1000:.0f} ms —— {vis['fps']:.0f} fps 下一帧 {frame * 1000:.0f} ms，"
              "D 与 τ 无法分得更细；但**两者的和**才是 lead 要用的量")
        lead += (st["D"] or 0.0) + (st["tau"] or 0.0)

    print("\n[3] 建议")
    print(f"    DSH_LEAD ≈ {lead * 1000:.0f} ms   ( = 视觉 {l_vis * 1000:.0f}"
          + (f" + D {st['D'] * 1000:.0f} + τ {st['tau'] * 1000:.0f}" if st and st["D"] is not None else "")
          + f" )   不确定度 ±{unc * 1000:.0f} ms")
    if st is None or st["D"] is None:
        print("    注意：这是**下界**，少算了云台的 D + τ，实际值一定更大。")
    print(f"    换算成度：3 rad/s → {lead * 3 * 57.2958:.1f}°   5 rad/s → {lead * 5 * 57.2958:.1f}°   7 rad/s → {lead * 7 * 57.2958:.1f}°")

    # ---- 容差表 ----
    half = math.atan((args.board / 2.0) / args.dist)
    print("\n[4] 命中容差（这就是为什么必须按毫秒调）")
    print(f"    {args.dist:.1f} m 处 {args.board * 1000:.0f} mm 板：张角 {2 * half * 57.2958:.2f}°，"
          f"半宽 {half * 57.2958:.2f}°")
    print(f"    {'ω (rad/s)':>10} {'整板通过':>10} {'打中需 |Δlead| <':>18}")
    for w in (3.0, 4.0, 5.0, 6.0, 7.0, 9.0):
        print(f"    {w:10.1f} {2 * half / w * 1000:9.1f}ms {half / w * 1000:16.1f}ms")

    print("\n    当前设定下，lead 偏差造成的残差：")
    print(f"    {'Δlead':>8} " + "".join(f"{w:>9.1f}" for w in (3.0, 5.0, 7.0)) + "   单位：度")
    for d_ms in (2, 5, 10, 20, 50):
        d = d_ms / 1000.0
        print(f"    {d_ms:6d}ms " + "".join(f"{w * d * 57.2958:9.1f}" for w in (3.0, 5.0, 7.0))
              + f"   (板宽 {2 * half * 57.2958:.1f}°)")

    print("\n[5] 下一步")
    print("    1) 把 DSH_LEAD 设成 [3] 的值，先打 3 发静止靶 —— 静止靶不吃 lead，")
    print("       打不中说明是 pitch 通道（符号 / offset），跟这个数无关。")
    print("    2) 静止靶能中，再打旋转靶；此时每差 5 ms 就差上表里的残差。")
    print("    3) 微调步长不要超过 [4] 的「打中需 |Δlead|」，否则一定跳过头。")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
