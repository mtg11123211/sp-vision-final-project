#!/usr/bin/env python3
"""录包 frames.csv 的离线分析（不依赖车上环境，只读 CSV）。

CSV 的列由 tools/recorder.cpp 固定为：
    # frame,t_sec,qw,qx,qy,qz,bullet_speed,bullet_count,mode

其中：
    qw..qz        曝光时刻云台姿态（四元数，wxyz 顺序）
    bullet_speed  电控上行弹速，0 表示这包没带（gimbal.cpp 里 state_.bullet_speed）
    bullet_count  电控上行累计发弹数，最准的发弹证据
    mode          电控上行挡位，0=IDLE 1=AUTO_AIM 2=SMALL_BUFF 3=BIG_BUFF

回答的问题（按现场最急的顺序）：
    1. 到底有没有真发弹        -> bullet_count 有没有涨
    2. 电控当时在什么挡位      -> mode 分段
    3. 云台实际转多快          -> 由相邻两帧四元数算测地角速度 |ω|
    4. 帧率/抖动/丢行          -> dt 分布
    5. 电控有没有报弹速        -> bullet_speed 非零比例

算不出来的：落点和"该提前多少"。CSV 里没有目标位置、也没有我们发出去的角度，
所以 §11.4 说的"用 w 和弹速反算提前量"只靠这个 CSV 做不到，
只能拿角速度去卡"外推帧开火"的误差上界（w × 外推时间）。

用法：
    py tools/analyze_frames.py recordings/synthetic_static
    py tools/analyze_frames.py ~/recordings              # 递归找所有 frames.csv
    py tools/analyze_frames.py all_csv_1017_1535.tar     # 直接读车上打回来的包
    py tools/analyze_frames.py <dir> --markdown          # 出 markdown 表，方便贴文档
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import tarfile

GEAR_NAMES = {0: "IDLE", 1: "AUTO_AIM", 2: "SMALL_BUFF", 3: "BIG_BUFF"}
COLS = ["frame", "t_sec", "qw", "qx", "qy", "qz", "bullet_speed", "bullet_count", "mode"]


# ---------------------------------------------------------------- 读数据

def parse_frames(text, name="frames.csv"):
    """把 CSV 文本解析成元组列表，坏行跳过并计数。"""
    rows, bad = [], 0
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < len(COLS):
            bad += 1
            continue
        try:
            frame = int(float(parts[0]))
            t = float(parts[1])
            q = tuple(float(x) for x in parts[2:6])
            speed = float(parts[6])
            count = int(float(parts[7]))
            mode = int(float(parts[8]))
        except ValueError:
            bad += 1
            continue
        rows.append((frame, t, q, speed, count, mode))
    return rows, bad


def load_sources(path):
    """把 path 展开成 [(名字, frames, 图片目录或 None)]。

    path 可以是：单个 frames.csv / 一个录包目录 / 上级目录（递归）/
    .tar/.tar.gz（车上打回来的包，里面只有 csv 没有图）。
    """
    out = []

    if os.path.isfile(path) and tarfile.is_tarfile(path):
        with tarfile.open(path) as tf:
            for m in tf.getmembers():
                if not m.isfile() or not m.name.endswith("frames.csv"):
                    continue
                f = tf.extractfile(m)
                if f is None:
                    continue
                text = f.read().decode("utf-8", errors="replace")
                rows, bad = parse_frames(text, m.name)
                out.append((m.name, rows, bad, None))
        return out

    if os.path.isfile(path):
        with open(path, encoding="utf-8", errors="replace") as f:
            rows, bad = parse_frames(f.read(), path)
        return [(path, rows, bad, os.path.dirname(os.path.abspath(path)))]

    if os.path.isdir(path):
        found = []
        for root, _dirs, files in os.walk(path):
            if "frames.csv" in files:
                found.append(os.path.join(root, "frames.csv"))
        found.sort()
        for csv_path in found:
            with open(csv_path, encoding="utf-8", errors="replace") as f:
                rows, bad = parse_frames(f.read(), csv_path)
            rel = os.path.relpath(csv_path, path)
            # 直接传一个录包目录时，名字用目录名，别只显示拿不出上下文的 frames.csv
            name = os.path.basename(os.path.abspath(path)) if rel == "frames.csv" else rel
            out.append((name, rows, bad, os.path.dirname(csv_path)))
        return out

    raise SystemExit(f"找不到 {path}")


# ---------------------------------------------------------------- 姿态

def quat_to_euler_deg(q):
    """wxyz 四元数 -> (yaw, pitch) 度，只用来显示范围，不做控制。"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return 0.0, 0.0
    w, x, y, z = w / n, x / n, y / n, z / n
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    return math.degrees(yaw), math.degrees(pitch)


def geodesic_rad(qa, qb):
    """两个四元数之间的转角（rad），用 atan2 保证小角度也稳定。"""
    wa, xa, ya, za = qa
    wb, xb, yb, zb = qb
    # dq = conj(qa) * qb
    dw = wa * wb + xa * xb + ya * yb + za * zb
    dx = wa * xb - xa * wb - (ya * zb - za * yb)
    dy = wa * yb - ya * wb - (za * xb - xa * zb)
    dz = wa * zb - za * wb - (xa * yb - ya * xb)
    v = math.sqrt(dx * dx + dy * dy + dz * dz)
    return 2.0 * math.atan2(v, abs(dw))


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def segments(modes, times):
    """把挡位压成时间分段，短于 0.15 s 的抖动并到上一段。"""
    if not modes:
        return []
    segs = []
    cur = modes[0]
    t0 = times[0]
    for m, t in zip(modes[1:], times[1:]):
        if m != cur:
            segs.append((t0, t, cur))
            cur, t0 = m, t
    segs.append((t0, times[-1], cur))
    merged = []
    for a, b, m in segs:
        if merged and (b - a) < 0.15 and merged[-1][2] == m:
            merged[-1] = (merged[-1][0], b, m)
        elif merged and (b - a) < 0.15:
            merged[-1] = (merged[-1][0], b, merged[-1][2])
        else:
            merged.append((a, b, m))
    return merged


# ---------------------------------------------------------------- 指标

def analyze(name, rows, bad, img_dir=None):
    m = {"name": name, "rows": len(rows), "bad_lines": bad}
    if len(rows) < 2:
        m["error"] = "有效行不足 2 行，没法算时域指标"
        return m

    t = [r[1] for r in rows]
    q = [r[2] for r in rows]
    speed = [r[3] for r in rows]
    count = [r[4] for r in rows]
    mode = [r[5] for r in rows]

    m["duration"] = t[-1] - t[0]
    m["monotonic"] = all(b >= a for a, b in zip(t, t[1:]))

    dts = [b - a for a, b in zip(t, t[1:])]
    good = [d for d in dts if d > 1e-6]
    m["dt_median"] = statistics.median(good) if good else float("nan")
    m["dt_p95"] = pct(good, 95)
    if good and m["dt_median"] > 0:
        m["fps"] = 1.0 / m["dt_median"]
        big = 3.0 * m["dt_median"]
        m["gaps"] = sum(1 for d in dts if d > big)
    else:
        m["fps"] = float("nan")
        m["gaps"] = 0

    # 弹速：电控没接上时会一直是 0，这正是现场"兜底值 25"的来源
    nz = [s for s in speed if s > 1e-6]
    m["speed_nonzero_ratio"] = len(nz) / len(speed)
    m["speed_mode"] = statistics.median([round(s, 3) for s in nz]) if nz else 0.0

    # 发弹：累计计数差，按 uint16 回绕处理
    rounds, incs, first_inc = 0, 0, None
    for a, b, tt in zip(count, count[1:], t[1:]):
        d = (b - a) % 65536
        if d:
            rounds += d
            incs += 1
            if first_inc is None:
                first_inc = tt - t[0]
    m["rounds"] = rounds
    m["count_start"], m["count_end"] = count[0], count[-1]
    m["count_steps"] = incs
    m["first_round_at"] = first_inc

    # 姿态与角速度
    angles = [quat_to_euler_deg(x) for x in q]
    yaws = [angles[0][0]]
    for a in angles[1:]:
        prev = yaws[-1]
        d = a[0] - (prev % 360.0)
        # 展开 ±180° 的跳变，否则绕着转的录包会显示 -179..+179 这种假范围
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        yaws.append(prev + d)
    m["yaw_range"] = (min(yaws), max(yaws))
    m["pitch_range"] = (min(a[1] for a in angles), max(a[1] for a in angles))
    omegas = []
    for i in range(1, len(q)):
        dt = t[i] - t[i - 1]
        if 1e-4 < dt < 0.5:
            omegas.append(geodesic_rad(q[i - 1], q[i]) / dt)
    m["omega"] = omegas
    m["omega_p50"] = pct(omegas, 50)
    m["omega_p95"] = pct(omegas, 95)
    m["omega_max"] = max(omegas) if omegas else 0.0
    m["omega_active"] = sum(1 for w in omegas if w > 0.5) / len(omegas) if omegas else 0.0

    # 挡位
    m["gear_ratio"] = {}
    for k, v in GEAR_NAMES.items():
        n = sum(1 for x in mode if x == k)
        if n:
            m["gear_ratio"][v] = n / len(mode)
    other = sum(1 for x in mode if x not in GEAR_NAMES)
    if other:
        m["gear_ratio"]["UNKNOWN"] = other / len(mode)
    m["gear_segments"] = segments(mode, t)

    # 图片是否齐（车上打回来的一般只有 csv）
    m["images"] = None
    if img_dir:
        try:
            files = [f for f in os.listdir(img_dir) if f.lower().endswith(".jpg")]
            m["images"] = len(files)
        except OSError:
            pass
    return m


# ---------------------------------------------------------------- 输出

def fmt_run(m):
    L = []
    L.append(f"=== {m['name']} ===")
    if "error" in m:
        L.append(f"  {m['error']}（{m['rows']} 行有效，{m['bad_lines']} 行坏）")
        return L
    L.append(
        f"  帧数 {m['rows']}   时长 {m['duration']:.2f} s   帧率 {m['fps']:.1f} fps"
        f"（dt 中位 {m['dt_median'] * 1000:.1f} ms / p95 {m['dt_p95'] * 1000:.1f} ms）"
    )
    flags = []
    if not m["monotonic"]:
        flags.append("时间戳非单调！")
    if m["gaps"]:
        flags.append(f"间隔缺口 {m['gaps']} 处（>3×中位）")
    if m["bad_lines"]:
        flags.append(f"坏行 {m['bad_lines']}")
    if m["images"] is not None:
        if m["images"] == 0:
            flags.append("只有 csv，没有图（车上打回来的包一般就是这样，不影响时域指标）")
        else:
            flags.append(f"图片 {m['images']} 张 / csv {m['rows']} 行")
            if m["images"] != m["rows"]:
                flags.append("图片与行数不一致")
    if flags:
        L.append("  " + "；".join(flags))

    L.append(
        f"  姿态 yaw {m['yaw_range'][0]:+.1f}°..{m['yaw_range'][1]:+.1f}°   "
        f"pitch {m['pitch_range'][0]:+.1f}°..{m['pitch_range'][1]:+.1f}°"
    )
    if m["omega"]:
        L.append(
            f"  角速度 |ω| p50 {m['omega_p50']:.2f}  p95 {m['omega_p95']:.2f}  "
            f"max {m['omega_max']:.2f} rad/s   >0.5 占 {m['omega_active'] * 100:.0f}%"
        )
        lead95 = m["omega_p95"] * 0.3  # 外推 300 ms 的转角
        leadmax = m["omega_max"] * 0.3
        L.append(
            f"  外推 300 ms 的转角：按 p95 {math.degrees(lead95):.0f}°，按最大 {math.degrees(leadmax):.0f}°"
            "（板缝约 59°，超了就打空）"
        )
    if m["speed_nonzero_ratio"] < 0.01:
        L.append("  弹速：电控一直在报 0（兜底值生效，提前量按配置算）")
    else:
        L.append(f"  弹速：{m['speed_nonzero_ratio'] * 100:.0f}% 的帧非零，中位 {m['speed_mode']:.2f} m/s")

    if m["rounds"] == 0:
        L.append(
            f"  发弹：{m['count_start']} → {m['count_end']}，共 +0 发 —— ★电控一次都没报过发弹"
        )
    else:
        at = f"{m['first_round_at']:.2f} s" if m["first_round_at"] is not None else "?"
        L.append(
            f"  发弹：{m['count_start']} → {m['count_end']}，共 +{m['rounds']} 发"
            f"（{m['count_steps']} 次增长，首次在 {at}）"
        )

    gear = "  ".join(f"{k} {v * 100:.0f}%" for k, v in sorted(m["gear_ratio"].items()))
    L.append(f"  挡位：{gear}")
    segs = m["gear_segments"]
    for a, b, gm in segs[:8]:
        L.append(f"    {a - m['gear_segments'][0][0]:6.2f}–{b - m['gear_segments'][0][0]:6.2f} s  {GEAR_NAMES.get(gm, gm)}")
    if len(segs) > 8:
        L.append(f"    …… 还有 {len(segs) - 8} 段")
    return L


def md_table(ms):
    head = ("录包", "秒", "fps", "发弹", "AUTO_AIM", "角速 p95", "弹速非零")
    rows = [head]
    for m in ms:
        if "error" in m:
            rows.append((m["name"], "—", "—", m["error"], "—", "—", "—"))
            continue
        auto = m["gear_ratio"].get("AUTO_AIM", 0.0)
        rows.append(
            (
                m["name"],
                f"{m['duration']:.1f}",
                f"{m['fps']:.1f}",
                f"+{m['rounds']}",
                f"{auto * 100:.0f}%",
                f"{m['omega_p95']:.2f}",
                f"{m['speed_nonzero_ratio'] * 100:.0f}%",
            )
        )
    w = [max(len(str(r[i])) for r in rows) for i in range(len(head))]
    out = []
    for i, r in enumerate(rows):
        out.append("| " + " | ".join(str(c).ljust(w[j]) for j, c in enumerate(r)) + " |")
        if i == 0:
            out.append("|" + "|".join("-" * (w[j] + 2) for j in range(len(head))) + "|")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="录包 frames.csv 离线分析")
    ap.add_argument("paths", nargs="+", help="frames.csv / 录包目录 / 上级目录 / .tar")
    ap.add_argument("--markdown", action="store_true", help="末尾额外出 markdown 表")
    ap.add_argument("--out", default=None, help="把报告按 UTF-8 写进这个文件（控制台乱码时看这个）")
    args = ap.parse_args(argv)

    lines = []
    all_ms = []
    for p in args.paths:
        p = os.path.expanduser(p)
        for name, rows, bad, img_dir in load_sources(p):
            m = analyze(name, rows, bad, img_dir)
            all_ms.append(m)
            lines.extend(fmt_run(m))
            lines.append("")

    if not all_ms:
        print("没找到任何 frames.csv")
        return 1

    # 汇总：现场最想知道的两件事 —— 谁真发过弹、谁在 AUTO_AIM
    fired = [m for m in all_ms if m.get("rounds", 0) > 0]
    auto = [m for m in all_ms if m.get("gear_ratio", {}).get("AUTO_AIM", 0) > 0.5]
    lines.append("=== 汇总 ===")
    lines.append(
        f"  录包 {len(all_ms)} 份；电控报过发弹的 {len(fired)} 份；"
        f"AUTO_AIM 占多数时间的 {len(auto)} 份"
    )
    if not fired:
        lines.append(
            "  ★所有录包里 bullet_count 都没涨过 —— 一枪都没真正打出去（问题在协议/机构，不在解算）"
        )
    if args.markdown:
        lines.append("")
        lines.extend(md_table(all_ms))

    text = "\n".join(lines)
    print(text)
    if args.out:
        with open(os.path.expanduser(args.out), "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"\n[已写入 {args.out}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
