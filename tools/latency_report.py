#!/usr/bin/env python3
"""把 task_2 探针写出来的 logs/latency.csv 分析成「链路延迟 + 云台到位」两份结论。

用法（笔记本上跑，py 3.x 即可，没有第三方依赖）：

    py tools/latency_report.py logs/latency.csv
    py tools/latency_report.py logs/step.csv --markdown

CSV 每帧一行，列含义：
    dev_ts    相机曝光时间戳（tick，单位随型号变，0 表示这台相机没给）
    grab_ns   我们取到这一帧的时刻（steady_clock，ns）
    infer_ns  推理 + 解算完的时刻
    send_ns   串口写完开火/角度指令的时刻
    cmd_yaw/cmd_pitch   我们发出去的角度（弧度）
    fb_yaw/fb_pitch     电控同一帧报回来的姿态（弧度，pitch 符号与指令相反）
    fire      这一帧有没有请求开火

能给出什么、不能给出什么（重要）：
  · grab → send 是**精确可测**的：相机到串口这一段。
  · 曝光 → grab 只能测出**变化量**：dev_ts 换算后与 grab_ns 的固定差里，同时含着
    「相机时钟与主机时钟的固定偏移」和「曝光完到进队列的固定传输延迟」，两者不可分离。
    要拿到绝对值，得拿手机毫秒表对着镜头拍一帧，记下屏幕时间和当时的 grab_ns 相减。
  · 残差随时间漂移 = 取流队列在积压，这会让实际延迟比标称值大，必须单独看。
"""

import argparse
import csv
import math
import statistics
import sys

# GBK 控制台（cp936）编码不了 ⚠/⇒/·：输出一重定向到文件就 UnicodeEncodeError，
# 这里统一降级成 '?'，别让一个符号把整份报告带走。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(errors="replace")


def load(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "dev": int(r["dev_ts"]),
                        "grab": int(r["grab_ns"]),
                        "infer": int(r["infer_ns"]),
                        "send": int(r["send_ns"]),
                        "cy": float(r["cmd_yaw"]),
                        "cp": float(r["cmd_pitch"]),
                        "fy": float(r["fb_yaw"]),
                        "fp": float(r["fb_pitch"]),
                        "fire": int(r["fire"]),
                        "dist": float(r.get("dist") or -1.0),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows


def ms(ns):
    return ns / 1e6


def stats(values):
    values = sorted(values)
    n = len(values)
    return {
        "n": n,
        "p50": values[n // 2],
        "p95": values[min(n - 1, int(n * 0.95))],
        "max": values[-1],
        "mean": sum(values) / n,
    }


def fit_device_clock(rows):
    """用最小二乘把 dev_ts 映射到 grab_ns：grab = a * dev + b。返回 (a, b, 残差列表)。"""
    xs = [r["dev"] for r in rows]
    ys = [r["grab"] for r in rows]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    residuals = [y - (a * x + b) for x, y in zip(xs, ys)]
    return a, b, residuals


def lag_ms(rows, key_cmd, key_fb, max_lag=60):
    """互相关找 fb 落后 cmd 多少个采样点。返回 (lag_ms, 相关系数)。"""
    c = [r[key_cmd] for r in rows]
    f = [r[key_fb] for r in rows]
    n = len(c)
    if n < 20:
        return None
    mc = sum(c) / n
    mf = sum(f) / n
    c = [x - mc for x in c]
    f = [x - mf for x in f]
    nc = math.sqrt(sum(x * x for x in c))
    nf = math.sqrt(sum(x * x for x in f))
    if nc == 0 or nf == 0:
        return None

    best = (0, -2.0)
    for k in range(-10, max_lag + 1):
        if k >= 0:  # fb 落后：cmd[i] 对 fb[i+k]
            pairs = [(c[i], f[i + k]) for i in range(n - k)]
        else:
            pairs = [(c[i - k], f[i]) for i in range(n + k)]
        if len(pairs) < 10:
            continue
        s = sum(x * y for x, y in pairs) / (nc * nf)
        if s > best[1]:
            best = (k, s)
    k = best[0]
    dt_ns = (rows[-1]["grab"] - rows[0]["grab"]) / max(1, n - 1)
    return k * ms(dt_ns), best[1]


def step_response(rows, key_cmd, key_fb, min_step=1e-3):
    """从 cmd 的跳变里找阶跃，量反馈的 T10 / T90 / 超调。返回 dict 或 None。

    只依赖波形形状，所以 cmd 与 fb 之间的零点偏置、符号都无所谓。
    """
    c = [r[key_cmd] for r in rows]
    f = [r[key_fb] for r in rows]
    n = len(c)
    if n < 40:
        return None

    dt_ms = (rows[-1]["grab"] - rows[0]["grab"]) / (n - 1) / 1e6

    # 阶跃沿＝相邻帧跳变远大于邻域其它跳变的点。这样平滑跟瞄（正弦形）不会被误判成阶跃。
    diffs = [abs(c[i + 1] - c[i]) for i in range(n - 1)]
    edges = []
    for i in range(5, n - 40):
        d = diffs[i]
        if d < min_step:
            continue
        neighbors = [diffs[j] for j in range(max(0, i - 10), min(n - 1, i + 12)) if abs(j - i) > 1]
        if d < 3.0 * max(neighbors or [0.0]):
            continue
        if all(d >= diffs[j] for j in range(i - 4, i + 5) if 0 <= j < len(diffs)):
            edges.append(i)

    t10, t90, overs = [], [], []
    for idx, i in enumerate(edges):
        # 平台窗口必须停在下一次跳变之前，否则「稳态值」会被下一段拉偏
        nxt = edges[idx + 1] if idx + 1 < len(edges) else n - 1
        if nxt - i < 8:
            continue
        base = sum(f[max(0, i - 4) : i]) / len(f[max(0, i - 4) : i])
        lo = min(i + 5, nxt - 1)
        hi = min(i + 25, nxt)
        if hi <= lo:
            continue
        final = sum(f[lo:hi]) / (hi - lo)
        span = final - base
        if abs(span) < min_step:
            continue
        crossed10 = crossed90 = None
        for j in range(i, min(n, i + 40)):
            ratio = (f[j] - base) / span
            if crossed10 is None and ratio >= 0.1:
                crossed10 = j - i
            if crossed90 is None and ratio >= 0.9:
                crossed90 = j - i
                break
        if crossed10 is not None:
            t10.append(crossed10 * dt_ms)
        if crossed90 is not None:
            t90.append(crossed90 * dt_ms)
            window = f[i : min(n, i + 40)]
            peak = max(window) if span > 0 else min(window)
            overs.append((peak - final) / span * 100.0)

    if not t90:
        return None
    return {
        "n": len(t90),
        "t10": statistics.median(t10) if t10 else None,
        "t90": statistics.median(t90),
        "overshoot": statistics.median(overs),
        "dt_ms": dt_ms,
    }


def report(path, rows, as_markdown):
    out = []
    add = out.append
    add(f"## {path}")
    add("")
    if len(rows) < 5:
        add("样本太少（不到 5 帧），先让程序对着靶子跑几秒再来分析。")
        return "\n".join(out)

    span = rows[-1]["grab"] - rows[0]["grab"]
    add(f"- 帧数 {len(rows)}，跨度 {span / 1e9:.2f} s，平均 {len(rows) / max(1e-9, span / 1e9):.1f} fps")
    add(f"- 请求开火 {sum(r['fire'] for r in rows)} 帧")
    dists = [r["dist"] for r in rows if r["dist"] > 0]
    if dists:
        add(f"- 目标距离中位数 {statistics.median(dists):.2f} m（用来反解弹速）")

    g2i = [r["infer"] - r["grab"] for r in rows]
    i2s = [r["send"] - r["infer"] for r in rows]
    g2s = [r["send"] - r["grab"] for r in rows]
    for name, values in (("取帧→解算完", g2i), ("解算完→串口写完", i2s), ("取帧→串口写完", g2s)):
        s = stats(values)
        add(
            f"- {name}：p50 {ms(s['p50']):.2f} ms，p95 {ms(s['p95']):.2f} ms，"
            f"最大 {ms(s['max']):.2f} ms"
        )

    # ---- 曝光时间戳 ----
    devs = [r["dev"] for r in rows]
    add("")
    if not any(devs):
        add("### 曝光时间戳")
        add("")
        add("dev_ts 全是 0 —— 这台相机没有输出设备时间戳，或者 SDK 没填。")
        add("只能拿到「取帧→发命令」这一段；想要曝光→取帧，得换方法（拍毫秒表）。")
    else:
        fit = fit_device_clock(rows)
        add("### 曝光时间戳")
        add("")
        if fit is None:
            add("dev_ts 没有变化，拟合不了。")
        else:
            a, b, residuals = fit
            if a > 0:
                add(f"- tick 频率 ≈ {1e9 / a / 1e6:.3f} MHz（由 grab_ns 对 dev_ts 回归得到）")
            add(
                f"- 固定偏差 b = {b / 1e6:.3f} ms（含相机时钟偏移 + 曝光到入队延迟，"
                "这一项**不可分离**）"
            )
            r = stats(residuals)
            add(f"- 回归残差抖动：p50 {ms(abs(r['p50'])):.3f} ms，最大 {ms(abs(r['max'])):.3f} ms")
            half = len(residuals) // 2
            drift = sum(residuals[half:]) / max(1, len(residuals) - half) - sum(
                residuals[:half]
            ) / max(1, half)
            if abs(drift) > 2e6:
                add(
                    f"- ⚠ 后半段残差比前半段偏了 {ms(drift):+.2f} ms —— 取流队列在积压，"
                    "实际延迟比标称大，先把 queue 或推理耗时压下来"
                )
            else:
                add("- 残差前后两段基本一致，说明取流队列没有积压")

    # ---- 云台跟随 ----
    add("")
    add("### 云台跟随（命令 vs 反馈）")
    add("")
    ly = lag_ms(rows, "cy", "fy")
    lp = lag_ms(rows, "cp", "fp")
    if ly is None:
        add("样本不足，算不出滞后。")
    elif ly[1] < 0:
        add(f"- ⚠ yaw 的相关系数是负的（{ly[1]:.2f}）：电控 yaw 的正方向和我们相反，先对符号")
    elif ly[1] < 0.3:
        add(f"- yaw 相关系数只有 {ly[1]:.2f}，这段数据太散，滞后读数不可信")
    else:
        add(f"- yaw 反馈比指令慢约 {ly[0]:.1f} ms（相关系数 {ly[1]:.2f}，量化到帧）")
    if lp is not None:
        add(f"- pitch 反馈比指令慢约 {lp[0]:.1f} ms（相关系数 {lp[1]:.2f}）")

    # 有阶跃才有 T90；普通跟瞄时 cmd 是缓变的，这里会跳过
    any_step = False
    for axis, kc, kf in (("yaw", "cy", "fy"), ("pitch", "cp", "fp")):
        sr = step_response(rows, kc, kf)
        if sr is None:
            continue
        any_step = True
        t10 = f"{sr['t10']:.0f} ms" if sr["t10"] is not None else "?"
        add(
            f"- {axis} 阶跃响应（{sr['n']} 次，帧间隔 {sr['dt_ms']:.1f} ms）："
            f"T10 ≈ {t10}，**T90 ≈ {sr['t90']:.0f} ms**，超调 {sr['overshoot']:+.0f}%"
        )
    if not any_step:
        add("- 没有检测到阶跃沿（这次是正常跟瞄，不是 DSH_STEP_DEG 测试）")
    add("")
    add(
        "注意：fb 是电控坐标系，pitch 在 gimbal.send 里取过反；这里比较的是**波形**，"
        "符号和零点偏置不影响滞后读数，但要看绝对值得先把两边的零点对齐。"
    )

    add("")
    add("### 怎么用这些数")
    add("")
    add("1. 「取帧→串口写完」就是我们可以直接用 `DSH_LEAD` 补掉的那部分的下界。")
    add("2. yaw 反馈的滞后是云台本身的跟随延迟，`DSH_LEAD` 补不掉它，只能靠提前量。")
    add("3. 把「取帧→串口」+「云台滞后」+「曝光到入队的固定偏差（拍毫秒表标定）」相加，")
    add("   才是应该写进 `DSH_LEAD` 的总滞后。")
    return "\n".join(out)


def main(argv):
    ap = argparse.ArgumentParser(description="分析 task_2 的延迟探针 CSV")
    ap.add_argument("csv", nargs="+", help="探针写出的 csv（可以给多个）")
    ap.add_argument("--markdown", action="store_true", help="输出 markdown（默认纯文本）")
    args = ap.parse_args(argv)

    chunks = []
    for path in args.csv:
        try:
            rows = load(path)
        except OSError as e:
            print(f"读不了 {path}：{e}", file=sys.stderr)
            continue
        chunks.append(report(path, rows, args.markdown))

    print("\n\n".join(chunks))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
