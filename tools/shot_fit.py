#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shot_fit.py —— 从「打出去的每一发的弹着点」反推参数，专门给只有几次发弹机会的现场用。

思路
----
每次发弹都是一次测量。只要能看到弹着点（靶纸/胶带）并量出它离板中心多远，
就能把「误差」翻译成角度，再把角度翻译成参数：

    横向（靶子运动方向上的落点偏移，mm）
        dx = d · ω · (L − T*)                      T* = 真正需要的总提前量
        只要知道 b（静止场量到的横向零点），就能对每一发直接解出 T*
            T* = L − (dx − b) / (d · ω)

    纵向（上下偏移，mm）
        dy = c1 · d + c0                           c1 才是「角度」误差，c0 不是
        随距离线性增长的部分 = 炮口视差 / 弹速误差 / 零点角度；
        与距离无关的常数 c0 多半是靶纸中心没标对，别当成角度去改 pitch_offset。

用法
----
    py tools/shot_fit.py shots.csv
    py tools/shot_fit.py shots.csv --lateral-zero-mm 15 --plate-width-mm 135
    py tools/shot_fit.py shots.csv --markdown

CSV 表头（顺序随意，必须有这几列）：
    shot,d_m,omega,lead_ms,dx_mm,dy_mm
    shot     发序号，随便记
    d_m      靶子到炮口的水平距离（米）
    omega    靶子当时的角速度（rad/s，取 EKF 的 w 或手动测；静止写 0）
    lead_ms  这一发开火时用的 DSH_LEAD（毫秒）
    dx_mm    弹着点相对板中心的横向偏移（mm，**正 = 落在靶子运动方向的反侧，也就是"落后"**）
    dy_mm    弹着点相对板中心的纵向偏移（mm，**正 = 偏上**）

符号怎么定：站在炮口看靶子，靶子往右转（俯视逆时针为正），弹着点落在板中心右边，
就是"落后"，dx 记正。拍照片时在靶纸左边画个箭头标住"靶子往这边跑"，就不会记混。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass


@dataclass
class Shot:
    shot: str
    d_mm: float
    omega: float
    lead_s: float
    dx_mm: float
    dy_mm: float


def load(path: str) -> list[Shot]:
    rows: list[Shot] = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        r = csv.DictReader(f)
        need = {"d_m", "omega", "lead_ms", "dx_mm", "dy_mm"}
        missing = need - set(r.fieldnames or [])
        if missing:
            sys.exit(f"CSV 缺列：{', '.join(sorted(missing))}")
        for i, row in enumerate(r, 1):
            try:
                rows.append(
                    Shot(
                        shot=(row.get("shot") or str(i)).strip(),
                        d_mm=float(row["d_m"]) * 1000.0,
                        omega=float(row["omega"]),
                        lead_s=float(row["lead_ms"]) / 1000.0,
                        dx_mm=float(row["dx_mm"]),
                        dy_mm=float(row["dy_mm"]),
                    )
                )
            except (TypeError, ValueError) as exc:
                sys.exit(f"第 {i} 行读不出来（{exc}）：{row}")
    if not rows:
        sys.exit("CSV 里没有数据行")
    return rows


def mean(xs: list[float]) -> float:
    return sum(xs) / len(xs)


def median(xs: list[float]) -> float:
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def std(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def linfit(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """最小二乘 y = a*x + b，返回 (a, b, R²)。"""
    n = len(xs)
    if n < 2:
        return float("nan"), float("nan"), float("nan")
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return float("nan"), float("nan"), float("nan")
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return a, b, r2


def main() -> int:
    ap = argparse.ArgumentParser(description="从弹着点反推 lead / 纵向零点")
    ap.add_argument("csv")
    ap.add_argument("--lateral-zero-mm", type=float, default=None,
                    help="静止场量到的横向零点 b（mm，静止靶、lead=0 的落点）。给了才能直接解出 T*")
    ap.add_argument("--plate-width-mm", type=float, default=135.0,
                    help="装甲板宽度（mm）。小装甲 135，大装甲 225，默认 135")
    ap.add_argument("--markdown", action="store_true", help="输出 markdown 表格")
    args = ap.parse_args()

    shots = load(args.csv)
    half_plate = args.plate_width_mm / 2.0
    rot = [s for s in shots if abs(s.omega) > 1e-6]
    print(f"共 {len(shots)} 发，其中旋转场 {len(rot)} 发、静止场 {len(shots) - len(rot)} 发")
    print()

    # ---------------- 每发的等效角误差 ----------------
    print("== 1. 逐发视角误差（mrad，1 mrad 在 3 m 处是 3 mm）==")
    print(f"{'shot':>6} {'d(m)':>6} {'ω':>6} {'L(ms)':>7} {'dx(mm)':>8} {'dy(mm)':>8} "
          f"{'横角':>7} {'纵角':>7}")
    for s in shots:
        ax = s.dx_mm / s.d_mm * 1000.0
        ay = s.dy_mm / s.d_mm * 1000.0
        print(f"{s.shot:>6} {s.d_mm/1000:>6.2f} {s.omega:>6.2f} {s.lead_s*1000:>7.1f} "
              f"{s.dx_mm:>8.1f} {s.dy_mm:>8.1f} {ax:>7.2f} {ay:>7.2f}")
    print()

    # ---------------- 横向：解 T* ----------------
    print("== 2. 横向：真正需要的总提前量 T* ==")
    if not rot:
        print("  这一份 CSV 里没有旋转场数据，跳过。")
    elif args.lateral_zero_mm is None:
        print("  ⚠ 没给 --lateral-zero-mm，只能用「相对变化」判断，解不出绝对值：")
        # 用 dx = d·ω·L − d·ω·T* + b 做回归：dx ~ (d·ω·L) 与 (d·ω)
        x1 = [s.d_mm * s.omega * s.lead_s for s in rot]  # 单位 mm：d(mm)·ω·L(s)
        ys = [s.dx_mm for s in rot]
        a, b, r2 = linfit(x1, ys)
        print(f"  回归 dx = {a:.3f}·(d·ω·L) + {b:.1f}    R²={r2:.3f}")
        print("  斜率理论值是 1.0；离 1 太远说明距离/角速度记错了。")
        print("  → 去静止场打一发量出 b，再用 --lateral-zero-mm 重跑，才能得到 T*。")
    else:
        b = args.lateral_zero_mm
        t_stars = []
        for s in rot:
            # T* = L − (dx − b) / (d·ω)     单位：s
            t_star = s.lead_s - (s.dx_mm - b) / (s.d_mm * s.omega)
            t_stars.append(t_star)
            print(f"  第 {s.shot:>4} 发：T* = {t_star*1000:6.1f} ms"
                  f"   （用的 L = {s.lead_s*1000:.1f} ms）")
        med = median(t_stars)
        spread_s = std(t_stars)
        print()
        print(f"  T* 中位数 {med*1000:.1f} ms，均值 {mean(t_stars)*1000:.1f} ms，"
              f"离散 {spread_s*1000:.1f} ms（越大越可疑）")
        # T* 的不确定度折算到靶上：Δx = d·ω·ΔT*
        dw = mean([s.d_mm * s.omega for s in rot])
        lateral = dw * spread_s
        print(f"  T* 离散 {spread_s*1000:.1f} ms → 落点散布 ±{lateral:.1f} mm"
              f"（板半宽 {half_plate:.1f} mm）"
              f" → {'够用' if lateral < half_plate * 0.5 else '样本太少或数据太散，再打几发'}")
        # 零点 b 不是提前量能修的，得用 yaw_offset 吃掉
        d_mean = mean([s.d_mm for s in rot])
        print(f"  零点 b = {b:.1f} mm 与 lead 无关，建议用 yaw_offset 吃掉："
              f"{b/d_mean*1000:.2f} mrad = {b/d_mean*180/math.pi:.2f}°（照着量到的方向修）")

    print()

    # ---------------- 纵向：零点 vs 距离相关 ----------------
    print("== 3. 纵向：把「角度误差」和「靶纸没标对」分开 ==")
    xs = [s.d_mm for s in shots]
    ys = [s.dy_mm for s in shots]
    if len(set(round(x, 1) for x in xs)) < 2:
        print("  所有发都在同一个距离上，分不开。至少要在两个距离各打一发。")
        print(f"  当前距离 {xs[0]/1000:.2f} m，平均纵向偏移 {mean(ys):.1f} mm "
              f"（{mean(ys)/xs[0]*1000:.2f} mrad ≈ {mean(ys)/xs[0]*180/math.pi:.2f}°）")
    else:
        a, b, r2 = linfit(xs, ys)
        print(f"  dy = {a:.4f}·d + {b:.1f}    R²={r2:.3f}")
        print(f"  距离相关部分（才是角度）：{a*1000:.2f} mrad = "
              f"{a*180/math.pi:.2f}°  → pitch_offset 应当按这个方向修")
        print(f"  与距离无关的常数：{b:.1f} mm → 这**不是**角度误差，"
              f"先检查靶纸中心是不是画在板中心上了")
    print()

    # ---------------- 换算表 ----------------
    print("== 4. 现场换算表：1 ms 的 lead 误差在靶上等于多少 mm ==")
    print("  （d·ω·1ms，超过板半宽就会漏）")
    print(f"{'距离':>6} {'ω=3':>8} {'ω=5':>8} {'ω=7':>8}")
    for d_m in sorted({round(s.d_mm / 1000, 2) for s in shots} or [3.0]):
        row = [f"{d_m:>5.1f}m"]
        for w in (3.0, 5.0, 7.0):
            row.append(f"{d_m*1000*w*0.001:>7.1f}")
        print(" ".join(row))
    print()
    print(f"  板半宽参考：小装甲 {135/2:.0f} mm / 大装甲 {225/2:.0f} mm")

    if args.markdown:
        print()
        print("| shot | d(m) | ω(rad/s) | L(ms) | dx(mm) | dy(mm) | 横角(mrad) | 纵角(mrad) |")
        print("|---|---|---|---|---|---|---|---|")
        for s in shots:
            print(f"| {s.shot} | {s.d_mm/1000:.2f} | {s.omega:.2f} | {s.lead_s*1000:.1f} | "
                  f"{s.dx_mm:.1f} | {s.dy_mm:.1f} | {s.dx_mm/s.d_mm*1000:.2f} | "
                  f"{s.dy_mm/s.d_mm*1000:.2f} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
