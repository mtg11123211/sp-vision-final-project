#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_fix.py —— 把「静止靶弹着点偏了多少 mm」换成 yaml 里该写多少 offset。

为什么需要它
------------
偏移量是 mm，参数是角度，中间还隔着两个符号约定，手算十有八九会错方向。
这个脚本把换算、符号、以及「这点偏移值不值得改」一次算完。

符号约定（从代码里定的，不是猜的）
----------------------------------
* `tools/math_tools.cpp:103` `xyz2ypd`：`yaw = atan2(y, x)`，本工程是 x 前 / y 左 / z 上，
  所以**正的 yaw 是往左**；
* `src/task_common.hpp:104` `yaw = atan2(aim_xyz.y(), aim_xyz.x())`，`pitch = atan2(z, 水平距离)`，
  所以**正的 pitch 是往上**；
* `src/task_2.cpp` 里 `aim_yaw += yaw_offset`、`aim_pitch += pitch_offset`，
  于是：**yaw_offset 变大 → 落点往左；pitch_offset 变大 → 落点往上**。

于是修正量：
    Δyaw_offset   = +(dx_mm / d_mm)      弧度（落点偏右就把它往左拉）
    Δpitch_offset = −(dy_mm / d_mm)      弧度（落点偏下就把它往上抬）

`gimbal.cpp:111` 的 `tx_data_.pitch = -pitch` 是电控那边的协议约定，不影响这里的推理。

用法
----
    py tools/zero_fix.py --d 2.4 --dx 10 --dy -10
    py tools/zero_fix.py --d 3 --dx 12 --dy -8 --yaw-offset 0.48 --pitch-offset 10
    py tools/zero_fix.py --d 2.4 --dx 10 --dy -10 --plate-width-mm 225

给的是负数也不用改：脚本自己会挑方向。
"""

from __future__ import annotations

import argparse
import math
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description="弹着点 mm → yaw_offset / pitch_offset")
    ap.add_argument("--d", type=float, required=True, help="靶子到炮口的水平距离（米）")
    ap.add_argument("--dx", type=float, required=True, help="横向偏移（mm），正 = 落点偏右")
    ap.add_argument("--dy", type=float, required=True, help="纵向偏移（mm），正 = 落点偏上")
    ap.add_argument("--yaw-offset", type=float, default=0.0, help="当前 yaml 里的 yaw_offset（度），缺省=当前 yaml 的 0.0")
    ap.add_argument("--pitch-offset", type=float, default=0.0, help="当前 yaml 里的 pitch_offset（度），缺省=当前 yaml 的 0.0")
    ap.add_argument("--plate-width-mm", type=float, default=135.0, help="板宽，小装甲 135 / 大装甲 225")
    args = ap.parse_args()

    if args.d <= 0:
        sys.exit("--d 得是正数")

    d_mm = args.d * 1000.0
    rad2deg = 180.0 / math.pi
    ax = args.dx / d_mm          # 横向角误差（rad），正 = 偏右
    ay = args.dy / d_mm          # 纵向角误差（rad），正 = 偏上

    dyaw = ax * rad2deg          # 要把落点往左拉，所以 yaw_offset 要加
    dpitch = -ay * rad2deg       # 要把落点往（上/下）拉
    new_yaw = args.yaw_offset + dyaw
    new_pitch = args.pitch_offset + dpitch

    print(f"距离 {args.d:.2f} m，弹着点 dx={args.dx:+.1f} mm（右正）、dy={args.dy:+.1f} mm（上正）")
    print(f"  折算角度：横向 {ax*1000:+.2f} mrad（{ax*rad2deg:+.3f}°）"
          f"   纵向 {ay*1000:+.2f} mrad（{ay*rad2deg:+.3f}°）")
    print()

    half = args.plate_width_mm / 2.0
    used = max(abs(args.dx), abs(args.dy)) / half * 100.0
    print(f"  这点偏移占掉板半宽（{half:.1f} mm）的 {used:.0f}%")
    if used < 30:
        print("  → 不影响命中。改不改都行，改是免费的（常数偏置，不用花发数验证）。")
    elif used < 100:
        print("  → 还没漏，但边角上会漏。建议改。")
    else:
        print("  → 已经偏出板外了，必须改。")

    # 同样的偏移，换算成「等价 lead 误差」——说明它对旋转场意味着什么
    print()
    print("  等价成提前量误差（偏移 / d / ω）：")
    for w in (3.0, 5.0, 7.0):
        t_ms = abs(ax) / w * 1000.0
        print(f"     ω={w:.0f} rad/s → {t_ms:.1f} ms")
    print("  （容差参考：3 m / 135 mm 板，ω=3 时约 ±7.5 ms；这里这个数远小于它，"
          "说明静态零点不是漏靶的原因。）")
    print()

    print("== 该写进 configs/standard.yaml 的值 ==")
    print(f"  yaw_offset:   {args.yaw_offset:.2f}   →  {new_yaw:.2f}   （Δ {dyaw:+.2f}°，"
          f"{'往左' if dyaw > 0 else '往右'}拉 {abs(dyaw):.2f}°）")
    print(f"  pitch_offset: {args.pitch_offset:.2f}  →  {new_pitch:.2f}  （Δ {dpitch:+.2f}°，"
          f"{'往上' if dpitch > 0 else '往下'}拉 {abs(dpitch):.2f}°）")
    print()
    print("前提：这两个数必须是**在当时的 yaw_offset / pitch_offset 已经生效**的情况下量的。")
    print("若量的时候用的是别的值，先把基准改回量测时的值再跑一遍这个脚本。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
