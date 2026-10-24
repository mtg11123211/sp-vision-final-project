#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""斜率法定标：算出该用多少 DSH_LEAD（总提前量，单位秒）。

原理
----
板的切向速度 v = r·ω（m/s）。把瞄点往前推 Δt 秒，落点就沿切线方向移动 v·Δt（米），
也就是"灵敏度" k = r·ω·1000（mm/s）——**这个值是能算的，不用测**。

所以只要在两档 lead 下各打几发、量出落点偏移（mm），就能反解出真正该补的总提前量。
（2026-10-21 修符号：原稿写成 T = lead + off/k，94.5 ms 的样例会被算成 19.4 ms。
 约定 off 正 = 落点沿转动方向**超前**（补多了）、off 负 = **落后**（补少了）；
 补 L 秒、真正需要 T 秒时 off = k·(L − T)，所以 T = L − off/k。）

    残差延迟 = T − 当前 lead = −off / k
    总延迟 T  = 当前 lead − 偏移 / k          （T 含链路 + 云台滞后 + 出弹延时 + 同包 1 帧）
    新 lead  = T                                （一次补到位）

用法
----
**推荐**（用 `~/run_measure.sh slope` 生成的模板，把 off_a / off_b 填上）::

    py tools\\lead_solve.py --from logs\\slope_xxx.txt

模板长这样（`#` 开头的是注释，`off_a` / `off_b` 留空等你填）::

    lead_a = 0.057
    off_a =
    lead_b = 0.171
    off_b =
    omega = 4
    r = 0.2
    dist = 3.15
    # b1 = 25      ← 视觉延迟 ms（填了就能顺带拆出 B3）
    # b2 = 32      ← 云台反馈滞后 ms
    # fps = 58

也可以直接给数（不用模板）::

    py tools\\lead_solve.py --lead-a 0.057 --off-a -30 --lead-b 0.171 --off-b 61 \
                           --omega 4 --r 0.2 --dist 3.15 [--b1 25] [--b2 32] [--fps 58]

参数含义
--------
--lead-a/--off-a  第一档：lead 值(s) 与该档落点偏移(mm，沿靶子转动方向为正)
--lead-b/--off-b  第二档（同上）
--omega   靶子角速度 rad/s（3~9）
--r       装甲板到旋转中心的半径 m（步兵约 0.2）
--dist    相机到靶子的距离 m（3.15）
--b1/--b2  已知的话：视觉延迟 / 云台反馈滞后（ms），用来反推出弹延时 B3
--fps     帧率（默认 58），用来算"开火位与角度同包"带来的 1 帧量化
--from    从上面那种模板文件读参数（命令行里给的同名参数优先）

⚠ 2026-10-21 在本机 py 3.14 上实跑过（--lead-a/-off-a 示例、--from 模板、反例拒绝三条路径）；
  车上环境（Linux / GBK 终端）未跑过，第一次用先 --help 确认。
"""

import argparse
import re
import sys

# GBK 控制台（cp936）编码不了 ⚠/⇒/·：输出一重定向到文件就 UnicodeEncodeError，
# 这里统一降级成 '?'，别让一个符号把整份报告带走。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(errors="replace")


def load_template(path):
    """读 'key = value' 模板；空值视为没填。"""
    out = {}
    with open(path, "r", encoding="utf-8-sig", errors="replace") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            k, _, v = line.partition("=")
            k = k.strip()
            v = v.strip()
            if k:
                out[k] = v
    return out


def fnum(v, name):
    if v is None or v == "":
        raise SystemExit(f"!! {name} 没填（模板里那一行是空的）")
    try:
        return float(v)
    except ValueError:
        raise SystemExit(f"!! {name} 不是数字：{v}")


def main():
    ap = argparse.ArgumentParser(
        description="斜率法定标：算出该用多少 DSH_LEAD（总提前量）"
    )
    ap.add_argument(
        "--from", dest="tpl", default="", help="记录模板文件（logs/slope_*.txt）"
    )
    ap.add_argument("--lead-a", type=float, default=None, help="第一档 lead（秒）")
    ap.add_argument(
        "--off-a",
        type=float,
        default=None,
        help="第一档落点偏移（mm，正=沿转动方向超前）",
    )
    ap.add_argument("--lead-b", type=float, default=None, help="第二档 lead（秒）")
    ap.add_argument("--off-b", type=float, default=None, help="第二档落点偏移（mm）")
    ap.add_argument("--omega", type=float, default=None, help="靶子角速度 rad/s")
    ap.add_argument("--r", type=float, default=None, help="装甲板到旋转中心的半径 m")
    ap.add_argument("--dist", type=float, default=None, help="相机到靶子距离 m")
    ap.add_argument("--b1", type=float, default=None, help="视觉延迟 ms（已知）")
    ap.add_argument("--b2", type=float, default=None, help="云台反馈滞后 ms（已知）")
    ap.add_argument(
        "--fps", type=float, default=None, help="帧率（算同包 1 帧的量化用）"
    )
    args = ap.parse_args()

    tpl = load_template(args.tpl) if args.tpl else {}

    def pick(name, cli):
        if cli is not None:
            return cli
        raw = tpl.get(name)
        if raw is None or raw == "":
            return None
        return float(raw)

    lead_a = fnum(pick("lead_a", args.lead_a), "lead_a")
    off_a = fnum(pick("off_a", args.off_a), "off_a")
    lead_b = fnum(pick("lead_b", args.lead_b), "lead_b")
    off_b = fnum(pick("off_b", args.off_b), "off_b")
    omega = fnum(pick("omega", args.omega), "omega")
    r = pick("r", args.r)
    dist = pick("dist", args.dist)
    b1 = pick("b1", args.b1)
    b2 = pick("b2", args.b2)
    fps = pick("fps", args.fps)
    r = 0.20 if r is None else r
    dist = 3.15 if dist is None else dist
    fps = 58.0 if fps is None else fps

    # ---- 理论量 ----
    k_theory = r * omega * 1000.0  # mm/s：落点位移对 lead 的灵敏度
    app_rate = r * omega / dist  # rad/s：板在画面里的视在角速度
    app_dps = app_rate * 180.0 / 3.141592653589793
    print("== 理论量 ==")
    print(
        f"  切向速度 r·ω        = {r * omega:.3f} m/s  ⇒ 灵敏度 k = {k_theory:.0f} mm/s"
    )
    print(
        f"  视在角速度 r·ω/d      = {app_dps:.2f} °/s  ⇒ lead 每 10 ms 补 {app_dps * 0.010:.3f}°"
    )

    # ---- 实测斜率 ----
    d_lead = lead_b - lead_a
    d_off = off_b - off_a
    if abs(d_lead) < 1e-6:
        print("!! 两档 lead 一样，算不出斜率")
        return 2
    k_meas = d_off / d_lead
    print("\n== 实测 ==")
    print(
        f"  两档差 {d_lead * 1000:.0f} ms，落点差 {d_off:.0f} mm  ⇒ 实测灵敏度 k = {k_meas:.0f} mm/s"
    )
    ratio = k_meas / k_theory if k_theory else float("nan")
    print(f"  k实测/k理论 = {ratio:.2f}")
    if abs(ratio) < 0.4 or abs(ratio) > 2.5:
        print(
            "  ⚠ 与理论差太多 ⇒ 先查：靶子真的在转吗？ω 对吗？落点读得准吗？"
            "（偏移的**正负方向**也要对：正=沿转动方向超前）"
        )
    # ---- 两档自洽性自检（必须用 k_theory，不能用 k_meas）----
    # 用 k_meas 反解两档必然恒等：k_meas 就是这两点连线的斜率，代回去只会得到同一条直线，
    # 自检不出任何东西。所以这里用理论灵敏度 k_theory 分别反解两档，看两个 T 是否落在同一点。
    if k_theory <= 1e-9:
        print("\n!! ω 或 r 为 0 ⇒ 理论灵敏度 k=0，反解没有意义；先确认靶子真的在转、ω 填对")
        return 2
    t_th_a = lead_a - off_a / k_theory
    t_th_b = lead_b - off_b / k_theory
    t_ref = 0.5 * (t_th_a + t_th_b)
    spread = (t_th_b - t_th_a) / t_ref if abs(t_ref) > 1e-9 else float("inf")
    print("\n== 两档自洽性（用 k理论 反解，不是 k实测）==")
    print(f"  A 档 ⇒ T = {t_th_a * 1000:+.1f} ms")
    print(f"  B 档 ⇒ T = {t_th_b * 1000:+.1f} ms")
    print(f"  两档相差 {spread * 100:+.1f}%（相对均值，阈值 15%）")
    if abs(spread) > 0.15:
        print("  ⚠ 两档对不上 ⇒ 至少一档的读数/方向有问题，这次的结果别用")
        print("     常见原因：off 正负号反了、中途改了转速、某一档打飞没量到板心")

    if k_meas <= 0:
        print("\n!! k实测 ≤ 0：两档落点方向相反（off 符号或转动方向搞错）")
        print("   不做反解、不给建议值 —— 先把两档 off 的正负号核对清楚再重打")
        return 2

    # ---- 解总提前量 ----
    k_use = k_meas if abs(k_meas) > 1e-6 else k_theory
    src = "实测斜率" if abs(k_meas) > 1e-6 else "理论斜率"
    T = lead_a - off_a / k_use
    new_lead = T
    print("\n== 结论 ==")
    print(f"  用 {src} k={k_use:.0f} mm/s 反解：")
    print(f"    残差延迟 = {(T - lead_a) * 1000:+.1f} ms")
    print(f"    总提前量 T = {T * 1000:.1f} ms")
    print(
        f"  ⇒ 新的 DSH_LEAD = {max(0.0, new_lead):.4f} s"
        f"   （补偿量 = {max(0.0, new_lead) * app_dps:.2f}°）"
    )
    if not (0.02 <= new_lead <= 0.20):
        print("  ⚠ 新值超出合理区间（0.02~0.20 s）⇒ 别直接用，先复核读数与 ω")

    # ---- 顺带拆出 B3（出弹延时） ----
    if b1 is not None and b2 is not None:
        frame_ms = 1000.0 / max(1e-6, fps)
        b3 = T * 1000.0 - b1 - b2 - frame_ms
        print("\n== 顺带拆出 B3（出弹延时）==")
        print(f"  总提前 T = {T * 1000:.1f} ms")
        print(f"    − 视觉延迟 B1   = {b1:.1f} ms")
        print(f"    − 云台滞后 B2   = {b2:.1f} ms")
        print(f"    − 同包 1 帧量化 = {frame_ms:.1f} ms（{fps:.0f} fps）")
        print(f"  ⇒ B3（摩擦轮出弹）≈ {b3:.1f} ms")
        if b3 < -5:
            print(
                "  ⚠ B3 为负 ⇒ B1/B2 之一偏大，或 T 里已含它们；方案①下 B3 只作记录，不参与补偿"
            )
        print("  （方案①：DSH_LEAD = T 就是全部；B3 填进 DSH_FIRE_DELAY 只作记录）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
