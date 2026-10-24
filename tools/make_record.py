#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成现场记录表：把 sessions.csv 与各场的汇总块汇成一张可打印/可归档的 Markdown。

用法（笔记本上）::

    py tools\\make_record.py --csv logs\\sessions.csv --out logs\\记录表.md
    py tools\\make_record.py --csv logs\\sessions.csv --out logs\\记录表.md --title "2026-10-21 现场"

它做三件事：
  1) 读 sessions.csv（由 tools/parse_session.py --csv 追加）⇒ 一行一个场次；
  2) 读 logs/summary_*.txt（程序退出时自动打出的汇总块）⇒ 补上 A/B/C 三组的数；
  3) 输出记录表：程序能测的列自动填好，**测出来的列留空**（B1/B2/B3、命中数、零位），
     现场手填即可。

只依赖标准库。
"""

import argparse
import csv
import re
import sys
from pathlib import Path

# GBK 控制台（cp936）编码不了 ⚠/⇒/·：输出一重定向到文件就 UnicodeEncodeError，
# 这里统一降级成 '?'，别让一个符号把整份报告带走。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(errors="replace")

RE_SUM_HEAD = re.compile(r"^=+\s*测试汇总\s+(\S+)\s*=+$")


def read_summaries(logdir):
    """返回 {tag: 汇总块文本}。"""
    out = {}
    for p in sorted(Path(logdir).glob("summary_*.txt")):
        try:
            txt = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        tag = p.stem.replace("summary_", "")
        # 汇总块的第一行里带场次名（可能是 p5_base_1021_1500 这种带时间戳的）
        m = RE_SUM_HEAD.match(txt.strip().splitlines()[0]) if txt.strip() else None
        if m:
            tag = m.group(1)
        out[tag] = txt
    return out


def pick(summary_text, key, default=""):
    """从汇总块里取 'key: value' 形式的那一项。"""
    for line in summary_text.splitlines():
        s = line.strip()
        if s.startswith(key) and ":" in s:
            v = s.partition(":")[2].strip()
            v = re.sub(r"\s{2,}.*$", "", v)  # 去掉后面的括号说明
            return v or default
    return default


def main():
    ap = argparse.ArgumentParser(description="生成现场记录表")
    ap.add_argument("--csv", default="logs/sessions.csv", help="场次总表 CSV")
    ap.add_argument("--logs", default="logs", help="日志目录（找 summary_*.txt）")
    ap.add_argument("--out", default="logs/记录表.md", help="输出文件")
    ap.add_argument("--title", default="现场记录表", help="标题")
    args = ap.parse_args()

    summaries = read_summaries(args.logs)

    rows = []
    csvp = Path(args.csv)
    if csvp.is_file():
        with csvp.open("r", encoding="utf-8", errors="replace", newline="") as f:
            rows = list(csv.DictReader(f))
    else:
        print(
            f"[warn] 找不到 {csvp}（先跑 parse_session.py --csv ...）", file=sys.stderr
        )

    L = [f"# {args.title}", ""]
    L.append(f"> 生成：`py tools\\make_record.py --csv {args.csv} --out {args.out}`")
    L.append("> 灰底列由程序自动填（来自各场退出时的汇总块）；**空白列要现场手填**。")
    L.append("")
    L.append("## 一、场次总表")
    L.append("")
    hdr = (
        "| 场次 | 挡位 | errY 均值/最大 | 门限(eff) | 到位门控挡下 | 选板切换 | dz 最大 | "
        "发/中 | DSH_LEAD | B1 视觉(ms) | B2 云台(ms) | B3 出弹(ms) | 零位 yaw/pitch | 备注 |"
    )
    L.append(hdr)
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        tag = r.get("tag", "")
        s = summaries.get(tag, "")
        L.append(
            "| {tag} | {gear} | {ey} | {tol} | {blk} | {sw} | {dz} | {rd} | {ld} |  |  |  |  |  |".format(
                tag=tag,
                gear=pick(s, "A1 电控挡位", "?"),
                ey=(r.get("errY_mean", "") + " / " + r.get("errY_max", ""))
                if r.get("errY_mean")
                else "",
                tol=r.get("tol_eff", ""),
                blk=r.get("blocked", ""),
                sw=r.get("switches", ""),
                dz=r.get("dz_max", ""),
                rd=r.get("rounds", ""),
                ld=r.get("lead", ""),
            )
        )
    if not rows:
        L.append("| （还没有场次） | | | | | | | | | | | | | |")
    L.append("")

    L.append("## 二、每场的关键数值（自动摘自汇总块）")
    L.append("")
    for tag, s in summaries.items():
        L.append(f"### {tag}")
        L.append("")
        L.append("| 组 | 项 | 值 |")
        L.append("|---|---|---|")
        for g, k in (
            ("A", "A1 电控挡位"),
            ("A", "A2 云台滞后 errY"),
            ("A", "A3 按键通道"),
            ("B", "B1 视觉延迟"),
            ("B", "B2 云台反馈滞后"),
            ("B", "B3 出弹延时"),
            ("B", "总提前量"),
            ("B", "飞行时间"),
            ("C", "C1 高度差 x[10]"),
            ("C", "C2 角速度 w"),
            ("C", "C3 选板切换"),
        ):
            v = pick(s, k)
            if v:
                L.append(f"| {g} | {k} | {v} |")
        L.append("")
        L.append("```")
        L.append(s.strip())
        L.append("```")
        L.append("")

    L.append("## 三、要手填的量（程序测不到）")
    L.append("")
    L.append("| 量 | 怎么测 | 备注 |")
    L.append("|---|---|---|")
    L.append(
        "| B1 视觉延迟 | `DSH_LAT_CSV` 跑 30 s → `py tools\\latency_report.py` | 别录像 |"
    )
    L.append("| B2 云台反馈滞后 | 同上（互相关） | 相关系数 ≥0.5 才认 |")
    L.append("| B3 出弹延时 | 斜率法反推：T − B1 − B2 − 17 ms | 方案①下只作记录 |")
    L.append("| 命中数 | 眼睛数（发/中） | 低位靶与人工门会明显影响 |")
    L.append(
        "| 零位 yaw/pitch | 静止靶 3 发 → `py tools\\zero_fix.py` | 写进 yaml，不用环境变量 |"
    )
    L.append("")

    Path(args.out).write_text("\n".join(L), encoding="utf-8")
    print(f"[ok] 已写出 {args.out}（{len(rows)} 个场次，{len(summaries)} 份汇总）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
