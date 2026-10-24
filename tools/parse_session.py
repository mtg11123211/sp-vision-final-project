#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立判读脚本：把一场自瞄测试的日志（含退出时自动打出的汇总块）解析成一份可归档的记录。

用法（Windows 笔记本）::

    py tools\\parse_session.py logs\\p5_base.log
    py tools\\parse_session.py logs\\p5_base.log --lat logs\\lat_1510.csv --tag p5_base
    py tools\\parse_session.py logs\\*.log --csv logs\\sessions.csv --out logs\\records.md

它做三件事：
  1) 从日志里抽出**实际生效的参数**（启动那几行）与**实测数值**（arrive / fire-gate / 选板变化）；
  2) 如果日志里已经有程序退出时打出的「测试汇总」块，就优先用汇总块里的数（那是程序自己算的，更准）；
  3) 输出一段 Markdown，可直接贴进 docs/record_session.md；--csv 还会往一张总表里追加一行。

只依赖标准库（argparse / re / csv / pathlib / statistics），车上、笔记本都能跑。

注意：日志行前面可能带 logger 的时间戳/level 前缀，所以所有匹配都**不锚定行首**。

⚠ 本脚本在开发机（Windows，无 Python）只做过语法检查，**没有实跑过**。
第一次用请先 `py tools\\parse_session.py --help` 确认能跑，再拿一份真实日志试；
若某项解析不出来，多半是日志格式变了，对照 `src/task_2.cpp` 里的格式串改这里的正则即可。
"""

import argparse
import csv
import re
import statistics
import sys
from pathlib import Path

# GBK 控制台（cp936）编码不了 ⚠/⇒/·：输出一重定向到文件就 UnicodeEncodeError，
# 这里统一降级成 '?'，别让一个符号把整份报告带走。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(errors="replace")

# ---------------------------------------------------------------- 正则（都不锚定行首）

RE_STARTUP = re.compile(r"\[task2\] (?P<body>.*)$")
RE_ARRIVE = re.compile(
    r"\[dbg\] arrive=(?P<arrive>\w+) tol=(?P<tol>[\d.]+)/(?P<tolph>[\d.]+)"
    r"°\(eff (?P<tol_eff>[\d.]+)°\) errY=(?P<errY>[\d.]+)° errP=(?P<errP>[\d.]+)"
    r"° Ymin/max=(?P<ymin>[\d.]+)/(?P<ymax>[\d.]+) n=(?P<n>\d+)"
    r" blocked=(?P<blocked>\d+) tripped=(?P<tripped>\w+) jump=(?P<jump>[\d.]+)°"
)
RE_GATE = re.compile(
    r"\[dbg\] fire-gate aim_id=(?P<aim>-?\d+) stable=(?P<stable>\w+)"
    r" conv=(?P<conv>\w+) frames=(?P<frames>\d+)/(?P<need>\d+)"
    r" nis=(?P<nis>[\d.]+)/(?P<nis_th>[\d.]+) interval=(?P<interval>\w+)"
    r" rounds=(?P<rounds>\w+) mode=(?P<mode>\w+) hold=(?P<hold>\w+)"
    r" plate=(?P<plate>\w+)\((?P<facing>-?[\d.]+)°→到达(?P<farr>-?[\d.]+)°\)"
    r" low=(?P<low>\w+)\(dz=(?P<dz>[\d.]+)m 低(?P<lowc>\d+)共(?P<tot>\d+)块\)"
    r" key=(?P<key>\w+) t_arr=(?P<tarr>\d+)ms assoc=(?P<assoc>-?[\d.]+)°"
)
RE_SWITCH = re.compile(r"\[dbg\] 选板变化 aim_id (?P<old>-?\d+) -> (?P<new>-?\d+)")
RE_SUM_HEAD = "测试汇总"
RE_SUM_TAIL = "测试汇总 结束"

# 启动日志里我们关心的几类行
STARTUP_KEYS = (
    "到位门控",
    "低位靶规则",
    "出弹延时",
    "预测时域",
    "offset",
    "⚠",
    "开火门",
)


def _b(v):
    """日志里的布尔可能是 true/True/1。"""
    return str(v).strip().lower() in ("true", "1", "yes")


def parse_log(path):
    """返回 (startup 行列表, arrive 行, fire-gate 行, 选板变化次数, 按键边沿次数, 汇总块文本)。"""
    startup, arrive_rows, gate_rows, switches, keyedges = [], [], [], 0, 0
    summary_text = ""
    in_summary = False
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.rstrip()
        if RE_SUM_HEAD in line and "结束" not in line:
            in_summary = True
            summary_text += line + "\n"
            continue
        if RE_SUM_TAIL in line:
            in_summary = False
            summary_text += line + "\n"
            continue
        if in_summary:
            summary_text += line + "\n"
            continue
        if "[task2] " in line:
            if "开火门按键边沿" in line:
                keyedges += 1
                continue
            if any(k in line for k in STARTUP_KEYS):
                startup.append(line[line.find("[task2]") :])
                continue
        m = RE_ARRIVE.search(line)
        if m:
            arrive_rows.append(m.groupdict())
            continue
        m = RE_GATE.search(line)
        if m:
            gate_rows.append(m.groupdict())
            continue
        if RE_SWITCH.search(line):
            switches += 1
    return startup, arrive_rows, gate_rows, switches, keyedges, summary_text


def from_rows(arrive_rows, gate_rows, switches, keyedges):
    d = {}
    if arrive_rows:
        errs = [float(r["errY"]) for r in arrive_rows]
        d["errY_mean"] = statistics.fmean(errs)
        d["errY_max"] = max(errs)
        d["errY_min"] = min(errs)
        d["tol"] = float(arrive_rows[-1]["tol"])
        d["tol_eff"] = float(arrive_rows[-1]["tol_eff"])
        d["blocked"] = int(arrive_rows[-1]["blocked"])
        d["jump_max"] = max(float(r["jump"]) for r in arrive_rows)
    if gate_rows:
        cnt = {}
        for r in gate_rows:
            for k in ("stable", "interval", "rounds", "mode", "hold", "plate", "key"):
                if not _b(r[k]):
                    cnt[k] = cnt.get(k, 0) + 1
        d["gate_false"] = cnt
        d["gate_rows"] = len(gate_rows)
        d["nis_mean"] = statistics.fmean([float(r["nis"]) for r in gate_rows])
        d["t_arr_mean"] = statistics.fmean([float(r["tarr"]) for r in gate_rows])
        d["dz_max"] = max(float(r["dz"]) for r in gate_rows)
        d["assoc_last"] = float(gate_rows[-1]["assoc"])
        d["rounds_last"] = gate_rows[-1]["rounds"]
    d["switches"] = switches
    d["keyedges"] = keyedges
    return d


def summary_pairs(summary_text):
    """从程序自己打的汇总块里抠出 key: value。"""
    out = {}
    for line in summary_text.splitlines():
        s = line.strip()
        if ":" not in s:
            continue
        k, _, v = s.partition(":")
        k = k.strip().lstrip("· ").strip()
        v = v.strip()
        if k and v and len(k) < 40:
            out.setdefault(k, v)
    return out


def render(tag, startup, d, summary_text, lat_path):
    sp = summary_pairs(summary_text)
    L = []
    L.append(f"### 场次 `{tag}`" if tag else "### 场次")
    L.append("")
    L.append("**本次实际生效的参数（启动日志原样）**")
    L.append("")
    L.append("```")
    for s in startup:
        L.append(s)
    L.append("```")
    L.append("")
    L.append("| 组 | 项 | 值 | 判据 |")
    L.append("|---|---|---|---|")

    def row(g, k, v, crit=""):
        L.append(f"| {g} | {k} | {v} | {crit} |")

    row("A", "A1 电控挡位", sp.get("A1 电控挡位", "（未测）"), "必须 AUTO_AIM")
    row(
        "A",
        "A2 云台滞后 errY",
        sp.get("A2 云台滞后 errY", "（未测）"),
        "均值应 < 门限；> 门限 ⇒ 门限太紧",
    )
    row(
        "A",
        "A3 按键通道",
        sp.get("A3 按键通道", "（未测）"),
        "evdev 最好；cvkey ⇒ 窗口没焦点",
    )
    row(
        "B",
        "B1 视觉延迟",
        sp.get("B1 视觉延迟", "（未测）"),
        "latency_report.py 读 lat CSV",
    )
    row(
        "B",
        "B2 云台反馈滞后",
        sp.get("B2 云台反馈滞后", "（未测）"),
        "同上；相关系数 ≥0.5 才认",
    )
    row(
        "B",
        "B3 出弹延时",
        sp.get("B3 出弹延时", "（未测）"),
        "与 B1+B2 之和接近 ⇒ 并进 DSH_LEAD",
    )
    row(
        "B",
        "总提前量 (DSH_LEAD)",
        sp.get("总提前量", "（未测）"),
        "方案①：唯一功能旋钮",
    )
    row("B", "飞行时间", sp.get("飞行时间", "（未测）"), "calc_aim 自动")
    row(
        "C",
        "C1 高度差 x[10]",
        sp.get("C1 高度差 x[10]", "（未测）"),
        "≈ 容差 ⇒ 低位靶规则在起作用",
    )
    row("C", "C2 角速度 w", sp.get("C2 角速度 w", "（未测）"), "与真值比")
    row(
        "C", "C3 选板切换", sp.get("C3 选板切换", "（未测）"), "4 rad/s 期望 ≈2.5 次/秒"
    )
    L.append("")
    L.append("**统计（程序退出时自动输出）**")
    L.append("")
    L.append("```")
    for s in summary_text.splitlines():
        L.append(s)
    L.append("```")
    L.append("")
    if d:
        L.append(
            "**从日志行直接统计（与上面的汇总块互为校验；fire-gate 每 48 帧才打一次 ⇒ "
            "这里是「日志行条数」不是帧数）**"
        )
        L.append("")
        L.append(
            f"- fire-gate 日志 {d.get('gate_rows', 0)} 条；选板变化 {d.get('switches', 0)} 次；"
            f"按键边沿 {d.get('keyedges', 0)} 次"
        )
        if "errY_mean" in d:
            L.append(
                f"- errY：均值 {d['errY_mean']:.2f}°，最小 {d['errY_min']:.2f}°，"
                f"最大 {d['errY_max']:.2f}°；门限 {d['tol']:.2f}°（eff {d['tol_eff']:.2f}°），"
                f"blocked 累计 {d['blocked']}，命令跳变最大 {d['jump_max']:.2f}°"
            )
        if "gate_false" in d:
            L.append(f"- 各判据在日志行里为假的次数：{d['gate_false']}")
        if "t_arr_mean" in d:
            L.append(
                f"- t_arr 均值 {d['t_arr_mean']:.0f} ms，dz 最大 {d['dz_max']:.3f} m，"
                f"assoc 末值 {d['assoc_last']:.1f}°，nis 均值 {d['nis_mean']:.2f}"
            )
    if lat_path:
        L.append("")
        L.append(
            f"（延迟 CSV：{lat_path} —— 用 `py tools/latency_report.py {lat_path}` 取 B1/B2）"
        )
    L.append("")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="判读一场自瞄测试的日志，输出可归档的记录")
    ap.add_argument(
        "log", nargs="+", help="日志文件（可多个，支持通配符由 shell 展开）"
    )
    ap.add_argument("--lat", default="", help="对应的延迟 CSV（lat_*.csv），只做备注")
    ap.add_argument("--tag", default="", help="场次名（默认取文件名）")
    ap.add_argument("--out", default="", help="输出的 Markdown 文件")
    ap.add_argument("--csv", default="", help="往这张总表里追加一行")
    args = ap.parse_args()

    blocks, rows = [], []
    for lg in args.log:
        p = Path(lg)
        if not p.is_file():
            print(f"[warn] 找不到 {p}", file=sys.stderr)
            continue
        startup, a, g, sw, ke, summ = parse_log(p)
        d = from_rows(a, g, sw, ke)
        tag = args.tag or p.stem
        blocks.append(render(tag, startup, d, summ, args.lat))
        rows.append(
            {
                "tag": tag,
                "log": p.name,
                "errY_mean": f"{d.get('errY_mean', 0.0):.2f}",
                "errY_max": f"{d.get('errY_max', 0.0):.2f}",
                "tol_eff": f"{d.get('tol_eff', 0.0):.2f}",
                "blocked": d.get("blocked", ""),
                "switches": d.get("switches", ""),
                "dz_max": f"{d.get('dz_max', 0.0):.3f}",
                "rounds": d.get("rounds_last", ""),
                "lead": summary_pairs(summ).get("总提前量", ""),
            }
        )

    if not blocks:
        print("[error] 没有可解析的日志", file=sys.stderr)
        return 1
    text = "\n\n---\n\n".join(blocks)
    print(text)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"\n[ok] 已写出 {args.out}", file=sys.stderr)
    if args.csv and rows:
        path = Path(args.csv)
        exists = path.exists()
        with path.open("a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            if not exists:
                w.writeheader()
            w.writerows(rows)
        print(f"[ok] 已追加 {len(rows)} 行到 {args.csv}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
