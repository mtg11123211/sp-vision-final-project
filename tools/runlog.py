#!/usr/bin/env python3
"""现场记录：把每次跑车的关键读数存成一条，最后横向对比。

为什么不是一张 Excel：现场是终端，敲一条命令比打开表格快得多；而数据要能
直接对着日志/CSV 复核，所以每条记录都带 evidence 路径。

用法（笔记本或车机都行，只依赖标准库）：

    py tools/runlog.py new run1_1018 --task task2 --stage P1
    py tools/runlog.py set run1_1018 fps=57.9 det_pct=100 lead_ms=0.075
    py tools/runlog.py lat run1_1018 logs/lat_1530.csv   # 自动填延迟和 fps
    py tools/runlog.py show
    py tools/runlog.py export                            # 汇总成 docs/runs/runs.csv

存哪：docs/runs/<名字>.json，一条一个文件。故意不用数据库——现场最怕
"一个文件坏了全没了"，一个 json 坏了只丢一场。
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

RUNS = Path(__file__).resolve().parent.parent / "docs" / "runs"

# 字段顺序 = show 的列顺序。数字型放前面，判定型放后面。
FIELDS = [
    ("task", str), ("stage", str), ("date", str),
    ("fps", float), ("det_pct", float),
    ("lat_vision_ms", float), ("lat_send_ms", float),
    ("D_ms", float), ("tau_ms", float),
    ("lead_suggest_ms", float), ("lead_ms", float),
    ("off_yaw", float), ("off_pitch", float),
    ("arrive_deg", float), ("color_order", str),
    ("shots", int), ("hits", int),
    ("dx_mm", float), ("dy_mm", float), ("dist_m", float),
    ("verdict", str), ("note", str), ("evidence", str),
]
CAST = dict(FIELDS)

# show 时优先显示这些列，其余只在 detail 里出现
BRIEF = ["stage", "fps", "lat_vision_ms", "tau_ms", "lead_ms",
         "shots", "hits", "dx_mm", "dy_mm", "verdict"]


def path_of(name: str) -> Path:
    return RUNS / f"{name}.json"


def load(name: str) -> dict:
    p = path_of(name)
    if not p.exists():
        sys.exit(f"没有这场记录：{p}（先 runlog.py new {name}）")
    return json.loads(p.read_text(encoding="utf-8"))


def save(name: str, rec: dict) -> None:
    RUNS.mkdir(parents=True, exist_ok=True)
    path_of(name).write_text(
        json.dumps(rec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def coerce(key: str, raw: str):
    fn = CAST.get(key, str)
    try:
        return fn(raw)
    except ValueError:
        sys.exit(f"字段 {key} 要 {fn.__name__}，收到 {raw!r}")


def cmd_new(args) -> None:
    p = path_of(args.name)
    if p.exists() and not args.force:
        sys.exit(f"已存在：{p}（要重建加 --force）")
    rec = {k: "" for k, _ in FIELDS}
    import datetime
    rec["date"] = datetime.date.today().isoformat()
    rec["task"] = args.task
    rec["stage"] = args.stage
    if args.evidence:
        rec["evidence"] = args.evidence
    save(args.name, rec)
    print(f"建好：{p}")


def cmd_set(args) -> None:
    rec = load(args.name)
    for item in args.kv:
        if "=" not in item:
            sys.exit(f"要 k=v 形式：{item}")
        k, v = item.split("=", 1)
        if k not in CAST:
            sys.exit(f"未知字段 {k}；可用：{', '.join(k for k, _ in FIELDS)}")
        rec[k] = coerce(k, v)
    save(args.name, rec)
    changed = ", ".join(f"{i.split('=', 1)[0]}={i.split('=', 1)[1]}" for i in args.kv)
    print(f"记下：{changed}")


def cmd_lat(args) -> None:
    """从探针 CSV 自动填视觉/串口延迟与 fps。

    只填「中位数」这两个最不会错的数；D 和 τ 需要阶跃拟合，交给
    latency_report.py，再用 set 抄回来——宁可手抄也不要在这里猜。
    """
    csv_path = Path(args.csv)
    if not csv_path.exists():
        sys.exit(f"找不到 {csv_path}")

    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("CSV 是空的")

    def ns(row, key):
        try:
            return int(row[key])
        except (KeyError, ValueError):
            return None

    vis, snd, grabs = [], [], []
    for r in rows:
        g, i, s = ns(r, "grab_ns"), ns(r, "infer_ns"), ns(r, "send_ns")
        if g is None:
            continue
        grabs.append(g)
        if i is not None and i > g:
            vis.append((i - g) / 1e6)
        if s is not None and i is not None and s > i:
            snd.append((s - i) / 1e6)

    if not vis:
        sys.exit("这行没有可用的 grab_ns/infer_ns（探针没打开？DSH_LAT_CSV 设了吗）")

    rec = load(args.name)
    rec["lat_vision_ms"] = round(statistics.median(vis), 2)
    if snd:
        rec["lat_send_ms"] = round(statistics.median(snd), 2)
    if len(grabs) > 1:
        span = (grabs[-1] - grabs[0]) / 1e9
        if span > 0:
            rec["fps"] = round((len(grabs) - 1) / span, 1)
    rec["evidence"] = str(csv_path)
    save(args.name, rec)

    print(f"{csv_path.name}: {len(rows)} 行")
    print(f"  视觉 p50 = {rec['lat_vision_ms']} ms   （取帧 → 解算完）")
    if snd:
        print(f"  串口 p50 = {rec['lat_send_ms']} ms   （解算完 → 串口写完）")
    if rec.get("fps"):
        print(f"  fps      = {rec['fps']}   （由 grab_ns 间隔反推）")
    print("  D / τ 还没填：跑 latency_report.py，然后把 D_ms=… tau_ms=… 用 set 抄进来")


def cmd_show(args) -> None:
    files = sorted(RUNS.glob("*.json")) if RUNS.exists() else []
    if args.name:
        files = [f for f in files if f.stem == args.name]
    if not files:
        sys.exit("还没有任何记录（runlog.py new <名字>）")

    rows = []
    for f in files:
        rec = json.loads(f.read_text(encoding="utf-8"))
        rows.append((f.stem, rec))

    name_w = max(len(r[0]) for r in rows) + 2

    table = []
    widths = [max(len(k), 4) for k in BRIEF]
    for _, rec in rows:
        cells = []
        for k in BRIEF:
            v = rec.get(k, "")
            cells.append(f"{v:.1f}" if isinstance(v, float) else str(v))
        table.append(cells)
        widths = [max(w, len(c)) for w, c in zip(widths, cells)]

    head = " ".join(k.replace("_", " ").ljust(w) for k, w in zip(BRIEF, widths))
    print("名字".ljust(name_w) + head)
    print("-" * (name_w + sum(widths) + len(widths) - 1))
    for (name, _), cells in zip(rows, table):
        print(name.ljust(name_w) + " ".join(c.ljust(w) for c, w in zip(cells, widths)))

    if args.verbose:
        print()
        for name, rec in rows:
            print(f"[{name}]")
            for k, _ in FIELDS:
                if rec.get(k, "") != "":
                    print(f"  {k:16s} {rec[k]}")
            print()


def cmd_export(args) -> None:
    files = sorted(RUNS.glob("*.json"))
    if not files:
        sys.exit("还没有任何记录")
    out = RUNS / "runs.csv"
    cols = ["name"] + [k for k, _ in FIELDS]
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for p in files:
            rec = json.loads(p.read_text(encoding="utf-8"))
            w.writerow([p.stem] + [rec.get(k, "") for k, _ in FIELDS])
    print(f"写出 {out}（{len(files)} 行）")


def main() -> None:
    ap = argparse.ArgumentParser(description="现场记录：一条记录一场")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("new", help="新建一场")
    p.add_argument("name")
    p.add_argument("--task", default="task2")
    p.add_argument("--stage", default="")
    p.add_argument("--evidence", default="")
    p.add_argument("--force", action="store_true")
    p.set_defaults(fn=cmd_new)

    p = sub.add_parser("set", help="写字段，k=v 可多个")
    p.add_argument("name")
    p.add_argument("kv", nargs="+")
    p.set_defaults(fn=cmd_set)

    p = sub.add_parser("lat", help="从探针 CSV 自动填延迟/fps")
    p.add_argument("name")
    p.add_argument("csv")
    p.set_defaults(fn=cmd_lat)

    p = sub.add_parser("show", help="所有场次对比表")
    p.add_argument("name", nargs="?", default="")
    p.add_argument("-v", "--verbose", action="store_true")
    p.set_defaults(fn=cmd_show)

    p = sub.add_parser("export", help="汇总成 runs.csv")
    p.set_defaults(fn=cmd_export)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
