#!/usr/bin/env python3
"""把录包的 frames.csv 画成 SVG，只依赖标准库。

存在的原因：车上没有 matplotlib，也没网装。`plot_frames.py` 要 matplotlib，
所以在「只有裸 python3」的机器上跑不了；这个脚本用字符串拼 SVG，浏览器
双击就能看，同样能看到转速曲线、挡位台阶和发弹计数。

用法：
    python3 tools/plot_svg.py recordings/run_1017 -o omega.svg
    py tools/plot_svg.py recordings/run_1017.tar -o omega.svg
    py tools/plot_svg.py recordings/run_1017 --panel w,count

frames.csv 的列（tools/recorder.cpp:49）：
    # frame,t_sec,qw,qx,qy,qz,bullet_speed,bullet_count,mode
"""

from __future__ import annotations

import argparse
import math
import sys
import tarfile
from pathlib import Path

W, H = 1080, 720
ML, MR, MT, MB = 78, 28, 40, 54   # 边距


# ---------------------------------------------------------------- 读数据

def read_frames_csv(path: Path) -> list[dict]:
    if path.is_dir():
        f = path / "frames.csv"
        if not f.exists():
            sys.exit(f"{path} 里没有 frames.csv")
        text = f.read_text(encoding="utf-8", errors="replace")
    elif path.suffix in (".tar", ".tgz", ".tar.gz"):
        with tarfile.open(path) as tf:
            member = next((m for m in tf.getmembers()
                           if m.name.endswith("frames.csv")), None)
            if member is None:
                sys.exit(f"{path} 里没有 frames.csv")
            text = tf.extractfile(member).read().decode("utf-8", "replace")
    else:
        text = path.read_text(encoding="utf-8", errors="replace")

    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines:
        sys.exit("frames.csv 是空的")

    header = lines[0].lstrip("#").strip().split(",")
    rows = []
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) < len(header):
            continue
        try:
            rec = {k: float(v) for k, v in zip(header, parts)}
        except ValueError:
            continue          # 半行（写到一半断电）直接跳过
        rows.append(rec)
    if not rows:
        sys.exit("没有可用的数据行")
    return rows


def omega_series(rows: list[dict]) -> list[tuple[float, float]]:
    """四元数差分 → 角速度（rad/s，取绝对值）。

    相邻帧 q1,q2：dq = q2 * conj(q1)，转角 = 2*acos(|dq.w|)，再除 dt。
    这是「两帧之间转了多少」，对匀速旋转就是真实角速度。
    """
    out = []
    for a, b in zip(rows, rows[1:]):
        dt = b["t_sec"] - a["t_sec"]
        if dt <= 0:
            continue
        q1 = (a["qw"], a["qx"], a["qy"], a["qz"])
        q2 = (b["qw"], b["qx"], b["qy"], b["qz"])
        w = (q2[0] * q1[0] + q2[1] * q1[1] + q2[2] * q1[2] + q2[3] * q1[3])
        w = max(-1.0, min(1.0, w))
        ang = 2.0 * math.acos(abs(w))
        out.append((b["t_sec"], ang / dt))
    return out


# ---------------------------------------------------------------- SVG 基础

def esc(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def nice_ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    if hi <= lo:
        return [lo]
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-9:
        ticks.append(round(v, 10))
        v += step
    return ticks


class Panel:
    def __init__(self, top, height, title, xlo, xhi, ylo, yhi, yunit=""):
        self.top, self.height, self.title = top, height, title
        self.xlo, self.xhi, self.ylo, self.yhi = xlo, xhi, ylo, yhi
        self.yunit = yunit

    def px(self, t):
        return ML + (t - self.xlo) / (self.xhi - self.xlo) * (W - ML - MR)

    def py(self, v):
        span = (self.yhi - self.ylo) or 1.0
        return self.top + self.height - (v - self.ylo) / span * self.height

    def frame(self) -> list[str]:
        s = [f'<text x="{ML}" y="{self.top - 10}" class="ttl">{esc(self.title)}</text>']
        # 横向网格 + y 轴刻度
        for v in nice_ticks(self.ylo, self.yhi, 4):
            y = self.py(v)
            s.append(f'<line x1="{ML}" y1="{y:.1f}" x2="{W - MR}" y2="{y:.1f}" class="grid"/>')
            s.append(f'<text x="{ML - 8}" y="{y + 4:.1f}" class="tick" text-anchor="end">'
                     f'{v:g}{self.yunit}</text>')
        # 纵向网格 + x 轴刻度
        for t in nice_ticks(self.xlo, self.xhi, 6):
            x = self.px(t)
            s.append(f'<line x1="{x:.1f}" y1="{self.top}" x2="{x:.1f}" '
                     f'y2="{self.top + self.height}" class="grid"/>')
            s.append(f'<text x="{x:.1f}" y="{self.top + self.height + 18}" '
                     f'class="tick" text-anchor="middle">{t:g}</text>')
        # 边框
        s.append(f'<rect x="{ML}" y="{self.top}" width="{W - ML - MR}" '
                 f'height="{self.height}" class="box"/>')
        return s

    def line(self, pts, color="#2b6cb0", width=1.2) -> list[str]:
        if len(pts) < 2:
            return []
        d = " ".join(f"{self.px(t):.1f},{self.py(v):.1f}" for t, v in pts)
        if len(pts) > 4000:      # 点太多时降采样，避免 SVG 巨大
            step = len(pts) // 4000 + 1
            d = " ".join(f"{self.px(t):.1f},{self.py(v):.1f}" for t, v in pts[::step])
        return [f'<polyline points="{d}" fill="none" stroke="{color}" '
                f'stroke-width="{width}" stroke-linejoin="round"/>']

    def dots(self, pts, color="#c53030", r=2.6) -> list[str]:
        return [f'<circle cx="{self.px(t):.1f}" cy="{self.py(v):.1f}" r="{r}" fill="{color}"/>'
                for t, v in pts]


# ---------------------------------------------------------------- 主流程

def main() -> None:
    ap = argparse.ArgumentParser(description="frames.csv → SVG（不依赖 matplotlib）")
    ap.add_argument("path", help="录包目录、frames.csv 或 .tar")
    ap.add_argument("-o", "--out", default="omega.svg")
    ap.add_argument("--panel", default="w,count,mode",
                    help="要画的曲线，逗号分隔：w,count,speed,mode")
    args = ap.parse_args()

    rows = read_frames_csv(Path(args.path))
    t = [r["t_sec"] for r in rows]
    t0 = t[0]
    t = [x - t0 for x in t]
    for r, x in zip(rows, t):
        r["t_sec"] = x

    w = omega_series(rows)
    want = [p.strip() for p in args.panel.split(",") if p.strip()]

    dur = max(t) or 1.0
    panels = []
    top = MT
    if "w" in want:
        vals = [v for _, v in w] or [0.0]
        hi = max(3.5, max(vals) * 1.15)
        panels.append(Panel(top, 190, "角速度 |ω|（rad/s，由四元数差分）",
                            w[0][0] if w else 0, dur, 0, hi, ""))
        top += 250
    if "count" in want:
        cs = [r.get("bullet_count", 0.0) for r in rows]
        lo, hi = min(cs), max(cs)
        panels.append(Panel(top, 120, "发弹计数 bullet_count",
                            t[0], dur, lo - 2, max(hi + 10, lo + 12), ""))
        top += 180
    if "speed" in want:
        sp = [r.get("bullet_speed", 0.0) for r in rows]
        lo, hi = min(sp), max(sp)
        panels.append(Panel(top, 120, "弹速 bullet_speed（m/s）",
                            t[0], dur, lo - 1, hi + 1, ""))
        top += 180
    if "mode" in want:
        md = [r.get("mode", 0.0) for r in rows]
        panels.append(Panel(top, 110, "模式 mode（0/1/2/3，台阶就是切换时刻）",
                            t[0], dur, -0.5, 3.5, ""))
        top += 170

    body = []
    for i, p in enumerate(panels):
        body += p.frame()
        if i == 0 and "w" in want:
            body += p.line(w, "#2b6cb0", 1.4)
        elif p.title.startswith("发弹计数"):
            pts = list(zip(t, [r.get("bullet_count", 0.0) for r in rows]))
            body += p.line(pts, "#2f855a", 1.6)
            body += p.dots([q for q in pts if q[1] > 0], "#2f855a", 2.0)
        elif p.title.startswith("弹速"):
            body += p.line(list(zip(t, [r.get("bullet_speed", 0.0) for r in rows])),
                           "#b7791f", 1.4)
        elif p.title.startswith("模式"):
            body += p.line(list(zip(t, [r.get("mode", 0.0) for r in rows])),
                           "#6b46c1", 1.6)

    n = len(rows)
    fps = (n - 1) / dur if dur > 0 else 0.0
    head = (f"{Path(args.path).name} — {n} 帧 / {dur:.1f} s / {fps:.1f} fps")

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}">',
           '<style>'
           'text{font-family:ui-sans-serif,Segoe UI,Helvetica,Arial;fill:#1a202c}'
           '.ttl{font-size:15px;font-weight:600}'
           '.tick{font-size:11px;fill:#4a5568}'
           '.hdr{font-size:13px;fill:#2d3748}'
           '.grid{stroke:#e2e8f0;stroke-width:1}'
           '.box{fill:none;stroke:#a0aec0;stroke-width:1}'
           '</style>',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{ML}" y="24" class="hdr">{esc(head)}</text>']
    svg += body
    svg.append("</svg>")

    out = Path(args.out)
    out.write_text("\n".join(svg) + "\n", encoding="utf-8")
    print(f"写出 {out}（{n} 帧，{len(panels)} 个面板，{out.stat().st_size // 1024} KB）")
    if w:
        vals = sorted(v for _, v in w)
        p50 = vals[len(vals) // 2]
        print(f"  |ω| 中位 {p50:.2f} rad/s，峰值 {vals[-1]:.2f} rad/s")


if __name__ == "__main__":
    main()
