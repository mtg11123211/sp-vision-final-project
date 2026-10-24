#!/usr/bin/env python3
"""把录包的 frames.csv 画成曲线，重点是把四元数换算成角速度。

为什么值得单独画：frames.csv 里没有 w 这一列，飞行器（靶机/云台）的角速度
只存在于四元数序列里。相邻两帧的四元数做差就能还原出这一帧的转动角速度，
这正是任务二那 20 分、任务三那 5 分要看的东西。

用法：
    python tools/plot_frames.py recordings/synthetic_rotate/frames.csv
    python tools/plot_frames.py recordings/            # 目录：递归找所有 frames.csv
    python tools/plot_frames.py all_csv_1017_1535.tar  # 车上打回来的包

输出：每份 csv 旁边生成一张 <录包名>_curves.png，并把关键数字打在终端上。

只依赖 numpy + matplotlib，没有 matplotlib 时会退化成纯统计（不画图）。
"""
import argparse
import csv
import io
import math
import os
import sys
import tarfile
from typing import List, Optional, Tuple

import numpy as np

COLS = ["frame", "t_sec", "qw", "qx", "qy", "qz", "bullet_speed", "bullet_count", "mode"]
MODE_NAMES = {0: "IDLE", 1: "AUTO_AIM", 2: "SMALL_BUFF", 3: "BIG_BUFF"}


class Rec:
    def __init__(self, name: str, path: str):
        self.name = name
        self.path = path
        self.t: List[float] = []
        self.q: List[Tuple[float, float, float, float]] = []
        self.speed: List[float] = []
        self.count: List[int] = []
        self.mode: List[int] = []
        self.bad_lines: List[int] = []

    # ---- 读 ----
    def load_from_text(self, text: str) -> None:
        reader = csv.reader(io.StringIO(text))
        header = next(reader, None)
        if header is None:
            return
        idx = {name.strip(): i for i, name in enumerate(header)}
        need = [c for c in COLS if c in idx]
        if len(need) < 4:
            raise ValueError("表头里认不出 %s，实际是 %s" % (COLS, header))
        for lineno, row in enumerate(reader, start=2):
            if not row or len(row) < len(header):
                if row:
                    self.bad_lines.append(lineno)
                continue
            try:
                t = float(row[idx["t_sec"]])
                q = (
                    float(row[idx["qw"]]),
                    float(row[idx["qx"]]),
                    float(row[idx["qy"]]),
                    float(row[idx["qz"]]),
                )
                speed = float(row[idx["bullet_speed"]]) if "bullet_speed" in idx else 0.0
                count = int(float(row[idx["bullet_count"]])) if "bullet_count" in idx else 0
                mode = int(float(row[idx["mode"]])) if "mode" in idx else -1
            except (ValueError, IndexError):
                self.bad_lines.append(lineno)
                continue
            self.t.append(t)
            self.q.append(q)
            self.speed.append(speed)
            self.count.append(count)
            self.mode.append(mode)

    # ---- 算 ----
    def analyse(self) -> dict:
        out = {"name": self.name, "path": self.path, "n": len(self.t)}
        if len(self.t) < 3:
            return out
        t = np.asarray(self.t, dtype=float)
        dt = np.diff(t)
        out["duration"] = float(t[-1] - t[0])
        out["dt_med"] = float(np.median(dt))
        out["fps"] = 1.0 / out["dt_med"] if out["dt_med"] > 0 else float("nan")
        out["dt_max"] = float(dt.max())
        gaps = np.where(dt > 3.0 * out["dt_med"])[0]
        out["gaps"] = [
            (float(t[i]), float(dt[i])) for i in gaps[:10]
        ]  # (发生在哪个时刻, 缺口多长)

        q = np.asarray(self.q, dtype=float)
        norm = np.linalg.norm(q, axis=1)
        out["q_norm_min"] = float(norm.min())
        out["q_norm_max"] = float(norm.max())
        q = q / norm[:, None]

        # 相邻两帧的相对转动：|dot| 消掉 q/-q 同一姿态的符号问题
        dot = np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), -1.0, 1.0)
        dtheta = 2.0 * np.arccos(dot)  # 每帧转过的角度(rad)

        # 机体坐标系下的角速度：dq = conj(q_k) * q_{k+1}
        qa, qb = q[:-1], q[1:]
        w = qa[:, 0]
        v = qa[:, 1:]
        dq_w = w * qb[:, 0] + np.sum(v * qb[:, 1:], axis=1)
        dq_v = (
            qb[:, 0:1] * (-v)
            + w[:, None] * qb[:, 1:]
            + np.cross(v, qb[:, 1:])
        )
        sgn = np.where(dq_w < 0, -1.0, 1.0)
        dq_v = dq_v * sgn[:, None]
        dq_w = dq_w * sgn

        safe_dt = np.where(dt > 1e-9, dt, np.nan)
        omega = 2.0 * dq_v / safe_dt[:, None]  # rad/s, 机体系 (x,y,z)
        mag = np.linalg.norm(omega, axis=1)

        out["omega"] = omega
        out["mag"] = mag
        out["dtheta"] = dtheta
        out["t_mid"] = 0.5 * (t[:-1] + t[1:])
        out["cum_deg"] = np.degrees(np.cumsum(dtheta))
        out["t_mid_all"] = t[:-1]

        finite = mag[np.isfinite(mag)]
        if finite.size:
            out["w_p50"] = float(np.percentile(finite, 50))
            out["w_p95"] = float(np.percentile(finite, 95))
            out["w_max"] = float(finite.max())
            out["w_mean"] = float(finite.mean())
        # 主轴：哪个分量占主导
        if finite.size:
            comp = np.nan_to_num(np.nanmean(np.abs(omega), axis=0), nan=0.0)
            total = float(comp.sum())
            if total > 1e-9:
                out["axis_share"] = comp / total
                out["axis"] = "xyz"[int(np.argmax(comp))]
            else:  # 完全静止：没有主轴可言
                out["axis_share"] = np.zeros(3)
                out["axis"] = "-"
        # 四元数总共扫过多少度
        out["sweep_deg"] = float(np.degrees(dtheta.sum()))

        out["speed_min"] = float(np.min(self.speed))
        out["speed_max"] = float(np.max(self.speed))
        out["count_first"] = int(self.count[0])
        out["count_last"] = int(self.count[-1])
        out["count_inc"] = int(self.count[-1] - self.count[0])
        out["mode_hist"] = {
            MODE_NAMES.get(m, str(m)): int(sum(1 for x in self.mode if x == m))
            for m in sorted(set(self.mode))
        }
        return out

    # ---- 画 ----
    def plot(self, a: dict, out_png: str) -> bool:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:  # noqa: BLE001
            print("  [warn] 没有 matplotlib，跳过画图：%s" % e)
            return False

        if "mag" not in a:
            return False

        tm = a["t_mid"]
        omega, mag = a["omega"], a["mag"]
        t_all = np.asarray(self.t, dtype=float)
        speed = np.asarray(self.speed, dtype=float)
        count = np.asarray(self.count, dtype=float)
        mode = np.asarray(self.mode, dtype=float)

        fig, axes = plt.subplots(4, 1, figsize=(12, 9.6), dpi=130, sharex=True)

        ax = axes[0]
        ax.plot(tm, mag, lw=1.6, color="#1f77b4", label="|w| from dq")
        ax.plot(tm, omega[:, 0], lw=0.9, color="#d62728", alpha=0.75, label="w_x")
        ax.plot(tm, omega[:, 1], lw=0.9, color="#2ca02c", alpha=0.75, label="w_y")
        ax.plot(tm, omega[:, 2], lw=0.9, color="#9467bd", alpha=0.75, label="w_z")
        ax.axhline(0, color="k", lw=0.6, alpha=0.4)
        ax.set_ylabel("angular velocity (rad/s)")
        ax.legend(loc="upper right", fontsize=8, ncol=4)
        ax.grid(alpha=0.3)
        ax.set_title(
            "%s   n=%d   fps=%.1f   |w| p50=%.2f p95=%.2f max=%.2f rad/s"
            % (a["name"], a["n"], a["fps"], a.get("w_p50", 0), a.get("w_p95", 0), a.get("w_max", 0)),
            fontsize=10,
        )

        ax = axes[1]
        ax.plot(a["t_mid_all"], a["cum_deg"], lw=1.6, color="#ff7f0e")
        ax.set_ylabel("cumulative rotation (deg)")
        ax.grid(alpha=0.3)
        ax.text(
            0.01, 0.95, "sweep = %.1f deg over %.3f s" % (a["sweep_deg"], a["duration"]),
            transform=ax.transAxes, fontsize=9, va="top",
        )

        ax = axes[2]
        ax.step(t_all, speed, where="post", lw=1.4, color="#8c564b", label="bullet_speed (m/s)")
        ax.set_ylabel("bullet_speed")
        ax.set_ylim(bottom=min(-1.0, float(speed.min()) - 1.0))
        ax2 = ax.twinx()
        ax2.step(t_all, count, where="post", lw=1.2, color="#17becf", label="bullet_count")
        ax2.set_ylabel("bullet_count", color="#17becf")
        ax2.tick_params(axis="y", colors="#17becf")
        # 发弹数常年是不变的 0，matplotlib 会自动缩放到 ±0.04 这种没意义的范围，
        # 反而让人以为它在动。这里手动给一个「看起来就是没涨」的窗口。
        cmin, cmax = float(count.min()), float(count.max())
        if cmax - cmin < 1.0:
            ax2.set_ylim(cmin - 2.0, cmax + 10.0)
        else:
            ax2.set_ylim(cmin - 1.0, cmax * 1.15 + 1.0)
        ax.grid(alpha=0.3)

        ax = axes[3]
        ax.step(t_all, mode, where="post", lw=1.6, color="#7f7f7f")
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(["IDLE", "AUTO", "S_BUFF", "B_BUFF"], fontsize=8)
        ax.set_ylabel("mode")
        ax.set_xlabel("t (s)")
        ax.grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(out_png)
        plt.close(fig)
        return True


# ---------- 收集输入 ----------
def gather(path: str) -> List[Rec]:
    recs: List[Rec] = []
    if not os.path.exists(path):
        raise SystemExit("找不到 %s" % path)

    if os.path.isdir(path):
        for root, _dirs, files in os.walk(path):
            for f in sorted(files):
                if f.lower().endswith(".csv"):
                    full = os.path.join(root, f)
                    recs.append(read_file(full))
        return recs

    if tarfile.is_tarfile(path):
        with tarfile.open(path) as tf:
            for m in tf.getmembers():
                if not m.isfile() or not m.name.lower().endswith(".csv"):
                    continue
                f = tf.extractfile(m)
                if f is None:
                    continue
                r = Rec(os.path.basename(os.path.dirname(m.name)) + "/" + os.path.basename(m.name), path)
                r.load_from_text(f.read().decode("utf-8", "replace"))
                recs.append(r)
        return recs

    recs.append(read_file(path))
    return recs


def read_file(full: str) -> Rec:
    name = os.path.basename(os.path.dirname(full)) or os.path.basename(full)
    r = Rec(name, full)
    with open(full, "rb") as fh:
        r.load_from_text(fh.read().decode("utf-8", "replace"))
    return r


def main() -> int:
    ap = argparse.ArgumentParser(description="把 frames.csv 画成曲线（含四元数反解角速度）")
    ap.add_argument("path", help="frames.csv / 录包目录 / .tar")
    ap.add_argument("--out-dir", default="", help="图片输出目录，默认跟 csv 放一起")
    args = ap.parse_args()

    recs = gather(args.path)
    if not recs:
        print("没找到任何 csv")
        return 1

    for r in recs:
        a = r.analyse()
        print("=" * 78)
        print("%s" % r.path)
        print("  帧数 %d   时长 %.3f s   帧率 %.1f fps   dt 中位 %.4f s   dt 最大 %.4f s"
              % (a["n"], a.get("duration", 0), a.get("fps", 0), a.get("dt_med", 0), a.get("dt_max", 0)))
        if a.get("gaps"):
            print("  时间戳缺口 %d 处：%s" % (len(a["gaps"]), a["gaps"]))
        if r.bad_lines:
            print("  坏行 %d 条：%s" % (len(r.bad_lines), r.bad_lines[:8]))
        if "mag" in a:
            print("  |w|: p50 %.2f  p95 %.2f  max %.2f  mean %.2f rad/s"
                  % (a["w_p50"], a["w_p95"], a["w_max"], a["w_mean"]))
            print("  主轴 %s   分量占比 x %.2f y %.2f z %.2f"
                  % (a["axis"], *a["axis_share"]))
            print("  累计转过 %.1f 度（%.3f s）" % (a["sweep_deg"], a["duration"]))
        print("  四元数模长 %.6f .. %.6f" % (a["q_norm_min"], a["q_norm_max"]))
        print("  bullet_speed %.2f .. %.2f    bullet_count %d -> %d (增 %d)"
              % (a["speed_min"], a["speed_max"], a["count_first"], a["count_last"], a["count_inc"]))
        print("  mode 分布 %s" % a["mode_hist"])

        out_dir = args.out_dir or (os.path.dirname(r.path) or ".")
        os.makedirs(out_dir, exist_ok=True)
        stem = os.path.splitext(os.path.basename(r.path))[0]
        owner = os.path.basename(os.path.dirname(r.path)) if os.path.dirname(r.path) else stem
        out_png = os.path.join(out_dir, "%s_curves.png" % owner)
        if r.plot(a, out_png):
            print("  图 -> %s" % out_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
