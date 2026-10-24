# -*- coding: utf-8 -*-
"""打完整源码包：整棵工程树，包内根目录统一为 fp_aim/（车机解压即得 ~/fp_aim）。

用法（笔记本，工程根）：
    py tools\\onsite\\make_full_pack.py
    py tools\\onsite\\make_full_pack.py --out D:\\DSH

不含 build/ logs/ .git/ __pycache__/ 与任何 *.bak；logs 由车机自己留。
"""
import argparse
import hashlib
import os
import tarfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SKIP_DIRS = {"build", "logs", ".git", "__pycache__", ".vscode", ".cache"}
SKIP_SUFFIX = (".bak", ".bak_aim", ".pyc")
SKIP_NAMES = {".DS_Store", "Thumbs.db"}


def skip_file(name):
    return name in SKIP_NAMES or name.endswith(SKIP_SUFFIX) or ".bak." in name or ".bak_" in name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=r"D:\DSH", help="输出目录（默认 D:\\DSH）")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    out = os.path.join(args.out, "fp_aim_full_%s.tar.gz" % time.strftime("%m%d_%H%M"))

    n = 0
    with tarfile.open(out, "w:gz", compresslevel=6) as tf:
        for dirpath, dirnames, filenames in os.walk(ROOT):
            dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
            rel = os.path.relpath(dirpath, ROOT)
            rel = "" if rel == "." else rel
            if rel:
                tf.addfile(tf.gettarinfo(dirpath, arcname="fp_aim/" + rel))
            for f in sorted(filenames):
                if skip_file(f):
                    continue
                arc = "fp_aim/" + (rel + "/" if rel else "") + f
                ti = tf.gettarinfo(os.path.join(dirpath, f), arcname=arc)
                # Windows 源文件没有可执行位 ⇒ 按扩展名在包里补上，
                # 否则 Linux 解压后 run_measure.sh 会 Permission denied
                ti.mode = 0o755 if f.endswith((".sh", ".py")) else 0o644
                ti.uid = ti.gid = 0
                ti.uname = ti.gname = "rm"
                with open(os.path.join(dirpath, f), "rb") as fh:
                    tf.addfile(ti, fh)
                n += 1

    size = os.path.getsize(out)
    h = hashlib.sha256()
    with open(out, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)

    print("out   :", out)
    print("files :", n)
    print("size  : %d B (%.1f MB)" % (size, size / 1048576.0))
    print("sha256:", h.hexdigest().upper())


if __name__ == "__main__":
    main()
