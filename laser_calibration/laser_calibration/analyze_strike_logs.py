#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_strike_logs.py —— 打击日志离线分析  v3.12.0 新增
========================================================
读 vision_servo v3.12.0 起落盘的 ~/strike_logs/strike_*.jsonl,
输出逐场与汇总统计(答辩/报告用),可选导出 CSV。

用法:
    ros2 run laser_calibration analyze_strike_logs             # 全部日志
    ros2 run laser_calibration analyze_strike_logs -- --last   # 只看最新一场
    ros2 run laser_calibration analyze_strike_logs -- --csv out.csv
    python3 analyze_strike_logs.py 某个.jsonl 另一个.jsonl      # 也可直接跑

每行记录字段(由 vision_servo._log_shot 写入):
    n, t, id, result(success/failed), x, y, final_distance,
    hit(true/false/null), hit_dist, hit_frames, duration
"""

import argparse
import glob
import json
import os
import sys


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def _fmt(v, suffix="", nd=2):
    return "--" if v is None else f"{round(v, nd)}{suffix}"


def load_file(path):
    shots = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                shots.append(json.loads(ln))
            except json.JSONDecodeError:
                pass
    return shots


def stats(shots):
    succ = [r for r in shots if r.get("result") == "success"]
    hit = [r for r in succ if r.get("hit") is True]
    miss = [r for r in succ if r.get("hit") is False]
    judged = len(hit) + len(miss)
    return {
        "shots": len(shots), "success": len(succ),
        "failed": len(shots) - len(succ),
        "success_rate": len(succ) / len(shots) if shots else None,
        "hit": len(hit), "miss": len(miss),
        "unjudged": len(succ) - judged,
        "hit_rate": len(hit) / judged if judged else None,
        "avg_lock_err": _mean([r.get("final_distance") for r in succ]),
        "avg_hit_dist": _mean([r.get("hit_dist") for r in hit + miss]),
        "avg_duration": _mean([r.get("duration") for r in succ]),
        "p90_duration": (sorted(
            [r["duration"] for r in succ if r.get("duration") is not None]
        )[max(0, int(0.9 * len(succ)) - 1)]
            if [r for r in succ if r.get("duration") is not None] else None),
    }


def print_block(title, st):
    print(f"\n── {title}")
    print(f"   发数 {st['shots']}  成功 {st['success']}  失败 {st['failed']}"
          f"  成功率 {_fmt(None if st['success_rate'] is None else st['success_rate'] * 100, '%', 1)}")
    print(f"   判定 命中 {st['hit']} / 脱靶 {st['miss']} / 未判定 {st['unjudged']}"
          f"   命中率 {_fmt(None if st['hit_rate'] is None else st['hit_rate'] * 100, '%', 1)}")
    print(f"   平均锁定误差 {_fmt(st['avg_lock_err'], 'px')}"
          f"   平均命中偏差 {_fmt(st['avg_hit_dist'], 'px')}")
    print(f"   平均单株耗时 {_fmt(st['avg_duration'], 's')}"
          f"   P90 耗时 {_fmt(st['p90_duration'], 's')}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="打击日志离线分析")
    ap.add_argument("files", nargs="*", help="jsonl 文件;缺省扫 ~/strike_logs")
    ap.add_argument("--dir", default="~/strike_logs")
    ap.add_argument("--last", action="store_true", help="只分析最新一场")
    ap.add_argument("--csv", default=None, help="把全部记录导出为 CSV")
    args = ap.parse_args(argv)

    files = args.files or sorted(
        glob.glob(os.path.join(os.path.expanduser(args.dir), "strike_*.jsonl")))
    if args.last and files:
        files = files[-1:]
    if not files:
        print(f"没找到日志。先跑几发打击(vision_servo v3.12.0 起自动落盘"
              f" {args.dir}),或用参数指定 jsonl 文件。")
        return 1

    all_shots = []
    for path in files:
        shots = load_file(path)
        all_shots += shots
        print_block(os.path.basename(path) + f"  ({len(shots)} 发)",
                    stats(shots))
    if len(files) > 1:
        print_block(f"汇总({len(files)} 场)", stats(all_shots))

    if args.csv:
        import csv
        cols = ["n", "t", "id", "result", "x", "y", "final_distance",
                "hit", "hit_dist", "hit_frames", "duration"]
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in all_shots:
                w.writerow(r)
        print(f"\nCSV 已写 {args.csv}({len(all_shots)} 行)")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
