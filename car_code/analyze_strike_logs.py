#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_strike_logs.py —— 从 vision_servo / strike_planner 日志提取打击指标
配套 laser_calibration v3.10.11(也兼容 v3.10.9/10 的旧日志,缺字段自动跳过)

用法:
    python3 analyze_strike_logs.py ~/log_servo.txt ~/log_planner.txt
    python3 analyze_strike_logs.py ~/log_servo.txt              # 只有执行层日志也能跑

输出:
    每发打击明细表(id/目标/重捕获距离/锁定精度/耗时/结果)
    汇总指标(成功率、平均单株耗时、锁定精度均值/最差、重捕获距离分布、门限拒收次数)
    每片(session)清场统计
纯标准库,无依赖,可直接在小车或电脑上跑。
"""
import re
import sys

TS = re.compile(r"\[(\d+)\.(\d+)\]")                       # ROS 日志时间戳 [epoch.nsec]

R_STRIKE_RX = re.compile(r"\[STRIKE\] 收到 planner 指令: id=(\d+) 目标\((\d+),(\d+)\)")
R_RESULT    = re.compile(r"\[STRIKE\] 回报结果: id=(\d+) (\w+) d=([\d.]+|--)")
R_REACQ     = re.compile(r"\[REACQ\] 锚点重捕获: 预测\((\d+),(\d+)\) → 选框\((\d+),(\d+)\) d=(\d+)px")
R_LOCK      = re.compile(r"PID 锁定: d=([\d.]+)px")
R_LOCK_BEST = re.compile(r"锁定于最佳点 .*\(d≈([\d.]+)px\)")
R_GATE      = re.compile(r"本帧不更新靶点")
R_COARSE    = re.compile(r"\[COARSE-绝对\] required_spot\((\d+),(\d+)\)")
R_DISPATCH  = re.compile(r"下发打击 id=(\d+) 目标\((\d+),(\d+)\)")
R_SESSION   = re.compile(r"本片清完[：:]\s*成功 (\d+) / 失败 (\d+) / 共 (\d+)")
R_QUEUE     = re.compile(r"开始清场[：:]本片 (\d+) 个目标")


def ts_of(line):
    m = TS.search(line)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) / 1e9


def parse_servo(path, attempts, gate_warns):
    """attempts: list[dict] —— 每次"收到指令"开一条尝试记录,重试不去重
    (同 id 失败重试是两条记录,否则失败被吞、指标虚高)。"""
    cur = None
    for line in open(path, encoding="utf-8", errors="replace"):
        t = ts_of(line)
        m = R_STRIKE_RX.search(line)
        if m:
            cur = dict(sid=int(m.group(1)),
                       tgt=(int(m.group(2)), int(m.group(3))),
                       t0=t, reacq=[])
            attempts.append(cur)
            continue
        m = R_REACQ.search(line)
        if m and cur is not None:
            cur["reacq"].append(int(m.group(5)))
            continue
        m = R_LOCK.search(line) or R_LOCK_BEST.search(line)
        if m and cur is not None:
            cur["lock_d"] = float(m.group(1))
            continue
        if R_GATE.search(line):
            gate_warns.append(t)
            continue
        m = R_RESULT.search(line)
        if m and cur is not None and int(m.group(1)) == cur["sid"]:
            cur["result"] = m.group(2)
            cur["t1"] = t
            if m.group(3) != "--":
                cur.setdefault("lock_d", float(m.group(3)))
            cur = None


def parse_planner(path, sessions):
    queued = None
    for line in open(path, encoding="utf-8", errors="replace"):
        m = R_QUEUE.search(line)
        if m:
            queued = int(m.group(1))
            continue
        m = R_SESSION.search(line)
        if m:
            sessions.append(dict(ok=int(m.group(1)), fail=int(m.group(2)),
                                 total=int(m.group(3)), queued=queued))
            queued = None


def fmt(v, n=1):
    return "--" if v is None else f"{v:.{n}f}"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    attempts, gate_warns, sessions = [], [], []
    for p in sys.argv[1:]:
        head = open(p, encoding="utf-8", errors="replace").read(4000)
        if "strike_planner" in head or "PLANNER" in head:
            parse_planner(p, sessions)
        else:
            parse_servo(p, attempts, gate_warns)

    if not attempts:
        print("没有解析到任何打击记录——确认传入的是 vision_servo 的日志文件。")
        sys.exit(0)

    print("═" * 78)
    print("每发打击明细")
    print("─" * 78)
    print(f"{'id':>4} {'目标(中心参考)':<16} {'重捕获d(px)':<12} "
          f"{'锁定精度(px)':<12} {'耗时(s)':<8} 结果")
    durs, locks, reacqs = [], [], []
    n_ok = n_fail = n_rej = 0
    ids_ok, ids_all = set(), set()
    for s in attempts:
        sid, tgt = s["sid"], s.get("tgt")
        dur = (s["t1"] - s["t0"]) if s.get("t0") and s.get("t1") else None
        res = s.get("result", "?")
        rq = s.get("reacq", [])
        ids_all.add(sid)
        if res == "success":
            n_ok += 1
            ids_ok.add(sid)
            if dur:
                durs.append(dur)
            if s.get("lock_d") is not None:
                locks.append(s["lock_d"])
            reacqs += rq
        elif res == "rejected":
            n_rej += 1
        elif res != "?":
            n_fail += 1
        print(f"{sid:>4} {str(tgt):<16} "
              f"{('/'.join(map(str, rq)) or '--'):<12} "
              f"{fmt(s.get('lock_d')):<12} {fmt(dur):<8} {res}")

    n_att = n_ok + n_fail
    print("═" * 78)
    print("汇总指标(rejected 不计入尝试)")
    print("─" * 78)
    print(f"  打击尝试 {n_att} 发:成功 {n_ok} / 失败 {n_fail} / 被拒 {n_rej}")
    if n_att:
        print(f"  一次命中率(按尝试) = {100.0 * n_ok / n_att:.1f}%"
              f"   最终清除率(按目标,含重试) = "
              f"{100.0 * len(ids_ok) / len(ids_all):.1f}%"
              f" ({len(ids_ok)}/{len(ids_all)} 株)")
    if durs:
        print(f"  单株耗时:均值 {sum(durs)/len(durs):.1f}s"
              f"  最快 {min(durs):.1f}s  最慢 {max(durs):.1f}s")
    if locks:
        print(f"  锁定精度:均值 {sum(locks)/len(locks):.1f}px"
              f"  最差 {max(locks):.1f}px(≈舵机 1° 量化地板 ~5px 为理论极限)")
    if reacqs:
        print(f"  重捕获距离:均值 {sum(reacqs)/len(reacqs):.1f}px"
              f"  最大 {max(reacqs)}px(应 < 门限 50px;均值越小盲跳越准)")
    print(f"  选框门限拒收 {len(gate_warns)} 次(偶发=目标闪烁被正确拦截;"
          f"频繁=门限偏紧或检测不稳)")
    if sessions:
        print("─" * 78)
        print("每片清场(planner)")
        for i, s in enumerate(sessions, 1):
            print(f"  第{i}片:建队 {s['queued'] if s['queued'] is not None else '?'} 个"
                  f" → 成功 {s['ok']} / 失败 {s['fail']} / 共 {s['total']}")
        tot = sum(s["total"] for s in sessions)
        ok = sum(s["ok"] for s in sessions)
        print(f"  合计 {len(sessions)} 片 {tot} 株,清除率 "
              f"{100.0 * ok / tot if tot else 0:.1f}%")
    print("═" * 78)


if __name__ == "__main__":
    main()
