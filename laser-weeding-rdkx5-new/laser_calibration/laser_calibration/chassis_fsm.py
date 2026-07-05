#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
chassis_fsm.py —— 车控状态机(纯逻辑,无 ROS 依赖)  v0.1
=====================================================
"走—停—清—走"链路的决策核心,被 chassis_controller(ROS 壳)驱动:

  STANDBY ──/chassis/start──▶ CRUISE ──侧相机连续N帧见weed──▶ BRAKE
     ▲                          ▲                              │ 等0.5s车身稳
     │/chassis/stop             │盲走期满                      ▼
     └──────(任意状态)──────  BLIND_ROLL ◀──patch_clear── CLEARING
                                (先归中云台,                    │清场超时
                                 盲走甩开刚清的片)              ▼
                                                              FAULT(停车,需 /chassis/start 复位)

设计要点:
  · 防抖:连续 STOP_VOTE_FRAMES 个"新帧含 weed"才停车(同 planner 投票思想),
    单帧误检/闪烁不触发;
  · 盲走锁定:清完起步后先盲走 BLIND_ROLL_SEC 不响应检测 —— 否则打失败被
    planner 放弃的残留草会让车原地无限循环停车;
  · 盲走开始即请求云台归中(车在动,归中在途中完成),保证巡航期的侧视检测
    始终来自中心参考位,下一次停车-投票的坐标系干净;
  · 清场超时(CLEAR_TIMEOUT_SEC)→ FAULT:planner/执行层若挂死,车不能带着
    可能亮着的激光继续跑,停下等人;
  · 安全:safety_stop 任意状态立即 FAULT;tick 每拍都返回速度指令,
    由壳层以固定频率持续发布(看门狗友好)。
"""

# ── 可调参数(现场调这里)─────────────────────────────────────────
CRUISE_SPEED_MPS  = 0.10   # 巡航速度 m/s(键盘默认 0.2 的一半,作业速度宁慢勿快)
STOP_VOTE_FRAMES  = 15      # 连续 N 个新帧检出 weed 才停车(防抖)
BRAKE_SETTLE_SEC  = 0.5    # 刹停后等车身稳定再触发清场
CLEAR_TIMEOUT_SEC = 90.0   # 清场等待上限;超时 = 下游挂死 → FAULT 停车
BLIND_ROLL_SEC    = 4.0    # 清完后盲走时长(0.1m/s × 4s = 40cm,把刚清的片甩出侧视野)

# ── 状态常量 ─────────────────────────────────────────────────────
ST_STANDBY    = "STANDBY"     # 上电默认:静止待命,等 /chassis/start
ST_CRUISE     = "CRUISE"      # 低速直行 + 侧视检测
ST_BRAKE      = "BRAKE"       # 刹停沉降
ST_CLEARING   = "CLEARING"    # 已发 start_clearing,等 patch_clear
ST_BLIND_ROLL = "BLIND_ROLL"  # 盲走锁定期(不响应检测)
ST_FAULT      = "FAULT"       # 故障/急停:静止,需 start 复位

# tick() 返回的事件(由 ROS 壳翻译成话题发布)
EV_START_CLEARING = "publish_start_clearing"
EV_RECENTER       = "publish_recenter"


class ChassisFSM:
    def __init__(self, log=print):
        self.state = ST_STANDBY
        self._log = log
        self._t_enter = 0.0          # 进入当前状态的时刻
        self._weed_streak = 0        # 连续含 weed 的新帧数(仅 CRUISE 累计)
        self._start_sent = False     # CLEARING 的 start_clearing 是否已发

    # ── 外部事件 ────────────────────────────────────────────────
    def on_start(self, now):
        """/chassis/start:STANDBY/FAULT → CRUISE(其余状态忽略)。"""
        if self.state in (ST_STANDBY, ST_FAULT):
            self._to(ST_CRUISE, now, "收到 start,开始巡航")

    def on_stop(self, now):
        """/chassis/stop:任意状态 → STANDBY(正常收工)。"""
        self._to(ST_STANDBY, now, "收到 stop,停车待命")

    def on_safety_stop(self, now):
        """/safety_stop:任意状态 → FAULT(急停,需 start 复位)。"""
        self._to(ST_FAULT, now, "⚠️ 收到 safety_stop,急停")

    def on_yolo_frame(self, has_weed, now):
        """每个【新】YOLO 帧调用一次(壳层负责 frame_id 去重与置信度过滤)。"""
        if self.state != ST_CRUISE:
            return
        self._weed_streak = self._weed_streak + 1 if has_weed else 0
        if self._weed_streak >= STOP_VOTE_FRAMES:
            self._to(ST_BRAKE, now,
                     f"连续 {self._weed_streak} 帧检出 weed → 刹停")

    def on_patch_clear(self, now):
        if self.state == ST_CLEARING:
            self._to(ST_BLIND_ROLL, now,
                     f"收到 patch_clear → 归中云台,盲走 {BLIND_ROLL_SEC:.0f}s")

    # ── 主循环:固定频率调用,返回 (前进速度 m/s, [事件...]) ────────
    def tick(self, now):
        ev = []
        dt = now - self._t_enter
        if self.state == ST_CRUISE:
            return CRUISE_SPEED_MPS, ev
        if self.state == ST_BRAKE:
            if dt >= BRAKE_SETTLE_SEC:
                self._to(ST_CLEARING, now, "车身已稳 → 触发清场")
                ev.append(EV_START_CLEARING)
            return 0.0, ev
        if self.state == ST_CLEARING:
            if dt > CLEAR_TIMEOUT_SEC:
                self._to(ST_FAULT, now,
                         f"⚠️ 等待 patch_clear 超过 {CLEAR_TIMEOUT_SEC:.0f}s,"
                         f"下游可能挂死 → 故障停车(排查后发 /chassis/start 复位)")
            return 0.0, ev
        if self.state == ST_BLIND_ROLL:
            if dt == 0.0:
                pass
            if dt >= BLIND_ROLL_SEC:
                self._to(ST_CRUISE, now, "盲走结束,恢复巡航")
            return CRUISE_SPEED_MPS, ev
        return 0.0, ev               # STANDBY / FAULT:静止

    # ── 内部 ────────────────────────────────────────────────────
    def _to(self, state, now, why):
        old = self.state
        self.state = state
        self._t_enter = now
        self._weed_streak = 0
        self._log(f"[CHASSIS] {old} → {state}:{why}")
        # 进入盲走的瞬间请求云台归中(车在动,归中在途中完成)
        if state == ST_BLIND_ROLL:
            self._pending_recenter = True

    def pop_recenter(self):
        """壳层在状态切换后查询:本次是否需要发归中。"""
        if getattr(self, "_pending_recenter", False):
            self._pending_recenter = False
            return True
        return False
