#!/usr/bin/env python3
"""
strike_planner.py —— Phase 3：多目标打击决策层  v0.6
=====================================================
v0.6（相对 v0.5，配合 vision_servo v3.11.2；决策逻辑零改动，只加"广播+记账"）：
  ★ 新增 /planner/session_state 会话状态广播(5Hz)：state/total/pending/current/
    struck/failed(+投票期的帧数与簇数)。vision_servo 网页订阅后把整个决策过程
    画出来 —— 视频/答辩里评委能直接看到"投票建队→贪心排序→逐个清除→不重打"。
  ★ 聚合执行层的蓝斑命中判定：strike_result 的 hit/hit_distance 附加字段记到
    struck 目标上,patch_clear 与收尾日志输出"判定命中 X / 脱靶 Y / 未检出 Z"。
    字段缺失(旧版执行层)时静默降级,完全向后兼容。

v0.5（相对 v0.4，配合 vision_servo v3.10.11）：
  ★ strike_cmd 新增 "exclude" 字段：本片已打目标的中心参考坐标。执行层
    重捕获/跟踪选框时排除其邻域 → 杜绝"打二又打回一"式重复打击。

v0.4（相对 v0.3，配合 vision_servo v3.10.10）：
  ★ **建队前归中（P1）**：执行层不归中时，上一片打完云台停在歪角；触发清场
    先发 /servo/recenter 请求归中，等 RECENTER_WAIT_SEC 行程走完，且只采
    "归中完成之后"的 YOLO 帧建队——保证投票坐标确实在中心参考系。
  ★ 失败/超时时 _gimbal_ref 同步到失败目标（P4）：盲跳已执行、云台实际停在
    该目标附近，贪心参考跟着走，排序不再脱节。

v0.3（相对 v0.2）：
  ★ 多目标策略升级：**时序投票建队**（解决 YOLO 无时间戳问题）
    旧版用"触发瞬间抓一帧"建队 —— 单帧里的误检/漏检会直接传导。
    新版：车停稳触发后开一个 VOTE_WINDOW_SEC（默认 0.6s）的墙钟窗口，
    把窗口内每帧 YOLO 的 weed 检测**按空间位置聚类**，只有"在窗口内出现
    帧数 / 总帧数 ≥ VOTE_MIN_RATIO"的簇才确认成目标，位置取均值。
      - 不需要任何时间戳：投票窗口内场景静止 → 投票纯靠空间聚类 + 出现频次。
      - ⚠️ 相机 + 激光同在云台：投票期间**云台必须停在参考位（居中）不动**，
        画面才静止、同一株草每帧才在同一像素。本节点不直接控云台，靠的是
        触发投票时 vision_servo 处于 IDLE（云台居中）、且其触发模式设 manual
        （不会自己乱转）。投票确认建好队列后再逐个下发打击。
      - 效果：滤掉单帧闪烁误检、容忍偶发漏检、目标坐标更稳。
    新增状态 ST_VOTING（IDLE → VOTING → CLEARING → WAIT_RESULT）。
  ★ gimbal_ref 与执行层归中保持一致（EXECUTOR_RECENTERS）：
    vision_servo 若每发归中（RECENTER_AFTER_FIRE=True），云台实际停在中心，
    故贪心的参考点也应是中心；若执行层不归中，参考点才是上一株草。
  ★ STRIKE_TIMEOUT 25s → 12s（正常一发最坏 ~7.4s，留 ~1.6x 裕度）。

v0.2：已对齐 yolo_detector.py 的真实输出（box 字段 cx/cy/w/h/confidence/label，
      payload 含 detected/boxes/frame_id），并加入 weed/crop 安全过滤。
"感知—决策—执行"三节点架构里的【决策】节点。

    yolo_detect (感知)  ──/yolo/...──▶  strike_planner (决策，本文件)
                                              │
                          /servo/strike_cmd   │  逐个下发目标
                                              ▼
                                       vision_servo (执行)
                                              │
                          /servo/strike_result│  回报每次打击结果
                                              ▼
                                       strike_planner

职责：把 YOLO 一帧里的多个杂草框，规划成一个有序打击队列，逐个交给
vision_servo 打，全部打完后对外发出"本片清完"信号。vision_servo 只懂
打单个目标，本节点负责"打哪些、按什么顺序、打完了没"。

工作流（配合"走—停—清场—再走"）：
    车控节点把车停稳 → 向 /planner/start_clearing 发一条消息
      → 本节点建队 → 逐个 strike_cmd → 收齐 strike_result
      → 向 /planner/patch_clear 发"清完"→ 车控节点驱车前进
每收到一条 start_clearing 只清场一次，避免对着烧过的草反复打。

决策逻辑：
  · 过滤  —— 低置信度 / 过大过小 / 贴画面边缘的框丢弃
  · 合并  —— 同株草被检测成多个框时合并成一个
  · 排序  —— 贪心：每次取离云台当前指向最近的目标（少转云台、清场快）
  · 容错  —— 打击失败重排队尾重试一次；执行层忙则退避重发；超时判失败

接线（在 laser_calibration 包里）：
  1) setup.py 的 entry_points 加：
       'strike_planner = laser_calibration.strike_planner:main'
  2) 运行：   ros2 run laser_calibration strike_planner
  3) vision_servo 触发模式设为 manual（否则 auto 会和本节点抢着打）
  4) 单机测试（无车控节点时）手动触发一次清场：
       ros2 topic pub --once /planner/start_clearing std_msgs/msg/Empty {}

⚠️ 安全：YOLO 模型有两类 ["weed","crop"]。本节点只把 label=="weed" 的框
   排进打击队列，crop 一律丢弃——绝不能用激光烧作物。见 TARGET_LABELS。
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

# YOLO 话题名直接从 config 取，保证与 vision_servo 完全一致
from laser_calibration.config import TOPIC_YOLO

# ── 话题 ────────────────────────────────────────────────────────
# ⚠️ 前两个必须与 vision_servo.py 的 TOPIC_STRIKE_CMD / TOPIC_STRIKE_RESULT 一致
TOPIC_STRIKE_CMD     = "/servo/strike_cmd"        # 本节点 → vision_servo
TOPIC_STRIKE_RESULT  = "/servo/strike_result"     # vision_servo → 本节点
TOPIC_PATCH_CLEAR    = "/planner/patch_clear"     # 本节点 → 车控节点："本片清完"
TOPIC_START_CLEARING = "/planner/start_clearing"  # 车控/人工 → 本节点："开始清场"
# v0.6: 会话状态广播 —— vision_servo 网页订阅做决策可视化(纯显示)。
#   ⚠️ 与 vision_servo.py 的 TOPIC_PLANNER_SESSION 字符串必须一致。
TOPIC_SESSION_STATE  = "/planner/session_state"   # 本节点 → vision_servo(显示)

# ── 画面尺寸（与 vision_servo 一致）──────────────────────────────
IMG_W, IMG_H = 640, 480

# ── 目标过滤 ────────────────────────────────────────────────────
# ⚠️ 安全关键：YOLO 模型类别为 ["weed","crop"]，只打 weed。
#    box 缺 label 时也按"非 weed"丢弃（fail-safe，不烧不确定的东西）。
TARGET_LABELS = {"weed"}

MIN_CONF      = 0.50      # YOLO 置信度下限（对应 box 的 "confidence" 字段）
MIN_BOX_AREA  = 80        # px²，过小的框当误检丢弃（YOLO 不输出 w/h 时此过滤失效）
MAX_BOX_AREA  = 120000    # px²，过大的框（占半屏）当误检丢弃
EDGE_MARGIN   = 18        # px，中心距画面边缘小于此 → 丢弃（部分出画 / 云台难瞄）
DEDUP_RADIUS  = 28        # px，两框中心距小于此 → 视为同株草，合并

# ── v0.3 时序投票（多帧累积，无需时间戳）──────────────────────
# 车停稳后开一个墙钟窗口，按空间聚类累积多帧 YOLO 检测，按出现频次确认目标。
VOTE_WINDOW_SEC = 0.6     # s，投票窗口时长（车不动，0.5~0.8s 足够攒到几帧）
VOTE_MIN_RATIO  = 0.5     # 簇出现帧数 / 窗口总帧数 ≥ 此值才确认（滤闪烁、容漏检）
                          #   设 0 则不投票（任意单帧出现即确认，退化为旧行为）

# ── v0.3→v0.4 执行层归中一致性 ──────────────────────────────────
# ⚠️ 必须与 vision_servo.py 的 RECENTER_AFTER_FIRE 保持一致！
# v3.10.9 起执行层改用【绝对角】方案：本队列仍在"参考位（云台居中）"投票建立、
# 坐标为中心参考像素；执行层收到后换算成云台绝对角盲跳，与当前姿态无关，
# 故【不再需要每发归中】，打完一发留在原地、直接斜跳下一株。
#   True  = 执行层每发归中（回退到旧行为）→ 贪心参考点 = 画面中心。
#   False = 执行层不归中（v3.10.9 默认）→ 贪心参考点 = 上一株位置（离上株最近优先，
#           转动幅度最小、链式打击更顺）。
EXECUTOR_RECENTERS = False

# ── v0.4 建队前归中（P1）────────────────────────────────────────
# 投票建队必须在"参考位（云台居中）"进行 —— 队列坐标全按中心参考系记账，
# 执行层的绝对角公式也按中心参考系解读。执行层不归中时，上一片打完云台停在
# 最后一株的歪角；若直接对歪角画面投票，整片队列坐标系全错（第二片必偏）。
# 故触发清场时先向执行层请求归中，等行程走完后【只采归中之后的帧】建队。
# （EXECUTOR_RECENTERS=True 时执行层每发已归中，跳过此步，保持 v3.10.8 行为。）
TOPIC_SERVO_RECENTER = "/servo/recenter"   # 本节点 → vision_servo：归中请求
RECENTER_WAIT_SEC    = 0.8     # s，归中行程 ~0.4s + 画面/检测稳定余量

# ── 调度与容错 ──────────────────────────────────────────────────
STRIKE_TIMEOUT  = 12.0    # s，下发后多久没收到结果就当失败（v0.3: 25→12，
                          #   正常一发最坏 ~7.4s = PID超时5 + 烧1 + 冷却1 + 归中0.4）
MAX_RETRY       = 1       # 单个目标失败后的重试次数
REJECT_BACKOFF  = 1.5     # s，被执行层拒绝（忙）后的重发等待
NO_YOLO_TIMEOUT = 5.0     # s，触发清场后多久收不到 YOLO 就放弃本次

# ── 节奏 ────────────────────────────────────────────────────────
PLANNER_TICK   = 0.20     # s，决策 FSM 周期（5Hz）
YOLO_FRESH_SEC = 1.0      # s，YOLO 帧新鲜度上限

# ── 决策层状态机 ────────────────────────────────────────────────
ST_IDLE     = "IDLE"        # 等待 start_clearing 触发
ST_VOTING   = "VOTING"      # v0.3: 投票窗口内，累积多帧 YOLO 检测
ST_CLEARING = "CLEARING"    # 队列非空，准备下发下一个目标
ST_WAITING  = "WAIT_RESULT" # 已下发，等 vision_servo 回报


class StrikePlanner(Node):

    def __init__(self):
        super().__init__("strike_planner")

        # ── 订阅 ──
        self.sub_yolo = self.create_subscription(
            String, TOPIC_YOLO, self._cb_yolo, 10)
        self.sub_result = self.create_subscription(
            String, TOPIC_STRIKE_RESULT, self._cb_strike_result, 10)
        self.sub_start = self.create_subscription(
            Empty, TOPIC_START_CLEARING, self._cb_start_clearing, 10)

        # ── 发布 ──
        self.pub_cmd = self.create_publisher(String, TOPIC_STRIKE_CMD, 10)
        self.pub_patch_clear = self.create_publisher(
            String, TOPIC_PATCH_CLEAR, 10)
        # v0.4 (P1): 建队前请求执行层归中
        self.pub_recenter = self.create_publisher(
            Empty, TOPIC_SERVO_RECENTER, 10)
        # v0.6: 会话状态广播(决策可视化,纯显示)
        self.pub_session = self.create_publisher(
            String, TOPIC_SESSION_STATE, 10)
        self._last_result_extra = None   # v0.6: 最近一条 strike_result 的命中附加字段

        # ── YOLO 最新帧 ──
        self._latest_boxes = None     # list[dict] | None
        self._boxes_at = 0.0
        self._latest_frame_id = None  # YOLO payload 里的帧计数（无时间戳，但可标识帧）

        # ── 决策状态 ──
        self.state = ST_IDLE
        self._start_requested = False
        self._request_at = 0.0
        self._recenter_done_at = 0.0  # v0.4 (P1): 此刻前的 YOLO 帧不可用于建队
        self._next_id = 1             # 全局递增目标 id

        # ── 本次清场会话 ──
        self._queue = []              # 待打目标 list[dict{id,x,y,conf,_retry}]
        self._current = None          # 当前正在打的目标
        self._struck = []             # 本会话成功的目标
        self._failed = []             # 本会话放弃的目标
        self._session_total = 0
        self._gimbal_ref = (IMG_W / 2.0, IMG_H / 2.0)  # 云台当前指向（近似）
        self._dispatch_at = 0.0
        self._dispatch_block_until = 0.0   # 退避：此刻前不下发

        # ── v0.3 时序投票状态 ──
        self._vote_clusters = []      # [{sum_x,sum_y,sum_conf,count}]，跨帧累积
        self._vote_frames = 0         # 窗口内已累积的 YOLO 帧数（去重后）
        self._vote_until = 0.0        # 投票窗口结束墙钟时刻
        self._vote_last_frame_id = None  # 已计入投票的最后一个 frame_id（去重用）

        self.timer = self.create_timer(PLANNER_TICK, self._tick)

        log = self.get_logger().info
        log("═══════════════════════════════════════════════")
        log("  strike_planner 决策层  v0.6")
        log("═══════════════════════════════════════════════")
        log(f"  订阅 YOLO:      {TOPIC_YOLO}")
        log(f"  ← 触发清场:     {TOPIC_START_CLEARING}")
        log(f"  → 下发打击:     {TOPIC_STRIKE_CMD}")
        log(f"  ← 打击结果:     {TOPIC_STRIKE_RESULT}")
        log(f"  → 本片清完:     {TOPIC_PATCH_CLEAR}")
        log(f"  → 会话广播:     {TOPIC_SESSION_STATE}（v0.6 决策可视化）")
        log("  ─────────────────────────────────────")
        if VOTE_MIN_RATIO > 0:
            log(f"  建队策略:       时序投票（窗口 {VOTE_WINDOW_SEC:.1f}s，"
                f"频次≥{VOTE_MIN_RATIO:.0%} 确认）")
        else:
            log(f"  建队策略:       单帧（投票关闭）")
        log(f"  排序策略:       贪心最近优先"
            f"（执行层{'归中→参考中心' if EXECUTOR_RECENTERS else '不归中→参考上株'}）")
        log(f"  过滤:           conf≥{MIN_CONF}  边距≥{EDGE_MARGIN}px  "
            f"合并半径{DEDUP_RADIUS}px")
        log(f"  容错:           失败重试{MAX_RETRY}次  超时{STRIKE_TIMEOUT:.0f}s")
        log("  等待 start_clearing 触发……")

    # ────────────────────────────────────────────────────────────
    #  回调
    # ────────────────────────────────────────────────────────────
    def _cb_yolo(self, msg):
        """缓存 YOLO 最新一帧的所有框。"""
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f"YOLO 解析失败：{e}")
            return
        if not d.get("detected", False):
            self._latest_boxes = []
            self._boxes_at = time.time()
            return
        boxes = d.get("boxes")
        if not boxes:
            # 单框兜底格式 {detected, cx, cy}
            cx, cy = d.get("cx"), d.get("cy")
            boxes = [{"cx": cx, "cy": cy}] if cx is not None else []
        self._latest_boxes = boxes
        self._latest_frame_id = d.get("frame_id")
        self._boxes_at = time.time()

        # v0.3: 投票窗口内，把本帧并入投票簇（_cb_yolo 与 _tick 同属互斥回调组，安全）。
        #   按 frame_id 去重：YOLO 以 10Hz 定时发布，推理慢时同一帧会被重复发布，
        #   不去重会把同一帧多算，扭曲投票频次。
        if self.state == ST_VOTING:
            fid = self._latest_frame_id
            if fid is None or fid != self._vote_last_frame_id:
                self._vote_add_frame(boxes)
                self._vote_last_frame_id = fid

    def _cb_start_clearing(self, _msg):
        """收到"开始清场"触发。仅在 IDLE 时受理。"""
        if self.state != ST_IDLE:
            self.get_logger().warn(
                f"[PLANNER] 收到 start_clearing，但当前 {self.state}，忽略")
            return
        self._start_requested = True
        self._request_at = time.time()
        if not EXECUTOR_RECENTERS:
            # v0.4 (P1): 建队前先把云台请回参考位（上一片打完云台停在歪角，
            #   歪角画面投出来的坐标整片都错）。归中后只采"归中完成之后"的帧。
            self.pub_recenter.publish(Empty())
            self._recenter_done_at = self._request_at + RECENTER_WAIT_SEC
            self.get_logger().info(
                f"[PLANNER] 收到开始清场触发 → 已请求云台归中，"
                f"{RECENTER_WAIT_SEC:.1f}s 后开始采帧建队")
        else:
            self._recenter_done_at = 0.0
            self.get_logger().info("[PLANNER] 收到开始清场触发")

    def _cb_strike_result(self, msg):
        """vision_servo 回报一次打击结果。"""
        try:
            d = json.loads(msg.data)
            rid = int(d["id"])
            result = str(d.get("result", "failed"))
            # v0.6: 执行层 v3.11.2 起附带蓝斑命中判定;旧版无此字段 → None,兼容
            self._last_result_extra = {
                "hit": d.get("hit"),                  # True/False/None
                "hit_dist": d.get("hit_distance"),
                "hit_frames": d.get("hit_frames"),
            }
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(f"strike_result 解析失败：{e}")
            return
        if self.state != ST_WAITING or self._current is None:
            self.get_logger().warn(
                f"[PLANNER] 收到结果 id={rid} 但当前未在等待，忽略")
            return
        if rid != self._current["id"]:
            self.get_logger().warn(
                f"[PLANNER] 结果 id={rid} 与当前目标 id={self._current['id']} "
                f"不符，忽略")
            return
        self._handle_outcome(result)

    # ────────────────────────────────────────────────────────────
    #  决策 FSM（5Hz）
    # ────────────────────────────────────────────────────────────
    def _tick(self):
        # v0.6: 会话快照放 finally —— 每拍【结束后】广播本拍转换完成的最新状态
        #   (放开头会让网页滞后一拍),任何 return 分支都覆盖,异常照常上抛。
        #   小 JSON @5Hz,开销可忽略;纯显示,丢了也不影响任何决策。
        try:
            self._tick_inner()
        finally:
            self._publish_session()

    def _tick_inner(self):
        now = time.time()

        # ── IDLE：等触发 → 建队 → 进 CLEARING ──
        if self.state == ST_IDLE:
            if not self._start_requested:
                return
            if now < self._recenter_done_at:
                return                      # v0.4 (P1): 等云台归中行程走完
            boxes = self._fresh_boxes()
            if boxes is None or self._boxes_at < self._recenter_done_at:
                # 没有新鲜帧，或最新帧仍是归中前（歪角姿态）拍的 → 继续等
                if now - self._request_at > NO_YOLO_TIMEOUT:
                    self.get_logger().warn(
                        "[PLANNER] 触发后收不到新鲜 YOLO 帧，放弃本次清场")
                    self._start_requested = False
                return
            self._start_requested = False

            # v0.3: VOTE_MIN_RATIO>0 → 开投票窗口；否则退化为单帧建队（旧行为）。
            if VOTE_MIN_RATIO > 0:
                self._vote_reset()
                self._vote_add_frame(boxes)          # 把触发瞬间这帧也算进去
                self._vote_until = now + VOTE_WINDOW_SEC
                self.get_logger().info(
                    f"[PLANNER] ▷ 进入投票窗口 {VOTE_WINDOW_SEC:.1f}s "
                    f"（累积多帧 YOLO，按位置聚类+频次确认）")
                self.state = ST_VOTING
                return

            queue = self._build_queue(boxes)
            if not queue:
                self.get_logger().info(
                    "[PLANNER] 视野内无有效目标，直接发布 patch_clear")
                self._session_total = 0
                self._struck, self._failed = [], []
                self._publish_patch_clear()
                return
            self._queue = queue
            self._struck, self._failed = [], []
            self._session_total = len(queue)
            self._gimbal_ref = (IMG_W / 2.0, IMG_H / 2.0)
            self.get_logger().info(
                f"[PLANNER] ▶ 开始清场：本片 {len(queue)} 个目标 "
                f"ids={[t['id'] for t in queue]}")
            self.state = ST_CLEARING
            # 落入下面 CLEARING 分支

        # ── VOTING：累积在 _cb_yolo 里进行，这里只等窗口结束并定稿 ──
        if self.state == ST_VOTING:
            if now < self._vote_until:
                return
            queue = self._finalize_vote()
            if not queue:
                self.get_logger().info(
                    "[PLANNER] 投票后无确认目标，直接发布 patch_clear")
                self._session_total = 0
                self._struck, self._failed = [], []
                self._publish_patch_clear()
                self.state = ST_IDLE
                return
            self._queue = queue
            self._struck, self._failed = [], []
            self._session_total = len(queue)
            self._gimbal_ref = (IMG_W / 2.0, IMG_H / 2.0)
            self.get_logger().info(
                f"[PLANNER] ▶ 开始清场：本片 {len(queue)} 个目标 "
                f"ids={[t['id'] for t in queue]}")
            self.state = ST_CLEARING
            # 落入下面 CLEARING 分支

        # ── CLEARING：队列空→清完；否则下发下一个 ──
        if self.state == ST_CLEARING:
            if now < self._dispatch_block_until:
                return
            if not self._queue:
                nh, nm, nu = self._hit_counts()
                self.get_logger().info(
                    f"[PLANNER] ■ 本片清完：成功 {len(self._struck)} / "
                    f"失败 {len(self._failed)} / 共 {self._session_total}"
                    f"  ｜命中判定: ✓{nh} ✗{nm} ?{nu}")
                self._publish_patch_clear()
                self.state = ST_IDLE
                return
            # 贪心：取离云台当前指向最近的目标
            tgt = min(self._queue, key=lambda t: self._dist(t, self._gimbal_ref))
            self._queue.remove(tgt)
            self._current = tgt
            self._dispatch(tgt)
            self.state = ST_WAITING
            return

        # ── WAIT_RESULT：结果由 _cb_strike_result 推进，这里只管超时 ──
        if self.state == ST_WAITING:
            if self._current is not None and \
               now - self._dispatch_at > STRIKE_TIMEOUT:
                self.get_logger().warn(
                    f"[PLANNER] 目标 id={self._current['id']} "
                    f"{STRIKE_TIMEOUT:.0f}s 无结果 → 当失败处理")
                self._handle_outcome("failed")
            return

    # ────────────────────────────────────────────────────────────
    #  打击结果处理
    # ────────────────────────────────────────────────────────────
    def _handle_outcome(self, result):
        cur = self._current
        if cur is None:
            return

        if result == "success":
            # v0.6: 把执行层的命中判定记到该目标上(缺失=None,不影响任何决策)
            extra = self._last_result_extra or {}
            cur["hit"] = extra.get("hit")
            cur["hit_dist"] = extra.get("hit_dist")
            self._last_result_extra = None
            _hit_txt = ("" if cur["hit"] is None else
                        ("  🎯判定命中" if cur["hit"] else "  ⚠️判定脱靶") +
                        (f" d={cur['hit_dist']}px" if cur["hit_dist"] is not None else ""))
            self.get_logger().info(f"[PLANNER] ✅ id={cur['id']} 打击成功{_hit_txt}")
            # v0.3: 与执行层归中保持一致。
            #   归中 → 云台实际回到画面中心，参考点用中心；
            #   不归中 → 云台停在该株草，参考点用该株草（贪心就近排序生效）。
            if EXECUTOR_RECENTERS:
                self._gimbal_ref = (IMG_W / 2.0, IMG_H / 2.0)
            else:
                self._gimbal_ref = (cur["x"], cur["y"])
            self._struck.append(cur)

        elif result == "rejected":
            # 执行层忙（如有人在用 UI 手动打）→ 退避后重发同一目标
            self.get_logger().warn(
                f"[PLANNER] id={cur['id']} 被执行层拒绝（忙），"
                f"{REJECT_BACKOFF:.0f}s 后重发")
            self._queue.append(cur)
            self._dispatch_block_until = time.time() + REJECT_BACKOFF

        else:  # failed / 未知 / 超时
            # v0.4 (P4): 失败时云台同样停在该目标附近（盲跳已执行），贪心参考
            #   跟着更新，避免与实际指向脱节导致次优排序。被拒(rejected)时
            #   执行层没动过云台，故不更新。
            if not EXECUTOR_RECENTERS:
                self._gimbal_ref = (cur["x"], cur["y"])
            n = cur.get("_retry", 0)
            if n < MAX_RETRY:
                cur["_retry"] = n + 1
                self.get_logger().warn(
                    f"[PLANNER] ⚠️ id={cur['id']} 失败，重排队尾重试 "
                    f"({cur['_retry']}/{MAX_RETRY})")
                self._queue.append(cur)
            else:
                self.get_logger().warn(
                    f"[PLANNER] ✗ id={cur['id']} 多次失败，放弃")
                self._failed.append(cur)

        self._current = None
        self.state = ST_CLEARING

    # ────────────────────────────────────────────────────────────
    #  建队：过滤 + 合并 + 分配 id
    # ────────────────────────────────────────────────────────────
    def _filter_and_merge(self, boxes):
        """对一帧 boxes 做 4 重过滤 + 帧内近邻合并。
        返回 (merged, n_crop, n_lowconf, n_size, n_edge)；merged 为 [{x,y,conf}]，
        **不分配 id**（投票时每帧都会调用，id 留到确认目标后再分配）。"""
        cand = []
        n_crop = n_lowconf = n_size = n_edge = 0
        for b in boxes:
            cx, cy = b.get("cx"), b.get("cy")
            if cx is None or cy is None:
                continue
            cx, cy = int(cx), int(cy)

            # 过滤① 类别——只打 weed，绝不打 crop（安全关键）。
            #   yolo_detector.py 的 box 一定带 "label"；缺失视为不可信 → 丢弃。
            if TARGET_LABELS and b.get("label") not in TARGET_LABELS:
                n_crop += 1
                continue
            # 过滤② 置信度（yolo_detector.py 字段名为 "confidence"）
            conf = float(b.get("confidence", b.get("conf", 1.0)))
            if conf < MIN_CONF:
                n_lowconf += 1
                continue
            # 过滤③ 尺寸（yolo_detector.py 一定带 w/h）
            w, h = b.get("w"), b.get("h")
            if w is not None and h is not None:
                area = float(w) * float(h)
                if not (MIN_BOX_AREA <= area <= MAX_BOX_AREA):
                    n_size += 1
                    continue
            # 过滤④ 贴画面边缘（部分出画 / 云台难瞄）
            if (cx < EDGE_MARGIN or cx > IMG_W - EDGE_MARGIN or
                    cy < EDGE_MARGIN or cy > IMG_H - EDGE_MARGIN):
                n_edge += 1
                continue
            cand.append({"x": cx, "y": cy, "conf": conf})

        # 帧内合并近邻重复框（同株草被检测成多个框）：保留置信度高的中心
        merged = []
        for c in cand:
            dup = None
            for m in merged:
                if ((m["x"] - c["x"]) ** 2 + (m["y"] - c["y"]) ** 2
                        < DEDUP_RADIUS ** 2):
                    dup = m
                    break
            if dup is None:
                merged.append(c)
            elif c["conf"] > dup["conf"]:
                dup.update(x=c["x"], y=c["y"], conf=c["conf"])
        return merged, n_crop, n_lowconf, n_size, n_edge

    def _build_queue(self, boxes):
        """单帧建队（VOTE_MIN_RATIO<=0 的退化路径 / 兜底）：过滤合并后直接分配 id。"""
        merged, n_crop, n_lowconf, n_size, n_edge = self._filter_and_merge(boxes)
        for m in merged:
            m["id"] = self._next_id
            m["_retry"] = 0
            self._next_id += 1
        self.get_logger().info(
            f"[PLANNER] 单帧建队(YOLO frame_id={self._latest_frame_id})：{len(boxes)} 框 "
            f"→ 丢弃(crop {n_crop} / 低置信 {n_lowconf} / 尺寸 {n_size} / "
            f"贴边 {n_edge}) → 合并后 {len(merged)} 个 weed 目标")
        return merged

    # ────────────────────────────────────────────────────────────
    #  v0.3 时序投票：跨帧累积 + 频次确认
    # ────────────────────────────────────────────────────────────
    def _vote_reset(self):
        self._vote_clusters = []
        self._vote_frames = 0
        self._vote_last_frame_id = None

    def _vote_add_frame(self, boxes):
        """把一帧的（过滤+帧内合并后）weed 检测并入投票簇。每调用一次算一帧。"""
        merged, *_ = self._filter_and_merge(boxes)
        for d in merged:
            placed = False
            for c in self._vote_clusters:
                rx = c["sum_x"] / c["count"]
                ry = c["sum_y"] / c["count"]
                if (rx - d["x"]) ** 2 + (ry - d["y"]) ** 2 < DEDUP_RADIUS ** 2:
                    c["sum_x"] += d["x"]; c["sum_y"] += d["y"]
                    c["sum_conf"] += d["conf"]; c["count"] += 1
                    placed = True
                    break
            if not placed:
                self._vote_clusters.append({
                    "sum_x": float(d["x"]), "sum_y": float(d["y"]),
                    "sum_conf": float(d["conf"]), "count": 1})
        self._vote_frames += 1

    def _finalize_vote(self):
        """窗口结束：按出现频次确认目标，位置取均值，分配 id。返回队列。"""
        frames = max(1, self._vote_frames)
        confirmed = []
        dropped = 0
        for c in self._vote_clusters:
            ratio = c["count"] / frames
            if ratio >= VOTE_MIN_RATIO:
                confirmed.append({
                    "x": int(round(c["sum_x"] / c["count"])),
                    "y": int(round(c["sum_y"] / c["count"])),
                    "conf": c["sum_conf"] / c["count"],
                    "id": self._next_id, "_retry": 0,
                    "_votes": c["count"]})
                self._next_id += 1
            else:
                dropped += 1
        self.get_logger().info(
            f"[PLANNER] 投票建队：窗口 {self._vote_frames} 帧 / 候选簇 "
            f"{len(self._vote_clusters)} 个 → 确认 {len(confirmed)}（频次≥"
            f"{VOTE_MIN_RATIO:.0%}）/ 丢弃低频 {dropped} → "
            f"目标 {[ (t['id'], t['_votes']) for t in confirmed ]}")
        return confirmed

    # ────────────────────────────────────────────────────────────
    #  收发原语
    # ────────────────────────────────────────────────────────────
    def _dispatch(self, tgt):
        msg = String()
        msg.data = json.dumps({
            "id": tgt["id"], "x": tgt["x"], "y": tgt["y"],
            # v0.5: 本片已打目标(中心参考系坐标)。执行层重捕获/跟踪时排除其
            #   邻域,杜绝"打二又打回一"式重复打击(两株草靠近时尤其关键)。
            "exclude": [[int(t["x"]), int(t["y"])] for t in self._struck],
            # v3.11.1: 本片其它待打目标(中心参考系坐标)。派发时 tgt 已从 _queue
            #   移除,故 _queue 即"其它待打"。执行层身份核验用它把候选框分类到正确
            #   目标,防抓到旁株未打目标(③那种)。
            "others": [[int(t["x"]), int(t["y"])] for t in self._queue],
        })
        self.pub_cmd.publish(msg)
        self._dispatch_at = time.time()
        self.get_logger().info(
            f"[PLANNER] → 下发打击 id={tgt['id']} 目标({tgt['x']},{tgt['y']})  "
            f"已打排除 {len(self._struck)} 个  其它待打 {len(self._queue)} 个  "
            f"队列剩余 {len(self._queue)}")

    def _hit_counts(self):
        """v0.6: 统计本会话已打目标的命中判定 (命中, 脱靶, 未检出/未启用)。"""
        nh = sum(1 for t in self._struck if t.get("hit") is True)
        nm = sum(1 for t in self._struck if t.get("hit") is False)
        nu = len(self._struck) - nh - nm
        return nh, nm, nu

    def _publish_session(self):
        """v0.6: 广播会话快照(5Hz,由 _tick 末尾调用)。纯显示数据。"""
        def _t(t):
            return {"id": t["id"], "x": t["x"], "y": t["y"]}
        def _ts(t):
            d = _t(t)
            d["hit"] = t.get("hit")
            d["hit_dist"] = t.get("hit_dist")
            return d
        nh, _, _ = self._hit_counts()
        payload = {
            "state": self.state,
            "total": self._session_total,
            "pending": [_t(t) for t in self._queue],
            "current": _t(self._current) if self._current else None,
            "struck": [_ts(t) for t in self._struck],
            "failed": [_t(t) for t in self._failed],
            "hits": nh,
        }
        if self.state == ST_VOTING:
            payload["voting"] = {"frames": self._vote_frames,
                                 "clusters": len(self._vote_clusters)}
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_session.publish(msg)

    def _publish_patch_clear(self):
        nh, nm, nu = self._hit_counts()
        msg = String()
        msg.data = json.dumps({
            "cleared": len(self._struck),
            "failed": len(self._failed),
            "total": self._session_total,
            # v0.6: 蓝斑命中判定统计(执行层不支持时全 0/None,消费端可忽略)
            "hit_confirmed": nh,
            "hit_missed": nm,
            "hit_unseen": nu,
        })
        self.pub_patch_clear.publish(msg)
        self.get_logger().info(
            f"[PLANNER] → 发布 patch_clear（车控节点可驱车前进）")

    # ────────────────────────────────────────────────────────────
    #  小工具
    # ────────────────────────────────────────────────────────────
    def _fresh_boxes(self):
        """返回新鲜的 YOLO 框列表；无 / 过期则 None。"""
        if self._latest_boxes is None:
            return None
        if time.time() - self._boxes_at > YOLO_FRESH_SEC:
            return None
        return self._latest_boxes

    @staticmethod
    def _dist(tgt, ref):
        return (tgt["x"] - ref[0]) ** 2 + (tgt["y"] - ref[1]) ** 2


def main(args=None):
    rclpy.init(args=args)
    node = StrikePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
