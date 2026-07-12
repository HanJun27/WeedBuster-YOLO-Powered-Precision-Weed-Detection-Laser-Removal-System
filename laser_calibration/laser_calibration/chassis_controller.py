#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
chassis_controller.py —— 车控节点(ROS 壳)  v0.1
=====================================================
把 chassis_fsm 的纯逻辑接到 ROS2 上,打通"走—停—清—走"完整闭环:

  订阅:
    /yolo/weed_detected   String(JSON)  侧面云台相机的检测(与 planner 同源)
    /planner/patch_clear  String        本片清完信号
    /chassis/start        Empty         开始巡航(STANDBY/FAULT → CRUISE)
    /chassis/stop         Empty         停车待命
    /safety_stop          Empty         全局急停(本节点停车;vision_servo 同时灭激光)
  发布:
    /cmd_vel              Twist         10Hz 持续发布(扬声底盘标准接口)
    /planner/start_clearing  Empty      刹停沉降后触发清场
    /servo/recenter       Empty         盲走开始时请求云台归中

  使用前提:
    · 扬声底盘驱动已运行(即键盘遥控能开动小车的那套 bringup);
    · vision_servo 触发模式 = manual(由本节点经 planner 统一调度);
    · 上电后默认 STANDBY 静止,必须手动发 /chassis/start 才会动 —— 安全默认。

  现场调参在 chassis_fsm.py 顶部(速度/防抖帧数/盲走时长等)。
"""
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String

from laser_calibration.chassis_fsm import ChassisFSM, EV_START_CLEARING

# 话题名(与 planner / vision_servo 侧常量保持一致)
TOPIC_YOLO           = "/yolo/weed_detected"
TOPIC_PATCH_CLEAR    = "/planner/patch_clear"
TOPIC_START_CLEARING = "/planner/start_clearing"
TOPIC_RECENTER       = "/servo/recenter"
TOPIC_CMD_VEL        = "cmd_vel"            # 与扬声键盘遥控同名(相对名)
TOPIC_CH_START       = "/chassis/start"
TOPIC_CH_STOP        = "/chassis/stop"
TOPIC_SAFETY_STOP    = "/safety_stop"
# v3.13.0: 底盘状态广播(纯发布) —— 任务面板显示 走/停/清 的实时状态。
#   ⚠️ 与 vision_servo.py 的 TOPIC_CHASSIS_STATE 字符串必须一致。
TOPIC_CH_STATE       = "/chassis/state"

MIN_CONF = 0.50         # 停车判定的置信度下限(与 planner 建队一致)
TICK_SEC = 0.10         # 10Hz:状态机节拍 = cmd_vel 发布节拍


def _now():
    import time
    return time.time()


class ChassisController(Node):
    def __init__(self):
        super().__init__("chassis_controller")
        self.fsm = ChassisFSM(log=lambda s: self.get_logger().info(s))

        self.pub_vel = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        self.pub_start_clearing = self.create_publisher(
            Empty, TOPIC_START_CLEARING, 10)
        self.pub_recenter = self.create_publisher(Empty, TOPIC_RECENTER, 10)
        # v3.13.0: 状态广播(纯显示,收不到也不影响任何控制)
        self.pub_state = self.create_publisher(String, TOPIC_CH_STATE, 10)

        self.create_subscription(String, TOPIC_YOLO, self._cb_yolo, 10)
        self.create_subscription(String, TOPIC_PATCH_CLEAR,
                                 self._cb_patch_clear, 10)
        self.create_subscription(Empty, TOPIC_CH_START, self._cb_start, 10)
        self.create_subscription(Empty, TOPIC_CH_STOP, self._cb_stop, 10)
        self.create_subscription(Empty, TOPIC_SAFETY_STOP,
                                 self._cb_safety, 10)

        self._last_frame_id = None
        self.create_timer(TICK_SEC, self._tick)
        self.get_logger().info(
            "[CHASSIS] 车控节点 v0.1 就绪(STANDBY)。"
            "发 /chassis/start 开始巡航;/chassis/stop 收工;/safety_stop 急停。")

    # ── 订阅回调 ────────────────────────────────────────────────
    def _cb_yolo(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # 同一帧可能重发:按 frame_id 去重,保证"连续 N 帧"数的是新帧
        fid = data.get("frame_id")
        if fid is not None and fid == self._last_frame_id:
            return
        self._last_frame_id = fid
        boxes = data.get("boxes") or []
        has_weed = any(
            b.get("label") == "weed"
            and float(b.get("confidence", b.get("conf", 0.0))) >= MIN_CONF
            for b in boxes)
        self.fsm.on_yolo_frame(has_weed, _now())

    def _cb_patch_clear(self, _msg):
        self.fsm.on_patch_clear(_now())

    def _cb_start(self, _msg):
        self.fsm.on_start(_now())

    def _cb_stop(self, _msg):
        self.fsm.on_stop(_now())

    def _cb_safety(self, _msg):
        self.fsm.on_safety_stop(_now())

    # ── 10Hz 主循环 ─────────────────────────────────────────────
    def _tick(self):
        vx, events = self.fsm.tick(_now())
        for ev in events:
            if ev == EV_START_CLEARING:
                self.pub_start_clearing.publish(Empty())
                self.get_logger().info("[CHASSIS] → 已发 start_clearing")
        if self.fsm.pop_recenter():
            self.pub_recenter.publish(Empty())
            self.get_logger().info("[CHASSIS] → 已发 /servo/recenter(盲走中归中)")
        t = Twist()
        t.linear.x = float(vx)
        self.pub_vel.publish(t)          # 每拍都发:速度为 0 也发,看门狗友好
        # v3.13.0: 每拍广播底盘状态(小 JSON,纯显示)
        sm = String()
        sm.data = json.dumps({"state": self.fsm.state, "vx": round(float(vx), 3)})
        self.pub_state.publish(sm)


def main():
    rclpy.init()
    node = ChassisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前确保停车
        try:
            node.pub_vel.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
