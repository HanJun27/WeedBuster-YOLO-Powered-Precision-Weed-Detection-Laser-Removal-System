#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
servo_jacobian_test.py (v2) —— 开环标定:逐轴测"云台转 1° 光斑移动多少像素"
                                + 检测诊断(分辨率/过曝/通道)

v2 改动(修上一版"中位测不到光斑"):
  · 检测改为**全画面搜索**(上一版只在 SPOT_HOME 周围 200px ROI 找,
    若相机分辨率非 640x480 或 SPOT_HOME 没实测,光斑就不在 ROI 里 → 漏检)。
  · 启动即打印**相机分辨率**(若非 640x480,则 SPOT_HOME 默认值是错的)。
  · 检测失败时打印诊断:全画面最大"红主导值"、该点 BGR、超阈值像素数
    → 一眼区分:分辨率问题 / 过曝(白芯,R≈G≈B≈255)/ 通道顺序 / 算法。

⚠️ 把白纸/标定板放在**实际打草的工作距离**上;视野里别有其它红色物。

运行(详见对话):
  pkill -f vision_servo            # 关掉 vision_servo,别和本脚本抢舵机
  # 终端A: ros2 run laser_calibration stereo_camera
  # 终端B: python3 ~/servo_jacobian_test.py   [--deg 10]
"""
import argparse
import sys
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from laser_calibration.config import (
    TOPIC_RGB, SERVO_YAW_CENTER, SERVO_PITCH_CENTER,
    SPOT_HOME_X, SPOT_HOME_Y, PIXEL_TO_YAW_DEG, PIXEL_TO_PITCH_DEG,
    RED_DOMINANCE_MIN, RED_SPOT_AREA_MIN, RED_SPOT_AREA_MAX,
    SPOT_CLOSE_KERNEL_SIZE,
)
from laser_calibration.robot_ctrl import set_servo, laser_ir, all_lasers_off


def detect_red_spot_fullframe(bgr):
    """全画面 R-max(G,B) 红主导 → 形态学闭 → 取面积最大的合格轮廓质心。
    与工程 find_red_spot 的检测核一致,只是固定走全画面(标定场景单光斑、最稳)。"""
    b, g, r = cv2.split(bgr)
    r_i, g_i, b_i = r.astype(np.int16), g.astype(np.int16), b.astype(np.int16)
    red_score = np.clip(r_i - np.maximum(g_i, b_i), 0, 255).astype(np.uint8)
    _, mask = cv2.threshold(red_score, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)
    k = np.ones((SPOT_CLOSE_KERNEL_SIZE, SPOT_CLOSE_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in cnts:
        a = cv2.contourArea(c)
        if not (RED_SPOT_AREA_MIN < a < RED_SPOT_AREA_MAX):
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        if a > best_area:
            best_area = a
            best = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
    return best


def diagnose_frame(bgr):
    """检测失败时,打印一帧的诊断信息,定位是分辨率/过曝/通道/算法。"""
    h, w = bgr.shape[:2]
    b, g, r = cv2.split(bgr)
    r_i, g_i, b_i = r.astype(np.int16), g.astype(np.int16), b.astype(np.int16)
    red_score = np.clip(r_i - np.maximum(g_i, b_i), 0, 255).astype(np.uint8)
    maxv = int(red_score.max())
    ys, xs = np.where(red_score == maxv)
    mx, my = int(xs[0]), int(ys[0])
    _, mask = cv2.threshold(red_score, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)
    n_over = int(cv2.countNonZero(mask))
    # 全画面最亮像素(查过曝)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    bright_v = int(gray.max())
    bys, bxs = np.where(gray == bright_v)
    bx, by = int(bxs[0]), int(bys[0])
    print("  ── 检测诊断 ─────────────────────────────")
    print(f"   相机分辨率 = {w}x{h}"
          + ("  (≠640x480 → SPOT_HOME 默认值是错的!)" if (w, h) != (640, 480) else ""))
    print(f"   全画面最大『红主导值 R-max(G,B)』= {maxv}  (检测阈值 RED_DOMINANCE_MIN={RED_DOMINANCE_MIN})")
    print(f"     该点@({mx},{my})  BGR=({int(b[my,mx])},{int(g[my,mx])},{int(r[my,mx])})")
    print(f"   超过红主导阈值的像素数 = {n_over}")
    print(f"   全画面最亮点@({bx},{by})  BGR=({int(b[by,bx])},{int(g[by,bx])},{int(r[by,bx])})  灰度={bright_v}")
    print("   解读:")
    if maxv < RED_DOMINANCE_MIN:
        print(f"     · 最大红主导值 {maxv} < 阈值 {RED_DOMINANCE_MIN} → 画面里没有足够『红』的点。")
        if bright_v >= 250 and abs(int(r[by,bx])-int(b[by,bx])) < 25:
            print("     · 但最亮点三通道都接近 255(白) → **过曝**:光斑芯被烧成白色,"
                  "不再红主导。解决:调低相机曝光/增益,或在镜头前减光。")
        else:
            print("     · 也可能是**通道顺序**问题:若你的红激光点在最亮处但 R 不是最大、"
                  "反而 B 最大 → 取流是 RGB 被当成 BGR 了。把这行 BGR 值发我。")
    else:
        print(f"     · 最大红主导值 {maxv} ≥ 阈值 → 有红点,但没通过面积过滤"
              f"(RED_SPOT_AREA_MIN={RED_SPOT_AREA_MIN}, MAX={RED_SPOT_AREA_MAX})。"
              "可能光斑太小/太大。把上面这些数发我。")
    print("  ─────────────────────────────────────────")


class JacobianTester(Node):
    def __init__(self):
        super().__init__("servo_jacobian_test")
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)

    def _cb_rgb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"解码失败: {e}")
            return
        with self._lock:
            self._frame = bgr

    def _get_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def measure_spot(self, label, n=15, gap=0.05):
        """连取 n 帧全画面检测,取中位数;失败则打印诊断。"""
        xs, ys = [], []
        last = None
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.2)
            frame = self._get_frame()
            if frame is None:
                time.sleep(gap)
                continue
            last = frame
            s = detect_red_spot_fullframe(frame)
            if s is not None:
                xs.append(s[0]); ys.append(s[1])
            time.sleep(gap)
        if len(xs) < max(3, n // 3):
            self.get_logger().error(
                f"  ✗ [{label}] 有效检测 {len(xs)}/{n} 帧,光斑没找稳。")
            if last is not None:
                diagnose_frame(last)
            return None
        sx, sy = int(np.median(xs)), int(np.median(ys))
        self.get_logger().info(f"  [{label}] 光斑=({sx},{sy})  (有效 {len(xs)}/{n})")
        return (sx, sy)

    def goto(self, yaw, pitch, settle):
        set_servo(int(round(yaw)), int(round(pitch)))
        t0 = time.time()
        while time.time() - t0 < settle:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deg", type=float, default=10.0)
    ap.add_argument("--settle", type=float, default=1.0)
    args = ap.parse_args()
    d = args.deg
    cy, cp = SERVO_YAW_CENTER, SERVO_PITCH_CENTER

    rclpy.init()
    node = JacobianTester()
    print("\n等待相机首帧 ...")
    t0 = time.time()
    while node._get_frame() is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.2)
    f0 = node._get_frame()
    if f0 is None:
        print(f"✗ 8s 没收到相机帧。确认 stereo_camera 在跑、话题={TOPIC_RGB}。")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    h, w = f0.shape[:2]
    print(f"✓ 收到相机帧,分辨率 = {w}x{h}"
          + ("  ⚠️ 非 640x480 → config 的 SPOT_HOME(320,240) 是错的!" if (w, h) != (640, 480) else ""))

    try:
        laser_ir(True)
        time.sleep(0.3)
        print("\n━━━━━━━━━ 开环 Jacobian 标定 ━━━━━━━━━")
        print(f"中位 yaw={cy} pitch={cp},单边 {d}°  ⚠️ 目标须在【工作距离】")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

        node.goto(cy, cp, args.settle)
        s_c = node.measure_spot("中位")
        if s_c is None:
            raise RuntimeError("中位测不到光斑;看上面诊断,先解决检测。")

        node.goto(cy + d, cp, args.settle); yp = node.measure_spot(f"yaw+{d:.0f}")
        node.goto(cy - d, cp, args.settle); ym = node.measure_spot(f"yaw-{d:.0f}")
        node.goto(cy, cp, args.settle)
        node.goto(cy, cp + d, args.settle); pp = node.measure_spot(f"pitch+{d:.0f}")
        node.goto(cy, cp - d, args.settle); pm = node.measure_spot(f"pitch-{d:.0f}")
        node.goto(cy, cp, args.settle)

        print("\n━━━━━━━━━━━━━ 结果 ━━━━━━━━━━━━━")
        dxh, dyh = s_c[0] - SPOT_HOME_X, s_c[1] - SPOT_HOME_Y
        print(f"[SPOT_HOME 检查] 静止光斑=({s_c[0]},{s_c[1]})  "
              f"配置=({SPOT_HOME_X},{SPOT_HOME_Y})  偏差=({dxh:+d},{dyh:+d})px")
        if abs(dxh) > 40 or abs(dyh) > 40:
            print(f"  → 偏差大,建议 config 的 SPOT_HOME 改成 ({s_c[0]},{s_c[1]})")

        two_d = 2.0 * d
        if yp and ym:
            dx, dy = yp[0] - ym[0], yp[1] - ym[1]
            print(f"\n[YAW] 转 {two_d:.0f}° 光斑位移 Δx={dx:+d} Δy={dy:+d} px")
            if abs(dx) < 5:
                print(f"  ⚠️ 光斑几乎不随 yaw 动({abs(dx)/two_d:.2f}px/°)"
                      " → 坐实:光斑是固定准星,『驱动光斑』的闭环必然不收敛。")
            else:
                print(f"  x 速率={dx/two_d:+.2f}px/°  → 建议 PIXEL_TO_YAW_DEG="
                      f"{two_d/dx:+.3f} (当前{PIXEL_TO_YAW_DEG:+.3f})")
                if abs(dy) > abs(dx) * 0.5:
                    print(f"  ⚠️ y 串扰大(Δy={dy:+d}),两轴不解耦/相机有 roll。")
        if pp and pm:
            dx, dy = pp[0] - pm[0], pp[1] - pm[1]
            print(f"\n[PITCH] 转 {two_d:.0f}° 光斑位移 Δx={dx:+d} Δy={dy:+d} px")
            if abs(dy) < 5:
                print(f"  ⚠️ 光斑几乎不随 pitch 动 → 同上,固定准星。")
            else:
                print(f"  y 速率={dy/two_d:+.2f}px/°  → 建议 PIXEL_TO_PITCH_DEG="
                      f"{two_d/dy:+.3f} (当前{PIXEL_TO_PITCH_DEG:+.3f}) [pitch 符号此前未验证]")
                if abs(dx) > abs(dy) * 0.5:
                    print(f"  ⚠️ x 串扰大(Δx={dx:+d}),两轴不解耦。")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")

    except KeyboardInterrupt:
        print("\n中断。")
    except Exception as e:
        print(f"\n✗ 出错: {e}")
    finally:
        try:
            all_lasers_off(); set_servo(int(cy), int(cp))
        except Exception:
            pass
        node.destroy_node(); rclpy.shutdown()
        print("已关激光、云台回中。")


if __name__ == "__main__":
    main()
