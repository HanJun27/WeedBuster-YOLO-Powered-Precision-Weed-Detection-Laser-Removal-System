#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
save_spot.py —— 开红激光、抓一帧、把检测到的光斑标在图上,存 PNG 供肉眼诊断。
用途:看清红光斑到底在画面哪里、是不是干净的点、有没有过曝糊成白、是否贴边。

运行:
  pkill -f vision_servo
  # 终端A: ros2 run laser_calibration stereo_camera
  # 终端B: source ~/yahboomcar_ws/install/setup.bash
  #        python3 ~/save_spot.py
输出: ~/spot_diag.png  (在文件管理器里打开,截图发我)
"""
import time, threading
import cv2, numpy as np, rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from laser_calibration.config import (
    TOPIC_RGB, SPOT_HOME_X, SPOT_HOME_Y, SERVO_YAW_CENTER, SERVO_PITCH_CENTER,
    RED_DOMINANCE_MIN, RED_SPOT_AREA_MIN, RED_SPOT_AREA_MAX, SPOT_CLOSE_KERNEL_SIZE,
)
from laser_calibration.robot_ctrl import laser_ir, all_lasers_off, set_servo

OUT = "/home/sunrise/spot_diag.png"

class Grab(Node):
    def __init__(self):
        super().__init__("save_spot")
        self.br = CvBridge(); self.lock = threading.Lock(); self.f = None
        self.create_subscription(Image, TOPIC_RGB, self._cb, 10)
    def _cb(self, m):
        try: bgr = self.br.imgmsg_to_cv2(m, "bgr8")
        except Exception: return
        with self.lock: self.f = bgr
    def get(self):
        with self.lock: return None if self.f is None else self.f.copy()

def main():
    rclpy.init(); n = Grab()
    print("等待相机帧..."); t0 = time.time()
    while n.get() is None and time.time()-t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.get() is None:
        print("✗ 没收到相机帧,确认 stereo_camera 在跑"); n.destroy_node(); rclpy.shutdown(); return
    try:
        # 云台居中 + 开红激光
        try: set_servo(SERVO_YAW_CENTER, SERVO_PITCH_CENTER)
        except Exception: pass
        laser_ir(True); time.sleep(0.8)
        for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
        bgr = n.get()
        h, w = bgr.shape[:2]

        # 全画面红主导检测
        b, g, r = cv2.split(bgr)
        rs = np.clip(r.astype(np.int16) - np.maximum(g.astype(np.int16), b.astype(np.int16)), 0, 255).astype(np.uint8)
        maxv = int(rs.max()); ys, xs = np.where(rs == maxv); mx, my = int(xs[0]), int(ys[0])
        _, mask = cv2.threshold(rs, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((SPOT_CLOSE_KERNEL_SIZE,)*2, np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        det = None; det_area = 0
        for c in cnts:
            a = cv2.contourArea(c)
            if not (RED_SPOT_AREA_MIN < a < RED_SPOT_AREA_MAX): continue
            m = cv2.moments(c)
            if m["m00"] == 0: continue
            if a > det_area:
                det_area = a; det = (int(m["m10"]/m["m00"]), int(m["m01"]/m["m00"]))

        # 标注:绿圈=检测到的合格光斑;红叉=全画面最红点;黄叉=配置 SPOT_HOME
        vis = bgr.copy()
        cv2.drawMarker(vis, (mx, my), (0,0,255), cv2.MARKER_CROSS, 30, 2)        # 最红点
        cv2.drawMarker(vis, (SPOT_HOME_X, SPOT_HOME_Y), (0,255,255), cv2.MARKER_TILTED_CROSS, 24, 2)  # 配置SPOT_HOME
        if det is not None:
            cv2.circle(vis, det, 18, (0,255,0), 2)                               # 检测合格光斑
        txt = [f"{w}x{h}", f"maxRed={maxv}(thr{RED_DOMINANCE_MIN})@({mx},{my})",
               f"det={det} area={int(det_area)}", f"SPOT_HOME=({SPOT_HOME_X},{SPOT_HOME_Y})"]
        for i, t in enumerate(txt):
            cv2.putText(vis, t, (8, 22+22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
            cv2.putText(vis, t, (8, 22+22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1)
        cv2.imwrite(OUT, vis)
        # 也存一张纯红主导热力图,看光斑形状/过曝
        cv2.imwrite(OUT.replace(".png", "_redscore.png"), rs)
        print(f"✓ 已存 {OUT}  (和 _redscore.png)")
        print(f"  分辨率 {w}x{h}; 最红点 maxRed={maxv}@({mx},{my}) BGR=({int(b[my,mx])},{int(g[my,mx])},{int(r[my,mx])})")
        print(f"  检测合格光斑 det={det} area={int(det_area)}")
        print("  绿圈=检测到的光斑, 红叉=最红点, 黄叉=配置SPOT_HOME(320,240)")
    finally:
        try: all_lasers_off()
        except Exception: pass
        n.destroy_node(); rclpy.shutdown(); print("激光已关")

if __name__ == "__main__":
    main()
