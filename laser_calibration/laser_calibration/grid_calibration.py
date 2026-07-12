#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grid_calibration.py —— 自动网格标定(方案②落地)  v3.12.0 新增
==============================================================
一条命令自动完成过去手工两点标定做的事,并且做得更细:

  云台自动扫 GRID_N×GRID_N 姿态网格,每个姿态:
    · 开红激光(S4) → find_red_spot   → 红斑像素 spot(yaw,pitch)
    · 短蓝脉冲(S3) → find_blue_spot  → Δ(yaw,pitch) = 蓝斑 − 红斑
  相邻姿态间:
    · cv2.phaseCorrelate 整幅相位相关 → 像素平移/角度 = K 的逐处采样
  结束后:
    · 最小二乘拟合 K(含轴间耦合 2×2 矩阵)、SPOT_HOME 漂移面、Δ 双线性面
    · 打印【残差报告】:若继续用 config 常量,网格各处瞄准点会偏多少像素
    · 打印【推荐常量】:把哪几个 config 值改成多少(全网格最小二乘意义最优)
    · 原始样本 + 拟合系数存 ~/calib_grid.json(逐姿态查表版留待后续接入)

定位:这是**独立维护工具**,不接入打击链路 —— 输出是"更准的常量 + 残差
证据"。跑一次 ≈2 分钟,替代 15 分钟手工标定,且给出全工作区残差数据
(答辩"系统标定"章节素材)。

运行(⚠️ 会自动开关激光并转动云台,先清场、放好 A4 白纸铺满扫描区):
    终端1: ros2 run laser_calibration stereo_camera
    终端2: ros2 run laser_calibration grid_calibration
    (不要同时运行 vision_servo —— 两个进程抢串口舵机)

安全:
    · 蓝脉冲仅 GRID_BLUE_PULSE_SEC(默认 0.15s),低功率打印靶无烧灼;
      仍不放心可 GRID_MEASURE_DELTA=False,只标 SPOT_HOME/K(不开蓝光)。
    · 任何异常/Ctrl+C → finally 强制 all_lasers_off + 归中。
"""

import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from laser_calibration.config import (
    TOPIC_RGB,
    SERVO_YAW_CENTER, SERVO_PITCH_CENTER,
    PIXEL_TO_YAW_DEG, PIXEL_TO_PITCH_DEG,
    SPOT_HOME_X, SPOT_HOME_Y,
)
from laser_calibration import robot_ctrl
# 复用执行层的两套光斑检测(只 import,不改 vision_servo 一行)
from laser_calibration.vision_servo import find_red_spot, find_blue_spot
from laser_calibration.calib_io import load_calib

# ── 可调参数 ──────────────────────────────────────────────────
GRID_N              = 5      # 网格 N×N(5×5=25 姿态,~2 分钟)
GRID_YAW_SPAN_DEG   = 8      # 中心 ± 跨度(整数度;PWM 舵机 1° 分辨率)
GRID_PITCH_SPAN_DEG = 6
GRID_SETTLE_SEC     = 0.6    # 每姿态沉降
GRID_FRAMES         = 3      # 每姿态红斑取样帧数(取中位)
GRID_MEASURE_DELTA  = True   # 打短蓝脉冲测 Δ(False=只标红斑/K,不开蓝光)
GRID_BLUE_PULSE_SEC = 0.15
GRID_OUT_JSON       = "~/calib_grid.json"


def _bilinear_fit(us, vs, zs):
    """z ≈ a0 + a1·u + a2·v + a3·u·v 最小二乘。返回 (系数, 残差RMS)。"""
    A = np.stack([np.ones_like(us), us, vs, us * vs], axis=1)
    coef, *_ = np.linalg.lstsq(A, zs, rcond=None)
    rms = float(np.sqrt(np.mean((A @ coef - zs) ** 2)))
    return coef.tolist(), rms


class GridCalibration(Node):
    def __init__(self):
        super().__init__("grid_calibration")
        self.bridge = CvBridge()
        self._frame = None
        self._lock = threading.Lock()
        self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker_started = False
        self.create_timer(0.2, self._maybe_start)
        self.get_logger().info(
            f"网格标定 v3.12.0: {GRID_N}×{GRID_N} 网格, yaw±{GRID_YAW_SPAN_DEG}°"
            f" pitch±{GRID_PITCH_SPAN_DEG}°, Δ测量="
            f"{'开(短蓝脉冲)' if GRID_MEASURE_DELTA else '关'}  等相机帧…")

    def _cb_rgb(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._frame = f
        except Exception:
            pass

    def _get(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def _maybe_start(self):
        if not self._worker_started and self._get() is not None:
            self._worker_started = True
            self._worker.start()

    # ── 采样原语 ────────────────────────────────────────────
    def _grab_fresh(self, n=1, gap=0.05):
        out = []
        for _ in range(n):
            time.sleep(gap)
            f = self._get()
            if f is not None:
                out.append(f)
        return out

    def _median_spot(self, detector, hint):
        pts = []
        score = 0
        for f in self._grab_fresh(GRID_FRAMES):
            r = detector(f, hint[0], hint[1]) if hint else detector(f)
            if r is not None:
                pts.append((r[0], r[1]))
                if len(r) > 2:
                    score = max(score, r[2])
        if len(pts) < max(1, GRID_FRAMES - 1):
            return None, score
        xs = sorted(p[0] for p in pts)
        ys = sorted(p[1] for p in pts)
        return (xs[len(xs) // 2], ys[len(ys) // 2]), score

    # ── 主流程 ──────────────────────────────────────────────
    def _run(self):
        log = self.get_logger()
        try:
            log.warn("★ 3 秒后开始:云台将自动扫描并开关激光,确认场地清空 ★")
            time.sleep(3.0)
            half = GRID_N // 2
            yaws = [SERVO_YAW_CENTER +
                    round(GRID_YAW_SPAN_DEG * (i - half) / max(1, half))
                    for i in range(GRID_N)]
            pitches = [SERVO_PITCH_CENTER +
                       round(GRID_PITCH_SPAN_DEG * (i - half) / max(1, half))
                       for i in range(GRID_N)]
            samples = []       # 逐姿态: yaw,pitch,spot,delta,blue_score
            k_pairs = []       # (dyaw,dpitch,dpx_x,dpx_y) 相位相关采样
            prev_gray, prev_pose = None, None
            hann = None

            robot_ctrl.all_lasers_off()
            order = []
            for r_i, pv in enumerate(pitches):     # 蛇形扫描省行程
                row = yaws if r_i % 2 == 0 else list(reversed(yaws))
                order += [(yv, pv) for yv in row]

            for i, (yv, pv) in enumerate(order):
                robot_ctrl.set_servo(yv, pv)
                time.sleep(GRID_SETTLE_SEC)
                frames = self._grab_fresh(1)
                if not frames:
                    log.warn(f"姿态({yv},{pv}) 无帧,跳过")
                    continue
                gray = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY
                                    ).astype(np.float32)
                if hann is None:
                    hann = cv2.createHanningWindow(
                        (gray.shape[1], gray.shape[0]), cv2.CV_32F)
                if prev_gray is not None:
                    (sx, sy), resp = cv2.phaseCorrelate(prev_gray, gray, hann)
                    dyaw = yv - prev_pose[0]
                    dpitch = pv - prev_pose[1]
                    if resp > 0.1 and (dyaw or dpitch):
                        # 场景在画面中的平移 = −相机视轴移动;phaseCorrelate 给
                        # 的是 prev→cur 的图像位移,直接与 dAngle 做线性拟合
                        k_pairs.append((dyaw, dpitch, sx, sy))
                prev_gray, prev_pose = gray, (yv, pv)

                robot_ctrl.laser_ir(True)
                time.sleep(0.25)
                spot, _ = self._median_spot(
                    find_red_spot, (SPOT_HOME_X, SPOT_HOME_Y))
                robot_ctrl.laser_ir(False)

                delta, bscore = None, 0
                if GRID_MEASURE_DELTA and spot is not None:
                    robot_ctrl.laser_blue(True)
                    time.sleep(min(0.1, GRID_BLUE_PULSE_SEC))
                    bpt, bscore = self._median_spot(
                        find_blue_spot,
                        (spot[0] + 56, spot[1]))   # 粗 hint:红斑右移 ~Δ
                    robot_ctrl.laser_blue(False)
                    if bpt is not None:
                        delta = (bpt[0] - spot[0], bpt[1] - spot[1])
                samples.append({"yaw": yv, "pitch": pv,
                                "spot": spot, "delta": delta,
                                "blue_score": bscore})
                log.info(f"[{i + 1}/{len(order)}] ({yv},{pv}) spot={spot}"
                         f" Δ={delta}")

            robot_ctrl.all_lasers_off()
            robot_ctrl.center_servo()
            self._report(samples, k_pairs)
        except Exception as e:
            log.error(f"网格标定异常: {e}")
        finally:
            robot_ctrl.all_lasers_off()
            robot_ctrl.center_servo()
            log.info("标定流程结束,激光已关、云台已归中。Ctrl+C 退出。")

    # ── 拟合与报告 ──────────────────────────────────────────
    def _report(self, samples, k_pairs):
        log = self.get_logger()
        ok = [s for s in samples if s["spot"] is not None]
        if len(ok) < 4:
            log.error(f"有效姿态只有 {len(ok)} 个(<4),无法拟合。"
                      "检查红斑是否在所有姿态下都落在纸上/画面内。")
            return
        u = np.array([s["yaw"] - SERVO_YAW_CENTER for s in ok], float)
        v = np.array([s["pitch"] - SERVO_PITCH_CENTER for s in ok], float)
        sx = np.array([s["spot"][0] for s in ok], float)
        sy = np.array([s["spot"][1] for s in ok], float)

        # ① SPOT_HOME 漂移面
        csx, rms_sx = _bilinear_fit(u, v, sx)
        csy, rms_sy = _bilinear_fit(u, v, sy)
        home_fit = (round(csx[0], 1), round(csy[0], 1))
        drift = float(np.max(np.hypot(sx - csx[0], sy - csy[0])))

        # ② K:像素↔角度(2×2 含耦合)
        K = None
        if len(k_pairs) >= 3:
            A = np.array([[dyw, dpt] for dyw, dpt, _, _ in k_pairs], float)
            B = np.array([[dx, dy] for _, _, dx, dy in k_pairs], float)
            M, *_ = np.linalg.lstsq(A, B, rcond=None)   # dpx = dAng @ M
            M = M.T                                     # 2×2: [dx,dy]=M@[dyaw,dpitch]
            K = M.tolist()
            # 反演成"1 像素 = 多少度"(只取主对角,与 config 同语义)
            k_yaw_deg_per_px = 1.0 / M[0][0] if abs(M[0][0]) > 1e-6 else None
            k_pitch_deg_per_px = 1.0 / M[1][1] if abs(M[1][1]) > 1e-6 else None
        else:
            k_yaw_deg_per_px = k_pitch_deg_per_px = None

        # ③ Δ 面
        dsamp = [s for s in ok if s["delta"] is not None]
        delta_fit = delta_rms = delta_pitch_slope = None
        if len(dsamp) >= 4:
            du = np.array([s["yaw"] - SERVO_YAW_CENTER for s in dsamp], float)
            dv = np.array([s["pitch"] - SERVO_PITCH_CENTER
                           for s in dsamp], float)
            dx = np.array([s["delta"][0] for s in dsamp], float)
            dy = np.array([s["delta"][1] for s in dsamp], float)
            cdx, rdx = _bilinear_fit(du, dv, dx)
            cdy, rdy = _bilinear_fit(du, dv, dy)
            delta_fit = (round(cdx[0], 1), round(cdy[0], 1))
            delta_rms = round(max(rdx, rdy), 1)
            delta_pitch_slope = (round(cdx[2], 2), round(cdy[2], 2))

        # ④ 残差报告:继续用 config 常量,各姿态瞄准点会偏多少
        try:
            _c = load_calib()
            cfg_delta = (_c.delta_x, _c.delta_y)
        except Exception:
            cfg_delta = None
        err_const = [float(np.hypot(s["spot"][0] - SPOT_HOME_X,
                                    s["spot"][1] - SPOT_HOME_Y))
                     for s in ok]
        lines = [
            "", "═" * 62,
            "  网格标定报告 (grid_calibration v3.12.0)",
            "═" * 62,
            f"  有效姿态 {len(ok)}/{len(samples)}   Δ样本 {len(dsamp)}"
            f"   K样本 {len(k_pairs)}",
            "",
            "  ① 红斑基准 SPOT_HOME:",
            f"     config 现值 ({SPOT_HOME_X},{SPOT_HOME_Y})"
            f"  → 拟合中心 {home_fit}",
            f"     全网格漂移: 均值 {np.mean(err_const):.1f}px"
            f"  最大 {max(err_const):.1f}px  (拟合面残差 RMS"
            f" {max(rms_sx, rms_sy):.1f}px)",
        ]
        if k_yaw_deg_per_px is not None:
            lines += [
                "", "  ② 像素↔角度 K:",
                f"     PIXEL_TO_YAW_DEG   config {PIXEL_TO_YAW_DEG:+.3f}"
                f"  → 实测 {k_yaw_deg_per_px:+.4f}",
                f"     PIXEL_TO_PITCH_DEG config {PIXEL_TO_PITCH_DEG:+.3f}"
                f"  → 实测 {k_pitch_deg_per_px:+.4f}",
                f"     轴间耦合(2×2 非对角) dx/dpitch={K[0][1]:.2f}"
                f" dy/dyaw={K[1][0]:.2f} px/°",
            ]
        if delta_fit is not None:
            lines += [
                "", "  ③ 蓝红偏移 Δ:",
                f"     config 现值 {cfg_delta}  → 网格中心拟合 {delta_fit}"
                f"  (面残差 RMS {delta_rms}px)",
                f"     Δ随 pitch 斜率 {delta_pitch_slope} px/°"
                f"  —— |斜率|>1 说明固定 Δ 在俯仰边缘会偏 ≥{GRID_PITCH_SPAN_DEG}px",
            ]
        lines += [
            "", "  ▶ 建议:把上面'实测/拟合'值写回 config.py 后重建;",
            f"    原始数据与拟合系数已存 {GRID_OUT_JSON}(答辩残差图可由此绘制)。",
            "═" * 62,
        ]
        for ln in lines:
            log.info(ln)

        out = {
            "at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "grid_n": GRID_N,
            "samples": samples,
            "fit": {
                "spot_home": {"x_coef": csx, "y_coef": csy,
                              "center": home_fit,
                              "rms": [rms_sx, rms_sy]},
                "K_matrix_px_per_deg": K,
                "pixel_to_yaw_deg": k_yaw_deg_per_px,
                "pixel_to_pitch_deg": k_pitch_deg_per_px,
                "delta_center": delta_fit,
                "delta_pitch_slope": delta_pitch_slope,
            },
            "config_now": {
                "SPOT_HOME": [SPOT_HOME_X, SPOT_HOME_Y],
                "PIXEL_TO_YAW_DEG": PIXEL_TO_YAW_DEG,
                "PIXEL_TO_PITCH_DEG": PIXEL_TO_PITCH_DEG,
                "delta": cfg_delta,
            },
        }
        path = os.path.expanduser(GRID_OUT_JSON)
        try:
            with open(path, "w") as f:
                json.dump(out, f, ensure_ascii=False, indent=1)
            log.info(f"已写 {path}")
        except OSError as e:
            log.error(f"写 {path} 失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GridCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_ctrl.all_lasers_off()
        robot_ctrl.center_servo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
