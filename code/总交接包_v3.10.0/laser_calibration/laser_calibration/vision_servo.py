#!/usr/bin/env python3
"""
vision_servo.py —— Phase 3：视觉伺服闭环打击  v3.9.1
=====================================================
v3.9.1 修复（相对 v3.9.0）：
  * 加 [S3 ON] / [S3 OFF] / [S3 测试烧 0.5s] 手动按钮（用于排查 S3 接线）
  * 加「开环模式也自动开火」复选框（默认禁用，调试期可勾选验证全链路）
  * 加 _fire_test_thread 短时测试不进入伺服流程
  * _fire_sequence 加诊断日志（打印 LASER_BLUE_ID / angle）

v3.9.0 重大变更（相对 v3.8）：
  1. 画面源切到 RGB 摄像头（S4 实际是可见红激光，不是 IR）
  2. 光斑检测从 IR 灰度阈值切到 RGB HSV 红色双区间
  3. 修复 fallback bug：检测不到光斑时不再用 (320,240)，改用 SPOT_HOME 实测值
  4. 修复 PID required_spot 漂移 bug：每帧重新读最新 YOLO target
     （需 YOLO 队友把 publish 改成 10Hz 持续发布；不持续发也能跑，回退到 v3.8 行为）
  5. Shift_X/Y 退役：YOLO 和光斑都在 RGB，不再做跨相机映射
  6. 启动自检：开机检查光斑是否能被检测到、位置是否合理

工作原理（v3.9）：
  1. 订阅 YOLO 的 /yolo/weed_detected （RGB 坐标）
  2. 订阅 RGB 摄像头 /camera/rgb/image_raw
  3. 用标定二的 Delta_X/Y（RGB 坐标系下）反算"红光斑应到位置"
     Required_Spot_RGB = Target_RGB - Delta_RGB
  4. 开环粗对准：按 PIXEL_TO_YAW/PITCH_DEG 比例转一次
  5. PID 闭环精对准：每帧重检测红光斑、重读 YOLO target、重算误差、收敛
  6. 锁定后开 S3 蓝紫激光烧 1 秒

两种触发模式（浏览器开关切换）:
  manual : 浏览器按【开始打击】才执行（默认，调试期安全）
  auto   : YOLO 消息到达就自动触发（带防抖）

两种伺服模式（浏览器单选）:
  open_loop   : 仅开环粗对准，停在 LOCKED 不开火，供调参观察
  closed_loop : 开环 + PID 闭环精对准，收敛后自动开火（主要工作模式）

前置:
  ros2 run laser_calibration stereo_camera

运行:
  ros2 run laser_calibration vision_servo

浏览器:
  http://localhost:8093
  http://<小车IP>:8093
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from laser_calibration.calib_io import load_calib
from laser_calibration.config import (
    FIRE_COOLDOWN_SEC, FIRE_DURATION_SEC,
    LASER_BLUE_ID, LASER_ON_ANGLE,
    PID_KD, PID_KI, PID_KP,
    PID_LOCK_FRAMES, PID_OUTPUT_LIMIT,
    PID_DEADBAND_PX, PID_SATURATION_FRAMES, FSM_TICK_PERIOD_SEC,
    PID_TIMEOUT_SEC, PID_TOLERANCE_PX,
    PIXEL_TO_PITCH_DEG, PIXEL_TO_YAW_DEG,
    RED_DOMINANCE_MIN, RED_SPOT_AREA_MAX, RED_SPOT_AREA_MIN,
    SPOT_CLOSE_KERNEL_SIZE, SPOT_ROI_SIZE,
    SPOT_JUMP_MAX_PX, SPOT_JUMP_TOLERATE_FRAMES,
    SERVO_AUTO_DEBOUNCE, SERVO_DEFAULT_MODE,
    SERVO_PITCH_CENTER, SERVO_YAW_CENTER,
    SPOT_HOME_TOLERANCE, SPOT_HOME_X, SPOT_HOME_Y,
    TOPIC_RGB, TOPIC_YOLO,
    YOLO_FALLBACK_TO_LOCKED, YOLO_TARGET_FRESH_SEC,
)
from laser_calibration.robot_ctrl import (
    ROBOT_OK,
    all_lasers_off, center_servo,
    laser_blue, laser_ir, set_servo,
)

SERVO_HTTP_PORT = 8093

# 状态机
STATE_IDLE       = "IDLE"
STATE_GOT_TARGET = "GOT_TARGET"
STATE_COARSE     = "COARSE"
STATE_PID        = "PID"
STATE_LOCKED     = "LOCKED"
STATE_FIRING     = "FIRING"
STATE_COOLDOWN   = "COOLDOWN"
STATE_FAILED     = "FAILED"


# ══════════════════════════════════════════════════════════════
#  v3.9.5 核心：R-max(G,B) + ROI 红光斑检测
#  — 替换 v3.9.4 的 HSV 方案，专治"过曝白纸 + 不可调曝光"
#
#  算法（来自战友实测验证的伪代码）：
#    1. ROI 裁剪 — 物理隔绝远处干扰
#    2. red_score = R - max(G, B) — 红色相对优势，对环境光自适应
#    3. 形态学闭运算 — 填补过曝中心的"甜甜圈"黑洞
#    4. 取最大轮廓的几何质心
# ══════════════════════════════════════════════════════════════
def find_red_spot(bgr: np.ndarray, hint_x: int = None, hint_y: int = None):
    """红激光光斑检测，返回全图坐标 (cx, cy) 或 None。

    Args:
        bgr: BGR 图像
        hint_x, hint_y: ROI 中心提示（推荐传上一帧 spot 或 SPOT_HOME）
                         传 None 走全图搜索
    """
    if bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]

    # ─ Step 1: ROI 裁剪 ─────────────────────────────────────
    if hint_x is not None and hint_y is not None:
        half = SPOT_ROI_SIZE // 2
        y1 = max(0, int(hint_y) - half)
        y2 = min(h, int(hint_y) + half)
        x1 = max(0, int(hint_x) - half)
        x2 = min(w, int(hint_x) + half)
    else:
        y1, y2, x1, x2 = 0, h, 0, w  # 全图搜索

    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    # ─ Step 2: R-max(G,B) 红色优势 ──────────────────────────
    b, g, r = cv2.split(roi)
    r_i  = r.astype(np.int16)
    g_i  = g.astype(np.int16)
    b_i  = b.astype(np.int16)
    max_gb = np.maximum(g_i, b_i)
    red_score = np.clip(r_i - max_gb, 0, 255).astype(np.uint8)

    _, mask = cv2.threshold(red_score, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)

    # ─ Step 3: 形态学闭运算填补过曝中心 ────────────────────
    kernel = np.ones((SPOT_CLOSE_KERNEL_SIZE, SPOT_CLOSE_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # ─ Step 4: 找最大轮廓的几何质心 ────────────────────────
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    largest = max(cnts, key=cv2.contourArea)
    a = cv2.contourArea(largest)
    if not (RED_SPOT_AREA_MIN < a < RED_SPOT_AREA_MAX):
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])

    # 还原到全图坐标
    return (x1 + cx_local, y1 + cy_local)


# ══════════════════════════════════════════════════════════════
#  PID 控制器
# ══════════════════════════════════════════════════════════════
class PIDController:
    """单轴 PID。输入误差（像素），输出舵机角度增量（度）。

    v3.9.6: dt > 0.5s 时 reset（防丢帧积分爆炸）
    v3.9.9: deadband + saturated flag（抗振荡）
    """
    STALE_THRESHOLD_SEC = 0.5

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float,
                 deadband: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.deadband = deadband
        self.saturated = False  # v3.9.9: 本次 step 输出是否被 LIMIT 截断
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.saturated = False

    def step(self, error: float) -> float:
        now = time.time()
        if self.last_time is None:
            dt = 0.1
        else:
            dt_raw = now - self.last_time
            if dt_raw > self.STALE_THRESHOLD_SEC:
                # v3.9.6: 不连续 → reset
                self.integral = 0.0
                self.last_error = error
                dt = 0.1
            else:
                dt = max(1e-3, dt_raw)
        self.last_time = now

        self.integral += error * dt
        self.integral = max(-100.0, min(100.0, self.integral))
        derivative = (error - self.last_error) / dt
        self.last_error = error

        # v3.9.9: 死区内 P 项减半，防止收敛区震荡
        kp_eff = self.kp * (0.5 if abs(error) < self.deadband else 1.0)
        out = kp_eff * error + self.ki * self.integral + self.kd * derivative

        # v3.9.9: 限幅 + 饱和标志（外层用于检测系统跟不上）
        if abs(out) >= self.output_limit:
            self.saturated = True
            out = max(-self.output_limit, min(self.output_limit, out))
        else:
            self.saturated = False
        return out


# ══════════════════════════════════════════════════════════════
#  HTML 页面
# ══════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>视觉伺服 · Phase 3 (v3.9.1)</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:14px; }
  h1 { color:#0f0; margin:0 0 10px; font-size:18px; }
  .panel { background:#222; padding:10px; border-radius:6px; margin-bottom:10px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px;
         margin-bottom:4px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .btn.active { background:#0f0; color:#000; }
  .btn-fire { background:#3a1a1a; color:#f55; border-color:#f55; }
  .btn-fire:hover { background:#f55; color:#000; }
  .btn-stop { background:#3a0808; color:#ff0; border-color:#ff0; font-weight:bold; }
  .btn-stop:hover { background:#ff0; color:#000; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; width:640px; height:480px; }
  .cam-label { position:absolute; top:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#0f0; font-size:12px; z-index:10;
               pointer-events:none; }
  canvas { display:block; }
  .right-panel { width:480px; }
  .stat-box { background:#111; padding:8px; border-radius:4px;
              margin-bottom:6px; font-size:12px; }
  .stat-row { display:flex; justify-content:space-between; margin:2px 0; }
  .stat-key { color:#888; }
  .stat-val { color:#0f0; font-weight:bold; }
  .laser-state { display:inline-block; padding:3px 10px; border-radius:3px;
                 margin-right:8px; font-size:12px; border:1px solid #555; }
  .l-on { background:#330; color:#ff0; border-color:#ff0; }
  .l-fire { background:#600; color:#ff8; border-color:#f55;
            animation:blink 0.4s linear infinite; }
  @keyframes blink { 50% { opacity:0.4; } }
  .pid-chart { background:#0a0a0a; border:1px solid #333; }
  .group-title { color:#0f0; margin-bottom:4px; font-weight:bold; }
  .pid-input { background:#222; color:#0f0; border:1px solid #555; padding:2px 6px;
               width:60px; font-family:monospace; }
  .yolo-fresh-on  { color:#0f0; }
  .yolo-fresh-off { color:#fa0; }
  .info { color:#888; font-size:12px; margin-top:6px; }
</style>
</head>
<body>
<h1>🎯 视觉伺服 · Phase 3 v3.9.1 (RGB+HSV · S3 调试支持)</h1>

<div id="calib-banner" style="display:none; background:#600; color:#fff; padding:10px 14px;
     border:2px solid #f55; border-radius:4px; margin-bottom:10px;
     font-weight:bold; font-size:14px;">
  <span id="calib-banner-text"></span>
</div>

<div class="panel">
  <span style="color:#888">触发:</span>
  <button class="btn" id="trig-manual" onclick="setTrigger('manual')">手动 Manual</button>
  <button class="btn" id="trig-auto" onclick="setTrigger('auto')">自动 Auto</button>

  <span style="margin-left:20px;color:#888">伺服模式:</span>
  <button class="btn" id="loop-open" onclick="setLoopMode('open_loop')">开环 (调试)</button>
  <button class="btn" id="loop-closed" onclick="setLoopMode('closed_loop')">闭环 PID</button>

  <button class="btn btn-fire" id="btn-go" onclick="action('go')" style="margin-left:30px">[开始打击]</button>
  <button class="btn btn-stop" onclick="action('stop')">[紧急停止]</button>
  <button class="btn" onclick="action('center')">[云台归中]</button>
  <button class="btn" id="btn-ir-on"  onclick="manualIR(true)" style="margin-left:10px">[S4 ON]</button>
  <button class="btn" id="btn-ir-off" onclick="manualIR(false)">[S4 OFF]</button>
  <button class="btn btn-fire" id="btn-blue-on" onclick="manualBlue(true)" style="margin-left:8px">[S3 ON]</button>
  <button class="btn btn-fire" id="btn-blue-off" onclick="manualBlue(false)">[S3 OFF]</button>
  <button class="btn btn-fire" onclick="testFire()">[S3 测试烧 0.5s]</button>
</div>

<div class="panel" style="font-size:12px">
  <label style="color:#ccc; cursor:pointer">
    <input type="checkbox" id="fire-open-cb" onchange="toggleFireOpen()" style="vertical-align:middle">
    开环模式也自动开火 <span style="color:#888">(默认禁用，调试期勾选可烧白纸)</span>
  </label>
</div>

<div class="row">
  <div class="cam-box">
    <span class="cam-label">RGB 画面 (红光斑+目标 可视化)</span>
    <canvas id="cv" width="640" height="480"></canvas>

      <!-- YOLO Detection Frequency Control -->
      <div style="margin:15px 0; padding:12px; background:#f0f8ff; border-radius:8px; border:1px solid #b0d4f1;">
        <label style="font-weight:bold; color:#0066cc; display:block; margin-bottom:8px;">📡 YOLO Detection Frequency:</label>
        <div style="display:flex; align-items:center; gap:10px;">
          <input type="range" id="freqSlider" min="1" max="30" value="10" 
                 oninput="document.getElementById('freqValue').textContent=this.value+' Hz'" 
                 onchange="setPublishFreq(this.value)"
                 style="flex:1; height:6px;">
          <span id="freqValue" style="font-weight:bold; color:#0066cc; font-size:18px; min-width:60px;">10 Hz</span>
        </div>
        <div style="font-size:11px; color:#666; margin-top:6px;">
          Range: 1-30 Hz | Current: <span id="currentFreq">10</span> Hz
        </div>
      </div>

  </div>

  <div class="right-panel">
    <div class="stat-box">
      <div class="group-title">系统状态</div>
      <div class="stat-row"><span class="stat-key">触发模式</span><span class="stat-val" id="s-trigger">--</span></div>
      <div class="stat-row"><span class="stat-key">伺服模式</span><span class="stat-val" id="s-loop">--</span></div>
      <div class="stat-row"><span class="stat-key">FSM 状态</span><span class="stat-val" id="s-state">--</span></div>
      <div style="margin-top:6px">
        <span class="laser-state" id="ls-ir">S4 RED: OFF</span>
        <span class="laser-state" id="ls-blue">S3 BLUE: OFF</span>
      </div>
    </div>

    <div class="stat-box">
      <div class="group-title">坐标 / 误差 (全 RGB 像素)</div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#3af">●</span> YOLO 目标</span>
        <span class="stat-val" id="s-yolo">--</span>
      </div>
      <div class="stat-row"><span class="stat-key">YOLO 新鲜度</span><span class="stat-val" id="s-yolo-fresh">--</span></div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#ff0">○</span> 红光斑实测</span>
        <span class="stat-val" id="s-spot">--</span>
      </div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#c4f">✚</span> 蓝紫预测落点</span>
        <span class="stat-val" id="s-predicted">--</span>
      </div>
      <div class="stat-row"><span class="stat-key">PID 误差 (Px, Py)</span><span class="stat-val" id="s-error">--</span></div>
      <div class="stat-row"><span class="stat-key">蓝紫→目标 偏差</span><span class="stat-val" id="s-hit-error">--</span></div>
      <div class="stat-row"><span class="stat-key">连续锁定帧数</span><span class="stat-val" id="s-lock">0/0</span></div>
    </div>

    <div class="stat-box">
      <div class="group-title">舵机角度</div>
      <div class="stat-row"><span class="stat-key">S1 Yaw</span><span class="stat-val" id="s-yaw">--</span></div>
      <div class="stat-row"><span class="stat-key">S2 Pitch</span><span class="stat-val" id="s-pitch">--</span></div>
    </div>

    <div class="stat-box">
      <div class="group-title">PID 调参 (实时生效)</div>
      <div class="stat-row">
        <span class="stat-key">Kp</span>
        <input type="number" step="0.001" class="pid-input" id="pid-kp" value="0.05" onchange="updatePID()">
      </div>
      <div class="stat-row">
        <span class="stat-key">Ki</span>
        <input type="number" step="0.0001" class="pid-input" id="pid-ki" value="0.001" onchange="updatePID()">
      </div>
      <div class="stat-row">
        <span class="stat-key">Kd</span>
        <input type="number" step="0.001" class="pid-input" id="pid-kd" value="0.02" onchange="updatePID()">
      </div>
    </div>

    <div class="stat-box">
      <div class="group-title">误差曲线 (实时)</div>
      <canvas id="chart" width="450" height="160" class="pid-chart"></canvas>
      <div class="info">
        <span style="color:#0f0">●</span> Px (X 误差)
        <span style="color:#fa0;margin-left:14px">●</span> Py (Y 误差)
      </div>
    </div>
  </div>
</div>

<script>
const canvas = document.getElementById('cv');
const ctx = canvas.getContext('2d');
const liveImg = new Image();
let lastState = {};

function liveLoop() {
  liveImg.onload = () => {
    ctx.drawImage(liveImg, 0, 0, 640, 480);
    drawOverlays();
    setTimeout(liveLoop, 80);
  };
  liveImg.onerror = () => setTimeout(liveLoop, 300);
  liveImg.src = `/frame?t=${Date.now()}`;
}
liveLoop();

function drawOverlays() {
  if (!lastState) return;
  if (lastState.yolo) {
    ctx.strokeStyle = '#3af'; ctx.lineWidth = 2;
    ctx.strokeRect(lastState.yolo.x - 18, lastState.yolo.y - 18, 36, 36);
    ctx.fillStyle = '#3af'; ctx.font = '12px monospace';
    ctx.fillText(`YOLO(${lastState.yolo.x},${lastState.yolo.y})`,
                 lastState.yolo.x + 22, lastState.yolo.y - 22);
  }
  // v3.9.9: 去掉绿色"应到位置"十字（调试期辅助，逻辑已验证不再需要）
  if (lastState.spot) {
    ctx.strokeStyle = '#ff0'; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(lastState.spot.x, lastState.spot.y, 12, 0, Math.PI*2);
    ctx.stroke();
    ctx.fillStyle = '#ff0'; ctx.font = '12px monospace';
    ctx.fillText(`红光斑(${lastState.spot.x},${lastState.spot.y})`,
                 lastState.spot.x + 16, lastState.spot.y);
  }
  // Draw YOLO detection boxes
  if (lastState.yolo_boxes && lastState.yolo_boxes.length > 0) {
    lastState.yolo_boxes.forEach(function(box, idx) {
      const cx = box.cx || 0;
      const cy = box.cy || 0;
      const w = box.w || 50;
      const h = box.h || 50;
      const x1 = cx - w/2;
      const y1 = cy - h/2;
      
      // Draw rectangle
      ctx.strokeStyle = '#0ff';  // Cyan color
      ctx.lineWidth = 2;
      ctx.strokeRect(x1, y1, w, h);
      
      // Draw label
      const label = box.label || 'weed';
      const conf = box.confidence ? box.confidence.toFixed(2) : '?';
      ctx.fillStyle = '#0ff';
      ctx.font = 'bold 12px monospace';
      ctx.fillText(`${label} ${conf}`, x1, Math.max(y1 - 4, 12));
    });
  }

  // v3.9.3: 蓝紫激光预测落点 (current_spot + Delta) — 紫色十字
  if (lastState.predicted_hit) {
    drawCross(lastState.predicted_hit.x, lastState.predicted_hit.y, '#c4f',
              `蓝紫(${lastState.predicted_hit.x},${lastState.predicted_hit.y})`);
    // 红光斑 → 蓝紫预测落点 的实线（直观显示 Delta 偏移向量）
    if (lastState.spot) {
      ctx.strokeStyle = '#c4f'; ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(lastState.spot.x, lastState.spot.y);
      ctx.lineTo(lastState.predicted_hit.x, lastState.predicted_hit.y);
      ctx.stroke();
    }
  }
  // v3.9.9: 去掉红光斑↔应到位置的虚线
}

function drawCross(x, y, color, label) {
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x-20, y); ctx.lineTo(x+20, y);
  ctx.moveTo(x, y-20); ctx.lineTo(x, y+20);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(x, y, 14, 0, Math.PI*2); ctx.stroke();
  if (label) {
    ctx.fillStyle = color; ctx.font = '12px monospace';
    ctx.fillText(label, x + 18, y - 12);
  }
}

const chart = document.getElementById('chart');
const cctx = chart.getContext('2d');
let history_px = [], history_py = [];
const HISTORY = 80;

function drawChart() {
  cctx.fillStyle = '#0a0a0a';
  cctx.fillRect(0, 0, chart.width, chart.height);
  cctx.strokeStyle = '#333';
  cctx.beginPath();
  cctx.moveTo(0, chart.height/2);
  cctx.lineTo(chart.width, chart.height/2);
  cctx.stroke();
  cctx.strokeStyle = '#222';
  cctx.setLineDash([2,4]);
  const tol_h = chart.height/2 - 6;
  cctx.beginPath();
  cctx.moveTo(0, tol_h); cctx.lineTo(chart.width, tol_h);
  cctx.moveTo(0, chart.height - tol_h);
  cctx.lineTo(chart.width, chart.height - tol_h);
  cctx.stroke();
  cctx.setLineDash([]);
  drawSeries(history_px, '#0f0');
  drawSeries(history_py, '#fa0');
}

function drawSeries(arr, color) {
  if (arr.length < 2) return;
  cctx.strokeStyle = color;
  cctx.lineWidth = 1.5;
  cctx.beginPath();
  const scale = chart.height / 2 / 50;
  for (let i = 0; i < arr.length; i++) {
    const x = (i / HISTORY) * chart.width;
    const y = chart.height/2 - arr[i] * scale;
    if (i === 0) cctx.moveTo(x, y); else cctx.lineTo(x, y);
  }
  cctx.stroke();
}

function refreshState() {
  fetch('/api/state').then(r => r.json()).then(d => {
    lastState = d;
    document.getElementById('s-trigger').textContent = d.trigger || '--';
    document.getElementById('s-loop').textContent = d.loop_mode || '--';
    document.getElementById('s-state').textContent = d.fsm_state || '--';
    document.getElementById('s-state').style.color =
      ({'IDLE':'#888','GOT_TARGET':'#fa0','COARSE':'#fa0','PID':'#fa0',
        'LOCKED':'#0f0','FIRING':'#f55','COOLDOWN':'#888','FAILED':'#f00'})[d.fsm_state] || '#0f0';

    document.getElementById('s-yolo').textContent = d.yolo ? `(${d.yolo.x},${d.yolo.y})` : '--';
    const freshEl = document.getElementById('s-yolo-fresh');
    if (d.yolo_fresh === true) {
      freshEl.textContent = `新鲜 (${d.yolo_age_sec.toFixed(2)}s 前)`;
      freshEl.className = 'stat-val yolo-fresh-on';
    } else if (d.yolo_age_sec !== undefined && d.yolo_age_sec !== null) {
      freshEl.textContent = `过期 (${d.yolo_age_sec.toFixed(2)}s 前)`;
      freshEl.className = 'stat-val yolo-fresh-off';
    } else {
      freshEl.textContent = '无数据';
      freshEl.className = 'stat-val';
    }
    // v3.9.9: 移除 s-required 字段
    document.getElementById('s-spot').textContent = d.spot ? `(${d.spot.x},${d.spot.y})` : '--';
    document.getElementById('s-predicted').textContent = d.predicted_hit ? `(${d.predicted_hit.x},${d.predicted_hit.y})` : '--';
    // 蓝紫预测落点 vs YOLO 目标 的偏差（直观显示打击精度）
    if (d.predicted_hit && d.yolo) {
      const hex = d.predicted_hit.x - d.yolo.x;
      const hey = d.predicted_hit.y - d.yolo.y;
      const dist = Math.sqrt(hex*hex + hey*hey);
      document.getElementById('s-hit-error').textContent =
        `(${hex>=0?'+':''}${hex},${hey>=0?'+':''}${hey})  d=${dist.toFixed(1)}px`;
    } else {
      document.getElementById('s-hit-error').textContent = '--';
    }
    document.getElementById('s-error').textContent =
      d.error ? `(${d.error.x>=0?'+':''}${d.error.x},${d.error.y>=0?'+':''}${d.error.y})` : '--';
    document.getElementById('s-lock').textContent = `${d.lock_frames || 0}/${d.lock_target || 5}`;
    document.getElementById('s-yaw').textContent = d.yaw !== undefined ? d.yaw.toFixed(1) + '°' : '--';
    document.getElementById('s-pitch').textContent = d.pitch !== undefined ? d.pitch.toFixed(1) + '°' : '--';

    document.getElementById('trig-manual').className = 'btn' + (d.trigger === 'manual' ? ' active' : '');
    document.getElementById('trig-auto').className = 'btn' + (d.trigger === 'auto' ? ' active' : '');
    document.getElementById('loop-open').className = 'btn' + (d.loop_mode === 'open_loop' ? ' active' : '');
    document.getElementById('loop-closed').className = 'btn' + (d.loop_mode === 'closed_loop' ? ' active' : '');

    setLaser('ls-ir',   'S4 RED', d.laser_ir);
    setLaser('ls-blue', 'S3 BLUE', d.laser_blue);

    // 同步「开环也开火」复选框
    if (d.fire_in_open_loop !== undefined) {
      const cb = document.getElementById('fire-open-cb');
      if (cb && cb.checked !== d.fire_in_open_loop) cb.checked = d.fire_in_open_loop;
    }

    // v3.9.1 标定二状态横幅
    const banner = document.getElementById('calib-banner');
    const bannerText = document.getElementById('calib-banner-text');
    if (d.calib2_done === false) {
      banner.style.display = 'block';
      banner.style.background = '#604000';
      banner.style.borderColor = '#fa0';
      bannerText.innerHTML = '⚠️ 标定二未完成 — 请先关闭本节点，运行 <code>ros2 run laser_calibration calib_laser</code> 完成标定。';
    } else if (d.calib2_stale === true) {
      banner.style.display = 'block';
      banner.style.background = '#600';
      banner.style.borderColor = '#f55';
      bannerText.innerHTML =
        `⛔ 标定二坐标系不匹配 — Δ=(${d.delta_x>=0?'+':''}${d.delta_x},${d.delta_y>=0?'+':''}${d.delta_y}) ` +
        `是 ${d.calib2_frame || 'IR/历史'} 坐标系，本节点是 RGB 坐标系。` +
        ' 用错值跑伺服会偏几十像素。请重做 <code>calib_laser</code>。';
    } else {
      banner.style.display = 'none';
    }

    if (d.error) {
      history_px.push(d.error.x);
      history_py.push(d.error.y);
      if (history_px.length > HISTORY) { history_px.shift(); history_py.shift(); }
    }
    drawChart();
  }).catch(() => {});
}

function setLaser(id, name, st) {
  const el = document.getElementById(id);
  el.className = 'laser-state' + (st === 'fire' ? ' l-fire' : (st === 'on' ? ' l-on' : ''));
  el.textContent = `${name}: ${(st || 'off').toUpperCase()}`;
}

setInterval(refreshState, 100);

function setTrigger(m) { fetch('/api/trigger?m=' + m).then(refreshState); }
function setLoopMode(m) { fetch('/api/loop?m=' + m).then(refreshState); }
function action(act) { fetch('/api/' + act).then(refreshState); }
function manualIR(on) { fetch('/api/laser_ir?on=' + (on ? '1' : '0')).then(refreshState); }
function manualBlue(on) {
  const msg = on
    ? '⚠️ 即将开启 S3 蓝紫激光（持续，直到点 [S3 OFF]）！\n请确认:\n· 云台已对准白纸/防火垫\n· 激光路径无人/无易燃物\n· 已戴防护眼镜\n· 旁边备好灭火工具'
    : null;
  if (on && !confirm(msg)) return;
  fetch('/api/laser_blue?on=' + (on ? '1' : '0')).then(refreshState);
}
function testFire() {
  if (!confirm('⚠️ 即将开启 S3 蓝紫激光烧 0.5 秒（测试 S3 接口/接线）！\n请确认:\n· 云台已对准白纸/防火垫\n· 激光路径无人/无易燃物\n· 已戴防护眼镜')) {
    return;
  }
  fetch('/api/fire_test?dur=0.5').then(refreshState);
}
function toggleFireOpen() {
  const cb = document.getElementById('fire-open-cb');
  if (cb.checked) {
    if (!confirm('⚠️ 启用「开环模式自动开火」后，开环测试每次到达 LOCKED 状态都会自动烧 1 秒！\n仅在你能控制目标位置(放好白纸)、人员安全的情况下启用。\n确认启用？')) {
      cb.checked = false;
      return;
    }
  }
  fetch('/api/fire_open_toggle?on=' + (cb.checked ? '1' : '0')).then(refreshState);
}
function updatePID() {
  const kp = document.getElementById('pid-kp').value;
  const ki = document.getElementById('pid-ki').value;
  const kd = document.getElementById('pid-kd').value;
  fetch(`/api/pid?kp=${kp}&ki=${ki}&kd=${kd}`).then(refreshState);
}

fetch('/api/state').then(r => r.json()).then(d => {
  if (d.kp !== undefined) document.getElementById('pid-kp').value = d.kp;
  if (d.ki !== undefined) document.getElementById('pid-ki').value = d.ki;
  if (d.kd !== undefined) document.getElementById('pid-kd').value = d.kd;
});

refreshState();

// YOLO publish frequency control
let currentPublishFreq = 10;

function setPublishFreq(freq) {
  freq = parseFloat(freq);
  if (freq < 1 || freq > 30) {
    alert('Frequency must be between 1-30 Hz');
    return;
  }
  
  fetch('/api/set_yolo_freq?freq=' + freq)
    .then(response => response.json())
    .then(data => {
      if (data.success) {
        currentPublishFreq = freq;
        document.getElementById('currentFreq').textContent = freq;
        console.log('✅ Frequency set to:', freq, 'Hz');
      } else {
        alert('Failed: ' + data.message);
        document.getElementById('freqSlider').value = currentPublishFreq;
        document.getElementById('freqValue').textContent = currentPublishFreq + ' Hz';
      }
    })
    .catch(err => {
      console.error('❌ Request failed:', err);
      alert('Network error');
    });
}


</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class VisionServoNode(Node):

    def __init__(self):
        super().__init__("vision_servo")
        self.bridge = CvBridge()
        self.calib = load_calib()

        # 系统状态
        self.trigger_mode = SERVO_DEFAULT_MODE
        self.loop_mode    = "closed_loop"
        self.fsm_state    = STATE_IDLE
        # v3.9.1: 开环模式默认不自动开火（调试安全），但可由 UI 勾选打开
        self.fire_in_open_loop = False

        # 实时数据
        self._rgb_frame = None
        self._lock = threading.Lock()

        # YOLO 目标（v3.9: 持续刷新，带时间戳）
        self.yolo_target = None
        self.yolo_target_at = 0.0
        # Cache all YOLO boxes for visualization
        self._yolo_boxes = []
        # PID 启动时锁定的首帧 target（YOLO 不持续发布时的 fallback 用）
        self._locked_yolo_target = None

        self.required_spot = None
        self.current_spot = None
        self.error = None
        self.lock_count = 0

        # v3.9.4: 帧间稳定性追踪
        self._last_valid_spot = None    # (x, y) 上一次稳定接受的光斑
        self._spot_jump_count = 0       # 连续大跳变帧数

        # v3.9.5: 防日志泛滥 — LOCKED "停留" 信息只打一次
        self._locked_log_done = False
        self.servo_yaw   = SERVO_YAW_CENTER
        self.servo_pitch = SERVO_PITCH_CENTER
        self.laser_ir_state   = "off"
        self.laser_blue_state = "off"

        # v3.9.1: 调参模式 — 对准后不开火，停在 LOCKED 供观察
        # 默认 False（开火）；调试 PIXEL_TO_YAW/PITCH_DEG 时勾选 True 不烧白纸
        self.dry_run = False

        self._pid_started_at = 0.0
        self._locked_at = 0.0

        # PID
        self.kp = PID_KP
        self.ki = PID_KI
        self.kd = PID_KD
        self.pid_x = PIDController(self.kp, self.ki, self.kd, PID_OUTPUT_LIMIT,
                                    deadband=PID_DEADBAND_PX)
        self.pid_y = PIDController(self.kp, self.ki, self.kd, PID_OUTPUT_LIMIT,
                                    deadband=PID_DEADBAND_PX)
        # v3.9.9 新增：闭环状态追踪
        self._pid_saturation_count = 0       # 连续输出饱和帧数
        self._pid_actively_moving = False    # PID 当前正在主动驱动舵机

        # 自动模式防抖
        self._auto_seen_at = 0.0
        self._auto_last_tx = None

        # 订阅（v3.9：从 IR 切到 RGB）
        self.sub_rgb  = self.create_subscription(Image,  TOPIC_RGB,  self._cb_rgb,  10)
        self.sub_yolo = self.create_subscription(String, TOPIC_YOLO, self._cb_yolo, 10)

        all_lasers_off()
        center_servo()
        time.sleep(0.3)

        self._start_http()
        self.timer = self.create_timer(FSM_TICK_PERIOD_SEC, self._fsm_step)

        log = self.get_logger().info
        log("═══════════════════════════════════════════════════════")
        log("  视觉伺服节点  v3.9.1 (RGB+HSV+S3 调试)")
        log("═══════════════════════════════════════════════════════")
        log(f"  SDK 状态:   {'✅ 已连接' if ROBOT_OK else '❌ 未连接（仅模拟）'}")
        log(f"  画面源:     RGB ({TOPIC_RGB})")
        log(f"  本机访问:   http://localhost:{SERVO_HTTP_PORT}")
        log(f"  远程访问:   http://<小车IP>:{SERVO_HTTP_PORT}")
        log("  ─────────────────────────────────────")
        log("  标定状态（v3.9 仅依赖标定二）:")
        log(f"    Delta_X={self.calib.delta_x:+d}  Delta_Y={self.calib.delta_y:+d}  "
            f"{'✅' if self.calib.calib2_done else '❌ 未完成'}  "
            f"frame={self.calib.calib2_frame or '<未标记>'}")
        log(f"    SPOT_HOME=({SPOT_HOME_X},{SPOT_HOME_Y})  容差={SPOT_HOME_TOLERANCE}px")
        log(f"  PIXEL_TO_YAW_DEG={PIXEL_TO_YAW_DEG}  PIXEL_TO_PITCH_DEG={PIXEL_TO_PITCH_DEG}")
        log("  按 Ctrl+C 退出（自动关闭所有激光）")
        log("═══════════════════════════════════════════════════════")

        # v3.9.1 关键检查：标定二的坐标系是否匹配当前 vision_servo 的画面源
        # 必须匹配，否则 Delta 偏移会跟实际坐标系错位
        self.calib2_stale = False  # 标定二是否疑似过期/坐标系不匹配
        if self.calib.calib2_done:
            if self.calib.calib2_frame != "rgb":
                self.calib2_stale = True
                warn = self.get_logger().warn
                warn("═══════════════════════════════════════════════════════")
                warn("  ⛔ 严重警告：标定二数据坐标系不匹配！")
                warn("═══════════════════════════════════════════════════════")
                warn(f"  当前 vision_servo: RGB 摄像头画面下工作")
                warn(f"  标定二数据坐标系:  {self.calib.calib2_frame or 'IR (推断 — v3.8 历史数据)'}")
                warn(f"  当前 Delta=({self.calib.delta_x:+d},{self.calib.delta_y:+d})")
                warn(f"  → 这是 IR 像素坐标下的偏移，不能用于 RGB 视觉伺服")
                warn(f"  → 用错误 Delta 跑伺服，云台会朝错误方向偏移很多像素")
                warn("  ─────────────────────────────────────")
                warn("  ✅ 解决办法：")
                warn("     1. 关闭本节点（Ctrl+C）")
                warn("     2. ros2 run laser_calibration calib_laser  # 用白纸重做")
                warn("     3. 重新启动本节点")
                warn("  ─────────────────────────────────────")
                warn("  ⚠️ 如执意带错误 Delta 启动，HTML 顶部会持续显示红色横幅")
                warn("═══════════════════════════════════════════════════════")
        else:
            self.get_logger().warn(
                "⚠️ 标定二未完成。"
                "请运行 ros2 run laser_calibration calib_laser  做完再来跑伺服。"
            )

        threading.Thread(target=self._self_check_after_start, daemon=True).start()

    # ── 启动自检 ─────────────────────────────────────────────
    def _self_check_after_start(self):
        time.sleep(3.0)
        log = self.get_logger().info
        warn = self.get_logger().warn

        log("───── 启动自检：检测红光斑可见性 ─────")
        was_on = (self.laser_ir_state == "on")
        if not was_on:
            self._set_ir_laser(True)
            time.sleep(0.5)

        rgb = self._get_rgb()
        if rgb is None:
            warn("⚠️ RGB 画面未到达，跳过自检。请确认 stereo_camera 正在运行。")
            if not was_on:
                self._set_ir_laser(False)
            return

        spot = find_red_spot(rgb)   # 全图搜索（自检时不知道 hint）
        if spot is None:
            warn("⚠️ 自检：归中状态下检测不到红光斑！")
            warn("   可能原因：")
            warn("   1. S4 红激光实际未点亮（电源/接线问题）")
            warn("   2. RGB 摄像头视野里没有光斑（指向太偏）")
            warn("   3. R-max(G,B) 阈值偏高 → 调小 RED_DOMINANCE_MIN（默认 30）")
            warn("   4. 摄像头白平衡漂了导致红光斑变粉/紫")
        else:
            sx, sy = spot
            dist = ((sx - SPOT_HOME_X)**2 + (sy - SPOT_HOME_Y)**2) ** 0.5
            log(f"✅ 自检：红光斑实测位置 ({sx},{sy})")
            log(f"   配置 SPOT_HOME=({SPOT_HOME_X},{SPOT_HOME_Y})  距离={dist:.0f}px")
            if dist > SPOT_HOME_TOLERANCE:
                warn(f"⚠️ 实测位置与 SPOT_HOME 偏离 {dist:.0f}px > {SPOT_HOME_TOLERANCE}px")
                warn(f"   建议把 config.py 里 SPOT_HOME_X/Y 改成 ({sx},{sy}) 减少 fallback 误差")

        if not was_on:
            self._set_ir_laser(False)
        log("───── 自检结束 ─────")

    # ── 回调 ─────────────────────────────────────────────────
    def _cb_rgb(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._rgb_frame = f
        except Exception as e:
            self.get_logger().error(f"RGB 解码失败：{e}")

    def _cb_yolo(self, msg):
        try:
            data = json.loads(msg.data)
            detected = data.get("detected", False)
            if not detected:
                return  # 队友持续 publish 时也会发 detected=False 的"心跳"
            boxes = data.get("boxes")
            if boxes:
                cx_target, cy_target = 320, 240
                best = min(boxes,
                    key=lambda b: (b.get("cx", 0) - cx_target)**2 +
                                  (b.get("cy", 0) - cy_target)**2)
                tx, ty = int(best["cx"]), int(best["cy"])
            else:
                tx = int(data.get("cx", 0))
                ty = int(data.get("cy", 0))

            self.yolo_target = {"x": tx, "y": ty}
            self.yolo_target_at = time.time()

            # Cache all boxes for visualization
            if boxes:
                self._yolo_boxes = boxes
            else:
                self._yolo_boxes = []

            if self.trigger_mode == "auto" and self.fsm_state == STATE_IDLE:
                now = time.time()
                if (self._auto_last_tx is None or
                    abs(self._auto_last_tx[0] - tx) > 30 or
                    abs(self._auto_last_tx[1] - ty) > 30):
                    self._auto_seen_at = now
                    self._auto_last_tx = (tx, ty)
                elif now - self._auto_seen_at >= SERVO_AUTO_DEBOUNCE:
                    self.get_logger().info(f"[AUTO] 触发: 目标({tx},{ty}) 稳定 {SERVO_AUTO_DEBOUNCE}s")
                    self._start_servo()
                    self._auto_seen_at = now + 999
        except Exception as e:
            self.get_logger().error(f"YOLO 消息解析失败：{e}")

    def _get_rgb(self):
        with self._lock:
            return None if self._rgb_frame is None else self._rgb_frame.copy()

    # ── 启动伺服 ─────────────────────────────────────────────
    def _start_servo(self):
        if self.yolo_target is None:
            self.get_logger().warn("无 YOLO 目标，无法启动")
            return False
        if self.fsm_state not in (STATE_IDLE, STATE_GOT_TARGET, STATE_FAILED):
            self.get_logger().warn(f"当前状态 {self.fsm_state}，请等待完成")
            return False

        rgb_x, rgb_y = self.yolo_target["x"], self.yolo_target["y"]
        self._locked_yolo_target = {"x": rgb_x, "y": rgb_y}

        req_x, req_y = self.calib.target_to_required_spot(rgb_x, rgb_y)
        req_x = max(0, min(639, req_x))
        req_y = max(0, min(479, req_y))
        self.required_spot = {"x": req_x, "y": req_y}

        self.lock_count = 0
        self.pid_x.reset()
        self.pid_y.reset()
        self.fsm_state = STATE_COARSE
        self.get_logger().info(
            f"启动伺服: YOLO({rgb_x},{rgb_y}) → 应到光斑({req_x},{req_y}) "
            f"[Delta={self.calib.delta_x:+d},{self.calib.delta_y:+d}]"
        )
        return True

    # ── 舵机控制 ─────────────────────────────────────────────
    def _set_yaw_pitch(self, yaw: float, pitch: float):
        self.servo_yaw   = max(45, min(135, yaw))
        self.servo_pitch = max(60, min(120, pitch))
        set_servo(int(round(self.servo_yaw)), int(round(self.servo_pitch)))

    def _set_ir_laser(self, on: bool):
        laser_ir(on)
        self.laser_ir_state = "on" if on else "off"

    def _set_blue_laser(self, on: bool, fire: bool = False):
        laser_blue(on)
        self.laser_blue_state = ("fire" if fire else "on") if on else "off"

    # ── 实时光斑检测 ─────────────────────────────────────────
    def _detect_spot_now(self):
        """v3.9.5: ROI 模式 — 用上一帧 spot 或 SPOT_HOME 作为搜索中心
                   ROI 内找不到时自动退化到全图搜索
        v3.9.9: PID 主动转动时禁用 jump 抑制（已知视场在变，光斑位置正常会大跳）
        """
        if self.laser_ir_state != "on":
            self.current_spot = None
            self._last_valid_spot = None
            self._spot_jump_count = 0
            return None
        rgb = self._get_rgb()
        if rgb is None:
            return None

        # 提示中心：优先上一帧位置，其次 SPOT_HOME
        if self._last_valid_spot is not None:
            hint_x, hint_y = self._last_valid_spot
        else:
            hint_x, hint_y = SPOT_HOME_X, SPOT_HOME_Y

        # 优先 ROI 检测
        raw_spot = find_red_spot(rgb, hint_x, hint_y)
        if raw_spot is None:
            # ROI 没找到 → 退化到全图搜索
            raw_spot = find_red_spot(rgb, None, None)
            if raw_spot is None:
                return None

        # v3.9.9 关键修复：PID 主动转动时禁用 jump 抑制
        # 原因：转动 5° → 视场偏移 → 光斑像素跳 50-100px 是正常的！
        # 旧逻辑会抑制这种"正常跳变" → last_valid_spot 滞后 → PID 用过时数据 → 死锁
        if self._pid_actively_moving:
            # 直接信任新位置，不做帧间稳定性检查
            self._spot_jump_count = 0
            self._last_valid_spot = raw_spot
            self.current_spot = {"x": raw_spot[0], "y": raw_spot[1]}
            return raw_spot

        # PID 静止状态（IDLE / 已收敛）才做帧间稳定性检查
        if self._last_valid_spot is not None:
            dx = raw_spot[0] - self._last_valid_spot[0]
            dy = raw_spot[1] - self._last_valid_spot[1]
            if dx*dx + dy*dy > SPOT_JUMP_MAX_PX * SPOT_JUMP_MAX_PX:
                self._spot_jump_count += 1
                if self._spot_jump_count < SPOT_JUMP_TOLERATE_FRAMES:
                    return self._last_valid_spot

        self._spot_jump_count = 0
        self._last_valid_spot = raw_spot
        self.current_spot = {"x": raw_spot[0], "y": raw_spot[1]}
        return raw_spot

    # ── v3.9 关键：每帧重新拿最新 target 算 required_spot ─────
    def _refresh_required_spot(self):
        """v3.9 修复：每帧重算 required_spot，避免 v3.8 把首帧当常量的偏差。
        YOLO 不持续发布时按 YOLO_FALLBACK_TO_LOCKED 决定是否回退到首帧锁定值。
        """
        now = time.time()
        age = now - self.yolo_target_at if self.yolo_target_at > 0 else 1e9
        is_fresh = (self.yolo_target is not None and age < YOLO_TARGET_FRESH_SEC)

        if is_fresh:
            tx, ty = self.yolo_target["x"], self.yolo_target["y"]
        elif YOLO_FALLBACK_TO_LOCKED and self._locked_yolo_target is not None:
            tx, ty = self._locked_yolo_target["x"], self._locked_yolo_target["y"]
        else:
            return

        req_x, req_y = self.calib.target_to_required_spot(tx, ty)
        req_x = max(0, min(639, req_x))
        req_y = max(0, min(479, req_y))
        self.required_spot = {"x": req_x, "y": req_y}

    # ── FSM 主循环（10Hz）────────────────────────────────────
    def _fsm_step(self):
        # v3.9.9 关键修复：只在 PID/COARSE 状态时标记"主动转动"
        # 这样 _detect_spot_now 在 IDLE/LOCKED 状态仍会做帧间稳定性检查（抑制误检）
        # 但在 PID 主动转动时跳过抑制（视场在变是正常的）
        self._pid_actively_moving = (self.fsm_state in (STATE_COARSE, STATE_PID))

        self._detect_spot_now()

        if self.fsm_state == STATE_IDLE:
            return

        if self.fsm_state == STATE_COARSE:
            self._step_coarse()
        elif self.fsm_state == STATE_PID:
            self._step_pid()
        elif self.fsm_state == STATE_LOCKED:
            self._step_locked()

    def _step_coarse(self):
        """开环粗对准

        物理模型（v3.9 文档化）：
          - 摄像头和激光绑死在云台上，云台一转：
            · 激光在画面像素位置 ≈ 不变
            · 目标在画面像素位置反向滑（视场跟着转）
          - 单次开环转动量 = (required_spot - 当前光斑) × K
            数学等价于"让转动后激光像素=目标像素"

        v3.9 bug 修复：检测不到光斑时不再 fallback 到 (320,240)，
                       改用 SPOT_HOME（启动后实测得到的归中光斑位置）
        """
        # 确保 S4 红激光是开的
        if self.laser_ir_state != "on":
            self._set_ir_laser(True)
            time.sleep(0.2)
            self._detect_spot_now()

        # 每帧重新算 required_spot
        self._refresh_required_spot()
        if self.required_spot is None:
            self.fsm_state = STATE_IDLE
            return

        if self.current_spot is not None:
            spot_x, spot_y = self.current_spot["x"], self.current_spot["y"]
            spot_source = "实测"
        else:
            spot_x, spot_y = SPOT_HOME_X, SPOT_HOME_Y
            spot_source = f"SPOT_HOME({SPOT_HOME_X},{SPOT_HOME_Y})"

        dx_pixel = self.required_spot["x"] - spot_x
        dy_pixel = self.required_spot["y"] - spot_y

        delta_yaw   = dx_pixel * PIXEL_TO_YAW_DEG
        delta_pitch = dy_pixel * PIXEL_TO_PITCH_DEG  # v3.9.4: 去除多余负号

        new_yaw   = self.servo_yaw + delta_yaw
        new_pitch = self.servo_pitch + delta_pitch
        self._set_yaw_pitch(new_yaw, new_pitch)

        self.get_logger().info(
            f"[COARSE] 光斑({spot_x},{spot_y})[{spot_source}] → "
            f"目标({self.required_spot['x']},{self.required_spot['y']})  "
            f"偏移=({dx_pixel:+d},{dy_pixel:+d})  "
            f"→ 转动({delta_yaw:+.1f}°,{delta_pitch:+.1f}°)  "
            f"→ 新角度=({new_yaw:.1f},{new_pitch:.1f})"
        )
        time.sleep(0.4)
        self._detect_spot_now()

        if self.loop_mode == "open_loop":
            self.get_logger().info("[OPEN_LOOP] 粗对准完成，停在 LOCKED 供观察。S4 保持开启。")
            self.fsm_state = STATE_LOCKED
            self._locked_at = time.time()
        else:
            self.fsm_state = STATE_PID
            self._pid_started_at = time.time()
            self.get_logger().info("[PID] 进入闭环精对准")

    def _step_pid(self):
        """PID 闭环精对准

        v3.9.6: spot/required 丢失时 reset PID + hypot 收敛判据
        v3.9.9: 关键修复
          - 设置 _pid_actively_moving 标志 → _detect_spot_now 跳过 jump 抑制
            （这是修复"目标坐标只用第一次"bug 的核心）
          - 输出饱和检测 → 连续 N 帧饱和说明系统跟不上，跳一帧给舵机响应时间
        """
        # v3.9.9: 标记 PID 正在主动驱动舵机（影响 _detect_spot_now 行为）
        self._pid_actively_moving = True

        if time.time() - self._pid_started_at > PID_TIMEOUT_SEC:
            self.get_logger().warn(f"⚠️ PID 超时（{PID_TIMEOUT_SEC}s 未收敛）")
            self._set_ir_laser(False)
            self.fsm_state = STATE_FAILED
            self._pid_actively_moving = False
            return

        spot = self._detect_spot_now()
        if spot is None:
            self.pid_x.reset()
            self.pid_y.reset()
            self._pid_saturation_count = 0
            return  # 不 unset actively_moving，下一帧还在 PID 状态

        self._refresh_required_spot()
        if self.required_spot is None:
            self.pid_x.reset()
            self.pid_y.reset()
            self._pid_saturation_count = 0
            return

        spot_x, spot_y = self.current_spot["x"], self.current_spot["y"]
        ex = self.required_spot["x"] - spot_x
        ey = self.required_spot["y"] - spot_y
        self.error = {"x": ex, "y": ey}

        # 收敛判据：欧几里得距离
        distance = (ex * ex + ey * ey) ** 0.5
        if distance < PID_TOLERANCE_PX:
            self.lock_count += 1
            self._pid_saturation_count = 0
            if self.lock_count >= PID_LOCK_FRAMES:
                self.get_logger().info(
                    f"✅ PID 收敛: 误差=({ex:+d},{ey:+d}) d={distance:.1f}px "
                    f"连续 {self.lock_count} 帧 < {PID_TOLERANCE_PX}px"
                )
                self._set_ir_laser(False)
                self.fsm_state = STATE_LOCKED
                self._locked_at = time.time()
                self._pid_actively_moving = False
                return
        else:
            self.lock_count = 0

        delta_yaw   = self.pid_x.step(ex)
        delta_pitch = self.pid_y.step(ey)

        # v3.9.9: 输出饱和 → 系统跟不上，跳本帧让舵机+相机响应
        if self.pid_x.saturated or self.pid_y.saturated:
            self._pid_saturation_count += 1
            if self._pid_saturation_count >= PID_SATURATION_FRAMES:
                self.get_logger().warn(
                    f"⚠️ PID 连续 {self._pid_saturation_count} 帧饱和 "
                    f"(d={distance:.1f}px)，跳帧给系统响应"
                )
                # 部分泄 integral 防 windup
                self.pid_x.integral *= 0.5
                self.pid_y.integral *= 0.5
                self._pid_saturation_count = 0
                return  # 本帧不发舵机命令
        else:
            self._pid_saturation_count = 0

        new_yaw   = self.servo_yaw + delta_yaw
        new_pitch = self.servo_pitch + delta_pitch
        self._set_yaw_pitch(new_yaw, new_pitch)

    def _step_locked(self):
        """已锁定，开火
        v3.9.1: 开环模式默认不自动开火（调试安全），但用户可在 UI 勾选 fire_in_open_loop=True 来打开
        v3.9.5: 防日志泛滥 — "停留" info 只在第一次进入此状态时打
        """
        if self.loop_mode == "open_loop" and not self.fire_in_open_loop:
            if not self._locked_log_done:
                self.get_logger().info(
                    "[OPEN_LOOP+LOCKED] 停留，未开火（如需开火请在网页勾选「开环也自动开火」）。"
                    "按 [紧急停止] 或 [云台归中] 可重置 FSM 接受下次打击。"
                )
                self._locked_log_done = True
            return
        # 进入开火流程：清掉 log 标志，下次 LOCKED 时可以重打
        self._locked_log_done = False
        self.fsm_state = STATE_FIRING
        threading.Thread(target=self._fire_sequence, daemon=False).start()

    def _fire_sequence(self):
        # v3.9.1 诊断日志：方便排查 S3 接线/PWM 问题
        self.get_logger().info(
            f"⚡ 蓝紫激光(S3) ON → ID={LASER_BLUE_ID}, angle={LASER_ON_ANGLE}, "
            f"持续 {FIRE_DURATION_SEC}s"
        )
        self._set_blue_laser(True, fire=True)
        time.sleep(FIRE_DURATION_SEC)
        self._set_blue_laser(False)
        self.get_logger().info(f"   蓝紫激光 OFF，冷却 {FIRE_COOLDOWN_SEC}s")
        self.fsm_state = STATE_COOLDOWN
        time.sleep(FIRE_COOLDOWN_SEC)
        self.fsm_state = STATE_IDLE
        self.lock_count = 0
        self.error = None
        self.current_spot = None
        self.required_spot = None
        self._locked_yolo_target = None
        self.get_logger().info("   伺服周期完成，回到 IDLE 等待下个目标")

    def _fire_test_thread(self, duration: float = 0.5):
        """v3.9.1: S3 独立测试烧（不进入伺服流程，仅验证 S3 接口/接线）"""
        self.get_logger().info(
            f"🧪 [S3 测试] 蓝紫激光烧 {duration}s ID={LASER_BLUE_ID} angle={LASER_ON_ANGLE}"
        )
        self._set_blue_laser(True, fire=True)
        time.sleep(duration)
        self._set_blue_laser(False)
        self.get_logger().info("🧪 [S3 测试] 完成")

    def _emergency_stop(self):
        all_lasers_off()
        self.laser_ir_state = "off"
        self.laser_blue_state = "off"
        self.fsm_state = STATE_IDLE
        self.lock_count = 0
        # v3.9.5: 紧急停止也彻底清状态，避免下次卡 LOCKED
        self.error = None
        self.required_spot = None
        self._locked_yolo_target = None
        self._locked_log_done = False
        self.get_logger().warn("🛑 紧急停止：所有激光关闭，FSM → IDLE")

    # ── HTTP ─────────────────────────────────────────────────
    def _start_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send_json(self, data, code=200):
                body = json.dumps(data).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def _send_jpeg(self, frame):
                if frame is None:
                    blank = np.zeros((480, 640, 3), np.uint8)
                    cv2.putText(blank, "Waiting for RGB camera...", (60, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 200), 2)
                    frame = blank
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    self.send_response(500); self.end_headers(); return
                data = buf.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = self.path.split("?")[0]
                qs = {}
                if "?" in self.path:
                    for kv in self.path.split("?", 1)[1].split("&"):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            qs[k] = v

                if path in ("/", "/index.html"):
                    body = HTML_PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == "/frame":
                    self._send_jpeg(node._get_rgb())
                    return

                if path == "/api/state":
                    now = time.time()
                    yolo_age = (now - node.yolo_target_at) if node.yolo_target_at > 0 else None
                    yolo_fresh = (yolo_age is not None and yolo_age < YOLO_TARGET_FRESH_SEC)
                    # v3.9.3: 蓝紫激光预测落点 = 红光斑实测位置 + Delta
                    # 仅在红光斑可见 + 标定二有效 时显示
                    predicted_hit = None
                    if node.current_spot is not None and node.calib.calib2_done:
                        px = node.current_spot["x"] + node.calib.delta_x
                        py = node.current_spot["y"] + node.calib.delta_y
                        predicted_hit = {"x": px, "y": py}
                    self._send_json({
                        "trigger":   node.trigger_mode,
                        "loop_mode": node.loop_mode,
                        "fsm_state": node.fsm_state,
                        "yolo":      node.yolo_target,
                        "yolo_age_sec": yolo_age,
                        "yolo_fresh": yolo_fresh,
                        "required":  node.required_spot,
                        "spot":      node.current_spot,
                        "error":     node.error,
                        "lock_frames": node.lock_count,
                        "lock_target": PID_LOCK_FRAMES,
                        "yaw":   node.servo_yaw,
                        "pitch": node.servo_pitch,
                        "laser_ir":   node.laser_ir_state,
                        "laser_blue": node.laser_blue_state,
                        "kp": node.kp, "ki": node.ki, "kd": node.kd,
                        "locked": node.lock_count >= PID_LOCK_FRAMES,
                        "fire_in_open_loop": node.fire_in_open_loop,
                        "calib2_stale": node.calib2_stale,
                        "calib2_frame": node.calib.calib2_frame,
                        "calib2_done":  node.calib.calib2_done,
                        "delta_x": node.calib.delta_x,
                        "delta_y": node.calib.delta_y,
                        "predicted_hit": predicted_hit,
                        "yolo_boxes": node._yolo_boxes,
                    })
                    return

                if path == "/api/trigger":
                    m = qs.get("m", "manual")
                    if m in ("manual", "auto"):
                        node.trigger_mode = m
                        node.get_logger().info(f"触发模式 → {m}")
                    self._send_json({"ok": True})
                    return

                if path == "/api/loop":
                    m = qs.get("m", "closed_loop")
                    if m in ("open_loop", "closed_loop"):
                        node.loop_mode = m
                        node.get_logger().info(f"伺服模式 → {m}")
                    self._send_json({"ok": True})
                    return

                if path == "/api/pid":
                    try:
                        node.kp = float(qs.get("kp", node.kp))
                        node.ki = float(qs.get("ki", node.ki))
                        node.kd = float(qs.get("kd", node.kd))
                        node.pid_x.kp = node.pid_y.kp = node.kp
                        node.pid_x.ki = node.pid_y.ki = node.ki
                        node.pid_x.kd = node.pid_y.kd = node.kd
                        node.get_logger().info(
                            f"PID 更新: Kp={node.kp} Ki={node.ki} Kd={node.kd}")
                    except ValueError:
                        pass
                    self._send_json({"ok": True})
                    return

                if path == "/api/go":
                    ok = node._start_servo()
                    self._send_json({"ok": ok})
                    return

                if path == "/api/stop":
                    node._emergency_stop()
                    self._send_json({"ok": True})
                    return

                if path == "/api/laser_ir":
                    on = qs.get("on", "0") == "1"
                    node._set_ir_laser(on)
                    node.get_logger().info(f"[手动] S4 RED → {'ON' if on else 'OFF'}")
                    self._send_json({"ok": True, "ir": node.laser_ir_state})
                    return

                # v3.9.1: 手动控制 S3 蓝紫激光（用于排查接线/PWM 问题）
                if path == "/api/laser_blue":
                    on = qs.get("on", "0") == "1"
                    node._set_blue_laser(on)
                    node.get_logger().warn(
                        f"[手动] S3 BLUE → {'ON' if on else 'OFF'}  "
                        f"(ID={LASER_BLUE_ID}, angle={LASER_ON_ANGLE if on else 0})"
                    )
                    self._send_json({"ok": True, "blue": node.laser_blue_state})
                    return

                # v3.9.1: S3 短时测试烧（默认 0.5s，不进入伺服流程）
                if path == "/api/fire_test":
                    if node.fsm_state in (STATE_FIRING, STATE_COOLDOWN):
                        self._send_json({"ok": False, "msg": "正在开火中"}, 400)
                        return
                    try:
                        dur = float(qs.get("dur", "0.5"))
                        dur = max(0.1, min(2.0, dur))  # 安全限制 0.1-2 秒
                    except ValueError:
                        dur = 0.5
                    threading.Thread(
                        target=node._fire_test_thread, args=(dur,), daemon=False
                    ).start()
                    self._send_json({"ok": True, "duration": dur})
                    return

                # v3.9.1: 开环模式是否自动开火 开关
                if path == "/api/fire_open_toggle":
                    on = qs.get("on", "0") == "1"
                    node.fire_in_open_loop = on
                    node.get_logger().info(
                        f"[配置] 开环模式自动开火 → {'启用' if on else '禁用'}"
                    )
                    self._send_json({"ok": True, "fire_in_open_loop": on})
                    return

                if path == "/api/center":
                    center_servo()
                    node.servo_yaw = SERVO_YAW_CENTER
                    node.servo_pitch = SERVO_PITCH_CENTER
                    # v3.9.5 修 Bug: 归中也重置 FSM，避免卡在 LOCKED 导致下次 [开始打击] 被拒
                    prev_state = node.fsm_state
                    node.fsm_state = STATE_IDLE
                    node.lock_count = 0
                    node.error = None
                    node.required_spot = None
                    node._locked_yolo_target = None
                    node._locked_log_done = False
                    node.get_logger().info(
                        f"云台已归中（FSM: {prev_state} → IDLE）"
                    )
                    self._send_json({"ok": True})
                    return

                self.send_response(404); self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", SERVO_HTTP_PORT), Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = VisionServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        all_lasers_off()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
