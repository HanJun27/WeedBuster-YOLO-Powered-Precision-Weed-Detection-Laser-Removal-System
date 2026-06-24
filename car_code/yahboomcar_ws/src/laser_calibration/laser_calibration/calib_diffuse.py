#!/usr/bin/env python3
"""
calib_diffuse.py —— 标定四：主动光场 + 灰卡反射率定标  v3.10
==============================================================
专门服务 active canopy sensor 路径的低成本 vicarious calibration：

  - 主动光源（850nm IR LED + 白光 LED + 柔光罩）打到工作面
  - ISP 锁定下，画面里固定一块 18% 灰卡 / PTFE 板
  - 三步走：测暗电流 → 测灰卡 → 保存

数学原理：
  NDVI = (K·NIR' − R') / (K·NIR' + R')
    R'   = max(0, DN_R   − dark_R)        ← 减暗电流
    NIR' = max(0, DN_NIR − dark_NIR)
    K    = R_gray' / NIR_gray'            ← 灰卡 ROI 内均值（已减暗电流）

  灰卡反射率近似平坦时，K 把 NIR 通道拉到 R 通道同基准。
  之后任意像素的 NDVI 就接近真值，距离/材质效应在比值中抵消。

操作流程（浏览器中走一遍）：
  [Step 1] 盖镜头盖 → 点【采样暗电流】→ 等 30 帧均值
  [Step 2] 摆好灰卡 + 主动光源 → 拖框定位灰卡 ROI → 点【采样灰卡】→ 等 30 帧
           （拖框时画面是冻结的，方便准确选区）
  [Step 3] 选光源类型 + 工作距离 → 点【保存】
  按【紧急停止】或关闭终端 → 安全退出

前置：
  ros2 run laser_calibration stereo_camera

运行：
  ros2 run laser_calibration calib_diffuse

浏览器：
  http://localhost:8094         (本机)
  http://<小车IP>:8094           (远程)

⚠️ 注意：
  - 暗电流和灰卡必须在同一光照、同一曝光参数下做（ISP 锁死保证这一点）
  - 灰卡 ROI 必须只覆盖灰卡，不要圈到其他物体
  - 不同场景（室内/户外/换光源）必须重新标定
"""

import datetime
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

from laser_calibration.calib_io import save_calib, linearize_dn
from laser_calibration.config import (
    CAM_FPS, STREAM_QUALITY,
    DARK_FRAME_COUNT, GRAY_FRAME_COUNT, GRAY_MIN_DN,
    GRAY_CARD_REFLECTANCE, SENSOR_GAMMA,
    GRAY_ROI_X, GRAY_ROI_Y, GRAY_ROI_W, GRAY_ROI_H,
    TOPIC_IR, TOPIC_RGB,
)

DIFFUSE_HTTP_PORT = 8094


# ══════════════════════════════════════════════════════════════
#  HTML 页面（带拖框 ROI 选择 + 三步流程）
# ══════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>标定四 · 主动光场 + 灰卡定标 (v3.10)</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1 { color:#0f0; margin:0 0 12px; font-size:18px; }
  .panel { background:#222; padding:12px; border-radius:8px; margin-bottom:12px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px;
         margin-bottom:4px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .btn-stop { background:#3a0808; color:#ff0; border-color:#ff0; font-weight:bold; }
  .btn-stop:hover { background:#ff0; color:#000; }
  .step { display:inline-block; padding:3px 10px; border:1px solid #555;
          border-radius:3px; margin-right:6px; font-size:13px; }
  .step.done { color:#0f0; border-color:#0f0; }
  .step.active { color:#fa0; border-color:#fa0; background:#332200; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; overflow:hidden;
             width:640px; height:480px; cursor:crosshair; }
  .cam-box.live { cursor:default; }
  .cam-label { position:absolute; top:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#0f0; font-size:12px; z-index:10;
               pointer-events:none; }
  canvas { display:block; }
  .stat-box { min-width:280px; }
  .stat-row { display:flex; justify-content:space-between; padding:3px 0;
              border-bottom:1px dotted #333; font-size:13px; }
  .stat-key { color:#888; }
  .stat-val { color:#0f0; }
  .sub-info { color:#888; font-size:12px; margin-top:8px; line-height:1.6; }
  .v { color:#0f0; }
  .w { color:#fa0; }
  .danger { color:#f55; }
  select, input { background:#2a2a2a; color:#0f0; border:1px solid #555;
                  padding:4px 6px; font-family:monospace; font-size:13px; }
  .status-line { color:#fa0; min-height:18px; padding:6px 0; }
</style>
</head>
<body>
<h1>📐 标定四 · 主动光场 + 灰卡漫反射定标 (v3.10)</h1>

<div class="panel">
  <span class="step active" id="step1">① 暗电流</span>
  <span class="step" id="step2">② 灰卡 ROI</span>
  <span class="step" id="step3">③ 保存</span>
  <span style="margin-left:20px;color:#888">参考物反射率：</span>
  <span class="v" id="ref-refl">-</span>
</div>

<div class="panel">
  <button class="btn" id="btn1" onclick="startDark()">[1] 采样暗电流（盖镜头）</button>
  <button class="btn" id="btn-freeze" onclick="freezeForRoi()" disabled>[2a] 冻结画面拖框</button>
  <button class="btn" id="btn2" onclick="sampleGray()" disabled>[2b] 采样灰卡 ROI</button>
  <button class="btn" id="btn3" onclick="confirmSave()" disabled>[3] 保存</button>
  <button class="btn" id="btn-reset" onclick="reset()">[R] 重做</button>
  <button class="btn btn-stop" onclick="emergencyStop()">[紧急停止]</button>
  <div class="status-line" id="status">就绪。请把镜头盖盖好，然后按 [1]。</div>
</div>

<div class="row">
  <div>
    <div class="cam-box live" id="cam-rgb">
      <span class="cam-label">RGB 画面</span>
      <canvas id="cv-rgb" width="640" height="480"></canvas>
    </div>
  </div>
  <div>
    <div class="cam-box live" id="cam-ir">
      <span class="cam-label">IR 画面（与 RGB 共用 ROI）</span>
      <canvas id="cv-ir" width="640" height="480"></canvas>
    </div>
  </div>
  <div class="stat-box panel">
    <div style="color:#888;font-size:13px;margin-bottom:6px">实时统计</div>

    <div class="stat-row"><span class="stat-key">暗电流 R (DN)</span>
      <span class="stat-val" id="s-dark-r">--</span></div>
    <div class="stat-row"><span class="stat-key">暗电流 NIR (DN)</span>
      <span class="stat-val" id="s-dark-nir">--</span></div>

    <div class="stat-row" style="margin-top:6px"><span class="stat-key">灰卡 ROI</span>
      <span class="stat-val" id="s-roi">--</span></div>
    <div class="stat-row"><span class="stat-key">灰卡 R (DN)</span>
      <span class="stat-val" id="s-gray-r">--</span></div>
    <div class="stat-row"><span class="stat-key">灰卡 NIR (DN)</span>
      <span class="stat-val" id="s-gray-nir">--</span></div>
    <div class="stat-row"><span class="stat-key">K = R'/NIR'</span>
      <span class="stat-val" id="s-k">--</span></div>

    <div class="stat-row" style="margin-top:8px"><span class="stat-key">光源类型</span>
      <select id="light-type">
        <option value="indoor_active" selected>室内 主动光场（IR LED+白光）</option>
        <option value="indoor_window">室内 自然光（窗户）</option>
        <option value="indoor_halogen">室内 卤素灯</option>
        <option value="outdoor_sun">户外 太阳光</option>
      </select>
    </div>
    <div class="stat-row"><span class="stat-key">工作距离 (cm)</span>
      <input type="number" id="distance-cm" value="50" min="10" max="200" style="width:60px">
    </div>

    <div class="sub-info">
      <span class="v">①</span> 镜头盖好 → [1]<br>
      <span class="v">②</span> 摆灰卡 + 开主动光源 → [2a] 冻结 → 拖框 → [2b] 采样<br>
      <span class="v">③</span> 选光源 + 距离 → [3] 保存
    </div>
  </div>
</div>

<div class="sub-info" style="margin-top:8px">
  <span class="w">⚠️ 暗电流和灰卡必须在同一曝光、同一光照下采集</span>。<br>
  ROI 选择只圈灰卡，不要圈到周围物体。<br>
  K 值合理范围（参考）：白光 LED 多、IR 弱 → K&lt;1（NIR 通道要被乘大）<br>
  K 值合理范围（参考）：IR 强 → K&gt;1（NIR 通道要被压小）
</div>

<script>
let frozen = false;
let drawingRoi = false;
let roiStart = null;
let currentRoi = null;   // {x, y, w, h}
let frozenFrameRgb = null;

const cvRgb = document.getElementById('cv-rgb');
const ctxRgb = cvRgb.getContext('2d');
const cvIr  = document.getElementById('cv-ir');
const ctxIr  = cvIr.getContext('2d');

// ── MJPEG 流接收（直接画到 canvas） ────────────────────
function startStream(canvas, url) {
  const ctx = canvas.getContext('2d');
  const img = new Image();
  img.onload = function() {
    if (canvas.id === 'cv-rgb' && frozen && frozenFrameRgb) {
      ctx.drawImage(frozenFrameRgb, 0, 0);
    } else {
      ctx.drawImage(img, 0, 0);
    }
    drawRoiOverlay(ctx);
  };
  function loop() {
    img.src = url + '?t=' + Date.now();
    setTimeout(loop, 1000 / 15);
  }
  loop();
}

function drawRoiOverlay(ctx) {
  // 实时 ROI 框（蓝色虚线）
  if (currentRoi) {
    ctx.strokeStyle = '#3af';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(currentRoi.x, currentRoi.y, currentRoi.w, currentRoi.h);
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(51,170,255,0.2)';
    ctx.fillRect(currentRoi.x, currentRoi.y, currentRoi.w, currentRoi.h);
  }
  // 拖框中
  if (drawingRoi && roiStart) {
    const cur = roiStart.last;
    ctx.strokeStyle = '#fa0';
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(roiStart.x, roiStart.y,
                   cur.x - roiStart.x, cur.y - roiStart.y);
    ctx.setLineDash([]);
  }
}

// ── ROI 拖框（仅冻结状态生效） ─────────────────────────
cvRgb.addEventListener('mousedown', (e) => {
  if (!frozen) return;
  const rect = cvRgb.getBoundingClientRect();
  const x = Math.round(e.clientX - rect.left);
  const y = Math.round(e.clientY - rect.top);
  drawingRoi = true;
  roiStart = { x, y, last: { x, y } };
});
cvRgb.addEventListener('mousemove', (e) => {
  if (!drawingRoi || !roiStart) return;
  const rect = cvRgb.getBoundingClientRect();
  roiStart.last.x = Math.round(e.clientX - rect.left);
  roiStart.last.y = Math.round(e.clientY - rect.top);
});
cvRgb.addEventListener('mouseup', () => {
  if (!drawingRoi || !roiStart) return;
  const x1 = Math.min(roiStart.x, roiStart.last.x);
  const y1 = Math.min(roiStart.y, roiStart.last.y);
  const w  = Math.abs(roiStart.last.x - roiStart.x);
  const h  = Math.abs(roiStart.last.y - roiStart.y);
  if (w >= 10 && h >= 10) {
    currentRoi = { x: x1, y: y1, w, h };
    fetch(`/api/set_roi?x=${x1}&y=${y1}&w=${w}&h=${h}`);
    document.getElementById('s-roi').textContent = `(${x1},${y1}) ${w}×${h}`;
    document.getElementById('btn2').disabled = false;
    setStatus(`ROI 已选定 (${x1},${y1}) ${w}×${h}，点 [2b] 采样灰卡`);
  } else {
    setStatus('框太小，请重新拖框（至少 10×10）');
  }
  drawingRoi = false;
  roiStart = null;
});

function setStatus(msg) {
  document.getElementById('status').textContent = msg;
}

// ── 按钮动作 ──────────────────────────────────────────
async function startDark() {
  setStatus('采样暗电流中... 30 帧约 1 秒');
  document.getElementById('btn1').disabled = true;
  const r = await fetch('/api/sample_dark');
  const d = await r.json();
  if (d.ok) {
    document.getElementById('s-dark-r').textContent = d.dark_R.toFixed(2);
    document.getElementById('s-dark-nir').textContent = d.dark_NIR.toFixed(2);
    document.getElementById('step1').className = 'step done';
    document.getElementById('step2').className = 'step active';
    document.getElementById('btn-freeze').disabled = false;
    setStatus(`✅ 暗电流: R=${d.dark_R.toFixed(2)}, NIR=${d.dark_NIR.toFixed(2)}。摆灰卡，开光源，按 [2a]`);
  } else {
    document.getElementById('btn1').disabled = false;
    setStatus('❌ 失败：' + (d.err || ''));
  }
}

async function freezeForRoi() {
  // 先取一张当前 RGB 帧作为冻结底图
  const img = new Image();
  img.onload = () => {
    frozenFrameRgb = img;
    frozen = true;
    document.getElementById('cam-rgb').classList.remove('live');
    setStatus('画面已冻结，在 RGB 画面上拖框圈选灰卡');
  };
  img.src = '/snapshot/rgb?t=' + Date.now();
}

async function sampleGray() {
  if (!currentRoi) { setStatus('请先拖框圈选灰卡 ROI'); return; }
  setStatus('采样灰卡中... 30 帧约 1 秒');
  document.getElementById('btn2').disabled = true;
  const r = await fetch('/api/sample_gray');
  const d = await r.json();
  if (d.ok) {
    document.getElementById('s-gray-r').textContent = d.dn_r_gray.toFixed(2);
    document.getElementById('s-gray-nir').textContent = d.dn_nir_gray.toFixed(2);
    document.getElementById('s-k').textContent = d.k_active.toFixed(4);
    document.getElementById('step2').className = 'step done';
    document.getElementById('step3').className = 'step active';
    document.getElementById('btn3').disabled = false;
    frozen = false;
    document.getElementById('cam-rgb').classList.add('live');
    setStatus(`✅ K=${d.k_active.toFixed(4)}。选光源类型 + 距离，点 [3] 保存`);
  } else {
    document.getElementById('btn2').disabled = false;
    setStatus('❌ 失败：' + (d.err || ''));
  }
}

async function confirmSave() {
  const lt = document.getElementById('light-type').value;
  const dc = parseInt(document.getElementById('distance-cm').value || '50');
  if (!confirm(`确认保存？\n光源: ${lt}\n距离: ${dc}cm\n\n会写入 ~/calib_params.yaml`)) return;
  const r = await fetch(`/api/save?light=${lt}&distance=${dc}`);
  const d = await r.json();
  if (d.ok) {
    document.getElementById('step3').className = 'step done';
    document.getElementById('btn3').disabled = true;
    setStatus(`🎉 已保存。time=${d.timestamp}, light=${d.light}, dist=${d.distance}cm`);
  } else {
    setStatus('❌ 保存失败：' + (d.err || ''));
  }
}

async function reset() {
  if (!confirm('清空当前会话所有采样数据？标定文件不变')) return;
  await fetch('/api/reset');
  document.getElementById('s-dark-r').textContent = '--';
  document.getElementById('s-dark-nir').textContent = '--';
  document.getElementById('s-gray-r').textContent = '--';
  document.getElementById('s-gray-nir').textContent = '--';
  document.getElementById('s-k').textContent = '--';
  document.getElementById('s-roi').textContent = '--';
  currentRoi = null;
  frozen = false;
  document.getElementById('cam-rgb').classList.add('live');
  document.getElementById('btn1').disabled = false;
  document.getElementById('btn-freeze').disabled = true;
  document.getElementById('btn2').disabled = true;
  document.getElementById('btn3').disabled = true;
  document.getElementById('step1').className = 'step active';
  document.getElementById('step2').className = 'step';
  document.getElementById('step3').className = 'step';
  setStatus('已重置。请把镜头盖盖好，按 [1]');
}

async function emergencyStop() {
  await fetch('/api/stop');
  setStatus('⛔ 紧急停止');
}

// ── 启动 ──────────────────────────────────────────────
fetch('/api/state').then(r => r.json()).then(d => {
  document.getElementById('ref-refl').textContent =
    d.gray_reflectance.toFixed(2) + ' (' + d.gray_label + ')';
  // 若已有 default ROI，显示出来
  if (d.default_roi) {
    currentRoi = d.default_roi;
    document.getElementById('s-roi').textContent =
      `(${d.default_roi.x},${d.default_roi.y}) ${d.default_roi.w}×${d.default_roi.h}`;
  }
});

startStream(cvRgb, '/snapshot/rgb');
startStream(cvIr,  '/snapshot/ir');
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class CalibDiffuseNode(Node):
    def __init__(self):
        super().__init__("calib_diffuse")
        self.bridge = CvBridge()

        # 帧缓存
        self._rgb = None
        self._ir = None
        self._lock = threading.Lock()

        # ROI（默认从 config 读取，可通过 web 拖框覆盖）
        self.roi_x = GRAY_ROI_X
        self.roi_y = GRAY_ROI_Y
        self.roi_w = GRAY_ROI_W
        self.roi_h = GRAY_ROI_H

        # 标定结果
        self.dark_R = None
        self.dark_NIR = None
        self.dn_r_gray = None
        self.dn_nir_gray = None
        self.k_active = None

        # 订阅
        self.sub_rgb = self.create_subscription(
            Image, TOPIC_RGB, self._cb_rgb, 10)
        self.sub_ir = self.create_subscription(
            Image, TOPIC_IR, self._cb_ir, 10)

        # 启动 HTTP
        self._start_http()

        log = self.get_logger().info
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log("  标定四 · 主动光场 + 灰卡定标  v3.10")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log(f"  参考物反射率: {GRAY_CARD_REFLECTANCE} (config.GRAY_CARD_REFLECTANCE)")
        log(f"  传感器伽马:   {SENSOR_GAMMA} (去伽马,经验线法前置)")
        log(f"  默认 ROI:    ({GRAY_ROI_X},{GRAY_ROI_Y}) {GRAY_ROI_W}×{GRAY_ROI_H}")
        log(f"  HTTP 端口:   {DIFFUSE_HTTP_PORT}")
        log(f"  本机访问:    http://localhost:{DIFFUSE_HTTP_PORT}")
        log(f"  远程访问:    http://<小车IP>:{DIFFUSE_HTTP_PORT}")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    # ── 回调 ────────────────────────────────────────────────
    def _cb_rgb(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._rgb = f
        except Exception as e:
            self.get_logger().error(f"RGB 解码失败：{e}")

    def _cb_ir(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._ir = f
        except Exception as e:
            self.get_logger().error(f"IR 解码失败：{e}")

    def _get_frames(self):
        """获取一对 RGB 和 IR 帧（已转灰度）"""
        with self._lock:
            rgb = self._rgb
            ir = self._ir
        if rgb is None or ir is None:
            return None, None
        ir_gray = cv2.cvtColor(ir, cv2.COLOR_BGR2GRAY)
        return rgb, ir_gray

    # ── 标定核心：采暗电流 ──────────────────────────────────
    def sample_dark(self):
        """连续采 N 帧 RGB 红通道 + IR 灰度的均值（去伽马后），作为暗电流。"""
        rs = []
        nirs = []
        t0 = time.time()
        deadline = t0 + 5.0  # 最多等 5 秒
        while len(rs) < DARK_FRAME_COUNT and time.time() < deadline:
            rgb, ir = self._get_frames()
            if rgb is None:
                time.sleep(0.05)
                continue
            # 去伽马到线性空间后再求均值（经验线法要求线性）
            r_lin = linearize_dn(rgb[:, :, 2], SENSOR_GAMMA)
            nir_lin = linearize_dn(ir, SENSOR_GAMMA)
            rs.append(float(np.mean(r_lin)))
            nirs.append(float(np.mean(nir_lin)))
            time.sleep(1.0 / CAM_FPS)

        if len(rs) < 5:
            return False, "RGB 或 IR 帧不足，请确认 stereo_camera 在跑"

        self.dark_R = float(np.mean(rs))
        self.dark_NIR = float(np.mean(nirs))

        # 物理合理性检查：暗电流应该很小
        if self.dark_R > 30 or self.dark_NIR > 30:
            self.get_logger().warn(
                f"暗电流异常高 (R={self.dark_R:.1f}, NIR={self.dark_NIR:.1f})。"
                "镜头是否真的盖好了？或 ISP 没锁，环境光透过缝隙进入？"
            )
        self.get_logger().info(
            f"✅ 暗电流: dark_R={self.dark_R:.2f}, dark_NIR={self.dark_NIR:.2f} "
            f"({len(rs)} 帧均值)"
        )
        return True, ""

    # ── 标定核心：采灰卡 ────────────────────────────────────
    def sample_gray(self):
        """在 ROI 内采 N 帧灰卡 R 和 NIR DN 均值，算 K = R'/NIR'。"""
        if self.dark_R is None or self.dark_NIR is None:
            return False, "请先采样暗电流（步骤 1）"
        if self.roi_w < 10 or self.roi_h < 10:
            return False, "ROI 太小（至少 10×10）"

        rs = []
        nirs = []
        deadline = time.time() + 5.0
        while len(rs) < GRAY_FRAME_COUNT and time.time() < deadline:
            rgb, ir = self._get_frames()
            if rgb is None:
                time.sleep(0.05)
                continue
            roi_rgb = rgb[self.roi_y:self.roi_y + self.roi_h,
                          self.roi_x:self.roi_x + self.roi_w]
            roi_ir = ir[self.roi_y:self.roi_y + self.roi_h,
                        self.roi_x:self.roi_x + self.roi_w]
            if roi_rgb.size == 0 or roi_ir.size == 0:
                time.sleep(0.05)
                continue
            # 去伽马到线性空间后再求均值
            r_lin = linearize_dn(roi_rgb[:, :, 2], SENSOR_GAMMA)
            nir_lin = linearize_dn(roi_ir, SENSOR_GAMMA)
            rs.append(float(np.mean(r_lin)))
            nirs.append(float(np.mean(nir_lin)))
            time.sleep(1.0 / CAM_FPS)

        if len(rs) < 5:
            return False, "采样帧不足"

        # 注意：rs/nirs 已是去伽马后的线性空间值
        dn_r_lin = float(np.mean(rs))
        dn_nir_lin = float(np.mean(nirs))
        # 减暗电流（dark_R/dark_NIR 也是线性空间值）
        dn_r_corr = max(1.0, dn_r_lin - self.dark_R)
        dn_nir_corr = max(1.0, dn_nir_lin - self.dark_NIR)

        # 灰卡过暗检查（可能 ROI 圈错地方了）
        if dn_r_corr < GRAY_MIN_DN or dn_nir_corr < GRAY_MIN_DN:
            return False, (f"灰卡 DN 过低 (R'={dn_r_corr:.0f}, NIR'={dn_nir_corr:.0f}, "
                           f"阈值 {GRAY_MIN_DN})。ROI 是否圈错？光源是否够亮？")

        self.dn_r_gray = dn_r_lin
        self.dn_nir_gray = dn_nir_lin
        self.k_active = dn_r_corr / dn_nir_corr

        self.get_logger().info(
            f"✅ 灰卡(线性空间): R={dn_r_lin:.2f} R'={dn_r_corr:.2f}, "
            f"NIR={dn_nir_lin:.2f} NIR'={dn_nir_corr:.2f}, "
            f"K={self.k_active:.4f} ({len(rs)} 帧均值)"
        )

        # K 异常范围警告（一般在 0.3 ~ 3.0 之间）
        if self.k_active < 0.2 or self.k_active > 5.0:
            self.get_logger().warn(
                f"K={self.k_active:.4f} 数值异常，物理上一般在 0.3~3.0。"
                "请检查光源比例和 ROI 是否合理。"
            )
        return True, ""

    # ── 保存 ────────────────────────────────────────────────
    def save_calib_data(self, light_type: str, distance_cm: int):
        if self.dark_R is None or self.k_active is None:
            return False, "未完成采样"
        ts = datetime.datetime.now().isoformat(timespec="seconds")
        params = {
            "dark_R":              float(self.dark_R),
            "dark_NIR":            float(self.dark_NIR),
            "k_active":            float(self.k_active),
            "gray_reflectance":    float(GRAY_CARD_REFLECTANCE),
            "gamma":               float(SENSOR_GAMMA),
            "calib4_done":         True,
            "calib4_timestamp":    ts,
            "calib4_light":        light_type,
            "calib4_distance_cm":  int(distance_cm),
        }
        try:
            save_calib(params)
        except Exception as e:
            return False, f"写入失败：{e}"
        self.get_logger().info(
            f"🎉 标定四已保存: K={self.k_active:.4f}, "
            f"dark_R={self.dark_R:.2f}, dark_NIR={self.dark_NIR:.2f}, "
            f"light={light_type}, dist={distance_cm}cm"
        )
        return True, ts

    def reset(self):
        self.dark_R = None
        self.dark_NIR = None
        self.dn_r_gray = None
        self.dn_nir_gray = None
        self.k_active = None
        self.roi_x = GRAY_ROI_X
        self.roi_y = GRAY_ROI_Y
        self.roi_w = GRAY_ROI_W
        self.roi_h = GRAY_ROI_H
        self.get_logger().info("已重置当前采样会话")

    # ── HTTP 服务 ───────────────────────────────────────────
    def _start_http(self):
        ref = self

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send_json(self, data, code=200):
                body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def _send_jpeg(self, frame):
                if frame is None:
                    self.send_response(503)
                    self.end_headers()
                    return
                ok, jpg = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY])
                if not ok:
                    self.send_response(500)
                    self.end_headers()
                    return
                body = jpg.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def _qs(self):
                """解析查询参数"""
                qs = {}
                if "?" in self.path:
                    for kv in self.path.split("?", 1)[1].split("&"):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            qs[k] = v
                return qs

            def do_GET(self):
                p = self.path.split("?", 1)[0]

                # ── 首页 ──
                if p == "/" or p == "/index.html":
                    body = HTML_PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                # ── 单帧快照 ──
                if p == "/snapshot/rgb":
                    rgb, _ = ref._get_frames()
                    self._send_jpeg(rgb)
                    return
                if p == "/snapshot/ir":
                    _, ir = ref._get_frames()
                    if ir is not None:
                        # 灰度转伪彩，方便看
                        ir_bgr = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
                        self._send_jpeg(ir_bgr)
                    else:
                        self._send_jpeg(None)
                    return

                # ── 设定 ROI ──
                if p == "/api/set_roi":
                    qs = self._qs()
                    try:
                        ref.roi_x = int(qs.get("x", 0))
                        ref.roi_y = int(qs.get("y", 0))
                        ref.roi_w = int(qs.get("w", 80))
                        ref.roi_h = int(qs.get("h", 80))
                        self._send_json({"ok": True,
                                         "roi": {"x": ref.roi_x, "y": ref.roi_y,
                                                 "w": ref.roi_w, "h": ref.roi_h}})
                    except Exception as e:
                        self._send_json({"ok": False, "err": str(e)}, 400)
                    return

                # ── 采样暗电流 ──
                if p == "/api/sample_dark":
                    ok, err = ref.sample_dark()
                    if ok:
                        self._send_json({
                            "ok": True,
                            "dark_R": ref.dark_R,
                            "dark_NIR": ref.dark_NIR,
                        })
                    else:
                        self._send_json({"ok": False, "err": err}, 500)
                    return

                # ── 采样灰卡 ──
                if p == "/api/sample_gray":
                    ok, err = ref.sample_gray()
                    if ok:
                        self._send_json({
                            "ok": True,
                            "dn_r_gray": ref.dn_r_gray,
                            "dn_nir_gray": ref.dn_nir_gray,
                            "k_active": ref.k_active,
                        })
                    else:
                        self._send_json({"ok": False, "err": err}, 400)
                    return

                # ── 保存 ──
                if p == "/api/save":
                    qs = self._qs()
                    light = qs.get("light", "indoor_active")
                    try:
                        distance = int(qs.get("distance", "50"))
                    except ValueError:
                        distance = 50
                    ok, msg = ref.save_calib_data(light, distance)
                    if ok:
                        self._send_json({
                            "ok": True,
                            "timestamp": msg,
                            "light": light,
                            "distance": distance,
                        })
                    else:
                        self._send_json({"ok": False, "err": msg}, 500)
                    return

                # ── 重置 ──
                if p == "/api/reset":
                    ref.reset()
                    self._send_json({"ok": True})
                    return

                # ── 紧急停止（这里其实就是清状态，不涉及激光）──
                if p == "/api/stop":
                    ref.reset()
                    self._send_json({"ok": True})
                    return

                # ── 状态 ──
                if p == "/api/state":
                    label = "PTFE" if GRAY_CARD_REFLECTANCE > 0.85 else \
                            ("白板" if GRAY_CARD_REFLECTANCE > 0.5 else "18%灰卡")
                    self._send_json({
                        "gray_reflectance": GRAY_CARD_REFLECTANCE,
                        "gray_label": label,
                        "default_roi": {
                            "x": ref.roi_x, "y": ref.roi_y,
                            "w": ref.roi_w, "h": ref.roi_h,
                        },
                        "dark_done": ref.dark_R is not None,
                        "gray_done": ref.k_active is not None,
                    })
                    return

                self.send_response(404)
                self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", DIFFUSE_HTTP_PORT), _Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = CalibDiffuseNode()
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
