#!/usr/bin/env python3
"""
calib_reflectance.py —— 标定三：反射率定标（真NDVI）  v3.5
=============================================================
解决"伪 NDVI"的根本缺陷：消除距离/材质对 NDVI 数值的污染。

原理：
  通过黑/灰/白三块已知反射率的色卡，建立"DN 像素值 → 真实反射率"的线性映射。
  最小二乘解出红光通道和近红外通道的 k/b 参数。
  之后所有 NDVI 计算都先把 DN 值映射回真实反射率，再做比值，
  这样距离效应在分子分母中被等比抵消。

操作流程（浏览器中走一遍）:
  1. 把黑/灰/白三块色卡同时摆在镜头前（同一光照、同一距离）
  2. 网页上看到 RGB 实时画面
  3. 用鼠标在 RGB 画面上 [拖框] 圈选白卡区域 → 点【采样白】
  4. 同样圈选 灰卡 + 黑卡，分别点【采样灰】【采样黑】
  5. 三组数据齐了，自动算最小二乘 → 显示拟合直线 + R²
  6. 点【保存】写入 calib_params.yaml

前置：
  ros2 run laser_calibration stereo_camera

运行：
  ros2 run laser_calibration calib_refl

浏览器：
  http://localhost:8092         (本机)
  http://<小车IP>:8092           (远程)

⚠️ 标定有时效性：
  环境光变了（云飘过、补光灯亮度变了、换房间）必须重做。
  节点启动时会显示上次标定时间，提示是否需要重新标定。
"""

import datetime
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from laser_calibration.calib_io import load_calib, save_calib
from laser_calibration.config import (
    PANEL_BLACK, PANEL_GRAY18, PANEL_GRAY50, PANEL_WHITE,
    TOPIC_IR, TOPIC_RGB,
)

CALIB_HTTP_PORT = 8092


HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>标定三 · 反射率定标 (真NDVI)</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1 { color:#0f0; margin:0 0 12px; font-size:18px; }
  .panel { background:#222; padding:12px; border-radius:8px; margin-bottom:12px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px;
         margin-bottom:4px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .btn-w { color:#fff; border-color:#fff; }
  .btn-w:hover { background:#fff; color:#000; }
  .btn-g { color:#aaa; border-color:#aaa; }
  .btn-g:hover { background:#aaa; color:#000; }
  .btn-k { color:#888; border-color:#888; }
  .btn-k:hover { background:#888; color:#000; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; overflow:hidden;
             width:640px; height:480px; }
  .cam-label { position:absolute; top:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#0f0; font-size:12px; z-index:10;
               pointer-events:none; }
  canvas { display:block; cursor:crosshair; }
  .status { color:#fa0; font-size:14px; padding:8px; min-height:18px; }
  .info { color:#888; font-size:12px; margin-top:8px; }
  .v { color:#0f0; }
  .table { border-collapse:collapse; margin-top:6px; }
  .table th, .table td { border:1px solid #333; padding:4px 12px;
                          text-align:right; font-size:12px; }
  .table th { color:#0f0; background:#222; text-align:center; }
  .table td.row-label { color:#fa0; text-align:left; }
  .sampled { color:#0f0; }
  .pending { color:#666; }
  .info-panel { background:#111; padding:10px; border-radius:6px;
                margin-top:8px; max-width:780px; }
  .stage { display:inline-block; padding:2px 10px; margin:2px;
           border:1px solid #555; border-radius:3px; }
  .stage.done { border-color:#0f0; color:#0f0; }
  .stage.active { border-color:#fa0; color:#fa0; background:#332200; }
</style>
</head>
<body>
<h1>🎨 标定三 · 反射率定标 (真NDVI)</h1>

<div class="panel">
  <span class="stage" id="st-w">① 白卡 (90%)</span>
  <span class="stage" id="st-g50">② 灰50%</span>
  <span class="stage" id="st-g18">③ 灰18%</span>
  <span class="stage" id="st-k">④ 黑卡 (10%)</span>
  <span class="stage" id="st-s">⑤ 保存</span>
</div>

<div class="panel">
  <button class="btn btn-w" id="btn-w" onclick="sampleColor('white')" disabled>[1] 采样白90%</button>
  <button class="btn btn-g" id="btn-g50" onclick="sampleColor('gray50')" disabled>[2] 采样灰50%</button>
  <button class="btn btn-g" id="btn-g18" onclick="sampleColor('gray18')" disabled>[3] 采样灰18%</button>
  <button class="btn btn-k" id="btn-k" onclick="sampleColor('black')" disabled>[4] 采样黑10%</button>
  <button class="btn" id="btn-fit" onclick="action('fit')" disabled>[F] 最小二乘拟合</button>
  <button class="btn" id="btn-save" onclick="action('save')" disabled>[C] 保存</button>
  <button class="btn" id="btn-reset" onclick="action('reset')">[R] 重做</button>
  <div class="status" id="status">→ 把四色卡摆在镜头前同一光照下，鼠标拖框圈选白卡，按 [1]</div>
</div>

<div class="row">
  <div class="cam-box">
    <span class="cam-label">RGB (拖动鼠标圈选区域)</span>
    <canvas id="cv" width="640" height="480"></canvas>
  </div>
  <div>
    <table class="table">
      <tr>
        <th>色卡</th>
        <th>真值反射率</th>
        <th>R 通道 DN</th>
        <th>NIR 通道 DN</th>
      </tr>
      <tr>
        <td class="row-label">白 (90%)</td>
        <td>0.90</td>
        <td id="dn-r-w" class="pending">--</td>
        <td id="dn-n-w" class="pending">--</td>
      </tr>
      <tr>
        <td class="row-label">灰 (50%)</td>
        <td>0.50</td>
        <td id="dn-r-g50" class="pending">--</td>
        <td id="dn-n-g50" class="pending">--</td>
      </tr>
      <tr>
        <td class="row-label">灰 (18%)</td>
        <td>0.18</td>
        <td id="dn-r-g18" class="pending">--</td>
        <td id="dn-n-g18" class="pending">--</td>
      </tr>
      <tr>
        <td class="row-label">黑 (10%)</td>
        <td>0.10</td>
        <td id="dn-r-k" class="pending">--</td>
        <td id="dn-n-k" class="pending">--</td>
      </tr>
    </table>

    <div class="info-panel" id="fit-result">
      <div class="info">最小二乘拟合结果将显示在这里</div>
    </div>
  </div>
</div>

<div class="info">
  操作: <span class="v">①</span> 鼠标拖框 →
  <span class="v">②</span> 按 [1/2/3] 采样三色卡 →
  <span class="v">③</span> [F] 拟合 →
  <span class="v">④</span> [C] 保存
</div>
<div class="info">
  快捷键: <span class="v">1/2/3</span> 采样白/灰/黑 ·
  <span class="v">F</span> 拟合 ·
  <span class="v">C</span> 保存 ·
  <span class="v">R</span> 重做
</div>

<script>
const W = 640, H = 480;
const canvas = document.getElementById('cv');
const ctx = canvas.getContext('2d');
let liveImg = new Image();
let frame = null;
let selRect = null;
let dragStart = null;
let samples = { white: null, gray50: null, gray18: null, black: null };
let fitted = false;

// ══ 拉流 ══
function liveLoop() {
  liveImg.onload = () => {
    frame = liveImg;
    redraw();
    setTimeout(liveLoop, 50);
  };
  liveImg.onerror = () => setTimeout(liveLoop, 300);
  liveImg.src = `/frame?t=${Date.now()}`;
}
liveLoop();

// ══ 鼠标拖框 ══
canvas.addEventListener('mousedown', (e) => {
  const rect = canvas.getBoundingClientRect();
  dragStart = {
    x: Math.round(e.clientX - rect.left),
    y: Math.round(e.clientY - rect.top),
  };
  selRect = null;
});

canvas.addEventListener('mousemove', (e) => {
  if (!dragStart) return;
  const rect = canvas.getBoundingClientRect();
  const x2 = Math.round(e.clientX - rect.left);
  const y2 = Math.round(e.clientY - rect.top);
  const x = Math.max(0, Math.min(dragStart.x, x2));
  const y = Math.max(0, Math.min(dragStart.y, y2));
  const w = Math.min(W - x, Math.abs(x2 - dragStart.x));
  const h = Math.min(H - y, Math.abs(y2 - dragStart.y));
  selRect = { x, y, w, h };
  redraw();
});

canvas.addEventListener('mouseup', (e) => {
  dragStart = null;
  if (selRect && selRect.w > 5 && selRect.h > 5) {
    refreshUI();
  } else {
    selRect = null;
    redraw();
  }
});

function redraw() {
  if (frame) ctx.drawImage(frame, 0, 0, W, H);
  drawSavedRect(samples.white,  '#ffffff', 'White90');
  drawSavedRect(samples.gray50, '#bbbbbb', 'Gray50');
  drawSavedRect(samples.gray18, '#777777', 'Gray18');
  drawSavedRect(samples.black,  '#444444', 'Black10');
  if (selRect) {
    ctx.strokeStyle = '#ff0';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(selRect.x, selRect.y, selRect.w, selRect.h);
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(255,255,0,0.15)';
    ctx.fillRect(selRect.x, selRect.y, selRect.w, selRect.h);
    ctx.fillStyle = '#ff0'; ctx.font = '12px monospace';
    ctx.fillText(`${selRect.w}x${selRect.h}`,
                 selRect.x + 4, selRect.y + 14);
  }
}

function drawSavedRect(s, color, label) {
  if (!s) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.strokeRect(s.x, s.y, s.w, s.h);
  ctx.fillStyle = color;
  ctx.font = 'bold 12px monospace';
  ctx.fillText(label, s.x + 4, s.y + 14);
}

// ══ 采样 ══
function sampleColor(color) {
  if (!selRect) {
    alert('请先在画面上拖框选择' + color + '卡区域');
    return;
  }
  const r = selRect;
  fetch(`/api/sample?color=${color}&x=${r.x}&y=${r.y}&w=${r.w}&h=${r.h}`)
    .then(res => res.json())
    .then(d => {
      if (d.ok) {
        samples[color] = { ...r, dn_r: d.dn_r, dn_nir: d.dn_nir };
        const labels = { white: 'w', gray50: 'g50', gray18: 'g18', black: 'k' };
        document.getElementById(`dn-r-${labels[color]}`).textContent = d.dn_r.toFixed(1);
        document.getElementById(`dn-r-${labels[color]}`).className = 'sampled';
        document.getElementById(`dn-n-${labels[color]}`).textContent = d.dn_nir.toFixed(1);
        document.getElementById(`dn-n-${labels[color]}`).className = 'sampled';
        selRect = null;
        redraw();
        refreshUI();
      } else {
        alert('采样失败: ' + (d.msg || ''));
      }
    });
}

function action(act) {
  fetch('/api/' + act).then(r => r.json()).then(d => {
    if (act === 'fit') {
      fitted = true;
      showFitResult(d);
    } else if (act === 'save') {
      document.getElementById('status').textContent =
        '✅ 已保存 (timestamp: ' + d.timestamp + ')，可重启 ndvi_node 看真NDVI效果';
    } else if (act === 'reset') {
      samples = { white: null, gray50: null, gray18: null, black: null };
      fitted = false;
      ['w', 'g50', 'g18', 'k'].forEach(c => {
        ['r', 'n'].forEach(ch => {
          const el = document.getElementById(`dn-${ch}-${c}`);
          el.textContent = '--';
          el.className = 'pending';
        });
      });
      document.getElementById('fit-result').innerHTML =
        '<div class="info">最小二乘拟合结果将显示在这里</div>';
      selRect = null;
      redraw();
    }
    refreshUI();
  });
}

function showFitResult(d) {
  const html = `
    <div style="color:#0f0;font-weight:bold;margin-bottom:6px">最小二乘拟合结果</div>
    <table class="table" style="width:100%">
      <tr><th>通道</th><th>k (斜率)</th><th>b (截距)</th><th>R²</th><th>评估</th></tr>
      <tr>
        <td class="row-label">R (红光)</td>
        <td>${d.k1.toFixed(6)}</td>
        <td>${d.b1.toFixed(6)}</td>
        <td>${d.r2_r.toFixed(4)}</td>
        <td>${qualityTag(d.r2_r)}</td>
      </tr>
      <tr>
        <td class="row-label">NIR (近红外)</td>
        <td>${d.k2.toFixed(6)}</td>
        <td>${d.b2.toFixed(6)}</td>
        <td>${d.r2_nir.toFixed(4)}</td>
        <td>${qualityTag(d.r2_nir)}</td>
      </tr>
    </table>
    <div class="info" style="margin-top:6px">
      公式: R_real = ${d.k1.toFixed(4)} × DN_R + ${d.b1.toFixed(4)}<br>
      公式: NIR_real = ${d.k2.toFixed(4)} × DN_NIR + ${d.b2.toFixed(4)}
    </div>`;
  document.getElementById('fit-result').innerHTML = html;
}

function qualityTag(r2) {
  if (r2 > 0.99) return '<span style="color:#0f0">优秀</span>';
  if (r2 > 0.95) return '<span style="color:#fa0">良好</span>';
  if (r2 > 0.85) return '<span style="color:#fa0">尚可</span>';
  return '<span style="color:#f55">差，建议重做</span>';
}

function refreshUI() {
  const has_sel = selRect && selRect.w > 5 && selRect.h > 5;
  document.getElementById('btn-w').disabled = !has_sel;
  document.getElementById('btn-g50').disabled = !has_sel;
  document.getElementById('btn-g18').disabled = !has_sel;
  document.getElementById('btn-k').disabled = !has_sel;
  const all_done = samples.white && samples.gray50 && samples.gray18 && samples.black;
  document.getElementById('btn-fit').disabled = !all_done;
  document.getElementById('btn-save').disabled = !fitted;

  setStage('st-w',   samples.white  ? 'done' : '');
  setStage('st-g50', samples.gray50 ? 'done' : '');
  setStage('st-g18', samples.gray18 ? 'done' : '');
  setStage('st-k',   samples.black  ? 'done' : '');
  setStage('st-s',   fitted ? 'active' : '');

  let tip;
  if (!samples.white)       tip = '→ 拖框圈选白卡(90%)区域，按 [1]';
  else if (!samples.gray50) tip = '→ 拖框圈选灰卡(50%)区域，按 [2]';
  else if (!samples.gray18) tip = '→ 拖框圈选灰卡(18%)区域，按 [3]';
  else if (!samples.black)  tip = '→ 拖框圈选黑卡(10%)区域，按 [4]';
  else if (!fitted)         tip = '→ 四色卡都已采样，按 [F] 拟合';
  else                      tip = '→ 按 [C] 保存到 calib_params.yaml';
  document.getElementById('status').textContent = tip;
}

function setStage(id, cls) {
  document.getElementById(id).className = 'stage' + (cls ? ' ' + cls : '');
}

document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === '1' && selRect) sampleColor('white');
  else if (e.key === '2' && selRect) sampleColor('gray50');
  else if (e.key === '3' && selRect) sampleColor('gray18');
  else if (e.key === '4' && selRect) sampleColor('black');
  else if (e.key === 'f') action('fit');
  else if (e.key === 'c' && fitted) action('save');
  else if (e.key === 'r') action('reset');
});

refreshUI();
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  最小二乘拟合（带 R² 评估）
# ══════════════════════════════════════════════════════════════
def linear_fit(x: np.ndarray, y: np.ndarray):
    """
    解 y = k*x + b，返回 (k, b, r2)
    r² = 1 - SS_res / SS_tot
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    n = len(x)
    if n < 2:
        return 1.0, 0.0, 0.0
    x_mean = x.mean()
    y_mean = y.mean()
    sxy = ((x - x_mean) * (y - y_mean)).sum()
    sxx = ((x - x_mean) ** 2).sum()
    if sxx == 0:
        return 1.0, 0.0, 0.0
    k = sxy / sxx
    b = y_mean - k * x_mean
    y_pred = k * x + b
    ss_res = ((y - y_pred) ** 2).sum()
    ss_tot = ((y - y_mean) ** 2).sum()
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return float(k), float(b), float(r2)


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class CalibReflectanceNode(Node):

    def __init__(self):
        super().__init__("calib_reflectance")
        self.bridge = CvBridge()

        self._rgb = None
        self._ir  = None
        self._lock = threading.Lock()

        # 加载已有的 Shift 用于 IR 区域对齐
        self.calib = load_calib()

        # 三色卡采样数据：{ "white": (DN_R, DN_NIR), ... }
        self.samples = {}

        # 拟合结果
        self.fit_result = None    # {k1, b1, k2, b2, r2_r, r2_nir}

        self.sub_rgb = self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)
        self.sub_ir  = self.create_subscription(Image, TOPIC_IR,  self._cb_ir,  10)

        self._start_http()

        log = self.get_logger().info
        log("═══════════════════════════════════════════════════════")
        log("  标定三：反射率定标 (真NDVI)  v3.5 (浏览器版)")
        log("═══════════════════════════════════════════════════════")
        log(f"  本机访问:  http://localhost:{CALIB_HTTP_PORT}")
        log(f"  远程访问:  http://<小车IP>:{CALIB_HTTP_PORT}")
        log("  色卡反射率:  白={:.0%} / 灰50={:.0%} / 灰18={:.0%} / 黑={:.0%}".format(
            PANEL_WHITE, PANEL_GRAY50, PANEL_GRAY18, PANEL_BLACK))
        log(f"  Shift_X={self.calib.shift_x:+d}, Shift_Y={self.calib.shift_y:+d} "
            "(用于 IR 区域对齐)")
        if self.calib.refl_calibrated:
            log(f"  上次标定时间:  {self.calib.refl_timestamp}")
            log("  ⚠️ 如果环境光变了，请重新标定")
        log("═══════════════════════════════════════════════════════")

    def _cb_rgb(self, msg: Image):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._rgb = f
        except Exception as e:
            self.get_logger().error(f"RGB 解码失败：{e}")

    def _cb_ir(self, msg: Image):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._ir = f
        except Exception as e:
            self.get_logger().error(f"IR 解码失败：{e}")

    def _get_frames(self):
        with self._lock:
            return (None if self._rgb is None else self._rgb.copy(),
                    None if self._ir is None else self._ir.copy())

    # ── 核心：在 RGB 框选区域同时采样 R 和 IR ─────────────
    def _sample_panel(self, x: int, y: int, w: int, h: int):
        """
        在 RGB 图上 (x,y,w,h) 区域 → R 通道均值
        在 IR 图上 同区域（按 Shift 平移对齐）→ IR 灰度均值
        """
        rgb, ir = self._get_frames()
        if rgb is None or ir is None:
            return None

        # RGB 区域裁剪 + 边界保护
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(rgb.shape[1], x + w), min(rgb.shape[0], y + h)
        if x2 <= x1 or y2 <= y1:
            return None
        rgb_roi = rgb[y1:y2, x1:x2]
        # OpenCV BGR 通道序：R 是第 2 通道
        dn_r = float(rgb_roi[:, :, 2].mean())

        # IR 区域：把 RGB 框按 Shift 平移到 IR 坐标系
        ir_x1 = max(0, x1 - self.calib.shift_x)
        ir_y1 = max(0, y1 - self.calib.shift_y)
        ir_x2 = min(ir.shape[1], x2 - self.calib.shift_x)
        ir_y2 = min(ir.shape[0], y2 - self.calib.shift_y)
        if ir_x2 <= ir_x1 or ir_y2 <= ir_y1:
            return None
        ir_roi = ir[ir_y1:ir_y2, ir_x1:ir_x2]
        ir_gray = cv2.cvtColor(ir_roi, cv2.COLOR_BGR2GRAY)
        dn_nir = float(ir_gray.mean())

        return dn_r, dn_nir

    # ── 拟合四组数据 ──────────────────────────────────────
    def _do_fit(self):
        needed = ("white", "gray50", "gray18", "black")
        if not all(c in self.samples for c in needed):
            return None
        # 真值 = config.py 里的反射率
        truth = np.array([PANEL_WHITE, PANEL_GRAY50, PANEL_GRAY18, PANEL_BLACK])
        dn_r = np.array([
            self.samples["white"][0],
            self.samples["gray50"][0],
            self.samples["gray18"][0],
            self.samples["black"][0],
        ])
        dn_nir = np.array([
            self.samples["white"][1],
            self.samples["gray50"][1],
            self.samples["gray18"][1],
            self.samples["black"][1],
        ])
        # 拟合：refl = k * dn + b
        k1, b1, r2_r   = linear_fit(dn_r,   truth)
        k2, b2, r2_nir = linear_fit(dn_nir, truth)
        return {
            "k1": k1, "b1": b1, "r2_r":   r2_r,
            "k2": k2, "b2": b2, "r2_nir": r2_nir,
        }

    # ── HTTP 服务 ────────────────────────────────────────
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
                    cv2.putText(blank, "Waiting for camera...", (60, 240),
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
                    rgb, _ = node._get_frames()
                    self._send_jpeg(rgb)
                    return

                if path == "/api/sample":
                    color = qs.get("color", "")
                    if color not in ("white", "gray50", "gray18", "black"):
                        self._send_json({"ok": False, "msg": "color 错误"}, 400)
                        return
                    try:
                        x = int(qs.get("x", "0")); y = int(qs.get("y", "0"))
                        w = int(qs.get("w", "0")); h = int(qs.get("h", "0"))
                    except ValueError:
                        self._send_json({"ok": False, "msg": "坐标格式错"}, 400)
                        return
                    res = node._sample_panel(x, y, w, h)
                    if res is None:
                        self._send_json({"ok": False, "msg": "采样失败（可能两路图未就绪或区域无效）"}, 400)
                        return
                    dn_r, dn_nir = res
                    node.samples[color] = (dn_r, dn_nir)
                    node.get_logger().info(
                        f"[{color:>5s} 卡] 区域=({x},{y}) {w}×{h}  "
                        f"DN_R={dn_r:.1f}  DN_NIR={dn_nir:.1f}"
                    )
                    self._send_json({
                        "ok": True, "color": color,
                        "dn_r": dn_r, "dn_nir": dn_nir,
                    })
                    return

                if path == "/api/fit":
                    fit = node._do_fit()
                    if fit is None:
                        self._send_json({"ok": False, "msg": "请先采样四色卡"}, 400)
                        return
                    node.fit_result = fit
                    log = node.get_logger().info
                    log("═══ 最小二乘拟合结果 (4 点) ═══")
                    log(f"  R 通道:   refl = {fit['k1']:.6f} × DN + {fit['b1']:.6f}  "
                        f"R²={fit['r2_r']:.4f}")
                    log(f"  NIR 通道: refl = {fit['k2']:.6f} × DN + {fit['b2']:.6f}  "
                        f"R²={fit['r2_nir']:.4f}")
                    if min(fit['r2_r'], fit['r2_nir']) < 0.85:
                        log("  ⚠️ R² 偏低，建议重新采样")
                    self._send_json({"ok": True, **fit})
                    return

                if path == "/api/save":
                    if node.fit_result is None:
                        self._send_json({"ok": False, "msg": "请先拟合"}, 400)
                        return
                    ts = datetime.datetime.now().isoformat(timespec="seconds")
                    save_calib({
                        "k1": node.fit_result["k1"],
                        "b1": node.fit_result["b1"],
                        "k2": node.fit_result["k2"],
                        "b2": node.fit_result["b2"],
                        "refl_calibrated": True,
                        "refl_timestamp":  ts,
                        "refl_r2_red":     node.fit_result["r2_r"],
                        "refl_r2_nir":     node.fit_result["r2_nir"],
                    })
                    node.get_logger().info(f"✅ 反射率定标已保存 (timestamp={ts})")
                    self._send_json({"ok": True, "timestamp": ts})
                    return

                if path == "/api/reset":
                    node.samples.clear()
                    node.fit_result = None
                    node.get_logger().info("已清空采样数据")
                    self._send_json({"ok": True})
                    return

                self.send_response(404); self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", CALIB_HTTP_PORT), Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = CalibReflectanceNode()
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
