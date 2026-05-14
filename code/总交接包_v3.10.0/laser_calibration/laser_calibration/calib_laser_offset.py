#!/usr/bin/env python3
"""
calib_laser_offset.py —— 标定二：激光偏移量（浏览器版+缩放+放大镜）  v3.9
==========================================================================
v3.9 重大变更（相对 v3.4/v3.8）：
  1. 画面源从 IR 切到 RGB（S4 实际是可见红激光，不是 IR）
  2. 光斑检测：IR 灰度阈值 → RGB HSV 红色双区间
  3. 烧痕检测：在 RGB 灰度通道里找暗斑（白纸 + 蓝紫激光 烧出焦黑点）
     物理：白纸 V≈230，焦黑斑 V<60，对比强烈
  4. Delta 现在是 RGB 坐标系下的偏移：
     Delta = burn_pos_rgb - spot_pos_rgb
     蓝紫激光真实落点 (RGB) = 红光斑 (RGB) + Delta
  5. 操作建议：用普通白纸代替感光纸（白底黑斑对比更稳）

计算 (Delta_X, Delta_Y)：
    蓝紫激光真实落点 = 红光斑坐标 + (Delta_X, Delta_Y)

硬件控制（SunriseRobot SDK，PWM 直驱）:
    S3 → 蓝紫高能激光（枪管，烧灼）
    S4 → 红激光（瞄准镜，定位，可见 650nm 左右）
    angle=180 ON, angle=0 OFF

前置:
    ros2 run laser_calibration stereo_camera

运行:
    ros2 run laser_calibration calib_laser

浏览器:
    http://localhost:8091         (本机)
    http://<小车IP>:8091           (远程)

操作流程：
    [0] 在工作面放一张白纸（普通 A4 即可），云台对准白纸中央
    [1] 开红激光(S4) + RGB 画面自动检测红光斑 → 点击锁定 Spot
    [2] 烧纸：红激光关、蓝紫激光开 1 秒、冷却 2 秒
    [3] 冻结画面 + 自动检测黑色烧痕 → 点击确认 Burn
    [C] 保存 Delta_X/Y（写入 ~/calib_params.yaml）
    任何时候按 [紧急停止] 或关闭终端 → 所有激光立即关闭

⚠️ 安全提示：
  - 蓝紫激光功率高，能烧灼物体，标定前清理周围易燃物
  - 不要直视激光，戴防护眼镜
  - 烧前会有 3 秒倒计时，如发现异常立即按【紧急停止】
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

from laser_calibration.calib_io import save_calib
from laser_calibration.config import (
    BURN_AREA_MAX, BURN_AREA_MIN, BURN_THRESHOLD,
    RED_DOMINANCE_MIN, RED_SPOT_AREA_MAX, RED_SPOT_AREA_MIN,
    SPOT_CLOSE_KERNEL_SIZE, SPOT_ROI_SIZE,
    TOPIC_RGB,
)
from laser_calibration.robot_ctrl import (
    ROBOT_OK,
    all_lasers_off, center_servo, fire_blue_pulse,
    laser_blue, laser_ir,
)

CALIB_HTTP_PORT = 8091   # 注意和标定一的 8090 不同，避免端口冲突


# ══════════════════════════════════════════════════════════════
#  v3.9.5: R-max(G,B) + ROI 红光斑检测（与 vision_servo 同算法）
# ══════════════════════════════════════════════════════════════
def find_red_spot(bgr: np.ndarray, hint_x: int = None, hint_y: int = None):
    """红激光光斑检测，返回全图坐标 (cx, cy) 或 None。

    标定二节点里通常用全图搜索（hint 都为 None）—— 标定时用户
    会调整云台让光斑出现在画面中央，不需要 ROI 加速。
    """
    if bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]

    if hint_x is not None and hint_y is not None:
        half = SPOT_ROI_SIZE // 2
        y1 = max(0, int(hint_y) - half)
        y2 = min(h, int(hint_y) + half)
        x1 = max(0, int(hint_x) - half)
        x2 = min(w, int(hint_x) + half)
    else:
        y1, y2, x1, x2 = 0, h, 0, w

    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    b, g, r = cv2.split(roi)
    r_i = r.astype(np.int16)
    g_i = g.astype(np.int16)
    b_i = b.astype(np.int16)
    max_gb = np.maximum(g_i, b_i)
    red_score = np.clip(r_i - max_gb, 0, 255).astype(np.uint8)

    _, mask = cv2.threshold(red_score, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)
    kernel = np.ones((SPOT_CLOSE_KERNEL_SIZE, SPOT_CLOSE_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

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
    return int(x1 + M["m10"] / M["m00"]), int(y1 + M["m01"] / M["m00"])


def find_burn_mark(gray: np.ndarray):
    _, th = cv2.threshold(gray, BURN_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    k = np.ones((5, 5), np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, k)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, k)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_a = None, 0
    for c in cnts:
        a = cv2.contourArea(c)
        if BURN_AREA_MIN < a < BURN_AREA_MAX and a > best_a:
            best, best_a = c, a
    if best is None:
        return None
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


# ══════════════════════════════════════════════════════════════
#  HTML 页面（带缩放/平移/放大镜 + 激光控制）
# ══════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>标定二 · 激光偏移量标定 (v3.9 RGB)</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1 { color:#0f0; margin:0 0 12px; font-size:18px; }
  .panel { background:#222; padding:12px; border-radius:8px; margin-bottom:12px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px;
         margin-bottom:4px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .btn-fire { background:#3a1a1a; color:#f55; border-color:#f55; }
  .btn-fire:hover { background:#f55; color:#000; }
  .btn-stop { background:#3a0808; color:#ff0; border-color:#ff0; font-weight:bold; }
  .btn-stop:hover { background:#ff0; color:#000; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; overflow:hidden;
             width:640px; height:480px; }
  .cam-box.active { border-color:#0f0; }
  .cam-box.frozen { border-color:#fa0; }
  .cam-box.live canvas { cursor:default; }
  .cam-box.frozen canvas { cursor:crosshair; }
  .cam-box.frozen.panning canvas { cursor:grabbing; }
  .cam-label { position:absolute; top:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#0f0; font-size:12px; z-index:10;
               pointer-events:none; }
  .zoom-info { position:absolute; bottom:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#fa0; font-size:11px; z-index:10;
               pointer-events:none; }
  canvas { display:block; }
  .loupe { position:absolute; width:160px; height:160px;
           border:2px solid #0f0; background:#000;
           pointer-events:none; z-index:20;
           box-shadow:0 4px 12px rgba(0,0,0,0.7); }
  .status { color:#fa0; font-size:14px; padding:8px; min-height:18px; }
  .info { color:#888; font-size:12px; margin-top:8px; }
  .v { color:#0f0; }
  .danger { color:#f55; }
  .step { display:inline-block; padding:2px 8px; border:1px solid #555;
          border-radius:3px; margin-right:6px; }
  .step.done { color:#0f0; border-color:#0f0; }
  .step.active { color:#fa0; border-color:#fa0; background:#332200; }
  .laser-state { display:inline-block; padding:4px 10px; border-radius:3px;
                 margin-left:10px; font-size:13px; }
  .laser-on { background:#330; color:#ff0; border:1px solid #ff0; }
  .laser-off { background:#222; color:#666; border:1px solid #666; }
  .laser-fire { background:#600; color:#ff8; border:1px solid #f55;
                animation:blink 0.4s linear infinite; }
  @keyframes blink { 50% { opacity:0.4; } }
</style>
</head>
<body>
<h1>🎯 标定二 · 激光偏移量标定 (v3.9 · RGB+HSV 红光斑)</h1>

<div class="panel">
  <span class="step" id="step1">① 锁定光斑</span>
  <span class="step" id="step2">② 烧纸</span>
  <span class="step" id="step3">③ 锁定烧痕</span>
  <span class="step" id="step4">④ 保存</span>
  <span class="laser-state laser-off" id="ls-ir">S4 RED: OFF</span>
  <span class="laser-state laser-off" id="ls-blue">S3 BLUE: OFF</span>
</div>

<div class="panel">
  <button class="btn" id="btn-1" onclick="action('start_spot')">[1] 开红激光检测光斑</button>
  <button class="btn btn-fire" id="btn-2" onclick="confirmFire()" disabled>[2] 烧纸 1 秒</button>
  <button class="btn" id="btn-3" onclick="action('detect_burn')" disabled>[3] 冻结检测烧痕</button>
  <button class="btn" id="btn-save" onclick="action('save')" disabled>[C] 保存</button>
  <button class="btn" id="btn-reset" onclick="action('reset')">[R] 重做</button>
  <button class="btn btn-stop" onclick="action('stop')">[紧急停止]</button>
  <span style="margin-left:20px">
    <button class="btn" onclick="zoom(1.5)">[+]</button>
    <button class="btn" onclick="zoom(1/1.5)">[-]</button>
    <button class="btn" onclick="resetView()">[复位]</button>
  </span>
  <div class="status" id="status">→ 把白纸放在工作面，按 [1] 开始</div>
</div>

<div class="cam-box live" id="box-ir">
  <span class="cam-label">RGB 画面</span>
  <span class="zoom-info" id="zoom-info">1.00x</span>
  <canvas id="cv-ir" width="640" height="480"></canvas>
</div>

<div class="info">
  <span class="danger">⚠️ 安全提示</span>:
  蓝紫激光会灼烧物体，使用时勿对人/动物，远离易燃物，附近备好灭火工具
</div>
<div class="info">
  快捷键: <span class="v">1</span>开IR · <span class="v">2</span>烧纸 ·
  <span class="v">3</span>检测烧痕 · <span class="v">C</span>保存 ·
  <span class="v">R</span>重做 · 滚轮缩放 · 拖动平移
</div>
<div class="info" id="result"></div>

<script>
const W = 640, H = 480;
let state = 'IDLE';            // IDLE | DETECT_SPOT | SPOT_LOCKED | FIRING | DETECT_BURN | DONE
let spotPt = null;             // 红激光光斑（RGB 坐标）
let burnPt = null;             // 烧痕（IR图坐标）
let autoSpot = null;           // 自动检测出的光斑（实时）
let autoBurn = null;           // 自动检测出的烧痕（冻结后）

const canvas = document.getElementById('cv-ir');
const ctx = canvas.getContext('2d');
const box = document.getElementById('box-ir');
const zoomInfo = document.getElementById('zoom-info');
let frozenImg = null;
let scale = 1, tx = 0, ty = 0;
let dragStart = null;
let liveImg = new Image();
let loupe = null;

// ══ 缩放/平移 ══
canvas.addEventListener('wheel', (e) => {
  if (state !== 'DETECT_SPOT' && state !== 'DETECT_BURN') return;
  e.preventDefault();
  const factor = e.deltaY < 0 ? 1.2 : 1/1.2;
  const rect = canvas.getBoundingClientRect();
  zoomAt(factor, e.clientX - rect.left, e.clientY - rect.top);
}, { passive: false });

canvas.addEventListener('mousedown', (e) => {
  if (state !== 'DETECT_SPOT' && state !== 'DETECT_BURN') return;
  dragStart = { mx:e.clientX, my:e.clientY, tx:tx, ty:ty, moved:false };
});
canvas.addEventListener('mousemove', (e) => {
  if (dragStart) {
    const dx = e.clientX - dragStart.mx;
    const dy = e.clientY - dragStart.my;
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
      dragStart.moved = true;
      tx = dragStart.tx + dx; ty = dragStart.ty + dy;
      box.classList.add('panning');
      redraw();
    }
  }
  if (state === 'DETECT_BURN' && frozenImg) showLoupe(e);
});
canvas.addEventListener('mouseup', (e) => {
  if (!dragStart) return;
  const wasMove = dragStart.moved;
  dragStart = null;
  box.classList.remove('panning');
  if (!wasMove) handleClick(e);
});
canvas.addEventListener('mouseleave', () => {
  dragStart = null;
  box.classList.remove('panning');
  hideLoupe();
});

function zoomAt(factor, sx, sy) {
  const newScale = Math.max(0.5, Math.min(8, scale * factor));
  const realFactor = newScale / scale;
  tx = sx - (sx - tx) * realFactor;
  ty = sy - (sy - ty) * realFactor;
  scale = newScale;
  zoomInfo.textContent = scale.toFixed(2) + 'x';
  redraw();
}
function zoom(f) { zoomAt(f, W/2, H/2); }
function resetView() { scale = 1; tx = 0; ty = 0; zoomInfo.textContent = '1.00x'; redraw(); }

function screenToImage(sx, sy) {
  return {
    x: Math.round((sx - tx) / scale),
    y: Math.round((sy - ty) / scale),
  };
}

function handleClick(e) {
  const rect = canvas.getBoundingClientRect();
  const pt = screenToImage(e.clientX - rect.left, e.clientY - rect.top);
  if (pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H) return;

  if (state === 'DETECT_SPOT') {
    spotPt = pt;
    state = 'SPOT_LOCKED';
    fetch(`/api/lock_spot?x=${pt.x}&y=${pt.y}`);
    refreshUI();
  } else if (state === 'DETECT_BURN') {
    burnPt = pt;
    state = 'DONE';
    fetch(`/api/lock_burn?x=${pt.x}&y=${pt.y}`);
    refreshUI();
  }
}

// ══ 实时画面拉流 ══
function liveLoop() {
  if (state === 'DETECT_BURN' || state === 'DONE') {
    setTimeout(liveLoop, 200);
    return;
  }
  liveImg.onload = () => {
    if (state !== 'DETECT_BURN' && state !== 'DONE') {
      // v3.8: 支持缩放，按 scale/tx/ty 绘制
      ctx.fillStyle = '#000';
      ctx.fillRect(0, 0, W, H);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(liveImg, tx, ty, W * scale, H * scale);
      if (state === 'DETECT_SPOT') {
        fetch('/api/auto_spot').then(r => r.json()).then(d => {
          autoSpot = d.spot;
          if (autoSpot) drawAutoMark(autoSpot, '#ff0', 'auto');
        }).catch(() => {});
      }
      if (spotPt) drawCross(spotPt, '#0f0', 'Spot');
    }
    setTimeout(liveLoop, 100);
  };
  liveImg.onerror = () => setTimeout(liveLoop, 300);
  liveImg.src = `/frame?t=${Date.now()}`;
}
liveLoop();

// ══ 冻结画面 ══
function loadFrozen() {
  const img = new Image();
  img.onload = () => {
    frozenImg = img;
    box.classList.remove('live');
    box.classList.add('frozen');
    redraw();
    // 启动后自动查询一次烧痕检测
    fetch('/api/auto_burn').then(r => r.json()).then(d => {
      autoBurn = d.burn;
      redraw();
    }).catch(() => {});
  };
  img.src = `/frozen?t=${Date.now()}`;
}

function unfreeze() {
  frozenImg = null;
  box.classList.remove('frozen');
  box.classList.add('live');
  scale = 1; tx = 0; ty = 0;
  zoomInfo.textContent = '1.00x';
  hideLoupe();
}

function redraw() {
  if (frozenImg) {
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(frozenImg, tx, ty, W*scale, H*scale);
    if (autoBurn && state === 'DETECT_BURN') {
      drawAutoMark(autoBurn, '#ff0', 'auto');
    }
    if (spotPt) drawCross(spotPt, '#0f0', 'Spot');
    if (burnPt) drawCross(burnPt, '#f80', 'Burn');
  }
}

function drawCross(pt, color, label) {
  const sx = pt.x * scale + tx;
  const sy = pt.y * scale + ty;
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(sx-24, sy); ctx.lineTo(sx+24, sy);
  ctx.moveTo(sx, sy-24); ctx.lineTo(sx, sy+24);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(sx, sy, 16, 0, Math.PI*2); ctx.stroke();
  ctx.fillStyle = color; ctx.font = '13px monospace';
  ctx.fillText(`${label}(${pt.x},${pt.y})`, sx+20, sy-14);
}

function drawAutoMark(pt, color, label) {
  const sx = pt.x * scale + tx;
  const sy = pt.y * scale + ty;
  ctx.strokeStyle = color; ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(sx, sy, 22, 0, Math.PI*2); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color; ctx.font = '11px monospace';
  ctx.fillText(`${label}(${pt.x},${pt.y})`, sx+24, sy+4);
}

// ══ 放大镜 ══
function showLoupe(e) {
  if (!loupe) {
    loupe = document.createElement('canvas');
    loupe.width = 160; loupe.height = 160;
    loupe.className = 'loupe';
    box.appendChild(loupe);
  }
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;
  const pt = screenToImage(sx, sy);
  if (pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H) {
    hideLoupe(); return;
  }
  const lc = loupe.getContext('2d');
  lc.imageSmoothingEnabled = false;
  lc.fillStyle = '#000'; lc.fillRect(0, 0, 160, 160);
  const half = 20;
  lc.drawImage(frozenImg, pt.x-half, pt.y-half, 2*half, 2*half, 0, 0, 160, 160);
  lc.strokeStyle = '#0f0'; lc.lineWidth = 1;
  lc.beginPath();
  lc.moveTo(80, 60); lc.lineTo(80, 100);
  lc.moveTo(60, 80); lc.lineTo(100, 80);
  lc.stroke();
  lc.fillStyle = '#0f0'; lc.font = '11px monospace';
  lc.fillText(`(${pt.x},${pt.y})`, 6, 152);
  loupe.style.right = '8px';
  loupe.style.top = '8px';
}
function hideLoupe() {
  if (loupe) { loupe.remove(); loupe = null; }
}

// ══ 动作 ══
function confirmFire() {
  if (!confirm('⚠️ 即将开启蓝紫激光烧纸 1 秒！\n请确认:\n· 白纸已放置正确，云台对准白纸中央\n· 激光路径无人/无易燃物\n· 已戴防护眼镜\n· 红光斑(Spot)已锁定')) {
    return;
  }
  action('fire');
}

function action(act) {
  fetch('/api/' + act).then(r => r.json()).then(d => {
    if (act === 'start_spot') {
      state = 'DETECT_SPOT';
      spotPt = null; burnPt = null;
    } else if (act === 'fire') {
      state = 'FIRING';
    } else if (act === 'detect_burn') {
      state = 'DETECT_BURN';
      loadFrozen();
    } else if (act === 'save') {
      state = 'SAVED';
      document.getElementById('result').innerHTML =
        `✅ 已保存: Delta_X=<span class="v">${d.delta_x>=0?'+':''}${d.delta_x}</span>` +
        ` Delta_Y=<span class="v">${d.delta_y>=0?'+':''}${d.delta_y}</span>`;
    } else if (act === 'reset' || act === 'stop') {
      state = 'IDLE';
      spotPt = null; burnPt = null;
      unfreeze();
    }
    refreshUI();
  });
}

function refreshUI() {
  document.getElementById('btn-1').disabled = !(state === 'IDLE' || state === 'DETECT_SPOT');
  document.getElementById('btn-2').disabled = (state !== 'SPOT_LOCKED');
  document.getElementById('btn-3').disabled = (state !== 'SPOT_LOCKED' && state !== 'DETECT_BURN' && state !== 'FIRING_DONE');
  document.getElementById('btn-save').disabled = (state !== 'DONE');

  // 步骤指示
  setStep('step1', spotPt ? 'done' : (state === 'DETECT_SPOT' ? 'active' : ''));
  setStep('step2', state === 'FIRING_DONE' || state === 'DETECT_BURN' || state === 'DONE' || state === 'SAVED' ? 'done' :
                   (state === 'FIRING' ? 'active' : ''));
  setStep('step3', burnPt ? 'done' : (state === 'DETECT_BURN' ? 'active' : ''));
  setStep('step4', state === 'SAVED' ? 'done' : (state === 'DONE' ? 'active' : ''));

  const tip = {
    'IDLE':         '→ 把白纸放在工作面，按 [1] 开红激光检测光斑',
    'DETECT_SPOT':  '→ 看到红光斑(黄圈=自动检测)，鼠标点击锁定 Spot',
    'SPOT_LOCKED':  `→ Spot=(${spotPt?spotPt.x:''},${spotPt?spotPt.y:''}) 已锁定，按 [2] 烧纸`,
    'FIRING':       '🔥 蓝紫激光开火中... 1秒+冷却2秒 (期间不可操作)',
    'FIRING_DONE':  '→ 烧纸完成，按 [3] 冻结画面检测黑色烧痕',
    'DETECT_BURN':  '→ 滚轮放大画面，鼠标点击烧痕中心 (黄圈=自动检测)',
    'DONE':         `→ Burn=(${burnPt?burnPt.x:''},${burnPt?burnPt.y:''}) 已锁定，按 [C] 保存`,
    'SAVED':        '✅ 标定二完成！(RGB 坐标系下)',
  };
  document.getElementById('status').textContent = tip[state] || '';
}

function setStep(id, cls) {
  const el = document.getElementById(id);
  el.className = 'step' + (cls ? ' ' + cls : '');
}

// ══ 激光状态轮询 ══
setInterval(() => {
  fetch('/api/laser_state').then(r => r.json()).then(d => {
    setLaserUI('ls-ir', d.ir);
    setLaserUI('ls-blue', d.blue);
    if (d.fire_done && state === 'FIRING') {
      state = 'FIRING_DONE';
      refreshUI();
    }
  }).catch(() => {});
}, 500);

function setLaserUI(id, st) {
  const el = document.getElementById(id);
  const cls = st === 'fire' ? 'laser-fire' : (st === 'on' ? 'laser-on' : 'laser-off');
  el.className = 'laser-state ' + cls;
  const name = id === 'ls-ir' ? 'S4 RED' : 'S3 BLUE';
  el.textContent = `${name}: ${st.toUpperCase()}`;
}

document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === '1') action('start_spot');
  else if (e.key === '2' && state === 'SPOT_LOCKED') confirmFire();
  else if (e.key === '3' && (state === 'SPOT_LOCKED' || state === 'FIRING_DONE')) action('detect_burn');
  else if (e.key === 'c' && state === 'DONE') action('save');
  else if (e.key === 'r') action('reset');
});

refreshUI();
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class CalibLaserOffsetNode(Node):

    def __init__(self):
        super().__init__("calib_laser_offset")
        self.bridge = CvBridge()

        # v3.9: 切到 RGB
        self._rgb = None
        self._lock = threading.Lock()

        self.spot_pt = None
        self.burn_pt = None
        self.frozen_rgb = None
        self.state = "IDLE"

        # 激光状态用于网页显示
        self.ir_state = "off"      # off | on  (S4 红激光)
        self.blue_state = "off"    # off | on | fire
        self.fire_done = False

        # v3.9: 订阅 RGB 而不是 IR
        self.sub_rgb = self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)

        # 上电安全
        all_lasers_off()
        self.get_logger().info("云台归中中...")
        center_servo()
        time.sleep(0.5)

        self._start_http()

        log = self.get_logger().info
        log("═══════════════════════════════════════════════════════")
        log("  标定二：激光偏移量标定  v3.9 (RGB+HSV 红光斑)")
        log("═══════════════════════════════════════════════════════")
        log(f"  SDK 状态:  {'✅ 已连接' if ROBOT_OK else '❌ 未连接（仅模拟）'}")
        log(f"  画面源:    RGB ({TOPIC_RGB})")
        log(f"  本机访问:  http://localhost:{CALIB_HTTP_PORT}")
        log(f"  远程访问:  http://<小车IP>:{CALIB_HTTP_PORT}")
        log("  操作: 在工作面放白纸 → 点[1]检测红光斑 → 点[2]烧纸 → 点[3]检测烧痕 → 点[C]保存")
        log("  按 Ctrl+C 退出节点（自动关闭所有激光）")
        log("═══════════════════════════════════════════════════════")

    def _cb_rgb(self, msg: Image):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._rgb = f
        except Exception as e:
            self.get_logger().error(f"RGB 解码失败：{e}")

    def _get_rgb(self):
        with self._lock:
            return None if self._rgb is None else self._rgb.copy()

    def _detect_spot_now(self):
        """v3.9: 在 RGB 画面里 HSV 检测红光斑"""
        rgb = self._get_rgb()
        if rgb is None:
            return None
        return find_red_spot(rgb)

    def _detect_burn_now(self):
        """v3.9: 在冻结 RGB 画面的灰度通道里检测黑色烧痕"""
        if self.frozen_rgb is None:
            return None
        gray = cv2.cvtColor(self.frozen_rgb, cv2.COLOR_BGR2GRAY)
        return find_burn_mark(gray)

    # ── 激光控制（带状态记录）─────────────────────────
    def _set_ir(self, on: bool):
        laser_ir(on)
        self.ir_state = "on" if on else "off"

    def _fire_blue_thread(self, duration: float = 1.0):
        """异步执行：S4 关 → S3 开 1 秒 → S3 关 → 冷却 2 秒 → fire_done=True"""
        self._set_ir(False)
        self.get_logger().info("⚡ 蓝紫激光(S3) ON → 烧纸中...")
        self.blue_state = "fire"
        laser_blue(True)
        time.sleep(duration)
        laser_blue(False)
        self.blue_state = "off"
        self.get_logger().info("   蓝紫激光 OFF，冷却 2 秒...")
        time.sleep(2.0)
        self.fire_done = True
        self.get_logger().info("   冷却完毕 → 浏览器中按 [3] 检测烧痕")

    # ── HTTP 服务 ────────────────────────────────────────────
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
                    cv2.putText(blank, "Waiting for IR camera...", (60, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 200), 2)
                    frame = blank
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
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

                if path == "/frozen":
                    self._send_jpeg(node.frozen_rgb)
                    return

                # 实时光斑自动检测
                if path == "/api/auto_spot":
                    pt = node._detect_spot_now()
                    self._send_json({"spot": {"x": pt[0], "y": pt[1]} if pt else None})
                    return

                # 烧痕自动检测（冻结后）
                if path == "/api/auto_burn":
                    pt = node._detect_burn_now()
                    self._send_json({"burn": {"x": pt[0], "y": pt[1]} if pt else None})
                    return

                # 激光状态查询
                if path == "/api/laser_state":
                    fd = node.fire_done
                    if fd:
                        node.fire_done = False  # 取一次后清零
                    self._send_json({
                        "ir": node.ir_state,
                        "blue": node.blue_state,
                        "fire_done": fd,
                    })
                    return

                # 开始检测光斑：开 S4 红激光
                if path == "/api/start_spot":
                    node._set_ir(True)
                    node.state = "DETECT_SPOT"
                    node.spot_pt = None
                    node.burn_pt = None
                    node.frozen_rgb = None
                    node.get_logger().info("红激光(S4) ON → 进入光斑检测")
                    self._send_json({"ok": True})
                    return

                # 锁定光斑
                if path == "/api/lock_spot":
                    try:
                        x = int(qs.get("x", "0")); y = int(qs.get("y", "0"))
                    except ValueError:
                        x = y = 0
                    node.spot_pt = (x, y)
                    node.state = "SPOT_LOCKED"
                    node.get_logger().info(f"✓ Spot 锁定: ({x}, {y})")
                    self._send_json({"ok": True, "spot": {"x": x, "y": y}})
                    return

                # 烧纸（异步触发）
                if path == "/api/fire":
                    if node.state != "SPOT_LOCKED":
                        self._send_json({"ok": False, "msg": "请先锁定光斑"}, 400)
                        return
                    node.state = "FIRING"
                    threading.Thread(
                        target=node._fire_blue_thread, args=(1.0,), daemon=False
                    ).start()
                    self._send_json({"ok": True})
                    return

                # 冻结画面进入烧痕检测
                if path == "/api/detect_burn":
                    rgb = node._get_rgb()
                    if rgb is None:
                        self._send_json({"ok": False, "msg": "无 RGB 画面"}, 400)
                        return
                    node.frozen_rgb = rgb.copy()
                    node.state = "DETECT_BURN"
                    node.get_logger().info("画面已冻结 → 进入烧痕检测")
                    self._send_json({"ok": True})
                    return

                # 锁定烧痕
                if path == "/api/lock_burn":
                    try:
                        x = int(qs.get("x", "0")); y = int(qs.get("y", "0"))
                    except ValueError:
                        x = y = 0
                    node.burn_pt = (x, y)
                    node.state = "DONE"
                    if node.spot_pt:
                        dx = x - node.spot_pt[0]
                        dy = y - node.spot_pt[1]
                        node.get_logger().info(f"✓ Burn 锁定: ({x}, {y})")
                        node.get_logger().info(
                            f"  → Delta_X = {x} - {node.spot_pt[0]} = {dx:+d}"
                        )
                        node.get_logger().info(
                            f"  → Delta_Y = {y} - {node.spot_pt[1]} = {dy:+d}"
                        )
                    self._send_json({"ok": True, "burn": {"x": x, "y": y}})
                    return

                # 保存
                if path == "/api/save":
                    if node.state != "DONE" or not node.spot_pt or not node.burn_pt:
                        self._send_json({"ok": False, "msg": "请先完成两步检测"}, 400)
                        return
                    dx = node.burn_pt[0] - node.spot_pt[0]
                    dy = node.burn_pt[1] - node.spot_pt[1]
                    save_calib({
                        "delta_x": int(dx),
                        "delta_y": int(dy),
                        "spot_pos": list(node.spot_pt),
                        "burn_pos": list(node.burn_pt),
                        "calib2_done": True,
                        "calib2_frame": "rgb",   # v3.9.1: 标记 RGB 坐标系
                    })
                    node.state = "SAVED"
                    node.get_logger().info(
                        f"✅ 标定二完成！Delta_X={dx:+d}, Delta_Y={dy:+d}"
                    )
                    self._send_json({"ok": True, "delta_x": dx, "delta_y": dy})
                    return

                # 重置
                if path == "/api/reset":
                    all_lasers_off()
                    node.ir_state = "off"
                    node.blue_state = "off"
                    node.spot_pt = None
                    node.burn_pt = None
                    node.frozen_rgb = None
                    node.state = "IDLE"
                    node.get_logger().info("已重置，所有激光关闭")
                    self._send_json({"ok": True})
                    return

                # 紧急停止
                if path == "/api/stop":
                    all_lasers_off()
                    node.ir_state = "off"
                    node.blue_state = "off"
                    node.state = "IDLE"
                    node.get_logger().warn("🛑 紧急停止：所有激光关闭")
                    self._send_json({"ok": True})
                    return

                self.send_response(404); self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", CALIB_HTTP_PORT), Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = CalibLaserOffsetNode()
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
