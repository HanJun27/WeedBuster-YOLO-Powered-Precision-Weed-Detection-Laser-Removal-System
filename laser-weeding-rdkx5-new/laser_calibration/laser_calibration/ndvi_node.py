#!/usr/bin/env python3
"""
ndvi_node.py —— NDVI 植物健康检测  v3.10
=========================================

v3.10 改动（相对 v3.0）：
  ★ 新增 active diffuse mode (标定四) 优先级最高的真 NDVI 计算路径：
      NDVI = (K·NIR' − R') / (K·NIR' + R')  with R/NIR 减暗电流
    走 calib.active_ndvi() 这个新方法，物理意义最准
  ★ 三层优先级（自动选择）：
      1. calib4_done  → active mode (calib_diffuse 标定的真 NDVI)
      2. refl_calibrated → v3.0 4 点色卡反射率定标
      3. else         → 伪 NDVI (兜底)
  ★ 健康三分级 (config.NDVI_HEALTHY_MIN / MODERATE_MIN / PLANT_MIN)
  ★ 灰卡 ROI 输出端遮挡（GRAY_ROI_MASK_ON_OUTPUT）：
    系统内部用灰卡 ROI 做实时校验，但演示画面把它涂黑遮掉
  ★ 网页 UI 显示当前标定模式 (active / refl / pseudo)

原理：
    NDVI = (NIR - R) / (NIR + R)
    NIR  = IR 摄像头灰度值（近红外反射）
    R    = RGB 摄像头红色通道（可见光红）
    健康植物：叶绿素强烈吸收红光、强烈反射近红外 → NDVI 高（绿色）
    枯死/病害：反射率下降 → NDVI 低（红色/黄色）

输出：
    1. MJPEG 网络流 http://<车IP>:8082  NDVI 伪彩色热力图
    2. ROS topic /ndvi/image    sensor_msgs/Image
    3. ROS topic /ndvi/result   std_msgs/String (JSON, 含分级和占比)

运行：
    ros2 run laser_calibration ndvi_node
前置：
    ros2 run laser_calibration stereo_camera
推荐：
    ros2 run laser_calibration calib_diffuse  （先做标定四，否则只有伪 NDVI）
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
    CAM_FPS, STREAM_QUALITY,
    NDVI_DEFAULT_MODE, NDVI_PLANT_THRESHOLD,
    NDVI_HEALTHY_MIN, NDVI_MODERATE_MIN, NDVI_PLANT_MIN,
    ACTIVE_MODE_DEFAULT,
    GRAY_ROI_MASK_ON_OUTPUT,
    TOPIC_IR, TOPIC_RGB, TOPIC_YOLO,
)

NDVI_STREAM_PORT = 8082


# ══════════════════════════════════════════════════════════════
#  NDVI 计算 + 配色 LUT
# ══════════════════════════════════════════════════════════════
def _build_ndvi_lut() -> np.ndarray:
    """红→黄→绿 LUT。0→红，128→黄，255→绿"""
    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        if i < 128:
            lut[i] = [0, i * 2, 255]
        else:
            lut[i] = [0, 255, 255 - (i - 128) * 2]
    return lut

_NDVI_LUT = _build_ndvi_lut()


def compute_ndvi(rgb_bgr: np.ndarray, ir_gray: np.ndarray,
                 calib=None, prefer_active: bool = True) -> tuple:
    """
    计算逐像素 NDVI 矩阵。返回 (ndvi_map, mode_tag)。

    优先级（v3.10）：
      1. active mode  (calib4_done & prefer_active)  → 真 NDVI
      2. refl mode    (refl_calibrated)              → 真 NDVI
      3. pseudo mode  (兜底)                          → 伪 NDVI

    ★ 关键①：强转 float32！否则 uint8 减法会溢出或截断。
    ★ 关键②：active 模式下减暗电流 + K 修正后，距离/材质效应消除。
    """
    R = rgb_bgr[:, :, 2].astype(np.float32)   # OpenCV BGR：R 是第 2 通道
    NIR = ir_gray.astype(np.float32)

    if (calib is not None and prefer_active and
            getattr(calib, "calib4_done", False)):
        # 优先级 1: active mode (主动光源 + 灰卡 K 修正)
        ndvi = calib.active_ndvi(R, NIR)
        return ndvi, "ACTIVE"

    if calib is not None and getattr(calib, "refl_calibrated", False):
        # 优先级 2: 老的 4 点色卡反射率定标
        R_real = calib.k1 * R + calib.b1
        NIR_real = calib.k2 * NIR + calib.b2
        R_real = np.clip(R_real, 0.0, 2.0)
        NIR_real = np.clip(NIR_real, 0.0, 2.0)
        ndvi = (NIR_real - R_real) / (NIR_real + R_real + 1e-5)
        return np.clip(ndvi, -1.0, 1.0), "REFL"

    # 优先级 3: 伪 NDVI 兜底
    ndvi = (NIR - R) / (NIR + R + 1e-5)
    return np.clip(ndvi, -1.0, 1.0), "PSEUDO"


def ndvi_to_colormap(ndvi: np.ndarray) -> np.ndarray:
    """NDVI [-1,1] → BGR 伪彩色图（红=枯死/裸土，绿=健康）"""
    norm = ((ndvi + 1.0) / 2.0 * 255).astype(np.uint8)
    return _NDVI_LUT[norm]


def render_ndvi(ndvi: np.ndarray, rgb_bgr: np.ndarray,
                mode: str, threshold: float = 0.25) -> np.ndarray:
    """
    根据模式渲染 NDVI 可视化图。
        colormap   —— 全图染红黄绿渐变
        mask_solid —— 非植物完全灰度，仅植物染色
        mask_blend —— 非植物保留淡色，植物高亮
    """
    if mode == "colormap":
        return ndvi_to_colormap(ndvi)

    plant_mask = (ndvi > threshold).astype(np.uint8)

    gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    plant_color = ndvi_to_colormap(ndvi)

    if mode == "mask_solid":
        m3 = plant_mask[..., None]
        out = gray_bgr * (1 - m3) + plant_color * m3
        return out.astype(np.uint8)

    if mode == "mask_blend":
        non_plant = (gray_bgr.astype(np.float32) * 0.6 +
                     rgb_bgr.astype(np.float32) * 0.4).astype(np.uint8)
        m3 = plant_mask[..., None]
        out = non_plant * (1 - m3) + plant_color * m3
        return out.astype(np.uint8)

    return ndvi_to_colormap(ndvi)


def ndvi_health_label(ndvi_value: float) -> str:
    """
    v3.10 三级分级（配合 active mode 真 NDVI 阈值）：
      ndvi >= NDVI_HEALTHY_MIN  → healthy（深绿）
      ndvi >= NDVI_MODERATE_MIN → moderate（黄绿，亚健康）
      ndvi >= NDVI_PLANT_MIN    → stressed（仍是植物，但严重亚健康/枯萎）
      ndvi <  NDVI_PLANT_MIN    → non-plant（泥土 / 水泥 / 背景）
    """
    if ndvi_value >= NDVI_HEALTHY_MIN:
        return "healthy"
    if ndvi_value >= NDVI_MODERATE_MIN:
        return "moderate"
    if ndvi_value >= NDVI_PLANT_MIN:
        return "stressed"
    return "non-plant"


def health_color(label: str) -> tuple:
    """健康分级 → BGR 颜色（用于 YOLO 框上色）"""
    return {
        "healthy":   (0, 220, 0),    # 鲜绿
        "moderate":  (0, 220, 220),  # 黄
        "stressed":  (0, 80, 220),   # 红
        "non-plant": (180, 180, 180),  # 灰
    }.get(label, (180, 180, 180))


def mask_gray_roi(vis: np.ndarray, roi: dict,
                  color=(20, 20, 20)) -> np.ndarray:
    """
    输出端遮挡灰卡 ROI（演示视频里看不见参考物）。
    系统内部仍读这块区域做 K 校验，演示画面这块涂黑。

    roi 是 {"x":, "y":, "w":, "h":} 字典；任一字段缺失则不遮挡。
    """
    if not roi:
        return vis
    try:
        x, y, w, h = int(roi["x"]), int(roi["y"]), int(roi["w"]), int(roi["h"])
    except (KeyError, TypeError, ValueError):
        return vis
    if w <= 0 or h <= 0:
        return vis
    cv2.rectangle(vis, (x, y), (x + w, y + h), color, -1)
    return vis


# ══════════════════════════════════════════════════════════════
#  MJPEG 流服务器（带模式切换控件）
# ══════════════════════════════════════════════════════════════
class MJPEGServer:
    def __init__(self, port: int, name: str, node_ref=None):
        self.port = port
        self.name = name
        self.node = node_ref
        self._lock = threading.Lock()
        self._jpeg = b""

    def push_frame(self, frame_bgr: np.ndarray):
        ret, jpeg = cv2.imencode(
            ".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY]
        )
        if ret:
            with self._lock:
                self._jpeg = jpeg.tobytes()

    def get_jpeg(self) -> bytes:
        with self._lock:
            return self._jpeg

    def start(self):
        ref = self

        class _Handler(BaseHTTPRequestHandler):
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

            def do_GET(self):
                # ── MJPEG 流 ──
                if self.path == "/stream":
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=--boundary",
                    )
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    try:
                        while True:
                            jpeg = ref.get_jpeg()
                            if jpeg:
                                self.wfile.write(b"--boundary\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(
                                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                )
                                self.wfile.write(jpeg)
                                self.wfile.write(b"\r\n")
                            time.sleep(1.0 / CAM_FPS)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return

                # ── 切换渲染模式 ──
                if self.path.startswith("/api/mode"):
                    qs = {}
                    if "?" in self.path:
                        for kv in self.path.split("?", 1)[1].split("&"):
                            if "=" in kv:
                                k, v = kv.split("=", 1)
                                qs[k] = v
                    m = qs.get("m", "")
                    if m in ("colormap", "mask_solid", "mask_blend") and ref.node:
                        ref.node.render_mode = m
                        ref.node.get_logger().info(f"NDVI 渲染模式 → {m}")
                        self._send_json({"ok": True, "mode": m})
                    else:
                        self._send_json({"ok": False}, 400)
                    return

                # ── 调阈值 ──
                if self.path.startswith("/api/threshold"):
                    qs = {}
                    if "?" in self.path:
                        for kv in self.path.split("?", 1)[1].split("&"):
                            if "=" in kv:
                                k, v = kv.split("=", 1)
                                qs[k] = v
                    try:
                        t = float(qs.get("t", "0.25"))
                        t = max(-1.0, min(1.0, t))
                    except ValueError:
                        t = 0.25
                    if ref.node:
                        ref.node.plant_threshold = t
                        ref.node.get_logger().info(f"NDVI 阈值 → {t:.2f}")
                    self._send_json({"ok": True, "threshold": t})
                    return

                # ── 切换 active 模式开关 ──
                if self.path.startswith("/api/active"):
                    qs = {}
                    if "?" in self.path:
                        for kv in self.path.split("?", 1)[1].split("&"):
                            if "=" in kv:
                                k, v = kv.split("=", 1)
                                qs[k] = v
                    on = qs.get("on", "1") == "1"
                    if ref.node:
                        ref.node.prefer_active = on
                        ref.node.get_logger().info(f"prefer_active → {on}")
                    self._send_json({"ok": True, "prefer_active": on})
                    return

                # ── 切换 ROI 遮挡 ──
                if self.path.startswith("/api/mask_roi"):
                    qs = {}
                    if "?" in self.path:
                        for kv in self.path.split("?", 1)[1].split("&"):
                            if "=" in kv:
                                k, v = kv.split("=", 1)
                                qs[k] = v
                    on = qs.get("on", "1") == "1"
                    if ref.node:
                        ref.node.mask_roi_on_output = on
                    self._send_json({"ok": True, "mask_roi": on})
                    return

                # ── 状态查询 ──
                if self.path == "/api/state":
                    if ref.node:
                        n = ref.node
                        # 当前实际生效模式
                        if n.calib.calib4_done and n.prefer_active:
                            current = "active"
                        elif n.calib.refl_calibrated:
                            current = "refl"
                        else:
                            current = "pseudo"
                        self._send_json({
                            "render_mode": n.render_mode,
                            "threshold": n.plant_threshold,
                            "plant_ratio": n.last_plant_ratio,
                            "ndvi_mode": current,
                            "calib4_done": n.calib.calib4_done,
                            "calib4_light": n.calib.calib4_light,
                            "calib4_distance_cm": n.calib.calib4_distance_cm,
                            "k_active": n.calib.k_active,
                            "dark_R": n.calib.dark_R,
                            "dark_NIR": n.calib.dark_NIR,
                            "prefer_active": n.prefer_active,
                            "mask_roi": n.mask_roi_on_output,
                            "health": n.last_health_stats,
                        })
                    else:
                        self._send_json({})
                    return

                # ── 主页 ──
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                html = _NDVI_HTML.encode("utf-8")
                self.wfile.write(html)

        def _serve():
            HTTPServer(("0.0.0.0", self.port), _Handler).serve_forever()

        threading.Thread(target=_serve, daemon=True).start()
        print(f"[NDVIStream] {ref.name} → http://0.0.0.0:{ref.port}")


# ── NDVI 网页 HTML（v3.10 显示标定模式 + 健康分级面板） ────────────
_NDVI_HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>NDVI 植物健康检测 v3.10</title>
<style>
  body { background:#111; color:#eee; font-family:monospace; margin:0; padding:14px; }
  h2 { color:#0f0; margin:0 0 10px; font-size:18px; }
  .panel { background:#1a1a1a; padding:10px; border-radius:6px; margin-bottom:10px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn.active { background:#0f0; color:#000; }
  img { max-width:100%; border:1px solid #333; }
  .info { color:#888; font-size:12px; margin-top:6px; }
  .v { color:#0f0; }
  .stat { display:inline-block; background:#222; padding:4px 12px; border-radius:3px;
          margin-right:8px; font-size:13px; }
  .stat.green { background:#0a3a0a; color:#0f0; border:1px solid #0f0; }
  .stat.yellow { background:#3a3a0a; color:#fa0; border:1px solid #fa0; }
  .stat.red { background:#3a0a0a; color:#f55; border:1px solid #f55; }
  input[type=range] { vertical-align:middle; width:200px; }
  .row { display:flex; gap:14px; flex-wrap:wrap; align-items:flex-start; }
  .calib-box { font-size:12px; min-width:280px; }
  .calib-row { display:flex; justify-content:space-between;
               border-bottom:1px dotted #333; padding:2px 0; }
  .health-bar { display:flex; height:18px; border-radius:3px; overflow:hidden;
                margin-top:6px; background:#111; }
  .health-seg { display:flex; align-items:center; justify-content:center;
                color:#000; font-size:11px; font-weight:bold; min-width:0; }
</style>
</head>
<body>
<h2>🌿 NDVI 植物健康检测 v3.10 (active canopy sensor)</h2>

<div class="panel">
  <span style="color:#888;margin-right:8px">渲染模式:</span>
  <button class="btn" id="m-colormap" onclick="setMode('colormap')">全图渐变</button>
  <button class="btn" id="m-mask_solid" onclick="setMode('mask_solid')">仅植物高亮</button>
  <button class="btn" id="m-mask_blend" onclick="setMode('mask_blend')">半透明高亮</button>
  <span style="margin-left:20px;color:#888">阈值:</span>
  <input type="range" id="th" min="-0.5" max="0.8" step="0.01" value="0.25"
         oninput="setThreshold(this.value)">
  <span class="v" id="th-val">0.25</span>
  <span style="margin-left:14px">
    <input type="checkbox" id="cb-active" checked onchange="toggleActive(this.checked)">
    <label for="cb-active" style="font-size:12px;color:#aaa">优先 active 模式</label>
  </span>
  <span style="margin-left:8px">
    <input type="checkbox" id="cb-mask" checked onchange="toggleMask(this.checked)">
    <label for="cb-mask" style="font-size:12px;color:#aaa">遮挡灰卡 ROI（演示用）</label>
  </span>
</div>

<div class="row">
  <div>
    <img src="/stream" style="width:640px">
    <div class="info">
      <span class="stat" id="s-mode">渲染: --</span>
      <span class="stat" id="s-cal">--</span>
      <span class="stat" id="s-ratio">植物像素占比: --</span>
    </div>
  </div>
  <div class="panel calib-box">
    <div style="color:#888;font-size:13px;margin-bottom:6px">标定状态</div>
    <div class="calib-row"><span>当前 NDVI 模式</span><span class="v" id="c-mode">--</span></div>
    <div class="calib-row"><span>K (active)</span><span class="v" id="c-k">--</span></div>
    <div class="calib-row"><span>暗电流 R</span><span class="v" id="c-dark-r">--</span></div>
    <div class="calib-row"><span>暗电流 NIR</span><span class="v" id="c-dark-nir">--</span></div>
    <div class="calib-row"><span>光源类型</span><span class="v" id="c-light">--</span></div>
    <div class="calib-row"><span>工作距离</span><span class="v" id="c-dist">--</span></div>

    <div style="color:#888;font-size:13px;margin:10px 0 4px">健康分级（全图）</div>
    <div class="health-bar" id="hbar">
      <div class="health-seg" style="background:#0c0" id="seg-h">--</div>
      <div class="health-seg" style="background:#cc0" id="seg-m">--</div>
      <div class="health-seg" style="background:#c50" id="seg-s">--</div>
      <div class="health-seg" style="background:#888" id="seg-n">--</div>
    </div>
    <div style="font-size:11px;color:#888;margin-top:4px">
      <span style="color:#0c0">■</span>健康
      <span style="color:#cc0;margin-left:8px">■</span>亚健康
      <span style="color:#c50;margin-left:8px">■</span>枯萎
      <span style="color:#888;margin-left:8px">■</span>非植物
    </div>
  </div>
</div>

<div class="info" style="margin-top:8px">
  <span class="v">active 模式</span> = 灰卡 K 修正 + 暗电流减除（标定四完成才有）<br>
  <span class="v">refl 模式</span>   = v3.0 4 点色卡反射率定标（已暂停的老路径）<br>
  <span class="v">pseudo 模式</span> = 直接 DN 相除（兜底，未标定时）
</div>

<script>
function setMode(m) {
  fetch('/api/mode?m=' + m).then(() => refreshState());
}
function setThreshold(t) {
  document.getElementById('th-val').textContent = parseFloat(t).toFixed(2);
  fetch('/api/threshold?t=' + t).then(() => refreshState());
}
function toggleActive(on) {
  fetch('/api/active?on=' + (on ? 1 : 0));
}
function toggleMask(on) {
  fetch('/api/mask_roi?on=' + (on ? 1 : 0));
}
function refreshState() {
  fetch('/api/state').then(r => r.json()).then(d => {
    ['colormap','mask_solid','mask_blend'].forEach(m => {
      const el = document.getElementById('m-'+m);
      if (el) el.className = 'btn' + (d.render_mode === m ? ' active' : '');
    });
    document.getElementById('s-mode').textContent = '渲染: ' + (d.render_mode || '--');

    // 顶部模式徽章
    const sCal = document.getElementById('s-cal');
    if (d.ndvi_mode === 'active') {
      sCal.textContent = '✅ 真 NDVI (active)';
      sCal.className = 'stat green';
    } else if (d.ndvi_mode === 'refl') {
      sCal.textContent = '✅ 真 NDVI (refl)';
      sCal.className = 'stat green';
    } else {
      sCal.textContent = '⚠️ 伪 NDVI';
      sCal.className = 'stat yellow';
    }

    document.getElementById('s-ratio').textContent =
      '植物像素占比: ' + (d.plant_ratio * 100).toFixed(1) + '%';

    // 标定面板
    document.getElementById('c-mode').textContent = d.ndvi_mode;
    document.getElementById('c-k').textContent =
      d.k_active != null ? d.k_active.toFixed(4) : '--';
    document.getElementById('c-dark-r').textContent =
      d.dark_R != null ? d.dark_R.toFixed(2) : '--';
    document.getElementById('c-dark-nir').textContent =
      d.dark_NIR != null ? d.dark_NIR.toFixed(2) : '--';
    document.getElementById('c-light').textContent = d.calib4_light || '--';
    document.getElementById('c-dist').textContent =
      (d.calib4_distance_cm || '--') + ' cm';

    // 健康分级条
    if (d.health) {
      const h = d.health;
      const total = (h.healthy || 0) + (h.moderate || 0) +
                    (h.stressed || 0) + (h.non_plant || 0) + 1e-9;
      const pct = (v) => Math.max(0.5, (v / total) * 100);
      document.getElementById('seg-h').style.flexBasis = pct(h.healthy) + '%';
      document.getElementById('seg-m').style.flexBasis = pct(h.moderate) + '%';
      document.getElementById('seg-s').style.flexBasis = pct(h.stressed) + '%';
      document.getElementById('seg-n').style.flexBasis = pct(h.non_plant) + '%';
      document.getElementById('seg-h').textContent = (h.healthy/total*100).toFixed(0) + '%';
      document.getElementById('seg-m').textContent = (h.moderate/total*100).toFixed(0) + '%';
      document.getElementById('seg-s').textContent = (h.stressed/total*100).toFixed(0) + '%';
      document.getElementById('seg-n').textContent = (h.non_plant/total*100).toFixed(0) + '%';
    }

    if (d.threshold !== undefined) {
      document.getElementById('th').value = d.threshold;
      document.getElementById('th-val').textContent = d.threshold.toFixed(2);
    }
    document.getElementById('cb-active').checked = !!d.prefer_active;
    document.getElementById('cb-mask').checked = !!d.mask_roi;
  }).catch(() => {});
}
setInterval(refreshState, 1000);
refreshState();
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class NDVINode(Node):

    def __init__(self):
        super().__init__("ndvi_node")
        self.bridge = CvBridge()

        # 加载标定参数
        self.calib = load_calib()
        if not self.calib.calib1_done:
            self.get_logger().warn(
                "标定一未完成！IR 未对齐，NDVI 有偏差。"
                "请先: ros2 run laser_calibration calib_camera"
            )
        else:
            self.get_logger().info(
                f"标定参数已加载: Shift_X={self.calib.shift_x:+d}, "
                f"Shift_Y={self.calib.shift_y:+d}"
            )

        self._rgb = None
        self._ir = None
        self._lock = threading.Lock()

        # YOLO 检测框缓存
        self._yolo_boxes = []
        self._yolo_lock = threading.Lock()

        # 渲染模式 + 阈值（可通过 HTTP API 实时切换）
        self.render_mode = NDVI_DEFAULT_MODE
        self.plant_threshold = NDVI_PLANT_THRESHOLD
        self.last_plant_ratio = 0.0
        self.last_health_stats = {
            "healthy": 0, "moderate": 0, "stressed": 0, "non_plant": 0
        }

        # v3.10 active mode 开关 + ROI 遮挡开关
        self.prefer_active = ACTIVE_MODE_DEFAULT
        self.mask_roi_on_output = GRAY_ROI_MASK_ON_OUTPUT
        # 灰卡 ROI（沿用 config 默认值；后续可从 calib4 标定文件读取扩展）
        from laser_calibration.config import (
            GRAY_ROI_X, GRAY_ROI_Y, GRAY_ROI_W, GRAY_ROI_H
        )
        self.gray_roi = {
            "x": GRAY_ROI_X, "y": GRAY_ROI_Y,
            "w": GRAY_ROI_W, "h": GRAY_ROI_H,
        }

        self.sub_rgb = self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)
        self.sub_ir = self.create_subscription(Image, TOPIC_IR, self._cb_ir, 10)
        self.sub_yolo = self.create_subscription(String, TOPIC_YOLO, self._cb_yolo, 10)

        self.pub_img = self.create_publisher(Image, "/ndvi/image", 10)
        self.pub_result = self.create_publisher(String, "/ndvi/result", 10)

        self.stream = MJPEGServer(NDVI_STREAM_PORT, "NDVI Heatmap", node_ref=self)
        self.stream.start()

        self.timer = self.create_timer(0.1, self._process)

        self._log_startup()

    def _log_startup(self):
        log = self.get_logger().info
        warn = self.get_logger().warn
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log("  NDVI 植物健康检测节点  v3.10")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        if self.calib.calib4_done and self.prefer_active:
            log("  模式: ✅ 真 NDVI (active canopy sensor)")
            log(f"  K = {self.calib.k_active:.4f}")
            log(f"  dark_R = {self.calib.dark_R:.2f}, "
                f"dark_NIR = {self.calib.dark_NIR:.2f}")
            log(f"  光源 = {self.calib.calib4_light or '<未填>'}")
            log(f"  距离 = {self.calib.calib4_distance_cm} cm")
            log(f"  上次标定: {self.calib.calib4_timestamp}")
        elif self.calib.refl_calibrated:
            log("  模式: ✅ 真 NDVI (4 点色卡反射率定标)")
            log(f"  R 通道:   refl = {self.calib.k1:.6f} × DN + {self.calib.b1:.6f}  "
                f"R²={self.calib.refl_r2_red:.3f}")
            log(f"  NIR 通道: refl = {self.calib.k2:.6f} × DN + {self.calib.b2:.6f}  "
                f"R²={self.calib.refl_r2_nir:.3f}")
        else:
            warn("  模式: ⚠️ 伪 NDVI (未做任何反射率标定)")
            warn("  → 数值会受距离/材质影响，仅作开发验证用")
            warn("  → 推荐: ros2 run laser_calibration calib_diffuse")
            warn("       (主动光场 + 灰卡，新版标定四，3 步即可完成)")
        log(f"  prefer_active = {self.prefer_active} (网页可切换)")
        log(f"  灰卡 ROI 输出端遮挡 = {self.mask_roi_on_output}")
        log(f"  NDVI 热力图流: http://0.0.0.0:{NDVI_STREAM_PORT}")
        log("  /ndvi/image   → NDVI 伪彩色图 (ROS Image)")
        log("  /ndvi/result  → 逐株健康状态 (JSON String)")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    # ── 图像回调 ──────────────────────────────────────────────
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

    def _cb_yolo(self, msg: String):
        """接收 YOLO 检测结果，缓存检测框供 NDVI 叠加"""
        try:
            data = json.loads(msg.data)
            with self._yolo_lock:
                if "boxes" in data:
                    self._yolo_boxes = data["boxes"]
                elif data.get("detected"):
                    self._yolo_boxes = [{
                        "cx": data.get("cx", 0),
                        "cy": data.get("cy", 0),
                        "w":  data.get("w", 50),
                        "h":  data.get("h", 50),
                        "label": data.get("label", "weed"),
                        "confidence": data.get("confidence", 0),
                        "frame_id": data.get("frame_id", 0),
                    }]
                else:
                    self._yolo_boxes = []
        except Exception as e:
            self.get_logger().error(f"YOLO 消息解析失败：{e}")

    # ── 核心处理循环 ──────────────────────────────────────────
    def _process(self):
        with self._lock:
            rgb = self._rgb
            ir = self._ir

        if rgb is None or ir is None:
            return

        # Step 1：IR 对齐
        aligned_ir = self._align_ir(ir)
        ir_gray = cv2.cvtColor(aligned_ir, cv2.COLOR_BGR2GRAY)

        # Step 2：算 NDVI（v3.10 三层优先级）
        ndvi_map, mode_tag = compute_ndvi(
            rgb, ir_gray,
            calib=self.calib,
            prefer_active=self.prefer_active,
        )

        # Step 3：渲染
        vis = render_ndvi(
            ndvi_map, rgb,
            mode=self.render_mode,
            threshold=self.plant_threshold,
        ).copy()

        # Step 4：植物像素占比 + 三级健康统计
        plant_pixels = int(np.sum(ndvi_map > self.plant_threshold))
        total_pixels = ndvi_map.size
        self.last_plant_ratio = plant_pixels / total_pixels if total_pixels else 0.0

        # 三级分级（v3.10）
        h_count = int(np.sum(ndvi_map >= NDVI_HEALTHY_MIN))
        m_count = int(np.sum((ndvi_map < NDVI_HEALTHY_MIN) &
                             (ndvi_map >= NDVI_MODERATE_MIN)))
        s_count = int(np.sum((ndvi_map < NDVI_MODERATE_MIN) &
                             (ndvi_map >= NDVI_PLANT_MIN)))
        n_count = int(np.sum(ndvi_map < NDVI_PLANT_MIN))
        self.last_health_stats = {
            "healthy": h_count, "moderate": m_count,
            "stressed": s_count, "non_plant": n_count,
        }

        # Step 5：YOLO 框叠加 + 逐株 NDVI
        results = []
        with self._yolo_lock:
            boxes = list(self._yolo_boxes)

        if boxes:
            for b in boxes:
                cx = int(b.get("cx", 0))
                cy = int(b.get("cy", 0))
                w = int(b.get("w", 50))
                h = int(b.get("h", 50))
                x1 = max(0, cx - w // 2)
                y1 = max(0, cy - h // 2)
                x2 = min(vis.shape[1] - 1, cx + w // 2)
                y2 = min(vis.shape[0] - 1, cy + h // 2)

                roi_ndvi = ndvi_map[y1:y2, x1:x2]
                mean_val = float(np.mean(roi_ndvi)) if roi_ndvi.size > 0 else 0.0
                label = ndvi_health_label(mean_val)
                box_color = health_color(label)

                cv2.rectangle(vis, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(
                    vis, f"NDVI:{mean_val:+.2f} {label}",
                    (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA
                )

                results.append({
                    "cx": cx, "cy": cy,
                    "ndvi": round(mean_val, 4),
                    "health": label,
                    "label": b.get("label", "weed"),
                    "confidence": b.get("confidence", 0),
                })
        else:
            mean_global = float(np.mean(ndvi_map))
            results.append({
                "cx": -1, "cy": -1,
                "ndvi": round(mean_global, 4),
                "health": ndvi_health_label(mean_global),
                "label": "global",
            })

        # Step 6：OSD
        ts = time.strftime("%H:%M:%S")
        global_mean = float(np.mean(ndvi_map))
        plant_pct = self.last_plant_ratio * 100.0
        osd_text = (
            f"NDVI [{mode_tag}] {self.render_mode} | {ts} | "
            f"mean:{global_mean:+.3f} | plant:{plant_pct:.1f}%"
        )
        ov = vis.copy()
        cv2.rectangle(ov, (0, 0), (vis.shape[1], 26), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.55, vis, 0.45, 0, vis)
        text_color = (0, 220, 0) if mode_tag != "PSEUDO" else (0, 220, 220)
        cv2.putText(vis, osd_text, (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, text_color, 1, cv2.LINE_AA)

        # Step 7：灰卡 ROI 输出端遮挡（演示用，系统内部仍读它）
        if self.mask_roi_on_output and self.calib.calib4_done:
            mask_gray_roi(vis, self.gray_roi)

        # Step 8：推 MJPEG 流
        self.stream.push_frame(vis)

        # Step 9：发布 ROS topic
        try:
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "ndvi"
            self.pub_img.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"NDVI image publish 失败：{e}")

        result_msg = String()
        result_msg.data = json.dumps({
            "timestamp": ts,
            "ndvi_mode": mode_tag,  # ACTIVE / REFL / PSEUDO
            "global_ndvi": round(global_mean, 4),
            "plant_ratio": round(self.last_plant_ratio, 4),
            "health_stats": self.last_health_stats,
            "plants": results,
        }, ensure_ascii=False)
        self.pub_result.publish(result_msg)

    # ── IR 对齐 ───────────────────────────────────────────────
    def _align_ir(self, ir: np.ndarray) -> np.ndarray:
        if self.calib.shift_x == 0 and self.calib.shift_y == 0:
            return ir
        h, w = ir.shape[:2]
        M = np.float32([
            [1, 0, self.calib.shift_x],
            [0, 1, self.calib.shift_y],
        ])
        return cv2.warpAffine(
            ir, M, (w, h),
            borderMode=cv2.BORDER_REPLICATE,
        )


def main(args=None):
    rclpy.init(args=args)
    node = NDVINode()
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
