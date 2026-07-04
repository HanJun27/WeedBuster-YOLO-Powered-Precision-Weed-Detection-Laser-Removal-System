#!/usr/bin/env python3
"""
calib_camera_align.py —— 标定一：摄像头基线对齐（浏览器版+缩放）  v3.4
========================================================================
v3.4 升级：
  - 冻结后图片可滚轮缩放、拖动平移，定靶心更精确
  - 鼠标悬停时显示局部放大镜（4倍），像素级对齐
  - [+/-] 按钮和 [复位] 按钮辅助操作
  - 不论怎么缩放，点击坐标永远换算回原图(640x480)坐标

前置：
  ros2 run laser_calibration stereo_camera

运行：
  ros2 run laser_calibration calib_camera

浏览器打开：
  http://localhost:8090         (小车本机)
  http://<小车IP>:8090           (远程)
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from laser_calibration.calib_io import save_calib
from laser_calibration.config import TOPIC_IR, TOPIC_RGB

CALIB_HTTP_PORT = 8090


HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>标定一 · 摄像头基线对齐</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1 { color:#0f0; margin:0 0 12px; font-size:18px; }
  .panel { background:#222; padding:12px; border-radius:8px; margin-bottom:12px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; overflow:hidden;
             width:640px; height:480px; flex:0 0 auto; }
  .cam-box.active { border-color:#0f0; }
  .cam-box.done { border-color:#fa0; }
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
  .zoom-tools { display:inline-block; margin-left:16px; }
</style>
</head>
<body>
<h1>📷 标定一 · 摄像头基线对齐</h1>

<div class="panel">
  <button class="btn" id="btn-freeze" onclick="action('freeze')">[S] 冻结画面</button>
  <button class="btn" id="btn-reset"  onclick="action('reset')" disabled>[R] 重做</button>
  <button class="btn" id="btn-save"   onclick="action('save')"  disabled>[C] 保存</button>
  <span class="zoom-tools">
    <button class="btn" onclick="zoomAll(1.5)">[+] 放大</button>
    <button class="btn" onclick="zoomAll(1/1.5)">[-] 缩小</button>
    <button class="btn" onclick="resetView()">[复位]</button>
  </span>
  <div class="status" id="status">→ 把十字靶放在镜头前 30cm，按 [冻结画面] 开始</div>
</div>

<div class="row">
  <div class="cam-box live" id="box-rgb">
    <span class="cam-label">RGB</span>
    <span class="zoom-info" id="zoom-info-rgb">1.00x</span>
    <canvas id="cv-rgb" width="640" height="480"></canvas>
  </div>
  <div class="cam-box live" id="box-ir">
    <span class="cam-label">IR</span>
    <span class="zoom-info" id="zoom-info-ir">1.00x</span>
    <canvas id="cv-ir" width="640" height="480"></canvas>
  </div>
</div>

<div class="info">
  操作: <span class="v">1)</span> 冻结 →
  <span class="v">2)</span> 滚轮缩放/拖动平移找靶心 →
  <span class="v">3)</span> 点 RGB 靶心 →
  <span class="v">4)</span> 点 IR 同一点 →
  <span class="v">5)</span> 保存
</div>
<div class="info">
  快捷键: <span class="v">S</span>冻结 · <span class="v">R</span>重做 ·
  <span class="v">C</span>保存 · 滚轮缩放 · 鼠标按住拖动 · 悬停显示放大镜
</div>
<div class="info" id="result"></div>

<script>
const W = 640, H = 480;

class CamView {
  constructor(name) {
    this.name = name;
    this.canvas = document.getElementById('cv-' + name);
    this.ctx = this.canvas.getContext('2d');
    this.box = document.getElementById('box-' + name);
    this.zoomInfo = document.getElementById('zoom-info-' + name);
    this.frozenImg = null;
    this.scale = 1;
    this.tx = 0; this.ty = 0;
    this.point = null;
    this.dragStart = null;
    this.loupe = null;
    this._setupEvents();
    this._startLiveLoop();
  }

  _setupEvents() {
    this.canvas.addEventListener('wheel', (e) => {
      if (!this.frozenImg) return;
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.2 : 1/1.2;
      const rect = this.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      this._zoomAt(factor, mx, my);
    }, { passive: false });

    this.canvas.addEventListener('mousedown', (e) => {
      if (!this.frozenImg) return;
      this.dragStart = { mx:e.clientX, my:e.clientY, tx:this.tx, ty:this.ty, moved:false };
    });

    this.canvas.addEventListener('mousemove', (e) => {
      if (this.dragStart) {
        const dx = e.clientX - this.dragStart.mx;
        const dy = e.clientY - this.dragStart.my;
        if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
          this.dragStart.moved = true;
          this.tx = this.dragStart.tx + dx;
          this.ty = this.dragStart.ty + dy;
          this.box.classList.add('panning');
          this.redraw();
        }
      }
      if (this.frozenImg) {
        this._showLoupe(e);
      }
    });

    this.canvas.addEventListener('mouseup', (e) => {
      if (!this.dragStart) return;
      const wasMove = this.dragStart.moved;
      this.dragStart = null;
      this.box.classList.remove('panning');
      if (!wasMove && this.frozenImg) {
        this._handleClick(e);
      }
    });

    this.canvas.addEventListener('mouseleave', () => {
      this.dragStart = null;
      this.box.classList.remove('panning');
      this._hideLoupe();
    });
  }

  _screenToImage(sx, sy) {
    return {
      x: Math.round((sx - this.tx) / this.scale),
      y: Math.round((sy - this.ty) / this.scale),
    };
  }

  _zoomAt(factor, sx, sy) {
    const newScale = Math.max(0.5, Math.min(8, this.scale * factor));
    const realFactor = newScale / this.scale;
    this.tx = sx - (sx - this.tx) * realFactor;
    this.ty = sy - (sy - this.ty) * realFactor;
    this.scale = newScale;
    this.zoomInfo.textContent = this.scale.toFixed(2) + 'x';
    this.redraw();
  }

  zoom(factor) { this._zoomAt(factor, W/2, H/2); }

  resetView() {
    this.scale = 1; this.tx = 0; this.ty = 0;
    this.zoomInfo.textContent = '1.00x';
    this.redraw();
  }

  _handleClick(e) {
    const rect = this.canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const pt = this._screenToImage(sx, sy);
    if (pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H) return;

    if (state === 'FROZEN' && this.name === 'rgb') {
      this.point = pt;
      state = 'RGB_DONE';
      this.redraw(); refreshUI();
      fetch(`/api/click?cam=rgb&x=${pt.x}&y=${pt.y}`);
    } else if (state === 'RGB_DONE' && this.name === 'ir') {
      this.point = pt;
      state = 'IR_DONE';
      this.redraw(); refreshUI();
      fetch(`/api/click?cam=ir&x=${pt.x}&y=${pt.y}`);
    }
  }

  _showLoupe(e) {
    if (!this.loupe) {
      this.loupe = document.createElement('canvas');
      this.loupe.width = 160; this.loupe.height = 160;
      this.loupe.className = 'loupe';
      this.box.appendChild(this.loupe);
    }
    const rect = this.canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const pt = this._screenToImage(sx, sy);
    if (pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H) {
      this._hideLoupe();
      return;
    }
    const lctx = this.loupe.getContext('2d');
    lctx.imageSmoothingEnabled = false;
    lctx.fillStyle = '#000';
    lctx.fillRect(0, 0, 160, 160);
    const half = 20;
    lctx.drawImage(
      this.frozenImg,
      pt.x - half, pt.y - half, 2*half, 2*half,
      0, 0, 160, 160
    );
    lctx.strokeStyle = '#0f0'; lctx.lineWidth = 1;
    lctx.beginPath();
    lctx.moveTo(80, 60); lctx.lineTo(80, 100);
    lctx.moveTo(60, 80); lctx.lineTo(100, 80);
    lctx.stroke();
    lctx.strokeStyle = 'rgba(0,255,0,0.4)';
    lctx.strokeRect(75, 75, 10, 10);
    lctx.fillStyle = '#0f0'; lctx.font = '11px monospace';
    lctx.fillText(`(${pt.x},${pt.y})`, 6, 152);

    this.loupe.style.right = '8px';
    this.loupe.style.top = '8px';
  }

  _hideLoupe() {
    if (this.loupe) { this.loupe.remove(); this.loupe = null; }
  }

  _startLiveLoop() {
    const img = new Image();
    const loop = () => {
      if (this.frozenImg) { setTimeout(loop, 200); return; }
      img.onload = () => {
        if (!this.frozenImg) {
          this.ctx.drawImage(img, 0, 0, W, H);
        }
        setTimeout(loop, 50);
      };
      img.onerror = () => setTimeout(loop, 300);
      img.src = `/frame/${this.name}?t=${Date.now()}`;
    };
    loop();
  }

  loadFrozen() {
    const img = new Image();
    img.onload = () => {
      this.frozenImg = img;
      this.box.classList.remove('live');
      this.box.classList.add('frozen');
      this.redraw();
    };
    img.src = `/frozen/${this.name}?t=${Date.now()}`;
  }

  redraw() {
    if (!this.frozenImg) return;
    this.ctx.fillStyle = '#000';
    this.ctx.fillRect(0, 0, W, H);
    this.ctx.imageSmoothingEnabled = false;
    const w = W * this.scale, h = H * this.scale;
    this.ctx.drawImage(this.frozenImg, this.tx, this.ty, w, h);

    if (this.point) {
      const sx = this.point.x * this.scale + this.tx;
      const sy = this.point.y * this.scale + this.ty;
      this._drawCross(sx, sy);
    }
  }

  _drawCross(sx, sy) {
    const c = this.ctx;
    c.strokeStyle = '#0f0'; c.lineWidth = 2;
    c.beginPath();
    c.moveTo(sx - 24, sy); c.lineTo(sx + 24, sy);
    c.moveTo(sx, sy - 24); c.lineTo(sx, sy + 24);
    c.stroke();
    c.beginPath(); c.arc(sx, sy, 16, 0, Math.PI*2); c.stroke();
    c.fillStyle = '#0f0'; c.font = '13px monospace';
    c.fillText(`(${this.point.x},${this.point.y})`, sx + 20, sy - 14);
  }

  clearPoint() {
    this.point = null;
    this.redraw();
  }
}

let state = 'LIVE';
const camR = new CamView('rgb');
const camI = new CamView('ir');

function zoomAll(factor) {
  camR.zoom(factor);
  camI.zoom(factor);
}
function resetView() {
  camR.resetView();
  camI.resetView();
}

function action(act) {
  fetch('/api/' + act).then(r => r.json()).then(d => {
    if (act === 'freeze') {
      state = 'FROZEN';
      camR.loadFrozen();
      camI.loadFrozen();
    } else if (act === 'reset') {
      state = 'FROZEN';
      camR.clearPoint(); camI.clearPoint();
    } else if (act === 'save') {
      state = 'SAVED';
      document.getElementById('result').innerHTML =
        `✅ 已保存: Shift_X=<span class="v">${d.shift_x>=0?'+':''}${d.shift_x}</span>` +
        ` Shift_Y=<span class="v">${d.shift_y>=0?'+':''}${d.shift_y}</span>` +
        ` (Shift_Y 应≈0，否则两镜头不平行)`;
    }
    refreshUI();
  });
}

function refreshUI() {
  document.getElementById('btn-freeze').disabled = (state !== 'LIVE');
  document.getElementById('btn-reset').disabled  = !(state === 'RGB_DONE' || state === 'IR_DONE');
  document.getElementById('btn-save').disabled   = (state !== 'IR_DONE');

  document.getElementById('box-rgb').className = 'cam-box ' +
    (state === 'LIVE' ? 'live' : 'frozen') +
    (state === 'FROZEN' ? ' active' : (camR.point ? ' done' : ''));
  document.getElementById('box-ir').className = 'cam-box ' +
    (state === 'LIVE' ? 'live' : 'frozen') +
    (state === 'RGB_DONE' ? ' active' : (camI.point ? ' done' : ''));

  const tip = {
    'LIVE':     '→ 把十字靶放在镜头前 30cm，按 [冻结画面] 开始',
    'FROZEN':   '→ 滚轮放大 RGB 画面，对准靶心点击',
    'RGB_DONE': '→ 滚轮放大 IR 画面，点击同一个靶心',
    'IR_DONE':  '→ 按 [保存] 写入参数文件，或 [重做] 重新点击',
    'SAVED':    '✅ 标定完成！可以关闭此页面',
  };
  document.getElementById('status').textContent = tip[state] || '';
}

document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === 's' && state === 'LIVE')      action('freeze');
  else if (e.key === 'r' && (state === 'RGB_DONE' || state === 'IR_DONE')) action('reset');
  else if (e.key === 'c' && state === 'IR_DONE') action('save');
});

refreshUI();
</script>
</body>
</html>
"""


class CalibCameraAlignNode(Node):

    def __init__(self):
        super().__init__("calib_camera_align")
        self.bridge = CvBridge()

        self._rgb = None
        self._ir  = None
        self._lock = threading.Lock()

        self.frozen_rgb = None
        self.frozen_ir  = None

        self.rgb_pt = None
        self.ir_pt  = None

        self.state = "LIVE"

        self.sub_rgb = self.create_subscription(Image, TOPIC_RGB, self._cb_rgb, 10)
        self.sub_ir  = self.create_subscription(Image, TOPIC_IR,  self._cb_ir,  10)

        self._start_http()

        log = self.get_logger().info
        log("═══════════════════════════════════════════════════════")
        log("  标定一：摄像头基线对齐  v3.4 (浏览器版+缩放+放大镜)")
        log("═══════════════════════════════════════════════════════")
        log(f"  本机访问:  http://localhost:{CALIB_HTTP_PORT}")
        log(f"  远程访问:  http://<小车IP>:{CALIB_HTTP_PORT}")
        log("  支持: 滚轮缩放、拖动平移、悬停放大镜（4倍）")
        log("  按 Ctrl+C 退出节点")
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

    def _get_frame(self, name: str):
        with self._lock:
            return self._rgb if name == "rgb" else self._ir

    def _get_frozen(self, name: str):
        return self.frozen_rgb if name == "rgb" else self.frozen_ir

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

                if path == "/" or path == "/index.html":
                    body = HTML_PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path.startswith("/frame/"):
                    cam = path.split("/")[-1]
                    self._send_jpeg(node._get_frame(cam))
                    return

                if path.startswith("/frozen/"):
                    cam = path.split("/")[-1]
                    self._send_jpeg(node._get_frozen(cam))
                    return

                if path == "/api/freeze":
                    with node._lock:
                        if node._rgb is not None and node._ir is not None:
                            node.frozen_rgb = node._rgb.copy()
                            node.frozen_ir  = node._ir.copy()
                            node.state = "FROZEN"
                            node.get_logger().info("画面已冻结 → 浏览器中放大点击 RGB 靶心")
                    self._send_json({"ok": True, "state": node.state})
                    return

                if path == "/api/click":
                    cam = qs.get("cam", "")
                    try:
                        x = int(qs.get("x", "0"))
                        y = int(qs.get("y", "0"))
                    except ValueError:
                        x = y = 0
                    if cam == "rgb" and node.state == "FROZEN":
                        node.rgb_pt = (x, y)
                        node.state = "RGB_DONE"
                        node.get_logger().info(
                            f"RGB 靶心已记录：({x}, {y})  → 请点击 IR 同一个靶心"
                        )
                    elif cam == "ir" and node.state == "RGB_DONE":
                        node.ir_pt = (x, y)
                        node.state = "IR_DONE"
                        sx = node.rgb_pt[0] - x
                        sy = node.rgb_pt[1] - y
                        log = node.get_logger().info
                        log(f"IR 靶心已记录：({x}, {y})")
                        log(f"  → Shift_X = {node.rgb_pt[0]} - {x} = {sx:+d}")
                        log(f"  → Shift_Y = {node.rgb_pt[1]} - {y} = {sy:+d}  (应≈0)")
                    self._send_json({
                        "ok": True, "state": node.state,
                        "rgb_pt": node.rgb_pt, "ir_pt": node.ir_pt,
                    })
                    return

                if path == "/api/reset":
                    node.rgb_pt = None
                    node.ir_pt  = None
                    node.state  = "FROZEN"
                    node.get_logger().info("已重置，请重新点击 RGB 靶心")
                    self._send_json({"ok": True, "state": node.state})
                    return

                if path == "/api/save":
                    if node.state != "IR_DONE":
                        self._send_json({"ok": False, "msg": "请先点击两个靶心"}, code=400)
                        return
                    sx = node.rgb_pt[0] - node.ir_pt[0]
                    sy = node.rgb_pt[1] - node.ir_pt[1]
                    save_calib({
                        "shift_x": int(sx),
                        "shift_y": int(sy),
                        "calib1_done": True,
                    })
                    node.state = "SAVED"
                    node.get_logger().info(
                        f"✅ 标定一完成！Shift_X={sx:+d}, Shift_Y={sy:+d}"
                    )
                    self._send_json({"ok": True, "shift_x": sx, "shift_y": sy})
                    return

                self.send_response(404); self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", CALIB_HTTP_PORT), Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = CalibCameraAlignNode()
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
