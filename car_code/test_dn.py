#!/usr/bin/env python3
"""快速诊断：实时显示鼠标位置的 R 通道 DN 值"""
import cv2, numpy as np, threading, sys
from http.server import BaseHTTPRequestHandler, HTTPServer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.insert(0, '/home/sunrise/yahboomcar_ws/install/laser_calibration/lib/python3.10/site-packages')
from laser_calibration.config import TOPIC_RGB

HTML_PAGE = """<html><body style="background:#111;color:#0f0;font-family:monospace;margin:0">
<canvas id=c width=640 height=480 style="cursor:crosshair;display:block"></canvas>
<div id=info style="padding:10px;font-size:14px">移动鼠标看任意点的 R/G/B 值（30x30 区域均值，Rstd=该区域 R 通道标准差）</div>
<script>
const c=document.getElementById('c'),x=c.getContext('2d'),img=new Image();
function loop(){img.onload=()=>{x.drawImage(img,0,0,640,480);setTimeout(loop,80)};
img.onerror=()=>setTimeout(loop,300);img.src='/frame?t='+Date.now()}loop();
c.addEventListener('mousemove',e=>{
  const r=c.getBoundingClientRect();
  fetch(`/api/sample?x=${Math.round(e.clientX-r.left)}&y=${Math.round(e.clientY-r.top)}`)
    .then(r=>r.json()).then(d=>{
      document.getElementById('info').innerHTML=
      `(${d.x},${d.y}) <span style="color:#f55">R=${d.R}</span>`+
      ` <span style="color:#5f5">G=${d.G}</span> <span style="color:#55f">B=${d.B}</span>`+
      ` Rstd=${d.Rstd} ${d.Rstd>5?'<span style="color:#fa0">⚠️ 噪声大</span>':'✓'}`;
    });
});
</script></body></html>"""

class DNProbe(Node):
    def __init__(self):
        super().__init__('dn_probe')
        self.frame = None
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.create_subscription(Image, TOPIC_RGB, self.cb, 10)
        threading.Thread(target=self.serve, daemon=True).start()
        print("浏览器打开 http://<小车IP>:8099 看 RGB 画面，悬停看 DN 值")
    def cb(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.lock:
                self.frame = f
        except Exception as e:
            print(e)
    def serve(self):
        node = self
        class H(BaseHTTPRequestHandler):
            def log_message(s, *a): pass
            def do_GET(s):
                if s.path.startswith('/frame'):
                    with node.lock: f = node.frame
                    if f is None:
                        f = np.zeros((480,640,3), np.uint8)
                    ok, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    s.send_response(200)
                    s.send_header('Content-Type','image/jpeg')
                    s.end_headers()
                    s.wfile.write(buf.tobytes())
                    return
                if s.path.startswith('/api/sample'):
                    qs = dict(kv.split('=') for kv in s.path.split('?')[1].split('&'))
                    x, y = int(qs['x']), int(qs['y'])
                    with node.lock: f = node.frame
                    if f is None:
                        s.send_response(500); s.end_headers(); return
                    h, w = f.shape[:2]
                    x = max(20, min(w-20, x)); y = max(20, min(h-20, y))
                    roi = f[y-15:y+15, x-15:x+15]
                    R = float(roi[:,:,2].mean())
                    G = float(roi[:,:,1].mean())
                    B = float(roi[:,:,0].mean())
                    Rstd = float(roi[:,:,2].std())
                    body = ('{"x":%d,"y":%d,"R":%.1f,"G":%.1f,"B":%.1f,"Rstd":%.2f}' % (x,y,R,G,B,Rstd)).encode()
                    s.send_response(200); s.send_header('Content-Type','application/json')
                    s.send_header('Content-Length',str(len(body))); s.end_headers()
                    s.wfile.write(body); return
                # main page
                body = HTML_PAGE.encode('utf-8')
                s.send_response(200); s.send_header('Content-Type','text/html; charset=utf-8')
                s.send_header('Content-Length',str(len(body))); s.end_headers()
                s.wfile.write(body)
        HTTPServer(('0.0.0.0', 8099), H).serve_forever()

rclpy.init()
n = DNProbe()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
finally:
    n.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
