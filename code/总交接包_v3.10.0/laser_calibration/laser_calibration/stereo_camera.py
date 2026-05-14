#!/usr/bin/env python3
"""
stereo_camera.py —— 双目摄像头驱动节点  v3.0
=============================================

v3.0 核心升级：
  1. ★ udev by-id 稳定路径打开摄像头（USB 插拔顺序变化不再影响识别）
  2. ★ 自动锁定 ISP：关自动曝光 + 关自动白平衡 + 固定曝光/色温
     —— 这是 NDVI 能不能跑的生死线，不锁 DN 值就漂移，NDVI 彻底失效
  3. ★ OSD 新增同步指示：显示本路帧号+时间戳 + 与另一路的 Δt
     调试帧同步时一眼看出偏差
  4. 保留 v2 的 IR MJPEG 强制模式（解决 USB 带宽读帧失败）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
为 YOLO 同学预留的接口（BPU 部署版）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
YOLO 节点只需：

  【订阅】
    /camera/rgb/image_raw   sensor_msgs/Image   做目标检测
    （YOLOv8 模型部署到 RDK X5 BPU，通过 hobot_dnn 加载 .bin）

  【发布】
    /yolo/weed_detected     std_msgs/String     JSON 检测结果

  JSON 格式（单框版）：
  {
      "detected":   true,
      "cx":         320,       # 杂草中心 X（RGB 图坐标）
      "cy":         240,       # 杂草中心 Y（RGB 图坐标）
      "w":          40,        # 框宽（可选，NDVI 和模板匹配会用）
      "h":          60,        # 框高（可选）
      "confidence": 0.92,
      "label":      "weed",
      "frame_id":   1234
  }

  Phase 3 视觉伺服节点会订阅 /yolo/weed_detected，
  收到后底盘刹车，云台开始 IBVS 闭环打击。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

网络流访问（同一局域网任意设备浏览器）：
  http://<小车IP>:8080  →  RGB 实时画面
  http://<小车IP>:8081  →  IR  实时画面

运行：
  ros2 run laser_calibration stereo_camera
"""

import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from laser_calibration.config import (
    CAM_FPS, CAM_HEIGHT, CAM_WIDTH,
    IR_DEVICE, IR_EXPOSURE,
    LOCK_ISP,
    RGB_DEVICE, RGB_EXPOSURE, RGB_WB_TEMP,
    STREAM_PORT_IR, STREAM_PORT_RGB, STREAM_QUALITY,
    TOPIC_IR, TOPIC_RGB,
)


# ══════════════════════════════════════════════════════════════
#  V4L2 ISP 锁定工具（NDVI 必需）
# ══════════════════════════════════════════════════════════════
def _v4l2_set(device: str, aliases: list, value, logger, cam_name: str) -> bool:
    """
    通过 v4l2-ctl 设置单个 V4L2 控件，兼容新旧 kernel 的命名差异。
    kernel 5.13+ 控件名发生变更，按 aliases 顺序依次尝试，首个成功即返回。

    aliases 示例：
        ['auto_exposure', 'exposure_auto']                 # 自动曝光开关
        ['exposure_time_absolute', 'exposure_absolute']    # 绝对曝光值
        ['white_balance_automatic', 'white_balance_temperature_auto']
    """
    last_err = ""
    for name in aliases:
        try:
            r = subprocess.run(
                ['v4l2-ctl', '-d', device, '-c', f'{name}={value}'],
                capture_output=True, text=True, timeout=2,
            )
            if r.returncode == 0:
                logger.info(f"[{cam_name}] v4l2-ctl: {name}={value} ✓")
                return True
            last_err = (r.stderr or r.stdout or "").strip()
        except FileNotFoundError:
            logger.warn(
                f"[{cam_name}] v4l2-ctl 未安装，ISP 无法通过命令行锁定 "
                "(安装: sudo apt install -y v4l-utils)"
            )
            return False
        except subprocess.TimeoutExpired:
            last_err = "timeout"
            continue
    logger.warn(
        f"[{cam_name}] 所有控件别名都失败: {aliases}={value} "
        f"(stderr: {last_err or 'unknown'})"
    )
    return False


def lock_isp(device: str, cam_name: str, logger,
             exposure: int | None = None,
             wb_temp: int | None = None,
             skip_wb: bool = False):
    """
    锁死 ISP 避免 NDVI 漂移。
    ★ 操作顺序至关重要：必须先把 auto_exposure 设为 manual，
       之后设置 exposure_time_absolute 才会生效。

    Args:
        device   : V4L2 设备路径（如 /dev/v4l/by-id/...）
        cam_name : 日志前缀 ("RGB" / "IR")
        exposure : 绝对曝光值（单位 100μs，None 表示不锁）
        wb_temp  : 白平衡色温（K），IR 相机无需
        skip_wb  : IR 灰度相机设为 True，跳过白平衡控件
    """
    if not LOCK_ISP:
        logger.info(f"[{cam_name}] LOCK_ISP=False，跳过 ISP 锁定（NDVI 会失效！）")
        return

    # ① 自动曝光→手动（V4L2_EXPOSURE_MANUAL = 1）
    _v4l2_set(device, ['auto_exposure', 'exposure_auto'], 1, logger, cam_name)

    # ② 固定曝光时间
    if exposure is not None:
        _v4l2_set(
            device,
            ['exposure_time_absolute', 'exposure_absolute'],
            exposure, logger, cam_name,
        )

    # ③ 自动白平衡→关 + 固定色温
    if not skip_wb:
        _v4l2_set(
            device,
            ['white_balance_automatic', 'white_balance_temperature_auto'],
            0, logger, cam_name,
        )
        if wb_temp is not None:
            _v4l2_set(
                device, ['white_balance_temperature'],
                wb_temp, logger, cam_name,
            )

    logger.info(f"[{cam_name}] ISP 锁定完成 → DN 值稳定，NDVI 可用")


# ══════════════════════════════════════════════════════════════
#  双路同步追踪（OSD Δt 显示用）
# ══════════════════════════════════════════════════════════════
class FrameSync:
    """
    两路相机的最新帧号/时间戳共享状态。
    让每路 OSD 能显示 "与另一路的 Δt"，便于调试帧同步。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.rgb_id = 0
        self.rgb_ts = 0.0
        self.ir_id  = 0
        self.ir_ts  = 0.0

    def update(self, cam_name: str, frame_id: int, ts: float):
        with self._lock:
            if cam_name == "RGB":
                self.rgb_id, self.rgb_ts = frame_id, ts
            else:
                self.ir_id, self.ir_ts = frame_id, ts

    def peer_delta_ms(self, cam_name: str, ts: float) -> int:
        """返回本路时间戳相对另一路最新帧的 Δt（毫秒，带符号）"""
        with self._lock:
            peer_ts = self.ir_ts if cam_name == "RGB" else self.rgb_ts
        if peer_ts == 0.0:
            return 0
        return int((ts - peer_ts) * 1000)


# ══════════════════════════════════════════════════════════════
#  MJPEG 视频流服务器
# ══════════════════════════════════════════════════════════════
class MJPEGServer:
    """极简 MJPEG over HTTP，浏览器打开 http://IP:PORT 即可查看。"""

    def __init__(self, port: int, cam_name: str):
        self.port     = port
        self.cam_name = cam_name
        self._lock    = threading.Lock()
        self._jpeg    = b""

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
        server_ref = self

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def do_GET(self):
                if self.path in ("/", "/stream"):
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
                            jpeg = server_ref.get_jpeg()
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
                else:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    html = (
                        f"<html><head>"
                        f"<title>{server_ref.cam_name}</title>"
                        f"<style>body{{background:#111;color:#eee;"
                        f"font-family:monospace;text-align:center}}"
                        f"img{{max-width:100%;border:2px solid #444}}</style>"
                        f"</head><body>"
                        f"<h2>{server_ref.cam_name} 实时画面</h2>"
                        f"<img src='/stream'>"
                        f"</body></html>"
                    ).encode("utf-8")
                    self.wfile.write(html)

        def _serve():
            srv = HTTPServer(("0.0.0.0", self.port), _Handler)
            srv.serve_forever()

        threading.Thread(target=_serve, daemon=True).start()
        print(f"[MJPEGServer] {self.cam_name} 流已启动 → http://0.0.0.0:{self.port}")


# ══════════════════════════════════════════════════════════════
#  OSD 绘制
# ══════════════════════════════════════════════════════════════
def _draw_osd(frame: np.ndarray, cam_name: str,
              frame_id: int, ts: float, dt_ms: int):
    """
    半透明顶条 OSD：
      RGB | #001234 | 14:23:45.67 | dt=+12ms vs IR
    Δt 绝对值超过 50ms 时显红色高亮，提示帧同步问题。
    注：OpenCV 默认 Hershey 字体不支持 Unicode，所以用 ASCII 'dt' 代替 'Δt'。
    """
    h, w = frame.shape[:2]
    ov = frame.copy()
    cv2.rectangle(ov, (0, 0), (w, 32), (0, 0, 0), -1)
    cv2.addWeighted(ov, 0.55, frame, 0.45, 0, frame)

    ts_str = time.strftime("%H:%M:%S", time.localtime(ts)) + \
             f".{int((ts % 1) * 100):02d}"
    peer = "IR" if cam_name == "RGB" else "RGB"
    dt_sign = f"{dt_ms:+d}ms"
    text = f"{cam_name} | #{frame_id:06d} | {ts_str} | dt={dt_sign} vs {peer}"

    # Δt 超阈值变红，提示同步偏差
    color = (0, 255, 200) if abs(dt_ms) < 50 else (80, 80, 255)
    cv2.putText(frame, text, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════
#  单路摄像头采集线程
# ══════════════════════════════════════════════════════════════
class CameraThread(threading.Thread):
    """
    独立线程采集单路摄像头：
      - 开机先 v4l2-ctl 锁 ISP
      - VideoCapture 打开，设 FPS / 分辨率 / MJPEG（IR）
      - 循环：读帧 → 发布 ROS Image（纯净）→ 推 MJPEG 流（带 OSD）
    """

    def __init__(
        self,
        device: str,
        topic: str,
        ros_node: Node,
        bridge: CvBridge,
        mjpeg: MJPEGServer,
        cam_name: str,
        sync: FrameSync,
        force_mjpeg: bool = False,
        exposure: int | None = None,
        wb_temp: int | None = None,
        skip_wb: bool = False,
    ):
        super().__init__(daemon=True)
        self.device      = device
        self.topic       = topic
        self.node        = ros_node
        self.bridge      = bridge
        self.mjpeg       = mjpeg
        self.cam_name    = cam_name
        self.sync        = sync
        self.force_mjpeg = force_mjpeg
        self.exposure    = exposure
        self.wb_temp     = wb_temp
        self.skip_wb     = skip_wb

        self.pub         = ros_node.create_publisher(Image, topic, 10)
        self.frame_id    = 0
        self._latest     = None
        self._frame_lock = threading.Lock()
        self._running    = True

    def get_latest_frame(self) -> np.ndarray:
        """线程安全取最新帧（同包其他节点绕过 ROS 订阅时用）"""
        with self._frame_lock:
            return None if self._latest is None else self._latest.copy()

    def stop(self):
        self._running = False

    def run(self):
        logger = self.node.get_logger()

        # Step 1: 解析符号链接为真实 /dev/videoN 路径
        # （OpenCV V4L2 后端在部分版本对 by-id 符号链接有 "can't capture by name" 的兼容问题）
        device_real = os.path.realpath(self.device)
        if device_real != self.device:
            logger.info(
                f"[{self.cam_name}] 解析 {self.device} → {device_real}"
            )

        # Step 2：打开 VideoCapture
        # ISP 已在主线程里串行锁过了，这里不重复做
        cap = cv2.VideoCapture(device_real, cv2.CAP_V4L2)

        # IR 强制 MJPEG 格式，解决 USB 带宽读帧失败
        if self.force_mjpeg:
            cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
            )

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)    # 最小缓冲降延迟

        if not cap.isOpened():
            logger.error(
                f"[{self.cam_name}] 无法打开 {self.device}！\n"
                f"  排查: ls -l /dev/v4l/by-id/  确认 udev 链接存在\n"
                f"  权限: sudo chmod 666 /dev/video*"
            )
            return

        logger.info(
            f"[{self.cam_name}] 摄像头已打开：{self.device}"
            + ("  (MJPEG 模式)" if self.force_mjpeg else "")
        )

        # Step 3：打印 ISP 读回值，验证锁定是否真正生效
        self._log_isp_readback(logger)

        # ★ Step 4：采集主循环
        consecutive_fail = 0
        while self._running:
            ret, frame = cap.read()

            if not ret:
                consecutive_fail += 1
                if consecutive_fail <= 5:
                    logger.warn(
                        f"[{self.cam_name}] 读帧失败 ({consecutive_fail})，重试..."
                    )
                elif consecutive_fail == 6:
                    logger.warn(
                        f"[{self.cam_name}] 持续读帧失败，已静默重试 "
                        f"(可用 'v4l2-ctl --device={self.device} "
                        f"--list-formats-ext' 排查)"
                    )
                time.sleep(0.05)
                continue

            consecutive_fail = 0
            self.frame_id += 1
            ts = time.time()

            # 更新同步追踪器（供另一路 OSD 计算 Δt）
            self.sync.update(self.cam_name, self.frame_id, ts)
            dt_ms = self.sync.peer_delta_ms(self.cam_name, ts)

            # 缓存原始帧
            with self._frame_lock:
                self._latest = frame.copy()

            # 发布 ROS Image（纯净，不含 OSD）
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp    = self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.cam_name
                self.pub.publish(msg)
            except Exception as e:
                logger.error(f"[{self.cam_name}] ROS publish 失败：{e}")

            # 推 MJPEG 流（带 OSD 和 Δt 同步指示）
            stream_frame = frame.copy()
            _draw_osd(stream_frame, self.cam_name, self.frame_id, ts, dt_ms)
            self.mjpeg.push_frame(stream_frame)

        cap.release()
        logger.info(f"[{self.cam_name}] 摄像头已释放")

    def _log_isp_readback(self, logger):
        """调用 v4l2-ctl 读回关键 ISP 参数，确认锁定真正生效。"""
        try:
            r = subprocess.run(
                ['v4l2-ctl', '-d', os.path.realpath(self.device), '--get-ctrl',
                 'auto_exposure,exposure_time_absolute,'
                 'white_balance_automatic,white_balance_temperature'],
                capture_output=True, text=True, timeout=2,
            )
            if r.returncode == 0 and r.stdout.strip():
                for line in r.stdout.strip().split('\n'):
                    logger.info(f"[{self.cam_name}] 读回: {line.strip()}")
        except Exception:
            pass   # 读回失败不影响运行


# ══════════════════════════════════════════════════════════════
#  ROS2 主节点
# ══════════════════════════════════════════════════════════════
class StereoCameraNode(Node):

    def __init__(self):
        super().__init__("stereo_camera")
        self.bridge = CvBridge()
        self.sync   = FrameSync()

        # 两路 MJPEG 流
        self.mjpeg_rgb = MJPEGServer(STREAM_PORT_RGB, "RGB")
        self.mjpeg_ir  = MJPEGServer(STREAM_PORT_IR,  "IR")
        self.mjpeg_rgb.start()
        self.mjpeg_ir.start()

        # ★ v3.1: 串行锁定 ISP（两颗相机共用 USB hub 时并发会撞车）
        # 顺序：先 RGB 再 IR，完全做完再启动采集线程
        logger = self.get_logger()
        logger.info("━━ 开始串行 ISP 锁定 ━━")
        lock_isp(
            os.path.realpath(RGB_DEVICE), "RGB", logger,
            exposure=RGB_EXPOSURE, wb_temp=RGB_WB_TEMP, skip_wb=False,
        )
        lock_isp(
            os.path.realpath(IR_DEVICE), "IR", logger,
            exposure=IR_EXPOSURE, wb_temp=None, skip_wb=True,
        )
        logger.info("━━ ISP 锁定完成 ━━")

        # 两路采集线程（IR 强制 MJPEG + 跳过白平衡）
        self.cam_rgb = CameraThread(
            RGB_DEVICE, TOPIC_RGB, self, self.bridge,
            self.mjpeg_rgb, "RGB", self.sync,
            force_mjpeg=False,
            exposure=RGB_EXPOSURE,
            wb_temp=RGB_WB_TEMP,
            skip_wb=False,
        )
        self.cam_ir = CameraThread(
            IR_DEVICE, TOPIC_IR, self, self.bridge,
            self.mjpeg_ir, "IR", self.sync,
            force_mjpeg=True,
            exposure=IR_EXPOSURE,
            wb_temp=None,
            skip_wb=True,            # IR 是灰度，没 WB 概念
        )
        self.cam_rgb.start()
        self.cam_ir.start()

        self._banner()

    def _banner(self):
        isp_tag = "✅ 已锁死" if LOCK_ISP else "⚠️  未锁（NDVI 会失效）"
        log = self.get_logger().info
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log("  双目摄像头节点已启动  v3.2")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log(f"  ISP 锁定     : {isp_tag}")
        log(f"  RGB topic    : {TOPIC_RGB}")
        log(f"  IR  topic    : {TOPIC_IR}")
        log(f"  RGB 网络流   : http://0.0.0.0:{STREAM_PORT_RGB}")
        log(f"  IR  网络流   : http://0.0.0.0:{STREAM_PORT_IR}")
        log("  ─────────────────────────────────────────")
        log("  【YOLO 同学接口】")
        log(f"  订阅 {TOPIC_RGB} 做目标检测 (BPU)")
        log("  发布 /yolo/weed_detected (std_msgs/String, JSON)")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    def get_rgb_frame(self) -> np.ndarray:
        """供同包其他节点直接取最新 RGB 帧（无需 ROS 订阅）"""
        return self.cam_rgb.get_latest_frame()

    def get_ir_frame(self) -> np.ndarray:
        return self.cam_ir.get_latest_frame()

    def destroy_node(self):
        self.cam_rgb.stop()
        self.cam_ir.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
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
