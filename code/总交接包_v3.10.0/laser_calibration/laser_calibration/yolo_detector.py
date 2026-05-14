#!/usr/bin/env python3
"""
yolo_detector.py —— YOLO 杂草检测节点 (v3.9.9 统一版)

启动时优先尝试 BPU 推理（hobot_dnn）；失败则回退到 CPU 推理（ultralytics）。
两种模式接口完全一致。

性能：BPU ~26ms/帧（76 FPS），CPU ~150ms/帧。

订阅: /camera/rgb/image_raw  (sensor_msgs/Image, bgr8, 640x480)
发布: /yolo/weed_detected   (std_msgs/String, JSON, 10Hz 持续)

模型信息（hrt_model_exec model_info 验证）:
  输入: NV12, (1,3,640,640), HB_DNN_INPUT_FROM_PYRAMID
  输出: Float32, (1,6,8400,1)
        6 = 4 (cx,cy,w,h 已 dist2bbox 解码到 640 坐标系)
          + 2 (类别概率，已 sigmoid)
        8400 = 80² + 40² + 20²
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from laser_calibration.config import TOPIC_RGB, TOPIC_YOLO

# ═════════════════════════════════════════════════════════════
MODEL_PATH_BPU = "/home/sunrise/yahboomcar_ws/src/laser_calibration/models/quant.bin"
MODEL_PATH_CPU = "/home/sunrise/yahboomcar_ws/src/laser_calibration/models/best.pt"
CONF_THRESHOLD = 0.5
IOU_THRESHOLD  = 0.45
NUM_CLASSES    = 2
INPUT_SIZE     = 640
SRC_W, SRC_H   = 640, 480
CLASS_NAMES    = ["weed", "crop"]


# ═════════════════════════════════════════════════════════════
#  BPU 预处理 + 后处理
# ═════════════════════════════════════════════════════════════
def letterbox(img_bgr, target=INPUT_SIZE, pad_value=114):
    """640×480 BGR → 640×640 with grey padding"""
    h, w = img_bgr.shape[:2]
    scale = min(target / w, target / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    if (new_w, new_h) != (w, h):
        img_bgr = cv2.resize(img_bgr, (new_w, new_h),
                             interpolation=cv2.INTER_LINEAR)
    pad_w = (target - new_w) // 2
    pad_h = (target - new_h) // 2
    canvas = np.full((target, target, 3), pad_value, dtype=np.uint8)
    canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = img_bgr
    return canvas, scale, pad_w, pad_h


def bgr_to_nv12(bgr):
    """OpenCV BGR → NV12 byte array"""
    h, w = bgr.shape[:2]
    yuv_i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    flat = yuv_i420.reshape(-1)
    y_size  = h * w
    uv_size = y_size // 4
    nv12 = np.empty(y_size + 2 * uv_size, dtype=np.uint8)
    nv12[:y_size] = flat[:y_size]
    nv12[y_size + 0::2] = flat[y_size : y_size + uv_size]
    nv12[y_size + 1::2] = flat[y_size + uv_size : y_size + 2 * uv_size]
    return nv12


def postprocess_bpu(raw_output, scale, pad_w, pad_h):
    """YOLOv8 (1,6,8400,1) Float32 → boxes 列表 (640×480 坐标)"""
    out = raw_output.reshape(6, 8400).T  # (8400, 6)
    boxes_xywh = out[:, :4]
    cls_scores = out[:, 4 : 4 + NUM_CLASSES]

    max_scores  = cls_scores.max(axis=1)
    max_classes = cls_scores.argmax(axis=1)

    keep = max_scores >= CONF_THRESHOLD
    if not keep.any():
        return []
    boxes_xywh = boxes_xywh[keep]
    scores     = max_scores[keep]
    classes    = max_classes[keep]

    xyxy = np.empty_like(boxes_xywh)
    xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
    xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
    xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
    xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2

    indices = cv2.dnn.NMSBoxes(
        xyxy.tolist(),
        scores.astype(float).tolist(),
        CONF_THRESHOLD,
        IOU_THRESHOLD,
    )
    if len(indices) == 0:
        return []
    if hasattr(indices, "flatten"):
        indices = indices.flatten()

    results = []
    for i in indices:
        cx, cy, w, h = boxes_xywh[i]
        cx = (cx - pad_w) / scale
        cy = (cy - pad_h) / scale
        w  = w / scale
        h  = h / scale
        cx = int(max(0, min(SRC_W - 1, round(cx))))
        cy = int(max(0, min(SRC_H - 1, round(cy))))
        w  = int(max(1, min(SRC_W, round(w))))
        h  = int(max(1, min(SRC_H, round(h))))
        cls_id = int(classes[i])
        results.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "confidence": round(float(scores[i]), 3),
            "label": CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"class_{cls_id}",
        })
    return results


# ═════════════════════════════════════════════════════════════
#  CPU 后处理（ultralytics）
# ═════════════════════════════════════════════════════════════
def postprocess_cpu(results_cpu, cpu_model):
    boxes = []
    if len(results_cpu) == 0 or results_cpu[0].boxes is None:
        return boxes
    for box in results_cpu[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0].cpu().numpy())
        cls  = int(box.cls[0].cpu().numpy())
        label = cpu_model.names[cls] if cls < len(cpu_model.names) else "unknown"
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        w  = int(x2 - x1)
        h  = int(y2 - y1)
        cx = max(0, min(SRC_W - 1, cx))
        cy = max(0, min(SRC_H - 1, cy))
        w  = max(1, min(SRC_W, w))
        h  = max(1, min(SRC_H, h))
        boxes.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "confidence": round(conf, 3),
            "label": label,
        })
    return boxes


# ═════════════════════════════════════════════════════════════
#  ROS2 节点
# ═════════════════════════════════════════════════════════════
class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.bridge = CvBridge()
        self.use_bpu = False
        self.bpu_model = None
        self.cpu_model = None

        self.get_logger().info(f"[YOLO] 尝试 BPU 模式: {MODEL_PATH_BPU}")
        try:
            from hobot_dnn import pyeasy_dnn as dnn
            self.bpu_models = dnn.load(MODEL_PATH_BPU)
            self.bpu_model = self.bpu_models[0]
            inp = self.bpu_model.inputs[0]
            out = self.bpu_model.outputs[0]
            self.get_logger().info("✅ BPU 模型加载成功")
            self.get_logger().info(f"  输入: {inp.name}")
            self.get_logger().info(f"  输出: {out.name}")
            self.use_bpu = True
        except Exception as e:
            self.get_logger().warn(f"⚠️ BPU 不可用: {e}")
            self.get_logger().info("[YOLO] 回退到 CPU 模式")
            self._load_cpu_model()

        # 缓存
        self._lock = threading.Lock()
        self._latest_boxes = []
        self._latest_frame_id = 0
        self._last_inference_time = 0.0
        self._inference_history = []

        # ROS2
        self.sub = self.create_subscription(
            Image, TOPIC_RGB, self.image_callback, 10
        )
        self.pub = self.create_publisher(String, TOPIC_YOLO, 10)
        self.timer = self.create_timer(0.1, self.publish_timer_callback)  # 10Hz

        device = "BPU" if self.use_bpu else "CPU"
        self.get_logger().info(
            f"🚀 YOLO 节点已启动 ({device})，10Hz 发布到 {TOPIC_YOLO}"
        )

    def _load_cpu_model(self):
        try:
            from ultralytics import YOLO
            self.cpu_model = YOLO(MODEL_PATH_CPU)
            info = self.cpu_model.info()
            self.get_logger().info(
                f"✅ CPU 模型加载成功 层数={info[0]} 参数={info[1]:,}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ CPU 模型加载失败: {e}")
            raise

    def _infer_bpu(self, bgr):
        letterboxed, scale, pad_w, pad_h = letterbox(bgr, INPUT_SIZE)
        nv12 = bgr_to_nv12(letterboxed)
        outputs = self.bpu_model.forward(nv12)
        raw = outputs[0].buffer
        return postprocess_bpu(raw, scale, pad_w, pad_h)

    def _infer_cpu(self, bgr):
        results = self.cpu_model(
            bgr,
            conf=CONF_THRESHOLD,
            iou=IOU_THRESHOLD,
            verbose=False,
            max_det=10,
        )
        return postprocess_cpu(results, self.cpu_model)

    def image_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像解码失败: {e}")
            return

        t0 = time.time()
        try:
            if self.use_bpu:
                boxes = self._infer_bpu(bgr)
            else:
                boxes = self._infer_cpu(bgr)
        except Exception as e:
            self.get_logger().error(f"推理失败: {e}")
            return
        inference_ms = (time.time() - t0) * 1000

        with self._lock:
            self._latest_boxes = boxes
            self._latest_frame_id += 1
            self._last_inference_time = time.time()
            self._inference_history.append(inference_ms)
            if len(self._inference_history) > 30:
                self._inference_history.pop(0)

        if self._latest_frame_id % 30 == 0:
            avg = sum(self._inference_history) / len(self._inference_history)
            device = "BPU" if self.use_bpu else "CPU"
            self.get_logger().info(
                f"[{device}] frame={self._latest_frame_id}  "
                f"inference={inference_ms:.1f}ms (avg {avg:.1f}ms)  "
                f"boxes={len(boxes)}"
            )

    def publish_timer_callback(self):
        with self._lock:
            boxes = list(self._latest_boxes)
            frame_id = self._latest_frame_id
            age = (time.time() - self._last_inference_time
                   if self._last_inference_time > 0 else 999)

        if age > 1.0:
            payload = {"detected": False, "boxes": [], "stale": True}
        elif boxes:
            payload = {"detected": True, "frame_id": frame_id, "boxes": boxes}
        else:
            payload = {"detected": False, "boxes": []}

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
