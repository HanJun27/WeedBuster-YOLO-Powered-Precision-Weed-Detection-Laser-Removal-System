#!/usr/bin/env python3
"""
yolo_detector.py —— YOLO 杂草检测节点 (v3.9.9 统一版；v3.10.7 加 stamp 字段)

启动时优先尝试 BPU 推理（hobot_dnn）；失败则回退到 CPU 推理（ultralytics）。
两种模式接口完全一致。

性能：BPU ~26ms/帧（76 FPS）；CPU 回退实测因机器而异（数百 ms 量级）。

订阅: /camera/rgb/image_raw  (sensor_msgs/Image, bgr8, 640x480)
发布: /yolo/weed_detected   (std_msgs/String, JSON, 10Hz 持续)
      payload: {detected, frame_id, stamp(源图时间戳/秒), boxes:[{cx,cy,w,h,
      confidence,label}]}。stamp 为 v3.10.7 新增，当前消费端不依赖（走-停-清场），
      留作将来"边走边打"/里程计融合用。

模型信息（hrt_model_exec model_info 验证）:
  输入: NV12, (1,3,640,640), HB_DNN_INPUT_FROM_PYRAMID
  输出: Float32, (1,6,8400,1)
        6 = 4 (cx,cy,w,h 已 dist2bbox 解码到 640 坐标系)
          + 2 (类别概率，已 sigmoid)
        8400 = 80² + 40² + 20²
"""

import json
import os
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
#  绿色占比过滤  v3.11.2
# ═════════════════════════════════════════════════════════════
# 为每个检测框计算 ROI 内 HSV 绿色像素占比。
# 不直接删框，改为对绿色不足的框施以置信度惩罚，由决策层(strike_planner)
# 通过 MIN_CONF 阈值自行决定是否打击。
#
# ⚡ v3.11.2 安全优先策略：
#   旧版(v3.11.0)直接丢弃低绿框 → 可能误删真杂草。
#   新版改为置信度惩罚：green_ratio < 阈值时 confidence × penalty。
#   —— 框永远保留，只是置信度降低，决策层用已有 MIN_CONF 过滤。
#
#   V 阈值继续自适应（v3.11.1）：V_min = max(10, min(80, median_V × 0.3))。
#
#   🌐 阈值和惩罚系数可通过 vision_servo 网页界面(8093)实时调节：
#      vision_servo 写入 ~/.laser_calibration/green_filter.json，
#      本节点每帧推理时读取（缓存 1s），即调即生效，无需重启。

# 默认值（JSON 文件不存在时使用）
GREEN_FILTER_FILE = os.path.expanduser(
    "~/.laser_calibration/green_filter.json")

GREEN_H_MIN = 35                   # H 下限（°），OpenCV 0-180 范围
GREEN_H_MAX = 85                   # H 上限（°）
GREEN_S_MIN = 20                   # S 下限——低曝光时植物饱和度的降，从40降至20


def apply_green_penalty(bgr, boxes, threshold=0.15, penalty=0.3):
    """计算每框绿色占比，低绿框施以置信度惩罚（不删框）。
    
    对每个框的 ROI 独立做 HSV → inRange，V_min 自适应。
    所有框都保留并附带 green_ratio 字段；低于 threshold 的 confidence 乘以 penalty。
    
    Args:
        bgr: 原始 BGR 图像 (H×W×3)
        boxes: list[dict]，每框含 cx/cy/w/h/label/confidence
        threshold: 绿色占比阈值（默认 0.15）
        penalty: 置信度惩罚系数（默认 0.3）
    
    Returns:
        (penalized_boxes, n_total, n_penalized)
        - penalized_boxes: 全部框（被惩罚的 confidence 已降低）
        - n_total: 总框数
        - n_penalized: 被惩罚的框数
    """
    if not boxes or bgr is None:
        return boxes, len(boxes), 0
    
    n_total = len(boxes)
    n_penalized = 0
    
    for box in boxes:
        cx, cy, w, h = box["cx"], box["cy"], box["w"], box["h"]
        # ROI 边界（裁剪到图像有效范围）
        x1 = max(0, cx - w // 2)
        y1 = max(0, cy - h // 2)
        x2 = min(bgr.shape[1], cx + (w + 1) // 2)
        y2 = min(bgr.shape[0], cy + (h + 1) // 2)
        
        roi_bgr = bgr[y1:y2, x1:x2]
        if roi_bgr.size == 0:
            box["green_ratio"] = 0.0
            continue
        
        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        
        # 自适应 V 阈值：取 ROI 内 V 中位数 × 0.3，再限幅到 [10, 80]
        v_median = float(np.median(roi_hsv[:, :, 2]))
        v_min = max(10, min(80, int(round(v_median * 0.30))))
        
        lower = (GREEN_H_MIN, GREEN_S_MIN, v_min)
        upper = (GREEN_H_MAX, 255, 255)
        green_mask = cv2.inRange(roi_hsv, lower, upper)
        
        green_pixels = cv2.countNonZero(green_mask)
        total_pixels = roi_hsv.shape[0] * roi_hsv.shape[1]
        ratio = green_pixels / total_pixels if total_pixels > 0 else 0.0
        
        # 附加绿色占比
        box["green_ratio"] = round(ratio, 3)
        
        # 绿色不足 → 置信度惩罚（但保留框）
        if ratio < threshold:
            box["confidence"] = round(box["confidence"] * penalty, 3)
            box["green_penalty"] = True
            n_penalized += 1
        else:
            box["green_penalty"] = False
    
    return boxes, n_total, n_penalized


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
        self._latest_stamp = None       # v3.10.7: 源图 header.stamp（秒，float）
        self._last_inference_time = 0.0
        self._inference_history = []

        # v3.11.2: 从 vision_servo 网页写入的 shared JSON 动态加载绿滤参数
        self._green_threshold = 0.15
        self._green_penalty   = 0.3
        self._green_config_last_load = 0.0
        self._reload_green_config()

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

    def _reload_green_config(self):
        """从 vision_servo 写入的 shared JSON 加载绿滤参数（缓存 1s）。"""
        now = time.time()
        if now - self._green_config_last_load < 1.0:
            return
        self._green_config_last_load = now
        try:
            with open(GREEN_FILTER_FILE, "r") as f:
                d = json.load(f)
            t = float(d.get("threshold", 0.15))
            p = float(d.get("penalty", 0.3))
            t = max(0.0, min(1.0, t))
            p = max(0.01, min(1.0, p))
            if (abs(t - self._green_threshold) > 0.001
                    or abs(p - self._green_penalty) > 0.001):
                self._green_threshold = t
                self._green_penalty   = p
                self.get_logger().info(
                    f"[绿滤] 已加载新参数: threshold={t:.0%} penalty=×{p:.2f}")
        except (FileNotFoundError, json.JSONDecodeError,
                KeyError, ValueError, TypeError):
            pass  # 文件不存在/损坏 → 沿用当前值

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

        # v3.10.7: 记录源图采集时间戳（秒）。当前消费端（vision_servo/strike_planner）
        #   走"走-停-清场"不依赖它；保留下来为将来"边走边打"/里程计融合留路。
        _st = msg.header.stamp
        src_stamp = float(_st.sec) + float(_st.nanosec) * 1e-9

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

        # v3.11: 绿色占比置信度惩罚 —— 对每个框计算 HSV 绿色像素占比，
        #   低于阈值的不直接丢弃，而是降低其置信度，由决策层决定是否打击。
        self._reload_green_config()
        n_before = len(boxes)
        boxes, _, n_penalized = apply_green_penalty(
            bgr, boxes, self._green_threshold, self._green_penalty)
        if n_before > 0 and self._latest_frame_id % 10 == 0:
            ratios_str = ", ".join(
                f"{b.get('green_ratio', 0):.0%}" for b in boxes)
            self.get_logger().info(
                f"[绿滤] green_ratio=[{ratios_str}]  "
                f"阈={self._green_threshold:.0%} 惩×{self._green_penalty}  "
                f"→ {n_penalized}/{n_before} 被罚")
        elif n_penalized > 0 and self._latest_frame_id % 30 == 0:
            self.get_logger().info(
                f"[绿滤] {n_penalized}/{n_before} 被罚"
            )

        with self._lock:
            self._latest_boxes = boxes
            self._latest_frame_id += 1
            self._latest_stamp = src_stamp
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
            stamp = self._latest_stamp
            age = (time.time() - self._last_inference_time
                   if self._last_inference_time > 0 else 999)

        if age > 1.0:
            payload = {"detected": False, "boxes": [], "stale": True}
        elif boxes:
            # v3.10.7: 附 frame_id + stamp（源图时间戳，秒）。消费端可选用。
            payload = {"detected": True, "frame_id": frame_id,
                       "stamp": stamp, "boxes": boxes}
        else:
            payload = {"detected": False, "frame_id": frame_id,
                       "stamp": stamp, "boxes": []}

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
