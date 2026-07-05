#!/usr/bin/env python3
"""
yolo_detector.py —— YOLO 杂草检测节点 (v3.9.9 统一版；v3.10.7 加 stamp 字段)

v3.11.1: ExG 假草过滤改为运行时可切换。新增订阅 /yolo/exg_enable (Bool):
  vision_servo 8093 网页[ExG]按钮发布 → 本节点即时开/关 self._exg_enable
  (filter_boxes_by_exg 接收 enable 参数,不再硬读 config 常量),无需重启即可现场
  A/B 对比"开/关 ExG"、验证新模型是否真不空锁。初值仍取 config 的 EXG_FILTER_ENABLE。

v3.11.0: 集成负反馈训练新模型(仅替换 models/ 下 quant.bin/best.pt/best.onnx/
  quant_info.json,**代码逻辑零改动**,仅本头注释 + 三处版本号变更)。新模型加
  泥土负样本重训,解决旧模型空场景"空锁"误报。指标 mAP50=0.913 / mAP50-95=0.665
  / P=0.910 / R=0.866。int8 量化输出余弦相似度 0.9991(行为与 float 几乎一致,
  负样本"看到空地不报警"成果完整保留)。BPU 计算估算 ~175 FPS(端到端不变)。
  模型 IO 与旧版完全一致(输入 NV12 1x3x640x640、输出 (1,6,8400,1)),drop-in。
  新模型既已解决空锁,config.py 的 EXG_FILTER_ENABLE 可按需置 False(本版未改,
  保持原值,留给你自行决定)。

v3.10.15: 修复 v3.10.13 引入的回归 —— 插入 ExG 函数时误删了 postprocess_bpu
  的函数头(def 行),致 BPU 推理报 "name 'postprocess_bpu' is not defined"。
  已补回函数头并加 AST 导入级自检(py_compile 查不出"未定义名",故新增此检查)。

v3.10.13: 新增 ExG 超绿指数假草过滤(EXG_* 配置见 config.py)。模型训练缺
  纯背景负样本,空场景会"空锁"误报;推理后对每个框查中心区"活体绿"占比,
  过低判为假草丢弃,全丢则该帧 detected=False。过渡防御,模型重训后可关。
  boxes 每项新增 green_ratio 字段(答辩可用的可解释性数据)。

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
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

from laser_calibration.config import (
    TOPIC_RGB, TOPIC_YOLO, TOPIC_EXG_ENABLE,
    EXG_FILTER_ENABLE, EXG_MIN_RATIO, EXG_CENTER_FRAC, EXG_THRESH,
    EXG_G_DOMINANCE, EXG_OVEREXP, EXG_SHADOW_SUM,
)

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


def exg_green_ratio(bgr, cx, cy, w, h):
    """v3.10.13: 框中心区"活体绿"像素占比,用于拦截 YOLO 空锁误检。
    判据(同时满足):① 归一化超绿指数 ExG=2g-r-b > EXG_THRESH;
    ② G > R×EXG_G_DOMINANCE(绿压过红,排除中性灰褐土壤/枯叶);
    ③ 非过曝白、非死黑。返回 [0,1] 占比。纯计算,无副作用。"""
    if bgr is None or bgr.size == 0:
        return 0.0
    ih, iw = bgr.shape[:2]
    x1 = max(0, int(cx - w / 2)); x2 = min(iw, int(cx + w / 2))
    y1 = max(0, int(cy - h / 2)); y2 = min(ih, int(cy + h / 2))
    crop = bgr[y1:y2, x1:x2]
    if crop.size == 0:
        return 0.0
    ch, cw = crop.shape[:2]
    # 只取中心区,避开框边缘混入的土壤
    mh = int(ch * (1 - EXG_CENTER_FRAC) / 2)
    mw = int(cw * (1 - EXG_CENTER_FRAC) / 2)
    roi = crop[mh:ch - mh, mw:cw - mw] if (ch > 4 and cw > 4) else crop
    if roi.size == 0:
        return 0.0
    b = roi[:, :, 0].astype(np.float32)
    g = roi[:, :, 1].astype(np.float32)
    r = roi[:, :, 2].astype(np.float32)
    s = r + g + b
    s[s == 0] = 1e-6
    exg = 2.0 * (g / s) - (r / s) - (b / s)
    g_dominant  = g > (r * EXG_G_DOMINANCE)
    overexposed = (r > EXG_OVEREXP) & (g > EXG_OVEREXP) & (b > EXG_OVEREXP)
    shadows     = s < EXG_SHADOW_SUM
    mask = (exg > EXG_THRESH) & g_dominant & (~overexposed) & (~shadows)
    return float(np.sum(mask)) / mask.size


def filter_boxes_by_exg(bgr, boxes, logger=None, enable=EXG_FILTER_ENABLE):
    """对一帧的 boxes 逐个做 ExG 过滤,返回保留的 boxes(每个补 green_ratio 字段)。
    全部被过滤则返回空列表 → 上层 detected 自然置 False。
    v3.11.1: enable 由运行时开关(/yolo/exg_enable)传入,缺省取 config 的 EXG_FILTER_ENABLE。"""
    if not enable or not boxes:
        return boxes
    kept, dropped = [], 0
    for bx in boxes:
        ratio = exg_green_ratio(bgr, bx["cx"], bx["cy"], bx["w"], bx["h"])
        bx["green_ratio"] = round(ratio, 3)
        if ratio >= EXG_MIN_RATIO:
            kept.append(bx)
        else:
            dropped += 1
    if dropped and logger is not None:
        logger.warn(
            f"[ExG] 拦截 {dropped} 个疑似空锁假草"
            f"(绿占比 < {EXG_MIN_RATIO*100:.0f}%),保留 {len(kept)} 个真目标")
    return kept


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

        # ROS2
        self.sub = self.create_subscription(
            Image, TOPIC_RGB, self.image_callback, 10
        )
        self.pub = self.create_publisher(String, TOPIC_YOLO, 10)
        # v3.11.1: ExG 假草过滤运行时开关 —— 网页(vision_servo 8093)按钮发布,
        #   不重启即可现场 A/B 对比。初值取 config 的 EXG_FILTER_ENABLE。
        self._exg_enable = EXG_FILTER_ENABLE
        self.sub_exg = self.create_subscription(
            Bool, TOPIC_EXG_ENABLE, self._cb_exg_enable, 10)
        self.timer = self.create_timer(0.1, self.publish_timer_callback)  # 10Hz

        device = "BPU" if self.use_bpu else "CPU"
        self.get_logger().info(
            f"🚀 YOLO 节点已启动 ({device})，10Hz 发布到 {TOPIC_YOLO}"
        )
        self.get_logger().info(
            f"  ExG 假草过滤: {'开' if self._exg_enable else '关'}"
            f"（运行时可经 {TOPIC_EXG_ENABLE} 切换）")

    def _cb_exg_enable(self, msg):
        """v3.11.1: 运行时开/关 ExG 假草过滤。"""
        new_state = bool(msg.data)
        if new_state != self._exg_enable:
            self._exg_enable = new_state
            self.get_logger().warn(
                f"[ExG] 运行时开关 → {'开启' if new_state else '关闭'}"
                f"（{'按 green_ratio 拦截疑似空锁' if new_state else '不再过滤,全部 YOLO 框直通'}）")

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

        # v3.10.13: ExG 物理光谱闸门 —— 拦截 YOLO 空锁假草(详见 config 注释)。
        #   在此统一过滤,BPU/CPU 两路都覆盖;bgr 即本帧原图。全被拦则 boxes 空,
        #   publish_timer 自然发 detected=False,下游从源头收不到假目标。
        boxes = filter_boxes_by_exg(bgr, boxes, self.get_logger(), self._exg_enable)

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
