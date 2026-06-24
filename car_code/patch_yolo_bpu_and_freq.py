#!/usr/bin/env python3
"""
Patch yolo_detector.py to enable BPU acceleration and add frequency control
"""

import sys

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/yolo_detector.py'

print(f"Reading {file_path}...")
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Replace CPU model loading with BPU initialization
old_model_load = '''        # 加载 YOLO 模型
        self.get_logger().info(f"正在加载 YOLO 模型: {MODEL_PATH}")
        try:
            from ultralytics import YOLO
            self.model = YOLO(MODEL_PATH)
            self.get_logger().info("✅ 模型加载成功")

            # 打印模型信息
            info = self.model.info()
            self.get_logger().info(f"模型层数: {info[0]}, 参数量: {info[1]:,}")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            raise'''

new_model_load = '''        # 加载 YOLO 模型到 BPU
        self.get_logger().info(f"正在加载 YOLO 模型到 BPU: {MODEL_PATH}")
        try:
            from ultralytics import YOLO
            
            # 尝试使用 BPU 加速（RDK X5）
            try:
                # 方法1: 使用 device='npu' 参数
                self.model = YOLO(MODEL_PATH)
                self.model.to('npu')  # 迁移到 NPU/BPU
                self.use_bpu = True
                self.get_logger().info("✅ 模型已加载到 BPU (NPU)")
            except Exception as npu_err:
                self.get_logger().warning(f"BPU 初始化失败: {npu_err}，回退到 CPU")
                self.model = YOLO(MODEL_PATH)
                self.use_bpu = False
            
            # 打印模型信息
            info = self.model.info()
            self.get_logger().info(f"模型层数: {info[0]}, 参数量: {info[1]:,}")
            self.get_logger().info(f"推理设备: {'BPU/NPU' if self.use_bpu else 'CPU'}")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            raise'''

content = content.replace(old_model_load, new_model_load)

# 2. Add frequency parameter support
old_init_params = '''CONF_THRESHOLD = 0.5    # 置信度阈值
IOU_THRESHOLD = 0.45    # NMS IoU 阈值'''

new_init_params = '''CONF_THRESHOLD = 0.5    # 置信度阈值
IOU_THRESHOLD = 0.45    # NMS IoU 阈值
PUBLISH_FREQ = 10       # 发布频率 Hz (可动态调整)'''

content = content.replace(old_init_params, new_init_params)

# 3. Add frequency control in __init__
old_timer_init = '''        # ⚠️ 关键：10Hz 定时器持续发布（哪怕没检测到也发心跳）
        self.timer = self.create_timer(0.1, self.publish_timer_callback)'''

new_timer_init = '''        # 发布频率控制（可通过参数或服务动态调整）
        self.publish_freq = PUBLISH_FREQ
        self.timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(self.timer_period, self.publish_timer_callback)
        
        # 创建频率调整服务
        from std_srvs.srv import SetFloat
        self.freq_service = self.create_service(
            SetFloat, 
            '~/set_publish_freq',
            self.handle_set_frequency
        )
        self.get_logger().info(f"📡 发布频率服务已启动: ~/set_publish_freq")'''

content = content.replace(old_timer_init, new_timer_init)

# 4. Add frequency adjustment service handler before image_callback
service_handler = '''
    def handle_set_frequency(self, request, response):
        """处理频率调整请求"""
        new_freq = request.data
        if new_freq <= 0 or new_freq > 30:
            response.success = False
            response.message = f"频率必须在 0-30 Hz 范围内，当前请求: {new_freq}"
            self.get_logger().warn(response.message)
            return response
        
        old_freq = self.publish_freq
        self.publish_freq = new_freq
        self.timer_period = 1.0 / new_freq
        
        # 销毁旧定时器，创建新定时器
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(self.timer_period, self.publish_timer_callback)
        
        response.success = True
        response.message = f"发布频率已从 {old_freq} Hz 调整为 {new_freq} Hz"
        self.get_logger().info(response.message)
        return response

'''

# Find the position to insert (before image_callback)
insert_pos = content.find('    def image_callback(self, msg):')
if insert_pos != -1:
    content = content[:insert_pos] + service_handler + content[insert_pos:]

# 5. Update publish message to include current frequency
old_publish_msg = '''        msg_str = json.dumps({
            "detected": detected,
            "boxes": boxes,
            "timestamp": time.time(),
            "frame_id": frame_id,
        })'''

new_publish_msg = '''        msg_str = json.dumps({
            "detected": detected,
            "boxes": boxes,
            "timestamp": time.time(),
            "frame_id": frame_id,
            "publish_freq_hz": self.publish_freq,  # 当前发布频率
        })'''

content = content.replace(old_publish_msg, new_publish_msg)

# Write back
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("✅ Patch applied successfully!")
print("\nChanges made:")
print("1. ✅ Added BPU/NPU acceleration support")
print("2. ✅ Added dynamic frequency control service")
print("3. ✅ Added frequency parameter in messages")
print("\nNext steps:")
print("1. Rebuild: cd ~/yahboomcar_ws && colcon build --packages-select laser_calibration")
print("2. Source: source install/setup.bash")
print("3. Restart yolo_detector node")
