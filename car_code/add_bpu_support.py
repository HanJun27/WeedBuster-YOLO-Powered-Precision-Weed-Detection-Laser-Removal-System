#!/usr/bin/env python3
"""Add BPU support to yolo_detector.py"""

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/yolo_detector.py'

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Check if already has BPU support
if 'model.to' in content and ('npu' in content or 'NPU' in content):
    print('✅ BPU support already exists')
    exit(0)

# Replace model loading section
old_text = '''        try:
            from ultralytics import YOLO
            self.model = YOLO(MODEL_PATH)
            self.get_logger().info("✅ 模型加载成功")

            # 打印模型信息
            info = self.model.info()
            self.get_logger().info(f"模型层数: {info[0]}, 参数量: {info[1]:,}")'''

new_text = '''        try:
            from ultralytics import YOLO
            
            # 尝试使用 BPU 加速（RDK X5）
            try:
                self.model = YOLO(MODEL_PATH)
                self.model.to('npu')
                self.use_bpu = True
                self.get_logger().info("✅ 模型已加载到 BPU (NPU)")
            except Exception as npu_err:
                self.get_logger().warning("BPU 初始化失败，回退到 CPU")
                self.model = YOLO(MODEL_PATH)
                self.use_bpu = False
            
            # 打印模型信息
            info = self.model.info()
            self.get_logger().info(f"模型层数: {info[0]}, 参数量: {info[1]:,}")
            device_str = 'BPU/NPU' if self.use_bpu else 'CPU'
            self.get_logger().info(f"推理设备: {device_str}")'''

if old_text in content:
    content = content.replace(old_text, new_text)
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print('✅ BPU support added successfully!')
else:
    print('❌ Could not find target text')
    print('Searching for similar patterns...')
    if 'self.model = YOLO(MODEL_PATH)' in content:
        print('Found YOLO model loading line')
    else:
        print('Model loading line not found')
