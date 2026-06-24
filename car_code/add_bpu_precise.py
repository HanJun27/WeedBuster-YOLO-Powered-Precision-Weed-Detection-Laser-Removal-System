#!/usr/bin/env python3
"""Add BPU support - precise version"""

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/yolo_detector.py'

with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find the model loading section and replace it
modified = False
for i in range(len(lines)):
    if '# 加载 YOLO 模型' in lines[i] and 'MODEL_PATH' in lines[i+1]:
        # Found the section, replace from line i to i+12 (the raise statement)
        indent = '        '
        new_section = [
            f'{indent}# 加载 YOLO 模型到 BPU\n',
            f'{indent}self.get_logger().info(f"正在加载 YOLO 模型到 BPU: {{MODEL_PATH}}")\n',
            f'{indent}try:\n',
            f'{indent}    from ultralytics import YOLO\n',
            f'{indent}    \n',
            f'{indent}    # 尝试使用 BPU 加速（RDK X5）\n',
            f"{indent}    try:\n",
            f'{indent}        self.model = YOLO(MODEL_PATH)\n',
            f"{indent}        self.model.to('npu')\n",
            f'{indent}        self.use_bpu = True\n',
            f'{indent}        self.get_logger().info("✅ 模型已加载到 BPU (NPU)")\n',
            f'{indent}    except Exception as npu_err:\n',
            f'{indent}        self.get_logger().warning("BPU 初始化失败，回退到 CPU")\n',
            f'{indent}        self.model = YOLO(MODEL_PATH)\n',
            f'{indent}        self.use_bpu = False\n',
            f'{indent}    \n',
            f'{indent}    # 打印模型信息\n',
            f'{indent}    info = self.model.info()\n',
            f'{indent}    self.get_logger().info(f"模型层数: {{info[0]}}, 参数量: {{info[1]:,}}")\n',
            f"{indent}    device_str = 'BPU/NPU' if self.use_bpu else 'CPU'\n",
            f'{indent}    self.get_logger().info(f"推理设备: {{device_str}}")\n',
            f'{indent}except Exception as e:\n',
            f'{indent}    self.get_logger().error(f"❌ 模型加载失败: {{e}}")\n',
            f'{indent}    raise\n',
        ]
        
        # Replace lines[i] to lines[i+13]
        lines[i:i+14] = new_section
        modified = True
        print(f'✅ BPU support added at line {i+1}')
        break

if modified:
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    print('✅ File saved successfully!')
else:
    print('❌ Could not find model loading section')
