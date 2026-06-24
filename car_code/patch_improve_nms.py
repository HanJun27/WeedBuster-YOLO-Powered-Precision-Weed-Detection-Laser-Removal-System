#!/usr/bin/env python3
"""
Improve NMS in yolo_detector.py to fix nested box issue
"""

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/yolo_detector.py'

print(f"Reading {file_path}...")
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find and modify the inference call to use better NMS parameters
modified = False
for i, line in enumerate(lines):
    # Look for the model inference call
    if 'self.model(' in line and 'img' in line:
        # Check if there are already NMS parameters
        context = ''.join(lines[max(0,i-2):i+5])
        
        if 'iou=' not in context and 'conf=' not in context:
            # Replace with enhanced inference call
            old_line = line.rstrip()
            new_lines = [
                line.replace(
                    'results = self.model(img',
                    'results = self.model(\n                img'
                ).rstrip(),
            ]
            
            # Find the closing parenthesis and add NMS parameters before it
            for j in range(i, min(i+10, len(lines))):
                if ')' in lines[j] and 'results' in ''.join(lines[i:j+1]):
                    # Insert NMS parameters before the closing )
                    indent = '                '
                    nms_params = [
                        f'{indent}conf=CONF_THRESHOLD,  # 置信度阈值\n',
                        f'{indent}iou=IOU_THRESHOLD,    # NMS IoU 阈值（抑制重叠框）\n',
                        f'{indent}max_det=10,           # 最大检测数量\n',
                        f'{indent}agnostic_nms=False,   # 类别感知 NMS\n',
                    ]
                    
                    # Insert before the closing )
                    lines[j] = lines[j].replace(')', ',\n' + ''.join(nms_params) + indent + ')')
                    modified = True
                    print(f"✅ Enhanced NMS parameters at line {j+1}")
                    break
            break

if not modified:
    print("⚠️  Could not find inference call to modify. Manual check needed.")
else:
    # Write back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    
    print("✅ NMS enhancement applied successfully!")
    print("\nImprovements:")
    print("1. ✅ Added conf threshold filtering")
    print("2. ✅ Added IoU threshold for NMS (reduces nested boxes)")
    print("3. ✅ Limited max detections to 10")
    print("4. ✅ Using class-aware NMS")
