#!/usr/bin/env python3
"""Add YOLO box visualization to vision_servo.py"""

import sys

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/vision_servo.py'

print(f"Reading {file_path}...")
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Total lines: {len(lines)}")

# Find and modify specific sections
modified = False

# 1. Add _yolo_boxes initialization after yolo_target_at = 0.0
for i, line in enumerate(lines):
    if 'self.yolo_target_at = 0.0' in line and '_yolo_boxes' not in ''.join(lines[max(0,i-2):i+5]):
        indent = '        '
        lines.insert(i+1, f'{indent}# Cache all YOLO boxes for visualization\n')
        lines.insert(i+2, f'{indent}self._yolo_boxes = []\n')
        print(f"✅ Added _yolo_boxes initialization at line {i+1}")
        modified = True
        break

# 2. Modify _cb_yolo to cache boxes
for i, line in enumerate(lines):
    if 'self.yolo_target_at = time.time()' in line and i > 800:  # Make sure it's in _cb_yolo
        # Check if we already added the cache code
        next_lines = ''.join(lines[i:i+10])
        if '_yolo_boxes' not in next_lines:
            indent = '            '
            lines.insert(i+1, '\n')
            lines.insert(i+2, f'{indent}# Cache all boxes for visualization\n')
            lines.insert(i+3, f'{indent}if boxes:\n')
            lines.insert(i+4, f'{indent}    self._yolo_boxes = boxes\n')
            lines.insert(i+5, f'{indent}else:\n')
            lines.insert(i+6, f'{indent}    self._yolo_boxes = []\n')
            print(f"✅ Added box caching in _cb_yolo at line {i+1}")
            modified = True
        break

# 3. Add yolo_boxes to API state response
for i, line in enumerate(lines):
    if '"predicted_hit": predicted_hit,' in line:
        # Check if already added
        if 'yolo_boxes' not in line:
            indent = '                        '
            lines.insert(i+1, f'{indent}"yolo_boxes": node._yolo_boxes,\n')
            print(f"✅ Added yolo_boxes to API response at line {i+1}")
            modified = True
        break

# 4. Add JavaScript to draw boxes - find the right place in HTML
js_code = '''  // Draw YOLO detection boxes
  if (lastState.yolo_boxes && lastState.yolo_boxes.length > 0) {
    lastState.yolo_boxes.forEach(function(box, idx) {
      const cx = box.cx || 0;
      const cy = box.cy || 0;
      const w = box.w || 50;
      const h = box.h || 50;
      const x1 = cx - w/2;
      const y1 = cy - h/2;
      
      // Draw rectangle
      ctx.strokeStyle = '#0ff';  // Cyan color
      ctx.lineWidth = 2;
      ctx.strokeRect(x1, y1, w, h);
      
      // Draw label
      const label = box.label || 'weed';
      const conf = box.confidence ? box.confidence.toFixed(2) : '?';
      ctx.fillStyle = '#0ff';
      ctx.font = 'bold 12px monospace';
      ctx.fillText(`${label} ${conf}`, x1, Math.max(y1 - 4, 12));
    });
  }
'''

for i, line in enumerate(lines):
    if '// v3.9.3: 蓝紫激光预测落点' in line and i > 200:  # In HTML section
        # Insert before this line
        lines.insert(i, js_code + '\n')
        print(f"✅ Added JavaScript box drawing at line {i}")
        modified = True
        break

if modified:
    # Write back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    print("\n✅ File modified successfully!")
    print("\nNext steps:")
    print("1. cd ~/yahboomcar_ws && colcon build --packages-select laser_calibration")
    print("2. source install/setup.bash")
    print("3. Restart vision_servo node")
    print("4. Open browser and check for cyan detection boxes!")
else:
    print("\n⚠️ No modifications made - patches may already be applied")
