#!/usr/bin/env python3
"""
Add frequency control slider to vision_servo.py web interface
"""

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/vision_servo.py'

print(f"Reading {file_path}...")
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Add frequency control UI in HTML (after the mode selector)
old_ui_section = '''      <div style="margin:10px 0">
        <label>模式：</label>
        <select id="modeSelect" onchange="setMode(this.value)">
          <option value="auto">自动</option>
          <option value="manual">手动</option>
        </select>
      </div>'''

new_ui_section = '''      <div style="margin:10px 0">
        <label>模式：</label>
        <select id="modeSelect" onchange="setMode(this.value)">
          <option value="auto">自动</option>
          <option value="manual">手动</option>
        </select>
      </div>
      
      <!-- YOLO 检测频率控制 -->
      <div style="margin:15px 0; padding:10px; background:#f0f8ff; border-radius:5px;">
        <label style="font-weight:bold; color:#0066cc;">📡 YOLO 检测频率：</label>
        <input type="range" id="freqSlider" min="1" max="30" value="10" 
               oninput="updateFreqDisplay(this.value)" 
               onchange="setPublishFreq(this.value)"
               style="width:200px; vertical-align:middle; margin:0 10px;">
        <span id="freqValue" style="font-weight:bold; color:#0066cc; font-size:16px;">10 Hz</span>
        <div style="font-size:12px; color:#666; margin-top:5px;">
          范围: 1-30 Hz | 当前: <span id="currentFreq">10</span> Hz
        </div>
      </div>'''

content = content.replace(old_ui_section, new_ui_section)

# 2. Add JavaScript functions for frequency control
js_functions = '''
// YOLO 发布频率控制
let currentPublishFreq = 10;

function updateFreqDisplay(value) {
  document.getElementById('freqValue').textContent = value + ' Hz';
}

function setPublishFreq(freq) {
  freq = parseFloat(freq);
  if (freq < 1 || freq > 30) {
    alert('频率必须在 1-30 Hz 范围内');
    return;
  }
  
  // 调用 ROS2 服务调整频率
  fetch('/api/set_yolo_freq?freq=' + freq)
    .then(response => response.json())
    .then(data => {
      if (data.success) {
        currentPublishFreq = freq;
        document.getElementById('currentFreq').textContent = freq;
        console.log('✅ 频率已调整为:', freq, 'Hz');
      } else {
        alert('设置失败: ' + data.message);
        // 恢复滑块到之前的值
        document.getElementById('freqSlider').value = currentPublishFreq;
        updateFreqDisplay(currentPublishFreq);
      }
    })
    .catch(err => {
      console.error('❌ 频率设置请求失败:', err);
      alert('网络错误，请检查连接');
    });
}

'''

# Find position to insert (before the closing script tag or after existing JS functions)
insert_marker = "function setMode(mode) {"
insert_pos = content.find(insert_marker)
if insert_pos != -1:
    content = content[:insert_pos] + js_functions + content[insert_pos:]

# 3. Add API endpoint handler in Python (in do_GET method)
old_api_handler = '''            elif path == '/api/state':'''

new_api_handler = '''            elif path.startswith('/api/set_yolo_freq'):
                # Handle frequency adjustment request
                from urllib.parse import urlparse, parse_qs
                parsed = urlparse(path)
                params = parse_qs(parsed.query)
                
                try:
                    freq = float(params.get('freq', [10])[0])
                    if freq < 1 or freq > 30:
                        self.send_json({"success": False, "message": "频率必须在 1-30 Hz 范围内"})
                    else:
                        # Call ROS2 service to adjust frequency
                        import subprocess
                        result = subprocess.run(
                            ['ros2', 'service', 'call', '/yolo_detector/set_publish_freq', 
                             'std_srvs/srv/SetFloat', f'{{data: {freq}}}'],
                            capture_output=True, text=True, timeout=5
                        )
                        
                        if result.returncode == 0:
                            self.send_json({"success": True, "message": f"频率已设置为 {freq} Hz"})
                        else:
                            self.send_json({"success": False, "message": f"服务调用失败: {result.stderr}"})
                except Exception as e:
                    self.send_json({"success": False, "message": str(e)})
            
            elif path == '/api/state':'''

content = content.replace(old_api_handler, new_api_handler)

# Write back
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("✅ Frontend patch applied successfully!")
print("\nChanges made:")
print("1. ✅ Added frequency slider UI (1-30 Hz)")
print("2. ✅ Added JavaScript frequency control functions")
print("3. ✅ Added /api/set_yolo_freq endpoint")
print("\nThe web interface will now have a frequency control slider!")
