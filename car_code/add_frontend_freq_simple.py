#!/usr/bin/env python3
"""Add simple frequency control to vision_servo.py"""

file_path = '/home/sunrise/yahboomcar_ws/src/laser_calibration/vision_servo.py'

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Check if already added
if 'freqSlider' in content or '检测频率' in content:
    print('✅ Frequency control already exists in frontend')
    exit(0)

# Find a good place to add the UI - after the canvas element
insert_marker = '<canvas id="cv"'
insert_pos = content.find(insert_marker)

if insert_pos == -1:
    print('❌ Could not find canvas element')
    exit(1)

# Find the end of the canvas line
line_end = content.find('\n', insert_pos)
if line_end == -1:
    line_end = insert_pos + 100

# Add frequency control UI after canvas
ui_html = '''
      <!-- YOLO Detection Frequency Control -->
      <div style="margin:15px 0; padding:12px; background:#f0f8ff; border-radius:8px; border:1px solid #b0d4f1;">
        <label style="font-weight:bold; color:#0066cc; display:block; margin-bottom:8px;">📡 YOLO Detection Frequency:</label>
        <div style="display:flex; align-items:center; gap:10px;">
          <input type="range" id="freqSlider" min="1" max="30" value="10" 
                 oninput="document.getElementById('freqValue').textContent=this.value+' Hz'" 
                 onchange="setPublishFreq(this.value)"
                 style="flex:1; height:6px;">
          <span id="freqValue" style="font-weight:bold; color:#0066cc; font-size:18px; min-width:60px;">10 Hz</span>
        </div>
        <div style="font-size:11px; color:#666; margin-top:6px;">
          Range: 1-30 Hz | Current: <span id="currentFreq">10</span> Hz
        </div>
      </div>
'''

content = content[:line_end] + '\n' + ui_html + content[line_end:]

# Add JavaScript function before closing script tag
js_func = '''
// YOLO publish frequency control
let currentPublishFreq = 10;

function setPublishFreq(freq) {
  freq = parseFloat(freq);
  if (freq < 1 || freq > 30) {
    alert('Frequency must be between 1-30 Hz');
    return;
  }
  
  fetch('/api/set_yolo_freq?freq=' + freq)
    .then(response => response.json())
    .then(data => {
      if (data.success) {
        currentPublishFreq = freq;
        document.getElementById('currentFreq').textContent = freq;
        console.log('✅ Frequency set to:', freq, 'Hz');
      } else {
        alert('Failed: ' + data.message);
        document.getElementById('freqSlider').value = currentPublishFreq;
        document.getElementById('freqValue').textContent = currentPublishFreq + ' Hz';
      }
    })
    .catch(err => {
      console.error('❌ Request failed:', err);
      alert('Network error');
    });
}

'''

# Find position to insert (before </script>)
script_end = content.rfind('</script>')
if script_end != -1:
    content = content[:script_end] + js_func + '\n' + content[script_end:]
else:
    print('⚠️ Could not find </script> tag')

# Write back
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print('✅ Frontend frequency control added successfully!')
