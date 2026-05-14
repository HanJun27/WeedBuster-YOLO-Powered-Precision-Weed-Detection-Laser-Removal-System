#!/usr/bin/env python3
"""Test YOLO model loading on the car"""

from ultralytics import YOLO

model_path = "/home/sunrise/yahboomcar_ws/src/laser_calibration/models/best.pt"
print(f"Loading model from: {model_path}")

try:
    model = YOLO(model_path)
    print("✅ Model loaded successfully!")
    
    info = model.info()
    print(f"Layers: {info[0]}")
    print(f"Parameters: {info[1]:,}")
    print(f"GFLOPs: {info[3]:.2f}")
    print(f"Classes: {model.names}")
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
