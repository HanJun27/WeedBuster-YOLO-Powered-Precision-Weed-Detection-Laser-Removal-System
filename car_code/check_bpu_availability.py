#!/usr/bin/env python3
"""Check BPU/NPU availability on RDK X5"""

print("=" * 60)
print("  BPU/NPU Availability Check")
print("=" * 60)
print()

# 1. Check torch
try:
    import torch
    print(f"✅ PyTorch version: {torch.__version__}")
    print(f"   CUDA available: {torch.cuda.is_available()}")
    
    # Try to use NPU
    try:
        import torch_npu
        print(f"✅ torch-npu available: {torch_npu.__version__}")
        
        # Check NPU devices
        npu_count = torch.npu.device_count()
        print(f"   NPU device count: {npu_count}")
        
        if npu_count > 0:
            print(f"   NPU device name: {torch.npu.get_device_name(0)}")
            print("\n✅ NPU is available and ready to use!")
        else:
            print("\n⚠️  No NPU devices found")
    except ImportError:
        print("❌ torch-npu not installed")
        print("   This is expected on RDK X5 - using different backend")
except Exception as e:
    print(f"❌ Error checking torch: {e}")

print()

# 2. Check Ultralytics YOLO
try:
    from ultralytics import YOLO
    print("✅ Ultralytics YOLO imported successfully")
    
    # Check available backends
    import platform
    print(f"   Platform: {platform.machine()}")
    print(f"   System: {platform.system()}")
    
except Exception as e:
    print(f"❌ Error importing YOLO: {e}")

print()

# 3. Check Horizon BPU SDK
try:
    import hbdk
    print("✅ Horizon BPU SDK (hbdk) found")
except ImportError:
    print("❌ Horizon BPU SDK not found")
    print("   This may be why model.to('npu') fails")

print()

# 4. Check device files
import os
bpu_devices = [f for f in os.listdir('/dev') if f.startswith('hb')]
if bpu_devices:
    print(f"✅ BPU devices found: {bpu_devices}")
else:
    print("⚠️  No /dev/hb* devices found")
    print("   BPU driver may not be loaded")

print()
print("=" * 60)
print("  Recommendation")
print("=" * 60)
print()
print("On RDK X5, Ultralytics YOLO typically uses:")
print("  1. CPU inference (default)")
print("  2. HBNN (Horizon Neural Network) acceleration")
print()
print("The model.to('npu') syntax may not work directly.")
print("You may need to use Horizon's specific API instead.")
print()
