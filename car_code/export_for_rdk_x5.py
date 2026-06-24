#!/usr/bin/env python3
"""
export_for_rdk_x5.py - Export YOLO model to ONNX for RDK X5 BPU

This script exports Ultralytics YOLO model to ONNX format compatible with RDK X5.
"""

import argparse
from pathlib import Path
from ultralytics import YOLO


def export_for_rdk(pt_model: str, opset: int = 11, imgsz: int = 640):
    """
    Export YOLO model to ONNX format compatible with RDK X5
    
    Args:
        pt_model: Path to .pt model file
        opset: ONNX opset version (11 for X5/X3, 19 for S100)
        imgsz: Input image size
    """
    print("=" * 70)
    print("  Exporting YOLO model to ONNX for RDK X5")
    print("=" * 70)
    print()
    
    # Load model
    print(f"Loading model: {pt_model}")
    model = YOLO(pt_model)
    
    # Export to ONNX with specific settings for RDK X5
    print(f"Exporting to ONNX (imgsz={imgsz}, opset={opset})...")
    
    onnx_path = model.export(
        format='onnx',
        imgsz=imgsz,
        simplify=True,          # Simplify ONNX graph
        opset=opset,            # ONNX opset version (11 for X5/X3)
        dynamic=False,          # Fixed input size for BPU
        half=False,             # Keep FP32 for calibration
    )
    
    print(f"\n✅ ONNX model exported: {onnx_path}")
    print()
    
    # Verify ONNX model
    try:
        import onnx
        onnx_model = onnx.load(onnx_path)
        onnx.checker.check_model(onnx_model)
        print("✅ ONNX model verification passed")
        
        # Print model info
        print(f"\nModel Information:")
        print(f"  Inputs: {len(onnx_model.graph.input)}")
        for inp in onnx_model.graph.input:
            shape = [d.dim_value for d in inp.type.tensor_type.shape.dim]
            print(f"    - {inp.name}: {shape}")
        
        print(f"\n  Outputs: {len(onnx_model.graph.output)}")
        for out in onnx_model.graph.output:
            shape = [d.dim_value for d in out.type.tensor_type.shape.dim]
            print(f"    - {out.name}: {shape}")
            
    except Exception as e:
        print(f"⚠️  ONNX verification warning: {e}")
    
    print()
    print("=" * 70)
    print("  Next steps:")
    print("  1. Use hb_mapper to convert ONNX to BIN")
    print("  2. Prepare calibration images")
    print("  3. Generate config.yaml")
    print("=" * 70)
    
    return onnx_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Export YOLO for RDK X5')
    parser.add_argument('--pt', type=str, required=True, help='Path to .pt model')
    parser.add_argument('--opset', type=int, default=11, 
                       help='ONNX opset version (11 for X5/X3, 19 for S100)')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    
    args = parser.parse_args()
    
    export_for_rdk(args.pt, args.opset, args.imgsz)
