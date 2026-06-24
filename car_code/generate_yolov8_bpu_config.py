#!/usr/bin/env python3
"""
generate_yolov8_bpu_config.py - Generate BPU config for YOLOv8 model

This script generates a proper hb_mapper configuration file for YOLOv8
based on Horizon RDK official template (yolox5 style).
"""

import argparse
from pathlib import Path
import yaml


def generate_yolov8_config(onnx_path: str, calib_dir: str, output_dir: str = None):
    """
    Generate BPU conversion configuration for YOLOv8
    
    Args:
        onnx_path: Path to ONNX model file
        calib_dir: Directory containing calibration images
        output_dir: Output directory for config file
    """
    print("=" * 70)
    print("  Generating YOLOv8 BPU Configuration")
    print("=" * 70)
    print()
    
    # Validate inputs
    onnx_file = Path(onnx_path)
    if not onnx_file.exists():
        print(f"❌ Error: ONNX file not found: {onnx_path}")
        return False
    
    calib_path = Path(calib_dir)
    if not calib_path.exists():
        print(f"❌ Error: Calibration directory not found: {calib_dir}")
        return False
    
    image_count = len(list(calib_path.glob("*.jpg"))) + len(list(calib_path.glob("*.png")))
    print(f"✅ ONNX model: {onnx_file.name} ({onnx_file.stat().st_size / 1024 / 1024:.1f} MB)")
    print(f"✅ Calibration images: {image_count}")
    print()
    
    # Set output directory
    if output_dir is None:
        output_dir = onnx_file.parent
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Generate YOLOv8-specific BPU configuration
    # Based on Horizon RDK official yolox5 template
    config = {
        'model_parameters': {
            'march': 'bayes-e',              # BPU architecture version
            'layer_out_dump': False,          # Don't dump layer outputs
        },
        'input_parameters': {
            'input_name': 'images',           # Input tensor name
            'input_shape': '1x3x640x640',     # NCHW format as string
            'input_type_rt': 'nv12',          # Runtime input format (NV12 for RDK X5)
            'input_type_train': 'rgb',        # Training input format (RGB)
            'input_layout_train': 'NCHW',     # Training layout
            'norm_type': 'data_scale',        # Normalization type
            'scale_value': '0.003921568627451',  # 1/255 for pixel normalization
        },
        'calibration_parameters': {
            'cal_data_type': 'float32',       # Calibration data type
            'calibration_type': 'default',    # Default calibration method
            'calibration_data': str(calib_path.absolute()),
            'calibration_size': min(image_count, 100),
            'optimization': 'set_Softmax_input_int8,set_Softmax_output_int8',  # Softmax optimization
        },
        'compiler_parameters': {
            'compile_mode': 'latency',        # Optimize for low latency
            'debug': True,                    # Enable debug info
            'optimize_level': 'O3',           # Maximum optimization level
        },
    }
    
    config_file = output_path / 'config_yolov8.yaml'
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"✅ Configuration saved to: {config_file}")
    print()
    print("Configuration details:")
    print(f"  - Model: YOLOv8 (ONNX opset 11)")
    print(f"  - Architecture: bayes-e (RDK X5/X3)")
    print(f"  - Input: NV12 (runtime) / RGB (training)")
    print(f"  - Precision: INT8 quantization")
    print(f"  - Optimization: O3 (maximum)")
    print(f"  - Compile mode: latency (low latency)")
    print()
    print("Next steps:")
    print("  1. Check configuration:")
    print(f"     hb_mapper checker --model-type onnx --config {config_file}")
    print()
    print("  2. Compile to BIN:")
    print(f"     hb_mapper makertbin --config {config_file}")
    print()
    print("  3. Test performance:")
    print(f"     hrt_model_exec perf --model_file <generated_bin> --thread_num 1")
    print()
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Generate YOLOv8 BPU configuration')
    parser.add_argument('--onnx', type=str, required=True,
                       help='Path to ONNX model file')
    parser.add_argument('--calib-dir', type=str, default=None,
                       help='Directory containing calibration images')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for config file')
    
    args = parser.parse_args()
    
    # Default calibration directory
    if args.calib_dir is None:
        onnx_parent = Path(args.onnx).parent
        calib_dir = onnx_parent / 'calibration_images'
        if not calib_dir.exists():
            # Try common locations
            possible_dirs = [
                Path.home() / 'yahboomcar_ws/src/laser_calibration/models/calibration_images',
                Path('/home/sunrise/yahboomcar_ws/src/laser_calibration/models/calibration_images'),
            ]
            for d in possible_dirs:
                if d.exists():
                    calib_dir = d
                    break
    else:
        calib_dir = Path(args.calib_dir)
    
    if not calib_dir.exists():
        print(f"❌ Error: Calibration directory not found")
        print(f"   Please specify --calib-dir or create calibration_images directory")
        return
    
    # Generate configuration
    success = generate_yolov8_config(
        onnx_path=args.onnx,
        calib_dir=str(calib_dir),
        output_dir=args.output_dir
    )
    
    if success:
        print("=" * 70)
        print("  ✅ YOLOv8 BPU configuration generated successfully!")
        print("=" * 70)
    else:
        print("=" * 70)
        print("  ❌ Failed to generate configuration")
        print("=" * 70)


if __name__ == '__main__':
    main()
