#!/usr/bin/env python3
"""
export_yolo26_detect_bpu.py - Export YOLOv8/YOLO26 model for RDK X5 BPU

This script prepares the ONNX model for BPU conversion by:
1. Validating the ONNX model
2. Preparing calibration data configuration
3. Generating hb_mapper compatible config

According to Horizon RDK official documentation.
"""

import argparse
import os
from pathlib import Path
import yaml


def prepare_bpu_config(onnx_path: str, calib_dir: str, output_dir: str = None):
    """
    Prepare BPU conversion configuration
    
    Args:
        onnx_path: Path to ONNX model file
        calib_dir: Directory containing calibration images
        output_dir: Output directory for config files
    """
    print("=" * 70)
    print("  Preparing BPU Conversion Configuration")
    print("=" * 70)
    print()
    
    # Validate ONNX file
    onnx_file = Path(onnx_path)
    if not onnx_file.exists():
        print(f"❌ Error: ONNX file not found: {onnx_path}")
        return False
    
    print(f"✅ ONNX model found: {onnx_file.name} ({onnx_file.stat().st_size / 1024 / 1024:.1f} MB)")
    
    # Validate calibration directory
    calib_path = Path(calib_dir)
    if not calib_path.exists():
        print(f"❌ Error: Calibration directory not found: {calib_dir}")
        return False
    
    image_count = len(list(calib_path.glob("*.jpg"))) + len(list(calib_path.glob("*.png")))
    print(f"✅ Calibration images: {image_count}")
    
    if image_count < 100:
        print(f"⚠️  Warning: Recommended at least 100 calibration images (found {image_count})")
    
    # Set output directory
    if output_dir is None:
        output_dir = onnx_file.parent
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Generate hb_mapper configuration
    config = {
        'model_configuration': {
            'input_shape': [1, 3, 640, 640],  # NCHW format
            'input_type': 'NV12',  # RDK X5 uses NV12 input
            'output_type': 'FP32',
        },
        'compiler_options': {
            'working_mode': 'bayes',  # Use Bayes mode for better performance
            'optimize_level': 'O3',   # Maximum optimization
        },
        'quantization': {
            'calibration_data': str(calib_path.absolute()),
            'calibration_size': min(image_count, 100),
            'precision': 'int8',  # INT8 quantization for BPU
        },
        'input_parameters': [
            {
                'name': 'images',
                'type': 'image',
                'format': 'NV12',
            }
        ],
    }
    
    config_file = output_path / 'config.yaml'
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print()
    print(f"✅ Configuration saved to: {config_file}")
    print()
    print("Next steps:")
    print("  1. Check configuration:")
    print(f"     hb_mapper checker --model-type onnx --config {config_file}")
    print()
    print("  2. Compile to BIN:")
    print(f"     hb_mapper makertbin --config {config_file}")
    print()
    print("  3. Test performance:")
    print(f"     hrt_model_exec perf --model_file <generated_bin_file> --thread_num 1")
    print()
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Export YOLO26 for RDK X5 BPU')
    parser.add_argument('--weight', type=str, required=True,
                       help='Path to ONNX model file or .pt model file')
    parser.add_argument('--calib-dir', type=str, default=None,
                       help='Directory containing calibration images')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for config files')
    
    args = parser.parse_args()
    
    # If weight is a .pt file, first check if ONNX exists
    weight_path = Path(args.weight)
    if weight_path.suffix == '.pt':
        onnx_path = weight_path.with_suffix('.onnx')
        if not onnx_path.exists():
            print(f"❌ Error: ONNX file not found: {onnx_path}")
            print("   Please run export_monkey_patch.py first to generate ONNX model")
            return
        
        print(f"Using ONNX model: {onnx_path}")
    else:
        onnx_path = weight_path
    
    # Default calibration directory
    if args.calib_dir is None:
        # Look for calibration_images in same directory as model
        calib_dir = weight_path.parent / 'calibration_images'
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
        print(f"❌ Error: Calibration directory not specified and not found")
        print(f"   Please specify --calib-dir or create calibration_images directory")
        return
    
    # Prepare BPU configuration
    success = prepare_bpu_config(
        onnx_path=str(onnx_path),
        calib_dir=str(calib_dir),
        output_dir=args.output_dir
    )
    
    if success:
        print()
        print("=" * 70)
        print("  ✅ BPU configuration prepared successfully!")
        print("=" * 70)
    else:
        print()
        print("=" * 70)
        print("  ❌ Failed to prepare BPU configuration")
        print("=" * 70)


if __name__ == '__main__':
    main()
