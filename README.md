# WeedBuster - YOLO-Powered Precision Weed Detection & Laser Removal System

基于YOLOv8深度学习模型的低成本精准农业解决方案，集成双目视觉、BPU加速推理、IBVS闭环控制和NDVI植物健康检测功能。

## 🎯 核心功能

- **实时杂草检测**: YOLOv8n模型在RDK X5 BPU上实现76 FPS高速推理（27ms/帧）
- **精准激光打击**: IBVS视觉伺服PID控制，3像素精度定位，蓝紫激光物理除草
- **多光谱健康监测**: 850nm IR摄像头 + RGB双模采集，支持active/reflection/pseudo三种NDVI计算模式
- **边缘端部署优化**: 模型量化压缩至3.2MB，支持地瓜机器人RDK X5平台高效运行

## 🚀 技术亮点

- ✅ BPU硬件加速：相比CPU推理提速67倍（1800ms → 27ms）
- ✅ 解耦架构设计：图像采集、目标检测、伺服控制独立运行，互不阻塞
- ✅ 抗振荡PID算法：动态调整策略，避免云台抖动
- ✅ 完整标定流程：双目对齐、激光偏移、灰卡反射率、暗电流补偿四步标定
- ✅ 云端训练支持：提供数据集划分、断点续训、可视化评估全套工具链

## 📦 项目结构

```
WeedBuster/
├── code/                      # ROS2激光校准包
│   └── 总交接包_v3.10.0/
│       ├── laser_calibration/ # ROS2 package (9个功能节点)
│       │   ├── laser_calibration/  # Python模块
│       │   ├── models/             # 模型文件
│       │   ├── package.xml
│       │   └── setup.py
│       ├── servo_direction_test.py
│       └── README_v3.10.0_交接.md
├── 模型/                     # 训练完成的YOLOv8n权重
│   ├── Best1/
│   │   └── best.pt (~6MB)
│   └── Best2/
│       ├── best.pt (~6MB)
│       └── training.log
└── 量化后/                   # BPU量化模型及评估报告
    └── X系列量化任务-61657045_all_results/
        ├── quant.bin (~5.8MB)
        ├── quant_model_quantized_model.onnx
        └── quantization_result.json
```

## 🛠️ 快速开始

### 环境要求

- Python 3.8-3.11
- PyTorch >= 1.8.0
- ROS2 Humble (用于激光小车部署)
- CUDA (可选，用于GPU加速训练)

### 安装依赖

```bash
pip install ultralytics opencv-python numpy pyyaml
```

### 模型推理

```python
from ultralytics import YOLO

# 加载模型
model = YOLO("模型/Best2/best.pt")

# 预测
results = model("test_image.jpg")
results[0].show()
```

### ROS2部署（激光小车）

```bash
# 在小车上解压代码包
cd ~/yahboomcar_ws/src
unzip laser_calibration_v3_10_0.zip

# 编译
cd ~/yahboomcar_ws
colcon build --packages-select laser_calibration
source install/setup.bash

# 启动节点
ros2 run laser_calibration stereo_camera
ros2 run laser_calibration yolo_detector
ros2 run laser_calibration vision_servo
```

详细部署指南请参考：[code/总交接包_v3.10.0/README_v3.10.0_交接.md](code/总交接包_v3.10.0/README_v3.10.0_交接.md)

## 📊 性能指标

| 指标 | 数值 |
|------|------|
| mAP@0.5 | ≥ 0.85 |
| BPU推理速度 | 27ms/帧 (76 FPS) |
| CPU推理速度 | 1800ms/帧 |
| PID收敛精度 | ±3像素 |
| 模型大小（原始） | 6 MB |
| 模型大小（量化后） | 3.2 MB |
| 硬件成本 | ~¥300 |

## 🔧 主要功能节点

### laser_calibration ROS2 Package

1. **stereo_camera** - 双目相机驱动（RGB + IR）
2. **yolo_detector** - YOLO目标检测（支持BPU/CPU）
3. **vision_servo** - IBVS视觉伺服PID控制
4. **calib_camera_align** - 标定一：双目摄像头对齐
5. **calib_laser_offset** - 标定二：激光偏移量校准
6. **calib_reflectance** - 标定三：灰卡反射率采样
7. **calib_diffuse** - 标定四：暗电流+主动光场标定
8. **ndvi_node** - NDVI植物健康检测
9. **show_calib** - 标定参数查看工具

## 📝 许可证

MIT License

## 👥 开发团队

精准农业AI项目组

## 📧 联系方式

如有问题或合作意向，欢迎联系！

---

**硬件平台**: 亚博RDK X5 / Ubuntu 22.04 ARM64 / ROS2 Humble  
**核心技术**: YOLOv8 · BPU加速 · IBVS控制 · NDVI监测 · ROS2
