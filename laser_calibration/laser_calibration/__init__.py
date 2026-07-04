"""
laser_calibration —— 双目多光谱感知与闭环激光打击系统

阶段一（本包已实现）：开机静态标定
  - stereo_camera       ：双目驱动 + ISP 锁定 + 网络流
  - calib_camera_align  ：标定一（Shift_X / Shift_Y）
  - calib_laser_offset  ：标定二（Delta_X / Delta_Y）
  - show_calib          ：查看标定结果
  - ndvi_node           ：NDVI 植物健康检测（Phase 2，已接入）

阶段二/三（Phase 2/3）后续添加节点时统一从这里 import：
  from laser_calibration.calib_io   import load_calib
  from laser_calibration.robot_ctrl import set_servo, laser_blue, laser_ir
"""

__version__ = "3.11.1"
