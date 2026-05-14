"""
config.py —— 全局硬件配置  v3.0
=================================
所有节点只读这一个文件，硬件有变动只改这里。

v3.0 改动（相对 v2）：
  1. 摄像头路径改用 udev by-id 稳定符号链接，USB 插拔顺序变化不影响识别
  2. 新增 ISP 锁定参数（NDVI 前置条件）：关自动曝光、关自动白平衡、锁固定值
  3. 工作空间路径：/home/sunrise/yahboomcar_ws/src/laser_calibration

硬件对照表
──────────────────────────────────────────────────
PWM接口  用途              SDK 调用
S1       云台水平舵机(偏航)  robot.set_pwm_servo(1, angle) angle=[0,180]
S2       云台俯仰舵机        robot.set_pwm_servo(2, angle)
S3       蓝紫高能激光(枪管)  robot.set_pwm_servo(3, 180=ON / 0=OFF)
S4       红外指示激光(瞄准镜) robot.set_pwm_servo(4, 180=ON / 0=OFF)
──────────────────────────────────────────────────
S3/S4 不是真的舵机，而是把 PWM 接口当开关用。
angle=180 → 高占空比 → 激光开
angle=0   → 低占空比 → 激光关
如激光模块高低电平定义相反，把 LASER_ON_ANGLE / LASER_OFF_ANGLE 对调即可。
"""

import os

# ══════════════════════════════════════════════════════════════
#  摄像头设备（udev by-id 稳定路径，USB 顺序变化不影响识别）
# ══════════════════════════════════════════════════════════════
# 如果 by-id 路径错了，执行以下命令查看实际名称：
#   ls -l /dev/v4l/by-id/
RGB_DEVICE = "/dev/v4l/by-id/usb-RGB_USB_RGB_Camera_20220508-video-index0"
IR_DEVICE  = "/dev/v4l/by-id/usb-IR_USB_IR_Camera_20220508-video-index0"

CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 30

# ══════════════════════════════════════════════════════════════
#  ISP 锁定（NDVI 前置条件，极其关键）
# ══════════════════════════════════════════════════════════════
# 物理要求：在室内无阳光直射、固定补光灯环境下使用
# 不锁死 ISP，NDVI 算出来就是"伪数值"——环境光一变就完全失效
LOCK_ISP = True     # 总开关，调试时可临时关闭

# ── RGB 相机（彩色，需要锁曝光 + 白平衡）──────────
# 曝光时间单位：100μs（V4L2 标准），250 → 25ms = 1/40 秒
# 太暗 → 调大；过曝 → 调小。常见调参范围 100-500
RGB_EXPOSURE = 250

# 白平衡色温 (K)
#   冷白日光灯：6500
#   日光：     5500
#   暖白灯：   3000-4000
RGB_WB_TEMP = 4600

# ── IR 相机（灰度，无白平衡概念，只锁曝光）────────
# 通常 IR 光强较弱，曝光时间比 RGB 稍大
IR_EXPOSURE = 300

# ══════════════════════════════════════════════════════════════
#  ROS2 Topic 名称
# ══════════════════════════════════════════════════════════════
TOPIC_RGB  = "/camera/rgb/image_raw"   # sensor_msgs/Image（YOLO 同学订阅）
TOPIC_IR   = "/camera/ir/image_raw"    # sensor_msgs/Image

# YOLO 同学发布的检测结果（Phase 3 视觉伺服会订阅）
TOPIC_YOLO = "/yolo/weed_detected"     # std_msgs/String (JSON)

# ══════════════════════════════════════════════════════════════
#  网络视频流
# ══════════════════════════════════════════════════════════════
STREAM_PORT_RGB = 8080    # http://<车IP>:8080
STREAM_PORT_IR  = 8081    # http://<车IP>:8081
STREAM_QUALITY  = 75      # JPEG 质量 [1-100]

# ══════════════════════════════════════════════════════════════
#  亚博小车串口（SunriseRobot SDK）
# ══════════════════════════════════════════════════════════════
ROBOT_SERIAL = "/dev/myserial"

# ── 云台 PWM 舵机（S1/S2）────────────────────────────────
SERVO_YAW_ID       = 1       # S1：水平偏航
SERVO_PITCH_ID     = 2       # S2：俯仰
SERVO_YAW_CENTER   = 90      # 正前方
SERVO_PITCH_CENTER = 90
SERVO_YAW_MIN      = 45      # 防撞限位
SERVO_YAW_MAX      = 135
SERVO_PITCH_MIN    = 60
SERVO_PITCH_MAX    = 120

# ── 激光 PWM 开关（S3/S4）────────────────────────────────
LASER_BLUE_ID   = 3      # S3：蓝紫高能激光（枪管，烧灼）
LASER_IR_ID     = 4      # S4：红外指示激光（瞄准镜，定位）
LASER_ON_ANGLE  = 180
LASER_OFF_ANGLE = 0

# ══════════════════════════════════════════════════════════════
#  标定文件路径
# ══════════════════════════════════════════════════════════════
CALIB_FILE = os.path.expanduser("~/calib_params.yaml")

# ══════════════════════════════════════════════════════════════
#  反射率标定板（国赛进阶版 真NDVI 用）
# ══════════════════════════════════════════════════════════════
# 4 点标定：白 / 灰50 / 灰18 / 黑（点越多最小二乘越准）
#   18% 灰是摄影曝光标准基准，比 50% 灰更专业
PANEL_WHITE  = 0.90
PANEL_GRAY50 = 0.50
PANEL_GRAY18 = 0.18    # 18% 灰，摄影标准
PANEL_BLACK  = 0.10

# NDVI 植物阈值：高于此值才认为是"真植物"
# 经验范围：真叶子真NDVI ≈ +0.3 ~ +0.7；非植物 ≈ -0.2 ~ +0.2
# 调小 → 更敏感（误把弱反差物体当植物）
# 调大 → 更严格（错过弱叶绿素的枯萎植物）
NDVI_PLANT_THRESHOLD = 0.25

# NDVI 网页流默认渲染模式
#   colormap   —— 全图红黄绿渐变（科研标准）
#   mask_solid —— 非植物完全灰度，仅植物染色（视频效果最炸）
#   mask_blend —— 非植物保留淡色，植物高亮（折中）
NDVI_DEFAULT_MODE = "mask_solid"


# ══════════════════════════════════════════════════════════════
#  Phase 3 视觉伺服 (vision_servo) 参数
# ══════════════════════════════════════════════════════════════
# v3.9 重大变更：
#   - vision_servo 从 IR 摄像头切到 RGB 摄像头
#     原因：S4 实际是可见红激光（650nm 左右），不是 IR。IR 摄像头有补光灯
#     淹没问题；RGB 摄像头能直接看到红光斑，不需要 Shift 转换。
#   - Shift_X/Y 从 vision_servo 流程中退役（仍可用于 NDVI 像素对齐）
#   - Delta_X/Y 重新定义为 RGB 画面下的偏移（旧值需重新标定）
#   - PIXEL_TO_PITCH_DEG 待开机实测验证（pitch 装向未确认）

# 开环粗对准的 像素→舵机角度 比例
# 估算逻辑：相机水平视场角 ~60°，画面宽 640，→ 1 像素 ≈ 60/640 ≈ 0.094°
# 实际值由舵机机械构造和镜头参数决定，开环模式下逐步调节这个值
PIXEL_TO_YAW_DEG   = -0.10       # 1 像素 → 多少度 yaw（v3.8 经验：本车 yaw 装反，用负值）
PIXEL_TO_PITCH_DEG = 0.10        # 1 像素 → 多少度 pitch（v3.9 标定后验证：方向正负请实测）

# ══════════════════════════════════════════════════════════════
# PID 闭环参数（v3.9.9: 全面抗振荡调整）
# ══════════════════════════════════════════════════════════════
# 振荡根因分析：
#   旧值 KP=0.05, OUTPUT_LIMIT=5° → 100ms 内云台可转 5°，画面像素移动 50px
#   恰好等于 SPOT_JUMP_MAX_PX 阈值 → 触发帧间抑制 → 光斑读数滞后 → PID 失稳
# 修复哲学："让 PID 单步 < 帧间稳定性能容忍的范围"
PID_KP = 0.03                    # v3.9.9: 0.05→0.03 比例降低 40%
PID_KI = 0.001                   # 积分（不变）
PID_KD = 0.025                   # v3.9.9: 0.02→0.025 微分略增加阻尼

PID_TOLERANCE_PX  = 3            # 收敛判据：误差欧氏距离 < 此像素数算"到位"
PID_LOCK_FRAMES   = 5            # 连续 N 帧到位 = 稳定锁定
PID_TIMEOUT_SEC   = 5.0          # v3.9.9: 3.0→5.0 给慢移留更多时间
PID_OUTPUT_LIMIT  = 1.5          # v3.9.9: 5.0→1.5 单步最多 1.5°（≈15 像素移动）

# v3.9.9 新增：抗振荡机制
# 死区：误差<此值时 P 项减半（防收敛区震荡）
PID_DEADBAND_PX   = 6
# 输出饱和检测：连续 N 帧输出打满 → 系统跟不上 → 跳一帧 + 衰减积分
PID_SATURATION_FRAMES = 3
# 主循环周期（秒）。10Hz=0.1。振荡严重时可调 0.15 (6.7Hz)
FSM_TICK_PERIOD_SEC = 0.1

# 自动 / 手动触发模式
SERVO_DEFAULT_MODE = "manual"    # "manual" 或 "auto"
SERVO_AUTO_DEBOUNCE = 1.5

# YOLO 目标新鲜度
YOLO_TARGET_FRESH_SEC = 0.5      # 0.5s 内的 YOLO 数据视为"新鲜"
YOLO_FALLBACK_TO_LOCKED = True

# 烧灼参数
FIRE_DURATION_SEC = 1.0
FIRE_COOLDOWN_SEC = 1.0


# ══════════════════════════════════════════════════════════════
#  光斑检测：v3.9.5 全新算法 — R-max(G,B) 红色优势 + ROI 跟踪
# ══════════════════════════════════════════════════════════════
# 战友实测发现：HSV 在"过曝白纸 + 不可调曝光"场景下不稳定。
# v3.9.5 改用更稳健的"红色相对优势"算法：
#   red_score = R - max(G, B)
# - 白纸 R≈G≈B → 分数 0，自动忽略
# - 真激光 R 远大于 G/B → 分数高，被检测到
# - 过曝中心（白色）也是 R≈G≈B，但被周围的"红环"通过形态学闭运算填回
#
# 这套算法不依赖任何亮度阈值，适应各种环境光。

# R-max(G,B) 阈值：超过此分数视为红光斑像素
# 30 比较保守，30~50 都可调
RED_DOMINANCE_MIN = 30

# ROI 模式：以"上一帧位置 / SPOT_HOME"为中心做局部检测
# 优点：1. 物理隔绝远处的红色干扰物（地砖、织物等）
#      2. 加速（小图 200x200 vs 640x480）
# ROI 边长——光斑在云台旋转时画面位移很小，200 像素足够
SPOT_ROI_SIZE = 200

# 形态学闭运算核大小——填补过曝白色"甜甜圈"中心
# 11x11 能填补半径约 5 像素的中心，常见过曝光斑足够
SPOT_CLOSE_KERNEL_SIZE = 11

# 最小连通域面积（过滤孤立噪点）
RED_SPOT_AREA_MIN  = 8
RED_SPOT_AREA_MAX  = 5000

# 历史保留（与之前兼容，仍引用）
RED_HSV_LOWER1 = (0,   80,  150)    # DEPRECATED in v3.9.5
RED_HSV_UPPER1 = (10,  255, 255)
RED_HSV_LOWER2 = (170, 80,  150)
RED_HSV_UPPER2 = (179, 255, 255)
RED_CORE_V_MIN = 230                # DEPRECATED in v3.9.5

# 启动自检：归中状态下红光斑应该出现在画面里这个位置附近
# 实测后请把这两个值改成你那台车归中时光斑实际位置（从 vision_servo 启动自检日志读）
SPOT_HOME_X = 320                   # 实测后修改
SPOT_HOME_Y = 240
SPOT_HOME_TOLERANCE = 80            # 与 SPOT_HOME 距离超过此值则视为异常

# v3.9.4 新增：帧间稳定性
SPOT_JUMP_MAX_PX = 100              # v3.9.9: 50→100 容忍 PID 单步带来的正常视场移动
SPOT_JUMP_TOLERATE_FRAMES = 2       # v3.9.9: 3→2 减少抑制延迟

# ══════════════════════════════════════════════════════════════
#  烧痕检测：RGB 白纸上的焦黑色斑（v3.9 标定二用）
# ══════════════════════════════════════════════════════════════
# 蓝紫激光烧白纸 → 焦黑色（V<60，S较低，颜色接近无色）
BURN_THRESHOLD = 60                 # 灰度 < 此值视为烧痕
BURN_AREA_MIN  = 8
BURN_AREA_MAX  = 5000

# v3.8 的 IR 光斑常量保留为别名，避免老节点 import 错误
# 标定一节点（calib_camera_align）可能仍引用
IR_SPOT_THRESHOLD = 200             # DEPRECATED: 仅 calib_camera_align 引用
IR_SPOT_AREA_MIN  = 2               # DEPRECATED
IR_SPOT_AREA_MAX  = 2000            # DEPRECATED


# ════════════════════════════════════════════════════════════════
#  Active diffuse 标定（标定四）+ NDVI 健康分级（v3.10 新增）
#  用于 ndvi_node.py 和 calib_diffuse.py 节点
# ════════════════════════════════════════════════════════════════
# 主动光场标定的物理原理：
#   1) ISP 锁死 → DN 值稳定
#   2) 主动光源开启（850nm IR LED + 白光 LED）→ 受控光场
#   3) 灰卡 ROI 实时测 K = R_gray / NIR_gray
#   4) 任意像素 NDVI = (K·NIR' - R') / (K·NIR' + R')

# 默认启用 active 模式（calib4 完成后才生效）
# False 时即使 calib4_done 也走伪 NDVI（debug 用）
ACTIVE_MODE_DEFAULT = True

# 参考物已知反射率
# 0.18 = 摄影 18% 灰卡（淘宝 ¥30~80）
# 0.50 = 50% 灰卡
# 0.90 = 白色 PVC 板 / 白纸（粗略）
# 0.99 = PTFE 漫反射板（NDVI 首选）
GRAY_CARD_REFLECTANCE = 0.18

# 灰卡 ROI 默认位置（演示时网页可拖框调整）
# 原则：装在画面边角，演示时遮黑就看不见
GRAY_ROI_X = 20
GRAY_ROI_Y = 20
GRAY_ROI_W = 80
GRAY_ROI_H = 80

# 标定阶段采样帧数（多帧均值降噪）
DARK_FRAME_COUNT = 30
GRAY_FRAME_COUNT = 30

# 灰卡像素 DN 下限（防止灰卡被误标到全黑区域）
GRAY_MIN_DN = 20

# 演示输出时自动遮挡灰卡 ROI（黑色矩形覆盖）
# 系统内部仍用它做 K 修正，但观众看不到
GRAY_ROI_MASK_ON_OUTPUT = True

# 健康分级阈值（active 模式真 NDVI）
NDVI_HEALTHY_MIN  = 0.45   # > 此值 → 健康（深绿）
NDVI_MODERATE_MIN = 0.25   # > 此值 → 亚健康（黄绿）
NDVI_PLANT_MIN    = 0.10   # > 此值 → 植物（含枯萎）；< 此值 → 非植物背景
# 分级:
#   ndvi >= 0.45        → healthy
#   0.25 <= ndvi < 0.45 → moderate
#   0.10 <= ndvi < 0.25 → stressed/dead
#   ndvi < 0.10         → non-plant
