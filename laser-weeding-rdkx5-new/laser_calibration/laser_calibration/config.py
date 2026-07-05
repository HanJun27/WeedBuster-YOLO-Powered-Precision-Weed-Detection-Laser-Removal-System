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
# 曝光时间单位：100μs（V4L2 标准）。250→25ms 长曝光会被阳光灌爆 → 过曝全白。
# v3.10.7 实测：压到 8（=0.8ms）红激光点才不被烧白、能稳定检出(maxRed≈119)。
# ⚠️ 这是【除草线/激光点检测】用值。两个反向需求要注意：
#   · YOLO 杂草检测 / NDVI 需要更亮 → 8 在室内暗光下可能太暗看不清杂草；
#     田间露天阳光强时 8 合适。跑整链路时确认 YOLO 还能认出杂草，否则折中。
#   · NDVI 另需固定中等曝光，跑 NDVI 时用回它自己的值。
RGB_EXPOSURE = 8

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

# v3.11.1: ExG 假草过滤运行时开关（vision_servo 网页按钮发布 → yolo_detector 订阅）。
#   不重启即可现场 A/B 对比"开/关 ExG"，并验证新模型是否真不空锁。
TOPIC_EXG_ENABLE = "/yolo/exg_enable"  # std_msgs/Bool

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
# ⚠️ v3.10.2+ 重要提示：下面 PID_KP / PID_KI / PID_KD / PID_OUTPUT_LIMIT 这几个
#    对 vision_servo 已是【死参数】——vision_servo 用"量化感知整数步进"控制器，
#    在文件顶部用 PID_KP_DEFAULT=0.8 等覆盖，不再 import 这里的 KP/KI/KD/LIMIT。
#    改这里【不会】影响伺服；要调伺服 PID 请改 vision_servo.py 顶部的 *_DEFAULT，
#    或用网页滑块（会存盘到 ~/.laser_calibration/pid_tuning.json）。
#    这里保留仅为历史/其他可能的消费者；PID_LOCK_FRAMES / PID_TIMEOUT_SEC /
#    PID_DEADBAND_PX / PID_TOLERANCE_PX 仍被 vision_servo 使用。
# 振荡根因分析（历史，针对旧连续 PID）：
#   旧值 KP=0.05, OUTPUT_LIMIT=5° → 100ms 内云台可转 5°，画面像素移动 50px
#   恰好等于 SPOT_JUMP_MAX_PX 阈值 → 触发帧间抑制 → 光斑读数滞后 → PID 失稳
PID_KP = 0.03                    # 【死参数】vision_servo 用 PID_KP_DEFAULT 覆盖
PID_KI = 0.001                   # 【死参数】
PID_KD = 0.025                   # 【死参数】

PID_TOLERANCE_PX  = 3            # 收敛判据：误差欧氏距离 < 此像素数算"到位"
PID_LOCK_FRAMES   = 5            # 连续 N 帧到位 = 稳定锁定
PID_TIMEOUT_SEC   = 5.0          # v3.9.9: 3.0→5.0 给慢移留更多时间
PID_OUTPUT_LIMIT  = 1.5          # 【死参数】vision_servo 用 PID_OUTPUT_LIMIT_PX/MAX_DEG_PER_STEP

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

# v3.10.8: 亮度门 —— 激光是【强亮点】，而暗红背景（土壤/纸箱）虽红但不亮。
#   开启后：像素必须同时满足"红主导≥RED_DOMINANCE_MIN" 且 "亮度≥SPOT_BRIGHT_MIN"
#   才算光斑候选 → 把激光从大片暗红背景里剥离，解决"补光后 OpenCV 乱锁背景红"。
#   亮度 = max(R,G,B)。SPOT_BRIGHT_MIN 经验 90~150：小→更易检出但更易混背景；
#   大→更挑亮点。配合曝光滑块把白纸压到浅灰(RGB~180-210)、激光点仍很亮时最稳。
#   关掉(False) = 退回 v3.10.7 纯 R-max(G,B) 行为。
SPOT_REQUIRE_BRIGHT = True
SPOT_BRIGHT_MIN     = 90

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

# v3.10.8: 紧凑度(圆度)过滤 —— 激光点近似圆，室内灯管/边缘反光是长条。
#   紧凑度 = 轮廓面积 / 最小外接圆面积，圆≈1、长条≪1。
#   只保留 紧凑度 ≥ SPOT_MIN_COMPACTNESS 的候选 → 剔除长条形反光。
#   0.55 是经验门槛(Gemini 建议)；设 0 可关闭此过滤。
SPOT_MIN_COMPACTNESS = 0.55

# v3.10.7: 当给了 ROI 中心 hint 时，从"通过面积过滤的多个红色轮廓"里选谁。
#   True  = 选离 hint 最近的（hint 是上一帧光斑 / 按云台角预测的位置）。
#           对抗"画面里混进比光斑更大的红色物体"——大红物体若不在预期光斑位置
#           附近就不会被误选。配合 ROI（已物理隔绝远处红色），鲁棒性更好。
#   False = 旧行为：选面积最大的轮廓。
# 注：无 hint（全画面兜底）时不受此开关影响，仍取面积最大。
RED_SPOT_PREFER_NEAREST_HINT = True

# 历史保留（与之前兼容，仍引用）
RED_HSV_LOWER1 = (0,   80,  150)    # DEPRECATED in v3.9.5
RED_HSV_UPPER1 = (10,  255, 255)
RED_HSV_LOWER2 = (170, 80,  150)
RED_HSV_UPPER2 = (179, 255, 255)
RED_CORE_V_MIN = 230                # DEPRECATED in v3.9.5

# 启动自检：归中状态下红光斑应该出现在画面里这个位置附近
# 实测后请把这两个值改成你那台车归中时光斑实际位置（从 vision_servo 启动自检日志读）
SPOT_HOME_X = 275                   # v3.10.7 实测(曝光8,云台居中)静止光斑
SPOT_HOME_Y = 161                   # v3.10.7 实测
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

# 传感器伽马(de-gamma)—— 经验线法的前置线性化(NDVI v3.10.1 新增)
# 消费级 USB 相机 DN 经过 gamma 编码(非线性),经验线法要求线性空间,
# 故标定和计算都先做:DN_linear = (DN/255)^gamma × 255
#   2.2  = sRGB 标准值(大多数 USB 相机适用,推荐默认)
#   1.0  = 不做校正(相机已输出线性 raw,或调试时)
#   精确值可用经验法实测(NDVIpi 论文实测约 2.13)
SENSOR_GAMMA = 2.2

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


# ════════════════════════════════════════════════════════════════
#  ★ NDVI v3.10.1 新增:时序健康监测 (ndvi_monitor) 配置
# ════════════════════════════════════════════════════════════════
# 时序监测历史文件(每次快照追加一条记录)
import os as _os
NDVI_HISTORY_FILE = _os.path.expanduser("~/ndvi_history.json")

# 趋势判定阈值(基于"相对基线百分比"每天的变化斜率)
# 第一条快照设为基线 100%,后续按 global_ndvi / baseline 归一化
# 斜率单位:百分点 / 天
NDVI_TREND_DECLINE = -3.0   # 斜率 < -3%/天 → 判定"健康下降"
NDVI_TREND_IMPROVE = 3.0    # 斜率 > +3%/天 → 判定"健康改善"
# 介于两者之间 → "稳定"

# 自动快照模式:节点每隔 N 小时自动记录一次(0 = 关闭,仅手动)
NDVI_AUTO_SNAPSHOT_HOURS = 0

# 触发"亚健康预警"的连续下降天数
NDVI_ALERT_DECLINE_DAYS = 3


# ════════════════════════════════════════════════════════════════
#  ★ v3.10.13 新增:ExG 超绿指数假草过滤 (yolo_detector)
# ════════════════════════════════════════════════════════════════
# 背景:当前 YOLO 模型训练时缺少"纯背景"负样本,空场景下会对地面花纹/
#   龟裂/阴影"空锁"误报。重训前在检测节点加一道物理光谱闸门:对每个
#   YOLO 框看其中心区"活体绿"像素占比,过低则判为假草丢弃;一帧内所有框
#   都被丢则该帧 detected=False,下游(planner/chassis)从源头收不到假目标。
# 注意:这是【过渡防御】。模型重训补齐负样本后,可调高 EXG_MIN_RATIO 让其
#   仅作冗余复核,或置 EXG_FILTER_ENABLE=False 关闭。绿色作物田慎用(会误伤)。
EXG_FILTER_ENABLE = True    # 总开关。绿叶作物场景或模型已修好可关
EXG_MIN_RATIO     = 0.15    # 框中心区活体绿像素占比下限(实测起点,见下方调参注)
EXG_CENTER_FRAC   = 0.60    # 只看框中心这一比例的区域,避开边缘土壤干扰
EXG_THRESH        = 0.10    # 归一化超绿指数 ExG=2g-r-b(g,r,b 为各通道占比)门限
EXG_G_DOMINANCE   = 1.15    # 硬判据:G > R×此值 才算绿(压住中性灰褐土壤的微正 ExG)
EXG_OVEREXP       = 245     # R,G,B 同时 > 此值 → 过曝白反光,剔除
EXG_SHADOW_SUM    = 40      # R+G+B < 此值 → 死黑阴影,剔除
# ── 现场调参 ──
#   误拦真草(把真草当假草丢) → 调低 EXG_MIN_RATIO(如 0.10)或 EXG_G_DOMINANCE(如 1.08)
#   漏拦假草(地面噪声仍进队)  → 调高 EXG_MIN_RATIO(如 0.20~0.25)
#   强阳光过曝严重            → 适当调低 EXG_OVEREXP(如 235)
#   日志看 [ExG] 行的 ratio 值,据此定门限最直接
