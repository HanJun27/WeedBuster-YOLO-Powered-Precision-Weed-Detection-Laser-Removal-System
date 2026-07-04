"""
robot_ctrl.py —— 亚博 SunriseRobot SDK 统一封装  v3.0
======================================================
把 SDK 初始化、云台舵机、激光开关都集中在这里。
其他模块只需：
    from laser_calibration.robot_ctrl import (
        robot, set_servo, center_servo,
        laser_blue, laser_ir, all_lasers_off, fire_blue_pulse,
    )

PWM 直驱说明：
    S1/S2 是常规舵机，set_pwm_servo(id, angle) angle=[0,180]
    S3/S4 被当作开关使用：
        angle=180 → PWM 高占空比 → 激光开
        angle=0   → PWM 低占空比 → 激光关
    如你的激光驱动模块高低电平定义相反，修改 config.py 里的
    LASER_ON_ANGLE / LASER_OFF_ANGLE 对调即可。

SDK 未加载时（如离线开发）自动降级，所有控制函数打印日志但不会报错。
"""

import time

from laser_calibration.config import (
    LASER_BLUE_ID, LASER_IR_ID,
    LASER_OFF_ANGLE, LASER_ON_ANGLE,
    ROBOT_SERIAL,
    SERVO_PITCH_CENTER, SERVO_PITCH_ID,
    SERVO_PITCH_MAX, SERVO_PITCH_MIN,
    SERVO_YAW_CENTER, SERVO_YAW_ID,
    SERVO_YAW_MAX, SERVO_YAW_MIN,
)

# ── SDK 初始化（软失败，SDK 不在时降级运行）──────────────────
try:
    from SunriseRobotLib import SunriseRobot as _SDK
    robot = _SDK(com=ROBOT_SERIAL)
    robot.create_receive_threading()
    ROBOT_OK = True
    print(f"[robot_ctrl] SunriseRobot SDK 已连接 ({ROBOT_SERIAL})")
except Exception as _e:
    robot = None
    ROBOT_OK = False
    print(f"[robot_ctrl] SDK 加载失败（{_e}），硬件控制将被跳过")


# ══════════════════════════════════════════════════════════════
#  云台 PWM 舵机（S1/S2）
# ══════════════════════════════════════════════════════════════
def set_servo(yaw: int, pitch: int):
    """
    控制云台 PWM 舵机。
    yaw / pitch 范围 [0, 180]，90 为中立正前方。
    超出 config.py 限位范围的值会被自动裁剪。
    """
    yaw   = max(SERVO_YAW_MIN,   min(SERVO_YAW_MAX,   int(yaw)))
    pitch = max(SERVO_PITCH_MIN, min(SERVO_PITCH_MAX, int(pitch)))
    if ROBOT_OK:
        robot.set_pwm_servo(SERVO_YAW_ID,   yaw)
        robot.set_pwm_servo(SERVO_PITCH_ID, pitch)
    else:
        print(f"[robot_ctrl] 跳过 set_servo: yaw={yaw}, pitch={pitch}")


def center_servo():
    """云台归中（正前方）"""
    set_servo(SERVO_YAW_CENTER, SERVO_PITCH_CENTER)


# ══════════════════════════════════════════════════════════════
#  激光 PWM 开关（S3/S4）
# ══════════════════════════════════════════════════════════════
def laser_blue(on: bool):
    """
    控制 S3 蓝紫高能激光（枪管）。
    on=True → 打开，on=False → 关闭
    """
    angle = LASER_ON_ANGLE if on else LASER_OFF_ANGLE
    if ROBOT_OK:
        robot.set_pwm_servo(LASER_BLUE_ID, angle)
    else:
        print(f"[robot_ctrl] 跳过 laser_blue({'ON' if on else 'OFF'})")


def laser_ir(on: bool):
    """
    控制 S4 红外指示激光（瞄准镜）。
    on=True → 打开，on=False → 关闭
    """
    angle = LASER_ON_ANGLE if on else LASER_OFF_ANGLE
    if ROBOT_OK:
        robot.set_pwm_servo(LASER_IR_ID, angle)
    else:
        print(f"[robot_ctrl] 跳过 laser_ir({'ON' if on else 'OFF'})")


def all_lasers_off():
    """
    安全关闭所有激光。
    ★ 所有节点退出前必须调用，避免激光误开造成伤害。
    """
    laser_blue(False)
    laser_ir(False)


def fire_blue_pulse(duration: float = 1.0):
    """
    蓝紫激光脉冲：开 → 等 duration 秒 → 关。
    同步阻塞版本，如不想阻塞请在独立线程中调用。
    """
    laser_blue(True)
    time.sleep(duration)
    laser_blue(False)
