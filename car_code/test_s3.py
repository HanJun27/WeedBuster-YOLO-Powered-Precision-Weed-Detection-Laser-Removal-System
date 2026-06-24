#!/usr/bin/env python3
"""S3 激光开关独立测试 —— 不依赖 ROS2，纯 PWM 控制"""
import time
from SunriseRobotLib import SunriseRobot

SERVO_ID = 3
ANGLE_ON = 180
ANGLE_OFF = 0
SERIAL = "/dev/myserial"

print("正在连接 SunriseRobot SDK...")
robot = SunriseRobot(com=SERIAL)
robot.create_receive_threading()
time.sleep(0.5)
print("✓ SDK 已连接")

robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
print(f"S3 已初始化为 OFF (angle={ANGLE_OFF})")

print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  S3 激光开关测试
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
命令:
  on          打开激光
  off         关闭激光
  blink       闪烁 5 次（每次 0.3 秒）
  pulse       开 1 秒后自动关
  set <角度>  自定义角度 (0-180)，例如: set 90
  q           退出（自动关闭激光）

⚠️ 安全: 测试时镜头不要对人/眼睛，附近无易燃物
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")

try:
    while True:
        cmd = input("S3> ").strip().lower()
        if not cmd:
            continue
        if cmd == "on":
            robot.set_pwm_servo(SERVO_ID, ANGLE_ON)
            print(f"  → S3 ON  (angle={ANGLE_ON})")
        elif cmd == "off":
            robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
            print(f"  → S3 OFF (angle={ANGLE_OFF})")
        elif cmd == "blink":
            print("  → 闪烁 5 次...")
            for i in range(5):
                robot.set_pwm_servo(SERVO_ID, ANGLE_ON)
                time.sleep(0.3)
                robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
                time.sleep(0.3)
                print(f"    {i+1}/5")
            print("  ✓ 闪烁完成")
        elif cmd == "pulse":
            print("  → 开 1 秒...")
            robot.set_pwm_servo(SERVO_ID, ANGLE_ON)
            time.sleep(1.0)
            robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
            print("  ✓ 已自动关闭")
        elif cmd.startswith("set "):
            try:
                a = int(cmd.split()[1])
                a = max(0, min(180, a))
                robot.set_pwm_servo(SERVO_ID, a)
                print(f"  → S3 angle={a}")
            except (ValueError, IndexError):
                print("  ✗ 用法: set <0-180>")
        elif cmd in ("q", "quit", "exit"):
            break
        else:
            print(f"  ✗ 未知命令: {cmd}")
except KeyboardInterrupt:
    pass
finally:
    robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
    print("\n✓ S3 已关闭，退出")
