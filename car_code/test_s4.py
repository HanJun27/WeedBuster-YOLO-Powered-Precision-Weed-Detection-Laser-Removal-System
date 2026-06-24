#!/usr/bin/env python3
"""
test_s4.py —— S4 红外激光开关独立测试
直接调用 SunriseRobot SDK，不走 ROS2，纯 PWM 控制。

⚠️ 红外激光人眼不可见但有热效应，测试时不要直视镜头！
   建议拿一张白纸放镜头前，能看见微弱红色光斑就说明亮了。
   (波长 850nm 大部分人眼仅能看到很暗的暗红色)

用法: python3 test_s4.py
"""
import time
from SunriseRobotLib import SunriseRobot

# ── 参数（如果激光开关响应反了，把 ON 和 OFF 对调）─────
SERVO_ID  = 4       # S4
ANGLE_ON  = 180
ANGLE_OFF = 0
SERIAL    = "/dev/myserial"

print("正在连接 SunriseRobot SDK...")
robot = SunriseRobot(com=SERIAL)
robot.create_receive_threading()
time.sleep(0.5)
print("✓ SDK 已连接")

# 上电先关闭
robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
print(f"S4 已初始化为 OFF (angle={ANGLE_OFF})")

print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  S4 红外激光开关测试
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
命令:
  on          打开激光
  off         关闭激光
  blink       闪烁 5 次（每次 0.3 秒）
  pulse       开 1 秒后自动关
  long        长开 5 秒（看 IR 摄像头是否检测到光斑）
  set <角度>  自定义角度 (0-180)，例如: set 90
  q           退出（自动关闭激光）

⚠️ 安全:
   · 850nm 红外激光人眼几乎看不见，但仍有热效应
   · 不要直视镜头，不要对人/动物/眼睛
   · 用白纸/手背 (远距离!) 验证亮度
   · 用 IR 摄像头网络流 http://<小车IP>:8081 看光斑最直观
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")

try:
    while True:
        cmd = input("S4> ").strip().lower()
        if not cmd:
            continue
        if cmd == "on":
            robot.set_pwm_servo(SERVO_ID, ANGLE_ON)
            print(f"  → S4 ON  (angle={ANGLE_ON})")
        elif cmd == "off":
            robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
            print(f"  → S4 OFF (angle={ANGLE_OFF})")
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
        elif cmd == "long":
            print("  → 长开 5 秒（请打开 IR 流 http://<小车IP>:8081 看光斑）...")
            robot.set_pwm_servo(SERVO_ID, ANGLE_ON)
            for i in range(5, 0, -1):
                print(f"    剩余 {i} 秒...")
                time.sleep(1.0)
            robot.set_pwm_servo(SERVO_ID, ANGLE_OFF)
            print("  ✓ 已自动关闭")
        elif cmd.startswith("set "):
            try:
                a = int(cmd.split()[1])
                a = max(0, min(180, a))
                robot.set_pwm_servo(SERVO_ID, a)
                print(f"  → S4 angle={a}")
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
    print("\n✓ S4 已关闭，退出")
