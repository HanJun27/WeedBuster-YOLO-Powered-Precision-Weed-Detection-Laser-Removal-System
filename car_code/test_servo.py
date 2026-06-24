#!/usr/bin/env python3
"""
test_servo.py —— 云台 S1/S2 舵机独立测试
直接调用 SunriseRobot SDK，纯 PWM 控制。

用法: python3 test_servo.py
"""
import time
from SunriseRobotLib import SunriseRobot

# ── 参数 ────────────────────────────────────────
SERVO_YAW   = 1     # S1 水平偏航
SERVO_PITCH = 2     # S2 俯仰
CENTER      = 90
SERIAL      = "/dev/myserial"

print("正在连接 SunriseRobot SDK...")
robot = SunriseRobot(com=SERIAL)
robot.create_receive_threading()
time.sleep(0.5)
print("✓ SDK 已连接")

# 上电归中
robot.set_pwm_servo(SERVO_YAW,   CENTER)
robot.set_pwm_servo(SERVO_PITCH, CENTER)
time.sleep(0.5)
print(f"云台已归中 (yaw={CENTER}, pitch={CENTER})")

cur_yaw   = CENTER
cur_pitch = CENTER

print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  云台 S1/S2 舵机测试
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
当前角度命令:
  c                归中 (90, 90)
  y <角度>         设置 yaw,    例: y 60
  p <角度>         设置 pitch,  例: p 100
  yp <yaw> <pitch> 同时设置, 例: yp 70 80

相对移动（每次小幅）:
  l                yaw 左转 5°
  r                yaw 右转 5°
  u                pitch 上抬 5°
  d                pitch 下俯 5°

测试动作:
  scan             扫描动作（左右左右上下，验证两轴都能转）
  square           走方形（yaw 60→120, pitch 70→110，看清楚每次到位）
  show             显示当前角度

  q                退出
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")

def move_to(yaw, pitch, settle=0.4):
    global cur_yaw, cur_pitch
    yaw   = max(45, min(135, yaw))     # 限位
    pitch = max(60, min(120, pitch))
    robot.set_pwm_servo(SERVO_YAW,   int(yaw))
    robot.set_pwm_servo(SERVO_PITCH, int(pitch))
    cur_yaw, cur_pitch = yaw, pitch
    time.sleep(settle)
    print(f"  → yaw={cur_yaw}, pitch={cur_pitch}")

try:
    while True:
        cmd = input("S1S2> ").strip().lower()
        if not cmd:
            continue

        if cmd == "c":
            move_to(CENTER, CENTER)
        elif cmd == "show":
            print(f"  当前: yaw={cur_yaw}, pitch={cur_pitch}")
        elif cmd == "l":
            move_to(cur_yaw - 5, cur_pitch)
        elif cmd == "r":
            move_to(cur_yaw + 5, cur_pitch)
        elif cmd == "u":
            move_to(cur_yaw, cur_pitch - 5)   # pitch 减小 = 上抬
        elif cmd == "d":
            move_to(cur_yaw, cur_pitch + 5)
        elif cmd == "scan":
            print("  → 扫描动作...")
            for y in [60, 90, 120, 90]:
                move_to(y, 90, settle=0.6)
            for p in [70, 90, 110, 90]:
                move_to(90, p, settle=0.6)
            print("  ✓ 扫描完成")
        elif cmd == "square":
            print("  → 走方形（4 个角点）...")
            for y, p in [(60, 70), (120, 70), (120, 110), (60, 110), (90, 90)]:
                move_to(y, p, settle=0.8)
            print("  ✓ 方形完成")
        elif cmd.startswith("y "):
            try:
                a = int(cmd.split()[1])
                move_to(a, cur_pitch)
            except (ValueError, IndexError):
                print("  ✗ 用法: y <角度 0-180>")
        elif cmd.startswith("p "):
            try:
                a = int(cmd.split()[1])
                move_to(cur_yaw, a)
            except (ValueError, IndexError):
                print("  ✗ 用法: p <角度 0-180>")
        elif cmd.startswith("yp "):
            try:
                parts = cmd.split()
                y = int(parts[1])
                p = int(parts[2])
                move_to(y, p)
            except (ValueError, IndexError):
                print("  ✗ 用法: yp <yaw> <pitch>")
        elif cmd in ("q", "quit", "exit"):
            break
        else:
            print(f"  ✗ 未知命令: {cmd}")
except KeyboardInterrupt:
    pass
finally:
    # 退出前归中（保护舵机/避免下次开机抽搐）
    robot.set_pwm_servo(SERVO_YAW,   CENTER)
    robot.set_pwm_servo(SERVO_PITCH, CENTER)
    print("\n✓ 已归中，退出")
