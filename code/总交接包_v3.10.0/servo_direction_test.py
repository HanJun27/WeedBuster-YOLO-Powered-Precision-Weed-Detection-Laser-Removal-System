#!/usr/bin/env python3
"""
servo_direction_test.py —— 舵机方向最小测试（不依赖 vision_servo / 标定 / YOLO）
===============================================================================
目的：搞清楚 yaw/pitch 舵机角度增大时，云台物理上朝哪个方向转。

用法：
    cd ~/yahboomcar_ws
    source install/setup.bash
    python3 src/laser_calibration/servo_direction_test.py

每一步会等你按回车，给你时间观察云台动作。
全程开 S4 红激光做指示，看光斑打到墙上的位置最直观。

测完会告诉你 PIXEL_TO_YAW_DEG 和 PIXEL_TO_PITCH_DEG 应该是正还是负。
"""

import sys
import time

# 确保能 import laser_calibration（无论从哪跑）
try:
    from laser_calibration.robot_ctrl import (
        ROBOT_OK, set_servo, center_servo,
        laser_ir, all_lasers_off,
    )
    from laser_calibration.config import SERVO_YAW_CENTER, SERVO_PITCH_CENTER
except ImportError as e:
    print(f"❌ Import 失败：{e}")
    print("请先 source install/setup.bash")
    sys.exit(1)


def wait_user(prompt):
    print(f"\n  → {prompt}")
    input("  (按回车继续...)")


def main():
    print("═══════════════════════════════════════════════════════════")
    print("  舵机方向最小测试")
    print("═══════════════════════════════════════════════════════════")

    if not ROBOT_OK:
        print("❌ SDK 未连接，无法测试")
        sys.exit(1)

    print("\n本测试将：")
    print("  1. 归中云台 (yaw=90, pitch=90)")
    print("  2. 开 S4 红激光做指示")
    print("  3. 让 yaw 从 90 → 110，让你看云台朝哪边转")
    print("  4. 让 yaw 从 90 → 70，让你看云台朝哪边转")
    print("  5. 同样测 pitch 方向")
    print("  6. 给出 PIXEL_TO_YAW_DEG / PIXEL_TO_PITCH_DEG 的正确符号建议")
    print()
    print("  ⚠️ 准备一面墙或者白纸放在云台前方约 1 米处，红光斑会打在上面。")

    wait_user("准备好了吗？")

    # ─── Step 1: 归中 + 开红激光 ──────────────────────────────
    print(f"\n[1] 归中云台 (yaw={SERVO_YAW_CENTER}, pitch={SERVO_PITCH_CENTER})")
    center_servo()
    time.sleep(1.0)
    laser_ir(True)
    time.sleep(0.5)
    print("    红激光已开。看墙上红光斑位置 = 这是「中心」。")
    wait_user("记住中心位置")

    # ─── Step 2: yaw 加 20 度 ─────────────────────────────────
    yaw_test_high = SERVO_YAW_CENTER + 20
    print(f"\n[2] 测试 yaw: {SERVO_YAW_CENTER} → {yaw_test_high} (角度增大)")
    set_servo(yaw_test_high, SERVO_PITCH_CENTER)
    time.sleep(1.0)
    print(f"    云台 yaw 已设为 {yaw_test_high}。看红光斑相对刚才中心：")
    print("       a) 朝你的左边移了？")
    print("       b) 朝你的右边移了？")

    while True:
        ans_yaw = input("  输入 a 或 b (a=左, b=右): ").strip().lower()
        if ans_yaw in ("a", "b"):
            break
        print("  请输入 a 或 b")

    # 归回再测反向，避免视觉记忆混淆
    print(f"\n    归中再测反向...")
    center_servo()
    time.sleep(1.0)

    yaw_test_low = SERVO_YAW_CENTER - 20
    print(f"\n[2b] 测试 yaw: {SERVO_YAW_CENTER} → {yaw_test_low} (角度减小)")
    set_servo(yaw_test_low, SERVO_PITCH_CENTER)
    time.sleep(1.0)
    print("    确认：红光斑应该在反方向（即{}）".format("右边" if ans_yaw == "a" else "左边"))
    wait_user("确认 yaw 方向逻辑")

    # ─── Step 3: pitch 加 20 度 ───────────────────────────────
    print(f"\n[3] 归中后测试 pitch")
    center_servo()
    time.sleep(1.0)

    pitch_test_high = SERVO_PITCH_CENTER + 20
    print(f"    pitch: {SERVO_PITCH_CENTER} → {pitch_test_high} (角度增大)")
    set_servo(SERVO_YAW_CENTER, pitch_test_high)
    time.sleep(1.0)
    print(f"    云台 pitch 已设为 {pitch_test_high}。看红光斑相对刚才中心：")
    print("       a) 朝上移了？(云台仰头)")
    print("       b) 朝下移了？(云台低头)")

    while True:
        ans_pitch = input("  输入 a 或 b (a=上, b=下): ").strip().lower()
        if ans_pitch in ("a", "b"):
            break
        print("  请输入 a 或 b")

    print(f"\n    归中再测反向...")
    center_servo()
    time.sleep(1.0)

    pitch_test_low = SERVO_PITCH_CENTER - 20
    print(f"\n[3b] 测试 pitch: {SERVO_PITCH_CENTER} → {pitch_test_low} (角度减小)")
    set_servo(SERVO_YAW_CENTER, pitch_test_low)
    time.sleep(1.0)
    print("    确认：红光斑应该在反方向（即{}）".format("下方" if ans_pitch == "a" else "上方"))
    wait_user("确认 pitch 方向逻辑")

    # ─── Step 4: 关激光 + 归中 + 给结论 ────────────────────────
    print(f"\n[4] 测试结束，归中并关激光")
    center_servo()
    time.sleep(0.5)
    all_lasers_off()
    time.sleep(0.3)

    # ─── 结论 ──────────────────────────────────────────────────
    print("\n═══════════════════════════════════════════════════════════")
    print("  实测结果")
    print("═══════════════════════════════════════════════════════════")
    print(f"  yaw 角度增大 → 红光斑朝{'左' if ans_yaw == 'a' else '右'}移")
    print(f"  pitch 角度增大 → 红光斑朝{'上' if ans_pitch == 'a' else '下'}移")

    print("\n═══════════════════════════════════════════════════════════")
    print("  推导 PIXEL_TO_*_DEG 应该是正还是负")
    print("═══════════════════════════════════════════════════════════")

    print("\n物理逻辑：")
    print("  RGB 画面坐标系：x 向右增大，y 向下增大。")
    print("  视觉伺服需求：当目标在画面 x 大处（右），红光斑要朝右移动到目标位置。")
    print("  所以「红光斑朝右移」对应「dx_pixel > 0」对应「需要 yaw 增量 > 0」")
    print()
    print("  公式: delta_yaw = dx_pixel * PIXEL_TO_YAW_DEG")
    print("  要让 dx_pixel>0 时云台让光斑朝右移，需要：")

    # YAW
    print()
    if ans_yaw == "b":
        print("  ✅ yaw 增大 → 光斑朝右 (一致), PIXEL_TO_YAW_DEG > 0")
        suggest_yaw = "+0.10"
    else:
        print("  ⚠️ yaw 增大 → 光斑朝左 (反向), PIXEL_TO_YAW_DEG < 0")
        suggest_yaw = "-0.10"
    print(f"     建议: PIXEL_TO_YAW_DEG = {suggest_yaw}")

    # PITCH
    # 视觉伺服需求：dy_pixel > 0 (目标在画面下方)，光斑要朝下移
    # 公式 vision_servo._step_coarse: delta_pitch = -dy_pixel * PIXEL_TO_PITCH_DEG
    #   注意这里有个负号！代码里就是 -dy_pixel * PIXEL_TO_PITCH_DEG
    # 也就是 dy_pixel>0 时，需要 delta_pitch < 0 (即 pitch 减小) 让光斑朝下吗？还是相反？
    # 让我重新推：
    #   dy_pixel = required_y - spot_y。如果 required 在 spot 下方，dy_pixel>0
    #   要让光斑朝下，需要光斑的画面 y 增大
    #   光斑画面 y 增大 = 云台 pitch 朝"光斑朝下"那个方向转
    print()
    print("  Pitch 推导（注意代码里有 -dy_pixel * PIXEL_TO_PITCH_DEG 的负号）：")
    if ans_pitch == "b":
        # pitch 增大 → 光斑朝下
        # 要让 dy_pixel>0 时光斑朝下，需要 pitch 增大，即 delta_pitch>0
        # delta_pitch = -dy_pixel * PIXEL_TO_PITCH_DEG, dy_pixel>0
        # 要 delta_pitch>0 需要 PIXEL_TO_PITCH_DEG < 0
        print("  ⚠️ pitch 增大 → 光斑朝下，因为代码有 -dy_pixel 的负号，")
        print("     PIXEL_TO_PITCH_DEG 应取 < 0")
        suggest_pitch = "-0.10"
    else:
        # pitch 增大 → 光斑朝上
        # 要让 dy_pixel>0 时光斑朝下，需要 pitch 减小，即 delta_pitch<0
        # delta_pitch = -dy_pixel * PIXEL_TO_PITCH_DEG, dy_pixel>0
        # 要 delta_pitch<0 需要 PIXEL_TO_PITCH_DEG > 0
        print("  ✅ pitch 增大 → 光斑朝上，因为代码有 -dy_pixel 的负号，")
        print("     PIXEL_TO_PITCH_DEG 应取 > 0")
        suggest_pitch = "+0.10"
    print(f"     建议: PIXEL_TO_PITCH_DEG = {suggest_pitch}")

    print("\n═══════════════════════════════════════════════════════════")
    print(f"  请把 config.py 改成：")
    print(f"      PIXEL_TO_YAW_DEG   = {suggest_yaw}")
    print(f"      PIXEL_TO_PITCH_DEG = {suggest_pitch}")
    print("  然后 colcon build --packages-select laser_calibration")
    print("═══════════════════════════════════════════════════════════")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断，归中关激光...")
        try:
            center_servo()
            all_lasers_off()
        except Exception:
            pass
