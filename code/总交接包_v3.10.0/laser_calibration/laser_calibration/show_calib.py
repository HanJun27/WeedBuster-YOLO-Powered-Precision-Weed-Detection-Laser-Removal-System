#!/usr/bin/env python3
"""
show_calib.py —— 查看当前标定参数  v3.5
运行：
    ros2 run laser_calibration show_calib
"""
from laser_calibration.calib_io import load_calib


def main():
    p = load_calib()
    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  当前标定参数")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    s1 = "✅" if p.calib1_done else "❌ 未完成"
    s2 = "✅" if p.calib2_done else "❌ 未完成"
    s3 = "✅" if p.refl_calibrated else "❌ 未完成 (NDVI 退化为伪 NDVI)"

    print(f"\n[标定一] 摄像头基线对齐  {s1}")
    print(f"  Shift_X = {p.shift_x:+d} 像素  "
          f"(IR 图需向{'右' if p.shift_x > 0 else '左'}平移对齐 RGB)")
    print(f"  Shift_Y = {p.shift_y:+d} 像素  (参考值，应≈0)")

    print(f"\n[标定二] 激光偏移量      {s2}")
    print(f"  Delta_X = {p.delta_x:+d} 像素")
    print(f"  Delta_Y = {p.delta_y:+d} 像素")
    # v3.9.1: 显示标定二的坐标系
    if p.calib2_done:
        if p.calib2_frame == "rgb":
            print(f"  坐标系  = rgb ✅ (v3.9+ vision_servo 兼容)")
        elif p.calib2_frame == "ir":
            print(f"  坐标系  = ir  ⛔ (v3.8 历史数据，v3.9+ 不兼容，请重做)")
        else:
            print(f"  坐标系  = <未标记>  ⚠️ (推断为 v3.8 IR 历史数据，v3.9+ 请重做)")
    print(f"  含义：蓝紫激光真实落点 = 红光斑坐标 + "
          f"({p.delta_x:+d}, {p.delta_y:+d})")

    print(f"\n[标定三] 反射率定标 (真NDVI)  {s3}")
    if p.refl_calibrated:
        print(f"  R 通道:   refl = {p.k1:.6f} × DN + {p.b1:.6f}  "
              f"R²={p.refl_r2_red:.4f}")
        print(f"  NIR 通道: refl = {p.k2:.6f} × DN + {p.b2:.6f}  "
              f"R²={p.refl_r2_nir:.4f}")
        print(f"  上次标定时间: {p.refl_timestamp}")
        print("  ⚠️ 环境光变化后必须重新标定")
    else:
        print("  → ros2 run laser_calibration calib_refl")

    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    if p.is_ready() and p.refl_calibrated:
        print("✅ 全部三项标定完成，可进入 Phase 3 视觉伺服")
        print()
        print("  from laser_calibration.calib_io import load_calib")
        print("  p = load_calib()")
        print("  ir_x, ir_y  = p.rgb_to_ir(weed_x_rgb, weed_y_rgb)")
        print("  req_x,req_y = p.target_to_required_spot(target_x, target_y)")
        print("  refl_red    = p.dn_to_refl_red(dn_value)")
    else:
        print("⚠  标定未完成，请按顺序执行：")
        if not p.calib1_done:
            print("   ros2 run laser_calibration calib_camera")
        if not p.calib2_done:
            print("   ros2 run laser_calibration calib_laser")
        if not p.refl_calibrated:
            print("   ros2 run laser_calibration calib_refl    (真NDVI 必需)")
    print()


if __name__ == "__main__":
    main()
