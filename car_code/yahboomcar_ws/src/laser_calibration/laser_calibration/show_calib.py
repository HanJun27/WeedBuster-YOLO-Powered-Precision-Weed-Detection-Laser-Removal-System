#!/usr/bin/env python3
"""
show_calib.py —— 查看当前标定参数  v3.10.7
运行：
    ros2 run laser_calibration show_calib

v3.10.7：补上标定四（active diffuse / 相对 NDVI）显示，并修正就绪判定——
  · 激光除草线（Phase 3 视觉伺服）只依赖 标定一 + 标定二。
  · NDVI 线：标定四（主动光经验线法，相对 NDVI，本项目实际路径）
            或 标定三（4 点色卡 + 标准漫反射板，绝对 NDVI，需标准板）。
"""
from laser_calibration.calib_io import load_calib


def main():
    p = load_calib()
    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  当前标定参数")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    s1 = "✅" if p.calib1_done else "❌ 未完成"
    s2 = "✅" if p.calib2_done else "❌ 未完成"

    print(f"\n[标定一] 摄像头基线对齐  {s1}")
    print(f"  Shift_X = {p.shift_x:+d} 像素  "
          f"(IR 图需向{'右' if p.shift_x > 0 else '左'}平移对齐 RGB)")
    print(f"  Shift_Y = {p.shift_y:+d} 像素  (参考值，应≈0)")

    print(f"\n[标定二] 激光偏移量      {s2}")
    print(f"  Delta_X = {p.delta_x:+d} 像素")
    print(f"  Delta_Y = {p.delta_y:+d} 像素")
    if p.calib2_done:
        if p.calib2_frame == "rgb":
            print(f"  坐标系  = rgb ✅ (v3.9+ vision_servo 兼容)")
        elif p.calib2_frame == "ir":
            print(f"  坐标系  = ir  ⛔ (v3.8 历史数据，v3.9+ 不兼容，请重做)")
        else:
            print(f"  坐标系  = <未标记>  ⚠️ (推断为 v3.8 IR 历史数据，请重做)")
    print(f"  含义：蓝紫激光真实落点 = 红光斑坐标 + "
          f"({p.delta_x:+d}, {p.delta_y:+d})")

    # ── 标定三：绝对 NDVI（需标准漫反射板，本项目通常未做）──
    s3 = "✅" if p.refl_calibrated else "— 未做（无标准板时跳过）"
    print(f"\n[标定三] 反射率定标 (绝对NDVI / 需标准板)  {s3}")
    if p.refl_calibrated:
        print(f"  R 通道:   refl = {p.k1:.6f} × DN + {p.b1:.6f}  "
              f"R²={p.refl_r2_red:.4f}")
        print(f"  NIR 通道: refl = {p.k2:.6f} × DN + {p.b2:.6f}  "
              f"R²={p.refl_r2_nir:.4f}")
        print(f"  上次标定时间: {p.refl_timestamp}")
        print("  ⚠️ 环境光变化后必须重新标定")
    else:
        print("  → 有标准漫反射板时：ros2 run laser_calibration calib_refl")

    # ── 标定四：相对 NDVI（主动光经验线法，本项目实际路径）──
    s4 = "✅" if p.calib4_done else "❌ 未完成"
    print(f"\n[标定四] 主动光漫反射 (相对NDVI / 经验线法)  {s4}")
    if p.calib4_done:
        print(f"  K(灰卡 R'/NIR') = {p.k_active:.4f}   gamma = {p.gamma:.2f}")
        print(f"  暗电流  dark_R = {p.dark_R:.2f}  dark_NIR = {p.dark_NIR:.2f}")
        print(f"  参考灰卡反射率 = {p.gray_reflectance:.3f}")
        print(f"  光源/距离 = {p.calib4_light or '<未记录>'} / "
              f"{p.calib4_distance_cm} cm")
        print(f"  上次标定时间: {p.calib4_timestamp}")
        print("  公式：NDVI = (K·NIR' − R') / (K·NIR' + R')，先去伽马再减暗电流")
        print("  ⚠️ 输出为相对 NDVI（同设备/同光场可比，不等于绝对 NDVI）")
    else:
        print("  → ros2 run laser_calibration calib_diffuse  (相对 NDVI 必需)")

    # ── 就绪判定 ──
    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    weeding_ready = p.is_ready()           # 标定一 + 标定二
    ndvi_ready = p.calib4_done or p.refl_calibrated

    print(f"激光除草线 (Phase 3 视觉伺服):  "
          f"{'✅ 就绪（标定一+二完成）' if weeding_ready else '⚠ 未就绪'}")
    if not weeding_ready:
        if not p.calib1_done:
            print("   ros2 run laser_calibration calib_camera")
        if not p.calib2_done:
            print("   ros2 run laser_calibration calib_laser")

    if ndvi_ready:
        mode = "绝对(标定三)" if p.refl_calibrated else "相对(标定四)"
        print(f"NDVI 线:  ✅ 就绪（{mode}）")
    else:
        print("NDVI 线:  ⚠ 未就绪 → ros2 run laser_calibration calib_diffuse")

    if weeding_ready:
        print()
        print("  from laser_calibration.calib_io import load_calib")
        print("  p = load_calib()")
        print("  req_x, req_y = p.target_to_required_spot(target_x, target_y)")
        if p.calib4_done:
            print("  ndvi = p.active_ndvi(dn_r, dn_nir)   # 相对 NDVI")
    print()


if __name__ == "__main__":
    main()
