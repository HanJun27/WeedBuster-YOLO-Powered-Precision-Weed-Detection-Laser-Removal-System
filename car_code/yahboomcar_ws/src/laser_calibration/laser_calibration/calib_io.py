"""
calib_io.py —— 标定常数的读写与封装  v3.10.1
================================================
后续所有节点（NDVI、视觉伺服、时序监测）统一 import 这里。

v3.10.1 改动（相对 v3.10）：
  ★ 新增传感器去伽马（de-gamma）支持：
    - linearize_dn()  模块级函数：gamma 编码 DN → 线性 DN
    - CalibParams.gamma 字段
    - active_ndvi() 先去伽马到线性空间再做经验线法
  原因：消费级摄像头输出的 DN 经过 gamma 非线性编码，
       经验线法（Empirical Line Method）要求在线性空间进行，
       否则两点标定会引入系统性误差。

v3.10 改动（相对 v3.0）：
  - 新增标定四（active diffuse）字段
  - 新增 active_ndvi() 方法
  - 旧 yaml 文件兼容

用法：
    from laser_calibration.calib_io import load_calib
    p = load_calib()
    if p.calib4_done:
        ndvi = p.active_ndvi(dn_r_array, dn_nir_array)   # 已含去伽马
"""

import os
from dataclasses import dataclass

import numpy as np
import yaml

from laser_calibration.config import CALIB_FILE


# ══════════════════════════════════════════════════════════════
#  去伽马（de-gamma）—— 经验线法的前置线性化
# ══════════════════════════════════════════════════════════════
def linearize_dn(dn, gamma: float):
    """
    把 gamma 编码的 DN 值还原为线性 DN。

    消费级摄像头（含本项目的 USB 相机）输出的 DN 经过 gamma 编码，
    呈非线性。经验线法（Empirical Line Method）假设 DN 与反射率线性，
    所以标定与计算都应先在线性空间进行。

    公式：DN_linear = (DN/255)^gamma × 255
    gamma <= 1.0 时跳过（视为不需要校正 / 相机已输出线性 raw）。

    支持标量和 numpy 数组。
    """
    if gamma is None or gamma <= 1.0:
        if isinstance(dn, np.ndarray):
            return dn.astype(np.float32)
        return float(dn)
    if isinstance(dn, np.ndarray):
        norm = np.clip(dn.astype(np.float32) / 255.0, 0.0, 1.0)
        return np.power(norm, gamma) * 255.0
    norm = max(0.0, min(1.0, float(dn) / 255.0))
    return (norm ** gamma) * 255.0


@dataclass
class CalibParams:
    # ── 标定一：摄像头基线 ──────────────────────────────────
    shift_x: int  = 0
    shift_y: int  = 0
    calib1_done: bool = False

    # ── 标定二：激光偏移 ────────────────────────────────────
    delta_x: int  = 0
    delta_y: int  = 0
    calib2_done: bool = False
    calib2_frame: str = ""

    # ── 标定三：反射率定标（4 点色卡，v3.0~v3.9 已暂停）─────
    k1: float = 1.0 / 255.0
    b1: float = 0.0
    k2: float = 1.0 / 255.0
    b2: float = 0.0
    refl_calibrated: bool = False
    refl_timestamp: str   = ""
    refl_r2_red: float    = 0.0
    refl_r2_nir: float    = 0.0

    # ── 标定四：active diffuse（经验线法）──────────────────
    # 公式：NDVI = (K·NIR' − R') / (K·NIR' + R')
    #   先去伽马： R_lin = linearize_dn(DN_R, gamma)
    #   减暗电流： R'    = max(0, R_lin − dark_R)
    #             NIR'  = max(0, NIR_lin − dark_NIR)
    #   增益系数： K     = R_gray' / NIR_gray'（灰卡 ROI 均值，线性空间）
    dark_R: float        = 0.0     # R 通道暗电流（线性空间）
    dark_NIR: float      = 0.0     # NIR 通道暗电流（线性空间）
    k_active: float      = 1.0     # 灰卡 R'/NIR' 比值（线性空间）
    gray_reflectance: float = 0.18 # 经验线法参考物已知反射率
    gamma: float         = 2.2     # 传感器伽马（去伽马用；1.0=不校正）
    calib4_done: bool    = False
    calib4_timestamp: str = ""
    calib4_light: str    = ""
    calib4_distance_cm: int = 0

    # ───────────────────────────────────────────────────────
    def is_ready(self) -> bool:
        return self.calib1_done and self.calib2_done

    def rgb_to_ir(self, rgb_x: int, rgb_y: int):
        return rgb_x - self.shift_x, rgb_y - self.shift_y

    def ir_to_rgb(self, ir_x: int, ir_y: int):
        return ir_x + self.shift_x, ir_y + self.shift_y

    def spot_to_hit(self, spot_x: int, spot_y: int):
        return spot_x + self.delta_x, spot_y + self.delta_y

    def target_to_required_spot(self, target_x: int, target_y: int):
        return target_x - self.delta_x, target_y - self.delta_y

    # ── 反射率换算（v3.0 旧 4 点色卡路径，向后兼容）──────────
    def dn_to_refl_red(self, dn):
        return self.k1 * dn + self.b1

    def dn_to_refl_nir(self, dn):
        return self.k2 * dn + self.b2

    # ── active mode NDVI（经验线法 + 去伽马）─────────────────
    def active_ndvi(self, dn_r, dn_nir):
        """
        主动光场标定下计算相对 NDVI（向量化，支持标量或 numpy 数组）。

        步骤：
          1. 去伽马：把 gamma 编码 DN 还原为线性 DN
          2. 减暗电流：R' = max(0, R_lin − dark_R)
          3. K 修正：K·NIR' 把 NIR 通道拉到 R 通道同基准
          4. NDVI = (K·NIR' − R') / (K·NIR' + R')

        前置：calib4_done = True（否则结果无意义）
        """
        # Step 1: 去伽马到线性空间
        r_lin = linearize_dn(dn_r, self.gamma)
        nir_lin = linearize_dn(dn_nir, self.gamma)

        # Step 2: 减暗电流（dark_R/dark_NIR 已是线性空间的值）
        if isinstance(r_lin, np.ndarray):
            r = np.maximum(0.0, r_lin - self.dark_R)
            nir = np.maximum(0.0, nir_lin - self.dark_NIR)
        else:
            r = max(0.0, r_lin - self.dark_R)
            nir = max(0.0, nir_lin - self.dark_NIR)

        # Step 3 & 4: K 修正 + NDVI
        kn = self.k_active * nir
        denom = kn + r
        if isinstance(denom, np.ndarray):
            ndvi = (kn - r) / (denom + 1e-5)
            return np.clip(ndvi, -1.0, 1.0)
        else:
            if denom < 1e-5:
                return 0.0
            return max(-1.0, min(1.0, (kn - r) / denom))


def load_calib() -> CalibParams:
    """从 YAML 文件读取标定参数，文件不存在或字段缺失时使用默认值。"""
    if not os.path.exists(CALIB_FILE):
        return CalibParams()
    with open(CALIB_FILE, "r") as f:
        d = yaml.safe_load(f) or {}
    return CalibParams(
        shift_x     = d.get("shift_x",     0),
        shift_y     = d.get("shift_y",     0),
        calib1_done = d.get("calib1_done", False),
        delta_x     = d.get("delta_x",     0),
        delta_y     = d.get("delta_y",     0),
        calib2_done = d.get("calib2_done", False),
        calib2_frame = d.get("calib2_frame", ""),
        k1          = d.get("k1",          1.0 / 255.0),
        b1          = d.get("b1",          0.0),
        k2          = d.get("k2",          1.0 / 255.0),
        b2          = d.get("b2",          0.0),
        refl_calibrated = d.get("refl_calibrated", False),
        refl_timestamp  = d.get("refl_timestamp",  ""),
        refl_r2_red     = d.get("refl_r2_red",     0.0),
        refl_r2_nir     = d.get("refl_r2_nir",     0.0),
        dark_R       = d.get("dark_R",       0.0),
        dark_NIR     = d.get("dark_NIR",     0.0),
        k_active     = d.get("k_active",     1.0),
        gray_reflectance = d.get("gray_reflectance", 0.18),
        gamma        = d.get("gamma",        2.2),
        calib4_done  = d.get("calib4_done",  False),
        calib4_timestamp = d.get("calib4_timestamp", ""),
        calib4_light = d.get("calib4_light", ""),
        calib4_distance_cm = d.get("calib4_distance_cm", 0),
    )


def save_calib(params: dict):
    """增量式保存：合并写入 YAML，不覆盖其他字段。
    v3.10.7：改原子写（先写 .tmp 再 os.replace），防写一半被崩溃/并发损坏。"""
    existing = {}
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE, "r") as f:
            existing = yaml.safe_load(f) or {}
    existing.update(params)
    os.makedirs(os.path.dirname(CALIB_FILE) or ".", exist_ok=True)
    tmp = CALIB_FILE + ".tmp"
    with open(tmp, "w") as f:
        yaml.dump(existing, f, default_flow_style=False, allow_unicode=True)
    os.replace(tmp, CALIB_FILE)   # 原子替换
    print(f"[calib_io] 已保存到 {CALIB_FILE}")
