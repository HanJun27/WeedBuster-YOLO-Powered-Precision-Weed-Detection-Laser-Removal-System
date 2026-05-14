"""
calib_io.py —— 标定常数的读写与封装  v3.10
===========================================
后续所有节点（NDVI、视觉伺服）统一 import 这里。

v3.10 改动（相对 v3.0）：
  1. 新增标定四（active diffuse）字段，配合 calib_diffuse 节点：
     - dark_R / dark_NIR   暗电流（盖镜头测）
     - k_active            灰卡 R/NIR DN 比值（已减暗电流）
     - calib4_done / calib4_timestamp / calib4_light / calib4_distance_cm
  2. 新增 active_ndvi() 方法：暗电流减除 + K 修正后的真 NDVI
  3. 旧 yaml 文件兼容：新字段都有默认值，不影响 v3.0~v3.9 老数据加载

用法：
    from laser_calibration.calib_io import load_calib
    p = load_calib()
    if p.calib4_done:
        # 主动光场 active mode 算 NDVI
        ndvi = p.active_ndvi(dn_r_array, dn_nir_array)
    elif p.refl_calibrated:
        # 老的反射率定标（4 点色卡）
        ...
    else:
        # 伪 NDVI（兜底）
        ...
"""

import os
from dataclasses import dataclass

import numpy as np
import yaml

from laser_calibration.config import CALIB_FILE


@dataclass
class CalibParams:
    # ── 标定一：摄像头基线 ──────────────────────────────────
    shift_x: int  = 0      # IR 图在 X 轴需要平移的像素数（正值→向右平移）
    shift_y: int  = 0      # 理论应≈0
    calib1_done: bool = False

    # ── 标定二：激光偏移 ────────────────────────────────────
    delta_x: int  = 0      # 蓝紫激光落点 相对 红激光光斑 的 X 偏移
    delta_y: int  = 0      # 蓝紫激光落点 相对 红激光光斑 的 Y 偏移
    calib2_done: bool = False
    # v3.9 新增：标定二是在哪个画面坐标系下做的
    # ""    = 历史数据（v3.8 之前），坐标系未知
    # "ir"  = v3.8 时代，IR 摄像头画面下标定（v3.9 不可用）
    # "rgb" = v3.9+，RGB 摄像头画面下标定（v3.9+ 用这个）
    calib2_frame: str = ""

    # ── 标定三：反射率定标（4 点色卡，v3.0~v3.9 已暂停）─────
    # DN_R   →  真实红光反射率：   refl_red = k1 * DN_R   + b1
    # DN_NIR →  真实近红外反射率: refl_nir = k2 * DN_NIR + b2
    k1: float = 1.0 / 255.0    # 默认线性归一化（伪 NDVI 兜底）
    b1: float = 0.0
    k2: float = 1.0 / 255.0
    b2: float = 0.0
    refl_calibrated: bool = False
    refl_timestamp: str   = ""    # ISO 8601 时间戳，提示是否需要重新标定
    refl_r2_red: float    = 0.0   # 拟合质量评估
    refl_r2_nir: float    = 0.0

    # ── 标定四：active diffuse（v3.10 新增）─────────────────
    # 主动光源 + 单点灰卡 / PTFE 板 标定，专门服务 active canopy sensor
    # 公式：NDVI = (K·NIR' − R') / (K·NIR' + R')
    #       R'   = max(0, DN_R   − dark_R)
    #       NIR' = max(0, DN_NIR − dark_NIR)
    #       K    = R_gray' / NIR_gray'   （灰卡 ROI 内均值，已减暗电流）
    dark_R: float        = 0.0     # R 通道暗电流（盖镜头多帧均值）
    dark_NIR: float      = 0.0     # NIR 通道暗电流
    k_active: float      = 1.0     # 灰卡 R/NIR DN 比值
    gray_reflectance: float = 0.18 # 参考物已知反射率（18%灰卡=0.18; PTFE≈0.99）
    calib4_done: bool    = False
    calib4_timestamp: str = ""
    # 标定时光源类型，用于运行期校验是否切换了光源
    # "indoor_active" = 850nm IR LED + 白光 LED
    # "indoor_window" = 自然光透过窗户
    # "outdoor_sun"   = 户外阳光
    # "indoor_halogen"= 卤素灯
    calib4_light: str    = ""
    calib4_distance_cm: int = 0    # 标定时工作距离（cm），运行偏离过远要警告

    # ───────────────────────────────────────────────────────
    def is_ready(self) -> bool:
        return self.calib1_done and self.calib2_done

    def rgb_to_ir(self, rgb_x: int, rgb_y: int):
        """RGB 坐标系 → IR 坐标系（YOLO 检测结果转换用）"""
        return rgb_x - self.shift_x, rgb_y - self.shift_y

    def ir_to_rgb(self, ir_x: int, ir_y: int):
        """IR 坐标系 → RGB 坐标系"""
        return ir_x + self.shift_x, ir_y + self.shift_y

    def spot_to_hit(self, spot_x: int, spot_y: int):
        """红外光斑坐标 → 蓝紫激光真实落点。Hit = Spot + Delta"""
        return spot_x + self.delta_x, spot_y + self.delta_y

    def target_to_required_spot(self, target_x: int, target_y: int):
        """要让蓝紫激光打中 target，红光斑需要移动到哪。Spot_required = Target − Delta"""
        return target_x - self.delta_x, target_y - self.delta_y

    # ── 反射率换算（v3.0 旧的 4 点色卡路径，向后兼容）─────────
    def dn_to_refl_red(self, dn):
        """红光通道 DN 值 → 真实反射率（支持标量或 numpy 数组）"""
        return self.k1 * dn + self.b1

    def dn_to_refl_nir(self, dn):
        """近红外通道 DN 值 → 真实反射率"""
        return self.k2 * dn + self.b2

    # ── v3.10 新增：active mode NDVI ─────────────────────────
    def active_ndvi(self, dn_r, dn_nir):
        """
        主动光场标定下计算真 NDVI（向量化，支持标量或 numpy 数组）。

        步骤：
          1. 减暗电流：R' = max(0, DN_R − dark_R)
                      NIR' = max(0, DN_NIR − dark_NIR)
          2. 应用 K 修正：K·NIR' 把 NIR 通道拉到 R 通道同基准
          3. NDVI = (K·NIR' − R') / (K·NIR' + R')

        前置：calib4_done = True（否则结果无意义）
        """
        if isinstance(dn_r, np.ndarray):
            r = np.maximum(0.0, dn_r.astype(np.float32) - self.dark_R)
            nir = np.maximum(0.0, dn_nir.astype(np.float32) - self.dark_NIR)
        else:
            r = max(0.0, float(dn_r) - self.dark_R)
            nir = max(0.0, float(dn_nir) - self.dark_NIR)

        kn = self.k_active * nir
        denom = kn + r
        if isinstance(denom, np.ndarray):
            ndvi = (kn - r) / (denom + 1e-5)
            return np.clip(ndvi, -1.0, 1.0)
        else:
            if denom < 1e-5:
                return 0.0
            ndvi = (kn - r) / denom
            return max(-1.0, min(1.0, ndvi))


def load_calib() -> CalibParams:
    """从 YAML 文件读取标定参数，文件不存在或字段缺失时使用默认值。"""
    if not os.path.exists(CALIB_FILE):
        return CalibParams()
    with open(CALIB_FILE, "r") as f:
        d = yaml.safe_load(f) or {}
    p = CalibParams(
        # 标定一
        shift_x     = d.get("shift_x",     0),
        shift_y     = d.get("shift_y",     0),
        calib1_done = d.get("calib1_done", False),
        # 标定二
        delta_x     = d.get("delta_x",     0),
        delta_y     = d.get("delta_y",     0),
        calib2_done = d.get("calib2_done", False),
        calib2_frame = d.get("calib2_frame", ""),
        # 标定三
        k1          = d.get("k1",          1.0 / 255.0),
        b1          = d.get("b1",          0.0),
        k2          = d.get("k2",          1.0 / 255.0),
        b2          = d.get("b2",          0.0),
        refl_calibrated = d.get("refl_calibrated", False),
        refl_timestamp  = d.get("refl_timestamp",  ""),
        refl_r2_red     = d.get("refl_r2_red",     0.0),
        refl_r2_nir     = d.get("refl_r2_nir",     0.0),
        # 标定四（v3.10 新增）
        dark_R       = d.get("dark_R",       0.0),
        dark_NIR     = d.get("dark_NIR",     0.0),
        k_active     = d.get("k_active",     1.0),
        gray_reflectance = d.get("gray_reflectance", 0.18),
        calib4_done  = d.get("calib4_done",  False),
        calib4_timestamp = d.get("calib4_timestamp", ""),
        calib4_light = d.get("calib4_light", ""),
        calib4_distance_cm = d.get("calib4_distance_cm", 0),
    )
    return p


def save_calib(params: dict):
    """
    增量式保存：将新参数合并写入 YAML，不覆盖已有其他字段。
    这样标定一/二/三/四可以独立保存，互不影响。
    """
    existing = {}
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE, "r") as f:
            existing = yaml.safe_load(f) or {}
    existing.update(params)
    with open(CALIB_FILE, "w") as f:
        yaml.dump(existing, f, default_flow_style=False, allow_unicode=True)
    print(f"[calib_io] 已保存到 {CALIB_FILE}")
