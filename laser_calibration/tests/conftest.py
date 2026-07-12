# -*- coding: utf-8 -*-
"""离线测试公共桩:无 ROS2 环境(开发机)时注入 stub,车上有真 rclpy 则用真的。
运行: cd laser_calibration包根目录 && python3 -m pytest tests/ -v
"""
import sys, types, pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m

class _Any:
    def __init__(self, *a, **k): pass
    def __call__(self, *a, **k): return self
    def __getattr__(self, n): return _Any()

try:
    import rclpy  # noqa: F401  车上有真环境
except ImportError:
    rp = _stub("rclpy"); rp.__path__ = []
    _stub("rclpy.node", Node=object)
    _stub("rclpy.qos", QoSProfile=_Any, QoSDurabilityPolicy=_Any())
    _stub("rclpy.executors", MultiThreadedExecutor=_Any)
    _stub("std_msgs"); _stub("std_msgs.msg", String=_Any, Bool=_Any,
                             Empty=_Any, Float32=_Any)
    _stub("sensor_msgs"); _stub("sensor_msgs.msg", Image=_Any)
    _stub("cv_bridge", CvBridge=_Any)
