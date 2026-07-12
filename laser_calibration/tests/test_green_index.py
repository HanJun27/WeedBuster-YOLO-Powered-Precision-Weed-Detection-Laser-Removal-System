# -*- coding: utf-8 -*-
"""相对绿度健康指数(v3.13.0)用例。"""
import pytest
np = pytest.importorskip("numpy")
from laser_calibration.vision_servo import compute_green_index

def img(color, w=200, h=150):
    a = np.zeros((h, w, 3), np.uint8); a[:] = color; return a

def test_green_box_high_gray_low():
    frame = img((40, 40, 40))
    frame[40:100, 50:150] = (30, 180, 30)          # BGR 绿块
    v_green, n = compute_green_index(frame, [{"cx": 100, "cy": 70, "w": 80, "h": 50}])
    v_gray, _ = compute_green_index(frame, [{"cx": 30, "cy": 20, "w": 20, "h": 20}])
    assert n == 1 and v_green is not None and v_gray is not None
    assert v_green > 100 and v_gray < 5

def test_no_boxes_none():
    assert compute_green_index(img((0, 255, 0)), []) == (None, 0)
    assert compute_green_index(None, [{"cx": 1, "cy": 1}]) == (None, 0)

def test_mean_over_multiple_boxes():
    frame = img((40, 40, 40))
    frame[10:60, 10:60] = (30, 180, 30)
    frame[90:140, 140:190] = (30, 90, 30)          # 半绿
    v, n = compute_green_index(frame, [
        {"cx": 35, "cy": 35, "w": 40, "h": 40},
        {"cx": 165, "cy": 115, "w": 40, "h": 40}])
    assert n == 2 and 50 < v < 300

def test_degenerate_box_skipped():
    v, n = compute_green_index(img((0, 255, 0)), [{"cx": -50, "cy": -50, "w": 10, "h": 10}])
    assert v is None and n == 0
