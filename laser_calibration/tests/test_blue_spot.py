# -*- coding: utf-8 -*-
"""蓝斑检测(v3.11.2 命中判定)合成图用例。"""
import pytest
np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
from laser_calibration.vision_servo import find_blue_spot

def paper():
    return np.full((480, 640, 3), 190, np.uint8)

def test_overexposed_core_detected():
    img = paper()
    cv2.circle(img, (300, 240), 9, (255, 120, 100), -1)
    cv2.circle(img, (300, 240), 4, (255, 255, 255), -1)
    r = find_blue_spot(img, 305, 245)
    assert r is not None and abs(r[0] - 300) <= 4 and abs(r[1] - 240) <= 4

def test_plain_paper_none():
    assert find_blue_spot(paper(), 300, 240) is None

def test_red_spot_orthogonal():
    img = paper()
    cv2.circle(img, (300, 240), 9, (80, 90, 255), -1)
    cv2.circle(img, (300, 240), 4, (255, 255, 255), -1)
    assert find_blue_spot(img, 300, 240) is None

def test_roi_isolation():
    img = paper()
    cv2.circle(img, (60, 60), 9, (255, 120, 100), -1)
    assert find_blue_spot(img, 500, 400) is None

def test_elongated_reflection_rejected():
    img = paper()
    cv2.rectangle(img, (280, 236), (360, 243), (255, 120, 100), -1)
    assert find_blue_spot(img, 320, 240) is None

def test_dark_blue_rejected():
    img = np.full((480, 640, 3), 60, np.uint8)
    cv2.circle(img, (300, 240), 9, (85, 30, 20), -1)
    assert find_blue_spot(img, 300, 240) is None
