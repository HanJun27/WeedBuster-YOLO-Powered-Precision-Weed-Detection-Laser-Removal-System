# -*- coding: utf-8 -*-
from laser_calibration.vision_servo import compute_shot_stats

def test_empty():
    st = compute_shot_stats([])
    assert st["shots"] == 0 and st["success_rate"] is None

def test_mixed():
    shots = [
        {"result": "success", "hit": True,  "hit_dist": 4.0,
         "final_distance": 2.0, "duration": 5.0},
        {"result": "success", "hit": False, "hit_dist": 40.0,
         "final_distance": 3.0, "duration": 6.0},
        {"result": "success", "hit": None,  "hit_dist": None,
         "final_distance": 4.0, "duration": 7.0},
        {"result": "failed",  "hit": None,  "hit_dist": None,
         "final_distance": None, "duration": 9.0},
    ]
    st = compute_shot_stats(shots)
    assert st["shots"] == 4 and st["success"] == 3 and st["failed"] == 1
    assert st["success_rate"] == 0.75
    assert st["hit"] == 1 and st["miss"] == 1 and st["unjudged"] == 1
    assert st["hit_rate"] == 0.5
    assert st["avg_lock_err"] == 3.0          # 只统计成功发
    assert st["avg_hit_dist"] == 22.0         # 只统计有判定发
    assert st["avg_duration"] == 6.0          # 失败发不计耗时
