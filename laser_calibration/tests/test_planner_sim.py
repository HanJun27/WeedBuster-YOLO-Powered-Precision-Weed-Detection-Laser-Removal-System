# -*- coding: utf-8 -*-
"""strike_planner v0.6 端到端仿真(stub rclpy)。"""
import json, sys, types
import pytest

class _Logger:
    def info(self, s): pass
    def warn(self, s): pass
    def error(self, s): pass

class _Pub:
    def __init__(self, topic): self.topic, self.msgs = topic, []
    def publish(self, m): self.msgs.append(m)

class _Node:
    def __init__(self, name): self._pubs = {}
    def create_subscription(self, t, topic, cb, q): return (topic, cb)
    def create_publisher(self, t, topic, q):
        p = _Pub(topic); self._pubs[topic] = p; return p
    def create_timer(self, period, cb): return None
    def get_logger(self): return _Logger()

class String: data = ""
class Empty: pass

@pytest.fixture()
def planner(monkeypatch):
    fake_node = types.ModuleType("rclpy.node"); fake_node.Node = _Node
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_node)
    fs = types.ModuleType("std_msgs.msg"); fs.String, fs.Empty = String, Empty
    monkeypatch.setitem(sys.modules, "std_msgs.msg", fs)
    for m in list(sys.modules):
        if m.startswith("laser_calibration.strike_planner"):
            del sys.modules[m]
    import laser_calibration.strike_planner as sp
    n = sp.StrikePlanner()
    return sp, n

def _yolo(n, boxes, fid):
    m = String()
    m.data = json.dumps({"detected": bool(boxes), "frame_id": fid,
                         "boxes": boxes})
    n._cb_yolo(m)

def _box(x, y):
    return {"cx": x, "cy": y, "w": 40, "h": 40,
            "confidence": 0.9, "label": "weed"}

def _result(n, rid, **kw):
    m = String(); m.data = json.dumps({"id": rid, **kw}); n._cb_strike_result(m)

def test_full_session_with_hits(planner):
    sp, n = planner
    n._cb_start_clearing(Empty()); n._recenter_done_at = 0.0
    _yolo(n, [_box(200, 200), _box(420, 300)], 1); n._tick()
    assert n.state == "VOTING"
    _yolo(n, [_box(201, 199), _box(421, 301)], 2)
    _yolo(n, [_box(199, 201), _box(419, 299), _box(100, 100)], 3)  # 闪烁簇
    _yolo(n, [_box(200, 200), _box(420, 300)], 4)
    n._vote_until = 0.0; n._tick()
    assert n.state == "WAIT_RESULT" and n._session_total == 2
    _result(n, n._current["id"], result="success",
            hit=True, hit_distance=4.2, hit_frames=9)
    assert n._struck[0]["hit"] is True
    n._tick()
    _result(n, n._current["id"], result="success")     # 旧版执行层无命中字段
    n._tick()
    assert n.state == "IDLE"
    pc = json.loads(n._pubs["/planner/patch_clear"].msgs[-1].data)
    assert pc["cleared"] == 2 and pc["hit_confirmed"] == 1 and pc["hit_unseen"] == 1
    sess = json.loads(n._pubs["/planner/session_state"].msgs[-1].data)
    assert sess["state"] == "IDLE" and sess["hits"] == 1

def test_retry_then_giveup(planner):
    sp, n = planner
    n._cb_start_clearing(Empty()); n._recenter_done_at = 0.0
    _yolo(n, [_box(320, 240)], 1); n._tick()
    _yolo(n, [_box(320, 240)], 2); _yolo(n, [_box(320, 240)], 3)
    n._vote_until = 0.0; n._tick()
    rid = n._current["id"]
    _result(n, rid, result="failed"); n._tick()
    _result(n, rid, result="failed"); n._tick()
    pc = json.loads(n._pubs["/planner/patch_clear"].msgs[-1].data)
    assert pc["failed"] == 1 and pc["cleared"] == 0

def test_strike_cmd_carries_context(planner):
    sp, n = planner
    n._cb_start_clearing(Empty()); n._recenter_done_at = 0.0
    _yolo(n, [_box(200, 200), _box(420, 300)], 1); n._tick()
    _yolo(n, [_box(200, 200), _box(420, 300)], 2)
    _yolo(n, [_box(200, 200), _box(420, 300)], 3)
    n._vote_until = 0.0; n._tick()
    cmd = json.loads(n._pubs["/servo/strike_cmd"].msgs[0].data)
    assert "exclude" in cmd and "others" in cmd and len(cmd["others"]) == 1
