# -*- coding: utf-8 -*-
"""全场 RANSAC 全局关联(v3.12.0)离线用例。含 S1 场景复现。"""
from laser_calibration.vision_servo import associate_global

TOL, GATE = 20.0, 70.0

def test_s1_occluded_current_struck_near_anchor():
    """S1: 当前目标被遮挡,已打株的框挤进 anchor —— 必须拒收,不得当成当前目标。
    共识版在无旁证框时会退化(v3.11.1 README 诚实边界);全局版靠平移先验 tie-break 判对。"""
    locked = (260.0, 200.0)          # 当前目标(中心参考系)
    struck = [(200.0, 200.0)]        # 已打#1,相邻 60px
    e_true = (30.0, 0.0)             # 真实全局平移(anchor 由红斑给出≈真值)
    anchor = (locked[0] + e_true[0], locked[1] + e_true[1])
    boxes = [(200.0 + e_true[0], 200.0 + e_true[1])]   # 只有已打株的框,当前株漏检
    idx, info = associate_global(anchor, locked, boxes, struck, [], GATE, TOL)
    assert idx is None
    assert info["reason"] in ("cur_unmatched", "closer_to_other")

def test_s1_with_witness_box():
    """S1 变体:画面里另有一个旁证框(第三株待打) —— 内点计数直接定案。"""
    locked = (260.0, 200.0)
    struck = [(200.0, 200.0)]
    others = [(340.0, 260.0)]
    e = (30.0, 5.0)
    anchor = (locked[0] + e[0], locked[1] + e[1])
    boxes = [(230.0, 205.0), (370.0, 265.0)]   # 已打株框 + 旁证框;当前株漏检
    idx, info = associate_global(anchor, locked, boxes, struck, others, GATE, TOL)
    assert idx is None and info["reason"] == "cur_unmatched"

def test_normal_reacquire_with_consensus():
    """正常重捕获:三个框各归各位,当前目标框被全局一致选中。"""
    locked = (260.0, 200.0); struck = [(200.0, 200.0)]; others = [(340.0, 260.0)]
    e = (25.0, -10.0)
    anchor = (locked[0] + e[0], locked[1] + e[1])
    boxes = [(200 + e[0], 200 + e[1]), (260 + e[0] + 3, 200 + e[1] - 2),
             (340 + e[0], 260 + e[1])]
    idx, info = associate_global(anchor, locked, boxes, struck, others, GATE, TOL)
    assert idx == 1 and info["mode"] == "consensus" and info["inliers"] == 3

def test_large_residual_within_trust_region():
    """盲跳残差大(≈60px,仍在门限信任域内):anchor 先验几乎没用,
    全局版凭 3 个内点稳定选对当前目标,并报告平移修正量。
    信任域语义:|e−e0| > gate 的假设一律不采信(红斑先验的信任边界),
    残差超门限的场景本就该走 PID 超时→planner 重试重跳,不归关联层救。"""
    locked = (260.0, 200.0); others = [(340.0, 260.0), (180.0, 260.0)]
    anchor = (locked[0] + 45.0, locked[1] + 40.0)   # 先验偏 (45,40)≈60px < 门限70
    boxes = [(263.0, 201.0), (342.0, 258.0), (181.0, 262.0)]  # 真实平移≈(3,1)
    idx, info = associate_global(anchor, locked, boxes, [], others, GATE, TOL)
    assert idx == 0 and info["inliers"] == 3
    assert info["shift_corr"] > 50.0        # 报告了 ~60px 的先验修正量

def test_bad_prior_rescued_by_witness():
    """先验大偏差(≈81px>门限)但有旁证:2 内点假设压倒先验,复原正确目标。
    对照:共识版单候选规则在此场景会把旁株的框(离 anchor 22px)当成当前目标
    收下 —— 全局版是这里唯一能判对的。"""
    locked = (260.0, 200.0); others = [(340.0, 260.0)]
    anchor = (locked[0] + 60.0, locked[1] + 55.0)   # 先验偏 ≈81px
    boxes = [(263.0, 201.0), (342.0, 258.0)]        # 真实平移≈(3,1),两框各归各位
    idx, info = associate_global(anchor, locked, boxes, [], others, GATE, TOL)
    assert idx == 0 and info["inliers"] == 2 and info["mode"] == "consensus"

def test_lone_far_hypothesis_rejected():
    """信任域负例:无旁证、唯一解释需要 90px 先验修正 → 拒收(与共识版同样保守)。"""
    locked = (260.0, 200.0)
    anchor = (260.0, 200.0)
    boxes = [(350.0, 200.0)]                        # 唯一框离先验 90px
    idx, info = associate_global(anchor, locked, boxes, [], [(600.0, 50.0)],
                                 GATE, TOL)
    assert idx is None and info["reason"] == "lone_beyond_trust"

def test_lone_candidate_conservative():
    """孤立单框、无旁证:退化为保守规则 —— 门限内且不更靠近其它目标才收。"""
    locked = (260.0, 200.0); struck = [(500.0, 400.0)]
    anchor = (265.0, 203.0)
    boxes = [(268.0, 205.0)]
    idx, info = associate_global(anchor, locked, boxes, struck, [], GATE, TOL)
    assert idx == 0 and info["mode"] == "lone"

def test_empty_candidates():
    idx, info = associate_global((0, 0), (0, 0), [], [(1, 1)], [], GATE, TOL)
    assert idx is None and info["reason"] == "no_cand"
