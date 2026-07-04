#!/usr/bin/env python3
"""
ndvi_monitor.py —— NDVI 时序健康监测  v3.10.1
================================================
订阅 ndvi_node 发布的 /ndvi/result，对植物健康做长期时序追踪。

设计原则（与 ndvi_node 解耦）：
  - 本节点只订阅 /ndvi/result topic，不依赖 ndvi_node 内部实现
  - ndvi_node 故障不影响本节点已存的历史数据
  - 历史数据持久化到 ~/ndvi_history.json，节点重启不丢失

核心功能：
  1. 快照记录：手动点按钮 / 自动定时，记录当前 NDVI 统计为一条带日期的记录
  2. 动态基线：第一条快照设为基线 100%，后续按相对百分比表达
     —— 这是相对 NDVI 的正确用法：不纠结绝对值，只看相对变化趋势
  3. 趋势分析：对历史 mean_ndvi 做线性回归，输出斜率 + 健康判定
  4. 预警：连续 N 天下降触发"亚健康预警"
  5. 网页可视化：时序曲线图 + 趋势结论 + 健康分级堆叠

★ 时序可比的前提（务必保证）：
  - 每次拍摄用同一套设备、同一 ISP 锁定参数
  - 每次（或每天）用 calib_diffuse 重新标定，或确认标定未漂移
  - 尽量同一植物、同一视角、同一工作距离
  主动光场方案天然满足"光照一致"，比纯被动方案更适合时序监测。

运行：
    ros2 run laser_calibration ndvi_monitor
前置：
    ros2 run laser_calibration stereo_camera
    ros2 run laser_calibration ndvi_node

浏览器：
    http://localhost:8095         (本机)
    http://<小车IP>:8095           (远程)
"""

import datetime
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from laser_calibration.config import (
    NDVI_HISTORY_FILE,
    NDVI_TREND_DECLINE, NDVI_TREND_IMPROVE,
    NDVI_AUTO_SNAPSHOT_HOURS,
    NDVI_ALERT_DECLINE_DAYS,
)

MONITOR_HTTP_PORT = 8095
NDVI_RESULT_TOPIC = "/ndvi/result"


# ══════════════════════════════════════════════════════════════
#  历史数据存取
# ══════════════════════════════════════════════════════════════
def load_history() -> list:
    """读取历史快照列表（按日期排序）。文件不存在返回空列表。"""
    if not os.path.exists(NDVI_HISTORY_FILE):
        return []
    try:
        with open(NDVI_HISTORY_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return sorted(data, key=lambda e: e.get("datetime", ""))
    except Exception:
        pass
    return []


def save_history(history: list):
    """写回历史快照列表。"""
    try:
        with open(NDVI_HISTORY_FILE, "w", encoding="utf-8") as f:
            json.dump(history, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[ndvi_monitor] 历史写入失败：{e}")


# ══════════════════════════════════════════════════════════════
#  趋势分析
# ══════════════════════════════════════════════════════════════
def linear_regression(xs: list, ys: list):
    """最小二乘线性回归，返回 (slope, intercept)。点数 < 2 返回 (0, 均值)。"""
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0)
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-9:
        return 0.0, sy / n
    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n
    return slope, intercept


def analyze_trend(history: list) -> dict:
    """
    对历史快照做趋势分析。
    返回 dict: baseline, points(相对百分比序列), slope, verdict, alert
    """
    if not history:
        return {
            "n": 0, "baseline": None, "points": [],
            "slope": 0.0, "verdict": "无数据", "alert": False,
        }

    baseline = history[0].get("global_ndvi", 0.0)
    if abs(baseline) < 1e-6:
        baseline = 1e-6  # 防除零

    # 每条快照换算成相对基线的百分比
    points = []
    for i, entry in enumerate(history):
        gndvi = entry.get("global_ndvi", 0.0)
        pct = gndvi / baseline * 100.0
        points.append({
            "index": i,
            "date": entry.get("date", ""),
            "global_ndvi": round(gndvi, 4),
            "pct": round(pct, 1),
            "health": entry.get("health_stats", {}),
            "note": entry.get("note", ""),
        })

    # 线性回归（x=天序号, y=相对百分比）
    xs = [p["index"] for p in points]
    ys = [p["pct"] for p in points]
    slope, _ = linear_regression(xs, ys)

    # 趋势判定
    if len(points) < 2:
        verdict = "数据不足（至少 2 个快照）"
    elif slope <= NDVI_TREND_DECLINE:
        verdict = "健康下降"
    elif slope >= NDVI_TREND_IMPROVE:
        verdict = "健康改善"
    else:
        verdict = "基本稳定"

    # 连续下降预警：最近 N 个点是否每个都比前一个低
    alert = False
    if len(points) > NDVI_ALERT_DECLINE_DAYS:
        recent = points[-(NDVI_ALERT_DECLINE_DAYS + 1):]
        declining = all(
            recent[i]["pct"] < recent[i - 1]["pct"]
            for i in range(1, len(recent))
        )
        alert = declining

    return {
        "n": len(points),
        "baseline": round(baseline, 4),
        "points": points,
        "slope": round(slope, 3),
        "verdict": verdict,
        "alert": alert,
    }


# ══════════════════════════════════════════════════════════════
#  网页 HTML（时序曲线 + 趋势结论）
# ══════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>NDVI 时序健康监测 v3.10.1</title>
<style>
  body { background:#111; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1 { color:#0f0; margin:0 0 12px; font-size:18px; }
  .panel { background:#1a1a1a; padding:12px; border-radius:8px; margin-bottom:12px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:7px 16px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn-danger { color:#f55; border-color:#f55; }
  .btn-danger:hover { background:#f55; color:#000; }
  .row { display:flex; gap:14px; flex-wrap:wrap; align-items:flex-start; }
  canvas { background:#0a0a0a; border:1px solid #333; border-radius:4px; }
  .verdict { font-size:22px; font-weight:bold; padding:8px 0; }
  .verdict.up { color:#0f0; }
  .verdict.stable { color:#fa0; }
  .verdict.down { color:#f55; }
  .verdict.none { color:#888; }
  .stat-row { display:flex; justify-content:space-between; font-size:13px;
              border-bottom:1px dotted #333; padding:3px 0; }
  .k { color:#888; }
  .v { color:#0f0; }
  .alert-box { background:#3a0808; border:1px solid #f55; color:#f55;
               padding:10px; border-radius:6px; margin-bottom:12px;
               font-size:14px; display:none; }
  table { border-collapse:collapse; font-size:12px; width:100%; }
  th, td { border:1px solid #333; padding:4px 8px; text-align:right; }
  th { color:#888; }
  td.date { text-align:left; color:#aaa; }
  .info { color:#888; font-size:12px; margin-top:8px; line-height:1.7; }
  .status { color:#fa0; min-height:18px; padding:6px 0; }
</style>
</head>
<body>
<h1>📈 NDVI 时序健康监测 v3.10.1（动态基线追踪）</h1>

<div class="alert-box" id="alert">
  ⚠️ 亚健康预警：检测到连续多日 NDVI 下降趋势，建议人工核查植物状态。
</div>

<div class="panel">
  <button class="btn" onclick="snapshot()">📸 记录今日快照</button>
  <button class="btn" onclick="refresh()">🔄 刷新</button>
  <button class="btn btn-danger" onclick="delLast()">删除最后一条</button>
  <button class="btn btn-danger" onclick="clearAll()">清空全部</button>
  <div class="status" id="status">就绪。对准植物后点【记录今日快照】。</div>
</div>

<div class="row">
  <div class="panel">
    <div style="color:#888;font-size:13px;margin-bottom:6px">健康趋势曲线（相对基线 %）</div>
    <canvas id="chart" width="560" height="320"></canvas>
  </div>
  <div class="panel" style="min-width:260px">
    <div style="color:#888;font-size:13px">趋势结论</div>
    <div class="verdict none" id="verdict">无数据</div>
    <div class="stat-row"><span class="k">快照数</span><span class="v" id="s-n">0</span></div>
    <div class="stat-row"><span class="k">基线 NDVI</span><span class="v" id="s-base">--</span></div>
    <div class="stat-row"><span class="k">最新 NDVI</span><span class="v" id="s-last">--</span></div>
    <div class="stat-row"><span class="k">相对基线</span><span class="v" id="s-pct">--</span></div>
    <div class="stat-row"><span class="k">趋势斜率</span><span class="v" id="s-slope">--</span></div>
    <div class="stat-row"><span class="k">当前 NDVI 模式</span><span class="v" id="s-mode">--</span></div>
    <div class="info">
      斜率 = 相对基线百分比每天变化量<br>
      &lt; -3%/天 → 健康下降<br>
      &gt; +3%/天 → 健康改善<br>
      之间 → 基本稳定
    </div>
  </div>
</div>

<div class="panel">
  <div style="color:#888;font-size:13px;margin-bottom:6px">历史快照记录</div>
  <table id="histtable">
    <thead><tr>
      <th class="date">日期时间</th><th>NDVI</th><th>相对基线%</th>
      <th>健康%</th><th>亚健康%</th><th>枯萎%</th><th>非植物%</th><th>备注</th>
    </tr></thead>
    <tbody id="histbody"></tbody>
  </table>
</div>

<div class="info">
  <b>使用说明</b>：每天同一时间，对准同一植物，点【记录今日快照】一次。<br>
  连续记录数天后，曲线会显示该植物的健康趋势。<br>
  <b>加速演示</b>：若来不及等多天，可剪一段枝条任其失水，每隔 2~3 小时记录一次，
  数小时内即可得到一条真实的"下降"趋势曲线（剪下的枝条确实会因失水导致 NDVI 下降）。
</div>

<script>
const cv = document.getElementById('chart');
const ctx = cv.getContext('2d');

function drawChart(points) {
  ctx.clearRect(0, 0, cv.width, cv.height);
  const W = cv.width, H = cv.height;
  const padL = 46, padR = 16, padT = 16, padB = 36;
  const plotW = W - padL - padR, plotH = H - padT - padB;

  // 坐标轴
  ctx.strokeStyle = '#444';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + plotH);
  ctx.lineTo(padL + plotW, padT + plotH);
  ctx.stroke();

  if (!points || points.length === 0) {
    ctx.fillStyle = '#666';
    ctx.font = '13px monospace';
    ctx.fillText('暂无数据，点【记录今日快照】开始', padL + 30, padT + plotH / 2);
    return;
  }

  // Y 轴范围：相对百分比，固定 0~140 或自适应
  let maxPct = 110, minPct = 60;
  points.forEach(p => {
    if (p.pct > maxPct) maxPct = p.pct;
    if (p.pct < minPct) minPct = p.pct;
  });
  maxPct = Math.ceil(maxPct / 10) * 10;
  minPct = Math.floor(minPct / 10) * 10;
  const range = Math.max(1, maxPct - minPct);

  // Y 网格 + 标签
  ctx.fillStyle = '#888';
  ctx.font = '11px monospace';
  for (let v = minPct; v <= maxPct; v += 10) {
    const y = padT + plotH - (v - minPct) / range * plotH;
    ctx.strokeStyle = (v === 100) ? '#0a0' : '#222';
    ctx.beginPath();
    ctx.moveTo(padL, y); ctx.lineTo(padL + plotW, y);
    ctx.stroke();
    ctx.fillText(v + '%', 6, y + 4);
  }

  // 100% 基线标注
  const y100 = padT + plotH - (100 - minPct) / range * plotH;
  ctx.fillStyle = '#0a0';
  ctx.fillText('基线', padL + plotW - 36, y100 - 4);

  // 数据点 + 折线
  const n = points.length;
  const dx = n > 1 ? plotW / (n - 1) : 0;
  const px = (i) => padL + (n > 1 ? i * dx : plotW / 2);
  const py = (pct) => padT + plotH - (pct - minPct) / range * plotH;

  // 折线
  ctx.strokeStyle = '#0cf';
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((p, i) => {
    const x = px(i), y = py(p.pct);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // 数据点
  points.forEach((p, i) => {
    const x = px(i), y = py(p.pct);
    let color = '#0cf';
    if (p.pct >= 103) color = '#0f0';
    else if (p.pct <= 97) color = '#f55';
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fill();
    // X 轴日期标签（稀疏显示）
    if (n <= 10 || i % Math.ceil(n / 8) === 0) {
      ctx.fillStyle = '#888';
      ctx.save();
      ctx.translate(x, padT + plotH + 8);
      ctx.rotate(-Math.PI / 5);
      ctx.font = '10px monospace';
      ctx.fillText((p.date || '').slice(5), 0, 6);
      ctx.restore();
    }
  });
}

async function refresh() {
  try {
    const r = await fetch('/api/analyze');
    const d = await r.json();
    drawChart(d.points);

    // 趋势结论
    const ve = document.getElementById('verdict');
    ve.textContent = d.verdict;
    ve.className = 'verdict ' +
      (d.verdict.includes('改善') ? 'up' :
       d.verdict.includes('下降') ? 'down' :
       d.verdict.includes('稳定') ? 'stable' : 'none');

    document.getElementById('s-n').textContent = d.n;
    document.getElementById('s-base').textContent =
      d.baseline != null ? d.baseline.toFixed(3) : '--';
    document.getElementById('s-slope').textContent =
      d.slope != null ? (d.slope > 0 ? '+' : '') + d.slope.toFixed(2) + ' %/天' : '--';

    if (d.points && d.points.length) {
      const last = d.points[d.points.length - 1];
      document.getElementById('s-last').textContent = last.global_ndvi.toFixed(3);
      document.getElementById('s-pct').textContent = last.pct.toFixed(1) + '%';
    } else {
      document.getElementById('s-last').textContent = '--';
      document.getElementById('s-pct').textContent = '--';
    }

    document.getElementById('alert').style.display = d.alert ? 'block' : 'none';

    // 历史表格
    const tb = document.getElementById('histbody');
    tb.innerHTML = '';
    (d.points || []).forEach(p => {
      const h = p.health || {};
      const tot = (h.healthy||0)+(h.moderate||0)+(h.stressed||0)+(h.non_plant||0)+1e-9;
      const pc = (v) => ((v||0)/tot*100).toFixed(0);
      const tr = document.createElement('tr');
      tr.innerHTML =
        `<td class="date">${p.date}</td>` +
        `<td>${p.global_ndvi.toFixed(3)}</td>` +
        `<td>${p.pct.toFixed(1)}</td>` +
        `<td>${pc(h.healthy)}</td><td>${pc(h.moderate)}</td>` +
        `<td>${pc(h.stressed)}</td><td>${pc(h.non_plant)}</td>` +
        `<td class="date">${p.note||''}</td>`;
      tb.appendChild(tr);
    });
  } catch (e) {
    setStatus('刷新失败：' + e);
  }
}

async function snapshot() {
  setStatus('记录中...');
  try {
    const r = await fetch('/api/snapshot');
    const d = await r.json();
    if (d.ok) {
      setStatus(`✅ 已记录 ${d.date} | NDVI=${d.global_ndvi.toFixed(3)} | 模式=${d.ndvi_mode}`);
      document.getElementById('s-mode').textContent = d.ndvi_mode;
      refresh();
    } else {
      setStatus('❌ ' + (d.err || '记录失败'));
    }
  } catch (e) {
    setStatus('❌ 记录失败：' + e);
  }
}

async function delLast() {
  if (!confirm('删除最后一条快照记录？')) return;
  await fetch('/api/delete_last');
  refresh();
  setStatus('已删除最后一条');
}

async function clearAll() {
  if (!confirm('确认清空全部历史？不可恢复！')) return;
  await fetch('/api/clear');
  refresh();
  setStatus('历史已清空');
}

function setStatus(msg) {
  document.getElementById('status').textContent = msg;
}

setInterval(refresh, 5000);
refresh();
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class NDVIMonitorNode(Node):
    def __init__(self):
        super().__init__("ndvi_monitor")

        # 最近一次 /ndvi/result 缓存
        self._latest = None
        self._latest_lock = threading.Lock()

        # 历史数据
        self._history = load_history()
        self._hist_lock = threading.Lock()

        # 订阅 NDVI 结果（与 ndvi_node 解耦，只读 topic）
        self.sub = self.create_subscription(
            String, NDVI_RESULT_TOPIC, self._cb_result, 10)

        # 自动快照定时器（可选）
        if NDVI_AUTO_SNAPSHOT_HOURS > 0:
            period = NDVI_AUTO_SNAPSHOT_HOURS * 3600.0
            self.create_timer(period, self._auto_snapshot)

        self._start_http()

        log = self.get_logger().info
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log("  NDVI 时序健康监测节点  v3.10.1")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        log(f"  订阅:       {NDVI_RESULT_TOPIC}")
        log(f"  历史文件:   {NDVI_HISTORY_FILE}")
        log(f"  已有快照:   {len(self._history)} 条")
        if NDVI_AUTO_SNAPSHOT_HOURS > 0:
            log(f"  自动快照:   每 {NDVI_AUTO_SNAPSHOT_HOURS} 小时一次")
        else:
            log("  自动快照:   关闭（仅手动）")
        log(f"  HTTP 端口:  {MONITOR_HTTP_PORT}")
        log(f"  本机访问:   http://localhost:{MONITOR_HTTP_PORT}")
        log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    # ── 回调 ────────────────────────────────────────────────
    def _cb_result(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._latest_lock:
                self._latest = data
        except Exception as e:
            self.get_logger().error(f"/ndvi/result 解析失败：{e}")

    # ── 记录一条快照 ────────────────────────────────────────
    def take_snapshot(self, note: str = "manual"):
        """把当前最新的 NDVI 统计记录成一条带时间戳的历史条目。"""
        with self._latest_lock:
            latest = self._latest

        if latest is None:
            return False, "尚未收到 /ndvi/result 数据，确认 ndvi_node 在运行"

        now = datetime.datetime.now()
        entry = {
            "datetime":    now.isoformat(timespec="seconds"),
            "date":        now.strftime("%Y-%m-%d %H:%M"),
            "global_ndvi": float(latest.get("global_ndvi", 0.0)),
            "plant_ratio": float(latest.get("plant_ratio", 0.0)),
            "ndvi_mode":   latest.get("ndvi_mode", "?"),
            "health_stats": latest.get("health_stats", {}),
            "note":        note,
        }

        with self._hist_lock:
            self._history.append(entry)
            self._history.sort(key=lambda e: e.get("datetime", ""))
            save_history(self._history)

        self.get_logger().info(
            f"📸 快照记录: {entry['date']} | "
            f"NDVI={entry['global_ndvi']:.3f} | "
            f"模式={entry['ndvi_mode']} | note={note}"
        )
        return True, entry

    def _auto_snapshot(self):
        ok, _ = self.take_snapshot(note="auto")
        if not ok:
            self.get_logger().warn("自动快照失败：暂无 NDVI 数据")

    # ── HTTP 服务 ───────────────────────────────────────────
    def _start_http(self):
        ref = self

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _json(self, data, code=200):
                body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type",
                                 "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                p = self.path.split("?", 1)[0]

                if p == "/" or p == "/index.html":
                    body = HTML_PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                # 记录快照
                if p == "/api/snapshot":
                    ok, result = ref.take_snapshot("manual")
                    if ok:
                        self._json({
                            "ok": True,
                            "date": result["date"],
                            "global_ndvi": result["global_ndvi"],
                            "ndvi_mode": result["ndvi_mode"],
                        })
                    else:
                        self._json({"ok": False, "err": result}, 400)
                    return

                # 趋势分析
                if p == "/api/analyze":
                    with ref._hist_lock:
                        hist = list(ref._history)
                    self._json(analyze_trend(hist))
                    return

                # 原始历史
                if p == "/api/history":
                    with ref._hist_lock:
                        self._json(list(ref._history))
                    return

                # 删除最后一条
                if p == "/api/delete_last":
                    with ref._hist_lock:
                        if ref._history:
                            ref._history.pop()
                            save_history(ref._history)
                    self._json({"ok": True})
                    return

                # 清空
                if p == "/api/clear":
                    with ref._hist_lock:
                        ref._history = []
                        save_history(ref._history)
                    self._json({"ok": True})
                    return

                self.send_response(404)
                self.end_headers()

        def serve():
            HTTPServer(("0.0.0.0", MONITOR_HTTP_PORT),
                       _Handler).serve_forever()

        threading.Thread(target=serve, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = NDVIMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
