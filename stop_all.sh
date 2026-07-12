#!/usr/bin/env bash
# ============================================================
# stop_all.sh —— 停止所有除草系统节点
# ============================================================
# 用法:
#   ./stop_all.sh             # 停止所有节点
#   ./stop_all.sh --force     # kill -9 强制停止
# ============================================================

FORCE=""
if [ "$1" = "--force" ]; then
    FORCE="-9"
fi

echo "[stop] ========================================"
echo "[stop]  停止除草系统..."
echo "[stop] ========================================"

# 方法1: 从 PID 文件读取
PID_FILE="/tmp/laser_calibration_pids.txt"
if [ -f "$PID_FILE" ]; then
    while IFS= read -r pid; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "[stop]  停止 PID=$pid"
            kill $FORCE "$pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
fi

# 方法2: 按名称杀（兜底，确保杀干净）
for proc in \
    "Mcnamu_driver" \
    "stereo_camera" \
    "yolo_detector" \
    "vision_servo" \
    "strike_planner" \
    "chassis_controller" \
    "ndvi_node" \
    "ndvi_monitor"; do
    pids=$(pgrep -f "ros2.*$proc" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo "[stop]  按名称杀: $proc (PIDs: $(echo $pids | tr '\n' ' '))"
        kill $FORCE $pids 2>/dev/null || true
    fi
done

# 也杀残留的 laser_calibration 进程
pids=$(pgrep -f "laser_calibration" 2>/dev/null || true)
if [ -n "$pids" ]; then
    echo "[stop]  杀残留 laser_calibration 进程"
    kill $FORCE $pids 2>/dev/null || true
fi

sleep 0.5
echo "[stop]  完成"
echo "[stop] ========================================"
