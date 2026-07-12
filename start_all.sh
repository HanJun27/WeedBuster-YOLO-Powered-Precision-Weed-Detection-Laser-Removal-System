#!/usr/bin/env bash
# ============================================================
# start_all.sh —— 一键启动完整除草系统 (v3.14.0)
# ============================================================
# 用法:
#   ./start_all.sh            # 启动所有节点（日志保存到 ~/logs/）
#   ./start_all.sh --no-driver # 不启动底盘驱动（台架调试用）
#   ./start_all.sh --no-yolo   # 不启动 YOLO 检测（调试用）
#   ./start_all.sh --no-ndvi   # 不启动 NDVI 节点
# ============================================================
# 停止:
#   ./stop_all.sh             # 停止所有节点
# ============================================================

set -e

# ── 配置 ──────────────────────────────────────────────────
WORKSPACE="/home/sunrise/yahboomcar_ws"
LOG_DIR="$HOME/logs"
ROS_DISTRO="humble"

NODES=(
    "stereo_camera:laser_calibration"
    "yolo_detector:laser_calibration"
    "vision_servo:laser_calibration"
    "strike_planner:laser_calibration"
    "chassis_controller:laser_calibration"
    "ndvi_node:laser_calibration"
    "ndvi_monitor:laser_calibration"
)
DRIVER_CMD="ros2 run yahboomcar_bringup Mcnamu_driver"

# ── 解析参数 ──────────────────────────────────────────────
NO_DRIVER=false
NO_YOLO=false
NO_NDVI=false
for arg in "$@"; do
    case "$arg" in
        --no-driver) NO_DRIVER=true ;;
        --no-yolo)   NO_YOLO=true ;;
        --no-ndvi)   NO_NDVI=true ;;
        *) echo "[start] 未知选项: $arg (忽略)" ;;
    esac
done

# ── source 环境 ───────────────────────────────────────────
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "[start] ERROR: ROS2 $ROS_DISTRO 未安装"
    exit 1
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
else
    echo "[start] WARN: 工作空间未编译 ($WORKSPACE)"
fi

# ── 日志目录 ──────────────────────────────────────────────
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +"%m%d_%H%M%S")
echo "[start] ========================================"
echo "[start]  除草系统启动  v3.14.0"
echo "[start]  日志目录: $LOG_DIR/"
echo "[start]  时间戳:   $TIMESTAMP"
echo "[start] ========================================"

# ── PID 文件 ──────────────────────────────────────────────
PID_FILE="/tmp/laser_calibration_pids.txt"
: > "$PID_FILE"

# ── 启动函数 ──────────────────────────────────────────────
start_node() {
    local name="$1"
    local pkg="$2"
    local log_file="$LOG_DIR/${name}_${TIMESTAMP}.log"
    echo "[start] 启动 $pkg/$name ..."
    nohup ros2 run "$pkg" "$name" > "$log_file" 2>&1 &
    local pid=$!
    echo "$pid" >> "$PID_FILE"
    echo "[start]   PID=$pid  日志: ${name}_${TIMESTAMP}.log"
    sleep 0.5  # 等节点初始化，避免密集启动冲突
}

# ── 1. 底盘驱动 ────────────────────────────────────
if [ "$NO_DRIVER" = false ]; then
    echo ""
    echo "[start] --- ① 底盘驱动 ---"
    local log_file="$LOG_DIR/mcnamu_driver_${TIMESTAMP}.log"
    nohup $DRIVER_CMD > "$log_file" 2>&1 &
    local pid=$!
    echo "$pid" >> "$PID_FILE"
    echo "[start] 启动 Mcnamu_driver  PID=$pid"
    sleep 1.0
else
    echo "[start] --- ① 跳过底盘驱动 (--no-driver) ---"
fi

# ── 2. 感知节点 ────────────────────────────────────
echo ""
echo "[start] --- ② 感知节点 ---"
start_node "stereo_camera" "laser_calibration"

if [ "$NO_YOLO" = false ]; then
    start_node "yolo_detector" "laser_calibration"
else
    echo "[start] 跳过 yolo_detector (--no-yolo)"
fi

# ── 3. 执行节点 ────────────────────────────────────
echo ""
echo "[start] --- ③ 执行节点 ---"
start_node "vision_servo" "laser_calibration"

# ── 4. 决策节点 ────────────────────────────────────
echo ""
echo "[start] --- ④ 决策节点 ---"
start_node "strike_planner" "laser_calibration"

# ── 5. 车控节点 ────────────────────────────────────
echo ""
echo "[start] --- ⑤ 车控节点 ---"
start_node "chassis_controller" "laser_calibration"

# ── 6. NDVI（可选） ────────────────────────────────
if [ "$NO_NDVI" = false ]; then
    echo ""
    echo "[start] --- ⑥ NDVI 节点 ---"
    start_node "ndvi_node" "laser_calibration"
    start_node "ndvi_monitor" "laser_calibration"
else
    echo "[start] --- ⑥ 跳过 NDVI (--no-ndvi) ---"
fi

# ── 完成 ──────────────────────────────────────────
echo ""
echo "[start] ========================================"
echo "[start]  所有节点已启动！"
echo "[start] ========================================"
echo "[start]  8093 面板: http://172.20.10.2:8093"
echo "[start]  视频流:    http://172.20.10.2:8080"
echo "[start]  停止:      ./stop_all.sh"
echo "[start]  日志:      ls $LOG_DIR/"
echo "[start] ========================================"
