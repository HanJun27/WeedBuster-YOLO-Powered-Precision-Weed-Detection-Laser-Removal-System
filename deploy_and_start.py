"""部署修复后的代码 → 编译 → 重启 vision_servo → 验证"""
import os, sys, time, socket, paramiko

HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"
WS = "/home/sunrise/yahboomcar_ws"
SRC = os.path.join(WS, "src/laser_calibration/laser_calibration")
FILES = ["vision_servo.py", "asr_server.py", "config.py"]

def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)

def run(chan, cmd, timeout=30, title=""):
    if title: log(title)
    log(f"  $ {cmd[:120]}")
    chan.settimeout(timeout)
    chan.exec_command(cmd)
    exit_code = chan.recv_exit_status()
    out = ""
    err = ""
    while chan.recv_ready():
        out += chan.recv(4096).decode()
    while chan.recv_stderr_ready():
        err += chan.recv_stderr(4096).decode()
    out = out.strip()
    err = err.strip()
    if out: log(f"  out> {out[:300]}")
    if err: log(f"  err> {err[:300]}")
    if exit_code != 0:
        log(f"  \u26a0 退出码={exit_code}")
    return out, err, exit_code

log("=" * 50)
log(f"连接 {HOST} ...")
sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)
sftp = t.open_sftp_client()
log("✓ SSH 连接成功")

# Step 1: 上传修复后的文件
loc = r"e:\GongZuoTai\YOLO\laser_calibration\laser_calibration"
for fn in FILES:
    local = os.path.join(loc, fn)
    remote = f"{SRC}/{fn}"
    tmp = f"/tmp/{fn}"
    sftp.put(local, tmp)
    ch = t.open_session()
    ch.settimeout(10)
    ch.exec_command(f"cp {tmp} {remote}")
    ch.close()
    log(f"  ↑ {fn}")

# Step 2: colcon build
run(t.open_session(),
    f"bash -c 'cd {WS} && source /opt/ros/humble/setup.bash && colcon build --packages-select laser_calibration 2>&1'",
    timeout=180, title="编译 laser_calibration ...")

# Step 3: 杀旧进程 + 启动 vision_servo
run(t.open_session(),
    f"bash -c 'pkill -f vision_servo 2>/dev/null; sleep 1; echo killed'",
    timeout=10, title="停止旧 vision_servo")

run(t.open_session(),
    f"bash -c 'source {WS}/install/setup.bash && nohup ros2 run laser_calibration vision_servo > ~/logs/vision_servo.log 2>&1 &'",
    timeout=5, title="启动 vision_servo ...")
time.sleep(3)

# Step 4: 检查端口
out, _, _ = run(t.open_session(),
    "ss -tlnp | grep 8093 || echo NOT_LISTENING",
    timeout=5, title="检查 8093 端口")

# Step 5: 检查 ASR 日志
out2, _, _ = run(t.open_session(),
    "tail -30 ~/logs/vision_servo.log | grep -i 'asr\\|sherpa' || echo NO_ASR_LOG",
    timeout=5, title="ASR 引擎状态")

sftp.close()
t.close()
sock.close()
log("=" * 50)
log("完成！现在可以运行端口转发后打开 8093 面板")
log("  ssh -L 8093:172.20.10.2:8093 sunrise@172.20.10.2")
log("  → 浏览器打开 http://localhost:8093")
