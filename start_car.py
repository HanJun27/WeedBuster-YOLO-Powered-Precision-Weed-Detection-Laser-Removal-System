import paramiko, socket, time, sys
sys.stdout = open(1, "w", buffering=1)  # line-buffered
HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"

sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)

def run(cmd, timeout=10):
    c = t.open_session()
    c.settimeout(timeout)
    c.exec_command(cmd)
    time.sleep(1.5)
    out = b""
    while c.recv_ready():
        out += c.recv(4096)
    c.close()
    return out.decode().strip()

# Step 1: mkdir + kill old
print("=== 初始化 ===")
print(run("mkdir -p ~/logs && pkill -f ros2 2>/dev/null && sleep 1 && echo OK"))

# Step 2: start each node
BASE = 'bash -c ". /opt/ros/humble/setup.bash && . ~/yahboomcar_ws/install/setup.bash && nohup ros2 run laser_calibration'
for node in ["stereo_camera", "vision_servo", "strike_planner", "chassis_controller"]:
    time.sleep(1)
    cmd = f'{BASE} {node} > ~/logs/{node}.log 2>&1 &"'
    run(cmd, 3)
    print(f"  {node}: started")

# Step 3: verify
time.sleep(3)
procs = run("ps aux | grep ros2 | grep -v grep")
print("\n=== 运行中节点 ===")
if procs:
    for line in procs.split("\n"):
        print(f"  {line.strip()}")
else:
    print("  (none)")

t.close()
sock.close()
