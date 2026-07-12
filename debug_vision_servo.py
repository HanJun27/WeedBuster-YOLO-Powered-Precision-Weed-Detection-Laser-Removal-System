"""在车上跑 vision_servo 看完整错误"""
import socket, paramiko, time

HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"

sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)

# Run vision_servo in foreground for 5 seconds to capture errors
ch = t.open_session()
ch.settimeout(12)
ch.exec_command(
    'bash -c "source /home/sunrise/yahboomcar_ws/install/setup.bash '
    '&& timeout 5 ros2 run laser_calibration vision_servo 2>&1"'
)
time.sleep(6)
out = b""
while ch.recv_ready():
    out += ch.recv(4096)
while ch.recv_stderr_ready():
    out += ch.recv_stderr(4096)
print(out.decode(errors="replace")[-5000:])
print(f"\n=== exit code: {ch.recv_exit_status()}")
ch.close()
t.close()
sock.close()
