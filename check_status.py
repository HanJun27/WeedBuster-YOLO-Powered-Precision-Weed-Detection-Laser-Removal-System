"""检查 8093 端口和 ASR 状态"""
import socket, paramiko, time

HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"

sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)

# Check port 8093
ch = t.open_session()
ch.settimeout(5)
ch.exec_command("ss -tlnp | grep 8093")
time.sleep(1)
out = b""
while ch.recv_ready():
    out += ch.recv(4096)
err = b""
while ch.recv_stderr_ready():
    err += ch.recv_stderr(4096)
print("=== 8093 端口 ===")
print(out.decode())
print(err.decode())
print(f"exit: {ch.recv_exit_status()}")

# Check ASR in vision_servo log
ch2 = t.open_session()
ch2.settimeout(5)
ch2.exec_command("grep -i 'asr\\|sherpa\\|8093\\|http' ~/logs/vision_servo.log")
time.sleep(1)
out2 = b""
while ch2.recv_ready():
    out2 += ch2.recv(4096)
print("=== ASR/HTTP 日志 ===")
print(out2.decode())

# Check if all processes running
ch3 = t.open_session()
ch3.settimeout(5)
ch3.exec_command("ps aux | grep 'ros2 run' | grep -v grep | wc -l")
time.sleep(1)
out3 = b""
while ch3.recv_ready():
    out3 += ch3.recv(4096)
print(f"=== 运行中 ROS2 进程数: {out3.decode().strip()} ===")

t.close()
sock.close()
