"""检查 vision_servo 日志"""
import os, sys, time, socket, paramiko

HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"

sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)

ch = t.open_session()
ch.settimeout(10)
ch.exec_command("cat ~/logs/vision_servo.log")
out = ch.recv_exit_status()
data = b""
while ch.recv_ready():
    data += ch.recv(4096)
while ch.recv_stderr_ready():
    data += ch.recv_stderr(4096)
print(data.decode(errors="replace")[-3000:])
ch.close()

# Also check what processes are running
ch2 = t.open_session()
ch2.settimeout(10)
ch2.exec_command("ps aux | grep -E 'ros2|vision|servo' | grep -v grep")
print("\n=== 运行进程 ===")
out2 = b""
while ch2.recv_ready():
    out2 += ch2.recv(4096)
print(out2.decode(errors="replace"))

t.close()
sock.close()
