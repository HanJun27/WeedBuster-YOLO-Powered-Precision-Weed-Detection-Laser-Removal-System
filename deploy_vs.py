"""上传 vision_servo.py → 编译 → 重启"""
import socket, paramiko, time

HOST, USER, PASS = "172.20.10.2", "sunrise", "yahboom"
WS = "/home/sunrise/yahboomcar_ws"
SRC = WS + "/src/laser_calibration/laser_calibration"

sock = socket.socket()
sock.settimeout(15)
sock.connect((HOST, 22))
t = paramiko.Transport(sock)
t.connect()
t.auth_password(USER, PASS)
sftp = t.open_sftp_client()

# upload
local = r"e:\GongZuoTai\YOLO\laser_calibration\laser_calibration\vision_servo.py"
tmp = "/tmp/vision_servo.py"
sftp.put(local, tmp)
sftp.close()
ch = t.open_session()
ch.settimeout(10)
ch.exec_command("cp " + tmp + " " + SRC + "/vision_servo.py")
ch.close()
print("Upload OK")

# build
ch = t.open_session()
ch.settimeout(180)
ch.exec_command("bash -c 'cd " + WS + " && source /opt/ros/humble/setup.bash && colcon build --packages-select laser_calibration 2>&1'")
out = b""
while ch.recv_ready():
    out += ch.recv(4096)
print(out.decode()[-500:])
print("Build exit:", ch.recv_exit_status())
ch.close()

# kill + restart
ch = t.open_session()
ch.settimeout(10)
ch.exec_command("pkill -f vision_servo 2>/dev/null; sleep 1")
ch.close()
time.sleep(1)
ch = t.open_session()
ch.settimeout(10)
ch.exec_command("bash -c 'source " + WS + "/install/setup.bash && nohup ros2 run laser_calibration vision_servo > ~/logs/vision_servo.log 2>&1 &'")
ch.close()
t.close()
sock.close()
print("vision_servo restarted!")
