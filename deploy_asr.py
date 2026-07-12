#!/usr/bin/env python3
"""用 paramiko 部署 ASR 代码到小车并测试。"""
import os
import sys
import paramiko

HOST = "172.20.10.2"
USER = "sunrise"
PASS = "yahboom"
PORT = 22
WORKSPACE = "/home/sunrise/yahboomcar_ws"
PKG = "laser_calibration"

# 本地源码根目录
PKG_SRC = os.path.join(os.path.dirname(__file__), "laser_calibration", "laser_calibration")

# 需要上传的文件（本地路径 → 远程路径）
def _remote(path):
    """确保远程路径使用正斜杠。"""
    return f"/home/sunrise/yahboomcar_ws/src/laser_calibration/laser_calibration/{path}"


FILES = {
    os.path.join(PKG_SRC, "asr_server.py"):       _remote("asr_server.py"),
    os.path.join(PKG_SRC, "config.py"):            _remote("config.py"),
    os.path.join(PKG_SRC, "vision_servo.py"):      _remote("vision_servo.py"),
}

# ASR 模型文件（上传到 ~/laser_calibration/asr_models/paraformer-small/）
MODEL_DIR = os.path.join(os.path.dirname(__file__), "paraformer-small")
MODEL_REMOTE_BASE = "/home/sunrise/laser_calibration/asr_models/paraformer-small"
MODEL_FILES = [
    "model.int8.onnx",
    "tokens.txt",
    "config.yaml",
    "am.mvn",
]


def ssh_exec(client, command, timeout=30):
    """执行远程命令并返回 stdout。"""
    print(f"\n[SSH] $ {command[:100]}...")
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout, get_pty=False)
    exit_code = stdout.channel.recv_exit_status()
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()
    if out:
        print(f"  OUT: {out[:500]}")
    if err:
        print(f"  ERR: {err[:300]}")
    if exit_code != 0:
        print(f"  RC={exit_code}")
    return exit_code, out, err


def main():
    print("=" * 50)
    print("连接小车...")
    print("=" * 50)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(HOST, PORT, USER, PASS, timeout=10, look_for_keys=False,
                       allow_agent=False)
    except Exception as e:
        print(f"[FAIL] 连接失败: {e}")
        return 1
    print("[PASS] SSH 连接成功")
    print(f"  Host: {HOST}")
    print(f"  User: {USER}")

    # 1. 检查系统信息
    print("\n=== 1. 系统信息 ===")
    ssh_exec(client, "hostname && uname -a")

    # 2. 检查当前版本
    print("\n=== 2. 当前版本 ===")
    ssh_exec(client,
             f"bash -c 'source {WORKSPACE}/install/setup.bash 2>/dev/null; "
             f"python3 -c \"import laser_calibration; print(laser_calibration.__version__)\"'",
             timeout=10)

    # 3. 上传文件
    print("\n=== 3. 上传文件 ===")
    sftp = client.open_sftp()
    for local, remote in FILES.items():
        if not os.path.isfile(local):
            print(f"  [SKIP] 本地文件不存在: {local}")
            continue
        remote_dir = os.path.dirname(remote)
        try:
            sftp.stat(remote_dir)
        except FileNotFoundError:
            ssh_exec(client, f"mkdir -p {remote_dir}")
        sftp.put(local, remote)
        size = os.path.getsize(local)
        print(f"  [OK] {os.path.basename(local)} ({size} bytes)")
    sftp.close()

    # 3b. 上传 ASR 模型文件
    print("\n=== 3b. 上传 ASR 模型文件 ===")
    sftp = client.open_sftp()
    # 先创建远程目录
    ssh_exec(client, f"mkdir -p {MODEL_REMOTE_BASE}")
    for fname in MODEL_FILES:
        local = os.path.join(MODEL_DIR, fname)
        remote = os.path.join(MODEL_REMOTE_BASE, fname)
        if not os.path.isfile(local):
            print(f"  [SKIP] 本地文件不存在: {fname}")
            continue
        sftp.put(local, remote)
        size = os.path.getsize(local)
        mb = size / 1024 / 1024
        print(f"  [OK] {fname} ({mb:.1f} MB)")
    sftp.close()

    # 4. 编译
    print("\n=== 4. 编译包 ===")
    exit_code, out, err = ssh_exec(
        client,
        f"cd {WORKSPACE} && colcon build --packages-select {PKG} 2>&1 | tail -15",
        timeout=120)
    if "Finished" in out or exit_code == 0:
        print("[PASS] 编译成功")
    else:
        print("[WARN] 编译可能有警告")

    # 5. 验证版本
    print("\n=== 5. 验证版本 ===")
    ssh_exec(client,
             f"bash -c 'source {WORKSPACE}/install/setup.bash && "
             f"python3 -c \"import laser_calibration; print(laser_calibration.__version__)\"'",
             timeout=10)

    # 6. 测试 ASR 模块
    print("\n=== 6. 测试 ASR 模块 ===")
    ssh_exec(client,
             f"bash -c 'source {WORKSPACE}/install/setup.bash && python3 -c \"from laser_calibration.asr_server import match_command; print(match_command(\\\"发车\\\")); print(match_command(\\\"急停\\\")); print(match_command(\\\"归中\\\")); print(match_command(\\\"开激光\\\")); print(match_command(\\\"关激光\\\")); print(match_command(\\\"清场\\\")); print(match_command(\\\"你好世界\\\"))\"'",
             timeout=15)

    # 7. 检查 sherpa-onnx
    print("\n=== 7. 检查 sherpa-onnx ===")
    ssh_exec(client,
             "python3 -c \"import sherpa_onnx; print(f'sherpa-onnx: {sherpa_onnx.__version__}')\"",
             timeout=10)

    # 8. 检查 executables
    print("\n=== 8. 检查节点列表 ===")
    ssh_exec(client,
             f"bash -c 'source {WORKSPACE}/install/setup.bash && "
             f"ros2 pkg executables {PKG}'",
             timeout=10)

    client.close()
    print("\n" + "=" * 50)
    print("部署完成！")
    print("=" * 50)
    print(f"\n接下来在小车上安装 ASR 推理依赖:")
    print(f"  ssh {USER}@{HOST}")
    print(f"  pip install sherpa-onnx")
    print(f"  # 然后下载模型...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
