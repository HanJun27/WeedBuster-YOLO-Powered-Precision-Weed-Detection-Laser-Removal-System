@echo off
chcp 65001 >nul
title 本地 faster-whisper ASR 服务 :8094
echo ============================================
echo   faster-whisper 本地 ASR 服务
echo   监听端口: 8094
echo   供 8093 面板调用 (操作员端)
echo ============================================
echo.
echo 使用前请确保:
echo   1. pip install faster-whisper
echo   2. 已下载 faster-whisper 模型
echo.
echo 默认模型路径: ./语音模型
echo 可在 8093 面板中选择模型路径并点击"加载"
echo.

cd /d %~dp0
python laser_calibration\laser_calibration\local_asr_server.py --model ./语音模型

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo 启动失败! 请检查:
    echo   - Python 是否已安装
    echo   - faster-whisper 是否已安装 (pip install faster-whisper)
    echo   - 模型路径是否正确
    pause
)
