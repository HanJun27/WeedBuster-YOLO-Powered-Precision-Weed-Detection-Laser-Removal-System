#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_asr_server.py —— 操作员端 faster-whisper 本地 ASR 服务  v3.14.0
====================================================================
监听 :8094，供 8093 网页面板调用，作为 RDK 端侧 sherpa-onnx 的备用方案。

**使用方式**
    pip install faster-whisper
    python local_asr_server.py --model D:/models/faster-whisper-small

**架构**
    操作员PC浏览器(8093) ←→ 小车 RDK:8093 (vision_servo)
                  ↕ (本地 localhost:8094)
    操作员PC: local_asr_server.py (faster-whisper)

**API**
    GET  /health      → {"ok": true, "model_loaded": bool}
    GET  /model       → {"path": str, "loaded": bool, "size": str}
    POST /model       → {"path": "..."}  设置模型路径并加载
    POST /transcribe  → WAV 音频 body → {"text": str, "ok": bool}
"""

import argparse
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Lock


# ── 全局模型状态（跨请求共享） ──────────────────────────────────
_model = None
_model_path = None
_model_size = ""
_model_lock = Lock()


def _load_model(path):
    """加载 faster-whisper 模型，返回 (model, size_str) 或 (None, error_msg)。"""
    global _model, _model_path, _model_size
    path = os.path.expanduser(path)
    if not os.path.isdir(path):
        return None, f"目录不存在: {path}"
    try:
        from faster_whisper import WhisperModel
        # 检测模型大小
        size = "small"
        for s in ["large-v3", "large-v2", "large", "medium", "small", "base", "tiny"]:
            if s in path.lower():
                size = s
                break
        log(f"正在加载 faster-whisper 模型: {path} (size={size}) ...")
        t0 = time.time()
        m = WhisperModel(model_size_or_path=path,
                         device="auto",       # CUDA if available else CPU
                         compute_type="auto", # float16 for CUDA, float32 for CPU
                         local_files_only=True)
        elapsed = time.time() - t0
        log(f"模型加载成功! 耗时={elapsed:.1f}s, size={size}")
        return m, size
    except ImportError:
        return None, "faster-whisper 未安装: pip install faster-whisper"
    except Exception as e:
        return None, f"加载失败: {e}"


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ── HTTP Handler ──────────────────────────────────────────────
class AsrHandler(BaseHTTPRequestHandler):
    """处理 /health, /model, /transcribe 请求。"""

    def _json(self, data, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path == "/health":
            with _model_lock:
                ok = _model is not None
            self._json({"ok": True, "model_loaded": ok})

        elif self.path == "/model":
            with _model_lock:
                self._json({
                    "path": _model_path,
                    "loaded": _model is not None,
                    "size": _model_size,
                })

        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        path = self.path.split("?")[0]

        if path == "/model":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body)
                new_path = data.get("path", "").strip()
            except (json.JSONDecodeError, TypeError):
                self._json({"ok": False, "error": "JSON 解析失败"}, 400)
                return
            if not new_path:
                self._json({"ok": False, "error": "路径不能为空"}, 400)
                return
            global _model, _model_path, _model_size
            m, info = _load_model(new_path)
            with _model_lock:
                if m is not None:
                    _model = m
                    _model_path = new_path
                    _model_size = info
                    self._json({"ok": True, "loaded": True, "size": info})
                else:
                    self._json({"ok": False, "loaded": False, "error": info})

        elif path == "/transcribe":
            if _model is None:
                self._json({"ok": False, "text": "", "error": "模型未加载"})
                return
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length > 0 else b""
            if not body or len(body) < 100:
                self._json({"ok": False, "text": "", "error": "音频数据为空"})
                return
            t0 = time.time()
            try:
                import io
                import numpy as np
                # 将 WAV 字节写入临时 BytesIO 供 faster-whisper 读取
                # faster_whisper 支持直接传入文件路径或 bytes
                segments, info = _model.transcribe(
                    io.BytesIO(body),
                    language="zh",
                    beam_size=1,
                    vad_filter=True,
                    vad_parameters=dict(
                        min_silence_duration_ms=300,
                        threshold=0.5,
                    ),
                )
                result_text = ""
                for seg in segments:
                    result_text += seg.text
                result_text = result_text.strip()
                elapsed_ms = round((time.time() - t0) * 1000, 1)
                log(f"识别=\"{result_text}\" 耗时={elapsed_ms:.0f}ms "
                    f"(lang={info.language}, prob={info.language_probability:.2f})")
                self._json({
                    "ok": True,
                    "text": result_text,
                    "ms": elapsed_ms,
                    "language": info.language,
                })
            except Exception as e:
                log(f"推理异常: {e}")
                self._json({"ok": False, "text": "", "error": str(e)})

        else:
            self._json({"error": "not found"}, 404)


# ── 入口 ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="本地 faster-whisper ASR 服务 :8094")
    parser.add_argument("--model", "-m", default="./models/faster-whisper-small",
                        help="faster-whisper 模型路径")
    parser.add_argument("--port", "-p", type=int, default=8094,
                        help="监听端口 (默认 8094)")
    args = parser.parse_args()

    # 预加载模型
    m, info = _load_model(args.model)
    global _model, _model_path, _model_size
    if m is not None:
        _model = m
        _model_path = args.model
        _model_size = info
    else:
        log(f"⚠ 模型加载失败: {info}")
        log("  服务已启动，可通过 POST /model 设置路径后重新加载")

    server = HTTPServer(("0.0.0.0", args.port), AsrHandler)
    log(f"faster-whisper ASR 服务已启动: http://127.0.0.1:{args.port}")
    log(f"  API:")
    log(f"    GET  /health      — 健康检查")
    log(f"    GET  /model       — 当前模型状态")
    log(f"    POST /model       — 设置模型路径并重新加载")
    log(f"    POST /transcribe  — WAV 音频识别")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("服务已停止")
        server.server_close()


if __name__ == "__main__":
    main()
