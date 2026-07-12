#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
asr_server.py —— RDK 端侧 ASR 语音控制引擎  v3.14.0 新增
============================================================
封装 sherpa-onnx OfflineRecognizer，实现"按住说话→识别→语法约束→命令"管线。

**设计要点**
- 零 ROS2 依赖：纯 numpy + sherpa_onnx，可在任何 Python 环境独立使用。
- Graceful fallback：模型文件不存在时 self.available=False，不崩溃、不弹窗，
  所有异常都被 catch 吞掉，返回 dev_mode 模拟结果。
- 语法约束后过滤：sherpa-onnx 做开放中文识别，再用命令词表做子串/拼音
  模糊匹配 —— 小模型在受限词表上的抗噪表现远好于开放识别。

**用法**
    from laser_calibration.asr_server import AsrEngine
    engine = AsrEngine("~/laser_calibration/asr_models/paraformer-small")
    if engine.available:
        result = engine.recognize(wav_bytes)  # wav_bytes = 16kHz 16bit mono WAV
        print(result["text"], result["command"], result["confidence"])

**模型下载**
    mkdir -p ~/laser_calibration/asr_models
    cd ~/laser_calibration/asr_models
    wget https://github.com/k2-fsa/sherpa-onnx/releases/download/\
asr-models/sherpa-onnx-paraformer-zh-small-2024-03-09.tar.bz2
    tar xvf sherpa-onnx-paraformer-zh-small-2024-03-09.tar.bz2
    # → 目录结构: paraformer-small/encoder.onnx, decoder.onnx, tokens.txt, ...
"""

import os
import re
import time

# ── 命令词表（语法约束） ────────────────────────────────────────────
# key = 标准化命令名, value = (触发词列表, 映射说明)
COMMAND_VOCABULARY = {
    "START":     (["发车", "开车", "开始作业", "启动", "开始干活",
                   "start", "出发", "走", "走吧", "走啊",
                   "前进", "开始"],                    "/api/mission_start"),
    "STOP":      (["停车", "收工", "停止作业", "停下",
                   "stop", "停止", "停", "停啦"],     "/api/mission_stop"),
    "ESTOP":     (["急停", "紧急停止", "立刻停下", "紧急停车",
                   "estop", "紧急", "停停停"],          "/api/estop"),
    "CENTER":    (["归中", "回中", "居中", "重置",
                   "center", "中"],                     "/api/center"),
    "LASER_ON":  (["开激光", "打开激光", "开指示", "开红光",
                   "laser on", "激光", "开灯"],         "/api/laser_ir?on=1"),
    "LASER_OFF": (["关激光", "关闭激光", "关指示", "关红光",
                   "laser off", "关掉", "关灯",
                   "关闭"],                              "/api/laser_ir?on=0"),
    "FIRE_TEST": (["测试", "试射", "打一下", "试一下", "点火",
                   "fire", "打", "烧"],                "/api/fire_test"),
    "CLEAR":     (["清场", "开始清除", "清除", "清理",
                   "clear", "清零"],                    "/planner/start_clearing"),
    "STATS":     (["统计", "报表", "战果", "成绩", "报告",
                   "stats", "结果"],                    "仅显示"),
}

# 展平为 (substring, command) 快速匹配列表（越长越优先匹配）
_FLAT_VOCAB = []
for cmd, (triggers, _) in COMMAND_VOCABULARY.items():
    for t in triggers:
        _FLAT_VOCAB.append((t, cmd))
# 按触发词长度降序（长匹配优先于短匹配，如"紧急停止"优先于"停止"）
_FLAT_VOCAB.sort(key=lambda x: -len(x[0]))


def match_command(text, min_confidence=0.40):
    """语法约束匹配：对识别文本做子串/整词匹配，返回 (命令名, 置信度)。

    Args:
        text: ASR 识别文本（已去噪、小写化）。
        min_confidence: 最低匹配置信度 (0~1)，低于此值返回 None。

    Returns:
        (command_name, score) 或 (None, 0.0)。
    """
    if not text or not text.strip():
        return None, 0.0
    t = text.strip().lower()

    # ① 精确整词匹配（最高分 1.0）
    for keyword, cmd in _FLAT_VOCAB:
        if t == keyword.lower():
            return cmd, 1.0

    best_cmd, best_score = None, 0.0

    # ② 正向子串匹配：触发词 ∈ 识别文本（如发送"请开车"含"开车"）
    for keyword, cmd in _FLAT_VOCAB:
        kw = keyword.lower()
        if kw in t:
            # 分数 = 触发词长度 / 文本总长度，但上限 0.95（纯子串不如整词高）
            score = min(len(kw) / max(len(t), 1), 0.95)
            if score > best_score:
                best_cmd, best_score = cmd, score

    # ③ 反向子串匹配：识别文本 ∈ 触发词（如说"激光"含在"开激光"中）
    #    适用于短识别被词表中的长词覆盖的场景，置信度略低
    for keyword, cmd in _FLAT_VOCAB:
        kw = keyword.lower()
        if len(t) >= 2 and t in kw and t != kw:
            # 识别文本越接近触发词完整长度，分数越高
            score = min(len(t) / max(len(kw), 1), 0.85)
            if score > best_score:
                best_cmd, best_score = cmd, score

    if best_score >= min_confidence:
        return best_cmd, round(best_score, 2)
    return None, 0.0


class AsrEngine:
    """端侧 ASR 推理引擎（封装 sherpa-onnx OfflineRecognizer）。"""

    def __init__(self, model_dir, num_threads=2, logger=None):
        """初始化 ASR 引擎。

        Args:
            model_dir: 模型目录路径（含 encoder.onnx / decoder.onnx / tokens.txt）。
            num_threads: 推理线程数（RDK A55 建议 2）。
            logger: 可选的日志函数（如 ros logger.info / print）。
        """
        self.available = False
        self._recognizer = None
        # 兼容两种 logger: callable 函数 或 ROS RcutilsLogger(.info())
        if logger is None:
            self._log = lambda msg: None
        elif callable(logger):
            self._log = logger
        else:
            # 假设是 ROS logger 对象，用 .info() 方法
            self._log = lambda msg: logger.info(msg)

        model_dir = os.path.expanduser(model_dir)
        if not os.path.isdir(model_dir):
            self._log(f"[ASR] 模型目录不存在: {model_dir}")
            self._log(f"[ASR] 降级为 dev_mode（下载方式见 asr_server.py 注释）")
            return

        tokens = os.path.join(model_dir, "tokens.txt")
        if not os.path.isfile(tokens):
            self._log(f"[ASR] tokens.txt 缺失: {tokens}")
            self._log(f"[ASR] 降级为 dev_mode")
            return

        # 支持两种模型文件名称:
        #   格式A: model.int8.onnx (本项目的 FunASR Paraformer int8 量化模型)
        #   格式B: model.onnx (sherpa-onnx 标准模型)
        model_candidates = [
            os.path.join(model_dir, "model.int8.onnx"),
            os.path.join(model_dir, "model.onnx"),
        ]
        model_path = next((p for p in model_candidates if os.path.isfile(p)), None)
        if model_path is None:
            self._log(f"[ASR] 未找到模型文件(model.int8.onnx 或 model.onnx)")
            self._log(f"[ASR] 降级为 dev_mode")
            return
        model_label = os.path.basename(model_path)

        try:
            import sherpa_onnx
            self._recognizer = sherpa_onnx.OfflineRecognizer.from_paraformer(
                paraformer=model_path,
                tokens=tokens,
                num_threads=num_threads,
                sample_rate=16000,
                feature_dim=80,
                decoding_method="greedy_search",
                provider="cpu",
            )
            self.available = True
            self._log(f"[ASR] sherpa-onnx 引擎加载成功 (model={model_label}, threads={num_threads})")
        except ImportError:
            self._log("[ASR] sherpa-onnx 未安装: pip install sherpa-onnx")
            self._log("[ASR] 降级为 dev_mode")
        except Exception as e:
            self._log(f"[ASR] 加载失败: {e} (降级为 dev_mode)")

    def recognize(self, wav_bytes):
        """对 16kHz 16bit mono WAV 字节做语音识别 + 语法约束匹配。

        Args:
            wav_bytes: 完整的 WAV 文件字节（含 RIFF 头，16kHz 单声道 16bit PCM）。

        Returns:
            dict {text, command, confidence, ms}:
                text:       识别原始文本（空串表示未识别到语音）。
                command:    标准化命令名（START/STOP/ESTOP/...），未匹配则为 None。
                confidence: 命令匹配置信度 (0~1)。
                ms:         推理耗时（毫秒）。
        """
        if not self.available or self._recognizer is None:
            return self._dev_mode_fallback(wav_bytes)

        t0 = time.time()
        try:
            import numpy as np
            # 从 WAV 字节解析 PCM 数据
            samples = _wav_to_pcm(wav_bytes)
            if samples is None or len(samples) == 0:
                self._log("[ASR] WAV 解析为空或无效")
                return {"text": "", "command": None, "confidence": 0.0,
                        "ms": round((time.time() - t0) * 1000, 1)}

            # sherpa-onnx 要求 (samples,) float32 [-1, 1]
            stream = self._recognizer.create_stream()
            stream.accept_waveform(16000, samples.astype(np.float32) / 32768.0)
            # 加尾静音确保尾部语音不被截断
            tail = np.zeros(int(16000 * 0.2), dtype=np.float32)
            stream.accept_waveform(16000, tail)
            # 兼容不同版本 sherpa-onnx API
            try:
                stream.input_finished()
            except AttributeError:
                try:
                    stream.finalize()
                except AttributeError:
                    pass

            # 兼容不同版本 sherpa-onnx API：decode / decode_streams / decode_stream
            try:
                self._recognizer.decode([stream])
            except AttributeError:
                try:
                    self._recognizer.decode_streams([stream])
                except AttributeError:
                    self._recognizer.decode_stream(stream)
            text = stream.result.text.strip()
            elapsed_ms = round((time.time() - t0) * 1000, 1)

            if not text:
                return {"text": "", "command": None, "confidence": 0.0,
                        "ms": elapsed_ms}

            # 语法约束匹配
            cmd, conf = match_command(text)
            self._log(f"[ASR] 识别=\"{text}\" 命令={cmd} 置信度={conf} "
                      f"耗时={elapsed_ms:.0f}ms")
            return {"text": text, "command": cmd, "confidence": conf,
                    "ms": elapsed_ms}

        except Exception as e:
            self._log(f"[ASR] 推理异常: {e}")
            return {"text": "", "command": None, "confidence": 0.0,
                    "ms": round((time.time() - t0) * 1000, 1)}

    def _dev_mode_fallback(self, wav_bytes):
        """模型加载失败时的降级路径：模拟识别（开发调试用）。"""
        # 检查是否真的收到了音频（非空且大于 44 字节 WAV 头）
        is_silence = (wav_bytes is None or len(wav_bytes) < 1024)
        if is_silence:
            return {"text": "", "command": None, "confidence": 0.0,
                    "ms": 0.0, "_dev": True}
        # 试解析 PCM 能量判断是否有语音
        try:
            samples = _wav_to_pcm(wav_bytes)
            if samples is None or len(samples) == 0:
                return {"text": "", "command": None, "confidence": 0.0,
                        "ms": 0.0, "_dev": True}
            energy = (samples.astype(float) ** 2).mean()
            if energy < 10.0:  # 静音
                return {"text": "", "command": None, "confidence": 0.0,
                        "ms": 0.0, "_dev": True}
        except Exception:
            pass
        # 有声音但没模型 → 返回提示
        return {"text": "(dev_mode: ASR 模型未加载)", "command": None,
                "confidence": 0.0, "ms": 0.0, "_dev": True}

    @staticmethod
    def model_download_instructions():
        """返回模型下载指引（供日志/网页显示）。"""
        return (
            "ASR 模型下载:\n"
            "  mkdir -p ~/laser_calibration/asr_models\n"
            "  cd ~/laser_calibration/asr_models\n"
            "  wget https://github.com/k2-fsa/sherpa-onnx/releases/download/"
            "asr-models/sherpa-onnx-paraformer-zh-small-2024-03-09.tar.bz2\n"
            "  tar xvf sherpa-onnx-paraformer-zh-small-2024-03-09.tar.bz2\n"
            "  pip install sherpa-onnx"
        )


# ── WAV 解析工具 ──────────────────────────────────────────────────

def _wav_to_pcm(wav_bytes):
    """从 WAV 字节解析 16bit PCM 样本数组（float64）。

    支持: 16kHz / 8kHz 单声道 16bit PCM WAV (RIFF格式)。
    不支持: 多声道、非 PCM 格式。
    解析失败返回 None。
    """
    if not wav_bytes or len(wav_bytes) < 44:
        return None
    import struct
    import numpy as np
    try:
        # 解析 RIFF 头
        riff, size, wave = struct.unpack_from("<4sI4s", wav_bytes, 0)
        if riff != b"RIFF" or wave != b"WAVE":
            return None
        pos = 12
        fmt_found = False
        channels = 1
        sample_rate = 16000
        bits_per_sample = 16
        while pos < len(wav_bytes) - 8:
            chunk_id = wav_bytes[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", wav_bytes, pos + 4)[0]
            if chunk_id == b"fmt ":
                fmt_data = wav_bytes[pos + 8: pos + 8 + chunk_size]
                audio_format, channels, sample_rate, _, _, bits_per_sample = \
                    struct.unpack_from("<HHIIHH", fmt_data)
                fmt_found = True
            elif chunk_id == b"data":
                data = wav_bytes[pos + 8: pos + 8 + chunk_size]
                if bits_per_sample == 16:
                    samples = np.frombuffer(data, dtype=np.int16).astype(np.float64)
                elif bits_per_sample == 8:
                    samples = (np.frombuffer(data, dtype=np.uint8).astype(
                        np.float64) - 128.0)
                else:
                    return None
                if channels > 1:
                    samples = samples[::channels]  # 只取第一声道
                # 重采样到 16kHz（若为 8kHz）
                if sample_rate == 8000:
                    samples = np.interp(
                        np.linspace(0, len(samples), len(samples) * 2),
                        np.arange(len(samples)), samples)
                return samples
            pos += 8 + chunk_size
            # 对齐
            if chunk_size % 2:
                pos += 1
        return None
    except Exception:
        return None


# ── 独立测试 ──────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    engine = AsrEngine("~/laser_calibration/asr_models/paraformer-small",
                       logger=print)
    if not engine.available:
        print("[ASR] 模型未加载，运行 dev_mode")
        print(AsrEngine.model_download_instructions())
    else:
        print("[ASR] 引擎就绪，输入 WAV 文件路径测试:")
        for line in sys.stdin:
            path = line.strip()
            if not path or not os.path.isfile(path):
                continue
            with open(path, "rb") as f:
                data = f.read()
            r = engine.recognize(data)
            print(f"  文本: {r['text']}")
            print(f"  命令: {r['command']} (置信度: {r['confidence']})")
            print(f"  耗时: {r['ms']}ms")
