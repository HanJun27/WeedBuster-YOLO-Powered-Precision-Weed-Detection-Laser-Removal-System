#!/usr/bin/env python3
import json

# 读取quant_info.json
with open('/home/sunrise/yahboomcar_ws/src/laser_calibration/models/quant_info.json', 'r') as f:
    data = json.load(f)

print("=" * 60)
print("量化模型信息分析")
print("=" * 60)

# 1. 输入预处理层
print("\n【1. 输入预处理层】")
preprocess = data.get('HZ_PREPROCESS_FOR_images', {})
if preprocess:
    print(f"  类型: {preprocess.get('type')}")
    print(f"  Cosine相似度: {preprocess.get('cosine_similarity')}")
    print(f"  输入: {preprocess.get('inputs')}")
    print(f"  输出: {preprocess.get('outputs')}")
    if 'thresholds' in preprocess:
        print(f"  Thresholds: {preprocess['thresholds']}")

# 2. 统计信息
print(f"\n【2. 模型统计】")
print(f"  总层数: {len(data)}")

# 3. 输出层（最后几层）
print(f"\n【3. 输出层分析（最后5层）】")
last_layers = list(data.items())[-5:]
for name, info in last_layers:
    print(f"  {name}")
    print(f"    类型: {info.get('type')}")
    print(f"    Cosine: {info.get('cosine_similarity')}")
    if 'thresholds' in info:
        print(f"    Thresholds: {info['thresholds']}")

# 4. 查找所有Concat_3或最终输出
print(f"\n【4. 最终输出层】")
for key in ['output0_before', '/model.22/Concat_3']:
    if key in data:
        layer = data[key]
        print(f"  {key}:")
        print(f"    类型: {layer.get('type')}")
        print(f"    Cosine: {layer.get('cosine_similarity')}")
        print(f"    Outputs: {layer.get('outputs')}")
        if 'thresholds' in layer:
            print(f"    Thresholds: {layer['thresholds']}")

# 5. Softmax相关
print(f"\n【5. Softmax层】")
softmax_keys = [k for k in data.keys() if 'Softmax' in k]
for key in softmax_keys[:3]:  # 只显示前3个
    layer = data[key]
    print(f"  {key}:")
    print(f"    类型: {layer.get('type')}")
    print(f"    Cosine: {layer.get('cosine_similarity')}")

print("\n" + "=" * 60)
print("分析完成！")
print("=" * 60)
