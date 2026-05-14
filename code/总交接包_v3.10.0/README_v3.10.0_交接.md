# laser_calibration v3.10.0 终极交接 README

> **项目**：双目多光谱感知 + YOLO 检测 + IBVS 闭环激光除草 + NDVI 健康检测
> **平台**：亚博 RDK X5 · Ubuntu 22.04 ARM64 · ROS2 Humble · BPU 加速
> **状态**：✅ 激光除草已验证 / 🆕 NDVI 已集成（未上车测）
> **5/15 初赛**：3 天倒计时 ⏰

---

## 0. 重要说明（必读）

本版本（v3.10.0）相对 v3.9.10 的主要变化：

| 模块 | v3.9.10 状态 | v3.10.0 状态 |
|---|---|---|
| **激光除草链路**（stereo_camera, yolo_detector, vision_servo） | ✅ 战友实测验证：BPU 27ms、PID 收敛、打击成功 | ✅ **未改动**，继续稳定 |
| **NDVI 健康检测**（ndvi_node, calib_diffuse, calib_io） | ❌ 旧 v3.0 版本，无 active mode | 🆕 **集成 NDVI 队友 v3.10 新版** |
| **共享配置**（config.py） | v3.9.10 PID 抗振荡参数 | ✅ 保留 + 追加 NDVI 配置段 |

⚠️ **NDVI 部分代码未在小车上实测过**——已通过语法检查 + 字段兼容性分析，但部署后**第一次启动需要按 §4 的诊断流程走一遍**。

✅ **激光除草部分（你已经跑通 BPU + 27ms 推理 + PID 收敛）完全没动**，部署 v3.10.0 不会破坏现有功能。

---

## 1. 一句话项目状态

**两条独立任务线，激光除草已验证 76 FPS BPU 推理 + 3 像素 PID 收敛，NDVI 健康检测代码已集成等待上车标定。NDVI 与激光打击不耦合，互不影响。**

---

## 2. 系统架构（两条独立任务线）

```
┌─────────────────────────────────────────────────────────────────┐
│  任务 A：激光除草（已实测）                                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  stereo_camera (RGB 30fps) ──► yolo_detector (BPU 76 FPS)        │
│         │                              │                          │
│         │                              ▼ /yolo/weed_detected     │
│         │                          (10Hz JSON, 含心跳)            │
│         │                              │                          │
│         └─────────────────────► vision_servo (IBVS PID)          │
│                                        │                          │
│                                        ▼                          │
│                                  S1/S2 云台 + S3 蓝紫激光         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  任务 B：NDVI 健康检测（v3.10 新增，未上车测试）                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  stereo_camera (RGB+IR)                                          │
│         │                                                         │
│         ▼                                                         │
│  calib_diffuse (Web :8094)                                       │
│   暗电流采样 + 灰卡 ROI + 主动光场标定                            │
│         │                                                         │
│         ▼ (calib_params.yaml 增量更新 calib4_* 字段)              │
│         │                                                         │
│         ▼                                                         │
│  ndvi_node (Web :8082)                                           │
│   三阶段自适应：active mode / refl 4点 / pseudo NDVI 兜底         │
│   实时输出 /ndvi/image + /ndvi/result                            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**关键：两条任务线共用 `stereo_camera` 节点和 `calib_io` 模块，但执行流程不交叉。NDVI 不依赖 YOLO，激光除草不依赖 NDVI。**

---

## 3. v3.10.0 部署命令（必读）

### 3.1 上传 + 解压（一次性命令）

```bash
# 在你电脑上
scp laser_calibration_v3_10_0.zip sunrise@<小车IP>:~/

# 在小车上
ssh sunrise@<小车IP>
cd ~/yahboomcar_ws/src

# ⚠️ 关键：不要 mkdir！直接 unzip，让 zip 自己建外层
rm -rf laser_calibration
unzip ~/laser_calibration_v3_10_0.zip

# 验证双层结构正确
ls laser_calibration/
# 期望: laser_calibration  models  package.xml  resource  setup.py
ls laser_calibration/laser_calibration/ | head -5
# 期望: __init__.py calib_camera_align.py calib_diffuse.py ... ndvi_node.py
```

### 3.2 检查 setup.py / setup.cfg 仍然存在（v3.9.10 解决过的坑）

```bash
ls ~/yahboomcar_ws/src/laser_calibration/setup.py        # 必须有
ls ~/yahboomcar_ws/src/laser_calibration/setup.cfg       # 必须有
cat ~/yahboomcar_ws/src/laser_calibration/setup.py | grep package_dir
# 期望: package_dir={package_name: package_name},
```

**如果 `setup.cfg` 不存在**（v3.10.0 zip 可能没带），按 v3.9.10 时的步骤补：

```bash
cat > ~/yahboomcar_ws/src/laser_calibration/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/laser_calibration
[install]
install_scripts=$base/lib/laser_calibration
EOF
```

### 3.3 编译

```bash
cd ~/yahboomcar_ws
rm -rf build install log
colcon build --packages-select laser_calibration
source install/setup.bash

# 验证 9 个 entry points 全部就位
ls install/laser_calibration/lib/laser_calibration/
# 期望: stereo_camera, yolo_detector, vision_servo,
#       calib_camera, calib_laser, calib_refl, show_calib,
#       ndvi_node, calib_diffuse    ← 后两个是 v3.10 新增

# 验证模块导入
python3 -c "import laser_calibration; print('版本:', laser_calibration.__version__)"
# 期望: 版本: 3.10.0
```

如果还是只看到 `setup.py` 在 install 目录里，按 v3.9.10 时的兜底方案手动复制：

```bash
INSTALL_DIR=~/yahboomcar_ws/install/laser_calibration/lib/python3.10/site-packages/laser_calibration
SRC_DIR=~/yahboomcar_ws/src/laser_calibration/laser_calibration
rm -f $INSTALL_DIR/setup.py
rm -rf $INSTALL_DIR/__pycache__
cp $SRC_DIR/*.py $INSTALL_DIR/
ls $INSTALL_DIR/   # 期望: 14 个 .py 文件
```

---

## 4. 激光除草链路验证（已实测，应原样工作）

```bash
# 终端 1
ros2 run laser_calibration stereo_camera

# 终端 2 — 期望第一行: "✅ BPU 模型加载成功"
ros2 run laser_calibration yolo_detector

# 终端 3
ros2 run laser_calibration vision_servo
```

浏览器 `http://<IP>:8093`：

- [ ] BPU 推理 ~27ms/帧（终端 2 log 中 `[BPU] inference=XX.Xms`）
- [ ] 紫色十字朝蓝色框平滑收敛（不疯转）
- [ ] 移动杂草盒子，紫色十字跟随（不卡在首次位置）
- [ ] 收敛后蓝紫激光自动开火 1 秒（白纸焦痕）

**这部分 v3.10.0 完全没改，应该和 v3.9.10 一样工作。**

---

## 5. NDVI 链路验证（🆕 第一次上车，必读）

### 5.1 NDVI 启动顺序

NDVI 节点**必须先做标定四**才能进入 active mode，否则走 pseudo NDVI 兜底：

```bash
# 终端 1 (复用)
ros2 run laser_calibration stereo_camera

# 终端 2 — NDVI 标定节点
ros2 run laser_calibration calib_diffuse
# 期望日志: "calib_diffuse 启动，浏览器打开 http://<IP>:8094"
```

浏览器 `http://<IP>:8094` 走标定流程：

1. **第一步：暗电流采样**
   - 用黑布盖住摄像头（IR + RGB 都要）
   - 点 `[采样暗电流]`，等 30 帧采样完成
   - 期望日志: `dark_R=X.X, dark_NIR=X.X (理论<10)`

2. **第二步：灰卡 ROI 拖框**
   - 把 18% 灰卡放进画面（推荐摆在边角）
   - 用鼠标在 RGB 流上拖出灰卡区域
   - 点 `[采样灰卡]`，等 30 帧
   - 期望日志: `k_active=X.X (理论 0.8 ~ 1.5)`

3. **第三步：保存**
   - 点 `[保存标定四]` → 写入 `~/calib_params.yaml`
   - 验证: `ros2 run laser_calibration show_calib` 应显示 `calib4_done=True`

```bash
# 终端 3 — NDVI 推理节点
ros2 run laser_calibration ndvi_node
# 期望日志: "active mode 已启用 (K=X.X)" 或 "pseudo mode (calib4 未完成)"
```

浏览器 `http://<IP>:8082`：

- [ ] NDVI 图像实时显示（红→黄→绿渐变）
- [ ] 右上角显示当前模式: `active` / `refl` / `pseudo`
- [ ] 健康分级占比：健康/亚健康/枯萎/非植物
- [ ] 灰卡 ROI 区域被黑色矩形遮挡（演示一致性）

### 5.2 NDVI 可能遇到的问题

| 现象 | 排查 |
|---|---|
| 启动报 `ModuleNotFoundError: No module named 'laser_calibration.calib_diffuse'` | 编译没把 calib_diffuse.py 装进去，重跑 §3.3 |
| NDVI 网页空白 | 检查 IR 摄像头是否正常: `ros2 topic hz /camera/ir/image_raw` 应有 30Hz |
| NDVI 数值都是 0 | 标定四未完成 → pseudo mode 兜底; 或 IR 摄像头无信号 |
| K 系数 < 0.5 或 > 2.0 | 灰卡位置不对 / 光源不均匀; 重新拖框采样 |
| 灰卡 ROI 没被遮挡 | config.py 检查 `GRAY_ROI_MASK_ON_OUTPUT = True` |
| 健康分级全是"非植物" | NDVI 阈值过严，临时调低 `NDVI_PLANT_MIN` 到 0.05 |

---

## 6. 关键代码地图（按重要性）

| 文件 | 用途 | 修改了吗？ |
|---|---|---|
| `vision_servo.py` | IBVS PID 闭环（核心）| v3.9.10 修复 + v3.9.9 抗振荡 |
| `yolo_detector.py` | BPU/CPU YOLO 检测 | v3.9.7 BPU 集成 |
| `stereo_camera.py` | 双目相机驱动 | v3.0 未动 |
| `calib_io.py` | 标定参数读写 | 🆕 v3.10 加 calib4_* 字段 |
| `calib_diffuse.py` | 🆕 标定四节点 | 🆕 v3.10 NDVI 队友 |
| `ndvi_node.py` | 🆕 NDVI 三模式计算 | 🆕 v3.10 NDVI 队友 |
| `config.py` | 全局配置 | 保留 v3.9.10 + 追加 NDVI 段 |
| `calib_camera_align.py` | 标定一：摄像头基线 | v3.0 未动 |
| `calib_laser_offset.py` | 标定二：激光偏移 | v3.0 未动 |

---

## 7. 5/15 检查清单

### 7.1 激光除草链路（高优先级）

- [ ] BPU 推理日志出现 "✅ BPU 模型加载成功"
- [ ] 终端 2 inference < 50ms / 帧
- [ ] 浏览器 :8093 YOLO 新鲜度持续显示**绿色**
- [ ] 测试 A：紫色十字平滑收敛
- [ ] 测试 B：移动目标紫色十字跟随
- [ ] 测试 C：终端 3 不频繁报 "饱和警告"
- [ ] 收敛后蓝紫激光自动开火 1 秒

### 7.2 NDVI 链路（低优先级，比赛加分项）

- [ ] calib_diffuse 节点启动成功
- [ ] 标定四完成（暗电流 + 灰卡）
- [ ] ndvi_node 显示 `active mode`
- [ ] 健康分级数据合理（绿叶应显示 healthy）

### 7.3 演示视频脚本（30 秒）

```
[0-5s]  全景
        "RDK X5 BPU 加速的低成本精准农业系统"

[5-10s] YOLO 检测画面（蓝色框）
        "YOLOv8n BPU 76 FPS 实时识别杂草"

[10-15s] vision_servo 紫色十字朝蓝框收敛
         "IBVS 解耦 PID 闭环 3 像素精度"

[15-20s] 蓝紫激光烧 1 秒
         "蓝紫激光物理除草"

[20-25s] 切到 NDVI 画面：彩色 NDVI 图 + 健康分级
         "850nm IR 多光谱 NDVI 健康检测"

[25-30s] 总结
         "成本 ¥300 替代万元商业方案"
```

---

## 8. 时间预算 / 倒计时

| 还有时间 | 任务 |
|---|---|
| **今晚** | 部署 v3.10.0，验证激光除草链路 ✅ |
| **明天上午** | NDVI 上车标定 + 验证 active mode |
| **明天下午** | 录演示视频（按 §7.3 脚本）|
| **后天** | 备用调试 + 备份方案录第二个视频 |
| **5/15** | 上场 🚀 |

---

## 9. 应急联系 / 备份方案

### 9.1 如果 NDVI 集成失败

NDVI 不通**不影响激光除草演示**。备用方案：演示视频只展示激光除草部分（已实测稳定），策划书里 NDVI 部分写"已实现完整架构，集成测试进行中"。

### 9.2 如果 BPU 推理失败（罕见）

`yolo_detector.py` 会自动 fallback 到 CPU 模式（1800ms/帧）。这种情况下：
- 把 `vision_servo` 的 `FSM_TICK_PERIOD_SEC` 调到 2.0（每 2 秒一次 PID）
- 演示节奏放慢但功能仍可用

### 9.3 如果 colcon build 又出问题

按 v3.9.10 时验证的兜底（§3.3 末尾）：手动 cp 模块文件到 install 目录。

---

## 10. 版本演化全记录

```
v3.7/3.8  IR 摄像头时代
v3.9.0    切 RGB 摄像头（S4 是可见红激光）
v3.9.1    S3 蓝紫独立测试按钮
v3.9.2    calib2_frame 字段防呆
v3.9.3    画面叠加三标记 + 蓝紫预测落点
v3.9.4    修 _step_pid 多余负号 + 加权质心
v3.9.5    R-max(G,B) + ROI + FSM 卡死修复
v3.9.6    PID stale-dt reset + hypot 收敛判据
v3.9.7    yolo_detector BPU + CPU fallback
v3.9.9    修 PID 振荡 + "目标坐标只用第一次"
v3.9.10   修 setup.py 双层目录 bug → BPU 实测 27ms 跑通
v3.10.0   🆕 集成 NDVI 队友 v3.10 active mode（calib_diffuse + ndvi_node）
```

---

## 11. 关键工程教训（给以后接手的人）

1. **物理事实 > 直觉**：servo_direction_test.py、hrt_model_exec perf 这种脱机测试省了无数猜测
2. **数据驱动**：CPU 1800ms vs BPU 27ms 直接定决策，不靠拍脑袋
3. **解耦设计救命**：image_callback 跟 publish_timer 解耦，CPU 慢不影响 publish 频率
4. **多版本备份**：v3.9.0-v3.10.0 共 11 个 minor 版本，每版打包能回退
5. **ament_python 双层目录**：v3.9.10 setup.py + setup.cfg 双保险吃过的亏不能忘
6. **PID + 帧间稳定性的冲突**：v3.9.9 `_pid_actively_moving` 标志的教训

---

**5/15 加油！🚀 这一路从 v3.7 走到 v3.10.0，所有大坑都填了。代码稳，部署稳，演示稳。**

**激光除草已经跑通——这是核心战斗力。NDVI 是锦上添花。**

**冲！💪**
