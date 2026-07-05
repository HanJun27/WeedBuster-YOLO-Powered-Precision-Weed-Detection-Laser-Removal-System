# 面向智慧农业的多光谱感知与激光靶向除草系统<br/>Laser-Targeted Weeding & Multispectral Sensing on Horizon RDK X5

> **EN**: An edge-AI agricultural robot prototype on the Horizon **RDK X5** (10 TOPS BPU). It closes a full **perception → decision → execution** loop: YOLOv8 weed detection on the BPU, NDVI crop-health estimation on the CPU, multi-target planning with duplicate-strike rejection, and an **Eye-in-Hand** gimbal that aligns a laser onto each weed using **laser-spot visual-servo feedback** (an IBVS-style closed loop that needs no precise hand–eye calibration). Runs on ROS 2. Verified indoors with printed targets and UV photo-paper.
>
> **中文**：基于地平线 **RDK X5**（10 TOPS BPU）的边缘 AI 农业机器人**概念验证原型**。完整跑通“**感知—决策—执行**”闭环：BPU 上 YOLOv8 杂草识别、CPU 上 NDVI 作物健康评估、多目标决策与重复打击防护，以及“**手在眼上（Eye-in-Hand）**”云台——用**激光光斑视觉反馈**将激光精确对准杂草（一种无需精确手眼标定的 IBVS 式闭环）。基于 ROS 2，室内以打印靶标 + 紫外感光纸验证。

本仓库为参赛作品源码，遵循 **MIT 许可证**开源，欢迎复现与二次开发。

---

## 目录

- [1. 系统架构](#1-系统架构)
- [2. 核心算法原理 Core Algorithms](#2-核心算法原理-core-algorithms)
- [3. 硬件清单](#3-硬件清单)
- [4. 软件模块与 ROS 2 节点](#4-软件模块与-ros-2-节点)
- [5. 目录结构](#5-目录结构)
- [6. 环境依赖](#6-环境依赖)
- [7. 安装与编译](#7-安装与编译)
- [8. 运行方式](#8-运行方式)
- [9. Web 监控界面](#9-web-监控界面)
- [10. 关键参数说明](#10-关键参数说明)
- [11. 模型训练与转换（.pt → .onnx → .bin）](#11-模型训练与转换pt--onnx--bin)
- [12. 在 RDK X5 BPU 上部署模型](#12-在-rdk-x5-bpu-上部署模型)
- [13. 性能指标](#13-性能指标)
- [14. 已知局限与改进方向](#14-已知局限与改进方向)
- [15. 许可证](#15-许可证)

---

## 1. 系统架构

系统以“感知—决策—执行”为主线，各模块基于 ROS 2 话题解耦，全部运行于 RDK X5：

```
                 ┌─────────────────────────────────────────────────────┐
                 │                     RDK X5 (10 TOPS)                  │
  RGB+850nm ─┬──►│  感知  stereo_camera → yolo_detector(BPU YOLOv8)      │
  双光谱双目  │   │        └──────────► ndvi_node(CPU 配准 + NDVI 解算)    │
             │   │                                                       │
             │   │  决策  strike_planner  居中建队→贪心排序→去重→调度       │
             │   │                              │ 打击指令(含已打/待打坐标) │
             │   │  执行  vision_servo  绝对角盲跳→红斑重捕获→IBVS闭环→照射 │
             │   │                              │ PWM/GPIO                 │
             │   │  运动  chassis_controller  走—停—清—盲走 状态机          │
                 └─────────────────────────────────────────────────────┘
   云台(舵机) + 红光指示激光 + 蓝紫(405nm)作业激光  ◄── 与相机同步固连(Eye-in-Hand)
```

**数据流**：相机采集双光谱图像 → BPU 运行 YOLOv8 输出杂草框、CPU 解算 NDVI → 决策模块建队排序并逐个下发目标 → 执行模块盲跳、经红斑重捕获与视觉闭环精确对准后激光照射 → 回报结果、去重并调度下一目标 → 一片清完后底盘前进。

---

## 2. 核心算法原理 Core Algorithms

本系统在“相机随云台运动（Eye-in-Hand，手在眼上）”导致图像坐标漂移的条件下，仍能精确、鲁棒地完成目标定位与打击。算法围绕两个坐标系：**中心参考系**（云台归中采帧时的像素坐标，作为“原始坐标”记账）与**当前帧坐标系**（云台运动到某姿态时的实时坐标）。下面五个算法是系统的技术核心，均标注了对应源码位置。

### 2.1 绝对角盲跳　Absolute-Angle Blind Jump

> 📄 `laser_calibration/vision_servo.py`

将目标的中心参考坐标 $(u,v)$ 直接映射为云台**绝对角度**并一次转到位（粗对准）。设归中角 $\theta_c$、红斑基准位 $(u_0,v_0)$、每像素角度 $k$（标定量）、红斑到蓝光落点偏移 $\Delta=(\Delta u,\Delta v)$：

$$\theta_{yaw}=\theta_{c,yaw}+\big((u-\Delta u)-u_0\big)\times k_{yaw}$$

**关键**：每个目标都由其中心参考坐标**独立**解算绝对角，而非在上一目标基础上增量叠加，因此角度误差**不跨目标累积**——这是相对“增量式”方案的本质优势。

### 2.2 红斑反馈闭环重捕获　Laser-Spot Feedback Servoing

> 📄 `laser_calibration/vision_servo.py`

Eye-in-Hand 下红斑在图像中近似恒定。盲跳到位后检测红斑像素 $s=(s_x,s_y)$，定义**锚点**（蓝光预测落点 = 目标应现位置）：

$$\text{anchor}=s+\Delta$$

在当前帧候选杂草框 $\{b_i\}$ 中选离锚点最近、且在门限 $R$ 内者为靶点：

$$b^\*=\arg\min_i\lVert b_i-\text{anchor}\rVert_2,\quad \lVert b^\*-\text{anchor}\rVert\le R$$

**免标定原理**：锚点由红斑的**物理实测位置**确定；机械回程、舵机量化、标定漂移等造成的盲跳残差被红斑**自动吸收**，无需精确手眼标定即可像素级对准。这也是本方案区别于依赖图像雅可比矩阵的经典 IBVS 之处。

**红斑检测**（`find_red_spot`）采用多重判据保证野外鲁棒：R−max(G,B) 红主导评分 → 亮度门（剔除土壤等暗红背景）→ 形态学闭运算 → 面积窗 + 圆度过滤（剔除长条反光）→ ROI hint 加速（优先在上次位置附近搜索）。

### 2.3 共识平移身份核验　Consensus-Translation Identity Check

> 📄 `laser_calibration/vision_servo.py`（选框）、`strike_planner.py`（下发已打/待打坐标）

多目标 + 相机运动时，需判定候选框归属“当前目标 / 已清除 / 其它待清”，以**杜绝重复打击**。设当前目标中心参考坐标 $C_2$、其它目标集合 $\{P_j\}$，由 anchor 得中心参考系到当前帧的平移量 $\tau=\text{anchor}-C_2$，则其它目标当前帧预测位 $P_j'=P_j+\tau$。当多个候选逼近 anchor（歧义）时，对候选 $b$ 假设其为当前目标（平移 $e_2=b-\text{anchor}$），检验其它目标预测位是否各有**不同**的框印证（容差 $\tau_0$）：

$$\text{score}(b)=\#\{\,j:\exists\,b_k\ne b,\ \lVert b_k-(P_j'+e_2)\rVert\le \tau_0\,\}$$

取得分最大者为当前目标；全无印证则**安全拒收**（本帧不打）。**原理**：Eye-in-Hand 下全场景共享同一平移，唯有真正的当前目标其 $e_2$ 能同时对齐所有其它框，错框（已打）对不齐而被排除。该思想同于多目标跟踪的“相机运动补偿 + 数据关联”，**实测可拦截对已清除目标的重复打击**。

### 2.4 量化感知闭环控制　Quantization-Aware Servo Control

> 📄 `laser_calibration/vision_servo.py`

针对 1° 分辨率 PWM 舵机的量化限制。每 1° 对应像素位移 $q=1/k$（约 10–20 px）。由像素误差 $e$ 计算期望角度增量并**量化为整数度**下发：

$$\Delta\theta_{cmd}=\operatorname{round}(K_p\cdot k\cdot e)$$

锁定判据为**整数度移动量为零**（误差 $<q/2$ 时 $\operatorname{round}(\cdot)=0$），锁定于**可达最优网格点**，并记录历史最优、连续无改善则回锁。**原理**：承认量化精度地板（约 5 px），在“激光已能命中”的精度内主动收手，避免在栅格相邻格点间反复横跳/过冲——从而在廉价舵机上达成接近物理极限的**像素级**对准，体现“低成本硬件 + 量化感知算法 = 高精度”。

### 2.5 多目标决策与调度　Multi-Target Planning

> 📄 `laser_calibration/strike_planner.py`

云台归中后在中心参考系采帧得目标集合 $\{T_i\}$（原始坐标记账，避免运动干扰投票）；按**贪心最近**排序最小化累计转动；逐个下发（随附**已清除集合 $S$** 与**待清除集合 $Q$** 供身份核验）；成功则 $S\leftarrow S\cup\{T\}$，失败则重排回 $Q$ 重试；去重由 2.3 保证。一片清空后通知底盘前进，进入下一轮闭环。**系统只有这一个决策入口**——单目标即“队列长度为 1 的多目标”，同一套投票—建队—派发—核销流程（网页 manual/auto 为开发调试通道，默认关闭、不参与自动作业）。

### 2.6 NDVI 三层定标　NDVI with Vicarious Calibration

> 📄 `laser_calibration/ndvi_node.py`、`calib_diffuse.py`

区别于“两路 DN 值直接套公式”的朴素做法，本系统 NDVI 采用**低成本代理定标（vicarious calibration）**：暗电流减除 + PTFE 漫反射板（灰卡）K 系数修正，将 R / NIR 两通道拉至同一辐射基准：

$$NDVI=\frac{K\cdot NIR'-R'}{K\cdot NIR'+R'},\quad R'=DN_R-dark_R,\ \ K=\frac{R'_{gray}}{NIR'_{gray}}$$

系统按三层模式**自动降级**：① 主动定标（标定四完成，接近真值 NDVI）→ ② 反射率色卡定标 → ③ 相对（伪）NDVI 兜底（未标定时仅作长势相对比较）。距离/材质效应在比值与 K 修正中抵消；换光照/场景需重新标定。

---

## 3. 硬件清单

**平台声明**：本作品基于**亚博智能（Yahboom）RDK X5 麦克纳姆轮智能车平台**开发——底盘车体、电机驱动（`Mcnamu_driver`）与 `Rosmaster_Lib` 底层库来自平台厂商；**双光谱相机-激光云台模组的机械集成，以及全部上层软件（检测 / NDVI / 决策 / 伺服 / 车控状态机等 12 个 ROS 2 节点）为本团队自主设计实现**。

硬件以 RDK X5 为核心控制器，外接双光谱双目相机、承载相机与激光的二自由度云台、麦克纳姆轮底盘及供电系统。

| 部件 | 规格 | 作用 |
|---|---|---|
| 主控 | 地平线 RDK X5（旭日5，10 TOPS BPU） | 图像处理、AI 推理、决策与控制 |
| 相机 | RGB + 850nm 近红外 双光谱双目 | 杂草识别 + NDVI 解算 |
| 云台 | 二自由度 PWM 舵机（S1 偏航 / S2 俯仰） | 承载相机与激光，实现指向 |
| 指示激光 | 红光 | 作为“激光实际指向”的视觉反馈基准（红斑） |
| 作业激光 | 蓝紫 405nm（原型验证用低功率） | 靶向照射；配合紫外感光纸量化精度 |
| 底盘 | 麦克纳姆轮移动底盘 | 自主行进、区域清场 |

> 说明：作业激光在原型阶段使用低功率 405nm，配合**紫外感光纸**在室内安全地记录、量化激光落点精度，替代真实除草照射。请在使用任何激光时遵守激光安全规范、佩戴对应波段护目镜。

---

## 4. 软件模块与 ROS 2 节点

包名：`laser_calibration`。`ros2 pkg executables laser_calibration` 共 12 个可执行节点：

**主链路（5 个）**
| 节点 | 说明 |
|---|---|
| `stereo_camera` | 双光谱双目相机采集与推流 |
| `yolo_detector` | BPU 上运行 YOLOv8 杂草识别；可选 ExG 绿色度过滤（运行时可经话题开关） |
| `ndvi_node` | RGB/近红外配准 + NDVI 解算；暗电流减除 + PTFE 灰卡 K 修正的三层定标（active→refl→相对值），详见 2.6 |
| `vision_servo` | 视觉伺服：绝对角盲跳 → 红斑重捕获（含身份核验）→ 量化感知闭环 → 激光控制；内置 Web 监控（:8093） |
| `strike_planner` | 多目标决策：居中建队、贪心排序、依次调度、已打去重、失败重试 |
| `chassis_controller` | 底盘“巡航—停车—清场—盲走”有限状态机 |

**NDVI 监控与标定工具（辅助）**
| 节点 | 说明 |
|---|---|
| `ndvi_monitor` | NDVI 结果可视化监控 |
| `calib_camera` | 相机对齐标定 |
| `calib_laser` | 激光偏移（红斑↔蓝光落点 Δ）标定 |
| `calib_refl` / `calib_diffuse` | 反射率色卡定标 / 主动光场漫反射定标（暗电流 + PTFE 灰卡 K，供真值 NDVI） |
| `show_calib` | 标定结果查看 |

---

## 5. 目录结构

```
laser_calibration/
├── laser_calibration/           # ROS 2 节点源码
│   ├── stereo_camera.py         # 相机
│   ├── yolo_detector.py         # BPU YOLOv8 检测
│   ├── ndvi_node.py             # NDVI 解算
│   ├── ndvi_monitor.py          # NDVI 监控
│   ├── vision_servo.py          # 视觉伺服 + Web(:8093)
│   ├── strike_planner.py        # 多目标决策
│   ├── chassis_controller.py    # 底盘状态机
│   ├── chassis_fsm.py           # 底盘走停状态机逻辑
│   ├── config.py                # 全局硬件/话题/参数配置（先看这个）
│   ├── calib_*.py               # 标定工具
│   └── ...
├── models/                      # 模型与量化配置
│   ├── best.pt                  # 训练好的 YOLOv8 权重
│   ├── best.onnx                # 导出的 ONNX（opset11, 标准头）
│   ├── quant.bin                # 地瓜工具链量化后的 BPU 模型（部署用）
│   ├── quant_info.json          # 量化信息（逐层余弦相似度等）
│   ├── config.yaml              # 量化/编译配置
│   └── config_yolov8.yaml       # hb_mapper 转换配置
├── package.xml
├── setup.py
└── resource/
```

> **模型校准图**：量化所需的 `calibration_images/`（约 100 张现场图）体积较大、且仅在**重新量化**时需要，未包含在本仓库；如需自行量化请参照第 10 节自备。

---

## 6. 环境依赖

- **硬件/系统**：RDK X5，官方 Ubuntu 系统，预装 ROS 2 + TogetheROS.Bot（TROS.B）
- **BPU 推理**：地平线 `hobot_dnn`（板端自带）用于加载 `quant.bin`
- **Python**：`rclpy`、`opencv-python`、`numpy`、`cv_bridge`
- **底盘/舵机 SDK**：亚博 `Rosmaster_Lib`（`robot.set_pwm_servo(...)` 等，随底盘提供）
- **模型转换（PC 端，可选）**：`ultralytics`（训练/导出 ONNX）、地瓜 **OpenExplorer / hb_mapper** 工具链（量化，见第 10 节）

> 本项目默认工作空间路径为 `/home/sunrise/yahboomcar_ws`（RDK 板默认 `sunrise` 用户）。若你的用户名/路径不同，请相应修改 `config.py` 中的模型路径等。

---

## 7. 安装与编译

```bash
# 放入 ROS 2 工作空间的 src 下
cd ~/yahboomcar_ws/src
# （将本仓库的 laser_calibration/ 目录置于此处）

# 编译
cd ~/yahboomcar_ws
colcon build --packages-select laser_calibration
source install/setup.bash

# 验证
ros2 pkg executables laser_calibration | wc -l    # 应为 12
python3 -c "import laser_calibration; print(laser_calibration.__version__)"
md5sum src/laser_calibration/models/quant.bin      # 记录以校验模型完整
```

---

## 8. 运行方式

**每个终端都需先** `source ~/yahboomcar_ws/install/setup.bash`。按以下顺序启动（底盘驱动最先）：

```bash
# 终端0：底盘驱动（随底盘提供，例如）
ros2 run yahboomcar_bringup Mcnamu_driver
# 终端1：相机
ros2 run laser_calibration stereo_camera
# 终端2：YOLO 检测（BPU）
ros2 run laser_calibration yolo_detector
# 终端3：视觉伺服（Web :8093）
ros2 run laser_calibration vision_servo
# 终端4：多目标决策
ros2 run laser_calibration strike_planner
# 终端5：底盘状态机
ros2 run laser_calibration chassis_controller
# （可选）NDVI 解算与监控
ros2 run laser_calibration ndvi_node
ros2 run laser_calibration ndvi_monitor
```

**触发指令**（任意已 source 的终端）：

```bash
ros2 topic pub --once /chassis/start          std_msgs/msg/Empty '{}'   # 发车
ros2 topic pub --once /chassis/stop           std_msgs/msg/Empty '{}'   # 收工
ros2 topic pub --once /safety_stop            std_msgs/msg/Empty '{}'   # 急停（轮+激光同停）
ros2 topic pub --once /planner/start_clearing std_msgs/msg/Empty '{}'   # 台架单独清场（不发车）
```

> 键盘遥控与底盘控制互斥（都占用 `cmd_vel`），勿同时启用。

---

## 9. Web 监控界面

`vision_servo` 内置 Web 服务，浏览器访问 `http://<RDK_IP>:8093`：
- 实时相机画面、YOLO 检测框、目标靶点、红外光斑；
- 触发模式（手动/自动）、云台归中、曝光、激光通断、测试照射；
- **ExG 假草过滤开关**（运行时切换，无需重启）。

---

## 10. 关键参数说明

集中在 `config.py`（全局硬件/话题/ISP/标定）与 `vision_servo.py` 顶部（伺服/闭环）。常用可调项：

| 参数 | 位置 | 含义 |
|---|---|---|
| `PIXEL_TO_*_DEG` | config/vision_servo | 像素→云台角度换算（随工作距离标定） |
| `SPOT_HOME_*` | config | 红斑基准位（Eye-in-Hand 参考点） |
| `REACQ_MAX_DIST_PX` | vision_servo | 重捕获门限（默认 70） |
| `IDENTITY_MARGIN_PX` / `CONSENSUS_TOL_PX` | vision_servo | 身份核验（共识平移）参数 |
| `STOP_VOTE_FRAMES` | chassis_fsm | 连续多少帧检出杂草才停车（防抖） |
| `BLIND_ROLL_SEC` | chassis_fsm | 清完后盲走时长（甩出已清区，防重复停车） |
| `CRUISE_SPEED_MPS` | chassis_fsm | 巡航车速 |
| `EXG_FILTER_ENABLE` | config | ExG 假草过滤默认开关（运行时可经话题覆盖） |

---

## 11. 模型训练与转换（.pt → .onnx → .bin）

本项目 YOLOv8 模型经“训练 → 导出 ONNX → 地瓜工具链量化为 BPU bin”三步得到。仓库已附最终产物（`best.pt` / `best.onnx` / `quant.bin`），如需复现：

### 10.1 训练（PC，Ultralytics）
- 框架：Ultralytics YOLOv8，`imgsz=640`
- 类别：`nc=2`，`{0: weed（杂草）, 1: crop（作物）}`
- **数据要点**：除正样本外，**加入“空场景/土壤”负样本重训**，抑制空地误锁（假阳性）——这是本项目提升现场鲁棒性的关键。
- 参考指标：`mAP@0.5 = 0.913`，`mAP@0.5:0.95 = 0.665`，P=0.910，R=0.866。

### 10.2 导出 ONNX（保持标准检测头）
```bash
yolo export model=best.pt format=onnx opset=11 imgsz=640 simplify=True
```
- 保持 **Ultralytics 标准输出头**：输入 `images [1,3,640,640]`，输出 `output0 [1,6,8400]`，**不含 NMS**。
- ⚠️ **不要**采用部分模型仓库“修改输出头分离 bbox/cls”的导出方式——那会改变输出结构，与本项目 `yolo_detector.py` 的后处理不兼容。

### 10.3 量化为 BPU bin（地瓜 OpenExplorer / hb_mapper）
使用地瓜（D-Robotics）工具链，配置见 `models/config_yolov8.yaml`。**关键量化参数（本项目实测）**：

| 项 | 值 |
|---|---|
| `march`（芯片架构） | `bayes-e` |
| `input_type_rt` | `nv12` |
| `input_type_train` | `rgb` |
| `norm_type` / `scale_value` | `data_scale` / `0.003921568627451`（=1/255） |
| **校准数据** `calibration_data` | `calibration_images/`（现场采集，`calibration_size=100` 张） |
| `calibration_type` | `default` |
| `optimization` | `set_Softmax_input_int8,set_Softmax_output_int8` |
| `compile_mode` / `optimize_level` | `latency` / `O3` |
| 精度 | `int8` |

- **校准数据**：取约 100 张**与部署场景一致**的现场图（含杂草/作物/空地），量化时对激活值做统计定标。校准图应覆盖实际光照/距离/背景，直接影响量化精度。
- 量化质量：`quant_info.json` 记录逐层余弦相似度；本项目 `output0` 余弦相似度约 **0.9991**，行为与浮点模型高度一致（负样本“看到空地不误报”的成果完整保留）。
- 产物：`quant.bin`（约 5.9 MB），输入 NV12 `1x3x640x640`，输出 FP32 `[1,6,8400]`。

> 转换流程可在地瓜官方云端工具或本地 OpenExplorer Docker 中完成，具体见地瓜开发者社区文档。

---

## 12. 在 RDK X5 BPU 上部署模型

1. **放置模型**：将 `quant.bin` 置于包内 `models/` 目录。`yolo_detector.py` 通过 `config.py` 指定路径加载，默认：
   ```
   /home/sunrise/yahboomcar_ws/src/laser_calibration/models/quant.bin
   ```
   （用户名/路径不同请改 `config.py`。仅换模型、代码不变时，替换该文件即可，无需重新 `colcon build`。）
2. **BPU 推理**：`yolo_detector` 使用地瓜 `hobot_dnn` 加载 bin 于 BPU 运行；输入需转为 **NV12 640×640**，输出 `[1,6,8400]` 经节点内后处理（解码 + 阈值/NMS）得到检测框。
3. **启动**：
   ```bash
   ros2 run laser_calibration yolo_detector
   # 日志出现 [BPU] frame=... inference=~29ms boxes=N 即部署成功
   ```
4. **校验**：`md5sum models/quant.bin` 与记录值一致，确保模型未损坏/未被旧文件覆盖。

---

## 13. 性能指标

| 指标 | 数值 |
|---|---|
| 检测 mAP@0.5 / @0.5:0.95 | 0.913 / 0.665 |
| 端侧推理延迟 / 帧率 | ≈ 29 ms/帧 / ≈ 34 FPS（RDK X5 BPU） |
| 激光对准精度（中位数） | ≈ 5 px（静止）/ ≈ 6 px（移动） |
| 对准精度典型范围 | 2–12 px（接近 1° 舵机量化地板 ~5px） |
| 单株清除周期 | ≈ 5–6 s |
| 多目标清除成功率 | 100%（静止）/ ≥90%（移动），优化布置下 |
| 鲁棒机制 | 失败自动重试；共识平移身份核验实测拦截重复打击 |

> 指标基于多轮台架（静止）与行进（移动）测试统计。

---

## 14. 已知局限与改进方向

- **打击点为整株包围盒中心**：当前检测输出整株杂草框，激光打植株中心，可能落于叶片而非茎基生长点；后续可引入**关键点检测**定位茎基/生长点，提升清除致死效率。
- **视角泛化**：不同视角/光照下存在漏检；可扩充多视角数据重训，或引入**持续多帧跟踪（SORT/DeepSORT）**稳定目标 ID。
- **量化死区固定**：锁定死区未随工作距离自适应；可做在线可调或随“每度像素量”自适应。
- **离散作业**：目前为 Stop-and-Go；可升级为行进中连续伺服打击。
- **激光功率**：原型使用低功率 405nm + 感光纸验证，非真实除草功率。

---

## 15. 许可证

本项目基于 [MIT License](./LICENSE) 开源。你可自由使用、修改、分发，但需保留版权与许可声明。作品为竞赛用**概念验证原型**，作农业机器人视觉伺服与边缘 AI 的教学/科研参考。
