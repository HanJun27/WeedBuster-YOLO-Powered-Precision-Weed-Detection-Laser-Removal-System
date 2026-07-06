# laser_calibration v3.11.1 —— 多目标鲁棒性 + 显示修复 + ExG 运行时开关

> 基于 v3.11.0(已含负反馈新模型)。模型文件**未改**,仍是 mAP50=0.913、量化余弦 0.9991
> 的新模型。本版**只改代码**,做三件事(均已离线自检 + 逻辑用例验证)。

## 三项改动

### ① 身份核验式重捕获(解决"重打已打目标 / 抓旁株",顺带放宽门限)
- 重捕获/跟踪选框时,把 planner 下发的**已打 + 其它待打**目标(中心参考系坐标)按 anchor
  平移到当前画面得到各自预测位(eye-in-hand:相机+激光固定在云台,云台一转全场同量平移;
  红斑+Δ 吸收盲跳残差)。
- **anchor 附近唯一候选** → 收(除非它明显落在某个其它目标上 → 拒);
  **多个候选都逼近 anchor(歧义)** → 用**共识平移**消歧:对每个候选设 e2=框−anchor,看该
  e2 下其它目标预测位是否各有"另一个"框证实——正确框的 e2 能同时对齐全场,错框(如已打#1)
  对不齐 → 选证实最多者;全无证实则**安全拒打**(不重打/误打)。
- 重捕获门 `REACQ_MAX_DIST_PX` **50→70**:有身份核验兜底,放宽是安全的,解决孤立目标
  "差点打到却因门限放弃"。
- planner 同步:`strike_cmd` 新增 `others` 字段(本片其它待打目标坐标)。
- **离线验证(关键)**:本版早期曾用"逐框最近预测"简单版,离线用例 **S1**(三目标:打完①去
  打②、残差大使①的框逼近 anchor)复现了**它会误选已打①=重打**;故升级为共识平移版,S1 及
  另外 4 个场景(孤立目标/遮挡/三目标/单目标片)全部通过。
- 诚实边界:若当前目标本帧**被遮挡未检出**、且某已打目标恰好落到 anchor 附近、画面里又没有
  其它框可旁证 —— 单帧视觉无法分辨,会退化(后果是**漏打**当前目标,不是危险动作)。根治靠
  重标 PIXEL_TO_DEG/Δ 把残差压小。日常多目标(其它目标在画面里)已被共识版正确处理。

### ② 显示修复(蓝框/靶点框"跑出 YOLO 框"的观感问题)
- 根因:v3.10.13/14 在"本帧无检测"时粗暴清空青框,**掀开了 yolo_target 的固有跟踪滞后**
  (丢帧时蓝框钉在原地、草在滑动),显得"靶点跑出框"。打击本身不受影响(走红斑+PID 闭环)。
- 改法:`detected=False` 不再清青框,**保留上一帧框作"上次检测"**,前端按年龄画:
  **<0.25s 实线青 / 0.25~1.5s 虚线灰+"陈旧Xms" / >1.5s 不画**;**靶点框按新鲜度绿(新鲜)/灰(陈旧)**。
  既不残留"看着像实时"的假框误导演示,也保留视觉参照,不再误判打偏。

### ③ ExG 假草过滤运行时开关(前端→后端→话题整条接通)
- yolo_detector 订阅 `/yolo/exg_enable`(Bool),`filter_boxes_by_exg(enable=...)` 即时生效;
  vision_servo 8093 网页新增 **[ExG: 开/关]** 按钮 → 发布到该话题。**无需重启**即可现场
  A/B 对比"开/关 ExG",并验证新模型是否真不空锁。初值仍取 config 的 `EXG_FILTER_ENABLE`。

---

## 模型文件(未改,供核对仍是新模型)
| 文件 | md5 | 字节 |
|---|---|---|
| `models/quant.bin` | `b7690e8d48fca3ab961f6a291b0f3ca7` | 5,901,691 |
| `models/best.pt` | `e292c0377724bf2c3c8bb01d5b9aa14a` | 6,268,388 |
| `models/best.onnx` | `04e6b769d616f8ab16aa2fe15464e18e` | 12,266,036 |
| `models/quant_info.json` | `2ad2520fd8fafa94d394318380efaf2c` | 72,639 |

---

## 部署(本版改了代码,需标准重建;不能只换模型)

```bash
# 0) 清进程
pkill -9 -f "stereo_camera|yolo_detector|vision_servo|strike_planner|chassis_controller|Mcnamu_driver|yahboom_keyboard"
# 1) 解压
cd /home/sunrise && unzip -o handover_v3.11.1.zip -d hv3111
# 2) 替换源码包
cd /home/sunrise/yahboomcar_ws/src && rm -rf laser_calibration
unzip -o /home/sunrise/hv3111/laser_calibration_v3.11.1.zip
# 3) 干净重建(不带 --symlink-install)
cd /home/sunrise/yahboomcar_ws
find src/laser_calibration -name "*.egg-info" -exec rm -rf {} + 2>/dev/null
rm -rf build/laser_calibration install/laser_calibration
colcon build --packages-select laser_calibration
source install/setup.bash
# 4) 验证
ros2 pkg executables laser_calibration | wc -l    # 应=12
python3 -c "import laser_calibration; print(laser_calibration.__version__)"   # 应=3.11.1
md5sum src/laser_calibration/models/quant.bin     # 应=b7690e8d48fca3ab961f6a291b0f3ca7
```

## 运行(同前;ExG 开关在 8093 网页)
节点起停顺序不变:`Mcnamu_driver → stereo_camera → yolo_detector → vision_servo →
strike_planner → chassis_controller`,每个终端先 `source ~/yahboomcar_ws/install/setup.bash`。
触发:`/chassis/start` 发车、`/chassis/stop` 收工、`/safety_stop` 急停、
`/planner/start_clearing` 台架单独清场。

## 上车后逐项验证本版三件事
1. **ExG 开关**:打开 8093 网页 → 点 **[ExG]** 按钮,看它在"开/关"间切换;`yolo_detector` 终端
   会打印 `[ExG] 运行时开关 → 开启/关闭`。**关掉 ExG**,镜头对空地/纯纸面 → boxes 应稳定为 0
   (验证新模型本身不空锁,ExG 多余);对真草 → 正常框出。
2. **显示**:故意让目标短暂离开/遮挡 → 青框应变**虚线灰 + "陈旧Xms"**(而非消失),靶点框变灰;
   恢复检测后变回实线青/绿。不再出现"蓝框无端跑出框"的误判。
3. **身份核验(多目标)**:摆 2~3 株靠近的纸草跑一圈清场,盯 `vision_servo` 终端:正常时打
   `[REACQ] 锚点重捕获...`;若某帧候选与当前目标对不上会打 `[身份核验] ...不更新靶点`。
   重点确认**不再出现"打完一株又回去打同一株"**。建议用 `analyze_strike_logs.py` 复核成功率/
   是否有重复 id。

## 可调参数(vision_servo.py 顶部)
- `REACQ_MAX_DIST_PX = 70`(重捕获门,孤立目标放宽用)
- `CONSENSUS_TOL_PX = 20`(共识平移证实容差;现场框抖大可略放宽)
- `IDENTITY_MARGIN_PX = 15`(保留)
- ExG 各阈值与默认开关仍在 `config.py` 顶部 `EXG_*` 段。
