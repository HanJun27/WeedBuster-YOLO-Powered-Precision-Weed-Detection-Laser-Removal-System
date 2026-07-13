#!/usr/bin/env python3
"""
vision_servo.py —— Phase 3：视觉伺服闭环打击  v3.13.0
=====================================================
v3.13.0（任务面板:8093 升格为整机演示驾驶舱;全部为显示/控制面薄层,打击链零改动）：
  ① 任务控制栏:🚗发车(/chassis/start)·⏸收工(/chassis/stop)·🛑急停(/safety_stop,
     同时本地立即停光停伺服) 三按钮 + 底盘/决策实时状态灯(订阅 /chassis/state)。
  ② 🌱 相对绿度健康指数:1Hz 对 crop 类框算 ExG 均值(纸靶演示可用;NDVI 对打印
     纸无近红外反射,真植物场景请看 8080/8081 的 NDVI 流),折线+相对基线Δ%。
  ③ 📜 整片作业历史:订阅 /planner/patch_clear,逐片记录 清除/失败/命中统计。
=====================================================
v3.12.0（作业统计全管线 + 全场RANSAC全局关联[默认关]；打击几何/闭环零改动）：
  ① 作业统计：每发打击(成功/失败)记录 → 内存(网页📊面板:成功率/命中率/平均误差/
     单株耗时/逐发偏差折线) + 落盘 ~/strike_logs/*.jsonl(答辩数据,analyze_strike_logs
     离线出报告)。记录点在 _publish_strike_result 汇聚漏斗(rejected 不算发),纯观测。
  ② 全场 RANSAC 全局关联(REACQ_GLOBAL_ASSOC,默认 False=行为与 v3.11.2 逐字一致)：
     身份核验从"anchor 局部共识"升级为"全账本×全候选 1点RANSAC 估全局平移 + 贪心
     一对一分配"。当前目标被遮挡而已打株挤进 anchor 时(v3.11.1 承认的诚实边界),
     全局版能用任意旁证框(或平移先验 tie-break)判"该框属于已打株"→安全拒,不重打。
     开关只切换"选哪个框"这一步;共识版代码原样保留,现场 A/B 后决定是否启用。
=====================================================
v3.11.2（蓝斑命中判定 + 决策队列可视化；打击几何/闭环/身份核验零改动）：
v3.11.2（蓝斑命中判定 + 决策队列可视化；打击几何/闭环/身份核验零改动）：
  ① 蓝斑命中判定(纯观测)：打印靶无烧痕可验,但点火 1s 里蓝紫光斑就在纸上、相机
     看得见。灼烧段的 wait 切成 ~80ms 小片,片间采帧用 find_blue_spot(B−max(G,R),
     与红斑同管线)在瞄准点 ROI 检蓝斑,中位位置−瞄准点 ≤HIT_TOL_PX 判命中,
     检出帧不足判"未检出"(弃权,不算脱靶)。结果只进日志/strike_result 附加字段/
     网页战报,绝不改 FSM、不触发重打;_fire_cancel 中止语义与 v3.10.6 逐字一致。
     HIT_CHECK_ENABLE=False 完全恢复 v3.11.1 点火行为。
  ② 决策队列可视化：订阅 /planner/session_state(strike_planner v0.6 广播),
     侧栏列出 已打(含命中判定)/打击中/待打/放弃;画面叠加按 session_shift 平移
     的目标标记(云台居中→(0,0);伺服中→靶点−锁存,与身份核验同一个量;坐标系
     未知→不画叠加只留侧栏,绝不画错图)。全部纯显示。
=====================================================
v3.11.1（多目标鲁棒性 + 显示修复 + ExG 运行时开关；eye-in-hand：相机固定在云台、
  红/蓝激光固定在相机上表面，三者随云台同转）：
  ① 身份核验式重捕获(CMC + 数据关联,业界 BoT-SORT 同思路)：重捕获/跟踪选框时,把
     planner 下发的"已打 + 其它待打"目标(中心参考系)按 anchor 平移到当前画面得到各自
     预测位(平移量 = 锚点 − 本目标中心参考坐标；红斑+Δ 吸收盲跳残差)。anchor 附近唯一
     候选→收(除非明显落在某个其它目标上);多个候选逼近 anchor(歧义)→用【共识平移】消歧:
     对候选设 e2=框−anchor,看该 e2 下其它目标预测位是否各有"另一个"框证实(eye-in-hand
     全场同量平移,正确框的 e2 能同时对齐全场,错框对不齐)→ 选证实最多者,全无证实则安全
     拒打。替换了 v3.10.11 的固定半径"已打排除"(及本版早期"逐框最近预测"简单版——后者在
     残差大且目标相邻时会把已打#1误判为当前目标致重打,离线用例 S1 已复现并验证共识版修正)。
     重捕获门 REACQ_MAX_DIST_PX 50→70:有身份核验兜底,放宽安全(解决孤立目标"差点打到却放弃"),
     近距离歧义安全拒打,不重打/误打。planner 同步新增 strike_cmd "others" 字段下发其它待打目标。
  ② 显示修复：detected=False 时不再粗暴清 _yolo_boxes(那曾掀开 yolo_target 固有跟踪
     滞后,显得"靶点跑出框")。改为保留上一帧框作"上次检测",前端按年龄画:<0.25s 实线青
     / 0.25~1.5s 虚线灰+"陈旧Xms" / >1.5s 不画;靶点框按新鲜度绿(新鲜)/灰(陈旧)。
  ③ ExG 假草过滤运行时开关：网页[ExG]按钮 → 发布 Bool 到 /yolo/exg_enable →
     yolo_detector 订阅即时开/关(无需重启),便于现场 A/B 对比与验证新模型是否真不空锁。
  注:打击决策的几何/闭环未变,本版只改"选哪个框(身份核验)"、"怎么显示"、"ExG 开关"。
=====================================================
v3.10.14: 修网页检测框残留 —— detected=False 时回调原本直接 return,清空
  _yolo_boxes 的代码在函数底部够不到,导致上一帧的框一直留在画面不消失。
  改为早退前清空显示框(非伺服期一并清 yolo_target 让蓝框也消失);伺服中
  (PID/LOCKED)只清显示框、保留 yolo_target,不打断正在进行的闭环。
  注:此为纯显示层问题,打击决策走 yolo_target/planner 队列,从不受 _yolo_boxes 影响。
  (v3.11.1 已改为保留陈旧框+标注,见上。)
=====================================================
v3.10.12（相对 v3.10.11，配合新增 chassis_controller 打通"走—停—清—走"）：

  Ⓗ 新增 /safety_stop 全局急停话题（Empty）：与网页[紧急停止]同路径
     （激光立即灭 + 点火中止 + 命令漏斗复位）。chassis_controller 订阅
     同一话题自行停车 —— 视觉互锁/物理按钮桥接等任何安全源发一条消息，
     轮与光同时止动。
  Ⓘ 修 0612 实测暴露的捕获口子：伺服中重捕获那一帧若红斑恰好未检出，
     旧版退到"画面中心锚点 + 无门限"（41 发里出现 1 次 d=83px 的越门限
     捕获，id=37）。改为：伺服中无红斑则本帧不捕获，等光斑出现。

v3.10.11 修复（相对 v3.10.10，修台架实测的"打二又打回一"重复打击）：

  Ⓕ **真 Bug** ── 重捕获锚点偏了 Δ（≈56px）
     盲跳后草的预测位置在【蓝光落点 = 红斑 + Δ】，旧版锚在红斑本身，
     系统性偏 Δ；两株草靠近时"最近邻"经常翻到旁边那株（尤其刚打完的），
     一旦抓错、邻域跟踪反而咬死 → 同株重复烧、planner 还误记 success。
     修复：锚点改为 calib.spot_to_hit(红斑)（即红斑+Δ）。
  Ⓖ **选框三层防护**
     ① label/conf 过滤：执行层只瞄 weed 且 conf≥TRACK_MIN_CONF
       （旧版不滤——理论上锚点可能抓到 crop 框，安全隐患一并堵上）；
     ② 距离门限：重捕获 REACQ_MAX_DIST_PX / 跟踪 NEIGHBOR_MAX_JUMP_PX，
       目标本帧闪烁漏检时宁可不更新、绝不抓错株（兜底 PID_TIMEOUT→重试）；
     ③ 已打排除：planner 随 strike_cmd 下发本片已打坐标（中心参考系），
       执行层按"锚点−锁存坐标"求两坐标系平移、换算到当前画面后，
       STRUCK_EXCLUDE_PX 邻域内的框一律不选 → 已烧的草物理上选不中。
  新增 [REACQ]/[选框] 日志，现场可直接观测每次重捕获选了哪个框、距离多少。
  回退不变（exclude 字段缺失时执行层向后兼容）。

v3.10.10 修复（相对 v3.10.9，全部围绕"绝对角只认中心参考系坐标"这一约束）：

  Ⓐ **P0a 真 Bug** ── COARSE 绝对角盲跳吃到"上一姿态"的 live YOLO
     不归中模式下打完→IDLE 间隙 _cb_yolo 仍更新 yolo_target（歪角姿态像素），
     其 0.5s 新鲜窗 > 预热 0.2s + tick 0.1s + 指令时延 → compute 阶段的
     _refresh_required_spot 几乎必然采信它，绝对角公式按中心参考系解读歪角
     坐标 → 第二株起盲跳必偏。修复：不归中模式 compute 不再刷新，
     required_spot 固定用 _start_servo 锁存的中心参考坐标。
  Ⓑ **P0b 真 Bug** ── PID 首拍 fallback 到中心参考坐标
     settle 清掉 yolo_target 后、红斑锚点重捕获前，_refresh_required_spot
     会 fallback 到锁存坐标（中心参考系）→ 首拍按错坐标系甩一步（最大 5°≈50px），
     可能把红斑甩到别株草旁、锚点抓错。修复：①不归中模式下 PID 等
     yolo_target 重捕获再闭环（无检测由 PID_TIMEOUT_SEC 兜底）；
     ②锁存 fallback 仅限归中模式，不归中时沿用上一次 required_spot。
  Ⓒ **P2** ── 不归中模式下云台不在中心时，拒绝手动/auto 触发（live 像素
     不在中心参考系），提示先归中；planner 指令不受限。
  Ⓓ **P1** ── 新增 /servo/recenter 话题（Empty）：planner 建队前请求归中，
     行为与 /api/center 一致（立即 set 点火中止 + 经命令漏斗归中复位）。
  Ⓔ **P3** ── 不归中收尾日志文案更正（绝对角方案本身即坐标补偿）。
  回退不变：RECENTER_AFTER_FIRE=True + EXECUTOR_RECENTERS=True 仍完整恢复
  v3.10.8 行为（上述 Ⓐ–Ⓒ 的新分支只在不归中模式生效）。

v3.10.7 修复 / 重构（相对 v3.10.6）：

  ⓭ **真 Bug** ── _detect_spot_now 检测失败时不再留下陈旧 current_spot
     旧逻辑：激光开着但 ROI + 全画面都没找到光斑时，函数 return None，却
     **不更新 self.current_spot**，于是它保留上一帧的旧值。_step_pid 的
     `if current_spot is None` 判空因此失效，PID 会拿一个陈旧光斑算误差、
     下发一条错误的舵机命令。修复：硬失败路径显式置 current_spot=None，
     让 PID 在该帧 reset + 跳过，而不是用脏数据。

  ⓮ **并发** ── HTTP 控制指令改为"命令漏斗"汇入 FSM 线程，消除竞态
     旧结构：HTTP 服务在独立线程里直接改 fsm_state / required_spot /
     _locked_yolo_target 等多字段，与 10Hz FSM timer（ROS 执行器线程）
     并发——/api/go 与 /api/center 的多字段写有可能被 timer 读到"半更新"
     的中间态（一拍的瞬态毛刺）。改法：/api/go /api/stop /api/center 不再
     直接改状态，而是把指令压入线程安全队列，由 FSM timer 在每拍开头统一
     drain 执行——**所有 FSM 状态变更收敛到 timer 单线程**（ROS 回调本就与
     timer 互斥，安全）。紧急停止/归中仍**立即**关激光 + set 点火中止事件
     （安全动作不等下一拍），只把状态复位部分延后到 timer。

  ⓯ **安全一致性** ── 手动 [S3 测试烧] 也改为可中断
     _fire_test_thread 旧版用阻塞 time.sleep，紧急停止期间不会提前结束
     （虽然 all_lasers_off 已立即灭灯、无重燃风险，但行为不统一）。改为
     _fire_cancel.wait()，与 v3.10.6 的点火序列一致。

  ⓰ **几何更正 + 归中** ── 相机 + 激光同在云台上（一起随云台转）
     上一轮误以为"相机在车身"而改错了注释和加了"按云台角预测光斑"的 hint，
     现已全部更正/撤销：
       · _step_coarse 注释改回"相机+激光同在云台"模型（云台转 → 画面平移）。
       · _detect_spot_now 的 ROI hint 改回 last_valid_spot / SPOT_HOME
         （删掉错误的 _predicted_spot_for_angle）。
       · RECENTER_AFTER_FIRE 仍是开关，但理由更正为：相机在云台 → 云台一转
         画面平移 → 多目标队列（建在参考位）失效，故**打完必须归中回参考位**，
         队列坐标才仍有效。**相机在云台时归中是必需项，不是省时间可关的开关。**
     ⚠️ 同时暴露一个待确认项：SERVO_FREEZE_TARGET（冻结目标）只对"相机固定
        车身"正确；相机在云台时伺服中相机平移、目标在画面里移动，理论上应追
        live 目标（=False）。未擅改默认，详见 README §A。

  ⓱ 杂项：/api/pid_reset 删除重复 return 死代码；HTML 标题版本号刷到 3.10.7。

  说明：PIDController（量化感知整数步进）、点火序列安全模型、weed/crop 决策
        都未改。本次只动 vision_servo 的检测健壮性、并发收敛、几何注释/归中理由。

────────────────────────────────────────────────────────
v3.10.6 修复（相对 v3.10.4）：

  ❾ **安全关键** ── 紧急停止(E-Stop) 不再被点火线程拦截
     旧 _fire_sequence 内部用 time.sleep(0.2 / 1.0 / 2.0) 阻塞，期间紧急停止把
     fsm_state 置为 IDLE 也无法叫醒点火线程，0.2s 沉降后线程一觉醒来仍会
     霸道地 _set_blue_laser(True) 重新点亮高功率蓝紫激光。修复：
       · 新增 threading.Event `_fire_cancel`，_emergency_stop / /api/center 触发时立即 set()
       · 点火线程把所有 sleep 改为 _fire_cancel.wait(timeout) —— 一被 set() 立刻返回
       · 每次 wait 之后双重确认：事件 set 或 fsm_state 已被外部改 → 走 _cleanup_aborted_fire
       · 中止时关激光 + 给 planner 回报 failed + 清运行态，但不强改 fsm_state（尊重已生效的外部置位）

  ❿ PIDController 移除"按时间过期"重置 ── 修死结
     STALE_THRESHOLD_SEC=0.5 在网络抖动 / settle 慢一点时会误判数据不连续，
     强行清零 integral 与 derivative，让 PID 退化为纯 P；UI 把 Ki/Kd 调高再持久化
     的话尤其难受。改为：积分/微分只在显式 reset()（每轮伺服开始）时清零，
     运行中 dt 不论多大都正常使用（量化 round() 决定何时收敛，时间不再二次判定）。

  ⓫ PIDController 死区不再 halve Kp ── 与量化设计冲突
     "误差 < deadband 时 kp_eff *= 0.5" 是 v3.9.9 给连续控制做的抗震荡补丁；但
     量化舵机下,round() 已经提供天然死区(任何 |输出| < 0.5° 被吸到 0),再砍一刀
     反而让 6 ≤ |error| < 12 的可达点被砍成 move==0 → 系统在量化台阶外提前锁死。
     修复：kp_eff = self.kp 始终。deadband 参数保留（兼容性），但不再影响输出。

  ⓬ 网页 /api/state ── 伺服期间送"冻结目标"而非 live YOLO
     SERVO_FREEZE_TARGET=True 时,PID 用的是启动瞬间锁存的目标；但 /api/state 的
     "yolo" 字段一直送 live yolo_target,前端画的目标标记会因 YOLO 33Hz 抖动疯狂跳，
     调参时被误导成"系统不收敛"。修复：服务期间(COARSE/PID/LOCKED/FIRING/COOLDOWN)
     "yolo" 送冻结目标；额外新增 "yolo_live" 与 "target_frozen" 字段透明化。

  ─── 关于 Gemini 提到的"多目标像素刻舟求剑"：
       前提是相机随云台动，但本项目相机固定在车身上(USB stereo,不上云台),整套
       PID 是 image-based visual servoing,误差闭环在图像像素域里,任何时刻云台
       绝对角度都不进入误差计算。weed B 像素永远有效,与打完 A 时云台停在哪儿无关。
       已逐行核对,无 Bug。

────────────────────────────────────────────────────────
v3.10.4 新增（相对 v3.10.3）：

  ❽ strike_planner 决策层接口（多目标打击）
     新增两个话题，让独立的 strike_planner 节点能驱动本节点逐个清场：
       · 订阅 /servo/strike_cmd  (String/JSON {id,x,y})
         —— planner 下发"打这个目标"，本节点立即对该 RGB 坐标启动一次打击。
       · 发布 /servo/strike_result (String/JSON {id,result,x,y,final_distance})
         —— 一次打击结束（成功/失败/被拒）时回报，planner 据此推进队列。
     _start_servo() 增加可选 target/strike_id 参数：不传=手动/auto（行为不变），
     传入=planner 指定目标。手动 UI、auto 触发完全不受影响。
     注意：用 planner 时建议把触发模式设为 manual，避免 auto 自己抢着打。

────────────────────────────────────────────────────────
v3.10.3 新增（相对 v3.10.2）：

  ❼ PID 调参持久化
     网页上每改一次 Kp/Ki/Kd（/api/pid），立即写入磁盘
     ~/.laser_calibration/pid_tuning.json。节点下次启动时自动加载该文件，
     没有文件则用 PID_KP/KI/KD_DEFAULT。新增 [恢复默认] 按钮 / /api/pid_reset
     可一键清除存盘、回到默认值。

────────────────────────────────────────────────────────
v3.10.2 重大变更（相对 v3.10.1，针对"1° PWM 舵机量化"硬件约束）：

  ❶ 量化感知整数步进控制器
     set_servo→set_pwm_servo 只能吃整数度（1°≈10px）。旧设计把 PID 浮点输出
     当连续角度下发，<0.5° 修正被舵机/SDK round 抹掉 → 永远收不进
     PID_TOLERANCE_PX=3 → 必定超时 FAILED。新设计：每步算"移动几个整数度"
     最能消误差，统一 round() 成整数度；双轴整数度移动量都为 0 即落在最近
     网格点（≤半个量化台阶≈5px）、无法再改善 → 这就是锁定判据。

  ❷ best 跟踪 + 回到最佳点：连续 PID_NO_IMPROVE_LIMIT 次命令未刷新最佳距离
     （标定误差导致的量化极限环）→ 回到本轮最佳舵机位置锁定。超时同样走此
     兜底，几乎不再出现 FAILED。

  ❸ COARSE 解钳：v3.10.0 误把 COARSE 套了 1.5°/次 钳位，现在做完整整数度移动。

  ❹ 伺服期间冻结 YOLO 目标（SERVO_FREEZE_TARGET）：打击时车/草静止，COARSE/PID
     用启动时锁存的目标，避免 33Hz live 框抖动让误差越过容差边界。

  ❺ Kp 默认覆盖：量化感知需 Kp≈0.8~1.0；config 给连续 PID 的小 Kp 在此几乎
     不动舵机。本文件用 PID_KP_DEFAULT 覆盖，Ki/Kd 默认 0。

  ❻ 清理：移除饱和跳帧逻辑；_pid_actively_moving 收窄为"命令后 settle 窗内"；
     每 tick 只检测一次光斑。

  说明：本文件基于 v3.10.1 演进，仍含 v3.10.1 的 /api/set_yolo_freq 路由
        （惰性、不影响 PID，YOLO 节点侧未接时无副作用，可忽略）。

────────────────────────────────────────────────────────
v3.10.1 修复（相对 v3.10.0）：

  ❼ 接上前端的 YOLO 发布频率滑块。/api/set_yolo_freq 之前无后端路由（直接
     落到 404），滑块完全无效，是个摆设。本版补了路由。
     —— 但要明确：vision_servo 是 /yolo/weed_detected 的【订阅方】，无法直接
        改 YOLO 的发布率。本版做法是把"期望频率"发布到指令话题 /yolo/cmd_freq
        （Float32，TRANSIENT_LOCAL QoS），由 YOLO 检测节点订阅后自行重建
        发布 timer。YOLO 节点侧的订阅【需要队友配合实现】，片段见文件末尾。
        在队友接好之前，滑块只会让 vision_servo 发指令，YOLO 实际频率不变。

v3.10.0 重大变更（相对 v3.9.1，针对 Codex 报告的三个结构性问题）：

  ❶ PID 现在输出"像素域"修正量，末端统一用 _pixel_to_angle_delta()
     乘 PIXEL_TO_YAW_DEG / PIXEL_TO_PITCH_DEG 转角度。
     —— v3.9.1 bug：PID 直接把 kp*ex 当度数下发，绕过了 PIXEL_TO_YAW_DEG = -0.10
        的反向标定，yaw 轴 PID 会朝粗对准的相反方向修正。

  ❷ PID 加 settle gate：发完一次舵机命令后，PID_SETTLE_TIME_SEC（默认 0.25s）
     内只观测误差、不下发新命令。观测包括误差更新、收敛判定。
     —— 解决 10Hz tick 比舵机响应快导致的"连续盲发累加过冲"。

  ❸ PIDController.reset() 把 last_error 置为 None，step() 首帧跳过 D 项。
     —— v3.9.1 bug：reset 后 last_error=0，首次 step(error=30) 会算出
        derivative = 30/0.1 = 300，Kd 加权后单 D 项就爆 LIMIT。

  ❹ 顺带做完上轮诊断里的「去 sleep」：_step_coarse 改成非阻塞三阶段状态机
     （laser_warmup → compute → settle），消除 time.sleep(0.4/0.2)。

  ❺ main() 切到 MultiThreadedExecutor。沿用默认 MutuallyExclusive callback
     group（不引入新的并发风险），主要好处是 HTTP 线程和 ROS 回调彻底解耦，
     callback 短暂阻塞也不会拖垮整条流水线。

  ❻ PID 输出限幅语义从「度」变为「像素」(PID_OUTPUT_LIMIT_PX=15)，旧的
     PID_OUTPUT_LIMIT 不再使用（保持 config 导入避免破坏其他包）。
     角度域加二次钳位 MAX_DELTA_DEG_PER_TICK=1.5° 作为防御。

────────────────────────────────────────────────────────
v3.9.1 修复（相对 v3.9.0）：
  * 加 [S3 ON] / [S3 OFF] / [S3 测试烧 0.5s] 手动按钮
  * 加「开环模式也自动开火」复选框
  * 加 _fire_test_thread 短时测试不进入伺服流程

v3.9.0 重大变更（相对 v3.8）：
  画面源切到 RGB 摄像头；HSV→R-max(G,B) 红色检测；
  PID 每帧重新读最新 YOLO target；启动自检
────────────────────────────────────────────────────────

工作原理：
  1. 订阅 YOLO 的 /yolo/weed_detected （RGB 坐标）
  2. 订阅 RGB 摄像头 /camera/rgb/image_raw
  3. 用标定二的 Delta_X/Y 反算"红光斑应到位置"：
       Required_Spot_RGB = Target_RGB - Delta_RGB
  4. 开环粗对准：按 PIXEL_TO_YAW/PITCH_DEG 比例转一次
  5. PID 闭环精对准：像素域 PID → 经 PIXEL_TO_*_DEG 转角度 →
                    带 settle gate 下发 → 收敛后开火
  6. 锁定后开 S3 蓝紫激光烧 1 秒

运行:
  ros2 run laser_calibration stereo_camera   # 前置
  ros2 run laser_calibration vision_servo

浏览器:
  http://localhost:8093
  http://<小车IP>:8093
"""

import json
import os
import subprocess
import threading
import time
from collections import deque                                # v3.10.7: 命令漏斗
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor          # v3.10.0
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile      # v3.10.1
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float32, String, Bool      # v3.10.10: Empty(归中话题); v3.11.1: Bool(ExG开关)

from laser_calibration.calib_io import load_calib
from laser_calibration.config import (
    FIRE_COOLDOWN_SEC, FIRE_DURATION_SEC,
    LASER_BLUE_ID, LASER_ON_ANGLE,
    PID_LOCK_FRAMES, PID_DEADBAND_PX, FSM_TICK_PERIOD_SEC,
    PID_TIMEOUT_SEC, PID_TOLERANCE_PX,
    PIXEL_TO_PITCH_DEG, PIXEL_TO_YAW_DEG,
    RED_DOMINANCE_MIN, RED_SPOT_AREA_MAX, RED_SPOT_AREA_MIN,
    RED_SPOT_PREFER_NEAREST_HINT,
    SPOT_REQUIRE_BRIGHT, SPOT_BRIGHT_MIN, SPOT_MIN_COMPACTNESS,
    RGB_DEVICE, IR_DEVICE, RGB_EXPOSURE,
    SPOT_CLOSE_KERNEL_SIZE, SPOT_ROI_SIZE,
    SPOT_JUMP_MAX_PX, SPOT_JUMP_TOLERATE_FRAMES,
    SERVO_AUTO_DEBOUNCE, SERVO_DEFAULT_MODE,
    SERVO_PITCH_CENTER, SERVO_YAW_CENTER,
    SPOT_HOME_TOLERANCE, SPOT_HOME_X, SPOT_HOME_Y,
    TOPIC_RGB, TOPIC_YOLO, TOPIC_EXG_ENABLE,
    EXG_FILTER_ENABLE,
    YOLO_FALLBACK_TO_LOCKED, YOLO_TARGET_FRESH_SEC,
    # v3.14.0: 端侧 ASR
    ASR_ENABLE, ASR_MODEL_DIR, ASR_NUM_THREADS,
    ASR_CMD_CONFIDENCE, ASR_DEV_MODE_MSG, ASR_RECORD_MAX_SEC,
)
# v3.14.0: ASR 引擎
from laser_calibration.asr_server import AsrEngine
# v3.10.2: 不再导入 PID_KP/KI/KD/PID_OUTPUT_LIMIT/PID_SATURATION_FRAMES
#   —— Kp/Ki/Kd 在量化感知控制器下另有默认值；饱和跳帧逻辑已移除。
from laser_calibration.robot_ctrl import (
    ROBOT_OK,
    all_lasers_off, center_servo,
    laser_blue, laser_ir, set_servo,
)

SERVO_HTTP_PORT = 8093

# ══════════════════════════════════════════════════════════════
#  v3.10.2 量化感知控制常量
# ══════════════════════════════════════════════════════════════
# 舵机经 set_pwm_servo 控制，只能接收整数度（1° 分辨率）。
SERVO_QUANT_PX      = 1.0 / max(abs(PIXEL_TO_YAW_DEG), 1e-6)   # 1° 对应像素台阶 ≈ 10px
SERVO_HALF_QUANT_PX = SERVO_QUANT_PX / 2.0                      # ≈ 5px：可达精度地板

# 量化感知控制器下 Kp≈1 才有效；config 给连续 PID 用的小 Kp 在这里几乎不动舵机。
# 故在此覆盖默认值。想持久化可同步改 config.py 的 PID_KP/KI/KD。
PID_KP_DEFAULT = 0.8     # 0.8~1.0：每步做（近似）一次到位的整数度移动
PID_KI_DEFAULT = 0.0     # 量化执行器上积分易引发极限环，默认关闭
PID_KD_DEFAULT = 0.0     # 量化误差信号上微分基本是噪声，默认关闭

PID_OUTPUT_LIMIT_PX  = 50.0    # PIDController 内部像素域限幅（实际由 MAX_DEG_PER_STEP 约束）
MAX_DEG_PER_STEP     = 5.0     # PID 单步最大整数度（安全钳位）
PID_NO_IMPROVE_LIMIT = 4       # 连续 N 次命令未改善 → 回到最佳点锁定（抗量化极限环）
PID_SETTLE_TIME_SEC  = 0.25    # 发完舵机命令后的沉降窗

SERVO_FREEZE_TARGET  = False   # v3.10.8: 改为 False —— 相机+激光都在云台上，伺服中
                               # 相机平移、目标在画面里随之移动；必须每帧追 live YOLO
                               # 目标，蓝光才落在杂草【当前】像素=真实世界方向。
                               # =True（冻结）会瞄到"目标起始那一帧的像素"，云台转过后
                               # 那个像素已对应别的世界方向 → 蓝光偏掉≈云台转过量；
                               # 且冻结时误差只随 ds/dθ 变、与按 dt/dθ 标定的 PIXEL_TO_*
                               # 不匹配 → 过冲。live 模式下主导动力学=dt/dθ，匹配标定，
                               # Kp≈0.8 一步到位。YOLO 短暂不新鲜时回退到锁存值（见
                               # YOLO_FALLBACK_TO_LOCKED）保证不丢目标。
                               # （=True 仅适用于相机固定在车身，本项目不是。）

# v3.10.7→v3.10.9: 每发打完后是否把云台归中（回参考位）。
# 相机 + 激光同在云台 → 云台一转整个画面平移，【像素】队列换个云台角就失效。
# v3.10.8 之前靠"每发归中"维持像素队列有效（True）。
# v3.10.9 起改用【绝对角】方案：planner 在中心参考位投票建队，vision_servo 收到
#   中心参考像素后换算成【云台绝对角】盲跳（见 _step_coarse 的"不归中模式"分支）。
#   绝对角与当前姿态无关 → 不需要归中，打完一发留在原地、直接斜跳下一株，演示更顺。
#   要求：COARSE 走绝对角盲跳（已随本开关自动切换）+ 标定 PIXEL_TO_*（live 闭环已间接验证）。
#   True  = 每发归中（回退到 v3.10.8 行为，COARSE 自动改回"按当前光斑相对移动"）。
#   False = 不归中 + 绝对角盲跳（v3.10.9 默认，多目标链式打击）。
RECENTER_AFTER_FIRE  = False

# 粗对准非阻塞计时
COARSE_LASER_WARMUP_SEC = 0.20
COARSE_SETTLE_SEC       = 0.40

# ── v3.10.11: 执行层选框防护(修"打二又打回一"/打重)──────────────
TRACK_LABELS         = {"weed"}  # 执行层只允许瞄 weed 框(安全关键:旧版选框不滤
                                 #   label,锚点最近邻理论上可能抓到 crop 框)
TRACK_MIN_CONF       = 0.40      # 跟踪/捕获置信度下限(略低于 planner 建队的 0.50,
                                 #   防伺服中目标置信度小幅波动导致丢跟)
REACQ_MAX_DIST_PX    = 70.0      # v3.11.1: 重捕获门 50→70。配合下方身份核验后放宽是安全的:
                                 #   即便已打/旁株落进门限,身份核验会按"离当前目标比离任何
                                 #   其它目标都近"判定并拒掉它。解决孤立目标"差点打到却放弃"。
NEIGHBOR_MAX_JUMP_PX = 65.0      # 跟踪门:相邻两帧靶点跳变超此值视为换了株草,丢弃该帧。
                                 #   须 > 单步最大 5°≈50px 的画面平移,否则会误拒真目标
STRUCK_EXCLUDE_PX    = 30.0      # (保留,身份核验未覆盖的回退场景仍可参考;v3.11.1 选框主路径
                                 #   已改为身份核验,不再依赖此固定半径)
# v3.11.1: 身份核验式重捕获(CMC + 数据关联)。把"已打 + 其它待打"目标按 anchor 平移到当前
#   画面得到其预测位;候选框须"离当前目标预测位(anchor)比离任何其它目标预测位都近,且领先
#   ≥ 此 margin"才确认为当前目标。否则(更近某个其它目标 / 差距不足 margin=歧义)一律拒收。
#   → 杜绝重打已打目标、抓到旁株;两株挨得比残差还近时安全拒打(不会误打)。
IDENTITY_MARGIN_PX   = 15.0
# v3.11.1: 共识平移消歧容差。多个候选框都落在 anchor 附近(歧义)时,用"共识平移"区分:
#   假设候选 b 是当前目标→平移 e2=b−anchor;若该 e2 下其它目标的预测位(other_pred+e2)
#   各有"另一个"检测框落在此容差内证实,则 b 确为当前目标(其它框的存在反证了平移正确)。
#   挑证实数最多者;无任何证实(歧义且无旁证)则安全拒打。eye-in-hand 下全场景同量平移,
#   故正确平移能同时对齐所有框,错框(如把已打#1当#2)的平移对不齐其它框 → 被排除。
CONSENSUS_TOL_PX     = 20.0

# ── v3.11.2: 蓝斑命中判定(纯观测,不回写任何控制状态)────────────────
# 打的是 A4 打印靶,低功率蓝紫激光不留烧痕 → 无法"烧后复检";但点火那 1s 里
# 蓝紫光斑本身就打在纸上、RGB 相机看得见。于是在 _fire_sequence 灼烧期间以
# ~12Hz 采帧,用 find_blue_spot(B−max(G,R),与红斑同一套管线)在瞄准点附近的
# ROI 里检蓝斑,取中位数位置与瞄准点比距离 → 判 命中/脱靶/未检出。
# ⚠️ 安全边界:这是【只读】功能 —— 判定结果只进日志/strike_result 附加字段/
#   网页显示,绝不改 FSM、绝不触发重打;采样循环用 _fire_cancel.wait() 切片,
#   紧急停止的唤醒语义与 v3.10.6 完全一致(一 set 立刻退出→关激光→中止清理)。
# ⚠️ 上车第一步:405nm 蓝紫光在便宜 RGB sensor 上的通道响应因模组而异,先按
#   README 的验证步骤用 [S3 测试烧] 看一次日志里的蓝斑检出情况再定阈值。
HIT_CHECK_ENABLE      = True    # 关掉=完全恢复 v3.11.1 点火行为(整段跳过)
BLUE_DOMINANCE_MIN    = 25      # B−max(G,R) 阈值。405nm 常带红响应 → 比红斑的 30 略松;
                                #   日志打 [HIT] maxBlueScore,据此调:检不出→调低(如 15),
                                #   误检蓝色杂物→调高(ROI 已物理隔绝远处干扰,一般不用)
HIT_TOL_PX            = 30.0    # 蓝斑中位位置−瞄准点 ≤ 此值判"命中"。按打印靶半宽定:
                                #   靶宽 ~80px 就设 35~40;宁严勿松,视频里"命中"要有说服力
HIT_SAMPLE_PERIOD_SEC = 0.08    # 灼烧期采样间隔(1s 灼烧 ≈ 12 帧)
HIT_MIN_FRAMES        = 2       # 检出帧数 < 此值 → 判"未检出"(unseen),不判脱靶——
                                #   相机看不见蓝斑是传感器问题,不能冤枉打击链路

# v3.11.2: 决策层可视化 —— strike_planner 广播会话状态(队列/当前/已打/失败),
#   本节点订阅后透传给 8093 网页,叠加画到画面上(纯显示,不参与任何决策)。
#   ⚠️ 与 strike_planner.py 的同名常量字符串必须一致。
TOPIC_PLANNER_SESSION = "/planner/session_state"

# ── v3.12.0: 作业统计(纯观测) ────────────────────────────────────
STRIKE_LOG_ENABLE = True          # 每发打击落盘 JSONL(内存统计不受此开关影响)
STRIKE_LOG_DIR    = "~/strike_logs"   # 每次节点启动新开一个 strike_YYYYmmdd_HHMMSS.jsonl
SHOT_MEM_MAX      = 400           # 内存保留的最近发数(网页面板/统计用)

# ── v3.12.0: 全场 RANSAC 全局关联(身份核验升级,默认关) ──────────
# False = 选框逻辑与 v3.11.2 完全一致(走原共识平移代码)。上车 A/B:
#   台架摆 3 靶复跑 v3.11.1 README 的验证 3,再置 True 重跑;全局版应在
#   "当前目标遮挡+已打株挤进 anchor"场景打出 [关联] 拒收日志而非重打。
REACQ_GLOBAL_ASSOC = False

# ── v3.13.0: 任务面板(全部纯显示/薄控制面) ──────────────────────
# ⚠️ 与 chassis_controller.py 的同名常量字符串必须一致。
TOPIC_CHASSIS_START  = "/chassis/start"
TOPIC_CHASSIS_STOP   = "/chassis/stop"
TOPIC_CHASSIS_STATE  = "/chassis/state"
TOPIC_PATCH_CLEAR    = "/planner/patch_clear"
HEALTH_SAMPLE_SEC    = 1.0     # 健康指数采样周期
HEALTH_SERIES_MAX    = 900     # 序列最长(1Hz→15分钟)
PATCH_HISTORY_MAX    = 30      # 整片历史保留条数

# v3.10.3: PID 调参持久化文件 —— 网页改完参数自动存盘，下次启动自动加载
PID_TUNING_FILE = os.path.expanduser("~/.laser_calibration/pid_tuning.json")

# v3.10.4: strike_planner 决策层接口话题
#   ⚠️ strike_planner.py 里有同名常量，两边字符串必须一致。
TOPIC_STRIKE_CMD    = "/servo/strike_cmd"      # planner → 本节点：下发指定目标
TOPIC_STRIKE_RESULT = "/servo/strike_result"   # 本节点 → planner：回报打击结果
TOPIC_SERVO_RECENTER = "/servo/recenter"       # v3.10.10: planner → 本节点：归中
                                               #   （建队前回参考位，见 strike_planner）
TOPIC_SAFETY_STOP    = "/safety_stop"          # v3.10.12: 全局急停（任意安全源 → 本节点
                                               #   灭激光中止；chassis_controller 同时停车）

# v3.10.1: vision_servo → YOLO 检测节点的"期望发布频率"指令话题
# vision_servo 不产生 YOLO 检测，只能把期望频率发到这个话题，
# 由 YOLO 检测节点订阅后自行重建发布 timer（队友需在 YOLO 节点加订阅）
TOPIC_YOLO_FREQ_CMD = "/yolo/cmd_freq"
YOLO_FREQ_MIN       = 1.0
YOLO_FREQ_MAX       = 30.0
YOLO_FREQ_DEFAULT   = 10.0

# 状态机
STATE_IDLE       = "IDLE"
STATE_GOT_TARGET = "GOT_TARGET"
STATE_COARSE     = "COARSE"
STATE_PID        = "PID"
STATE_LOCKED     = "LOCKED"
STATE_FIRING     = "FIRING"
STATE_COOLDOWN   = "COOLDOWN"
STATE_FAILED     = "FAILED"


# ══════════════════════════════════════════════════════════════
#  v3.9.5 核心：R-max(G,B) + ROI 红光斑检测
#  v3.10.7：候选轮廓有多个时，给了 hint 则选离 hint 最近者（抗红色物体误检），
#           否则按面积最大。检测主算法（R-max(G,B) + 形态学闭运算）未改。
# ══════════════════════════════════════════════════════════════
def find_red_spot(bgr: np.ndarray, hint_x: int = None, hint_y: int = None):
    """红激光光斑检测，返回全图坐标 (cx, cy) 或 None。"""
    if bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]

    if hint_x is not None and hint_y is not None:
        half = SPOT_ROI_SIZE // 2
        y1 = max(0, int(hint_y) - half)
        y2 = min(h, int(hint_y) + half)
        x1 = max(0, int(hint_x) - half)
        x2 = min(w, int(hint_x) + half)
    else:
        y1, y2, x1, x2 = 0, h, 0, w

    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    b, g, r = cv2.split(roi)
    r_i  = r.astype(np.int16)
    g_i  = g.astype(np.int16)
    b_i  = b.astype(np.int16)
    max_gb = np.maximum(g_i, b_i)
    red_score = np.clip(r_i - max_gb, 0, 255).astype(np.uint8)

    _, mask = cv2.threshold(red_score, RED_DOMINANCE_MIN, 255, cv2.THRESH_BINARY)

    # v3.10.8: 亮度门 —— 只保留"又红又亮"的像素，剔除暗红背景(土壤/纸箱)。
    #   激光点(含红晕)很亮；背景暗红虽红但不亮。与红主导掩膜 AND 后再闭运算填白芯。
    if SPOT_REQUIRE_BRIGHT:
        bright = roi.max(axis=2)            # 每像素 max(B,G,R) = 亮度
        _, bmask = cv2.threshold(bright, SPOT_BRIGHT_MIN, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bmask)

    kernel = np.ones((SPOT_CLOSE_KERNEL_SIZE, SPOT_CLOSE_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    # 先按面积过滤出所有候选轮廓（连同质心）
    cands = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (RED_SPOT_AREA_MIN < a < RED_SPOT_AREA_MAX):
            continue
        # v3.10.8: 紧凑度(圆度)过滤 —— 激光点近似圆，长条形反光紧凑度低被剔除
        if SPOT_MIN_COMPACTNESS > 0:
            (_cx, _cy), _rad = cv2.minEnclosingCircle(c)
            circ_area = np.pi * _rad * _rad
            if circ_area <= 0 or (a / circ_area) < SPOT_MIN_COMPACTNESS:
                continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        lx = int(m["m10"] / m["m00"])
        ly = int(m["m01"] / m["m00"])
        cands.append((lx, ly, a))
    if not cands:
        return None

    # v3.10.7: 给了 hint 时选离 hint（=ROI 中心=预期光斑位置）最近的候选；
    #   否则（全画面兜底）按面积最大。前者对抗"ROI 里混进更大的红色物体"。
    if (RED_SPOT_PREFER_NEAREST_HINT and hint_x is not None
            and hint_y is not None):
        # hint 是全图坐标；候选质心是 ROI 局部坐标，换算到全图再比距离
        hx_local = int(hint_x) - x1
        hy_local = int(hint_y) - y1
        lx, ly, _ = min(
            cands,
            key=lambda t: (t[0] - hx_local) ** 2 + (t[1] - hy_local) ** 2)
    else:
        lx, ly, _ = max(cands, key=lambda t: t[2])

    return (x1 + lx, y1 + ly)


# ══════════════════════════════════════════════════════════════
#  v3.11.2: 蓝紫激光光斑检测（命中判定用，纯观测）
#  与 find_red_spot 同一套管线（主导通道差 + 亮度门 + 闭运算填过曝白芯 +
#  面积/紧凑度过滤 + 最近 hint 优先），只把红主导换成蓝主导 B−max(G,R)。
#  刻意不与 find_red_spot 合并成一个泛型函数 —— 红斑路径是 PID 闭环的命脉，
#  deadline 前不冒重构风险；克隆版怎么改都伤不到打击链路。
# ══════════════════════════════════════════════════════════════
def find_blue_spot(bgr: np.ndarray, hint_x: int = None, hint_y: int = None):
    """蓝紫激光光斑检测，返回 (cx, cy, max_score) 或 None。
    max_score = ROI 内 B−max(G,R) 的最大值（阈值调参观测用，日志会打印）。"""
    if bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]

    if hint_x is not None and hint_y is not None:
        half = SPOT_ROI_SIZE // 2
        y1 = max(0, int(hint_y) - half)
        y2 = min(h, int(hint_y) + half)
        x1 = max(0, int(hint_x) - half)
        x2 = min(w, int(hint_x) + half)
    else:
        y1, y2, x1, x2 = 0, h, 0, w

    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    b, g, r = cv2.split(roi)
    b_i  = b.astype(np.int16)
    g_i  = g.astype(np.int16)
    r_i  = r.astype(np.int16)
    max_gr = np.maximum(g_i, r_i)
    blue_score = np.clip(b_i - max_gr, 0, 255).astype(np.uint8)
    max_score = int(blue_score.max())

    _, mask = cv2.threshold(blue_score, BLUE_DOMINANCE_MIN, 255,
                            cv2.THRESH_BINARY)

    # 亮度门：激光斑（含蓝晕）很亮；暗蓝背景（阴影/蓝色印刷）虽蓝但不亮
    if SPOT_REQUIRE_BRIGHT:
        bright = roi.max(axis=2)
        _, bmask = cv2.threshold(bright, SPOT_BRIGHT_MIN, 255,
                                 cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bmask)

    kernel = np.ones((SPOT_CLOSE_KERNEL_SIZE, SPOT_CLOSE_KERNEL_SIZE),
                     np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    cands = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (RED_SPOT_AREA_MIN < a < RED_SPOT_AREA_MAX):
            continue
        if SPOT_MIN_COMPACTNESS > 0:
            (_cx, _cy), _rad = cv2.minEnclosingCircle(c)
            circ_area = np.pi * _rad * _rad
            if circ_area <= 0 or (a / circ_area) < SPOT_MIN_COMPACTNESS:
                continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        lx = int(m["m10"] / m["m00"])
        ly = int(m["m01"] / m["m00"])
        cands.append((lx, ly, a))
    if not cands:
        return None

    if hint_x is not None and hint_y is not None:
        hx_local = int(hint_x) - x1
        hy_local = int(hint_y) - y1
        lx, ly, _ = min(
            cands,
            key=lambda t: (t[0] - hx_local) ** 2 + (t[1] - hy_local) ** 2)
    else:
        lx, ly, _ = max(cands, key=lambda t: t[2])

    return (x1 + lx, y1 + ly, max_score)


# ══════════════════════════════════════════════════════════════
#  v3.12.0: 全场 2D 平移 1点RANSAC + 全局一对一分配(纯几何,可离线测试)
#  eye-in-hand 下云台一转全场同量平移 → 运动模型只有一个 2D 平移 e。
#  对(账本点×候选框)每一配对生成假设 e,数内点(账本预测位 tol 内有专属框),
#  取内点最多者(平手取离 anchor 先验 e0 最近者);再在最优 e* 下做贪心一对一
#  分配,看"当前目标"分到了哪个框。与共识版的区别:已打/待打株与候选框的
#  对应关系是全局一致解出来的,当前目标漏检时也能判"anchor 旁那框是已打株
#  的"→ 拒收;共识版此场景(无旁证框时)会退化(v3.11.1 README 诚实边界)。
# ══════════════════════════════════════════════════════════════
def associate_global(anchor, locked_ref, cand_pts, struck_ref, other_ref,
                     gate, tol):
    """返回 (chosen_idx or None, info)。chosen_idx 指向 cand_pts。
    anchor      : 当前目标在当前画面的预测位(红斑+Δ 或上帧靶点)
    locked_ref  : 当前目标中心参考系坐标
    cand_pts    : [(x,y)] 本帧过滤后的候选框中心(当前姿态坐标系)
    struck_ref / other_ref : 已打 / 其它待打目标(中心参考系)
    gate        : 平移修正量 |e−e0| 的信任域(用当前目标门限);None=不限
    tol         : 内点/分配容差(沿用 CONSENSUS_TOL_PX)"""
    ax, ay = float(anchor[0]), float(anchor[1])
    lx, ly = float(locked_ref[0]), float(locked_ref[1])
    e0 = (ax - lx, ay - ly)                 # anchor 先验平移
    ledger = [("cur", lx, ly)]
    ledger += [("struck", float(x), float(y)) for x, y in struck_ref]
    ledger += [("other",  float(x), float(y)) for x, y in other_ref]
    if not cand_pts:
        return None, {"reason": "no_cand"}
    hyps = [e0]
    for _, Lx, Ly in ledger:
        for bx, by in cand_pts:
            hyps.append((bx - Lx, by - Ly))
    best_key, best_e, best_assign = None, None, None
    for ex, ey in hyps:
        de0 = ((ex - e0[0]) ** 2 + (ey - e0[1]) ** 2) ** 0.5
        # 注意:信任域检查放在【接受】阶段而非此处 —— 在生成阶段就按 |e−e0|
        # 筛,会把"2+内点旁证的正确假设"错杀(先验偏大时),反而让贴着先验的
        # 1 内点错误假设胜出。多内点本身就是压倒先验的独立证据。
        pairs = []
        for li, (_, Lx, Ly) in enumerate(ledger):
            px, py = Lx + ex, Ly + ey
            for ci, (bx, by) in enumerate(cand_pts):
                d = ((bx - px) ** 2 + (by - py) ** 2) ** 0.5
                if d <= tol:
                    pairs.append((d, li, ci))
        pairs.sort()
        used_l, used_c, assign = set(), set(), {}
        for d, li, ci in pairs:             # 贪心一对一(近配优先)
            if li in used_l or ci in used_c:
                continue
            used_l.add(li); used_c.add(ci); assign[li] = (ci, d)
        key = (len(assign), -de0)
        if best_key is None or key > best_key:
            best_key, best_e, best_assign = key, (ex, ey), assign
    if best_key is None or not best_assign:
        return None, {"reason": "no_match", "e0": e0}
    info = {"e": (round(best_e[0], 1), round(best_e[1], 1)),
            "inliers": best_key[0],
            "shift_corr": round(-best_key[1], 1)}
    cur_m = best_assign.get(0)              # 账本 0 号 = 当前目标
    if cur_m is None:
        # 当前目标本帧无匹配(遮挡/漏检),而别的账本点认领了框 → 安全拒收
        return None, {**info, "reason": "cur_unmatched"}
    ci, d = cur_m
    _de0 = -best_key[1]                     # 先验修正量 |e*−e0|
    if best_key[0] >= 2:                    # 有旁证:全局一致
        # 信任域(宽,2×gate):旁证是压倒先验的证据,但修正量离谱仍拒 ——
        # 那意味着红斑/Δ 系统已彻底失灵,该走 PID 超时→planner 重试,不硬救。
        if gate is not None and _de0 > 2.0 * gate:
            return None, {**info, "reason": "consensus_beyond_trust"}
        return ci, {**info, "cur_d": round(d, 1), "mode": "consensus"}
    # 仅当前目标匹配、无旁证 → 退保守(等价于共识版单候选规则):
    #   信任域(严,gate):无旁证时只信先验附近的解
    if gate is not None and _de0 > gate:
        return None, {**info, "reason": "lone_beyond_trust"}
    bx, by = cand_pts[ci]
    da = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
    if gate is not None and da > gate:
        return None, {**info, "reason": "lone_beyond_gate"}
    opreds = [(x + best_e[0], y + best_e[1]) for _, x, y in ledger[1:]]
    if opreds:
        do = min(((bx - px) ** 2 + (by - py) ** 2) ** 0.5
                 for px, py in opreds)
        if do < d:
            return None, {**info, "reason": "closer_to_other"}
    return ci, {**info, "cur_d": round(d, 1), "mode": "lone"}


# ══════════════════════════════════════════════════════════════
#  v3.12.0: 作业统计聚合(纯函数,可离线测试)
# ══════════════════════════════════════════════════════════════
def compute_shot_stats(shots):
    """由打击记录列表算聚合统计。字段全部可能为 None(样本不足)。"""
    def _mean(vals):
        vals = [v for v in vals if v is not None]
        return round(sum(vals) / len(vals), 2) if vals else None
    n = len(shots)
    succ = [r for r in shots if r.get("result") == "success"]
    hit  = [r for r in succ if r.get("hit") is True]
    miss = [r for r in succ if r.get("hit") is False]
    judged = len(hit) + len(miss)
    return {
        "shots": n,
        "success": len(succ),
        "failed": n - len(succ),
        "success_rate": round(len(succ) / n, 3) if n else None,
        "hit": len(hit), "miss": len(miss),
        "unjudged": len(succ) - judged,
        "hit_rate": round(len(hit) / judged, 3) if judged else None,
        "avg_lock_err": _mean([r.get("final_distance") for r in succ]),
        "avg_hit_dist": _mean([r.get("hit_dist") for r in hit + miss]),
        "avg_duration": _mean([r.get("duration") for r in succ]),
    }


# ══════════════════════════════════════════════════════════════
#  v3.13.0: 相对绿度健康指数(纯函数,可离线测试)
#  对 crop 类框内像素算 ExG = 2G−R−B 的框均值再取平均。选 ExG 而非 NDVI
#  是演示现实:打印纸没有植被的近红外反射,NDVI 对纸靶恒≈0;真植物场景
#  的 NDVI 能力由 stereo_camera 的 8080/8081 页面承担。指数只做【相对】
#  趋势解读(同机位同光照下比变化),面板照此标注,不做绝对健康声明。
# ══════════════════════════════════════════════════════════════
def compute_green_index(bgr, boxes):
    """返回 (指数均值 or None, 参与框数)。boxes: [{cx,cy,w,h},...] 像素坐标。"""
    if bgr is None or bgr.size == 0 or not boxes:
        return None, 0
    h, w = bgr.shape[:2]
    vals = []
    for b in boxes:
        try:
            cx, cy = float(b["cx"]), float(b["cy"])
            bw, bh = float(b.get("w", 40)), float(b.get("h", 40))
        except (KeyError, TypeError, ValueError):
            continue
        x1 = max(0, int(cx - bw / 2)); x2 = min(w, int(cx + bw / 2))
        y1 = max(0, int(cy - bh / 2)); y2 = min(h, int(cy + bh / 2))
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue
        roi = bgr[y1:y2, x1:x2].astype(np.int16)
        exg = 2 * roi[:, :, 1] - roi[:, :, 2] - roi[:, :, 0]
        vals.append(float(np.clip(exg, 0, None).mean()))
    if not vals:
        return None, 0
    return round(sum(vals) / len(vals), 1), len(vals)


# ══════════════════════════════════════════════════════════════
#  PID 控制器
#  v3.10.0:
#    - output 现在是"像素域"修正量（调用方负责乘 PIXEL_TO_*_DEG）
#    - reset() last_error=None，step() 首帧跳过 D 项
#    - output_limit 单位是像素
# ══════════════════════════════════════════════════════════════
class PIDController:
    """单轴 PID。输入误差（像素），输出像素域修正量。

    v3.10.6 设计（修复 Gemini 报告的 Bug 3 / 4）：
      - first call (last_error is None): 仅 P 项输出，D=0，避免首帧 D 爆 LIMIT
      - 积分/微分**只在显式 reset() 时清零**（每轮伺服开始）；运行中 dt 多大都不重置
        （旧 STALE_THRESHOLD_SEC=0.5 会在 settle/抖动时误清零，让 PID 退化为纯 P）
      - **死区内不再 halve Kp**（旧逻辑与量化舵机冲突，会在台阶外提前锁死）
        死区由量化 round() 自然提供：|输出度| < 0.5° → move==0
      - saturated 标志：外层判断系统饱和并跳帧
      - deadband 参数保留（兼容旧调用），但 v3.10.6 起不再影响输出
    """

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float,
                 deadband: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit       # v3.10.0: 单位 = 像素
        self.deadband = deadband               # v3.10.6: 保留字段不再影响输出
        self.saturated = False
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = None   # v3.10.0: None 表示"未初始化"，step() 首帧跳过 D
        self.last_time = None
        self.saturated = False

    def step(self, error: float) -> float:
        now = time.time()

        # ─ 首帧（reset 后第一次 step）─ 仅 P，不算 D ─────────
        if self.last_time is None or self.last_error is None:
            self.last_time = now
            self.last_error = error
            return self._clamp(self.kp * error)

        # ─ 正常帧 ── v3.10.6: 不再做时间过期重置 ──────────────
        # 旧版 dt > 0.5s 会把 integral 和 derivative 清零（强制退化为 P）。
        # 实测中 settle(0.25s) + tick(0.1s) 已经接近门槛，又卡又看不出，弃用。
        # I/D 的"边界"由显式 reset() 在每轮伺服开始处划定，足够干净。
        dt = max(1e-3, now - self.last_time)
        derivative = (error - self.last_error) / dt
        self.last_time = now
        self.integral += error * dt
        self.integral = max(-100.0, min(100.0, self.integral))
        self.last_error = error

        # v3.10.6: kp_eff = kp 始终。死区由 round() 提供，不再 halve。
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return self._clamp(out)

    def _clamp(self, out: float) -> float:
        if abs(out) >= self.output_limit:
            self.saturated = True
            return max(-self.output_limit, min(self.output_limit, out))
        self.saturated = False
        return out


# ══════════════════════════════════════════════════════════════
#  HTML 页面
# ══════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>视觉伺服 · Phase 3 (v3.10.9)</title>
<style>
  body { background:#1a1a1a; color:#eee; font-family:monospace; margin:0; padding:14px; }
  h1 { color:#0f0; margin:0 0 10px; font-size:18px; }
  .panel { background:#222; padding:10px; border-radius:6px; margin-bottom:10px; }
  .btn { background:#2a2a2a; color:#0f0; border:1px solid #0f0; padding:6px 14px;
         cursor:pointer; font-family:monospace; font-size:13px; margin-right:6px;
         margin-bottom:4px; }
  .btn:hover { background:#0f0; color:#000; }
  .btn:disabled { color:#555; border-color:#555; cursor:not-allowed; }
  .btn.active { background:#0f0; color:#000; }
  .btn-fire { background:#3a1a1a; color:#f55; border-color:#f55; }
  .btn-fire:hover { background:#f55; color:#000; }
  .btn-stop { background:#3a0808; color:#ff0; border-color:#ff0; font-weight:bold; }
  .btn-stop:hover { background:#ff0; color:#000; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam-box { background:#000; border:2px solid #444; border-radius:4px;
             position:relative; width:640px; height:480px; }
  .cam-label { position:absolute; top:6px; left:8px; background:rgba(0,0,0,0.6);
               padding:2px 8px; color:#0f0; font-size:12px; z-index:10;
               pointer-events:none; }
  canvas { display:block; }
  .right-panel { width:480px; }
  .stat-box { background:#111; padding:8px; border-radius:4px;
              margin-bottom:6px; font-size:12px; }
  .stat-row { display:flex; justify-content:space-between; margin:2px 0; }
  .stat-key { color:#888; }
  .stat-val { color:#0f0; font-weight:bold; }
  .laser-state { display:inline-block; padding:3px 10px; border-radius:3px;
                 margin-right:8px; font-size:12px; border:1px solid #555; }
  .l-on { background:#330; color:#ff0; border-color:#ff0; }
  .l-fire { background:#600; color:#ff8; border-color:#f55;
            animation:blink 0.4s linear infinite; }
  @keyframes blink { 50% { opacity:0.4; } }
  .pid-chart { background:#0a0a0a; border:1px solid #333; }
  .group-title { color:#0f0; margin-bottom:4px; font-weight:bold; }
  .pid-input { background:#222; color:#0f0; border:1px solid #555; padding:2px 6px;
               width:60px; font-family:monospace; }
  .yolo-fresh-on  { color:#0f0; }
  .yolo-fresh-off { color:#fa0; }
  .info { color:#888; font-size:12px; margin-top:6px; }
  .settle-on  { color:#fa0; }
  .settle-off { color:#0f0; }
</style>
</head>
<body>
<h1>🎛 除草任务面板 · v3.13.0 (视觉伺服 · 决策可视化 · 作业统计 · 健康监测)</h1>

<!-- v3.13.0: 任务控制栏 -->
<div class="stat-box" style="display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin:8px 0">
  <button class="btn" style="background:#161;font-size:15px;padding:8px 22px" onclick="missionCmd('start')">🚗 发车(自动作业)</button>
  <button class="btn" style="background:#444;font-size:15px;padding:8px 22px" onclick="missionCmd('stop')">⏸ 收工</button>
  <button class="btn" style="background:#a11;font-size:15px;padding:8px 26px;font-weight:bold" onclick="missionCmd('estop')">🛑 急停</button>
  <span style="margin-left:8px">底盘 <span id="m-chassis" style="font-weight:bold;color:#888">--</span></span>
  <span>决策 <span id="m-planner" style="font-weight:bold;color:#888">--</span></span>
  <span>执行 <span id="m-servo" style="font-weight:bold;color:#888">--</span></span>
  <span>本场 <span id="m-brief" style="color:#0f0">0/0 发</span></span>
</div>

<!-- v3.14.0: 语音控制(ASR 端侧识别 + 操作员端 faster-whisper 备用) -->
<div class="stat-box" style="display:flex;align-items:center;gap:8px;margin:8px 0;flex-wrap:wrap">
  <button id="pttBtn" class="btn" style="background:#335;font-size:14px;padding:8px 20px;user-select:none"
          onmousedown="startRecord()" onmouseup="stopRecord()"
          ontouchstart="startRecord()" ontouchend="stopRecord()">🎤 按住说话</button>
  <select id="mic-select" style="background:#333;color:#0f0;border:1px solid #555;font-family:monospace;font-size:12px;max-width:220px" onchange="onMicChange()"></select>
  <select id="asr-backend" style="background:#333;color:#0f0;border:1px solid #555;font-family:monospace;font-size:12px" onchange="onAsrBackendChange()">
    <option value="rdk">🤖 RDK端侧 sherpa-onnx</option>
    <option value="local">💻 操作员端 faster-whisper</option>
  </select>
  <span id="local-asr-config" style="display:none;font-size:12px;color:#888">
    <button class="btn" style="padding:2px 8px;font-size:11px" onclick="browseAndLoadModel()">📂 选择模型目录</button>
    <span id="model-path-display" style="font-size:11px;color:#888"></span>
    <span id="model-status" style="font-size:11px"></span>
  </span>
  <span>语音 <span id="asr-status" style="color:#888">就绪</span></span>
  <span id="asr-result" style="color:#fa0;font-size:13px"></span>
  <button id="asr-log-btn" class="btn" style="padding:2px 8px;font-size:11px" onclick="toggleAsrLog()">📋 日志</button>
</div>

<!-- ASR 日志面板（隐藏，点击日志按钮展开） -->
<div id="asr-log-panel" style="display:none;background:#111;border:1px solid #444;border-radius:4px;padding:8px;margin:4px 0;max-height:300px;overflow-y:auto;font-size:12px;font-family:monospace">
  <div style="color:#0f0;margin-bottom:4px">ASR 识别日志 (最近 50 条)</div>
  <div id="asr-log-content" style="color:#aaa;line-height:1.6"></div>
</div>

<div id="calib-banner" style="display:none; background:#600; color:#fff; padding:10px 14px;
     border:2px solid #f55; border-radius:4px; margin-bottom:10px;
     font-weight:bold; font-size:14px;">
  <span id="calib-banner-text"></span>
</div>

<div class="panel">
  <span style="color:#888">触发:</span>
  <button class="btn" id="trig-manual" onclick="setTrigger('manual')">手动 Manual</button>
  <button class="btn" id="trig-auto" onclick="setTrigger('auto')">自动 Auto</button>

  <span style="margin-left:20px;color:#888">伺服模式:</span>
  <button class="btn" id="loop-open" onclick="setLoopMode('open_loop')">开环 (调试)</button>
  <button class="btn" id="loop-closed" onclick="setLoopMode('closed_loop')">闭环 PID</button>

  <button class="btn btn-fire" id="btn-go" onclick="action('go')" style="margin-left:30px">[开始打击]</button>
  <button class="btn btn-stop" onclick="action('stop')">[紧急停止]</button>
  <button class="btn" onclick="action('center')">[云台归中]</button>
  <button class="btn" id="btn-ir-on"  onclick="manualIR(true)" style="margin-left:10px">[S4 ON]</button>
  <button class="btn" id="btn-ir-off" onclick="manualIR(false)">[S4 OFF]</button>
  <button class="btn btn-fire" id="btn-blue-on" onclick="manualBlue(true)" style="margin-left:8px">[S3 ON]</button>
  <button class="btn btn-fire" id="btn-blue-off" onclick="manualBlue(false)">[S3 OFF]</button>
  <button class="btn btn-fire" onclick="testFire()">[S3 测试烧 0.5s]</button>

  <span style="margin-left:20px;color:#888">ExG假草过滤:</span>
  <button class="btn" id="btn-exg" onclick="toggleExg()">[ExG: ...]</button>
</div>

<div class="panel" style="font-size:12px">
  <label style="color:#ccc; cursor:pointer">
    <input type="checkbox" id="fire-open-cb" onchange="toggleFireOpen()" style="vertical-align:middle">
    开环模式也自动开火 <span style="color:#888">(默认禁用，调试期勾选可烧白纸)</span>
  </label>
</div>

<div class="row">
  <div class="cam-box">
    <span class="cam-label">RGB 画面 (红光斑+目标 可视化)</span>
    <canvas id="cv" width="640" height="480"></canvas>

      <!-- YOLO Detection Frequency Control -->
      <div style="margin:15px 0; padding:12px; background:#f0f8ff; border-radius:8px; border:1px solid #b0d4f1;">
        <label style="font-weight:bold; color:#0066cc; display:block; margin-bottom:8px;">📡 YOLO Detection Frequency:</label>
        <div style="display:flex; align-items:center; gap:10px;">
          <input type="range" id="freqSlider" min="1" max="30" value="10" 
                 oninput="document.getElementById('freqValue').textContent=this.value+' Hz'" 
                 onchange="setPublishFreq(this.value)"
                 style="flex:1; height:6px;">
          <span id="freqValue" style="font-weight:bold; color:#0066cc; font-size:18px; min-width:60px;">10 Hz</span>
        </div>
        <div style="font-size:11px; color:#666; margin-top:6px;">
          Range: 1-30 Hz | Current: <span id="currentFreq">10</span> Hz
        </div>
      </div>

      <!-- v3.10.8: 相机曝光/增益滑块（除草线调试；同步设 RGB+IR 两台相机，NDVI 不受影响）-->
      <div style="margin:15px 0; padding:12px; background:#fff4e6; border-radius:8px; border:1px solid #f1c79b;">
        <label style="font-weight:bold; color:#cc6600; display:block; margin-bottom:8px;">💡 相机曝光 / 增益（RGB+IR 同步）:</label>
        <div style="display:flex; align-items:center; gap:10px;">
          <span style="min-width:38px;">曝光</span>
          <input type="range" id="expSlider" min="1" max="500" value="8"
                 oninput="document.getElementById('expValue').textContent=this.value"
                 onchange="setCamExposure(this.value)" style="flex:1; height:6px;">
          <span id="expValue" style="font-weight:bold; color:#cc6600; font-size:18px; min-width:48px;">8</span>
        </div>
        <div style="display:flex; align-items:center; gap:10px; margin-top:8px;">
          <span style="min-width:38px;">增益</span>
          <input type="range" id="gainSlider" min="0" max="128" value="64"
                 oninput="document.getElementById('gainValue').textContent=this.value"
                 onchange="setCamGain(this.value)" style="flex:1; height:6px;">
          <span id="gainValue" style="font-weight:bold; color:#cc6600; font-size:18px; min-width:48px;">64</span>
        </div>
        <div style="font-size:11px; color:#666; margin-top:6px;">
          曝光小→暗(激光点更突出)，大→亮(YOLO/室内更清楚)。室内补光时在 8~220 间找"白纸浅灰、激光点仍亮"。两台相机同步，NDVI 用其锁定预设。
        </div>
      </div>

  </div>

  <div class="right-panel">
    <div class="stat-box">
      <div class="group-title">系统状态</div>
      <div class="stat-row"><span class="stat-key">触发模式</span><span class="stat-val" id="s-trigger">--</span></div>
      <div class="stat-row"><span class="stat-key">伺服模式</span><span class="stat-val" id="s-loop">--</span></div>
      <div class="stat-row"><span class="stat-key">FSM 状态</span><span class="stat-val" id="s-state">--</span></div>
      <div class="stat-row"><span class="stat-key">PID Settle</span><span class="stat-val" id="s-settle">--</span></div>
      <div style="margin-top:6px">
        <span class="laser-state" id="ls-ir">S4 RED: OFF</span>
        <span class="laser-state" id="ls-blue">S3 BLUE: OFF</span>
      </div>
    </div>

    <div class="stat-box">
      <div class="group-title">坐标 / 误差 (全 RGB 像素)</div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#3af">●</span> YOLO 目标</span>
        <span class="stat-val" id="s-yolo">--</span>
      </div>
      <div class="stat-row"><span class="stat-key">YOLO 新鲜度</span><span class="stat-val" id="s-yolo-fresh">--</span></div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#ff0">○</span> 红光斑实测</span>
        <span class="stat-val" id="s-spot">--</span>
      </div>
      <div class="stat-row">
        <span class="stat-key"><span style="color:#c4f">✚</span> 蓝紫预测落点</span>
        <span class="stat-val" id="s-predicted">--</span>
      </div>
      <div class="stat-row"><span class="stat-key">PID 误差 (Px, Py)</span><span class="stat-val" id="s-error">--</span></div>
      <div class="stat-row"><span class="stat-key">蓝紫→目标 偏差</span><span class="stat-val" id="s-hit-error">--</span></div>
      <div class="stat-row"><span class="stat-key">量化台阶 / 本轮最佳</span><span class="stat-val" id="s-quant">--</span></div>
      <div class="stat-row"><span class="stat-key">上步整数度移动</span><span class="stat-val" id="s-move">--</span></div>
      <div class="stat-row"><span class="stat-key">连续锁定帧数</span><span class="stat-val" id="s-lock">0/0</span></div>
    </div>

    <!-- v3.11.2: 蓝斑命中判定战报(纯观测) -->
    <div class="stat-box">
      <div class="group-title">🎯 打击战报 (蓝斑命中判定)</div>
      <div class="stat-row"><span class="stat-key">最近一发</span><span class="stat-val" id="s-hit-last">--</span></div>
      <div class="stat-row"><span class="stat-key">累计 命中/脱靶/未检出</span><span class="stat-val" id="s-hit-stats">0 / 0 / 0</span></div>
      <div class="info">判定纯观测,不影响打击流程;"未检出"=相机没看到蓝斑,弃权不算脱靶。</div>
    </div>

    <!-- v3.11.2: strike_planner 决策会话面板 -->
    <div class="stat-box">
      <div class="group-title">🗂 决策队列 (strike_planner)</div>
      <div class="stat-row"><span class="stat-key">会话状态</span><span class="stat-val" id="s-sess-state">--</span></div>
      <div id="s-sess-list" style="font-size:12px; line-height:1.7; margin-top:4px; color:#888">(无会话数据 —— strike_planner 未运行或未触发清场)</div>
    </div>

    <!-- v3.12.0: 作业统计(内存+JSONL 全管线) -->
    <div class="stat-box">
      <div class="group-title">📊 作业统计
        <button class="btn" style="float:right;padding:2px 10px;font-size:12px" onclick="statsReset()">重置</button>
      </div>
      <div class="stat-row"><span class="stat-key">打击 成功/总数</span><span class="stat-val" id="s-st-succ">0 / 0</span></div>
      <div class="stat-row"><span class="stat-key">判定命中率</span><span class="stat-val" id="s-st-hit">--</span></div>
      <div class="stat-row"><span class="stat-key">平均锁定误差</span><span class="stat-val" id="s-st-err">--</span></div>
      <div class="stat-row"><span class="stat-key">平均命中偏差</span><span class="stat-val" id="s-st-hd">--</span></div>
      <div class="stat-row"><span class="stat-key">平均单株耗时</span><span class="stat-val" id="s-st-dur">--</span></div>
      <canvas id="statChart" width="360" height="120" style="width:100%;background:#111;border-radius:4px;margin-top:6px"></canvas>
      <div class="info">逐发命中偏差折线:绿=命中 红=脱靶 灰=未判定 紫=失败;虚线=HIT_TOL。
        同步落盘 ~/strike_logs/*.jsonl,离线 <code>ros2 run laser_calibration analyze_strike_logs</code> 出报告。</div>
    </div>

    <!-- v3.13.0: 相对绿度健康 -->
    <div class="stat-box">
      <div class="group-title">🌱 植物健康(相对绿度指数)</div>
      <div class="stat-row"><span class="stat-key">当前指数 / 参与株数</span><span class="stat-val" id="s-hl-cur">--</span></div>
      <div class="stat-row"><span class="stat-key">相对基线变化</span><span class="stat-val" id="s-hl-delta">--</span></div>
      <canvas id="healthChart" width="360" height="100" style="width:100%;background:#111;border-radius:4px;margin-top:6px"></canvas>
      <div class="info">对 crop 类框逐秒采样 ExG(2G−R−B)均值,只做<b>相对</b>趋势解读(同机位同光照)。
        打印靶演示可用;真植物 NDVI 见 8080/8081 双光谱流。</div>
    </div>

    <!-- v3.13.0: 整片作业历史 -->
    <div class="stat-box">
      <div class="group-title">📜 整片作业历史</div>
      <div id="s-patch-list" style="font-size:12px;line-height:1.7;color:#888">(尚无 —— 每清完一片自动记一行)</div>
    </div>

    <div class="stat-box">
      <div class="group-title">舵机角度</div>
      <div class="stat-row"><span class="stat-key">S1 Yaw</span><span class="stat-val" id="s-yaw">--</span></div>
      <div class="stat-row"><span class="stat-key">S2 Pitch</span><span class="stat-val" id="s-pitch">--</span></div>
    </div>

    <div class="stat-box">
      <div class="group-title">PID 调参 (1° 舵机：Kp 为主，Ki/Kd 基本无效)</div>
      <div class="stat-row">
        <span class="stat-key">Kp</span>
        <input type="number" step="0.05" class="pid-input" id="pid-kp" value="0.8" onchange="updatePID()">
      </div>
      <div class="stat-row">
        <span class="stat-key">Ki</span>
        <input type="number" step="0.0001" class="pid-input" id="pid-ki" value="0" onchange="updatePID()">
      </div>
      <div class="stat-row">
        <span class="stat-key">Kd</span>
        <input type="number" step="0.001" class="pid-input" id="pid-kd" value="0" onchange="updatePID()">
      </div>
      <div class="info">Kp≈0.8~1.0 每步近似一次到位；&gt;1 易触发量化极限环。</div>
      <div class="stat-row" style="margin-top:6px; align-items:center">
        <span class="stat-key" id="pid-src" style="font-size:11px">参数来源: --</span>
        <button class="btn" style="padding:2px 10px; font-size:11px; margin:0"
                onclick="pidReset()">恢复默认</button>
      </div>
      <div class="info">改动即自动存盘 → 下次启动自动加载。</div>
    </div>

    <div class="stat-box">
      <div class="group-title">误差曲线 (实时)</div>
      <canvas id="chart" width="450" height="160" class="pid-chart"></canvas>
      <div class="info">
        <span style="color:#0f0">●</span> Px (X 误差)
        <span style="color:#fa0;margin-left:14px">●</span> Py (Y 误差)
      </div>
    </div>
  </div>
</div>

<script>
const canvas = document.getElementById('cv');
const ctx = canvas.getContext('2d');
const liveImg = new Image();
let lastState = {};

function liveLoop() {
  liveImg.onload = () => {
    ctx.drawImage(liveImg, 0, 0, 640, 480);
    drawOverlays();
    setTimeout(liveLoop, 80);
  };
  liveImg.onerror = () => setTimeout(liveLoop, 300);
  liveImg.src = `/frame?t=${Date.now()}`;
}
liveLoop();

function drawOverlays() {
  if (!lastState) return;
  if (lastState.yolo) {
    // v3.11.1: 靶点框按新鲜度着色 —— 新鲜=绿,陈旧(跟踪滞后/丢帧)=灰,
    //   一眼区分"靶点是否跟上当前检测"。
    var yFresh = (lastState.yolo_fresh === true);
    var yCol = yFresh ? '#0f0' : '#888';
    ctx.strokeStyle = yCol; ctx.lineWidth = 2;
    ctx.strokeRect(lastState.yolo.x - 18, lastState.yolo.y - 18, 36, 36);
    ctx.fillStyle = yCol; ctx.font = '12px monospace';
    ctx.fillText(`靶点${yFresh ? '' : '(陈旧)'}(${lastState.yolo.x},${lastState.yolo.y})`,
                 lastState.yolo.x + 22, lastState.yolo.y - 22);
  }
  if (lastState.spot) {
    ctx.strokeStyle = '#ff0'; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(lastState.spot.x, lastState.spot.y, 12, 0, Math.PI*2);
    ctx.stroke();
    ctx.fillStyle = '#ff0'; ctx.font = '12px monospace';
    ctx.fillText(`红光斑(${lastState.spot.x},${lastState.spot.y})`,
                 lastState.spot.x + 16, lastState.spot.y);
  }
  if (lastState.yolo_boxes && lastState.yolo_boxes.length > 0) {
    // v3.11.1: 按 yolo_boxes_age_sec 区分新鲜/陈旧/过老:
    //   <0.25s 新鲜→实线青;0.25~1.5s 陈旧→虚线灰+"陈旧Xms"(上次检测,非实时);
    //   >1.5s 过老→不画(场景多半已无目标)。这样既保留视觉参照,又不误导成实时框。
    var age = lastState.yolo_boxes_age_sec;
    var hasAge = (age !== undefined && age !== null);
    var tooOld = (hasAge && age > 1.5);
    var stale = (hasAge && age > 0.25);
    if (!tooOld) {
      lastState.yolo_boxes.forEach(function(box) {
        const cx = box.cx || 0;
        const cy = box.cy || 0;
        const w = box.w || 50;
        const h = box.h || 50;
        const x1 = cx - w/2;
        const y1 = cy - h/2;
        const label = box.label || 'weed';
        const conf = box.confidence ? box.confidence.toFixed(2) : '?';
        if (stale) {
          ctx.strokeStyle = '#888'; ctx.lineWidth = 2;
          ctx.setLineDash([6, 4]);
          ctx.strokeRect(x1, y1, w, h);
          ctx.setLineDash([]);
          ctx.fillStyle = '#888'; ctx.font = 'bold 12px monospace';
          ctx.fillText(`${label} 陈旧${Math.round(age*1000)}ms`, x1, Math.max(y1 - 4, 12));
        } else {
          ctx.strokeStyle = '#0ff'; ctx.lineWidth = 2;
          ctx.setLineDash([]);
          ctx.strokeRect(x1, y1, w, h);
          ctx.fillStyle = '#0ff'; ctx.font = 'bold 12px monospace';
          ctx.fillText(`${label} ${conf}`, x1, Math.max(y1 - 4, 12));
        }
      });
    }
  }
  if (lastState.predicted_hit) {
    drawCross(lastState.predicted_hit.x, lastState.predicted_hit.y, '#c4f',
              `蓝紫(${lastState.predicted_hit.x},${lastState.predicted_hit.y})`);
    if (lastState.spot) {
      ctx.strokeStyle = '#c4f'; ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(lastState.spot.x, lastState.spot.y);
      ctx.lineTo(lastState.predicted_hit.x, lastState.predicted_hit.y);
      ctx.stroke();
    }
  }

  // v3.11.2: 决策队列叠加 —— planner 会话目标(中心参考系)按 session_shift
  //   平移到当前画面。shift 为 null(坐标系未知)时不画,只留侧栏列表,绝不画错图。
  var ps = lastState.planner_session;
  var shift = lastState.session_shift;
  var psAge = lastState.planner_session_age_sec;
  if (ps && shift && psAge !== undefined && psAge !== null && psAge < 2.0 &&
      ps.state && ps.state !== 'IDLE') {
    var drawT = function(t, color, tag, dash) {
      if (!t) return;
      var x = t.x + shift.dx, y = t.y + shift.dy;
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      if (dash) ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.arc(x, y, 16, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = color; ctx.font = 'bold 13px monospace';
      ctx.fillText('#' + t.id + tag, x - 14, y - 22);
    };
    (ps.pending || []).forEach(function(t){ drawT(t, '#aaa', '', true); });
    (ps.failed  || []).forEach(function(t){ drawT(t, '#f55', ' ✗', false); });
    (ps.struck  || []).forEach(function(t){
      drawT(t, (t.hit === false ? '#fa0' : '#0f0'), ' ✓', false);
    });
    drawT(ps.current, '#ff0', ' ◀', false);
  }

  // v3.11.2: 最近一发命中判定的大标记(4s 内显示;下一发盲跳后画面已平移,隐藏)
  var lh = lastState.last_hit;
  var lhAge = lastState.last_hit_age_sec;
  if (lh && lhAge !== undefined && lhAge !== null && lhAge < 4.0 &&
      lastState.fsm_state !== 'COARSE') {
    var hp = lh.pos || lh.aim;
    if (hp) {
      var col = lh.verdict === 'hit' ? '#0f0'
              : lh.verdict === 'miss' ? '#f55' : '#888';
      var txt = lh.verdict === 'hit' ? ('✓ 命中 d=' + lh.dist + 'px')
              : lh.verdict === 'miss' ? ('✗ 脱靶 d=' + lh.dist + 'px')
              : '? 蓝斑未检出';
      ctx.strokeStyle = col; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(hp.x, hp.y, 24, 0, Math.PI * 2); ctx.stroke();
      ctx.fillStyle = col; ctx.font = 'bold 16px monospace';
      ctx.fillText(txt, Math.max(8, Math.min(hp.x + 30, 420)), hp.y + 5);
    }
  }
}

// v3.12.0: 作业统计折线(逐发命中偏差;无判定值退用锁定误差)
function drawStatsChart(shots, tol) {
  var cv = document.getElementById('statChart');
  if (!cv) return;
  var c2 = cv.getContext('2d');
  c2.clearRect(0, 0, cv.width, cv.height);
  if (!shots.length) {
    c2.fillStyle = '#555'; c2.font = '12px monospace';
    c2.fillText('暂无打击记录(打一发就有了)', 10, 22);
    return;
  }
  var vals = shots.map(function(s) {
    return (s.hit_dist != null) ? s.hit_dist
         : (s.final_distance != null ? s.final_distance : 0);
  });
  var W = cv.width, H = cv.height, pad = 18;
  var ymax = Math.max(tol || 30, Math.max.apply(null, vals)) * 1.2 + 1;
  var n = shots.length;
  var X = function(i) { return pad + (W - 2 * pad) * (n === 1 ? 0.5 : i / (n - 1)); };
  var Y = function(v) { return H - pad + 4 - (H - 2 * pad) * (v / ymax); };
  if (tol) {
    c2.strokeStyle = '#666'; c2.setLineDash([4, 4]);
    c2.beginPath(); c2.moveTo(pad, Y(tol)); c2.lineTo(W - pad, Y(tol)); c2.stroke();
    c2.setLineDash([]);
  }
  c2.strokeStyle = '#48f'; c2.lineWidth = 1; c2.beginPath();
  for (var i = 0; i < n; i++) {
    if (i === 0) c2.moveTo(X(i), Y(vals[i])); else c2.lineTo(X(i), Y(vals[i]));
  }
  c2.stroke();
  for (var j = 0; j < n; j++) {
    var sh = shots[j];
    var col = sh.result !== 'success' ? '#c4f'
            : (sh.hit === true ? '#0f0' : (sh.hit === false ? '#f55' : '#999'));
    c2.fillStyle = col;
    c2.beginPath(); c2.arc(X(j), Y(vals[j]), 3, 0, Math.PI * 2); c2.fill();
  }
  c2.fillStyle = '#888'; c2.font = '10px monospace';
  c2.fillText('0', 4, H - pad + 7);
  c2.fillText(String(Math.round(ymax)), 4, pad + 4);
  if (tol) c2.fillText('TOL', W - pad - 24, Y(tol) - 3);
}
// v3.13.0: 任务命令(急停零确认;发车确认一次防误触)
function missionCmd(cmd) {
  if (cmd === 'estop') { fetch('/api/estop').then(refreshState); return; }
  if (cmd === 'start' && !confirm('确认发车?小车将开始自动巡航-清除作业')) return;
  fetch('/api/mission_' + cmd).then(refreshState);
}
// v3.13.0: 健康折线(自动量程 + 基线虚线)
function drawHealthChart(series, baseline) {
  var cv = document.getElementById('healthChart');
  if (!cv) return;
  var c2 = cv.getContext('2d');
  c2.clearRect(0, 0, cv.width, cv.height);
  if (!series || series.length < 2) {
    c2.fillStyle = '#555'; c2.font = '12px monospace';
    c2.fillText('等待 crop 框进入画面…', 10, 20); return;
  }
  var vs = series.map(function(p) { return p[1]; });
  var lo = Math.min.apply(null, vs.concat([baseline || 1e9]));
  var hi = Math.max.apply(null, vs.concat([baseline || -1e9]));
  var pad = 14, W = cv.width, H = cv.height;
  if (hi - lo < 1) { hi += 1; lo -= 1; }
  var X = function(i) { return pad + (W - 2 * pad) * i / (series.length - 1); };
  var Y = function(v) { return H - pad - (H - 2 * pad) * (v - lo) / (hi - lo); };
  if (baseline != null) {
    c2.strokeStyle = '#666'; c2.setLineDash([4, 4]); c2.beginPath();
    c2.moveTo(pad, Y(baseline)); c2.lineTo(W - pad, Y(baseline));
    c2.stroke(); c2.setLineDash([]);
  }
  c2.strokeStyle = '#3d5'; c2.lineWidth = 1.5; c2.beginPath();
  for (var i = 0; i < series.length; i++) {
    if (i === 0) c2.moveTo(X(i), Y(vs[i])); else c2.lineTo(X(i), Y(vs[i]));
  }
  c2.stroke();
  c2.fillStyle = '#888'; c2.font = '10px monospace';
  c2.fillText(String(Math.round(hi)), 4, pad + 4);
  c2.fillText(String(Math.round(lo)), 4, H - pad + 4);
}

function statsReset() {
  if (confirm('清零本场作业统计?(磁盘 JSONL 日志不会删除)'))
    fetch('/api/stats_reset').then(refreshState);
}

function drawCross(x, y, color, label) {
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x-20, y); ctx.lineTo(x+20, y);
  ctx.moveTo(x, y-20); ctx.lineTo(x, y+20);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(x, y, 14, 0, Math.PI*2); ctx.stroke();
  if (label) {
    ctx.fillStyle = color; ctx.font = '12px monospace';
    ctx.fillText(label, x + 18, y - 12);
  }
}

const chart = document.getElementById('chart');
const cctx = chart.getContext('2d');
let history_px = [], history_py = [];
const HISTORY = 80;

function drawChart() {
  cctx.fillStyle = '#0a0a0a';
  cctx.fillRect(0, 0, chart.width, chart.height);
  cctx.strokeStyle = '#333';
  cctx.beginPath();
  cctx.moveTo(0, chart.height/2);
  cctx.lineTo(chart.width, chart.height/2);
  cctx.stroke();
  cctx.strokeStyle = '#222';
  cctx.setLineDash([2,4]);
  const tol_h = chart.height/2 - 6;
  cctx.beginPath();
  cctx.moveTo(0, tol_h); cctx.lineTo(chart.width, tol_h);
  cctx.moveTo(0, chart.height - tol_h);
  cctx.lineTo(chart.width, chart.height - tol_h);
  cctx.stroke();
  cctx.setLineDash([]);
  drawSeries(history_px, '#0f0');
  drawSeries(history_py, '#fa0');
}

function drawSeries(arr, color) {
  if (arr.length < 2) return;
  cctx.strokeStyle = color;
  cctx.lineWidth = 1.5;
  cctx.beginPath();
  const scale = chart.height / 2 / 50;
  for (let i = 0; i < arr.length; i++) {
    const x = (i / HISTORY) * chart.width;
    const y = chart.height/2 - arr[i] * scale;
    if (i === 0) cctx.moveTo(x, y); else cctx.lineTo(x, y);
  }
  cctx.stroke();
}

function refreshState() {
  fetch('/api/state').then(r => r.json()).then(d => {
    lastState = d;
    document.getElementById('s-trigger').textContent = d.trigger || '--';
    document.getElementById('s-loop').textContent = d.loop_mode || '--';
    document.getElementById('s-state').textContent = d.fsm_state || '--';
    document.getElementById('s-state').style.color =
      ({'IDLE':'#888','GOT_TARGET':'#fa0','COARSE':'#fa0','PID':'#fa0',
        'LOCKED':'#0f0','FIRING':'#f55','COOLDOWN':'#888','FAILED':'#f00'})[d.fsm_state] || '#0f0';

    // v3.10.0: settle gate indicator
    const settleEl = document.getElementById('s-settle');
    if (d.pid_settle_remaining_sec !== undefined && d.pid_settle_remaining_sec !== null) {
      if (d.pid_settle_remaining_sec > 0) {
        settleEl.textContent = `等待 ${d.pid_settle_remaining_sec.toFixed(2)}s`;
        settleEl.className = 'stat-val settle-on';
      } else {
        settleEl.textContent = '可下发';
        settleEl.className = 'stat-val settle-off';
      }
    } else {
      settleEl.textContent = '--';
      settleEl.className = 'stat-val';
    }

    document.getElementById('s-yolo').textContent = d.yolo ? `(${d.yolo.x},${d.yolo.y})` : '--';
    const freshEl = document.getElementById('s-yolo-fresh');
    if (d.yolo_fresh === true) {
      freshEl.textContent = `新鲜 (${d.yolo_age_sec.toFixed(2)}s 前)`;
      freshEl.className = 'stat-val yolo-fresh-on';
    } else if (d.yolo_age_sec !== undefined && d.yolo_age_sec !== null) {
      freshEl.textContent = `过期 (${d.yolo_age_sec.toFixed(2)}s 前)`;
      freshEl.className = 'stat-val yolo-fresh-off';
    } else {
      freshEl.textContent = '无数据';
      freshEl.className = 'stat-val';
    }
    document.getElementById('s-spot').textContent = d.spot ? `(${d.spot.x},${d.spot.y})` : '--';
    document.getElementById('s-predicted').textContent = d.predicted_hit ? `(${d.predicted_hit.x},${d.predicted_hit.y})` : '--';
    if (d.predicted_hit && d.yolo) {
      const hex = d.predicted_hit.x - d.yolo.x;
      const hey = d.predicted_hit.y - d.yolo.y;
      const dist = Math.sqrt(hex*hex + hey*hey);
      document.getElementById('s-hit-error').textContent =
        `(${hex>=0?'+':''}${hex},${hey>=0?'+':''}${hey})  d=${dist.toFixed(1)}px`;
    } else {
      document.getElementById('s-hit-error').textContent = '--';
    }
    document.getElementById('s-error').textContent =
      d.error ? `(${d.error.x>=0?'+':''}${d.error.x},${d.error.y>=0?'+':''}${d.error.y})` : '--';
    // v3.10.2: 量化台阶 / 本轮最佳距离
    const qp = (d.quant_px !== undefined && d.quant_px !== null) ? d.quant_px.toFixed(0) : '?';
    const bd = (d.pid_best_distance !== undefined && d.pid_best_distance !== null
                && isFinite(d.pid_best_distance)) ? d.pid_best_distance.toFixed(1) : '--';
    document.getElementById('s-quant').textContent = `${qp}px / ${bd}px`;
    // v3.10.2: 上一步整数度移动
    if (d.last_move) {
      document.getElementById('s-move').textContent =
        `(${d.last_move.yaw>=0?'+':''}${d.last_move.yaw}°, ${d.last_move.pitch>=0?'+':''}${d.last_move.pitch}°)`;
    } else {
      document.getElementById('s-move').textContent = '--';
    }
    document.getElementById('s-lock').textContent = `${d.lock_frames || 0}/${d.lock_target || 5}`;
    // v3.10.3: PID 参数来源指示
    const srcEl = document.getElementById('pid-src');
    if (srcEl) {
      srcEl.textContent = '参数来源: ' +
        (d.pid_tuning_source === 'saved' ? '已保存(自动加载)' : '默认值');
      srcEl.style.color = (d.pid_tuning_source === 'saved') ? '#0f0' : '#888';
    }
    document.getElementById('s-yaw').textContent = d.yaw !== undefined ? d.yaw.toFixed(1) + '°' : '--';
    document.getElementById('s-pitch').textContent = d.pitch !== undefined ? d.pitch.toFixed(1) + '°' : '--';

    document.getElementById('trig-manual').className = 'btn' + (d.trigger === 'manual' ? ' active' : '');
    document.getElementById('trig-auto').className = 'btn' + (d.trigger === 'auto' ? ' active' : '');
    document.getElementById('loop-open').className = 'btn' + (d.loop_mode === 'open_loop' ? ' active' : '');
    document.getElementById('loop-closed').className = 'btn' + (d.loop_mode === 'closed_loop' ? ' active' : '');

    // v3.11.1: ExG 开关按钮 —— 显示当前(已下发)状态
    var exgBtn = document.getElementById('btn-exg');
    if (exgBtn) {
      var exgOn = (d.exg_enable === true);
      exgBtn.textContent = exgOn ? '[ExG: 开 → 点击关]' : '[ExG: 关 → 点击开]';
      exgBtn.className = 'btn' + (exgOn ? ' active' : '');
    }

    setLaser('ls-ir',   'S4 RED', d.laser_ir);
    setLaser('ls-blue', 'S3 BLUE', d.laser_blue);

    if (d.fire_in_open_loop !== undefined) {
      const cb = document.getElementById('fire-open-cb');
      if (cb && cb.checked !== d.fire_in_open_loop) cb.checked = d.fire_in_open_loop;
    }

    const banner = document.getElementById('calib-banner');
    const bannerText = document.getElementById('calib-banner-text');
    if (d.calib2_done === false) {
      banner.style.display = 'block';
      banner.style.background = '#604000';
      banner.style.borderColor = '#fa0';
      bannerText.innerHTML = '⚠️ 标定二未完成 — 请先关闭本节点，运行 <code>ros2 run laser_calibration calib_laser</code> 完成标定。';
    } else if (d.calib2_stale === true) {
      banner.style.display = 'block';
      banner.style.background = '#600';
      banner.style.borderColor = '#f55';
      bannerText.innerHTML =
        `⛔ 标定二坐标系不匹配 — Δ=(${d.delta_x>=0?'+':''}${d.delta_x},${d.delta_y>=0?'+':''}${d.delta_y}) ` +
        `是 ${d.calib2_frame || 'IR/历史'} 坐标系，本节点是 RGB 坐标系。` +
        ' 用错值跑伺服会偏几十像素。请重做 <code>calib_laser</code>。';
    } else {
      banner.style.display = 'none';
    }

    // v3.11.2: 打击战报面板
    var lh2 = d.last_hit;
    var lastEl = document.getElementById('s-hit-last');
    if (lastEl) {
      if (lh2 && lh2.verdict === 'hit') {
        lastEl.textContent = '✓ 命中 d=' + lh2.dist + 'px (' + lh2.frames + '帧)';
        lastEl.style.color = '#0f0';
      } else if (lh2 && lh2.verdict === 'miss') {
        lastEl.textContent = '✗ 脱靶 d=' + lh2.dist + 'px (' + lh2.frames + '帧)';
        lastEl.style.color = '#f55';
      } else if (lh2) {
        lastEl.textContent = '? 蓝斑未检出 (' + (lh2.frames || 0) + '帧)';
        lastEl.style.color = '#888';
      } else {
        lastEl.textContent = '--';
        lastEl.style.color = '#888';
      }
    }
    var hsEl = document.getElementById('s-hit-stats');
    if (hsEl && d.hit_stats) {
      hsEl.textContent = d.hit_stats.hit + ' / ' + d.hit_stats.miss +
                         ' / ' + d.hit_stats.unseen;
    }

    // v3.11.2: 决策队列侧栏(始终可见,不依赖叠加层的坐标平移)
    var ps2 = d.planner_session;
    var ssEl = document.getElementById('s-sess-state');
    var slEl = document.getElementById('s-sess-list');
    if (ssEl && slEl) {
      if (ps2 && ps2.state) {
        var nStruck = (ps2.struck || []).length;
        ssEl.textContent = ps2.state +
          (ps2.total ? ('  ' + nStruck + '/' + ps2.total) : '');
        ssEl.style.color = ({'IDLE':'#888','VOTING':'#fa0',
          'CLEARING':'#0f0','WAIT_RESULT':'#ff0'})[ps2.state] || '#0f0';
        var html = '';
        if (ps2.state === 'VOTING' && ps2.voting) {
          html += '<div style="color:#fa0">⏳ 投票中: 已累积 ' +
                  ps2.voting.frames + ' 帧 / ' + ps2.voting.clusters +
                  ' 个候选簇</div>';
        }
        var line = function(icon, color, t, extra) {
          return '<div style="color:' + color + '">' + icon + ' #' + t.id +
                 ' (' + t.x + ',' + t.y + ')' + (extra || '') + '</div>';
        };
        (ps2.struck || []).forEach(function(t) {
          var ex = (t.hit === true)
            ? ' 已打 ✓命中' + (t.hit_dist != null ? ' d=' + t.hit_dist + 'px' : '')
            : (t.hit === false)
              ? ' 已打 ⚠判偏' + (t.hit_dist != null ? ' d=' + t.hit_dist + 'px' : '')
              : ' 已打 (蓝斑未检出)';
          html += line('●', (t.hit === false ? '#fa0' : '#0f0'), t, ex);
        });
        if (ps2.current) html += line('▶', '#ff0', ps2.current, ' 打击中…');
        (ps2.pending || []).forEach(function(t) {
          html += line('○', '#aaa', t, ' 待打');
        });
        (ps2.failed || []).forEach(function(t) {
          html += line('✗', '#f55', t, ' 放弃');
        });
        if (ps2.state === 'IDLE' && ps2.total) {
          html += '<div style="color:#0f0;margin-top:2px">■ 上一片: 成功 ' +
                  nStruck + ' / 共 ' + ps2.total +
                  (ps2.hits != null ? ('，判定命中 ' + ps2.hits) : '') + '</div>';
        }
        slEl.innerHTML = html || '(队列空)';
      } else {
        ssEl.textContent = '--';
        slEl.textContent = '(无会话数据 —— strike_planner 未运行或未触发清场)';
      }
    }

    // v3.13.0: 任务控制栏状态灯
    var ch = d.chassis, chAge = d.chassis_age_sec;
    var chEl = document.getElementById('m-chassis');
    if (chEl) {
      if (ch && chAge !== null && chAge < 2.0) {
        chEl.textContent = ch.state + (ch.vx ? (' ' + ch.vx + 'm/s') : '');
        chEl.style.color = ({'CRUISE':'#0f0','CLEARING':'#ff0','BRAKE':'#fa0',
          'BLIND_ROLL':'#4af','STANDBY':'#888','FAULT':'#f55'})[ch.state] || '#ccc';
      } else { chEl.textContent = '离线'; chEl.style.color = '#666'; }
    }
    var mp = document.getElementById('m-planner');
    if (mp) {
      var ps3 = d.planner_session;
      mp.textContent = (ps3 && ps3.state) ? ps3.state : '--';
      mp.style.color = (ps3 && ps3.state === 'CLEARING') ? '#0f0'
                     : (ps3 && ps3.state === 'VOTING') ? '#fa0' : '#888';
    }
    var ms = document.getElementById('m-servo');
    if (ms) { ms.textContent = d.fsm_state || '--';
      ms.style.color = d.fsm_state === 'IDLE' ? '#888' : '#0f0'; }
    var mb = document.getElementById('m-brief');
    if (mb && d.shot_stats) {
      mb.textContent = d.shot_stats.success + '/' + d.shot_stats.shots + ' 发' +
        (d.shot_stats.hit_rate != null
          ? (' · 命中 ' + Math.round(d.shot_stats.hit_rate * 100) + '%') : '');
    }

    // v3.13.0: 健康面板
    var hl = d.health;
    var hc = document.getElementById('s-hl-cur');
    var hd = document.getElementById('s-hl-delta');
    if (hc && hd) {
      if (hl) {
        hc.textContent = hl.cur + '  (' + hl.n_box + ' 株)';
        if (hl.delta_pct != null) {
          hd.textContent = (hl.delta_pct >= 0 ? '+' : '') + hl.delta_pct + '%';
          hd.style.color = hl.delta_pct >= -5 ? '#0f0' : '#fa0';
        } else hd.textContent = '--';
      } else { hc.textContent = '--(画面内无 crop 框)'; hd.textContent = '--'; }
    }
    drawHealthChart(hl ? hl.series : null, hl ? hl.baseline : null);

    // v3.13.0: 整片历史
    var pl = document.getElementById('s-patch-list');
    if (pl) {
      var hist = d.patch_history || [];
      if (hist.length) {
        pl.innerHTML = hist.slice().reverse().map(function(h) {
          var hit = (h.hit_confirmed != null)
            ? (' · 命中' + h.hit_confirmed + (h.hit_missed ? ('/偏' + h.hit_missed) : '')) : '';
          var col = (h.failed ? '#fa0' : '#0f0');
          return '<div style="color:' + col + '">[' + (h.at || '') + '] 清除 ' +
                 h.cleared + '/' + h.total + (h.failed ? (' · 放弃' + h.failed) : '') +
                 hit + '</div>';
        }).join('');
      }
    }

    // v3.12.0: 作业统计面板
    var st2 = d.shot_stats;
    var _set = function(id, v, color) {
      var el = document.getElementById(id);
      if (el) { el.textContent = v; if (color) el.style.color = color; }
    };
    if (st2) {
      _set('s-st-succ', st2.success + ' / ' + st2.shots +
        (st2.success_rate != null ? ' (' + Math.round(st2.success_rate * 100) + '%)' : ''),
        st2.failed > 0 ? '#fa0' : '#0f0');
      _set('s-st-hit', st2.hit_rate != null
        ? (Math.round(st2.hit_rate * 100) + '%  (' + st2.hit + '✓/' + st2.miss + '✗/' + st2.unjudged + '?)')
        : '--', st2.hit_rate != null && st2.hit_rate < 1 ? '#fa0' : '#0f0');
      _set('s-st-err', st2.avg_lock_err != null ? st2.avg_lock_err + ' px' : '--');
      _set('s-st-hd',  st2.avg_hit_dist != null ? st2.avg_hit_dist + ' px' : '--');
      _set('s-st-dur', st2.avg_duration != null ? st2.avg_duration + ' s' : '--');
    }
    drawStatsChart(d.shots || [], d.hit_tol_px);

    if (d.error) {
      history_px.push(d.error.x);
      history_py.push(d.error.y);
      if (history_px.length > HISTORY) { history_px.shift(); history_py.shift(); }
    }
    drawChart();
  }).catch(() => {});
}

function setLaser(id, name, st) {
  const el = document.getElementById(id);
  el.className = 'laser-state' + (st === 'fire' ? ' l-fire' : (st === 'on' ? ' l-on' : ''));
  el.textContent = `${name}: ${(st || 'off').toUpperCase()}`;
}

setInterval(refreshState, 100);

function setTrigger(m) { fetch('/api/trigger?m=' + m).then(refreshState); }
function setLoopMode(m) { fetch('/api/loop?m=' + m).then(refreshState); }
function action(act) { fetch('/api/' + act).then(refreshState); }
// v3.11.1: 切换 ExG 假草过滤(读当前状态取反下发)。
function toggleExg() {
  var cur = (lastState.exg_enable === true);
  fetch('/api/exg?on=' + (cur ? '0' : '1')).then(refreshState);
}
function manualIR(on) { fetch('/api/laser_ir?on=' + (on ? '1' : '0')).then(refreshState); }
function manualBlue(on) {
  const msg = on
    ? '⚠️ 即将开启 S3 蓝紫激光（持续，直到点 [S3 OFF]）！\n请确认:\n· 云台已对准白纸/防火垫\n· 激光路径无人/无易燃物\n· 已戴防护眼镜\n· 旁边备好灭火工具'
    : null;
  if (on && !confirm(msg)) return;
  fetch('/api/laser_blue?on=' + (on ? '1' : '0')).then(refreshState);
}
function testFire() {
  if (!confirm('⚠️ 即将开启 S3 蓝紫激光烧 0.5 秒（测试 S3 接口/接线）！\n请确认:\n· 云台已对准白纸/防火垫\n· 激光路径无人/无易燃物\n· 已戴防护眼镜')) {
    return;
  }
  fetch('/api/fire_test?dur=0.5').then(refreshState);
}
function toggleFireOpen() {
  const cb = document.getElementById('fire-open-cb');
  if (cb.checked) {
    if (!confirm('⚠️ 启用「开环模式自动开火」后，开环测试每次到达 LOCKED 状态都会自动烧 1 秒！\n仅在你能控制目标位置(放好白纸)、人员安全的情况下启用。\n确认启用？')) {
      cb.checked = false;
      return;
    }
  }
  fetch('/api/fire_open_toggle?on=' + (cb.checked ? '1' : '0')).then(refreshState);
}
function updatePID() {
  const kp = document.getElementById('pid-kp').value;
  const ki = document.getElementById('pid-ki').value;
  const kd = document.getElementById('pid-kd').value;
  fetch(`/api/pid?kp=${kp}&ki=${ki}&kd=${kd}`).then(refreshState);
}
function pidReset() {
  if (!confirm('恢复 PID 默认参数，并删除已保存的调参文件？')) return;
  fetch('/api/pid_reset').then(r => r.json()).then(d => {
    if (d.kp !== undefined) document.getElementById('pid-kp').value = d.kp;
    if (d.ki !== undefined) document.getElementById('pid-ki').value = d.ki;
    if (d.kd !== undefined) document.getElementById('pid-kd').value = d.kd;
    refreshState();
  });
}

fetch('/api/state').then(r => r.json()).then(d => {
  if (d.kp !== undefined) document.getElementById('pid-kp').value = d.kp;
  if (d.ki !== undefined) document.getElementById('pid-ki').value = d.ki;
  if (d.kd !== undefined) document.getElementById('pid-kd').value = d.kd;
  // v3.10.1: 滑块加载时反映后端当前指令频率（避免显示假的固定 10）
  if (d.yolo_cmd_freq !== undefined && d.yolo_cmd_freq !== null) {
    currentPublishFreq = d.yolo_cmd_freq;
    document.getElementById('freqSlider').value = d.yolo_cmd_freq;
    document.getElementById('freqValue').textContent = d.yolo_cmd_freq + ' Hz';
    document.getElementById('currentFreq').textContent = d.yolo_cmd_freq;
  }
});

// 加载麦克风列表（首次可能无标签，录音授权后会刷新）
loadMicList();

refreshState();

let currentPublishFreq = 10;

// v3.10.8: 相机曝光/增益滑块 —— 同步设 RGB+IR 两台相机（保持一致，NDVI 不受影响）
function setCamExposure(exp) {
  fetch('/api/set_cam?exposure=' + exp)
    .then(r => r.json())
    .then(d => { if (!d.success) console.warn('设曝光失败', d); });
}
function setCamGain(g) {
  fetch('/api/set_cam?gain=' + g)
    .then(r => r.json())
    .then(d => { if (!d.success) console.warn('设增益失败', d); });
}

function setPublishFreq(freq) {
  freq = parseFloat(freq);
  if (freq < 1 || freq > 30) {
    alert('Frequency must be between 1-30 Hz');
    return;
  }
  fetch('/api/set_yolo_freq?freq=' + freq)
    .then(response => response.json())
    .then(data => {
      if (data.success) {
        currentPublishFreq = freq;
        document.getElementById('currentFreq').textContent = freq;
        console.log('✅ Frequency set to:', freq, 'Hz');
      } else {
        alert('Failed: ' + data.message);
        document.getElementById('freqSlider').value = currentPublishFreq;
        document.getElementById('freqValue').textContent = currentPublishFreq + ' Hz';
      }
    })
    .catch(err => {
      console.error('❌ Request failed:', err);
      alert('Network error');
    });
}

// v3.14.0: ASR 语音控制 — 按住说话 → 浏览器采音 → POST 到小车 → 识别并执行
let mediaRecorder = null;
let audioChunks = [];
let isRecording = false;
let recordTimer = null;
let selectedDeviceId = '';
let ASR_BACKEND = 'rdk';  // 'rdk' | 'local'
const LOCAL_ASR_URL = 'http://127.0.0.1:8094';
let asrLogs = [];  // ASR 识别历史日志
const ASR_LOG_MAX = 50;

// 枚举麦克风设备并填充下拉框（先请求权限以获取设备标签）
function loadMicList() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) return;
  // 先请求权限，再枚举设备（确保获取到设备名称标签）
  var doEnum = function() {
    navigator.mediaDevices.enumerateDevices().then(devices => {
      var sel = document.getElementById('mic-select');
      if (!sel) return;
      var audioInputs = devices.filter(function(d) { return d.kind === 'audioinput'; });
      if (audioInputs.length === 0) {
        sel.innerHTML = '<option value="">(无麦克风)</option>';
        return;
      }
      var prevVal = sel.value;
      sel.innerHTML = audioInputs.map(function(d, i) {
        var label = d.label || ('麦克风 ' + (i + 1));
        return '<option value="' + d.deviceId + '">' + label + '</option>';
      }).join('');
      if (prevVal && Array.from(sel.options).some(function(o) { return o.value === prevVal; })) {
        sel.value = prevVal;
      }
      selectedDeviceId = sel.value;
    }).catch(function() {});
  };
  // 先请求权限，确保标签可用
  navigator.mediaDevices.getUserMedia({ audio: true }).then(function(stream) {
    stream.getTracks().forEach(function(t) { t.stop(); });
    doEnum();
  }).catch(function() {
    doEnum();  // 权限被拒也能枚举，只是无标签
  });
}

function onMicChange() {
  selectedDeviceId = document.getElementById('mic-select').value;
}

function onAsrBackendChange() {
  ASR_BACKEND = document.getElementById('asr-backend').value;
  var cfg = document.getElementById('local-asr-config');
  var sts = document.getElementById('asr-status');
  cfg.style.display = (ASR_BACKEND === 'local') ? 'inline' : 'none';
  if (ASR_BACKEND === 'local') {
    sts.textContent = '本地';
    // 检查本地服务状态
    fetch(LOCAL_ASR_URL + '/health', { signal: AbortSignal.timeout(2000) })
      .then(function(r) { return r.json(); })
      .then(function(d) {
        sts.textContent = d.model_loaded ? '本地 ✅' : '本地 ⚠ 待加载模型';
      })
      .catch(function() {
        sts.textContent = '本地 ❌ 未连接';
      });
  } else {
    sts.textContent = 'RDK端侧';
  }
}

function browseAndLoadModel() {
  var sts = document.getElementById('model-status');
  var display = document.getElementById('model-path-display');
  sts.textContent = '打开选择器...';
  sts.style.color = '#fa0';
  // 第一步：调用本地服务的系统文件夹选择器
  fetch(LOCAL_ASR_URL + '/browse', {
    method: 'POST',
    signal: AbortSignal.timeout(30000)
  }).then(function(r) { return r.json(); }).then(function(d) {
    if (!d.ok || !d.path) {
      sts.textContent = '❌ ' + (d.error || '取消选择');
      sts.style.color = '#f55';
      return;
    }
    var path = d.path;
    display.textContent = path;
    display.style.color = '#888';
    // 第二步：自动加载选中的模型
    sts.textContent = '加载中...';
    sts.style.color = '#fa0';
    fetch(LOCAL_ASR_URL + '/model', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path: path }),
      signal: AbortSignal.timeout(60000)
    }).then(function(r2) { return r2.json(); }).then(function(d2) {
      if (d2.ok && d2.loaded) {
        sts.textContent = '✅ ' + d2.size + ' 已加载';
        sts.style.color = '#0f0';
        document.getElementById('asr-status').textContent = '本地 ✅';
      } else {
        sts.textContent = '❌ ' + (d2.error || '加载失败');
        sts.style.color = '#f55';
      }
    }).catch(function(err) {
      sts.textContent = '❌ 加载失败: ' + err.message;
      sts.style.color = '#f55';
    });
  }).catch(function(err) {
    sts.textContent = '❌ 连接失败: ' + err.message;
    sts.style.color = '#f55';
  });
}

// 切换 ASR 日志面板显隐
function toggleAsrLog() {
  var panel = document.getElementById('asr-log-panel');
  if (!panel) return;
  var isHidden = (panel.style.display === 'none' || !panel.style.display);
  panel.style.display = isHidden ? 'block' : 'none';
  if (isHidden) renderAsrLog();
}

// 渲染 ASR 日志
function renderAsrLog() {
  var el = document.getElementById('asr-log-content');
  if (!el) return;
  if (asrLogs.length === 0) {
    el.innerHTML = '<span style="color:#666">(暂无日志 — 按住🎤开始识别)</span>';
    return;
  }
  el.innerHTML = asrLogs.slice().reverse().map(function(log) {
    var col = log.command ? '#0f0' : '#fa0';
    return '<div>' +
      '<span style="color:#666">[' + log.time + ']</span> ' +
      '<span style="color:#fff">"' + log.text + '"</span> ' +
      (log.command ? '<span style="color:' + col + '">→ ' + log.command + '</span>' : '<span style="color:#888">(未匹配)</span>') +
      ' <span style="color:#555">' + log.ms + 'ms</span>' +
      ' <span style="color:#555">[' + log.backend + ']</span>' +
      '</div>';
  }).join('');
}

function startRecord() {
  if (isRecording) return;
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    document.getElementById('asr-status').textContent = '❌ 不支持麦克风';
    return;
  }
  isRecording = true;
  audioChunks = [];
  document.getElementById('asr-status').textContent = '🔴 录音中...';
  document.getElementById('pttBtn').style.background = '#a33';
  document.getElementById('asr-result').textContent = '';

  var constraints = {
    sampleRate: 16000,
    channelCount: 1,
    echoCancellation: true,
    noiseSuppression: true
  };
  if (selectedDeviceId) {
    constraints.deviceId = { exact: selectedDeviceId };
  }
  navigator.mediaDevices.getUserMedia({ audio: constraints }).then(stream => {
    // 用低码率减少传输量
    mediaRecorder = new MediaRecorder(stream, {
      mimeType: MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
                ? 'audio/webm;codecs=opus' : 'audio/webm'
    });
    mediaRecorder.ondataavailable = e => {
      if (e.data.size > 0) audioChunks.push(e.data);
    };
    mediaRecorder.onstop = () => {
      stream.getTracks().forEach(t => t.stop());
      sendAudio();
    };
    mediaRecorder.start();
    // 超时自动停
    recordTimer = setTimeout(() => { if (isRecording) stopRecord(); }, 5000);
  }).catch(err => {
    isRecording = false;
    document.getElementById('asr-status').textContent = '❌ 麦克风被拒';
    document.getElementById('pttBtn').style.background = '#335';
  });
}

function stopRecord() {
  if (!isRecording || !mediaRecorder) return;
  isRecording = false;
  document.getElementById('pttBtn').style.background = '#335';
  if (recordTimer) { clearTimeout(recordTimer); recordTimer = null; }
  if (mediaRecorder.state === 'recording') mediaRecorder.stop();
}

// 将 Float32 Array 编码为 16kHz 16bit mono WAV Blob
function encodeWAV(samples, sampleRate) {
  var buffer = new ArrayBuffer(44 + samples.length * 2);
  var view = new DataView(buffer);
  function writeStr(offset, str) {
    for (var i = 0; i < str.length; i++) view.setUint8(offset + i, str.charCodeAt(i));
  }
  writeStr(0, 'RIFF');
  view.setUint32(4, 36 + samples.length * 2, true);
  writeStr(8, 'WAVE');
  writeStr(12, 'fmt ');
  view.setUint32(16, 16, true);       // chunk size
  view.setUint16(20, 1, true);        // PCM
  view.setUint16(22, 1, true);        // mono
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true);  // byte rate
  view.setUint16(32, 2, true);        // block align
  view.setUint16(34, 16, true);       // bits per sample
  writeStr(36, 'data');
  view.setUint32(40, samples.length * 2, true);
  for (var i = 0; i < samples.length; i++) {
    var s = Math.max(-1, Math.min(1, samples[i]));
    view.setInt16(44 + i * 2, s < 0 ? s * 0x8000 : s * 0x7FFF, true);
  }
  return new Blob([buffer], { type: 'audio/wav' });
}

// 将 webm blob 转为 16kHz mono WAV blob
function webmToWav(blob) {
  return new Promise(function(resolve, reject) {
    var reader = new FileReader();
    reader.onload = function() {
      var audioCtx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 16000 });
      audioCtx.decodeAudioData(reader.result, function(audioBuffer) {
        // 混音为单声道 + 降采样到 16kHz
        var orig = audioBuffer.getChannelData(0);
        var origRate = audioBuffer.sampleRate;
        // 直接重采样到 16kHz
        var targetLen = Math.round(orig.length * 16000 / origRate);
        var resampled = new Float32Array(targetLen);
        for (var i = 0; i < targetLen; i++) {
          var pos = i * origRate / 16000;
          var idx = Math.floor(pos);
          var frac = pos - idx;
          if (idx + 1 < orig.length) {
            resampled[i] = orig[idx] * (1 - frac) + orig[idx + 1] * frac;
          } else {
            resampled[i] = orig[idx] || 0;
          }
        }
        var wavBlob = encodeWAV(resampled, 16000);
        resolve(wavBlob);
      }, function() { reject('decodeAudioData 失败'); });
    };
    reader.onerror = function() { reject('FileReader 错误'); };
    reader.readAsArrayBuffer(blob);
  });
}

function sendAudio() {
  if (audioChunks.length === 0) return;
  document.getElementById('asr-status').textContent = '⏳ 转换中...';
  const webmBlob = new Blob(audioChunks, { type: 'audio/webm' });
  webmToWav(webmBlob).then(function(wavBlob) {
    var url = (ASR_BACKEND === 'rdk') ? '/api/asr' : LOCAL_ASR_URL + '/transcribe';
    document.getElementById('asr-status').textContent = '⏳ 识别中 (' + (ASR_BACKEND === 'rdk' ? 'RDK' : '本地') + ')...';
    fetch(url, { method: 'POST', body: wavBlob })
      .then(function(r) { return r.json(); })
      .then(function(d) {
        // 记录 ASR 日志
        var logEntry = {
          time: new Date().toLocaleTimeString(),
          text: d.text || '',
          command: d.command || null,
          ms: d.ms || 0,
          backend: ASR_BACKEND === 'rdk' ? 'RDK' : '本地'
        };
        asrLogs.push(logEntry);
        if (asrLogs.length > ASR_LOG_MAX) asrLogs.shift();

        if (d.ok && d.text) {
          document.getElementById('asr-result').textContent = '「' + d.text + '」';
          if (d.command) {
            document.getElementById('asr-status').textContent = '✅ → ' + d.command;
            // 自动执行命令
            if (d.command === 'ESTOP') {
              fetch('/api/estop').then(refreshState);
            } else if (d.command === 'START') {
              missionCmd('start');
            } else if (d.command === 'STOP') {
              missionCmd('stop');
            } else if (d.command === 'CENTER') {
              fetch('/api/center').then(refreshState);
            } else if (d.command === 'LASER_ON') {
              fetch('/api/laser_ir?on=1').then(refreshState);
            } else if (d.command === 'LASER_OFF') {
              fetch('/api/laser_ir?on=0').then(refreshState);
            } else if (d.command === 'FIRE_TEST') {
              fetch('/api/fire_test').then(refreshState);
            } else {
              document.getElementById('asr-status').textContent = '✅ 已识别: ' + d.command;
            }
          } else {
            document.getElementById('asr-status').textContent = (ASR_BACKEND === 'local') ? '本地 ✅' : '⚠️ 未识别指令';
          }
        } else if (d.msg && d.msg.indexOf('dev_mode') >= 0) {
          document.getElementById('asr-status').textContent = '⚠️ 仅开发模式';
          document.getElementById('asr-result').textContent = d.msg;
        } else {
          document.getElementById('asr-status').textContent = '❌ ' + (d.error || d.msg || '识别失败');
        }
      })
      .catch(function(err) {
        document.getElementById('asr-status').textContent = '❌ 网络错误';
        console.error('ASR fetch error:', err);
      });
  }).catch(function(err) {
    document.getElementById('asr-status').textContent = '❌ 转码失败';
    document.getElementById('asr-result').textContent = String(err);
  });
}


</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════
#  ROS2 节点
# ══════════════════════════════════════════════════════════════
class VisionServoNode(Node):

    def __init__(self):
        super().__init__("vision_servo")
        self.bridge = CvBridge()
        self.calib = load_calib()

        # 系统状态
        self.trigger_mode = SERVO_DEFAULT_MODE
        self.loop_mode    = "closed_loop"
        self.fsm_state    = STATE_IDLE
        self.fire_in_open_loop = False

        # 实时数据
        self._rgb_frame = None
        self._lock = threading.Lock()

        # YOLO 目标
        self.yolo_target = None
        self.yolo_target_at = 0.0
        self._yolo_boxes = []
        self._yolo_boxes_at = 0.0   # v3.11.1: 最近一次"有检测"的时刻,前端据此画陈旧/新鲜
        self._locked_yolo_target = None

        self.required_spot = None
        self.current_spot = None
        self.error = None
        self.lock_count = 0

        # 帧间稳定性追踪
        self._last_valid_spot = None
        self._spot_jump_count = 0

        # LOCKED 防日志泛滥
        self._locked_log_done = False
        self.servo_yaw   = SERVO_YAW_CENTER
        self.servo_pitch = SERVO_PITCH_CENTER
        self.laser_ir_state   = "off"
        self.laser_blue_state = "off"

        self.dry_run = False

        self._pid_started_at = 0.0
        self._locked_at = 0.0

        # ── v3.10.2 PID：像素域控制器，输出会被 round 成整数度 ──
        # Kp 用 PID_KP_DEFAULT 覆盖 config（量化感知需 Kp≈1，config 小 Kp 几乎不动舵机）
        self.kp = PID_KP_DEFAULT
        self.ki = PID_KI_DEFAULT
        self.kd = PID_KD_DEFAULT
        # v3.10.3: 若磁盘有上次保存的调参，覆盖默认值
        self._pid_tuning_source = "default"
        self._load_pid_tuning()
        self.pid_x = PIDController(self.kp, self.ki, self.kd, PID_OUTPUT_LIMIT_PX,
                                    deadband=PID_DEADBAND_PX)
        self.pid_y = PIDController(self.kp, self.ki, self.kd, PID_OUTPUT_LIMIT_PX,
                                    deadband=PID_DEADBAND_PX)
        self._pid_actively_moving = False

        # ── v3.10.2 量化感知 PID 运行态 ─────────────────────
        self._pid_best_distance = float("inf")  # 本轮见过的最小误差距离
        self._pid_best_yaw   = None             # 取得最佳距离时的舵机角
        self._pid_best_pitch = None
        self._pid_no_improve = 0                # 连续未刷新最佳距离的命令数
        self._last_move = {"yaw": 0, "pitch": 0}  # 上一步整数度移动量（诊断用）

        # ── settle gate 状态 ───────────────────────────────
        # 上一次 PID 下发舵机命令的时间戳；与 PID_SETTLE_TIME_SEC 比较决定是否可下发
        self._last_cmd_at = 0.0

        # ── 粗对准非阻塞状态机 ─────────────────────────────
        # None | "laser_warmup" | "compute" | "settle"
        self._coarse_phase = None
        self._coarse_phase_started_at = 0.0

        # 自动模式防抖
        self._auto_seen_at = 0.0
        self._auto_last_tx = None

        # 订阅
        self.sub_rgb  = self.create_subscription(Image,  TOPIC_RGB,  self._cb_rgb,  10)
        self.sub_yolo = self.create_subscription(String, TOPIC_YOLO, self._cb_yolo, 10)

        # v3.10.4: strike_planner 决策层接口
        # 当前是否处于"planner 指定的打击"中：None=手动/auto；非 None=planner 的目标 id
        self._strike_cmd_id = None
        # v3.10.11: planner 随指令下发的"已打目标"坐标(中心参考系),选框时排除其邻域
        self._strike_exclude_ref = []
        # v3.11.1: planner 随指令下发的"其它待打目标"坐标(中心参考系),身份核验时用于
        #   把候选框分类到正确目标 —— 防止抓到旁株未打目标(③那种)。
        self._other_targets_ref = []
        self._sel_gate_log_at = 0.0      # v3.10.11: 选框门限警告日志节流
        self.sub_strike_cmd = self.create_subscription(
            String, TOPIC_STRIKE_CMD, self._cb_strike_cmd, 10)
        self.pub_strike_result = self.create_publisher(
            String, TOPIC_STRIKE_RESULT, 10)
        # v3.11.1: ExG 假草过滤运行时开关 —— 网页按钮 → 本节点发布 → yolo_detector 订阅。
        #   _exg_ui_state 仅供网页显示当前(已下发的)状态,初值取 config 默认。
        self._exg_ui_state = bool(EXG_FILTER_ENABLE)
        self.pub_exg_enable = self.create_publisher(Bool, TOPIC_EXG_ENABLE, 10)
        # ── v3.11.2: 蓝斑命中判定(纯观测)运行态 ──────────────────
        #   _last_hit 只在点火线程整体赋值(dict 引用原子交换),HTTP 线程只读 → 无竞态
        self._last_hit = None          # {"id","verdict","dist","x","y","aim","frames","at"}
        self._hit_stats = {"hit": 0, "miss": 0, "unseen": 0}   # 本次运行累计
        # ── v3.11.2: planner 会话状态透传(纯显示) ────────────────
        self._planner_session = None   # strike_planner 广播的会话快照 dict
        self._planner_session_at = 0.0
        # ── v3.13.0: 任务面板运行态(纯显示/薄控制面) ──────────────
        self._chassis_state = None      # /chassis/state 最新 JSON
        self._chassis_state_at = 0.0
        self._patch_history = []        # /planner/patch_clear 逐片记录
        self._crop_boxes = []           # 最近一帧 crop 类框(健康指数用)
        self._health_series = []        # [(t, val, n_box)]
        self.pub_ch_start = self.create_publisher(Empty, TOPIC_CHASSIS_START, 10)
        self.pub_ch_stop  = self.create_publisher(Empty, TOPIC_CHASSIS_STOP, 10)
        self.pub_safety   = self.create_publisher(Empty, TOPIC_SAFETY_STOP, 10)
        self.sub_ch_state = self.create_subscription(
            String, TOPIC_CHASSIS_STATE, self._cb_chassis_state, 10)
        self.sub_patch    = self.create_subscription(
            String, TOPIC_PATCH_CLEAR, self._cb_patch_clear, 10)
        self.create_timer(HEALTH_SAMPLE_SEC, self._health_tick)
        # ── v3.12.0: 作业统计运行态(纯观测) ──────────────────────
        self._shots = []               # 每发打击记录(内存,SHOT_MEM_MAX 截断)
        self._shots_n = 0              # 全局发数计数(重置统计时清零)
        self._shot_lock = threading.Lock()   # fire线程/FSM定时器线程都会记账
        self._shot_fp = None           # JSONL 句柄:None=未开 False=开失败(禁用)
        self._servo_t0 = 0.0           # 本发伺服起始时刻(算单株耗时)
        self.sub_session = self.create_subscription(
            String, TOPIC_PLANNER_SESSION, self._cb_planner_session, 10)
        # v3.10.10 (P1): planner 建队前请求归中 —— 投票必须在参考位（居中）进行
        self.sub_recenter = self.create_subscription(
            Empty, TOPIC_SERVO_RECENTER, self._cb_recenter, 10)
        # v3.10.12: 全局急停 —— 视觉互锁/物理按钮桥接/任何安全源都可发布
        self.sub_safety = self.create_subscription(
            Empty, TOPIC_SAFETY_STOP, self._cb_safety_stop, 10)

        # v3.10.6: 点火序列的中止信号（紧急停止/归中时 set，点火线程立刻醒来终止）
        self._fire_cancel = threading.Event()

        # v3.10.7: HTTP 控制指令漏斗。HTTP 线程只把指令压进来（deque 的 append/
        #   popleft 是线程安全的），由 FSM timer 在每拍开头 drain 执行——所有 FSM
        #   状态变更收敛到 timer 单线程，消除 HTTP 线程与 timer 的多字段写竞态。
        self._cmd_queue = deque()

        # v3.10.1: YOLO 发布频率指令发布器
        # 注意：vision_servo 只能"建议"频率，真正改 timer 的是 YOLO 检测节点。
        # 用 TRANSIENT_LOCAL，晚启动的 YOLO 节点也能收到最后一次频率指令。
        self.yolo_cmd_freq = YOLO_FREQ_DEFAULT
        _freq_qos = QoSProfile(depth=1,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_yolo_freq = self.create_publisher(
            Float32, TOPIC_YOLO_FREQ_CMD, _freq_qos)

        all_lasers_off()
        center_servo()
        # NOTE: 这个 sleep 在 __init__ 里跑，节点还没 spin，不阻塞回调，保留
        time.sleep(0.3)

        # v3.14.0: 端侧 ASR 引擎
        self.asr = (AsrEngine(ASR_MODEL_DIR, num_threads=ASR_NUM_THREADS,
                              logger=lambda msg: self.get_logger().info(msg))
                    if ASR_ENABLE else None)
        self._asr_last_result = None   # 最后一条识别结果（供 API/state 查询）

        self._start_http()
        self.timer = self.create_timer(FSM_TICK_PERIOD_SEC, self._fsm_step)

        log = self.get_logger().info
        log("═══════════════════════════════════════════════════════")
        log("  视觉伺服节点  v3.10.7 (量化感知整数步进 · PID 调参持久化)")
        log("═══════════════════════════════════════════════════════")
        log(f"  SDK 状态:   {'✅ 已连接' if ROBOT_OK else '❌ 未连接（仅模拟）'}")
        log(f"  画面源:     RGB ({TOPIC_RGB})")
        log(f"  本机访问:   http://localhost:{SERVO_HTTP_PORT}")
        log(f"  远程访问:   http://<小车IP>:{SERVO_HTTP_PORT}")
        log("  ─────────────────────────────────────")
        log(f"  舵机量化:   1° = {SERVO_QUANT_PX:.1f}px  →  精度地板 ≈ {SERVO_HALF_QUANT_PX:.1f}px")
        log(f"  PID 参数:   Kp={self.kp}  Ki={self.ki}  Kd={self.kd}  "
            f"({'上次保存值' if self._pid_tuning_source == 'saved' else '默认值'})")
        log(f"  调参存盘:   {PID_TUNING_FILE}")
        log(f"  ASR 语音:    {'✅ 已就绪 (' + str(self.asr._recognizer.__class__.__name__) + ')' if self.asr and self.asr.available else '❌ 未加载(仅模拟/dev_mode)' if self.asr else '已禁用'}")
        log(f"  单步上限:   {MAX_DEG_PER_STEP:.0f}°    Settle: {PID_SETTLE_TIME_SEC*1000:.0f}ms")
        log(f"  目标冻结:   {'开' if SERVO_FREEZE_TARGET else '关'}")
        log(f"  执行器:     MultiThreadedExecutor (4 threads)")
        log("  ─────────────────────────────────────")
        log(f"  PID_TOLERANCE_PX(config)={PID_TOLERANCE_PX}px —— 仅作期望质量指标；")
        log(f"    锁定判据已改为'整数度移动量=0'，不再依赖该阈值。")
        if PID_TOLERANCE_PX < SERVO_HALF_QUANT_PX:
            log(f"    注意：{PID_TOLERANCE_PX}px 低于 {SERVO_HALF_QUANT_PX:.0f}px 精度地板，")
            log(f"    锁定会落在地板附近，日志将标注'硬件量化极限'。")
        log("  ─────────────────────────────────────")
        log("  标定状态（v3.9 仅依赖标定二）:")
        log(f"    Delta_X={self.calib.delta_x:+d}  Delta_Y={self.calib.delta_y:+d}  "
            f"{'✅' if self.calib.calib2_done else '❌ 未完成'}  "
            f"frame={self.calib.calib2_frame or '<未标记>'}")
        log(f"    SPOT_HOME=({SPOT_HOME_X},{SPOT_HOME_Y})  容差={SPOT_HOME_TOLERANCE}px")
        log(f"  PIXEL_TO_YAW_DEG={PIXEL_TO_YAW_DEG}  PIXEL_TO_PITCH_DEG={PIXEL_TO_PITCH_DEG}")
        log("  按 Ctrl+C 退出（自动关闭所有激光）")
        log("═══════════════════════════════════════════════════════")

        # 标定二坐标系检查
        self.calib2_stale = False
        if self.calib.calib2_done:
            if self.calib.calib2_frame != "rgb":
                self.calib2_stale = True
                warn = self.get_logger().warn
                warn("═══════════════════════════════════════════════════════")
                warn("  ⛔ 严重警告：标定二数据坐标系不匹配！")
                warn("═══════════════════════════════════════════════════════")
                warn(f"  当前 vision_servo: RGB 摄像头画面下工作")
                warn(f"  标定二数据坐标系:  {self.calib.calib2_frame or 'IR (推断 — v3.8 历史数据)'}")
                warn(f"  当前 Delta=({self.calib.delta_x:+d},{self.calib.delta_y:+d})")
                warn(f"  → 这是 IR 像素坐标下的偏移，不能用于 RGB 视觉伺服")
                warn(f"  → 用错误 Delta 跑伺服，云台会朝错误方向偏移很多像素")
                warn("  ─────────────────────────────────────")
                warn("  ✅ 解决办法：")
                warn("     1. 关闭本节点（Ctrl+C）")
                warn("     2. ros2 run laser_calibration calib_laser  # 用白纸重做")
                warn("     3. 重新启动本节点")
                warn("═══════════════════════════════════════════════════════")
        else:
            self.get_logger().warn(
                "⚠️ 标定二未完成。"
                "请运行 ros2 run laser_calibration calib_laser  做完再来跑伺服。"
            )

        threading.Thread(target=self._self_check_after_start, daemon=True).start()

    # ── 启动自检 ─────────────────────────────────────────────
    def _self_check_after_start(self):
        time.sleep(3.0)
        log = self.get_logger().info
        warn = self.get_logger().warn

        log("───── 启动自检：检测红光斑可见性 ─────")
        was_on = (self.laser_ir_state == "on")
        if not was_on:
            self._set_ir_laser(True)
            time.sleep(0.5)

        rgb = self._get_rgb()
        if rgb is None:
            warn("⚠️ RGB 画面未到达，跳过自检。请确认 stereo_camera 正在运行。")
            if not was_on:
                self._set_ir_laser(False)
            return

        spot = find_red_spot(rgb)
        if spot is None:
            warn("⚠️ 自检：归中状态下检测不到红光斑！")
            warn("   可能原因：")
            warn("   1. S4 红激光实际未点亮（电源/接线问题）")
            warn("   2. RGB 摄像头视野里没有光斑（指向太偏）")
            warn("   3. R-max(G,B) 阈值偏高 → 调小 RED_DOMINANCE_MIN（默认 30）")
            warn("   4. 摄像头白平衡漂了导致红光斑变粉/紫")
        else:
            sx, sy = spot
            dist = ((sx - SPOT_HOME_X)**2 + (sy - SPOT_HOME_Y)**2) ** 0.5
            log(f"✅ 自检：红光斑实测位置 ({sx},{sy})")
            log(f"   配置 SPOT_HOME=({SPOT_HOME_X},{SPOT_HOME_Y})  距离={dist:.0f}px")
            if dist > SPOT_HOME_TOLERANCE:
                warn(f"⚠️ 实测位置与 SPOT_HOME 偏离 {dist:.0f}px > {SPOT_HOME_TOLERANCE}px")
                warn(f"   建议把 config.py 里 SPOT_HOME_X/Y 改成 ({sx},{sy}) 减少 fallback 误差")

        if not was_on:
            self._set_ir_laser(False)
        log("───── 自检结束 ─────")

    # ── 回调 ─────────────────────────────────────────────────
    def _cb_rgb(self, msg):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._rgb_frame = f
        except Exception as e:
            self.get_logger().error(f"RGB 解码失败：{e}")

    def _set_yolo_boxes(self, boxes):
        """v3.11.1: 更新网页显示框 + 记录"本帧有新鲜检测"的时刻。
        前端用 (now − _yolo_boxes_at) 的年龄判定:新鲜→实线青;陈旧→虚线灰+"陈旧Xms";
        过老→不画。仅在"本帧确有检测"时调用(COARSE 早退路径不调,框保持陈旧)。"""
        self._yolo_boxes = boxes
        self._yolo_boxes_at = time.time()

    def _cb_yolo(self, msg):
        try:
            data = json.loads(msg.data)
            detected = data.get("detected", False)
            if not detected:
                # v3.11.1: 本帧无检测 → 不再清空 _yolo_boxes,保留上一帧框作"上次检测"
                #   陈旧显示(前端按 _yolo_boxes_at 年龄画虚线灰+标"陈旧Xms",过老则不画)。
                #   既不像 v3.10.14 之前残留一个"看着像实时"的假框误导演示,也保留视觉
                #   参照避免肉眼误判打偏。(v3.10.13/14 曾在此粗暴清框,反掀开了 yolo_target
                #   的固有跟踪滞后,显得"靶点跑出框"——本质是滞后被暴露,非打击出错。)
                #   伺服中(PID/LOCKED)不清 yolo_target:那由重捕获/PID 超时统一管。
                if self.fsm_state not in (STATE_PID, STATE_LOCKED, STATE_COARSE):
                    self.yolo_target = None
                return
            boxes = data.get("boxes")

            # v3.10.9: COARSE 盲大跳期间画面剧烈运动，拒收 YOLO，防止污染跟踪/账本坐标
            if self.fsm_state == STATE_COARSE:
                return

            if boxes:
                # v3.13.0: 顺手留存 crop 类框(健康指数采样用,纯观测)
                self._crop_boxes = [
                    b for b in boxes
                    if str(b.get("label", "")).lower() == "crop"]
                # v3.10.11: ① 框过滤 —— 只考虑 weed 且置信度达标的框。
                #   (旧版不滤,锚点最近邻可能选中 crop 框或低置信闪烁框)
                cand = [b for b in boxes
                        if b.get("label") in TRACK_LABELS
                        and float(b.get("confidence",
                                        b.get("conf", 0.0))) >= TRACK_MIN_CONF]

                servoing = self.fsm_state in (STATE_PID, STATE_LOCKED)

                # ② 锚点(v3.10.11 更正):
                #   跟踪中 → 上一帧靶点(它就是该草当前位置的最优预测);
                #   重捕获 → 红斑 + Δ —— 草的预测位置在【蓝光落点】,不在红斑!
                #     v3.10.9 锚在红斑,系统性偏 Δ(本机标定 ≈56px),两株草靠近时
                #     "最近邻"经常翻到旁边那株(尤其刚打完的) → 打重 + 漏打。
                #   gate:候选离锚点超出门限不收、等下一帧 —— 目标本帧闪烁漏检时
                #     宁可不更新,也不抓错株。
                if servoing and self.yolo_target is not None:
                    anchor_x = float(self.yolo_target["x"])
                    anchor_y = float(self.yolo_target["y"])
                    gate = NEIGHBOR_MAX_JUMP_PX
                elif self.current_spot is not None:
                    pred_x, pred_y = self.calib.spot_to_hit(
                        self.current_spot["x"], self.current_spot["y"])
                    anchor_x, anchor_y = float(pred_x), float(pred_y)
                    gate = REACQ_MAX_DIST_PX if servoing else None
                else:
                    # v3.10.12: 伺服中既无上一帧靶点也无红斑(光斑该帧未检出)
                    #   → 本帧不捕获,等光斑出现。旧版退到画面中心(320,240)且
                    #   无门限 —— 0612 实测 41 发里出现 1 次越门限捕获
                    #   (id=37, d=83px>50px 门限),即此路径,近草场景有抓错风险。
                    if servoing:
                        self._set_yolo_boxes(boxes)
                        return
                    anchor_x, anchor_y = 320.0, 240.0
                    gate = None

                # ③ v3.11.1 身份核验式重捕获(CMC + 数据关联,业界 BoT-SORT 同思路)。
                #   把"已打 + 其它待打"目标(planner 下发的中心参考系坐标)按 anchor 平移到
                #   当前画面得到各自预测位(平移量 = 锚点 − 本目标中心参考坐标;eye-in-hand:
                #   云台一转全场景同量平移,红斑/Δ 吸收盲跳残差)。
                #   · anchor 附近唯一候选 → 收(除非它明显落在某个其它目标上 → 拒);
                #   · 多个候选都逼近 anchor(歧义:残差大时已打/旁株会"挤"过来)→ 用【共识平移】
                #     消歧:对候选 b 设 e2=b−anchor;看该 e2 下各其它目标预测位是否各有"另一个"
                #     框证实(≤CONSENSUS_TOL_PX)。错框(如已打#1)的 e2 对不齐其它框→证实少;正确
                #     框的 e2 同时对齐全场→证实多。挑证实最多者;全无证实→安全拒打,不重打/误打。
                #   · anchor 附近无候选 → 不在此处理,交下方门限逻辑([选框]告警)。
                #   (注:简单"逐框最近预测"分类在残差大且目标相邻时会把已打#1误判成当前目标
                #    → 重打;故改用共识平移。离线用例 S1 已复现该误判并验证本法修正。)
                _ga_accept = False   # v3.12.0: 全局关联接受标志(可越门限,见下)
                if (REACQ_GLOBAL_ASSOC and servoing
                        and self._locked_yolo_target is not None and (
                        self._strike_exclude_ref or self._other_targets_ref)):
                    # ── v3.12.0 全场 RANSAC 全局关联(默认关) ──
                    #   替代下方共识版"选哪个框"这一步;其余(锚点/门限/日志节流)不变。
                    _cxy = [(float(b.get("cx", 0)), float(b.get("cy", 0)), b)
                            for b in cand]
                    _idx, _ai = associate_global(
                        (anchor_x, anchor_y),
                        (float(self._locked_yolo_target["x"]),
                         float(self._locked_yolo_target["y"])),
                        [(x, y) for x, y, _ in _cxy],
                        list(self._strike_exclude_ref),
                        list(self._other_targets_ref),
                        gate, CONSENSUS_TOL_PX)
                    _now = time.time()
                    if _now - self._sel_gate_log_at > 1.0:
                        self._sel_gate_log_at = _now
                        if _idx is None:
                            self.get_logger().warn(
                                f"[关联] 全局分配拒收本帧: {_ai.get('reason')}"
                                f" e={_ai.get('e')} 内点={_ai.get('inliers', 0)}")
                        elif _ai.get("mode") == "consensus":
                            self.get_logger().info(
                                f"[关联] 全局一致: e={_ai['e']} 内点="
                                f"{_ai['inliers']} 当前目标 d={_ai['cur_d']}px")
                    if _idx is not None:
                        cand = [_cxy[_idx][2]]
                        _ga_accept = True   # 全局一致可放行越门限的正确框
                    else:
                        cand = []
                elif servoing and self._locked_yolo_target is not None and (
                        self._strike_exclude_ref or self._other_targets_ref):
                    _others = (list(self._strike_exclude_ref) +
                               list(self._other_targets_ref))
                    _tdx = anchor_x - float(self._locked_yolo_target["x"])
                    _tdy = anchor_y - float(self._locked_yolo_target["y"])
                    _opred = [(ex + _tdx, ey + _tdy) for ex, ey in _others]
                    _cxy = [(float(b.get("cx", 0)), float(b.get("cy", 0)), b)
                            for b in cand]
                    _near = [t for t in _cxy if gate is None or
                             ((t[0] - anchor_x) ** 2 +
                              (t[1] - anchor_y) ** 2) ** 0.5 <= gate]
                    if _near:
                        _chosen = None
                        if len(_near) == 1:
                            ux, uy, _b = _near[0]
                            _dc = ((ux - anchor_x) ** 2 + (uy - anchor_y) ** 2) ** 0.5
                            _do = min(((ux - ox) ** 2 + (uy - oy) ** 2) ** 0.5
                                      for ox, oy in _opred)
                            _chosen = _b if _dc <= _do else None
                        else:
                            _best, _bscore, _bd = None, -1, 1e18
                            for ux, uy, _b in _near:
                                e2x, e2y = ux - anchor_x, uy - anchor_y
                                _dc = (e2x * e2x + e2y * e2y) ** 0.5
                                _score = 0
                                for ox, oy in _opred:
                                    px, py = ox + e2x, oy + e2y
                                    _corr = [((u - px) ** 2 + (v - py) ** 2) ** 0.5
                                             for (u, v, bb) in _cxy if bb is not _b]
                                    if _corr and min(_corr) <= CONSENSUS_TOL_PX:
                                        _score += 1
                                if _score > _bscore or (
                                        _score == _bscore and _dc < _bd):
                                    _best, _bscore, _bd = _b, _score, _dc
                            _chosen = _best if _bscore > 0 else None
                        if (_chosen is None or len(_near) > 1) and (
                                self._sel_gate_log_at == 0.0 or
                                time.time() - self._sel_gate_log_at > 1.0):
                            self._sel_gate_log_at = time.time()
                            if _chosen is None:
                                self.get_logger().warn(
                                    "[身份核验] anchor 附近候选与当前目标对不上"
                                    "(歧义/疑为已打或旁株)→ 本帧不更新靶点")
                            else:
                                self.get_logger().info(
                                    f"[身份核验] {len(_near)} 个候选逼近 anchor,"
                                    f"共识平移定当前目标,排除其余 {len(_near) - 1} 个")
                        cand = [_chosen] if _chosen is not None else []

                if not cand:
                    # 本帧无可用框(全被过滤/排除) → 不更新靶点,等下一帧
                    self._set_yolo_boxes(boxes)
                    return

                best = min(cand,
                    key=lambda b: (b.get("cx", 0) - anchor_x)**2 +
                                  (b.get("cy", 0) - anchor_y)**2)
                bx, by = float(best.get("cx", 0)), float(best.get("cy", 0))
                d = ((bx - anchor_x)**2 + (by - anchor_y)**2) ** 0.5
                if gate is not None and d > gate and not _ga_accept:
                    # 最近的合规框也超出门限:大概率目标本帧闪烁漏检 →
                    #   重捕获:继续等;跟踪:沿用旧靶点。长时间无合规框由
                    #   PID_TIMEOUT_SEC 兜底 → failed → planner 重试重跳。
                    _now = time.time()
                    if _now - self._sel_gate_log_at > 1.0:
                        self._sel_gate_log_at = _now
                        self.get_logger().warn(
                            f"[选框] 最近合规框({bx:.0f},{by:.0f}) 离锚点"
                            f"({anchor_x:.0f},{anchor_y:.0f}) d={d:.0f}px"
                            f" > 门限{gate:.0f}px,本帧不更新靶点")
                    self._set_yolo_boxes(boxes)
                    return
                # 重捕获成功打一条日志(排查"打重"问题的关键观测点)
                if servoing and self.yolo_target is None:
                    self.get_logger().info(
                        f"[REACQ] 锚点重捕获: 预测({anchor_x:.0f},{anchor_y:.0f})"
                        f" → 选框({bx:.0f},{by:.0f}) d={d:.0f}px"
                        f"  已打排除={len(self._strike_exclude_ref)}个")
                tx, ty = int(bx), int(by)
            else:
                tx = int(data.get("cx", 0))
                ty = int(data.get("cy", 0))

            self.yolo_target = {"x": tx, "y": ty}
            self.yolo_target_at = time.time()

            if boxes:
                self._set_yolo_boxes(boxes)
            else:
                self._yolo_boxes = []

            if self.trigger_mode == "auto" and self.fsm_state == STATE_IDLE:
                now = time.time()
                if (self._auto_last_tx is None or
                    abs(self._auto_last_tx[0] - tx) > 30 or
                    abs(self._auto_last_tx[1] - ty) > 30):
                    self._auto_seen_at = now
                    self._auto_last_tx = (tx, ty)
                elif now - self._auto_seen_at >= SERVO_AUTO_DEBOUNCE:
                    self.get_logger().info(f"[AUTO] 触发: 目标({tx},{ty}) 稳定 {SERVO_AUTO_DEBOUNCE}s")
                    self._start_servo()
                    self._auto_seen_at = now + 999
        except Exception as e:
            self.get_logger().error(f"YOLO 消息解析失败：{e}")

    def _get_rgb(self):
        with self._lock:
            return None if self._rgb_frame is None else self._rgb_frame.copy()

    # ── v3.10.3 PID 调参持久化 ───────────────────────────────
    def _load_pid_tuning(self):
        """从磁盘加载上次保存的 PID 参数。成功则覆盖 self.kp/ki/kd。"""
        try:
            with open(PID_TUNING_FILE, "r") as f:
                d = json.load(f)
            kp = float(d["kp"]); ki = float(d["ki"]); kd = float(d["kd"])
            # 合理性校验：有限、非负、kp 不过大（防手改文件改坏）
            for v in (kp, ki, kd):
                if v != v or v < 0.0 or v > 100.0:   # v!=v 检测 NaN
                    raise ValueError(f"参数越界: {v}")
            self.kp, self.ki, self.kd = kp, ki, kd
            self._pid_tuning_source = "saved"
            self.get_logger().info(
                f"📂 已加载上次保存的 PID 参数: Kp={kp} Ki={ki} Kd={kd}")
            return True
        except FileNotFoundError:
            self._pid_tuning_source = "default"
            self.get_logger().info(
                f"📂 无保存的 PID 参数，使用默认值 "
                f"Kp={self.kp} Ki={self.ki} Kd={self.kd}（网页改动会自动存盘）")
            return False
        except (KeyError, ValueError, TypeError,
                json.JSONDecodeError, OSError) as e:
            self._pid_tuning_source = "default"
            self.get_logger().warn(
                f"⚠️ PID 参数文件损坏/无法读取（{e}），改用默认值。"
                f"文件: {PID_TUNING_FILE}")
            return False

    def _save_pid_tuning(self):
        """把当前 PID 参数写入磁盘。返回是否成功。"""
        try:
            os.makedirs(os.path.dirname(PID_TUNING_FILE), exist_ok=True)
            tmp = PID_TUNING_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump({
                    "kp": self.kp, "ki": self.ki, "kd": self.kd,
                    "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                }, f, indent=2, ensure_ascii=False)
            os.replace(tmp, PID_TUNING_FILE)   # 原子替换，防写一半被读
            self._pid_tuning_source = "saved"
            return True
        except OSError as e:
            self.get_logger().warn(f"⚠️ PID 参数保存失败（{e}）")
            return False

    def _reset_pid_tuning(self):
        """恢复 PID 默认参数并删除存盘文件。"""
        self.kp = PID_KP_DEFAULT
        self.ki = PID_KI_DEFAULT
        self.kd = PID_KD_DEFAULT
        self.pid_x.kp = self.pid_y.kp = self.kp
        self.pid_x.ki = self.pid_y.ki = self.ki
        self.pid_x.kd = self.pid_y.kd = self.kd
        try:
            if os.path.exists(PID_TUNING_FILE):
                os.remove(PID_TUNING_FILE)
        except OSError as e:
            self.get_logger().warn(f"⚠️ 删除 PID 存盘文件失败（{e}）")
        self._pid_tuning_source = "default"
        self.get_logger().info(
            f"♻️ PID 参数已恢复默认: Kp={self.kp} Ki={self.ki} Kd={self.kd}（存盘已清除）")

    # ── v3.10.2 PID 运行态复位 ───────────────────────────────
    def _reset_pid_run_state(self):
        """每轮伺服开始 / 进入 PID 前 / 急停 / 归中时调用，清空 PID 运行态。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.lock_count = 0
        self._pid_best_distance = float("inf")
        self._pid_best_yaw   = None
        self._pid_best_pitch = None
        self._pid_no_improve = 0
        self._last_move = {"yaw": 0, "pitch": 0}
        self._last_cmd_at = 0.0

    # ── 启动伺服 ─────────────────────────────────────────────
    # ── v3.10.4 strike_planner 接口 ──────────────────────────
    def _cb_safety_stop(self, _msg):
        """v3.10.12: 全局急停（/safety_stop）。与网页[紧急停止]同路径:
        激光立即灭（安全动作不等下一拍）+ set 点火中止事件，状态复位经
        v3.10.7 命令漏斗在 FSM timer 线程执行。chassis_controller 订阅同一
        话题自行停车 —— 一个话题,整车(轮+光)同时止动。"""
        all_lasers_off()
        self._fire_cancel.set()
        self._cmd_queue.append("stop")
        self.get_logger().warn("[SAFETY] 收到 /safety_stop → 激光已灭,急停已入队")

    def _cb_recenter(self, _msg):
        """v3.10.10 (P1): ROS 端归中请求（planner 建队前发，亦可手动发布）。
        与 /api/center 同路径：立即 set 点火中止事件（安全动作不等下一拍），
        归中 + 状态复位由 FSM timer 的 _center_and_reset 在下一拍执行。"""
        self._fire_cancel.set()
        self._cmd_queue.append("center")
        self.get_logger().info("[RECENTER] 收到 /servo/recenter → 归中已入队")

    # ── v3.13.0: 任务面板回调(全部纯显示) ────────────────────────
    def _cb_chassis_state(self, msg):
        try:
            self._chassis_state = json.loads(msg.data)
            self._chassis_state_at = time.time()
        except (json.JSONDecodeError, TypeError):
            pass

    def _cb_patch_clear(self, msg):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        d["at"] = time.strftime("%H:%M:%S")
        self._patch_history.append(d)
        if len(self._patch_history) > PATCH_HISTORY_MAX:
            del self._patch_history[:len(self._patch_history)
                                    - PATCH_HISTORY_MAX]

    def _health_tick(self):
        """1Hz 采样相对绿度健康指数(crop 框;无 crop 则本拍无样本)。"""
        try:
            frame = self._get_rgb()
            val, n = compute_green_index(frame, list(self._crop_boxes))
            if val is not None:
                self._health_series.append((round(time.time(), 1), val, n))
                if len(self._health_series) > HEALTH_SERIES_MAX:
                    del self._health_series[:len(self._health_series)
                                            - HEALTH_SERIES_MAX]
        except Exception:
            pass                        # 纯显示,绝不反噬

    # ── v3.11.2: planner 会话状态透传(纯显示,不参与任何决策) ────
    def _cb_planner_session(self, msg):
        """缓存 strike_planner 广播的会话快照,供 8093 网页叠加显示。"""
        try:
            self._planner_session = json.loads(msg.data)
            self._planner_session_at = time.time()
        except (json.JSONDecodeError, TypeError):
            pass    # 显示层数据,坏了就不更新,绝不影响打击

    def _cb_strike_cmd(self, msg):
        """接收 strike_planner 下发的"打这个目标"指令并启动一次打击。"""
        try:
            d = json.loads(msg.data)
            sid = int(d["id"])
            x = int(d["x"]); y = int(d["y"])
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(f"strike_cmd 解析失败：{e}")
            return
        # v3.10.11: 可选 exclude 字段 —— 本片已打目标的中心参考坐标,
        #   选框时排除其邻域(防打重)。字段缺失/坏项静默忽略(向后兼容)。
        ex_list = []
        for p in d.get("exclude", []):
            try:
                ex_list.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError, IndexError):
                pass
        # v3.11.1: 可选 others 字段 —— 本片其它待打目标(中心参考坐标),
        #   身份核验时用于把候选框分类到正确目标,防抓到旁株未打目标。
        other_list = []
        for p in d.get("others", []):
            try:
                other_list.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError, IndexError):
                pass
        self.get_logger().info(
            f"[STRIKE] 收到 planner 指令: id={sid} 目标({x},{y})"
            f"  已打排除={len(ex_list)}个 其它待打={len(other_list)}个")
        # _start_servo 内部会处理"忙/无效"并自行回报 rejected
        self._start_servo(target={"x": x, "y": y}, strike_id=sid,
                          exclude_ref=ex_list, other_ref=other_list)

    # ── v3.12.0: 作业统计记账(纯观测,任何异常都不许影响打击) ────
    def _log_shot(self, strike_id, result, x, y, distance,
                  hit, hit_dist, hit_frames):
        try:
            now = time.time()
            with self._shot_lock:
                self._shots_n += 1
                rec = {
                    "n": self._shots_n, "t": round(now, 2),
                    "id": strike_id, "result": result,
                    "x": x, "y": y, "final_distance": distance,
                    "hit": hit, "hit_dist": hit_dist,
                    "hit_frames": hit_frames,
                    "duration": (round(now - self._servo_t0, 2)
                                 if self._servo_t0 > 0 else None),
                }
                self._shots.append(rec)
                if len(self._shots) > SHOT_MEM_MAX:
                    del self._shots[:len(self._shots) - SHOT_MEM_MAX]
                if STRIKE_LOG_ENABLE and self._shot_fp is None:
                    try:
                        d = os.path.expanduser(STRIKE_LOG_DIR)
                        os.makedirs(d, exist_ok=True)
                        path = os.path.join(
                            d, time.strftime("strike_%Y%m%d_%H%M%S.jsonl"))
                        self._shot_fp = open(path, "a", buffering=1)
                        self.get_logger().info(f"[STATS] 打击日志 → {path}")
                    except OSError as e:
                        self._shot_fp = False
                        self.get_logger().warn(
                            f"[STATS] 日志文件打开失败({e}),仅内存统计")
                if self._shot_fp:
                    self._shot_fp.write(
                        json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception as e:                     # 统计绝不反噬打击
            self.get_logger().warn(f"[STATS] 记账异常忽略: {e}")

    def _publish_strike_result(self, strike_id, result, x, y, distance,
                               hit=None, hit_dist=None, hit_frames=None):
        """向 strike_planner 回报一次打击结果。strike_id 为 None 时不发（手动/auto）。
        v3.11.2: 附加命中判定字段(hit=True/False/None, None=未检出或未启用)。
        planner 只读 id/result,附加字段向后兼容;新版 planner 会聚合进 patch_clear。
        v3.12.0: 本函数是打击终局的汇聚漏斗 —— 先记作业统计再回报;
        rejected(没真正开打)不算一发。手动打击(strike_id=None)只记账不回报。"""
        if result != "rejected":
            self._log_shot(strike_id, result, x, y, distance,
                           hit, hit_dist, hit_frames)
        if strike_id is None:
            return
        msg = String()
        msg.data = json.dumps({
            "id": strike_id, "result": result,
            "x": x, "y": y, "final_distance": distance,
            "hit": hit, "hit_distance": hit_dist, "hit_frames": hit_frames,
        })
        self.pub_strike_result.publish(msg)
        self.get_logger().info(
            f"[STRIKE] 回报结果: id={strike_id} {result} "
            f"d={distance if distance is not None else '--'}"
            f"{'' if hit is None else ('  命中✓' if hit else '  脱靶✗')}")

    def _start_servo(self, target=None, strike_id=None, exclude_ref=None,
                     other_ref=None):
        """启动一次伺服打击。
          target=None      → 用 self.yolo_target（手动按钮 / auto 触发，行为不变）
          target={x,y}     → planner 指定的 RGB 坐标
          strike_id        → planner 的目标 id；非 None 时结束会回报 strike_result
          exclude_ref      → v3.10.11: 本片已打目标坐标(中心参考系),选框排除用
          other_ref        → v3.11.1: 本片其它待打目标坐标(中心参考系),身份核验分类用
        """
        if target is None:
            target = self.yolo_target
        if target is None:
            self.get_logger().warn("无目标，无法启动")
            self._publish_strike_result(strike_id, "rejected", None, None, None)
            return False
        if self.fsm_state not in (STATE_IDLE, STATE_GOT_TARGET, STATE_FAILED):
            self.get_logger().warn(f"当前状态 {self.fsm_state}，请等待完成")
            self._publish_strike_result(strike_id, "rejected",
                                        target.get("x"), target.get("y"), None)
            return False

        # v3.10.10 (P2 修复)：不归中模式下，手动/auto 触发用的是 live 像素 ——
        #   那是【当前姿态】坐标系；绝对角盲跳公式只认【中心参考系】坐标。
        #   云台不在中心时直接启动必瞄偏；且归中后旧坐标也随之失效（画面已平移），
        #   只能拒绝并提示先归中再重新框选。
        #   planner 指令（strike_id 非 None）不受限：其坐标本就是投票期（居中）建账的。
        if (not RECENTER_AFTER_FIRE and strike_id is None and
                (abs(self.servo_yaw - SERVO_YAW_CENTER) > 0.5 or
                 abs(self.servo_pitch - SERVO_PITCH_CENTER) > 0.5)):
            self.get_logger().warn(
                f"⚠️ 不归中模式下云台未在中心（当前 {self.servo_yaw:.0f},"
                f"{self.servo_pitch:.0f}），live 目标坐标不在中心参考系，"
                f"拒绝手动/auto 触发。请先[云台归中]再试。")
            return False

        # v3.10.4: 记录本轮是否 planner 指定（决定结束时是否回报）
        self._strike_cmd_id = strike_id
        self._servo_t0 = time.time()   # v3.12.0: 单株耗时从此刻起算(纯统计)
        # v3.10.11: 已打排除列表 —— 仅 planner 打击携带;手动/auto 触发清空,防残留
        self._strike_exclude_ref = list(exclude_ref) if exclude_ref else []
        # v3.11.1: 其它待打目标列表 —— 身份核验分类用;手动/auto 触发清空
        self._other_targets_ref = list(other_ref) if other_ref else []

        rgb_x, rgb_y = target["x"], target["y"]
        # v3.10.2: 锁存目标 —— 伺服全程冻结，避免 live YOLO 抖动进入闭环
        self._locked_yolo_target = {"x": rgb_x, "y": rgb_y}

        req_x, req_y = self.calib.target_to_required_spot(rgb_x, rgb_y)
        req_x = max(0, min(639, req_x))
        req_y = max(0, min(479, req_y))
        self.required_spot = {"x": req_x, "y": req_y}

        self._reset_pid_run_state()
        self._coarse_phase = None              # 入 COARSE 走非阻塞状态机
        self.fsm_state = STATE_COARSE
        self.get_logger().info(
            f"启动伺服: {'[planner] ' if strike_id is not None else ''}"
            f"YOLO({rgb_x},{rgb_y}) → 应到光斑({req_x},{req_y}) "
            f"[Delta={self.calib.delta_x:+d},{self.calib.delta_y:+d}]  "
            f"(目标已锁存{'，伺服期间冻结' if SERVO_FREEZE_TARGET else ''})"
        )
        return True

    # ── 舵机控制 ─────────────────────────────────────────────
    def _set_yaw_pitch(self, yaw: float, pitch: float):
        self.servo_yaw   = max(45, min(135, yaw))
        self.servo_pitch = max(60, min(120, pitch))
        set_servo(int(round(self.servo_yaw)), int(round(self.servo_pitch)))

    def _set_ir_laser(self, on: bool):
        laser_ir(on)
        self.laser_ir_state = "on" if on else "off"

    def _set_blue_laser(self, on: bool, fire: bool = False):
        laser_blue(on)
        self.laser_blue_state = ("fire" if fire else "on") if on else "off"

    # ── v3.10.2：像素修正量 → 整数度舵机增量（量化感知）─────
    def _pixel_to_int_degree(self, dx_px: float, dy_px: float,
                             max_step_deg=None):
        """像素域修正量 → 整数度舵机增量。返回 (move_yaw, move_pitch)，均为 int。

        - 统一走 PIXEL_TO_*_DEG（含符号），再 round() 成整数度 —— 匹配
          set_pwm_servo 只能吃整数度的硬件约束。
        - move==0 表示该轴已在量化死区内（无法再改善）。
        - max_step_deg: PID 传安全钳位值；COARSE 传 None（只受绝对限位约束）。
        """
        yaw_deg   = dx_px * PIXEL_TO_YAW_DEG
        pitch_deg = dy_px * PIXEL_TO_PITCH_DEG
        if max_step_deg is not None:
            yaw_deg   = max(-max_step_deg, min(max_step_deg, yaw_deg))
            pitch_deg = max(-max_step_deg, min(max_step_deg, pitch_deg))
        return int(round(yaw_deg)), int(round(pitch_deg))

    # ── 实时光斑检测 ─────────────────────────────────────────
    def _detect_spot_now(self):
        """ROI 模式 — 用上一帧 spot / SPOT_HOME 作为搜索中心。
        PID/COARSE 主动转动时禁用 jump 抑制（视场在变是正常的）。
        v3.10.7：硬失败（ROI+全画面都没找到）时清 current_spot，不留脏值。"""
        if self.laser_ir_state != "on":
            self.current_spot = None
            self._last_valid_spot = None
            self._spot_jump_count = 0
            return None
        rgb = self._get_rgb()
        if rgb is None:
            return None

        if self._last_valid_spot is not None:
            hint_x, hint_y = self._last_valid_spot
        else:
            # 静止位（云台居中）时光斑在 SPOT_HOME；伺服中用上一帧光斑跟随
            # （相机+激光同在云台 → 光斑随云台平移，上一帧是好的预测）。
            hint_x, hint_y = SPOT_HOME_X, SPOT_HOME_Y

        raw_spot = find_red_spot(rgb, hint_x, hint_y)
        if raw_spot is None:
            raw_spot = find_red_spot(rgb, None, None)
            if raw_spot is None:
                # v3.10.7 Bug 修复：激光开着但 ROI + 全画面都没找到光斑 →
                #   必须显式清掉 current_spot，否则它保留旧值，_step_pid/_step_coarse
                #   会拿一个陈旧光斑算误差、下发错误命令。置 None 后 _step_pid 会
                #   在该帧 reset PID + 跳过（settle 窗内通常下一帧就恢复检测）。
                self.current_spot = None
                return None

        if self._pid_actively_moving:
            self._spot_jump_count = 0
            self._last_valid_spot = raw_spot
            self.current_spot = {"x": raw_spot[0], "y": raw_spot[1]}
            return raw_spot

        if self._last_valid_spot is not None:
            dx = raw_spot[0] - self._last_valid_spot[0]
            dy = raw_spot[1] - self._last_valid_spot[1]
            if dx*dx + dy*dy > SPOT_JUMP_MAX_PX * SPOT_JUMP_MAX_PX:
                self._spot_jump_count += 1
                if self._spot_jump_count < SPOT_JUMP_TOLERATE_FRAMES:
                    return self._last_valid_spot

        self._spot_jump_count = 0
        self._last_valid_spot = raw_spot
        self.current_spot = {"x": raw_spot[0], "y": raw_spot[1]}
        return raw_spot

    # ── 每帧重算 required_spot ───────────────────────────────
    def _refresh_required_spot(self):
        # v3.10.2: 冻结模式 —— 伺服全程用启动时锁存的目标，不追 live YOLO
        if SERVO_FREEZE_TARGET and self._locked_yolo_target is not None:
            tx, ty = self._locked_yolo_target["x"], self._locked_yolo_target["y"]
        else:
            now = time.time()
            age = now - self.yolo_target_at if self.yolo_target_at > 0 else 1e9
            is_fresh = (self.yolo_target is not None and age < YOLO_TARGET_FRESH_SEC)
            if is_fresh:
                tx, ty = self.yolo_target["x"], self.yolo_target["y"]
            elif (YOLO_FALLBACK_TO_LOCKED and RECENTER_AFTER_FIRE
                  and self._locked_yolo_target is not None):
                # v3.10.10: 锁存坐标是"伺服启动时"的中心参考系坐标，仅归中模式
                #   可作降级来源；不归中模式下与当前画面坐标系不符 →
                #   宁可沿用上一次算好的 required_spot（直接 return 不覆盖）。
                tx, ty = self._locked_yolo_target["x"], self._locked_yolo_target["y"]
            else:
                return

        req_x, req_y = self.calib.target_to_required_spot(tx, ty)
        req_x = max(0, min(639, req_x))
        req_y = max(0, min(479, req_y))
        self.required_spot = {"x": req_x, "y": req_y}

    # ── FSM 主循环（10Hz）────────────────────────────────────
    def _fsm_step(self):
        # v3.10.7: 先在 timer 线程里统一处理 HTTP 压入的控制指令，
        #   使 go/stop/center 的状态变更与本 timer 不再跨线程竞争。
        self._drain_cmd_queue()

        # v3.10.2: _pid_actively_moving = 舵机当前是否在动（影响 _detect_spot_now 的
        #   jump 抑制）。COARSE 全程 True；PID 仅"命令后 settle 窗内"True，
        #   settle 结束后舵机静止 → 恢复 jump 抑制以挡误检。
        now = time.time()
        in_pid_settle = (self.fsm_state == STATE_PID and
                         (now - self._last_cmd_at) < PID_SETTLE_TIME_SEC)
        self._pid_actively_moving = (self.fsm_state == STATE_COARSE) or in_pid_settle

        self._detect_spot_now()

        if self.fsm_state == STATE_IDLE:
            return

        if self.fsm_state == STATE_COARSE:
            self._step_coarse()
        elif self.fsm_state == STATE_PID:
            self._step_pid()
        elif self.fsm_state == STATE_LOCKED:
            self._step_locked()

    # ── v3.10.7: 命令漏斗 ────────────────────────────────────
    def _drain_cmd_queue(self):
        """在 FSM timer 线程里执行 HTTP 压入的控制指令。
        HTTP handler 只 append 指令串；这里 popleft 逐个执行，使所有 FSM
        状态变更都发生在本线程（ROS 回调与本 timer 互斥，故整体单线程安全）。
        紧急停止/归中的"关激光 + set 中止事件"已在 HTTP 端立即做掉（安全动作
        不等下一拍）；这里只做随后的状态复位。"""
        while True:
            try:
                cmd = self._cmd_queue.popleft()
            except IndexError:
                break
            if cmd == "go":
                self._start_servo()                      # 手动触发：用 self.yolo_target
            elif cmd == "stop":
                self._emergency_stop()                   # 幂等（激光已在 HTTP 端关）
            elif cmd == "center":
                self._center_and_reset()

    def _center_and_reset(self):
        """归中云台 + 复位 FSM（原 /api/center 的 body，移到 timer 线程执行）。"""
        center_servo()
        self.servo_yaw = SERVO_YAW_CENTER
        self.servo_pitch = SERVO_PITCH_CENTER
        prev_state = self.fsm_state
        self.fsm_state = STATE_IDLE
        self.error = None
        self.required_spot = None
        self._locked_yolo_target = None
        self._locked_log_done = False
        self._coarse_phase = None
        self._reset_pid_run_state()
        self.get_logger().info(f"云台已归中（FSM: {prev_state} → IDLE）")

    # ── v3.10.0 重写：非阻塞粗对准 ──────────────────────────
    def _step_coarse(self):
        """开环粗对准（三阶段非阻塞状态机）

        物理模型（v3.10.7 更正：相机 + 激光同在云台上，一起随云台转）：
          云台一转 → 相机视场平移：
            - 目标（杂草）在画面里移动（相机在转）
            - 激光光斑也随之扫过地面 → 在画面里移动
          PIXEL_TO_*_DEG = servo_direction_test.py 实测的"云台转 1° 光斑移动
          多少像素"的倒数（含符号）。
        单次开环转动量 = (required_spot − 当前光斑) × PIXEL_TO_*_DEG
          即：转动让光斑落到 required_spot(=目标−Δ)，蓝光随之落到目标上。
        （PIXEL_TO_*_DEG 是实测标量，本式与真实几何一致、model-free。）
        ⚠️ required_spot 里的"目标"取冻结值还是 live YOLO 由 SERVO_FREEZE_TARGET
           决定。相机在云台上时，伺服中相机会平移、目标在画面里会移动，理论上
           应追 live 目标；冻结只适用于"相机固定在车身"。详见 README §A。

        阶段：
          None         → 入口判定：S4 已开 → 进 compute；S4 未开 → 进 laser_warmup
          laser_warmup → 等 COARSE_LASER_WARMUP_SEC（让光斑稳定）→ 进 compute
          compute      → 算偏移 + 下发舵机命令 + 进 settle
          settle       → 等 COARSE_SETTLE_SEC（让舵机+画面到位）→ 进 PID/LOCKED
        """
        now = time.time()

        # ─ 入口 ────────────────────────────────────────────
        if self._coarse_phase is None:
            if self.laser_ir_state != "on":
                self._set_ir_laser(True)
                self._coarse_phase = "laser_warmup"
                self._coarse_phase_started_at = now
                return
            self._coarse_phase = "compute"
            # fallthrough 到 compute

        # ─ 阶段 1: 激光预热 ────────────────────────────────
        if self._coarse_phase == "laser_warmup":
            if now - self._coarse_phase_started_at < COARSE_LASER_WARMUP_SEC:
                return
            self._coarse_phase = "compute"
            # fallthrough

        # ─ 阶段 2: 计算并下发舵机命令 ──────────────────────
        if self._coarse_phase == "compute":
            if RECENTER_AFTER_FIRE:
                # 归中模式：云台在中心，live YOLO 像素就在中心参考系，可刷新
                self._refresh_required_spot()
            # v3.10.10 (P0a 修复)：不归中模式【不刷新】——
            #   required_spot 已在 _start_servo 由锁存的目标坐标（中心参考系）算好。
            #   云台此刻可能停在上一发的歪角；打完→IDLE 间隙 _cb_yolo 仍在更新
            #   yolo_target（那是【歪角姿态】像素），其新鲜窗 0.5s > 预热 0.2s +
            #   tick 0.1s + 指令时延 → _refresh_required_spot 几乎必然采信它，
            #   绝对角公式按"中心参考系"解读歪角坐标 → 盲跳必偏。坐标系不同，禁止混用。
            if self.required_spot is None:
                self.fsm_state = STATE_IDLE
                self._coarse_phase = None
                return

            if not RECENTER_AFTER_FIRE:
                # v3.10.9: 不归中模式 —— 云台可能停在上一发的歪角，不能按"当前光斑"
                #   算相对量（光斑此时不在 SPOT_HOME）。改算【绝对角】：从中心参考位
                #   把光斑送到 required_spot。等价于把蓝光直接压到草上
                #   （required_spot = 草 − Δ），比 Gemini"把草挪到画面中心"更准
                #   （省掉 ~85px 的 pitch 残差）。绝对角指令与当前姿态无关，从任意
                #   歪角都能一跳到位 → 支持打完不归中、直接斜跳下一株。
                dx_pixel = self.required_spot["x"] - SPOT_HOME_X
                dy_pixel = self.required_spot["y"] - SPOT_HOME_Y
                move_yaw, move_pitch = self._pixel_to_int_degree(
                    dx_pixel, dy_pixel, max_step_deg=None)
                abs_yaw   = SERVO_YAW_CENTER + move_yaw
                abs_pitch = SERVO_PITCH_CENTER + move_pitch
                self._set_yaw_pitch(abs_yaw, abs_pitch)
                self.get_logger().info(
                    f"[COARSE-绝对] required_spot"
                    f"({self.required_spot['x']},{self.required_spot['y']}) "
                    f"相对SPOT_HOME偏移=({dx_pixel:+d},{dy_pixel:+d}) "
                    f"→ 绝对角=({abs_yaw:.0f},{abs_pitch:.0f})"
                )
            else:
                # 归中模式（旧行为）：每发归中后云台在中心，按当前实测光斑算相对移动
                if self.current_spot is not None:
                    spot_x, spot_y = self.current_spot["x"], self.current_spot["y"]
                    spot_source = "实测"
                else:
                    spot_x, spot_y = SPOT_HOME_X, SPOT_HOME_Y
                    spot_source = f"SPOT_HOME({SPOT_HOME_X},{SPOT_HOME_Y})"

                dx_pixel = self.required_spot["x"] - spot_x
                dy_pixel = self.required_spot["y"] - spot_y

                # v3.10.2: 完整整数度移动，不做 1.5° 钳位（COARSE 解钳）
                move_yaw, move_pitch = self._pixel_to_int_degree(
                    dx_pixel, dy_pixel, max_step_deg=None)

                self._set_yaw_pitch(self.servo_yaw + move_yaw,
                                    self.servo_pitch + move_pitch)

                self.get_logger().info(
                    f"[COARSE] 光斑({spot_x},{spot_y})[{spot_source}] → "
                    f"目标({self.required_spot['x']},{self.required_spot['y']})  "
                    f"偏移=({dx_pixel:+d},{dy_pixel:+d})  "
                    f"→ 移动({move_yaw:+d}°,{move_pitch:+d}°)  "
                    f"→ 新角度=({self.servo_yaw:.0f},{self.servo_pitch:.0f})"
                )

            self._coarse_phase = "settle"
            self._coarse_phase_started_at = now
            return

        # ─ 阶段 3: 等舵机+画面到位 ─────────────────────────
        if self._coarse_phase == "settle":
            if now - self._coarse_phase_started_at < COARSE_SETTLE_SEC:
                return
            # 沉降完成
            if self.loop_mode == "open_loop":
                self.get_logger().info("[OPEN_LOOP] 粗对准完成，停在 LOCKED 供观察。S4 保持开启。")
                self.fsm_state = STATE_LOCKED
                self._locked_at = now
            else:
                self.fsm_state = STATE_PID
                self._pid_started_at = now
                self._reset_pid_run_state()    # v3.10.2: 进 PID 清空运行态（含 best/no_improve）
                if not RECENTER_AFTER_FIRE:
                    # v3.10.9: 不归中模式 —— 强制清靶点，逼下一帧 YOLO 走 _cb_yolo 的
                    #   "红光斑锚点重捕获"，把刚盲跳到的这株草精准勾过来（不被别株带跑）
                    self.yolo_target = None
                self.get_logger().info("[PID] 进入闭环精对准（量化感知整数步进）")
            self._coarse_phase = None
            return

    # ── v3.10.0 重写：PID 像素域 + settle gate ──────────────
    def _step_pid(self):
        """PID 闭环精对准 —— v3.10.2 量化感知整数步进

        针对 1° PWM 舵机的量化（1°≈10px）：
          - 每步把 PID 像素域输出 round() 成整数度（舵机只能吃整数度）
          - 双轴整数度移动量都为 0 → 已落在最近网格点（≤半个量化台阶）→ 锁定
          - best 跟踪 + 回到最佳点：抗标定误差导致的量化极限环
          - settle gate：命令后 PID_SETTLE_TIME_SEC 内只观测、不下发
        """
        now = time.time()

        # ─ 超时兜底：有像样的 best 就回到 best 锁定（尽量不 FAILED）─
        if now - self._pid_started_at > PID_TIMEOUT_SEC:
            if self._pid_best_yaw is not None and \
               self._pid_best_distance <= SERVO_QUANT_PX * 2.0:
                self.get_logger().warn(
                    f"⚠️ PID 超时但已有可用最佳点 (d={self._pid_best_distance:.1f}px)，"
                    f"回到最佳点锁定")
                self._lock_at_best()
            else:
                self.get_logger().warn(
                    f"⚠️ PID 超时且无可用最佳点 "
                    f"(best d={self._pid_best_distance:.1f}px) → FAILED")
                self._set_ir_laser(False)
                self.fsm_state = STATE_FAILED
                self._pid_actively_moving = False
                # v3.10.4: planner 指定的打击失败 → 回报 failed
                if self._strike_cmd_id is not None:
                    _stgt = self._locked_yolo_target or {}
                    self._publish_strike_result(
                        self._strike_cmd_id, "failed",
                        _stgt.get("x"), _stgt.get("y"), None)
                    self._strike_cmd_id = None
            return

        # v3.10.10 (P0b 修复)：不归中模式 —— 进 PID 时 yolo_target 已被 settle 处
        #   清空，必须等 _cb_yolo 用红斑锚点重捕获到【当前姿态】的目标后再闭环。
        #   此刻若直接 _refresh_required_spot，会 fallback 到锁存的【中心参考系】
        #   坐标——与当前画面坐标系不符，首拍会朝错误方向甩一大步
        #   （最大 MAX_DEG_PER_STEP=5°≈50px），甚至把红斑甩到别株草旁导致锚点抓错。
        #   长时间无检测的兜底交给上面的 PID_TIMEOUT_SEC。
        if not RECENTER_AFTER_FIRE and self.yolo_target is None:
            return

        # ─ 观测（_fsm_step 已调用 _detect_spot_now，这里直接用 current_spot）─
        if self.current_spot is None:
            self.pid_x.reset()
            self.pid_y.reset()
            return

        self._refresh_required_spot()
        if self.required_spot is None:
            self.pid_x.reset()
            self.pid_y.reset()
            return

        ex = self.required_spot["x"] - self.current_spot["x"]
        ey = self.required_spot["y"] - self.current_spot["y"]
        self.error = {"x": ex, "y": ey}
        distance = (ex * ex + ey * ey) ** 0.5

        # ─ settle gate：命令后 settle 窗内只观测，不下发、不推进锁定 ─
        if now - self._last_cmd_at < PID_SETTLE_TIME_SEC:
            return

        # ── settle 已过，舵机已静止，测量可信 ───────────────
        # best 跟踪：记录本轮见过的最小误差及其舵机位置
        if distance < self._pid_best_distance:
            self._pid_best_distance = distance
            self._pid_best_yaw   = self.servo_yaw
            self._pid_best_pitch = self.servo_pitch
            self._pid_no_improve = 0
        else:
            self._pid_no_improve += 1

        # ─ 量化感知：算出本步要移动的整数度 ───────────────
        delta_x_px = self.pid_x.step(ex)
        delta_y_px = self.pid_y.step(ey)
        move_yaw, move_pitch = self._pixel_to_int_degree(
            delta_x_px, delta_y_px, MAX_DEG_PER_STEP)
        self._last_move = {"yaw": move_yaw, "pitch": move_pitch}

        # v3.10.9: 单轴解耦清积分 —— 先到位的那一轴(move==0)单独清掉积分，防止它在
        #   原地憋积分、等另一轴收敛时突然破位过冲。纯 P(Ki=0)下是 no-op，加 Ki 后有用。
        if move_yaw == 0:
            self.pid_x.reset()
        if move_pitch == 0:
            self.pid_y.reset()

        # ─ 锁定判据 1：控制器无法再改善（双轴整数度移动量都为 0）─
        #   → 已落在离目标最近的网格点（≤ 半个量化台阶）
        if move_yaw == 0 and move_pitch == 0:
            self.lock_count += 1
            # 死区内不积分，防 windup
            self.pid_x.reset()
            self.pid_y.reset()
            if self.lock_count >= PID_LOCK_FRAMES:
                self._finish_lock(distance)
            return
        self.lock_count = 0

        # ─ 锁定判据 2：抗量化极限环 —— 连续 N 次命令未刷新最佳 ─
        if self._pid_no_improve >= PID_NO_IMPROVE_LIMIT:
            self.get_logger().info(
                f"[PID] 连续 {self._pid_no_improve} 次命令未改善 "
                f"(best d={self._pid_best_distance:.1f}px)，回到最佳点锁定")
            self._lock_at_best()
            return

        # ─ 执行整数度移动 ─────────────────────────────────
        self._set_yaw_pitch(self.servo_yaw + move_yaw,
                            self.servo_pitch + move_pitch)
        self._last_cmd_at = now           # 启动 settle 窗
        self.get_logger().info(
            f"[PID] 误差=({ex:+d},{ey:+d}) d={distance:.1f}px "
            f"→ 移动({move_yaw:+d}°,{move_pitch:+d}°)  best={self._pid_best_distance:.1f}px"
        )

    def _finish_lock(self, distance):
        """正常锁定：控制器收敛到量化死区（move==0 持续 PID_LOCK_FRAMES 帧）。"""
        if distance <= PID_TOLERANCE_PX:
            self.get_logger().info(
                f"✅ PID 锁定: d={distance:.1f}px ≤ 期望容差 {PID_TOLERANCE_PX}px")
        else:
            self.get_logger().info(
                f"✅ PID 锁定: d={distance:.1f}px（硬件量化极限）。"
                f"期望容差 {PID_TOLERANCE_PX}px 低于 1° 舵机 ~{SERVO_HALF_QUANT_PX:.0f}px "
                f"精度地板，已锁定在可达最优网格点。"
            )
        self._set_ir_laser(False)
        self.fsm_state = STATE_LOCKED
        self._locked_at = time.time()
        self._pid_actively_moving = False

    def _lock_at_best(self):
        """回到本轮最佳舵机位置并锁定（抗极限环 / 超时兜底）。"""
        if self._pid_best_yaw is not None:
            self._set_yaw_pitch(self._pid_best_yaw, self._pid_best_pitch)
        self.get_logger().info(
            f"[PID] 锁定于最佳点 yaw={self.servo_yaw:.0f} pitch={self.servo_pitch:.0f} "
            f"(d≈{self._pid_best_distance:.1f}px)"
        )
        self._set_ir_laser(False)
        self.fsm_state = STATE_LOCKED
        self._locked_at = time.time()
        self._pid_actively_moving = False

    def _step_locked(self):
        """已锁定，开火。开环模式默认不开火（由 fire_in_open_loop 控制）。"""
        if self.loop_mode == "open_loop" and not self.fire_in_open_loop:
            if not self._locked_log_done:
                self.get_logger().info(
                    "[OPEN_LOOP+LOCKED] 停留，未开火（如需开火请在网页勾选「开环也自动开火」）。"
                    "按 [紧急停止] 或 [云台归中] 可重置 FSM 接受下次打击。"
                )
                self._locked_log_done = True
            return
        self._locked_log_done = False
        self.fsm_state = STATE_FIRING
        threading.Thread(target=self._fire_sequence, daemon=False).start()

    def _fire_sequence(self):
        """v3.10.6: 全程可中止的点火序列。
          time.sleep() → _fire_cancel.wait()，紧急停止可立刻把线程叫醒。
          每个 wait 之后双重确认：事件 set 或 fsm_state 已被外部改 → 走中止清理。
        """
        # 进入序列前清空中止旗（前一次序列若已结束遗留 set 状态，这里 reset）
        self._fire_cancel.clear()

        # ─ 阶段 1: 点火前 0.2s 沉降（_lock_at_best 可能刚把云台移回最佳点）─
        if self._fire_cancel.wait(0.20) or self.fsm_state != STATE_FIRING:
            self.get_logger().warn(
                f"⛔ 点火前中止 (fsm_state={self.fsm_state}) → 取消")
            self._cleanup_aborted_fire()
            return

        self.get_logger().info(
            f"⚡ 蓝紫激光(S3) ON → ID={LASER_BLUE_ID}, angle={LASER_ON_ANGLE}, "
            f"持续 {FIRE_DURATION_SEC}s"
        )
        # v3.11.2: 点火前抓取瞄准点(命中判定的比较基准,纯观测)。
        #   优先 live 靶点(邻域跟踪一路咬到锁定,是目标在【当前姿态】画面里的
        #   最新位置);无则用 红斑+Δ(锁定时红斑≈required_spot,+Δ≈目标)。
        _aim = None
        if HIT_CHECK_ENABLE:
            if self.yolo_target is not None:
                _aim = (float(self.yolo_target["x"]),
                        float(self.yolo_target["y"]))
            elif self.current_spot is not None:
                _hx, _hy = self.calib.spot_to_hit(
                    self.current_spot["x"], self.current_spot["y"])
                _aim = (float(_hx), float(_hy))
        self._set_blue_laser(True, fire=True)

        # ─ 阶段 2: 灼烧 ─ wait 返回 True 表示中途被叫醒
        # v3.11.2: 整段 wait 切成 HIT_SAMPLE_PERIOD_SEC 小片,片间采帧检蓝斑
        #   (命中判定,纯观测)。中止语义与 v3.10.6 完全一致:_fire_cancel 一被
        #   set,当前这片 wait 立刻返回 True → 跳出 → 关激光 → 中止清理。
        #   HIT_CHECK_ENABLE=False 或无瞄准点时退化为原版单次 wait,行为零差异。
        _samples = []          # [(x, y)] 灼烧期间检到的蓝斑位置
        _max_score_seen = 0    # 诊断:ROI 内 B−max(G,R) 峰值(调 BLUE_DOMINANCE_MIN 用)
        if HIT_CHECK_ENABLE and _aim is not None:
            _t_end = time.time() + FIRE_DURATION_SEC
            cancelled = False
            while True:
                _remain = _t_end - time.time()
                if _remain <= 0:
                    break
                if self._fire_cancel.wait(min(HIT_SAMPLE_PERIOD_SEC, _remain)):
                    cancelled = True
                    break
                _frm = self._get_rgb()
                if _frm is None:
                    continue
                try:
                    _bs = find_blue_spot(_frm, int(_aim[0]), int(_aim[1]))
                except Exception:
                    _bs = None      # 判定挂了也绝不能影响点火序列
                if _bs is not None:
                    _samples.append((_bs[0], _bs[1]))
                    _max_score_seen = max(_max_score_seen, _bs[2])
        else:
            cancelled = self._fire_cancel.wait(FIRE_DURATION_SEC)
        self._set_blue_laser(False)              # 永远先关激光
        if cancelled:
            self.get_logger().warn("⛔ 灼烧中收到中止 → 提前关激光，跳过收尾")
            self._cleanup_aborted_fire()
            return

        # ── v3.11.2: 灼烧完毕,定命中判定(纯观测,只进日志/回报/网页) ──
        #   三态:hit(中位蓝斑距瞄准点≤HIT_TOL_PX) / miss(检到了但偏出容差) /
        #        unseen(检出帧不足 —— 相机看不见蓝斑,不冤枉打击链路)。
        _hit = None; _hit_dist = None; _hit_frames = len(_samples)
        if HIT_CHECK_ENABLE and _aim is not None:
            if _hit_frames >= HIT_MIN_FRAMES:
                _xs = sorted(p[0] for p in _samples)
                _ys = sorted(p[1] for p in _samples)
                _mx = _xs[len(_xs) // 2]
                _my = _ys[len(_ys) // 2]
                _hit_dist = ((_mx - _aim[0]) ** 2
                             + (_my - _aim[1]) ** 2) ** 0.5
                _hit = bool(_hit_dist <= HIT_TOL_PX)
                _verdict = "hit" if _hit else "miss"
                _hit_pos = {"x": int(_mx), "y": int(_my)}
            else:
                _verdict = "unseen"
                _hit_pos = None
            self._hit_stats[_verdict] += 1
            # 整体赋值(引用原子交换),HTTP 线程只读 → 无竞态
            self._last_hit = {
                "id": self._strike_cmd_id, "verdict": _verdict,
                "dist": (round(_hit_dist, 1) if _hit_dist is not None
                         else None),
                "pos": _hit_pos,
                "aim": {"x": int(_aim[0]), "y": int(_aim[1])},
                "frames": _hit_frames, "at": time.time(),
            }
            if _verdict == "hit":
                self.get_logger().info(
                    f"🎯 [HIT] 命中: 蓝斑中位({_hit_pos['x']},{_hit_pos['y']})"
                    f" 距瞄准点 d={_hit_dist:.1f}px ≤ {HIT_TOL_PX:.0f}px"
                    f"  ({_hit_frames} 帧检出, maxScore={_max_score_seen})")
            elif _verdict == "miss":
                self.get_logger().warn(
                    f"❌ [HIT] 脱靶: 蓝斑中位({_hit_pos['x']},{_hit_pos['y']})"
                    f" 距瞄准点 d={_hit_dist:.1f}px > {HIT_TOL_PX:.0f}px"
                    f"  ({_hit_frames} 帧检出, maxScore={_max_score_seen})")
            else:
                self.get_logger().warn(
                    f"❓ [HIT] 蓝斑未检出({_hit_frames} 帧 < {HIT_MIN_FRAMES},"
                    f" maxScore={_max_score_seen}) —— 判定弃权,不算脱靶。"
                    f" 若持续如此,按 README 降 BLUE_DOMINANCE_MIN 或核对曝光")

        self.get_logger().info(f"   蓝紫激光 OFF，冷却 {FIRE_COOLDOWN_SEC}s")
        self.fsm_state = STATE_COOLDOWN

        # ─ 阶段 3: 冷却 ─
        if self._fire_cancel.wait(FIRE_COOLDOWN_SEC):
            self.get_logger().warn("⛔ 冷却中收到中止 → 跳过收尾")
            self._cleanup_aborted_fire()
            return

        # ─ 阶段 4: 正常收尾 ─
        self.fsm_state = STATE_IDLE
        self.error = None
        self.current_spot = None
        self.required_spot = None
        # v3.10.4: 先抓取回报所需信息（下面 _locked_yolo_target / best 会被清空）
        _sid  = self._strike_cmd_id
        _stgt = self._locked_yolo_target or {}
        _sdist = (self._pid_best_distance
                  if self._pid_best_distance != float("inf") else None)
        self._locked_yolo_target = None
        self._reset_pid_run_state()              # v3.10.2: 清空 best/no_improve 等运行态
        self._strike_cmd_id = None
        self.get_logger().info("   伺服周期完成，回到 IDLE 等待下个目标")

        # ─ 收尾归中（v3.10.7 更正理由）────────────────────────
        # 相机 + 激光同在云台：打完这一发，云台停在本株草角度，**相机视场已平移**。
        # strike_planner 的多目标队列是在"参考位（云台居中）"那一帧建的，坐标只在
        # 参考位有效。所以打完必须归中、把相机转回参考位，剩下队列里那些草的像素
        # 坐标才仍然成立、下一发才不会瞄错。**相机在云台时归中是必需项。**
        #   RECENTER_AFTER_FIRE=True：每发归中（相机在云台必须这样）。
        #   =False：仅当已实现"按云台角对队列坐标做补偿变换"时可用（高级，README §B）。
        if RECENTER_AFTER_FIRE:
            center_servo()
            self.servo_yaw   = SERVO_YAW_CENTER
            self.servo_pitch = SERVO_PITCH_CENTER
            # PWM 舵机走完回中行程约需 ~0.4s（最坏 45°→90° 行程）
            if self._fire_cancel.wait(0.4):
                self.get_logger().warn("⛔ 归中中收到中止")
                self._cleanup_aborted_fire()
                return
            self.get_logger().info(
                f"   云台归中至 ({SERVO_YAW_CENTER},{SERVO_PITCH_CENTER})，"
                f"相机回参考位，多目标队列坐标仍有效")
        else:
            self.get_logger().info(
                f"   不归中（RECENTER_AFTER_FIRE=False），云台留在 "
                f"({self.servo_yaw:.0f},{self.servo_pitch:.0f})；"
                f"队列坐标为中心参考系，执行层按绝对角盲跳（与当前姿态无关），"
                f"无需归中即可打下一株")

        # v3.10.4: 状态已置 IDLE 后再回报 success —— 避免 planner 收到结果即抢发
        #          下一条 strike_cmd 时本节点还在 COOLDOWN 而被拒。
        # v3.11.2: 附带命中判定(hit/hit_dist/hit_frames),planner 聚合进战报。
        # v3.12.0: 不再限 _sid —— 手动打击也进作业统计(函数内 sid=None 只记账不回报)。
        self._publish_strike_result(_sid, "success",
                                    _stgt.get("x"), _stgt.get("y"), _sdist,
                                    hit=_hit,
                                    hit_dist=(round(_hit_dist, 1)
                                              if _hit_dist is not None
                                              else None),
                                    hit_frames=_hit_frames)

    def _cleanup_aborted_fire(self):
        """v3.10.6: 点火序列被中止时的最小清理。
        激光确保关、planner 报失败、清 PID 运行态。fsm_state **不在这里改**——
        若是 _emergency_stop 触发的，它已经设了 IDLE；尊重外部置位。"""
        self._set_blue_laser(False)
        self._set_ir_laser(False)
        if self._strike_cmd_id is not None:
            tgt = self._locked_yolo_target or {}
            self._publish_strike_result(
                self._strike_cmd_id, "failed",
                tgt.get("x"), tgt.get("y"), None)
            self._strike_cmd_id = None
        self._locked_yolo_target = None
        self._reset_pid_run_state()

    def _fire_test_thread(self, duration: float = 0.5):
        # v3.10.7: 与正式点火序列一致，用可中断 wait 取代阻塞 sleep。
        #   紧急停止/归中 set 了 _fire_cancel 时立即提前结束并确保关激光。
        self._fire_cancel.clear()
        self.get_logger().info(
            f"🧪 [S3 测试] 蓝紫激光烧 {duration}s ID={LASER_BLUE_ID} angle={LASER_ON_ANGLE}"
        )
        self._set_blue_laser(True, fire=True)
        cancelled = self._fire_cancel.wait(duration)
        self._set_blue_laser(False)              # 永远先关激光
        if cancelled:
            self.get_logger().warn("🧪 [S3 测试] 收到中止 → 提前关激光")
        else:
            self.get_logger().info("🧪 [S3 测试] 完成")

    def _emergency_stop(self):
        all_lasers_off()
        # v3.10.6: 先关激光、再立刻通知点火线程中止（如果它正在 sleep）
        self._fire_cancel.set()
        self.laser_ir_state = "off"
        self.laser_blue_state = "off"
        self.fsm_state = STATE_IDLE
        self.error = None
        self.required_spot = None
        self._locked_yolo_target = None
        self._locked_log_done = False
        # v3.10.2: 重置粗对准 + PID 全部运行态
        self._coarse_phase = None
        self._reset_pid_run_state()
        self.get_logger().warn("🛑 紧急停止：所有激光关闭，FSM → IDLE")

    # ── HTTP ─────────────────────────────────────────────────
    def _start_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send_json(self, data, code=200):
                body = json.dumps(data).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def _send_jpeg(self, frame):
                if frame is None:
                    blank = np.zeros((480, 640, 3), np.uint8)
                    cv2.putText(blank, "Waiting for RGB camera...", (60, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 200), 2)
                    frame = blank
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    self.send_response(500); self.end_headers(); return
                data = buf.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = self.path.split("?")[0]
                qs = {}
                if "?" in self.path:
                    for kv in self.path.split("?", 1)[1].split("&"):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            qs[k] = v

                if path in ("/", "/index.html"):
                    body = HTML_PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == "/frame":
                    self._send_jpeg(node._get_rgb())
                    return

                if path == "/api/state":
                    now = time.time()
                    yolo_age = (now - node.yolo_target_at) if node.yolo_target_at > 0 else None
                    yolo_fresh = (yolo_age is not None and yolo_age < YOLO_TARGET_FRESH_SEC)
                    predicted_hit = None
                    if node.current_spot is not None and node.calib.calib2_done:
                        px = node.current_spot["x"] + node.calib.delta_x
                        py = node.current_spot["y"] + node.calib.delta_y
                        predicted_hit = {"x": px, "y": py}
                    # v3.10.0: settle gate 剩余时间（仅 PID 状态有意义）
                    settle_remaining = None
                    if node.fsm_state == STATE_PID and node._last_cmd_at > 0:
                        elapsed = now - node._last_cmd_at
                        settle_remaining = max(0.0, PID_SETTLE_TIME_SEC - elapsed)
                    # v3.10.6: 伺服期间 "yolo" 字段送冻结目标 —— 与 PID 实际用的一致，
                    #         避免前端被 33Hz live YOLO 抖动误导成"系统不收敛"
                    servoing = node.fsm_state in (STATE_COARSE, STATE_PID,
                                                  STATE_LOCKED, STATE_FIRING,
                                                  STATE_COOLDOWN)
                    if (SERVO_FREEZE_TARGET and servoing
                            and node._locked_yolo_target is not None):
                        effective_yolo = node._locked_yolo_target
                        target_frozen = True
                    else:
                        effective_yolo = node.yolo_target
                        target_frozen = False
                    # ── v3.11.2: 决策叠加层的坐标平移量(纯显示) ──
                    # planner 会话里的目标坐标是【中心参考系】;eye-in-hand 下云台
                    # 一转全场同量平移。前端要把这些点画到当前画面上,需要一个平移:
                    #   · 云台在中心 → 平移 (0,0),直接画(投票/建队期正是这时);
                    #   · 伺服中且有 live 靶点 + 锁存中心参考坐标 → 平移 =
                    #     靶点 − 锁存(与 _cb_yolo 身份核验用的是同一个量);
                    #   · 其它情况 → null,前端不画叠加(只显示侧栏列表),不画错图。
                    at_center = (abs(node.servo_yaw - SERVO_YAW_CENTER) <= 0.5
                                 and abs(node.servo_pitch
                                         - SERVO_PITCH_CENTER) <= 0.5)
                    session_shift = None
                    if at_center:
                        session_shift = {"dx": 0, "dy": 0}
                    elif (servoing and node.yolo_target is not None
                          and node._locked_yolo_target is not None):
                        session_shift = {
                            "dx": node.yolo_target["x"]
                                  - node._locked_yolo_target["x"],
                            "dy": node.yolo_target["y"]
                                  - node._locked_yolo_target["y"]}
                    _lh = node._last_hit
                    last_hit_age = ((now - _lh["at"])
                                    if _lh is not None else None)
                    _shots_snap = list(node._shots)   # v3.12.0: 快照隔离并发
                    # v3.13.0: 健康序列快照 + 相对基线
                    _hs = list(node._health_series)
                    _health = None
                    if _hs:
                        _base = _hs[0][1]
                        _cur = _hs[-1][1]
                        _health = {
                            "cur": _cur, "n_box": _hs[-1][2],
                            "baseline": _base,
                            "delta_pct": (round((_cur - _base) / _base * 100, 1)
                                          if _base > 1e-6 else None),
                            "series": [[t, v] for t, v, _ in _hs[-240:]],
                        }
                    psess_age = ((now - node._planner_session_at)
                                 if node._planner_session_at > 0 else None)
                    self._send_json({
                        "trigger":   node.trigger_mode,
                        "loop_mode": node.loop_mode,
                        "fsm_state": node.fsm_state,
                        "yolo":      effective_yolo,      # v3.10.6: 伺服期间=冻结值
                        "yolo_live": node.yolo_target,    # v3.10.6: 永远是 live 值（调试用）
                        "target_frozen": target_frozen,   # v3.10.6: 前端可据此切换样式
                        "yolo_age_sec": yolo_age,
                        "yolo_fresh": yolo_fresh,
                        "required":  node.required_spot,
                        "spot":      node.current_spot,
                        "error":     node.error,
                        "lock_frames": node.lock_count,
                        "lock_target": PID_LOCK_FRAMES,
                        "yaw":   node.servo_yaw,
                        "pitch": node.servo_pitch,
                        "laser_ir":   node.laser_ir_state,
                        "laser_blue": node.laser_blue_state,
                        "kp": node.kp, "ki": node.ki, "kd": node.kd,
                        "locked": node.lock_count >= PID_LOCK_FRAMES,
                        "fire_in_open_loop": node.fire_in_open_loop,
                        "calib2_stale": getattr(node.calib, 'calib2_frame', None) is not None and getattr(node.calib, 'calib2_frame', '') != 'rgb',
                        "calib2_frame": getattr(node.calib, 'calib2_frame', 'rgb'),
                        "calib2_done":  node.calib.calib2_done,
                        "delta_x": node.calib.delta_x,
                        "delta_y": node.calib.delta_y,
                        "predicted_hit": predicted_hit,
                        "yolo_boxes": node._yolo_boxes,
                        # v3.11.1: 显示框年龄(秒)→ 前端画陈旧/新鲜;ExG 当前(已下发)状态
                        "yolo_boxes_age_sec": (
                            (now - node._yolo_boxes_at)
                            if node._yolo_boxes_at > 0 else None),
                        "exg_enable": node._exg_ui_state,
                        # v3.10.2 新增字段
                        "pid_settle_remaining_sec": settle_remaining,
                        "pid_settle_time_sec":      PID_SETTLE_TIME_SEC,
                        "quant_px":                 SERVO_QUANT_PX,
                        "half_quant_px":            SERVO_HALF_QUANT_PX,
                        "pid_best_distance": (
                            None if node._pid_best_distance == float("inf")
                            else node._pid_best_distance),
                        "last_move":                node._last_move,
                        "pid_tuning_source":        node._pid_tuning_source,
                        # v3.10.1: YOLO 频率指令（让前端滑块加载时显示真实值）
                        "yolo_cmd_freq":            node.yolo_cmd_freq,
                        # ── v3.11.2: 命中判定 + 决策会话(纯显示字段) ──
                        "last_hit":          _lh,
                        "last_hit_age_sec":  last_hit_age,
                        "hit_stats":         node._hit_stats,
                        "planner_session":   node._planner_session,
                        "planner_session_age_sec": psess_age,
                        "session_shift":     session_shift,
                        "at_center":         at_center,
                        # ── v3.12.0: 作业统计 ──
                        "shots":             _shots_snap[-60:],
                        "shot_stats":        compute_shot_stats(_shots_snap),
                        "hit_tol_px":        HIT_TOL_PX,
                        # ── v3.13.0: 任务面板 ──
                        "chassis":           node._chassis_state,
                        "chassis_age_sec":   ((now - node._chassis_state_at)
                                              if node._chassis_state_at > 0
                                              else None),
                        "patch_history":     node._patch_history[-10:],
                        "health":            _health,
                    })
                    return

                if path == "/api/trigger":
                    m = qs.get("m", "manual")
                    if m in ("manual", "auto"):
                        node.trigger_mode = m
                        node.get_logger().info(f"触发模式 → {m}")
                    self._send_json({"ok": True})
                    return

                # v3.11.1: ExG 假草过滤运行时开关。网页按钮 → 发布 Bool 到
                #   TOPIC_EXG_ENABLE → yolo_detector 订阅后即时开/关(无需重启)。
                if path == "/api/exg":
                    on = qs.get("on", "1") == "1"
                    node._exg_ui_state = on
                    bmsg = Bool(); bmsg.data = on
                    node.pub_exg_enable.publish(bmsg)
                    node.get_logger().info(
                        f"[ExG] 网页开关 → 下发 {'开启' if on else '关闭'}")
                    self._send_json({"ok": True, "exg_enable": on})
                    return

                if path == "/api/loop":
                    m = qs.get("m", "closed_loop")
                    if m in ("open_loop", "closed_loop"):
                        node.loop_mode = m
                        node.get_logger().info(f"伺服模式 → {m}")
                    self._send_json({"ok": True})
                    return

                if path == "/api/pid":
                    saved = False
                    try:
                        node.kp = float(qs.get("kp", node.kp))
                        node.ki = float(qs.get("ki", node.ki))
                        node.kd = float(qs.get("kd", node.kd))
                        node.pid_x.kp = node.pid_y.kp = node.kp
                        node.pid_x.ki = node.pid_y.ki = node.ki
                        node.pid_x.kd = node.pid_y.kd = node.kd
                        # v3.10.3: 改完即存盘 → 下次启动自动加载
                        saved = node._save_pid_tuning()
                        node.get_logger().info(
                            f"PID 更新: Kp={node.kp} Ki={node.ki} Kd={node.kd}  "
                            f"{'(已存盘)' if saved else '(存盘失败)'}")
                    except ValueError:
                        pass
                    self._send_json({"ok": True, "saved": saved})
                    return

                # v3.10.3: 恢复 PID 默认参数并清除存盘
                if path == "/api/pid_reset":
                    node._reset_pid_tuning()
                    self._send_json({"ok": True, "kp": node.kp,
                                     "ki": node.ki, "kd": node.kd})
                    return

                if path == "/api/go":
                    # v3.10.7: 压入命令队列，由 FSM timer 执行 _start_servo。
                    #   返回 queued=True（实际启动结果通过轮询 /api/state 的 fsm_state 看）。
                    node._cmd_queue.append("go")
                    self._send_json({"ok": True, "queued": True})
                    return

                if path == "/api/stop":
                    # v3.10.7: 安全动作立即做（关激光 + 叫醒点火线程），
                    #   状态复位延后到 FSM timer（命令漏斗）。
                    all_lasers_off()
                    node._fire_cancel.set()
                    node.laser_ir_state = "off"
                    node.laser_blue_state = "off"
                    node._cmd_queue.append("stop")
                    self._send_json({"ok": True})
                    return

                if path == "/api/laser_ir":
                    on = qs.get("on", "0") == "1"
                    node._set_ir_laser(on)
                    node.get_logger().info(f"[手动] S4 RED → {'ON' if on else 'OFF'}")
                    self._send_json({"ok": True, "ir": node.laser_ir_state})
                    return

                if path == "/api/laser_blue":
                    on = qs.get("on", "0") == "1"
                    node._set_blue_laser(on)
                    node.get_logger().warn(
                        f"[手动] S3 BLUE → {'ON' if on else 'OFF'}  "
                        f"(ID={LASER_BLUE_ID}, angle={LASER_ON_ANGLE if on else 0})"
                    )
                    self._send_json({"ok": True, "blue": node.laser_blue_state})
                    return

                if path == "/api/mission_start":
                    node.pub_ch_start.publish(Empty())
                    node.get_logger().info("[任务] 网页发车 → /chassis/start")
                    self._send_json({"ok": True}); return
                if path == "/api/mission_stop":
                    node.pub_ch_stop.publish(Empty())
                    node.get_logger().info("[任务] 网页收工 → /chassis/stop")
                    self._send_json({"ok": True}); return
                if path == "/api/estop":
                    # 双保险:广播全局急停 + 本地立即执行(不等自订阅回环)
                    node.pub_safety.publish(Empty())
                    try:
                        node._cb_safety_stop(None)
                    except Exception:
                        pass
                    node.get_logger().warn("[任务] 🛑 网页急停 → /safety_stop + 本地立停")
                    self._send_json({"ok": True}); return

                if path == "/api/stats_reset":
                    # v3.12.0: 清零内存统计(演示开跑前按);磁盘 JSONL 不动
                    with node._shot_lock:
                        node._shots = []
                        node._shots_n = 0
                    node._hit_stats = {"hit": 0, "miss": 0, "unseen": 0}
                    node._last_hit = None
                    node.get_logger().info("[STATS] 作业统计已重置(网页触发)")
                    self._send_json({"ok": True})
                    return

                if path == "/api/fire_test":
                    if node.fsm_state in (STATE_FIRING, STATE_COOLDOWN):
                        self._send_json({"ok": False, "msg": "正在开火中"}, 400)
                        return
                    try:
                        dur = float(qs.get("dur", "0.5"))
                        dur = max(0.1, min(2.0, dur))
                    except ValueError:
                        dur = 0.5
                    threading.Thread(
                        target=node._fire_test_thread, args=(dur,), daemon=False
                    ).start()
                    self._send_json({"ok": True, "duration": dur})
                    return

                if path == "/api/fire_open_toggle":
                    on = qs.get("on", "0") == "1"
                    node.fire_in_open_loop = on
                    node.get_logger().info(
                        f"[配置] 开环模式自动开火 → {'启用' if on else '禁用'}"
                    )
                    self._send_json({"ok": True, "fire_in_open_loop": on})
                    return

                # v3.10.1: YOLO 发布频率指令（补前端滑块缺失的后端路由）
                # 注意：本路由只把"期望频率"发到 TOPIC_YOLO_FREQ_CMD 话题。
                # 真正生效需要 YOLO 检测节点订阅该话题并重建发布 timer。
                if path == "/api/set_yolo_freq":
                    try:
                        freq = float(qs.get("freq", str(YOLO_FREQ_DEFAULT)))
                    except (ValueError, TypeError):
                        self._send_json(
                            {"success": False, "message": "freq 不是合法数字"}, 400)
                        return
                    if not (YOLO_FREQ_MIN <= freq <= YOLO_FREQ_MAX):
                        self._send_json(
                            {"success": False,
                             "message": f"freq 必须在 {YOLO_FREQ_MIN:.0f}-"
                                        f"{YOLO_FREQ_MAX:.0f} Hz"},
                            400)
                        return
                    node.yolo_cmd_freq = freq
                    m = Float32()
                    m.data = float(freq)
                    node.pub_yolo_freq.publish(m)
                    node.get_logger().info(
                        f"[YOLO 频率] 指令 → {freq:.1f} Hz "
                        f"(已发布到 {TOPIC_YOLO_FREQ_CMD}；"
                        f"是否生效取决于 YOLO 节点有无订阅)"
                    )
                    self._send_json({"success": True, "freq": freq})
                    return

                # v3.10.8: 相机曝光/增益（除草线调试用）。关键：同步设 RGB+IR 两台
                #   相机，保持两者一致，避免 NDVI 的红/近红外比值因曝光不匹配而失真。
                #   只在 vision_servo 运行期临时覆盖；NDVI 线另有锁定预设。
                if path == "/api/set_cam":
                    exp_s = qs.get("exposure")
                    gain_s = qs.get("gain")
                    applied = {}
                    for cam_name, dev in (("rgb", RGB_DEVICE), ("ir", IR_DEVICE)):
                        ok = True
                        try:
                            if exp_s is not None:
                                exp = max(1, min(10000, int(exp_s)))
                                subprocess.run(
                                    ["v4l2-ctl", "-d", dev, "-c", "auto_exposure=1"],
                                    timeout=2, capture_output=True)
                                r1 = subprocess.run(
                                    ["v4l2-ctl", "-d", dev, "-c",
                                     f"exposure_time_absolute={exp}"],
                                    timeout=2, capture_output=True)
                                ok = ok and (r1.returncode == 0)
                            if gain_s is not None:
                                gain = max(0, min(128, int(gain_s)))
                                r2 = subprocess.run(
                                    ["v4l2-ctl", "-d", dev, "-c", f"gain={gain}"],
                                    timeout=2, capture_output=True)
                                ok = ok and (r2.returncode == 0)
                        except Exception as e:
                            ok = False
                            node.get_logger().warn(f"[set_cam] {cam_name} 失败: {e}")
                        applied[cam_name] = ok
                    node.get_logger().info(
                        f"[set_cam] exposure={exp_s} gain={gain_s} "
                        f"→ RGB+IR 同步, 结果={applied}")
                    self._send_json({"success": all(applied.values()),
                                     "exposure": exp_s, "gain": gain_s,
                                     "applied": applied})
                    return

                if path == "/api/center":
                    # v3.10.7: 立即通知点火线程中止（安全动作不等下一拍），
                    #   归中 + 状态复位延后到 FSM timer 的 _center_and_reset。
                    node._fire_cancel.set()
                    node._cmd_queue.append("center")
                    self._send_json({"ok": True, "queued": True})
                    return

                self.send_response(404); self.end_headers()

            def do_POST(self):
                """v3.14.0: ASR 语音识别端点。接收浏览器 MediaRecorder 音频 POST。"""
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length) if length > 0 else b""
                path = self.path.split("?")[0]

                if path == "/api/asr":
                    if node.asr is None:
                        self._send_json({"ok": False, "text": "",
                                         "command": None,
                                         "msg": "ASR 已被禁用"})
                        return
                    if not node.asr.available:
                        self._send_json({"ok": False, "text": "",
                                         "command": None,
                                         "msg": ASR_DEV_MODE_MSG})
                        return
                    try:
                        result = node.asr.recognize(body)
                        node._asr_last_result = result
                        self._send_json({
                            "ok": True,
                            "text": result.get("text", ""),
                            "command": result.get("command"),
                            "confidence": result.get("confidence", 0.0),
                            "ms": result.get("ms", 0.0),
                        })
                    except Exception as e:
                        node.get_logger().error(f"[ASR] 推理异常: {e}")
                        self._send_json({"ok": False, "text": "",
                                         "command": None, "msg": str(e)})
                    return

                self._send_json({"ok": False}, 404)

        def serve():
            server = HTTPServer(("0.0.0.0", SERVO_HTTP_PORT), Handler)
            server.timeout = 0.5  # v3.14.0: 允许 POST body 完整接收
            server.serve_forever()

        threading.Thread(target=serve, daemon=True).start()


# ══════════════════════════════════════════════════════════════
#  v3.10.0: MultiThreadedExecutor
#  使用默认 MutuallyExclusive callback group（不引入新并发风险）。
#  好处：HTTP 线程和 ROS 回调彻底解耦；callback 短暂阻塞不会拖垮流水线。
#  注：本节点的 callback 已经全部非阻塞（移除了 _step_coarse 里的 sleep），
#       所以单线程也能工作；这里用多线程是防御性配置。
# ══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = VisionServoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        all_lasers_off()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# ══════════════════════════════════════════════════════════════
#  v3.10.1 —— YOLO 检测节点侧需要补的代码（给队友参考，不属于本文件）
# ══════════════════════════════════════════════════════════════
#  vision_servo 的频率滑块只是把"期望频率"发到 /yolo/cmd_freq。
#  要让它真正生效，YOLO 检测节点必须订阅这个话题并重建发布 timer。
#  把下面的逻辑加到 YOLO 检测节点里：
#
#  ── import ──
#    from std_msgs.msg import Float32
#
#  ── __init__ 里 ──
#    self.publish_freq = 10.0
#    self.detect_timer = self.create_timer(
#        1.0 / self.publish_freq, self.detect_and_publish)
#    # 订阅 vision_servo 发来的频率指令
#    self.create_subscription(
#        Float32, "/yolo/cmd_freq", self._on_freq_cmd, 10)
#
#  ── 新增回调 ──
#    def _on_freq_cmd(self, msg):
#        freq = max(1.0, min(30.0, float(msg.data)))
#        if abs(freq - self.publish_freq) < 0.01:
#            return  # 没变化，忽略
#        self.publish_freq = freq
#        # rclpy 的 timer 周期不能原地改 → 销毁重建
#        self.destroy_timer(self.detect_timer)
#        self.detect_timer = self.create_timer(
#            1.0 / freq, self.detect_and_publish)
#        self.get_logger().info(f"YOLO 发布频率 → {freq:.1f} Hz")
#
#  注意：
#    1. vision_servo 用 TRANSIENT_LOCAL QoS 发布，YOLO 节点用默认（VOLATILE）
#       QoS 订阅是兼容的——晚启动也能收到最后一次指令。
#    2. 频率指令话题名 "/yolo/cmd_freq" 两边必须一致（本文件 TOPIC_YOLO_FREQ_CMD）。
#    3. 若 YOLO 节点没接这段，滑块不会报错，但 YOLO 实际帧率不会变。
# ══════════════════════════════════════════════════════════════
