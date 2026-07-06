# Laser-Targeted Weeding & Multispectral Crop Sensing on Horizon RDK X5

**English** | [简体中文](./README_cn.md)

> An edge-AI agricultural robot **proof-of-concept** built on the Horizon **RDK X5** (10 TOPS BPU). It closes a full **perception → decision → execution** loop: YOLOv8 weed detection on the BPU, NDVI crop-health estimation on the CPU, multi-target planning with duplicate-strike rejection, and an **Eye-in-Hand** gimbal that aligns a laser onto each weed using **laser-spot visual-servo feedback** — an IBVS-style closed loop requiring only two coarse calibration constants (no image Jacobian, robust to calibration error). Runs on ROS 2. Verified indoors with printed targets and UV-sensitive photo paper.

This repository contains the full competition source code, released under the **MIT License**. Reproduction and derivative work are welcome.

---

## Contents

- [1. System Architecture](#1-system-architecture)
- [2. Core Algorithms](#2-core-algorithms)
- [3. Hardware](#3-hardware)
- [4. Software Modules & ROS 2 Nodes](#4-software-modules--ros-2-nodes)
- [5. Repository Layout](#5-repository-layout)
- [6. Dependencies](#6-dependencies)
- [7. Build & Install](#7-build--install)
- [8. Running](#8-running)
- [9. Web Dashboard](#9-web-dashboard)
- [10. Key Parameters](#10-key-parameters)
- [11. Model Training & Conversion (.pt → .onnx → .bin)](#11-model-training--conversion-pt--onnx--bin)
- [12. Deploying on the RDK X5 BPU](#12-deploying-on-the-rdk-x5-bpu)
- [13. Performance](#13-performance)
- [14. Known Limitations & Roadmap](#14-known-limitations--roadmap)
- [15. License](#15-license)

---

## 1. System Architecture

The system follows a **perception → decision → execution** pipeline. All modules run on the RDK X5 and communicate through decoupled ROS 2 topics:

```
                 ┌─────────────────────────────────────────────────────┐
                 │                     RDK X5 (10 TOPS)                  │
  RGB + 850nm ─┬►│  Perception  stereo_camera → yolo_detector (BPU)      │
  dual-band    │ │              └────────────► ndvi_node (CPU, NDVI)     │
  stereo cam   │ │                                                       │
               │ │  Decision   strike_planner  vote/queue→sort→dedup     │
               │ │                     │ strike cmd (with struck/pending)│
               │ │  Execution  vision_servo  blind-jump→re-acquire→      │
               │ │                            servo loop→fire            │
               │ │                     │ PWM / GPIO                      │
               │ │  Motion     chassis_controller  cruise-stop-clear-roll│
                 └─────────────────────────────────────────────────────┘
   Gimbal + red aiming laser + 405nm working laser ◄─ rigidly co-mounted
   with the camera (Eye-in-Hand)
```

**Data flow**: the dual-band camera feeds two independent branches — the BPU branch runs YOLOv8 weed detection while the CPU branch computes NDVI (fully decoupled). When the chassis brakes on weed detection, the planner votes and builds a strike queue, dispatching targets one by one; the servo node blind-jumps, re-acquires via the laser spot, closes the loop, and fires; results are reported back for bookkeeping. When a patch is cleared, the chassis rolls forward to the next one.

---

## 2. Core Algorithms

The system keeps working accurately although the camera itself moves with the gimbal (Eye-in-Hand), which makes image coordinates drift continuously. Two frames are used throughout: the **center reference frame** (pixel coordinates captured with the gimbal centered — the "original coordinates" used for bookkeeping) and the **current frame**. Each algorithm below links to its source file.

### 2.1 Absolute-Angle Blind Jump

> 📄 `laser_calibration/vision_servo.py`

Each target's center-reference coordinate $(u,v)$ is mapped **independently** to an absolute gimbal angle (coarse aiming). With centered angle $\theta_c$, laser-spot home $(u_0,v_0)$, degrees-per-pixel $k$ (calibrated), and spot-to-hit offset $\Delta=(\Delta u,\Delta v)$:

$$\theta_{yaw}=\theta_{c,yaw}+\big((u-\Delta u)-u_0\big)\times k_{yaw}$$

**Key property**: every jump is solved from the center reference frame in absolute terms — never incrementally from the previous target — so angular error **does not accumulate across targets**.

### 2.2 Laser-Spot Feedback Servoing

> 📄 `laser_calibration/vision_servo.py`

Under Eye-in-Hand, the red aiming spot stays nearly fixed in the image. After the blind jump, the spot pixel $s=(s_x,s_y)$ is detected and the **anchor** (predicted hit point of the working laser = where the target should appear) is:

$$\text{anchor}=s+\Delta$$

Among candidate weed boxes $\{b_i\}$ in the current frame, the target is the one nearest to the anchor within gate $R$:

$$b^\*=\arg\min_i\lVert b_i-\text{anchor}\rVert_2,\quad \lVert b^\*-\text{anchor}\rVert\le R$$

**Only two coarse calibration constants, robust to calibration error**: the whole system calibrates just $k$ and $\Delta$ (rough values suffice) — no image Jacobian, no high-precision hand–eye extrinsics. Because the anchor is grounded in the **physically measured** spot position, residuals from calibration, gear backlash and servo quantization are **absorbed automatically**. This distinguishes the approach from classic IBVS that depends on an interaction matrix.

Spot detection itself uses layered tests for field robustness: R−max(G,B) redness score → brightness gate (rejects dark-red soil) → morphological closing → area window + circularity filter (rejects elongated glare) → ROI hint for speed.

### 2.3 Consensus-Translation Identity Check

> 📄 `laser_calibration/vision_servo.py` (selection) · `strike_planner.py` (sends struck/pending sets)

With multiple targets and a moving camera, a candidate box near the anchor may actually be an **already-struck** weed. Let the current target's reference coordinate be $C_2$ and other targets (struck + pending) be $\{P_j\}$. The reference-to-current translation is $\tau=\text{anchor}-C_2$, giving predictions $P_j'=P_j+\tau$. When several candidates crowd the anchor, each candidate $b$ is hypothesized as the true target ($e_2=b-\text{anchor}$) and scored by how many *other* predictions are corroborated by a *different* detection within tolerance $\tau_0$:

$$\text{score}(b)=\#\{\,j:\exists\,b_k\ne b,\ \lVert b_k-(P_j'+e_2)\rVert\le \tau_0\,\}$$

The highest-scoring candidate wins; if none is corroborated, the frame is safely rejected (no strike). **Why it works**: under Eye-in-Hand the whole scene shares one translation, so only the true target's $e_2$ aligns all other boxes simultaneously — a wrong candidate (e.g. an already-struck weed) cannot. Conceptually this is camera-motion compensation + data association from multi-object tracking. **Field-tested: it intercepted repeated strikes on cleared weeds.**

### 2.4 Quantization-Aware Servo Control

> 📄 `laser_calibration/vision_servo.py`

The 1° PWM servos quantize motion; one degree spans $q=1/k\approx10\text{–}20$ px depending on working distance. The loop converts pixel error $e$ into integer-degree steps:

$$\Delta\theta_{cmd}=\operatorname{round}(K_p\cdot k\cdot e)$$

Locking criterion: **integer step = 0** (error below half a step), i.e. the gimbal settles on the best reachable grid point; the historical best is tracked and restored if no improvement persists. **Why**: the controller *acknowledges* the ~5 px quantization floor and stops once the laser can already hit, instead of oscillating between adjacent grid points — achieving pixel-level aiming near the physical limit of a cheap servo ("low-cost hardware + quantization-aware algorithm = high precision").

### 2.5 Multi-Target Planning

> 📄 `laser_calibration/strike_planner.py`

With the gimbal centered, detections are voted over multiple frames (filters single-frame false positives) to build the queue $\{T_i\}$ in reference coordinates; targets are sorted greedily by distance to minimize total gimbal travel; each dispatch carries the **struck set $S$** and **pending set $Q$** for identity checking; success commits $S\leftarrow S\cup\{T\}$, failure re-queues the target for retry. **This is the single decision entry of the system** — a single target is simply a queue of length 1 going through the same vote–queue–dispatch–commit flow (the web manual/auto triggers are development-only and disabled by default). When the queue empties, `patch_clear` releases the chassis.

### 2.6 NDVI with Vicarious Calibration

> 📄 `laser_calibration/ndvi_node.py` · `calib_diffuse.py`

Instead of naively plugging raw DN values into the formula, NDVI uses low-cost **vicarious calibration**: dark-current subtraction plus a PTFE diffuse-panel (gray-card) $K$ correction that brings the R / NIR channels onto one radiometric basis:

$$NDVI=\frac{K\cdot NIR'-R'}{K\cdot NIR'+R'},\quad R'=DN_R-dark_R,\ \ K=\frac{R'_{gray}}{NIR'_{gray}}$$

Three modes degrade automatically: ① active calibration (near-true NDVI) → ② reflectance-card calibration → ③ relative (pseudo) NDVI fallback. Distance/material effects cancel in the ratio; recalibrate when lighting changes.

**Time-series growth monitoring**: relative NDVI is not comparable across devices, but is temporally consistent under "same device, same calibration, same field". The system therefore supports day-by-day acquisition over the same crops, using the **relative trend** for growth monitoring and stress early-warning — a sustained drop typically precedes visible yellowing, flagging disease or drought stress early. With the PTFE panel calibrated, near-true health grading is available. The robot thus works in a "**weed while you scout**" mode: every pass both clears weeds and checks crop health.

> On the reference panel: laboratory panels (Spectralon) are expensive mainly for their NIST-traceable absolute reflectance. Our $K$ is a two-channel **ratio** correction that only requires the panel to be spectrally flat and diffusely stable across R/NIR — an ordinary PTFE sheet satisfies this at ~1% of the cost.

---

## 3. Hardware

**Platform statement**: this project is built on the **Yahboom RDK X5 Mecanum-wheel robot car**. The chassis, motor driver (`Mcnamu_driver`) and the `Rosmaster_Lib` low-level library come from the vendor; the **dual-band camera–laser gimbal integration and the entire upper software stack (12 ROS 2 nodes for detection / NDVI / planning / servoing / chassis FSM) are designed and implemented by our team**.

| Part | Spec | Role |
|---|---|---|
| Main board | Horizon RDK X5 (10 TOPS BPU) | Vision, inference, planning & control |
| Camera | RGB + 850 nm NIR dual-band stereo | Weed detection + NDVI input |
| Gimbal | 2-DoF PWM servos (S1 yaw / S2 pitch) | Points camera + lasers |
| Aiming laser | Red | Physical feedback reference (the "spot") |
| Working laser | 405 nm blue-violet (low power, prototype) | Targeted exposure; quantified on UV photo paper |
| Chassis | Mecanum-wheel mobile base | Autonomous row cruising |

> Safety: the prototype uses a low-power 405 nm laser with UV-sensitive photo paper to record and quantify hit accuracy indoors, in place of real weed burning. Always follow laser-safety practice and wear wavelength-matched goggles.

---

## 4. Software Modules & ROS 2 Nodes

Package `laser_calibration` — `ros2 pkg executables laser_calibration` lists 12 nodes.

**Main pipeline**
| Node | Description |
|---|---|
| `stereo_camera` | Dual-band stereo capture & streaming |
| `yolo_detector` | YOLOv8 on the BPU; optional ExG green filter (runtime-switchable via topic) |
| `ndvi_node` | R/NIR registration + NDVI with 3-tier calibration (active→refl→relative), see 2.6 |
| `vision_servo` | Blind jump → spot re-acquire (identity check) → quantization-aware loop → laser; built-in web dashboard (:8093) |
| `strike_planner` | Vote/queue → greedy sort → dispatch → dedup → retry |
| `chassis_controller` | Cruise–brake–clear–blind-roll finite state machine |

**NDVI monitor & calibration tools**: `ndvi_monitor`, `calib_camera`, `calib_laser` (spot-to-hit Δ), `calib_refl` / `calib_diffuse` (dark current + PTFE gray-card K), `show_calib`.

---

## 5. Repository Layout

```
laser_calibration/
├── laser_calibration/           # ROS 2 node sources
│   ├── stereo_camera.py         ├── yolo_detector.py
│   ├── ndvi_node.py             ├── ndvi_monitor.py
│   ├── vision_servo.py          ├── strike_planner.py
│   ├── chassis_controller.py    ├── chassis_fsm.py
│   ├── config.py                # global HW/topic/parameters — read this first
│   └── calib_*.py               # calibration tools
├── models/                      # model & quantization artifacts
│   ├── best.pt / best.onnx      # trained weights / exported ONNX (opset 11)
│   ├── quant.bin                # BPU model produced by the D-Robotics toolchain
│   ├── quant_info.json          # per-layer cosine similarity, etc.
│   └── config_yolov8.yaml       # hb_mapper conversion config
├── package.xml · setup.py · resource/
```

> The ~100 field images used for quantization calibration are not shipped (only needed when re-quantizing); see §11.

---

## 6. Dependencies

- **Board**: RDK X5 with the official Ubuntu image, ROS 2 + TogetheROS.Bot (TROS.B)
- **BPU runtime**: D-Robotics `hobot_dnn` (pre-installed) loads `quant.bin`
- **Python**: `rclpy`, `opencv-python`, `numpy`, `cv_bridge`
- **Chassis/servo SDK**: Yahboom `Rosmaster_Lib` (ships with the car)
- **Model conversion (PC, optional)**: `ultralytics` for training/export; D-Robotics **OpenExplorer / hb_mapper** for quantization (§11)

> Default workspace path is `/home/sunrise/yahboomcar_ws` (the stock `sunrise` user on RDK boards). Adjust the model path in `config.py` if yours differs.

---

## 7. Build & Install

```bash
cd ~/yahboomcar_ws/src           # place laser_calibration/ here
cd ~/yahboomcar_ws
colcon build --packages-select laser_calibration
source install/setup.bash

# verify
ros2 pkg executables laser_calibration | wc -l    # expect 12
python3 -c "import laser_calibration; print(laser_calibration.__version__)"
md5sum src/laser_calibration/models/quant.bin
```

---

## 8. Running

Source the workspace **in every terminal**: `source ~/yahboomcar_ws/install/setup.bash`, then start in order (chassis driver first):

```bash
ros2 run yahboomcar_bringup Mcnamu_driver        # T0 chassis driver (vendor)
ros2 run laser_calibration stereo_camera          # T1 camera
ros2 run laser_calibration yolo_detector          # T2 detector (BPU)
ros2 run laser_calibration vision_servo           # T3 servo + web :8093
ros2 run laser_calibration strike_planner         # T4 planner
ros2 run laser_calibration chassis_controller     # T5 chassis FSM
ros2 run laser_calibration ndvi_node              # (optional) NDVI
ros2 run laser_calibration ndvi_monitor           # (optional) NDVI monitor
```

Triggers (any sourced terminal):

```bash
ros2 topic pub --once /chassis/start          std_msgs/msg/Empty '{}'   # go
ros2 topic pub --once /chassis/stop           std_msgs/msg/Empty '{}'   # stop work
ros2 topic pub --once /safety_stop            std_msgs/msg/Empty '{}'   # e-stop (wheels + lasers)
ros2 topic pub --once /planner/start_clearing std_msgs/msg/Empty '{}'  # bench clearing (no driving)
```

> The keyboard teleop node and `chassis_controller` both publish `cmd_vel` — never run them together.

---

## 9. Web Dashboard

`vision_servo` serves `http://<RDK_IP>:8093`: live camera view with detection boxes, aim point and laser spot; trigger mode, gimbal centering, exposure and laser switches; and a runtime **ExG filter toggle** (no restart needed).

---

## 10. Key Parameters

Mostly in `config.py` (hardware/topics/ISP/calibration) and the top of `vision_servo.py` (servo loop):

| Parameter | Where | Meaning |
|---|---|---|
| `PIXEL_TO_*_DEG` | config / vision_servo | Degrees per pixel (calibrated per working distance) |
| `SPOT_HOME_*` | config | Laser-spot home position |
| `REACQ_MAX_DIST_PX` | vision_servo | Re-acquisition gate (default 70) |
| `IDENTITY_MARGIN_PX` / `CONSENSUS_TOL_PX` | vision_servo | Identity-check parameters |
| `STOP_VOTE_FRAMES` | chassis_fsm | Consecutive weed frames required to brake |
| `BLIND_ROLL_SEC` | chassis_fsm | Post-clearing blind roll (escapes the cleared patch) |
| `CRUISE_SPEED_MPS` | chassis_fsm | Cruise speed |
| `EXG_FILTER_ENABLE` | config | Default ExG state (runtime-overridable via topic) |

---

## 11. Model Training & Conversion (.pt → .onnx → .bin)

Artifacts are shipped in `models/`. To reproduce:

**Training (PC, Ultralytics)** — YOLOv8, `imgsz=640`, `nc=2` `{0: weed, 1: crop}`. Crucially, the training set includes **hard-negative soil/background samples**, which eliminated empty-scene false locks. Reference metrics: mAP@0.5 = 0.913, mAP@0.5:0.95 = 0.665, P = 0.910, R = 0.866.

**ONNX export (keep the stock head)**
```bash
yolo export model=best.pt format=onnx opset=11 imgsz=640 simplify=True
```
Input `images [1,3,640,640]`, output `output0 [1,6,8400]`, no NMS. ⚠️ Do **not** use "split-head" export recipes from some model zoos — they break the post-processing in `yolo_detector.py`.

**Quantization (D-Robotics OpenExplorer / hb_mapper)** — config in `models/config_yolov8.yaml`; key settings actually used:

| Item | Value |
|---|---|
| `march` | `bayes-e` |
| `input_type_rt` / `input_type_train` | `nv12` / `rgb` |
| `norm_type` / `scale_value` | `data_scale` / `0.003921568627451` (=1/255) |
| `calibration_data` | ~100 field images (`calibration_size=100`) |
| `calibration_type` | `default` |
| `optimization` | `set_Softmax_input_int8,set_Softmax_output_int8` |
| `compile_mode` / `optimize_level` | `latency` / `O3`, int8 |

Calibration images should match deployment conditions (lighting/distance/background) — they directly determine quantization quality. Result: per-layer cosine similarity recorded in `quant_info.json`; `output0` reaches **0.9991**, so the negative-sample behavior ("no false alarm on bare soil") survives quantization intact. Output: `quant.bin` (~5.9 MB), NV12 input `1x3x640x640`, FP32 output `[1,6,8400]`.

---

## 12. Deploying on the RDK X5 BPU

1. Place `quant.bin` under `models/`. The loader path in `config.py` defaults to
   `/home/sunrise/yahboomcar_ws/src/laser_calibration/models/quant.bin`
   (model-only swaps need no rebuild; code changes do).
2. `yolo_detector` loads the bin via D-Robotics `hobot_dnn`; frames are converted to **NV12 640×640**, and the `[1,6,8400]` output is decoded + thresholded/NMS'd in the node.
3. Start it:
   ```bash
   ros2 run laser_calibration yolo_detector
   # success looks like:  [BPU] frame=... inference=~29ms boxes=N
   ```
4. Verify `md5sum models/quant.bin` against your record.

---

## 13. Performance

| Metric | Value |
|---|---|
| Detection mAP@0.5 / @0.5:0.95 | 0.913 / 0.665 |
| End-to-end inference | ≈ 29 ms/frame ≈ 34 FPS (RDK X5 BPU) |
| Pure BPU latency (toolchain estimate) | 5.71 ms (theoretical 175 FPS) |
| Aiming accuracy (median) | ≈ 5 px static / ≈ 6 px moving |
| Typical accuracy range | 2–12 px (near the ~5 px 1°-servo quantization floor) |
| Per-weed cycle | ≈ 5–6 s |
| Multi-target clearing success | 100 % static / ≥ 90 % moving (optimized layout) |
| Robustness | Auto retry on failure; identity check field-proven against duplicate strikes |

> Metrics aggregate multiple bench (static) and driving (moving) test runs; the demo video records one full clearing run among them.

---

## 14. Known Limitations & Roadmap

- **Whole-field autonomy**: add visual/laser SLAM mapping and localization, upgrading row-following into full-field path planning — a "map–scout–weed" closed loop combined with NDVI patrol data.
- **Strike point = bounding-box center**: may land on a leaf rather than the stem base; keypoint detection of the growing point would raise kill efficiency.
- **View generalization**: misses occur under some viewpoints/lighting; broaden training data or add persistent tracking (SORT-family) for stable IDs.
- **Fixed quantization dead-zone**: could adapt to working distance (px-per-degree) or be tunable from the dashboard.
- **Stop-and-Go**: continuous strike-while-driving with motion compensation is the next efficiency step.
- **Laser power**: the prototype validates with low-power 405 nm + photo paper, not real weed-burning power.

---

## 15. License

Released under the [MIT License](./LICENSE). This is a competition **proof-of-concept**; use it as a reference platform for visual servoing and edge-AI agriculture. Keep the copyright notice when redistributing.
