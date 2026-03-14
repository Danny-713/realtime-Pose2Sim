# Pose2Sim 实时化可行性分析报告

## 一、当前管线架构总览

Pose2Sim 的处理管线由 8 个阶段串联组成，每个阶段目前都是**离线批处理**模式——先把所有帧处理完一个阶段，再进入下一个阶段。

```
Calibration → PoseEstimation → Synchronization → PersonAssociation
    → Triangulation → Filtering → MarkerAugmentation → Kinematics(.osim/.mot)
```

数据流：

```
视频/图像
    ↓
[poseEstimation] RTMPose (rtmlib) → JSON (OpenPose 格式)
    ↓
[calibration] 相机标定 → .toml
    ↓
[synchronization] 多相机同步（互相关计算偏移）
    ↓
[personAssociation] 跨视角人物关联
    ↓
[triangulation] 加权 DLT 三角化 → .trc (3D 标记)
    ↓
[filtering] 3D 轨迹滤波
    ↓
[markerAugmentation] (可选) LSTM 虚拟标记增强
    ↓
[kinematics] OpenSim Scaling + IK → .osim (缩放模型) + .mot (关节角)
```

---

## 二、各阶段实时化可行性逐一分析

### 1. Calibration（标定） — ✅ 无障碍

标定是离线一次性完成的，与实时无关。相机内外参标定好后可以一直复用。

**结论：不构成实时化障碍。**

---

### 2. Pose Estimation（2D 姿态估计） — ✅ 可实时

当前代码已经是逐帧处理的：

```python
# poseEstimation.py:351
keypoints, scores = pose_tracker(frame)
```

RTMPose (rtmlib) 本身就支持实时推理，在 GPU 上单帧延迟约 **10-30ms**。只需将输入从视频文件切换为摄像头流 (`cv2.VideoCapture(0)`) 即可。

**结论：完全可实时化，是最容易改造的环节。**

---

### 3. Synchronization（多相机同步） — ⚠️ 批处理，但可绕过

当前使用关键点垂直速度的互相关来计算相机时间偏移，需要完整序列。但在实时场景下：

- **硬件同步**（硬件触发器）可直接避免此问题
- 或在采集前做一次同步校准，得到固定偏移量后直接复用

**结论：通过硬件同步或预标定可绕过，不构成根本障碍。**

---

### 4. Person Association（人物关联） — ✅ 可实时

当前是批量处理所有帧的 JSON，但核心算法（基于重投影误差或极线距离的亲和度匹配）是逐帧独立的。改造为逐帧调用不存在算法障碍。

**结论：可实时化。**

---

### 5. Triangulation（三角化） — ✅ 核心逐帧，后处理需适配

三角化的**核心计算是逐帧**的：

```python
# triangulation.py:556
for f in tqdm(range(*f_range)):
    # 每帧独立做加权 DLT 三角化
```

每帧独立地对每个关键点做加权 DLT 三角化 + 重投影误差检验 + 相机筛选。这部分可以直接实时化。

**但后处理依赖完整序列：**

- `interpolate_zeros_nans`：插值填补缺失值，需要前后帧
- `indices_of_first_last_non_nan_chunks`：寻找有效片段
- TRC 文件的整体写入

在滑动窗口方案下，这些可以改为局部插值或完全跳过（实时场景容忍少量缺失）。

**结论：核心可实时化，后处理需适配。**

---

### 6. Filtering（滤波） — ⚠️ 需选择合适的滤波器

| 滤波器 | 能否实时 | 原因 |
|--------|---------|------|
| Butterworth (filtfilt) | ❌ 不能 | 零相位双向滤波，需完整序列 |
| Kalman (smooth=True) | ❌ 不能 | RTS smoother 需完整序列 |
| **Kalman (smooth=False)** | ✅ **可以** | 单向前向滤波，代码已注明 "if real-time" |
| **OneEuro** | ✅ **可以** | 逐点自适应滤波，天然适合实时 |
| Gaussian | ⚠️ 滑窗可用 | 但有边界效应 |
| Median | ⚠️ 滑窗可用 | 但有边界效应 |
| GCV Spline / LOESS | ❌ 不能 | 需完整序列 |

Config.toml 中已有相关注释：

```toml
smooth = true # should be true, unless you need real-time filtering
# Simpler and even faster alternative to Kalman filter for real-time
```

**结论：选用 Kalman(smooth=False) 或 OneEuro 滤波器即可实时化。**

---

### 7. Marker Augmentation（LSTM 标记增强） — ⚠️ 可适配

LSTM 模型理论上可以逐帧运行（ONNX 推理），但 LSTM 的隐藏状态依赖时序上下文。在滑动窗口方案下可以维持隐藏状态连续性。单帧推理时间约 **1-5ms**，不构成瓶颈。

**结论：可实时化，但可能需要跳过或做轻量化适配。**

---

### 8. Kinematics / OpenSim IK — ❗ 最关键的瓶颈

当前实现使用 `InverseKinematicsTool` 一次性处理整个时间范围：

```python
# kinematics.py:508-509
opensim.InverseKinematicsTool(str(ik_path_temp)).run()
```

这是当前**最大的实时化障碍**。

**解决方案：** OpenSim 提供了逐帧 IK 求解的 API——`opensim.InverseKinematicsSolver`，支持：

- 加载缩放模型
- 对每一帧设置 marker 坐标
- 调用 `solve()` 得到该帧的关节角度
- 可以实现逐帧或滑动窗口求解

此外，Scaling（模型缩放）只需在首次（或定期）执行一次，不需要每帧重复。

**结论：需要将 `InverseKinematicsTool` 替换为 `InverseKinematicsSolver` 逐帧求解，技术上可行但需要重写。**

---

## 三、各阶段实时化汇总表

| 模块 | 当前模式 | 能否实时化 | 主要瓶颈 | 改造难度 |
|------|---------|-----------|---------|---------|
| Calibration | 一次性 | ✅ 无需改造 | 无 | — |
| Pose Estimation | 逐帧 | ✅ 可直接实时 | GPU 推理延迟 | ⭐ 低 |
| Synchronization | 整序列 | ✅ 可绕过 | 需硬件同步 | ⭐ 低 |
| Person Association | 整序列 | ✅ 核心逐帧 | 算法已逐帧 | ⭐⭐ 中 |
| Triangulation | 逐帧+后处理 | ✅ 核心逐帧 | 后处理需适配 | ⭐⭐ 中 |
| Filtering | 整序列 | ✅ 需换滤波器 | 选 Kalman/OneEuro | ⭐ 低 |
| Marker Augmentation | 整序列 | ⚠️ 可适配 | LSTM 隐藏状态 | ⭐⭐ 中 |
| **Kinematics (IK)** | **整序列** | **⚠️ 需重写** | **OpenSim API 替换** | **⭐⭐⭐ 高** |

---

## 四、老师的建议评估：滑动窗口管线（1 秒输出一次）

> "是不是能实现建立处理管线，按照时间滑动窗口，比如 1s 输出一次"

### 结论：**完全可行。**

### 时间预算分析

假设帧率 30fps，窗口 1 秒 = 30 帧：

| 阶段 | 每窗口耗时估算 | 说明 |
|------|---------------|------|
| Pose Estimation (RTMPose, GPU) | ~30 × 15ms = 450ms | 可并行多相机 |
| Person Association | ~30 × 1ms = 30ms | 逐帧匹配 |
| Triangulation | ~30 × 2ms = 60ms | 逐帧加权 DLT |
| Filtering (Kalman/OneEuro) | ~30 × 0.1ms = 3ms | 极快 |
| Marker Augmentation (ONNX) | ~30 × 2ms = 60ms | 可跳过 |
| OpenSim IK (per-frame solver) | ~30 × 5-10ms = 150-300ms | 关键瓶颈 |
| **总计** | **~750-900ms** | **< 1s，可行** |

### 关键约束

1. **相机数量**：Pose Estimation 是按相机数倍增的。4 台相机串行 × 450ms = 1800ms，但可以多线程/多 GPU 并行
2. **OpenSim IK 单帧**：~5-10ms 取决于模型复杂度，simple model 更快
3. **I/O 开销**：当前大量使用 JSON 文件 I/O，实时化后应改为内存传递

---

## 五、推荐实现方案

### 方案 A：滑动窗口管线（推荐 ⭐，对应老师建议）

```
┌─────────────────────────────────────────────────┐
│              预处理（一次性）                      │
│  1. 相机标定 (Calibration)                       │
│  2. 相机同步 (硬件同步或预标定偏移)                │
│  3. 模型缩放 (前几秒数据 Scaling → .osim)         │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│           实时循环（每 1s / 30 帧）               │
│                                                  │
│  ┌──────────┐  多相机并行                        │
│  │Camera 1  │──→ RTMPose → 2D keypoints          │
│  │Camera 2  │──→ RTMPose → 2D keypoints          │
│  │Camera N  │──→ RTMPose → 2D keypoints          │
│  └──────────┘                                    │
│       ↓                                          │
│  Person Association（逐帧）                      │
│       ↓                                          │
│  Triangulation（逐帧加权 DLT）                   │
│       ↓                                          │
│  Kalman Filter / OneEuro（前向递推，维持状态）    │
│       ↓                                          │
│  [可选] Marker Augmentation（LSTM，维持 hidden） │
│       ↓                                          │
│  OpenSim IK（InverseKinematicsSolver，逐帧）     │
│       ↓                                          │
│  输出：追加写入 .mot / 实时可视化                 │
└─────────────────────────────────────────────────┘
```

### 方案 B：纯逐帧管线（更低延迟，精度略降）

不做窗口积累，每帧完成整个管线：

- 延迟：~20-40ms/帧（取决于硬件）
- 适合对延迟敏感、对精度要求稍低的场景
- 跳过插值和 Marker Augmentation

### 方案 C：双线程生产者-消费者模型

```
Thread 1（采集线程）: 相机采集 → Pose Estimation → 写入帧缓冲区
Thread 2（处理线程）: 从缓冲区读取 → Triangulation → Filter → IK → 输出
```

管线化可以隐藏延迟，提高吞吐量。

---

## 六、需要改造的关键代码

### 1. 输入源改造

将 `cv2.VideoCapture(video_path)` 改为多相机实时流 `cv2.VideoCapture(camera_id)`。

### 2. 数据传递改造

将所有基于文件 I/O（JSON → TRC → .mot）的中间结果改为**内存中的数据结构**（numpy array / pandas DataFrame），消除磁盘读写瓶颈。

### 3. 滤波器选择

强制使用 Kalman(smooth=False) 或 OneEuro，它们本身已在 `filtering.py` 中实现。

### 4. OpenSim IK 改造（最关键）

将 `InverseKinematicsTool` 替换为 `InverseKinematicsSolver` 的逐帧调用：

```python
import opensim

# 初始化（一次性）
model = opensim.Model('scaled_model.osim')
state = model.initSystem()
marker_refs = opensim.MarkersReference(...)
ik_solver = opensim.InverseKinematicsSolver(
    model, marker_refs,
    opensim.SimTKArrayCoordinateReference(), 0
)
ik_solver.assemble(state)

# 实时循环
for frame_markers in streaming_data:
    # 更新 marker 位置
    # ik_solver.track(state)
    joint_angles = get_angles_from_state(state)
    append_to_mot(joint_angles)
```

### 5. 输出方式改造

- **追加写入 .mot**：每个窗口追加新行到 .mot 文件
- **Socket/共享内存**：通过 ZMQ/TCP 将关节角实时推送给 OpenSim GUI 或自定义可视化器
- **实时可视化**：用 `opensim.Visualizer` 或自定义 3D 渲染器（如 Open3D、PyQtGraph）

---

## 七、可能遇到的困难

| 困难 | 严重程度 | 解决方案 |
|------|---------|---------|
| OpenSim `InverseKinematicsSolver` Python 绑定不完善 | 中 | 可用 C++ 调用或用轻量 IK 替代（如 scipy 优化） |
| 多相机 Pose Estimation 串行导致延迟 | 中 | 多进程 / 多 GPU 并行 |
| 实时可视化与 OpenSim GUI 集成 | 中 | 用独立可视化方案（Open3D、PyQtGraph） |
| 无硬件同步导致的帧不对齐 | 高 | 必须硬件同步或时间戳对齐 |
| Kalman 单向滤波精度不如 filtfilt | 低 | 可接受的精度损失 |
| 初始几秒需要 Scaling | 低 | 前 3-5 秒作为初始化期 |

---

## 八、最终结论

### ✅ 实时化是可行的

项目 README 的 roadmap 本身就将此列为未来目标：

> ☐ Real-time: Run Pose estimation, Person association, Triangulation, Kalman filter, IK frame by frame (instead of running each step for all frames)

### ✅ 老师提出的滑动窗口方案完全可以实现

理由：

1. 管线中大部分核心算法（Pose Estimation、Triangulation、Person Association）**已经是逐帧的**
2. 滤波器有现成的实时选项（Kalman single-pass、OneEuro）
3. OpenSim IK 有逐帧 API（`InverseKinematicsSolver`）
4. 1 秒窗口给了足够的计算余量（估算总耗时 ~750-900ms < 1s）
5. 主要工作是**架构改造**（文件 I/O → 内存流水线），而非算法重写

### 建议的实施优先级

1. **第一步**：实现单相机实时 Pose Estimation + Triangulation（验证基本流程）
2. **第二步**：集成 Kalman/OneEuro 实时滤波
3. **第三步**：改造 OpenSim IK 为逐帧求解
4. **第四步**：多相机并行 + 滑动窗口完整管线
5. **第五步**：实时可视化输出
