# Pose2Sim 实时化实施方案（定稿）

> 本文档为 Pose2Sim 在线显示 OpenSim 结果的最终实施方案，综合了三轮技术讨论的共识与修正。
>
> 所有性能数字均标注为"估算/待实测"，技术路线经过代码级验证，但 OpenSim Python API 的在线行为仍需阶段 0 实测确认。

---

## 一、目标定义

**在线显示 OpenSim 结果**——不是普通 3D 骨架可视化，而是：

1. OpenSim 模型姿态实时更新（API Visualizer 窗口）
2. OpenSim 关节角度实时输出（.mot 追加写入或数据流推送）

**不是**以下目标：

- 把现有离线管线简单加速
- 在 OpenSim GUI 应用中实时播放
- 一次性全模块实时化重构

---

## 二、当前管线现状

### 2.1 离线管线流程

```
Calibration → PoseEstimation → Synchronization → PersonAssociation
    → Triangulation → Filtering → MarkerAugmentation → Kinematics(.osim/.mot)
```

### 2.2 各阶段在线化基础

| 模块 | 当前模式 | 在线化基础 | 改造难度 |
|------|---------|-----------|---------|
| Calibration | 一次性 | 无需改造，直接复用 | — |
| Pose Estimation | 逐帧（RTMPose） | 核心已逐帧，改输入源即可 | 低 |
| Synchronization | 整序列互相关 | 通过硬件同步或预标定绕过 | 低 |
| Person Association | 整序列处理 | 核心算法逐帧独立 | 中 |
| Triangulation | 逐帧 + 整序列后处理 | 核心三角化逐帧，后处理需适配 | 中 |
| Filtering | 整序列 | 需切换为实时兼容滤波器 | 低 |
| Marker Augmentation | 整序列 ONNX | 第一版跳过 | — |
| Kinematics (IK) | 整序列 Tool 调用 | 需替换为 Solver 逐帧/小窗口 | 高 |

### 2.3 三个关键阻塞点

**阻塞点 1：文件 I/O 中转**

当前各阶段通过 JSON → TRC → XML → .mot 文件传递数据，在线化后必须改为内存传递。

**阻塞点 2：滤波器不兼容实时**

- Butterworth (filtfilt)：零相位双向，需完整序列
- Kalman (smooth=True)：RTS smoother，需完整序列
- OneEuro（当前实现）：**双向 pass（零相位）**，需完整序列

代码验证（`filtering.py:154-156`）：
```python
# Forward and backward passes (for zero-phase filtering)
filtered_forward = apply_filter(data)
filtered_backward = apply_filter(filtered_forward[::-1])[::-1]
```

可用于实时的选项：
- Kalman (smooth=False)：单向前向，代码已注明 "if real-time"
- OneEuro 改为**仅前向 pass**

**阻塞点 3：OpenSim IK 批处理**

当前 `kinematics.py:508-509`：
```python
opensim.InverseKinematicsTool(str(ik_path_temp)).run()
```
一次性处理整段时间范围，需替换为 `InverseKinematicsSolver` 的小窗口模式。

---

## 三、推荐路线

```
┌──────────────────────────────────────────────────────┐
│                 预处理（一次性）                        │
│  1. 相机标定 → Calib.toml                             │
│  2. 相机同步（硬件同步或预标定偏移）                     │
│  3. 模型缩放 → scaled_model.osim（预缩放或 warm-up）   │
└──────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────┐
│               实时循环（滑动窗口）                      │
│                                                       │
│  多相机采集（并行）                                    │
│       ↓                                               │
│  RTMPose 2D 姿态估计（逐帧）                          │
│       ↓                                               │
│  Person Association（逐帧）                           │
│       ↓                                               │
│  Triangulation（逐帧加权 DLT）                        │
│       ↓                                               │
│  Kalman(smooth=False) 或 OneEuro(单向)                │
│       ↓                                               │
│  Marker Buffer（滑动窗口，5-15 帧）                    │
│       ↓                                               │
│  OpenSim IK（InverseKinematicsSolver + 小窗口）       │
│       ↓                                               │
│  API Visualizer 实时显示 + .mot 追加输出               │
└──────────────────────────────────────────────────────┘
```

---

## 四、实施阶段

### 阶段 0：环境验证与基准测试

**工期：1-2 天**

这不是可选步骤。整个方案的成败取决于 OpenSim Python API 在目标环境中的实际行为。

**验证清单：**

| # | 验证项 | 验收标准 |
|---|--------|---------|
| 1 | `import opensim` | 成功导入，版本 ≥ 4.4 |
| 2 | `Model.setUseVisualizer(True)` | API Visualizer 窗口弹出 |
| 3 | `ModelVisualizer.show(state)` | 可多次调用刷新画面 |
| 4 | `TimeSeriesTableVec3` 程序化构造 | 可填入 marker 数据 |
| 5 | `MarkersReference` 从 table 构造 | 构造成功 |
| 6 | `InverseKinematicsSolver` 初始化 | 无异常 |
| 7 | `assemble()` + `track()` 调用 | 能求解并返回 state |
| 8 | simple model 单窗口 IK 耗时 | 记录实测值 |

**验证脚本框架：**

```python
import opensim
import time
import numpy as np

# --- 1. 加载模型 ---
model = opensim.Model("Pose2Sim/OpenSim_Setup/Model_Pose2Sim_simple.osim")
model.setUseVisualizer(True)
state = model.initSystem()

# --- 2. 验证 Visualizer ---
viz = model.getVisualizer()
viz.show(state)
print("[OK] Visualizer 可用")

# --- 3. 构造 TimeSeriesTableVec3 ---
marker_names = ["RHip", "LHip", "RKnee", "LKnee", "RAnkle", "LAnkle"]  # 示例
table = opensim.TimeSeriesTableVec3()
table.setColumnLabels(marker_names)

n_frames = 10
fps = 30.0
for i in range(n_frames):
    row = opensim.RowVectorVec3(len(marker_names))
    for j in range(len(marker_names)):
        row[j] = opensim.Vec3(
            np.random.randn() * 0.1,
            np.random.randn() * 0.1 + 1.0,
            np.random.randn() * 0.1
        )
    table.appendRow(i / fps, row)
print("[OK] TimeSeriesTableVec3 构造成功")

# --- 4. 构造 MarkersReference ---
marker_ref = opensim.MarkersReference(table)
print("[OK] MarkersReference 构造成功")

# --- 5. 构造 IK Solver ---
coord_refs = opensim.SimTKArrayCoordinateReference()
solver = opensim.InverseKinematicsSolver(model, marker_ref, coord_refs, float("inf"))
print("[OK] InverseKinematicsSolver 构造成功")

# --- 6. 性能基准 ---
working_state = opensim.State(state)
working_state.setTime(0.0)

t0 = time.perf_counter()
solver.assemble(working_state)
t_assemble = (time.perf_counter() - t0) * 1000

track_times = []
for i in range(1, n_frames):
    working_state.setTime(i / fps)
    t0 = time.perf_counter()
    solver.track(working_state)
    track_times.append((time.perf_counter() - t0) * 1000)

print(f"[BENCHMARK] assemble: {t_assemble:.1f} ms")
print(f"[BENCHMARK] track 平均: {np.mean(track_times):.1f} ms")
print(f"[BENCHMARK] track 最大: {np.max(track_times):.1f} ms")

# --- 7. Visualizer 刷新 ---
model.realizePosition(working_state)
viz.show(working_state)
print("[OK] Visualizer 刷新成功")
```

**阶段 0 的决策出口：**

| 实测结果 | 决策 |
|---------|------|
| 全部通过，track < 20ms | 进入阶段 1-2 |
| Visualizer 不可用但 IK 可用 | 进入阶段 1-2，可视化改用 Open3D 等替代 |
| IK Solver 可用但 track > 50ms | 进入阶段 1，IK 降频到 10Hz |
| IK Solver Python 绑定不可用 | 重新评估方案，考虑 scipy 自建轻量 IK 或 C++ 绑定 |

---

### 阶段 1：实时前半段管线

**工期：3-5 天**

**目标：** 从多相机实时采集到滤波后的 3D marker 输出，全部在内存中完成。

**新增文件：**

```
Pose2Sim/realtime/
    __init__.py
    pipeline.py           # 实时管线主入口
    capture.py            # 多相机并行采集
    triangulate_frame.py  # 逐帧三角化（从 triangulation.py 提取核心逻辑）
    filter_realtime.py    # 实时滤波器（Kalman 单向 / OneEuro 单向）
    marker_buffer.py      # 滑动窗口 marker 缓冲区
```

**关键设计决策：**

| 决策项 | 选择 | 理由 |
|--------|------|------|
| 数据传递 | 内存（numpy array） | 消除文件 I/O 瓶颈 |
| 多相机采集 | threading 并行 | Pose Estimation 是 CPU/GPU bound |
| 滤波器 | Kalman(smooth=False) | 代码已有实现，注释标注 "if real-time" |
| 三角化 | 提取 `triangulation_from_best_cameras()` | 核心函数已是逐帧的 |
| 后处理（插值/裁剪） | 跳过或局部处理 | 实时场景容忍少量缺失 |

**核心接口设计：**

```python
class RealtimePipeline:
    def __init__(self, config_path, calib_file):
        """加载配置、标定参数、初始化 pose tracker"""

    def process_frame(self, frames: list[np.ndarray]) -> np.ndarray:
        """
        处理一帧多相机图像

        参数: frames - 各相机当前帧图像列表
        返回: markers_3d - shape (n_markers, 3) 的滤波后 3D 坐标
        """

class SlidingMarkerBuffer:
    def __init__(self, window_size: int, n_markers: int):
        """初始化滑动窗口"""

    def push(self, markers_3d: np.ndarray, timestamp: float):
        """压入一帧 marker 数据"""

    def ready(self) -> bool:
        """窗口是否已满"""

    def get_window(self) -> tuple[np.ndarray, np.ndarray]:
        """返回 (marker_data, timestamps)"""
```

**从现有代码提取的核心函数：**

- `triangulation.py` → `triangulation_from_best_cameras()`：逐帧三角化核心
- `common.py` → `weighted_triangulation()`, `reprojection()`：加权 DLT 和重投影
- `common.py` → `computeP()`, `retrieve_calib_params()`：标定参数加载
- `filtering.py` → `kalman_filter()`：设 `smooth=False` 使用

---

### 阶段 2：OpenSim 在线 IK + 可视化

**工期：3-5 天（取决于阶段 0 实测结果）**

**目标：** 小窗口 IK 求解 + API Visualizer 实时显示。

**新增文件：**

```
Pose2Sim/realtime/
    opensim_ik.py    # 在线 IK 求解器封装
    opensim_viz.py   # API Visualizer 封装
```

**IK 求解策略（基于三轮讨论共识）：**

```
每个滑动窗口（5-15 帧）：
  1. 从 marker_buffer 取最新窗口数据
  2. 构造 TimeSeriesTableVec3
  3. 构造 MarkersReference
  4. 构造 InverseKinematicsSolver
  5. 首帧 assemble()（用上一窗口末状态作为初值）
  6. 后续帧 track()
  7. 取最后一帧 state 用于显示和输出
  8. 保存 state 作为下一窗口初值
```

**说明：**
- 每窗口重建 solver 是当前最合理的首版原型方案
- solver 重建的开销（构造小型数据表）在工程常识上远小于 IK 求解本身（迭代非线性优化），但最终是否构成瓶颈以阶段 0 实测为准
- 如果实测发现重建开销过高，可探索 solver 复用或 reference 更新机制

**Scaling 策略：**

| 方案 | 适用场景 | 做法 |
|------|---------|------|
| 预缩放（推荐） | 被试可以提前做一次试验 | 离线运行 `perform_scaling()` 生成 .osim，在线阶段直接加载 |
| Warm-up 缩放 | 被试可以先站立几秒 | 在线启动时采 3-5 秒数据，调用 `perform_scaling()` 后锁定模型 |

**不推荐** 在在线阶段持续自动缩放——README 明确提醒自动 scaling 不适用于蹲姿/坐姿主导试验。

**必须使用 `use_simple_model = True`：**

Config.toml 明确记载：
```toml
use_simple_model = false # >10 times faster IK if true.
                         # No muscles, no constraints
```

在线阶段 simple model 是前提条件，不是可选优化。

---

### 阶段 3：多线程管线集成

**工期：2-3 天**

**目标：** 将采集、处理、IK、可视化解耦到独立线程，避免相互阻塞。

**线程架构：**

```
Thread A（采集 + 姿态估计）:
    多相机同步采集 → RTMPose → 2D keypoints
    → 写入 frame_queue

Thread B（3D 重建 + 滤波）:
    从 frame_queue 读取
    → Person Association → Triangulation → Kalman Filter
    → 写入 marker_buffer

Thread C（IK + 可视化）:
    从 marker_buffer 取窗口
    → IK Solver → Visualizer.show(state)
    → 可选：追加写入 .mot
```

**设计原则：**

- 使用 `queue.Queue` 或 `collections.deque` 做线程间通信
- 允许丢旧帧：消费者跟不上时跳到最新数据，不堆积
- IK 频率可以低于采集频率（如采集 30Hz，IK 10-15Hz）
- 以"显示最新可用状态"为目标，不以"每帧绝不遗漏"为目标

---

### 阶段 4：输出与完善

**工期：1-2 天**

**输出方式（优先级排序）：**

1. **API Visualizer 实时显示**（阶段 2 已完成）
2. **追加写入 .mot**：每个窗口将最新关节角追加到文件，事后可在 OpenSim GUI 回放
3. **可选：自定义前端**：通过 WebSocket/ZMQ 推送关节角到 Web 或 Open3D 渲染器

---

## 五、第一版明确排除的范围

| 排除项 | 理由 |
|--------|------|
| OpenSim GUI 在线集成 | 官方对 GUI graphics window 的脚本控制有限，不适合高频实时渲染 |
| Marker Augmentation | 当前是整序列 ONNX 推理，与在线 IK 同时改造会显著增加风险 |
| C++ 重写 | 第一阶段完全不需要；如 Python API 实测不满足要求，可作为低优先级兜底选项 |
| 复杂肌肉骨骼模型 | 必须使用 simple model（>10x 加速），复杂模型留待后续优化 |
| 过早性能优化 | 先跑通，再优化 |

---

## 六、风险评估

| 风险 | 性质 | 应对措施 |
|------|------|---------|
| OpenSim IK Solver Python 绑定行为不符预期 | **当前最关键的不确定点**，可通过阶段 0 快速收敛 | 阶段 0 验证脚本全面覆盖；fallback: scipy 自建轻量 IK |
| IK 单帧耗时超出预算 | 集中且可验证 | simple model + 降频到 10-15Hz + 阶段 0 实测 |
| 每窗口 solver 重建开销 | 工程常识上不应成为瓶颈，但需实测确认 | 阶段 0/2 记录重建耗时；必要时探索 solver 复用 |
| API Visualizer 刷新不流畅 | 概率低 | 降低显示频率；或用 Open3D/PyQtGraph 替代 |
| 多相机 Pose Estimation 串行延迟 | 相机数量线性增长 | 多线程并行；降低 det_frequency |
| 硬件同步不到位 | 取决于设备配置 | 必须硬件同步或高精度时间戳对齐 |
| Kalman 单向滤波精度不如双向 | 确定会发生，幅度有限 | 可接受的精度损失；增大 trust_ratio 可缓解 |
| OneEuro 单向 pass 效果降低 | 确定会发生，幅度有限 | 调整 beta 和 min_cutoff 参数补偿 |

---

## 七、性能预期

> **重要声明：** 以下数字基于公开文献和工程估算，不是本项目目标环境的实测值。最终性能以阶段 0 基准测试为准。

**在 simple model 条件下，在线显示 OpenSim 结果有望达到 10-30Hz 的刷新频率。** 具体取决于：

- 硬件（GPU、CPU）
- 相机数量
- marker 数量
- 窗口大小
- OpenSim IK 在目标环境的实际耗时

多线程管线化后，系统吞吐量取决于最慢的一个阶段，而非各阶段耗时之和。

---

## 八、需要改造的现有代码

| 现有文件 | 改造方式 | 说明 |
|---------|---------|------|
| `triangulation.py` | 提取 `triangulate_frame()` | 从主循环中分离单帧三角化逻辑，去除文件 I/O |
| `filtering.py` | 新增单向 OneEuro | 当前实现是零相位双向，需新增仅前向 pass 的版本 |
| `filtering.py` | Kalman smooth=False 路径 | 已有实现，确保可独立调用 |
| `kinematics.py` | 提取可复用函数 | `load_scaled_model()`, `extract_joint_angles()`, `build_marker_weights()` |
| `common.py` | 直接复用 | `weighted_triangulation()`, `reprojection()`, `computeP()` 等 |
| `poseEstimation.py` | 参考其 PoseTracker 初始化 | 实时管线中直接使用 rtmlib |

**原则：不修改现有离线管线的行为，在线功能作为独立模块新增。**

---

## 九、实施优先级总览

```
阶段 0  ──→  通过  ──→  阶段 1  ──→  阶段 2  ──→  阶段 3  ──→  阶段 4
环境验证      │         前半段管线     IK+可视化     多线程集成     输出完善
1-2天        │         3-5天         3-5天         2-3天         1-2天
             │
             └→  不通过  ──→  重新评估方案
                              - scipy 轻量 IK
                              - C++ 绑定（低优先级兜底）
                              - 降级为 3D skeleton 可视化
```

**总工期预估：10-17 天**（不含调试迭代和硬件联调）

---

## 十、三轮讨论的完整共识记录

以下为三轮技术讨论（Claude 原始分析 → GPT-5.4 第一轮审阅 → Claude 修正 → GPT-5.4 第二轮审阅）后达成的共识：

| 议题 | 最终共识 |
|------|---------|
| 整体可行性 | 可行，但不是小改动 |
| 主路线 | 实时前半段 + 小窗口 IK + API Visualizer |
| 可视化方案 | API Visualizer 优先，非 OpenSim GUI |
| Scaling | 预缩放优先，warm-up 缩放备选 |
| 模型选择 | 必须 simple model |
| 滤波器 | Kalman(smooth=False) 或 OneEuro(改为单向) |
| Marker Augmentation | 第一版跳过 |
| 阶段 0 | 必须先做，不可跳过 |
| 文件 I/O | 必须改为内存传递 |
| IK 方案 | 小窗口 reference + solver + state 延续 |
| C++ | 当前不做，作为低优先级兜底选项保留 |
| 性能数字 | 基于文献估算，以阶段 0 实测为准 |
| IK 风险 | 集中在 API 行为，可通过阶段 0 快速收敛 |
