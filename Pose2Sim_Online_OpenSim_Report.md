# Pose2Sim 在线显示 OpenSim 结果方案报告

## 1. 目标重述

这份报告只回答一个问题：

**如何把 Pose2Sim 从“离线生成 `.osim` / `.mot` 后再看结果”，改成“在线显示 OpenSim 结果”。**

这里的“OpenSim 结果”不是普通 3D 关键点骨架，而是至少包含下面两类之一：

1. OpenSim 模型姿态实时更新；
2. OpenSim 关节角结果实时输出并驱动可视化。

## 2. 结论先说

### 2.1 可以做，但不能把它理解成一个小改动

基于仓库现状和 OpenSim 官方文档，我的判断是：

- **在线显示 OpenSim 结果是可行的。**
- **但当前 `Pose2Sim/kinematics.py` 不是改几行就能变成在线版。**
- **更现实的实现目标是：先做 OpenSim API Visualizer 在线显示，而不是强行把 OpenSim GUI 变成实时播放器。**

### 2.2 推荐路线

我推荐的主路线是：

1. 保留 Pose2Sim 前半段的 2D/3D 重建能力；
2. 新增一条实时 OpenSim 运行链路；
3. 使用 **OpenSim API Visualizer** 在 Python/OpenSim 进程里实时显示模型；
4. 用滑动窗口或小缓冲区驱动 IK；
5. 先用 `simple model`，先放弃 marker augmentation，先不用 OpenSim GUI 做在线显示。

### 2.3 不推荐路线

我**不推荐**第一阶段就做：

- “外部 Python 持续驱动 OpenSim GUI 实时刷新”
- “继续沿用 `InverseKinematicsTool + TRC/MOT` 文件流水线，只是更快写文件”
- “先追求多模块全实时，再考虑 OpenSim 显示”

这几条路线不是完全不可能，而是**工程代价大、稳定性差、收益不高**。

## 3. 和你最早那份 md 文档对比

我这里说的“原文档”，指 [Pose2Sim_Realtime_Analysis.md](/Users/danny/Downloads/Pose2Sim_Realtime_Analysis.md)。

## 3.1 原文档里保留成立的部分

原文档这些判断仍然成立：

1. 当前管线整体是离线批处理。
2. Pose Estimation、Person Association、Triangulation 的核心都有逐帧化基础。
3. OpenSim IK 是整条链路里最关键的难点。
4. 如果做在线化，必须把文件中转改成内存中转。
5. “先缩放，再做逐帧/小窗口 IK”这个方向是对的。

## 3.2 原文档需要修正的部分

### 修正 1：原文档把 `InverseKinematicsSolver` 的可用性写得太轻松

原文档的核心假设是：

> 只要把 `InverseKinematicsTool` 换成 `InverseKinematicsSolver`，就可以逐帧更新 marker 并得到关节角。

这个说法**方向上没错，但过于乐观**。

OpenSim 官方文档确实说明：

- `assemble()` 用于第一次求解；
- `track()` 适合在状态变化较小时高效更新；

但官方类文档公开暴露的更新接口主要是：

- 更新 marker 权重；
- 更新 coordinate reference；
- 设置时间是否由 reference 推进；

我**没有在官方公开类接口里看到一个明确的、专门用于“直接向已有 solver 注入新一帧 marker 观测值”的 public 方法**。  
这是我根据官方文档做出的推断，不是 100% 否定，而是说明：

- **流式 marker 喂给 IK，并不像原文档写的那样已经是现成现成的 Python 调用套路。**

### 修正 2：原文档默认“可以把结果推到 OpenSim GUI”这点缺少证据

原文档里把“Socket/共享内存推给 OpenSim GUI”当成一种输出方案。  
从我查到的 OpenSim 官方资料看，这条路没有现成官方工作流支持。

官方文档更明确支持的是：

- **API Visualizer**
- **GUI 内置脚本**

而不是“外部程序像游戏引擎一样实时遥控 GUI 可视化”。

因此如果目标是“在线显示 OpenSim 结果”，更可行的官方路径其实是：

- **API Visualizer**

而不是：

- **OpenSim GUI 外部实时遥控**

### 修正 3：原文档对 IK 耗时估算过于乐观

原文档给了单帧 IK 大致耗时预算，并据此推导 1 秒窗口可在 1 秒内处理完。  
这部分在仓库里**没有实测依据**，在当前本机环境也**无法验证**，因为当前环境没有安装 `opensim` 包。

所以实事求是地说：

- 这部分耗时估算只能当“可能范围”，不能当实施承诺；
- 你必须在目标机器上做基准测试后，才知道是不是能稳定在线显示。

### 修正 4：原文档没有区分“API Visualizer 在线显示”和“OpenSim GUI 在线显示”

这其实是整个问题里最应该先分清的点。

从实施难度看：

- **API Visualizer 在线显示**：可行性高，推荐。
- **OpenSim GUI 在线显示**：可行性低得多，不推荐作为第一阶段主线。

## 3.3 原文档里没有充分展开、但现在必须补上的点

### 补充点 1：自动缩放不适合所有在线场景

README 明确写了：

- 自动 scaling 不推荐用于“主要处于蹲姿或坐姿”的试验；

这意味着在线方案不能简单假设：

- “开机先采几秒数据自动缩放，然后永久在线跑”。

如果你的被试不是稳定站立、而是直接进入动作，自动 scaling 的可靠性会下降。

### 补充点 2：OpenSim 依赖不是这个仓库默认随装的

`pyproject.toml` 没有把 `opensim` 放进标准依赖里，仓库 README 也是要求单独安装 OpenSim Python API。  
这意味着：

1. 你的在线 OpenSim 模块必须单独考虑环境管理；
2. 不能把“仓库能跑”和“OpenSim 在线链路能跑”混为一谈。

## 4. 仓库现状：为什么它现在不适合直接在线显示 OpenSim 结果

## 4.1 当前 `kinematics.py` 完全是离线文件驱动

当前实现路径是：

1. 从 `.trc` 读取整段 3D marker 数据；
2. `perform_scaling(...)` 用 `ScaleTool` 做缩放；
3. `perform_IK(...)` 生成临时 IK setup xml；
4. `InverseKinematicsTool(...).run()`；
5. 输出 `.mot` 文件。

也就是说，当前链路不是“状态流”，而是：

```text
TRC file -> ScaleTool -> scaled model -> InverseKinematicsTool -> MOT file
```

这对于离线批处理没问题，但对在线显示 OpenSim 结果不适合。

## 4.2 当前 `perform_scaling()` 依赖整段 TRC，而不是启动时的一次轻量初始化

`perform_scaling()` 会：

- 读取整段 TRC；
- 删除快帧、低速帧、髋膝角度过大帧；
- 统计剩余帧上的 segment ratio；
- 再写缩放配置并运行 `ScaleTool`。

这说明当前 scaling 逻辑是：

- **序列统计型**

不是：

- **开机即刻型**
- **逐帧可更新型**

所以在线方案里，scaling 必须被单独重新定义。

## 4.3 当前 `perform_IK()` 绑定的是完整 TRC + Tool 流程

`perform_IK()` 里做的事非常明确：

1. 读 TRC 时间范围；
2. 改写 IK setup XML；
3. 调 `opensim.InverseKinematicsTool(...).run()`。

它不是求“当前帧 state”，而是求“这一整段 motion”。

所以如果你要在线显示 OpenSim 结果，**这里必须绕开 Tool 路线**。

## 4.4 当前前处理链路也还是文件流水线

在线 OpenSim 结果显示不仅卡在 IK，也卡在上游的数据通道：

- `poseEstimation.py` 每帧落 JSON；
- `personAssociation.py` 再读 JSON / 写 JSON；
- `triangulation.py` 再读 JSON，最后写 `.trc`；
- `kinematics.py` 再读 `.trc`。

如果不改这个结构，就算 IK 变快了，在线显示也会被中间 I/O 拉垮。

## 5. OpenSim 官方文档给出的关键信息

下面这些点来自 OpenSim 官方文档，而不是我主观猜测。

## 5.1 `InverseKinematicsSolver` 适合在初值接近时做高效更新

官方文档明确说明：

- 在 `assemble()` 之后，如果模型、目标数量不变，并且状态接近解，`track()` 是高效更新 configuration 的方法。

这说明：

- **在线 IK 不是没希望；**
- 但前提是你要维护稳定的状态延续，而不是每次都从头算。

## 5.2 `MarkersReference` 的标准构造方式是文件或 `TimeSeriesTableVec3`

官方类文档显示，`MarkersReference` 的主要构造方式是：

1. 从 marker 文件构造；
2. 从 `TimeSeriesTableVec3` 构造。

这很关键，因为它说明：

- 你至少可以程序化构造一个窗口大小的 marker table；
- 但它也说明当前公开接口更偏“表格 reference”而不是“逐帧 push 新观测”。

## 5.3 公开更新接口没有显示“直接更新 marker 观测值”

官方 `InverseKinematicsSolver` 文档清楚列出了：

- `updateMarkerWeight()`
- `updateMarkerWeights()`
- `updateCoordinateReference()`
- `setAdvanceTimeFromReference()`

但没有一个公开方法明确写成：

- `updateMarkerObservation(...)`
- `setMarkerValuesForCurrentFrame(...)`

因此我对这一点的实事求是判断是：

- **基于 Python API 做真正流式 marker 注入，可能需要比原文档设想更多的工程工作。**

### 这不是说做不到

而是说更可能出现下面三种实现方式：

1. **小窗口重建 `MarkersReference + IKSolver` 原型**
2. **重用 solver，但用时间 reference 驱动并维护小窗口表**
3. **如 Python 性能或接口不足，转 C++ 做自定义 streaming reference**

## 5.4 API 级别的实时可视化是官方支持的

OpenSim 官方文档明确支持：

- `Model.setUseVisualizer(true)`
- 然后通过 `ModelVisualizer.show(state)` 生成每一帧显示。

这说明如果你要“在线显示 OpenSim 结果”，官方最直接的可视化通路不是 GUI，而是：

- **ModelVisualizer / Simbody Visualizer**

## 5.5 GUI 脚本能力有限，不等于适合外部实时流控制

官方关于 GUI scripting 的文档说明：

- GUI 内置脚本可以访问很多 GUI 命令；
- 但对 graphics window 的访问是 **limited access**；
- 主要举例是 selection 和 camera control。

这意味着：

- GUI scripting 更适合半交互式自动化；
- 不适合你把它当成一个高频实时渲染后端。

## 5.6 Python/Matlab 外部脚本推荐使用 model/API visualizer

官方旧版 Python scripting 文档摘要里明确写了：

- 外部 Python 模式下不能直接访问 OpenSim plotter 或 graphics window；
- 建议用 model/API visualizer。

这进一步支持结论：

- **你如果要做真正的在线显示，首选 API Visualizer，不是 OpenSim GUI。**

## 6. 围绕“在线显示 OpenSim 结果”的三条路线

## 6.1 路线 A：OpenSim API Visualizer 在线显示

### 可行性

- **高**

### 说明

流程是：

1. 外部 Python/OpenSim 进程加载缩放后的 `.osim`；
2. 开启 `setUseVisualizer(true)`；
3. 实时求出每一帧 `state`；
4. 调 `model.getVisualizer().show(state)`；
5. 同时输出 joint angles / marker error / 日志。

### 优点

1. 这是官方支持的 API 路线；
2. 不依赖 OpenSim GUI 的脚本桥接；
3. 更适合和 Pose2Sim 的实时管线直接集成；
4. 代码可控性最高。

### 缺点

1. 仍然要解决在线 IK；
2. 可视化效果和 GUI 完整功能不完全等价；
3. 需要你自己处理 runtime 循环与状态同步。

### 我的判断

- **这是推荐主路线。**

## 6.2 路线 B：OpenSim GUI 在线显示

### 可行性

- **中低**

### 说明

大概会走下面几类办法：

1. GUI 内置脚本轮询某个 growing file；
2. GUI 脚本接收外部 IPC/Socket；
3. 外部进程持续改动数据文件，GUI 周期性重载 motion。

### 问题

1. 官方没有把它作为典型实时方案来支持；
2. GUI scripting 对 graphics window 的控制有限；
3. 文件轮询会引入额外延迟和稳定性问题；
4. 整体工程复杂度高，而且调试困难。

### 我的判断

- **如果你坚持“必须在 OpenSim GUI 里实时看”，可以做探索原型；**
- **但不应该作为第一阶段主线。**

## 6.3 路线 C：OpenSim 求解，外部自定义前端显示

### 可行性

- **高**

### 说明

这条路线不一定用 OpenSim 自带窗口，而是：

1. OpenSim 负责求 joint angles / model state；
2. 你把关节状态发给自定义 viewer（桌面或 Web）；
3. viewer 只负责显示 OpenSim 结果。

### 优点

1. UI 自由度高；
2. 更容易做“在线系统”；
3. 更容易挂到网页或大屏。

### 缺点

1. 严格来说不是“在 OpenSim 自身窗口里显示”；
2. 需要你做模型映射和显示层。

### 我的判断

- 如果你真正要的是“OpenSim 结果在线显示”，而不是“必须 OpenSim GUI 本体”，这条路线长期更强。

## 7. 推荐实施方案

我的推荐方案不是 GUI 方案，而是：

## 推荐方案：`Pose2Sim 实时前半段 + OpenSim API Visualizer 在线后半段`

总体结构：

```text
多相机输入
  -> RTMPose / 跟踪
  -> Person Association
  -> Triangulation
  -> 实时滤波
  -> Marker Buffer (滑动窗口)
  -> OpenSim Realtime IK
  -> ModelVisualizer.show(state)
  -> 可选输出 joint angles / logs / recorder
```

## 8. 具体实施步骤

## 步骤 0：先做环境与基准验证

这是必须先做的，不要跳。

### 要做什么

1. 在目标运行环境安装 OpenSim Python API；
2. 验证：
   - `import opensim`
   - `Model.setUseVisualizer(true)`
   - `ModelVisualizer.show(state)`
   - `InverseKinematicsSolver` 可导入
   - `MarkersReference(TimeSeriesTableVec3, ...)` 可构造
3. 做最小 benchmark：
   - simple model
   - 不带可视化
   - 单窗口 IK
   - 带可视化 show(state)

### 为什么必须先做

因为当前这个 workspace 里没有 `opensim` 包，我不能替你在本机做 API 级别验证。  
所以第一步必须先把“理论可行”变成“目标机器上可运行”。

### 验收标准

至少先回答这三个问题：

1. 目标机器上 OpenSim Python API 能不能稳定导入？
2. API Visualizer 能不能弹出并刷新？
3. 你用 `simple model` 时，单次小窗口 IK 大概多少毫秒？

## 步骤 1：把在线 OpenSim 方案从现有离线 `kinematics.py` 中分离出来

### 原因

当前 `kinematics.py` 明确是离线工具，不适合直接塞在线逻辑。

### 建议

新增一个单独模块，例如：

```text
Pose2Sim/realtime_opensim.py
```

或者：

```text
Pose2Sim/realtime/
    opensim_runtime.py
    ik_window.py
    visualizer.py
```

### 目的

让离线 `Pose2Sim.kinematics()` 保持不变，在线方案作为新入口实现。

## 步骤 2：先解决 scaling，不要让在线链路每次都重新缩放

### 推荐策略

第一版不要做“在线自动缩放”。

### 更稳妥的做法

用下面二选一：

1. **预先缩放**
   - 用一段站立 / A-pose / 高质量 trial 先离线生成个体化 `.osim`
   - 在线阶段直接加载这个 scaled model

2. **启动时 warm-up 缩放**
   - 只在被试先站稳的前提下
   - 采 3-5 秒
   - 成功后锁定模型，不再重缩放

### 为什么我更推荐预先缩放

因为 README 已经明确提醒：

- 自动 scaling 不推荐用于蹲姿 / 坐姿主导试验。

所以如果你的实验不是固定先站立，在线自动缩放会不稳。

## 步骤 3：把实时 OpenSim 的输入改成“滑动窗口 marker table”

### 原因

基于官方文档，`MarkersReference` 的标准程序化入口是：

- `TimeSeriesTableVec3`

因此在线原型最现实的做法不是“逐帧直接塞 marker 到 solver”，而是：

- **维护一个长度较小的 marker 滑动窗口**

例如：

- 5 帧
- 10 帧
- 15 帧

### 做法

每次取最近 `W` 帧 3D marker，构造成：

- `TimeSeriesTableVec3`
- `MarkersReference`

再驱动 IK。

### 这一步和原文档最大的差别

原文档默认“逐帧更新 marker”是直接存在的调用套路；  
我这里的建议更保守，也更贴近官方公开接口：

- **先用小窗口 TimeSeriesTableVec3 原型验证。**

## 步骤 4：用 `InverseKinematicsSolver` 做在线 IK 原型

### 第一版建议

先不追求“单 solver 永久复用”，而是先做能跑通的原型：

1. 加载 scaled model；
2. 初始化 state；
3. 当前窗口生成 `MarkersReference`；
4. 构造 `InverseKinematicsSolver`；
5. 用上一个窗口末状态作为初值；
6. 窗口首帧 `assemble()`；
7. 窗口后续帧 `track()`；
8. 输出窗口最后一帧 state 作为在线显示结果。

### 为什么先这样做

因为这是最符合当前官方接口暴露方式的方案。  
它不一定最优，但它是最容易验证对错的原型路线。

### 第二版优化

如果第一版性能不够，再做：

1. solver 对象复用；
2. reference 更新时间机制优化；
3. 必要时转 C++ 做 streaming reference。

## 步骤 5：先用 `use_simple_model = true`

这点不是“建议”，而是我认为在线阶段的默认前提。

README 明确说：

- `use_simple_model = true` 会让 IK 至少快 10 倍。

如果你第一版还坚持带 muscles / constraints 的复杂模型，极可能直接把在线性拖死。

### 第一版目标

- 先让 online OpenSim result display 跑起来；
- 不是先让 full musculoskeletal fidelity 跑起来。

## 步骤 6：把 OpenSim 显示和 IK 求解线程分开

### 建议结构

```text
Thread A: Pose2Sim 前半段 -> 3D markers
Thread B: Marker buffer -> OpenSim IK
Thread C: Visualizer.show(state)
```

### 原因

`show(state)` 不应该反向阻塞上游三角化与滤波。  
可视化只消费最新可用 state。

### 实施要点

1. 用 queue / ring buffer；
2. 允许丢旧帧，不要堆积；
3. 以“显示最新状态”为目标，不以“每帧绝不遗漏”为目标。

## 步骤 7：第一版先不要接 marker augmentation

### 原因

当前 `markerAugmentation.py` 是整段序列一次性 ONNX 推理，不是现成在线模块。  
如果你把它一起接进第一版，会同时把问题变成：

1. 在线 IK
2. 在线 augmentation

这会显著增加风险。

### 第一版建议

- 先直接用 Triangulation + Realtime Filter 的 markers 进 IK。

## 步骤 8：第一版先不要强求 OpenSim GUI

### 推荐显示路径

```python
model = opensim.Model("scaled_model.osim")
model.setUseVisualizer(True)
state = model.initSystem()
...
model.getVisualizer().show(state)
```

### 如果你后面非要 GUI

建议把 GUI 集成放到第二阶段，并单独立项验证。

## 9. 代码层面的具体改造建议

## 9.1 新增文件

建议新增：

```text
Pose2Sim/realtime/
    __init__.py
    packets.py
    marker_buffer.py
    opensim_runtime.py
    opensim_visualizer.py
    pipeline.py
```

### 各文件职责

- `packets.py`
  - 定义 `Pose3DPacket`
  - 定义 `OpenSimStatePacket`

- `marker_buffer.py`
  - 维护滑动窗口
  - 输出 `TimeSeriesTableVec3` 所需数据

- `opensim_runtime.py`
  - 加载 scaled model
  - 构造 `MarkersReference`
  - 构造 `InverseKinematicsSolver`
  - 求解最新窗口的 IK
  - 抽取 joint angles

- `opensim_visualizer.py`
  - 封装 `ModelVisualizer.show(state)`

- `pipeline.py`
  - 串接 Pose2Sim 前段与 OpenSim 后段

## 9.2 现有文件建议怎么改

### `Pose2Sim/kinematics.py`

不要直接把在线逻辑塞进 `perform_IK()`。

更好的做法是抽出下面这些可复用函数：

1. `load_scaled_model(...)`
2. `prepare_model_visualizer(...)`
3. `extract_joint_angles_from_state(...)`
4. `build_marker_weights(...)`

保留离线：

- `perform_scaling(...)`
- `perform_IK(...)`

新增在线：

- `run_realtime_ik_window(...)`

### `Pose2Sim/triangulation.py`

新增逐帧接口：

- `triangulate_frame(...)`

而不是让在线流程继续依赖整段 `Q_tot` 后处理。

### `Pose2Sim/filtering.py`

在线阶段只保留：

1. `kalman(smooth=False)`
2. 或新增真正单向 `OneEuro`

不要直接用当前 zero-phase OneEuro。

## 10. 第一版在线 OpenSim 原型的建议伪代码

下面这个伪代码不是最终 API 保证，只是推荐的原型方向：

```python
scaled_model = opensim.Model("subject_scaled.osim")
scaled_model.setUseVisualizer(True)
state = scaled_model.initSystem()

last_state = opensim.State(state)
marker_buffer = SlidingMarkerBuffer(size=10)

while True:
    pose3d_packet = get_latest_pose3d_packet()
    marker_buffer.push(pose3d_packet)

    if not marker_buffer.ready():
        continue

    marker_table = build_timeseries_table_vec3(marker_buffer)
    marker_ref = opensim.MarkersReference(marker_table, marker_weights)
    coord_refs = build_empty_coordinate_refs()

    solver = opensim.InverseKinematicsSolver(
        scaled_model,
        marker_ref,
        coord_refs,
        float("inf")
    )
    solver.setAdvanceTimeFromReference(False)

    working_state = opensim.State(last_state)

    for i, t in enumerate(marker_buffer.times()):
        working_state.setTime(t)
        if i == 0:
            solver.assemble(working_state)
        else:
            solver.track(working_state)

    last_state = opensim.State(working_state)
    scaled_model.realizePosition(last_state)
    scaled_model.getVisualizer().show(last_state)

    joint_angles = extract_joint_angles(last_state, scaled_model)
    publish_joint_angles(joint_angles)
```

### 说明

这个原型的关键思想是：

- **不是每帧直接操作离线 Tool；**
- **而是小窗口 reference + solver + state 延续。**

## 11. 风险清单

## 风险 1：Python 层在线 IK 性能不够

### 严重程度

- 高

### 应对

1. 先用 simple model；
2. 先把 IK 频率降到 10-15 Hz；
3. 三角化仍可 30 Hz；
4. 显示层做插值或只显示最新状态；
5. 如仍不够，再考虑 C++。

## 风险 2：在线 scaling 失败

### 严重程度

- 高

### 应对

1. 第一版直接使用预缩放模型；
2. 只在站立 warm-up 明确存在时才启用自动缩放。

## 风险 3：官方接口不能优雅支持 marker streaming

### 严重程度

- 高

### 应对

1. 先做窗口重构原型；
2. 如性能不足，转 C++ 做 streaming reference；
3. 或退一步做“准实时小窗口”而非严格逐帧实时。

## 风险 4：如果执意使用 OpenSim GUI，本身就会增加大量非核心复杂度

### 严重程度

- 中高

### 应对

1. 把 GUI 集成放到第二阶段；
2. 第一阶段以 API Visualizer 为准。

## 风险 5：当前本机环境无法直接验证 OpenSim 绑定

### 严重程度

- 中

### 应对

1. 先在目标环境做 API 验证；
2. 再开始正式代码改造。

## 12. 推荐的里程碑

## M1：OpenSim 在线显示最小闭环

目标：

1. 已有 scaled model；
2. 假数据或录制数据驱动 `ModelVisualizer.show(state)`；
3. 可以稳定刷新模型姿态。

## M2：在线 IK 原型

目标：

1. 实时 3D markers 进窗口 buffer；
2. `InverseKinematicsSolver` 输出当前状态；
3. 可以在线显示 OpenSim 模型姿态。

## M3：性能优化

目标：

1. 切到 simple model；
2. 减少窗口长度；
3. 分离显示线程；
4. 控制延迟和抖动。

## M4：可选 GUI 集成

目标：

1. 只有在 API Visualizer 路线稳定以后，才探索 GUI 方案；
2. 这一步应视作附加项，不应阻塞主线。

## 13. 最终建议

### 13.1 能不能基于你最早那份 md 文档继续改

能，但必须修改它的重点。

原文档适合回答：

- “整个管线能否实时化”

但你现在要回答的是：

- “如何在线显示 OpenSim 结果”

这就要求把重点从：

- Triangulation / Filtering 的一般实时化

转到：

- OpenSim 在线显示目标到底选哪条路；
- OpenSim IK 如何在线化；
- GUI 和 API Visualizer 的区别；
- scaling 如何在在线系统里落地。

### 13.2 我给你的最实事求是建议

如果你的核心目标就是“在线显示 OpenSim 结果”，那么第一阶段应该这样定：

1. **预先或启动时完成缩放；**
2. **用小窗口 marker table 驱动 `InverseKinematicsSolver` 原型；**
3. **用 OpenSim API Visualizer 实时显示 state；**
4. **先别碰 OpenSim GUI 在线集成。**

### 13.3 一句话结论

**在线显示 OpenSim 结果是可以做的，但推荐做成“OpenSim API Visualizer 在线显示系统”，而不是“把现有离线 Tool 文件流 + OpenSim GUI 硬改成实时播放器”。**

## 14. 说明

这份报告基于三类证据：

1. 你给的原分析文档；
2. 当前仓库代码；
3. OpenSim 官方文档。

同时需要明确：

- 当前本地 workspace 没有安装 `opensim` Python 包；
- 因此我这次给的是**带证据的工程判断和实施方案**；
- 但还没有在这台环境里完成 OpenSim API 的实机调用验证。

如果你下一步要我继续，我建议直接进入下一阶段：  
我可以再给你一份 **“代码级实施清单”**，把每个新模块、每个关键函数签名、每一步要改哪些文件写成开发任务表。
