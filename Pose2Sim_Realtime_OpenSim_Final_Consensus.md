# Pose2Sim 在线显示 OpenSim 结果最终定稿

## 1. 文档目的

本文件是三轮讨论后的最终收敛版本，目标是回答一个明确问题：

**如何把 Pose2Sim 从离线生成 `.osim` / `.mot` 后再查看结果，改造成能够在线显示 OpenSim 结果。**

这里的“OpenSim 结果”指：

1. OpenSim 模型姿态实时更新；
2. 或 OpenSim 关节角实时输出并驱动显示。

本定稿只保留已经形成共识的结论；  
凡是尚未在目标环境中实测验证的内容，一律写成“工程判断”或“待验证项”，不再写成确定结论。

---

## 2. 最终结论

### 2.1 总体判断

**该目标可行。**

但需要明确：

- 这不是对现有 `kinematics.py` 的小修小补；
- 这是一条新的实时链路；
- 当前最大的工程改造点仍然是 **OpenSim 在线 IK + 流式数据通道**。

### 2.2 最终推荐路线

最终推荐的技术路线为：

1. Pose2Sim 前半段改为实时内存流；
2. 使用预缩放模型，或在可控条件下 warm-up 缩放；
3. 维护小窗口 3D marker buffer；
4. 用 `MarkersReference + InverseKinematicsSolver` 做在线 IK 原型；
5. 使用 **OpenSim API Visualizer** 实时显示 `state`；
6. 第一版不以 OpenSim GUI 为目标。

### 2.3 不推荐作为第一阶段主线的路线

第一阶段不推荐：

1. 强行把 `OpenSim GUI` 做成实时显示后端；
2. 继续沿用 `TRC -> IK Tool -> MOT` 的离线文件流水线；
3. 把 `marker augmentation` 一起并入第一版；
4. 一开始就转 C++ 重写。

---

## 3. 已达成的核心共识

以下内容视为当前最终共识。

## 3.1 在线显示应优先使用 API Visualizer，而不是 OpenSim GUI

这是当前最明确的结论之一。

原因：

1. OpenSim 官方支持程序化可视化的路径是 API visualizer；
2. OpenSim GUI scripting 对 graphics window 的实时控制能力有限；
3. 如果目标是稳定的在线显示，`Model.setUseVisualizer(true)` + `show(state)` 是更合理的实现方式。

**结论：第一版在线显示以 API Visualizer 为准。**

## 3.2 第一版优先使用预缩放模型

当前 `perform_scaling()` 明显是面向离线统计型流程设计的，不适合默认直接搬到在线模式。

因此第一版推荐：

1. **首选：预缩放模型**
2. **备选：warm-up 站立缩放**

并且需要保留 README 中的约束：

- 如果试验主要是蹲姿 / 坐姿，自动 scaling 的稳定性会下降。

**结论：第一版不要假设“任意动作场景下都能在线自动缩放”。**

## 3.3 第一版必须先做阶段 0：环境验证与基准测试

这一点没有争议，必须保留。

阶段 0 要验证的不是“理论上是否可行”，而是：

1. 目标环境能否稳定导入 `opensim`；
2. `InverseKinematicsSolver` 是否可在 Python 中按预期使用；
3. `MarkersReference` / `TimeSeriesTableVec3` 是否能按计划构造；
4. `ModelVisualizer.show(state)` 是否能稳定刷新；
5. `simple model` 下的小窗口 IK 性能是否满足在线要求。

**结论：阶段 0 是必选步骤，不是附加步骤。**

## 3.4 第一版跳过 marker augmentation

当前 `markerAugmentation.py` 是整段序列一次性 ONNX 推理，不是现成在线模块。

因此第一版应明确排除：

- 在线 marker augmentation

**结论：第一版在线 OpenSim 结果显示，不包含 augmentation。**

## 3.5 当前仓库里的 OneEuro 不能直接用于实时

这也是已经确认的事实。

当前实现是双向零相位版本，不是单向在线版本。  
因此实时管线里：

1. 要么先用 `Kalman(smooth=False)`；
2. 要么重写成真正单向 `OneEuro`。

**结论：当前 OneEuro 实现不能直接视为“现成实时滤波器”。**

## 3.6 第一版不以 C++ 为主线

这点也已经收敛：

- 第一阶段完全不应该以 C++ 为实施主线。

但为了工程文档严谨性，也不再写成“绝对不需要 C++”。  
更准确的说法是：

- **当前阶段不考虑 C++；如 Python API 实测不满足要求，可保留为低优先级兜底选项。**

---

## 4. 当前仍属“工程判断”、尚未实测确认的部分

以下内容可以作为设计假设，但不能写成确定结论。

## 4.1 IK 的具体毫秒级耗时

当前不能把下列数值写成最终结论：

- 单帧 IK 若干毫秒
- 整体窗口总耗时若干毫秒

原因：

1. 这些值还不是目标环境实测结果；
2. 它们会受到模型复杂度、marker 数量、窗口大小、硬件和 Python/OpenSim 绑定行为影响。

**最终写法应为：**

- `simple model` 下，IK 有希望满足在线显示频率要求；
- 具体性能需由阶段 0 基准测试确认。

## 4.2 solver 每窗口重建的成本

当前较合理的工程判断是：

- solver 重建大概率不是主瓶颈；

但这仍然属于工程判断，不应在最终文档中写成确定结论。

**最终写法应为：**

- 小窗口重建 solver 是首版原型可接受的方案；
- 是否构成瓶颈，需要在原型阶段实测确认。

## 4.3 Python API 是否足以覆盖最终版本全部需求

当前判断是：

- 第一阶段完全值得直接在 Python/OpenSim API 上做；
- 但如果后续对延迟、刷新频率、扩展控制提出更高要求，再评估更底层方案。

**最终写法应为：**

- Python/OpenSim API 是当前主线；
- 是否需要更底层方案，不在第一阶段决策。

---

## 5. 最终实施方案

## 阶段 0：环境验证与基准测试

### 目标

确认 OpenSim Python API 在目标机器上能稳定支持在线原型。

### 需要验证的事项

1. `import opensim` 成功；
2. `Model.setUseVisualizer(true)` 可用；
3. `model.getVisualizer().show(state)` 能正常显示并刷新；
4. `InverseKinematicsSolver` 可构造；
5. `MarkersReference` / `TimeSeriesTableVec3` 可按预期构造；
6. `simple model` 条件下，窗口式 IK 频率是否满足在线目标。

### 阶段 0 输出

输出一份最小验证结果，至少回答：

1. OpenSim API 是否可用；
2. API Visualizer 是否稳定；
3. `simple model` 下 IK 的大致可用频率是多少；
4. 小窗口参考构造是否可行。

---

## 阶段 1：实时前半段管线

### 目标

将以下部分改造成内存流式处理：

1. 多相机采集
2. Pose Estimation
3. Person Association
4. Triangulation
5. Realtime Filtering

### 关键原则

1. 中间数据不再依赖 JSON / TRC 文件做主通道；
2. 各模块改为直接传递内存对象；
3. 实时版本和原离线版本分开，不直接破坏原有离线入口。

### 推荐结果

这一阶段完成后，应能持续输出：

- 滤波后的 3D marker 数据流

---

## 阶段 2：在线 OpenSim IK 原型

### 目标

基于小窗口 marker buffer，完成 OpenSim 在线 IK 原型。

### 推荐方法

1. 维护长度较小的 3D marker 窗口；
2. 用窗口数据构造 `TimeSeriesTableVec3`；
3. 基于它构造 `MarkersReference`；
4. 构造 `InverseKinematicsSolver` 原型；
5. 使用：
   - 首帧 `assemble()`
   - 后续帧 `track()`
6. 用窗口末状态作为下一窗口初值延续。

### 这里的最终共识表述

- 这是当前最合理、最值得优先验证的在线 IK 路线；
- 但具体 API 行为与开销仍以阶段 0 / 原型实测为准。

---

## 阶段 3：API Visualizer 在线显示

### 目标

在 OpenSim API 层实时显示求解得到的 `state`。

### 推荐路径

1. 加载 `scaled model`
2. 开启 `setUseVisualizer(true)`
3. 每次求解后调用 `show(state)`

### 显示目标

第一版只要求：

1. 模型姿态可连续刷新；
2. 关节角可同步输出；
3. 系统整体不被显示线程严重阻塞。

### 线程设计建议

可视化线程不应反向阻塞上游计算。  
显示层的目标是：

- **显示最新可用状态**

而不是：

- **绝不丢任何一帧**

---

## 阶段 4：系统集成与可选输出

### 第一版建议保留的输出

1. API Visualizer 实时显示；
2. 可选保存 joint angle 结果；
3. 可选记录 `.mot` / 日志用于离线分析。

### 第一版建议暂不纳入

1. OpenSim GUI 实时联动；
2. marker augmentation；
3. 自定义 C++ runtime；
4. 全功能网页前端。

---

## 6. 第一版范围边界

为了让项目真正可落地，第一版边界应写死。

## 第一版必须做

1. 阶段 0 环境验证
2. 实时前半段数据通道
3. 小窗口 IK 原型
4. API Visualizer 实时显示
5. simple model
6. 预缩放或 warm-up 缩放

## 第一版不做

1. OpenSim GUI 实时显示
2. marker augmentation
3. C++ 重写
4. 过早性能极限优化
5. 把所有离线模块一次性重构完

---

## 7. 最终风险表述

最终风险不再简单写成“高”或“中”，而采用更准确的表述方式。

## 风险 1：OpenSim Python API 的在线 IK 实测行为

这是当前最关键的不确定性。  
它集中体现在：

1. `InverseKinematicsSolver` 的窗口式使用方式；
2. `MarkersReference` / `TimeSeriesTableVec3` 的实际构造成本；
3. `track()` 在实时 marker 噪声下的稳定性。

**处理方式：** 通过阶段 0 和在线原型快速验证。

## 风险 2：在线 scaling 的适用性

自动 scaling 不是在所有动作场景下都稳。  
因此第一版必须优先依赖预缩放模型，或仅在可控站立 warm-up 条件下启用自动缩放。

## 风险 3：显示线程与求解线程互相阻塞

如果显示与 IK 求解串行耦合过深，会降低整体实时性。  
因此必须尽早设计分线程或缓冲策略。

## 风险 4：当前仓库的离线文件流水线迁移成本

上游模块当前严重依赖文件 I/O。  
因此实时版的难点不仅在 OpenSim，还在于整体数据通路要从“文件总线”改成“内存总线”。

---

## 8. 一句话定稿结论

**Pose2Sim 在线显示 OpenSim 结果是可行的；最终推荐路线是“实时前半段 + 小窗口 IK + OpenSim API Visualizer”，第一版优先使用预缩放模型与 simple model，所有具体性能与 solver 开销结论必须通过阶段 0 基准测试确认。**

---

## 9. 本定稿相对于前几轮讨论的最终收敛点

### 已彻底收敛的部分

1. 目标是在线显示 OpenSim 结果，而不是普通 3D skeleton
2. API Visualizer 优先于 GUI
3. 预缩放优先于在线自动缩放
4. 阶段 0 必须先做
5. 第一版跳过 marker augmentation
6. OneEuro 必须改成单向版本或先不用
7. 第一阶段不考虑 C++ 主线

### 仅保留为待验证项的部分

1. IK 的具体毫秒级耗时
2. solver 每窗口重建成本
3. Python/OpenSim API 的最终性能上限

---

## 10. 说明

本文件基于以下几轮材料收敛形成：

1. 原始实时化分析
2. 在线 OpenSim 结果方案审阅
3. 对 `Opus 4.6` 回应的再回应

并统一采用如下工程纪律：

- **路线可以靠代码与文档判断；**
- **性能与 API 行为必须靠目标环境实测确认。**
