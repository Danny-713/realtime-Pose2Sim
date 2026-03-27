# Pose2Sim Realtime 管线结构说明

本文档面向当前 `Pose2Sim/realtime/` 目录，详细说明这条 realtime 管线的设计目标、数据流、输入输出、内存数据形状，以及它相对原始离线 Pose2Sim 管线的改造方式。

本文档的技术立场与此前 `opus第三次.md`、`gpt第三次.md` 的共识一致：

- 实时化不是继续沿用 `JSON -> TRC -> MOT` 作为主通路
- 实时化应建立一条与离线流程平行的 `realtime/` 管线
- 在线 OpenSim 应优先走 `小窗口 IK + API Visualizer`
- 第一版范围收敛为：
  - 单人
  - 多视频回放
  - 同机显示
  - 不做自动同步
  - 不做 marker augmentation
  - 不做 OpenSim GUI
  - 不做服务器到客户端传输


## 1. 总体目标

当前 realtime V1 的目标不是“把现有离线脚本简单提速”，而是建立一条新的、以内存数据传递为主的在线管线：

```text
多机位视频回放
-> 2D 姿态估计
-> 单人多视角聚合
-> 逐帧三角化
-> 单向实时滤波
-> 滑动窗口 IK
-> OpenSim API Visualizer / recorder
```

这条管线的核心变化有两点：

1. 原离线 Pose2Sim 依赖磁盘中间文件串联阶段；
2. 当前 realtime 管线改为以内存对象作为阶段之间的数据契约。


## 2. 从离线文件链到 realtime 内存链

### 2.1 原离线管线的主通路

原始 Pose2Sim 更接近下面这种模式：

```text
视频
-> 2D 检测结果写 JSON
-> 同步 / 关联 / 三角化
-> 3D 点写 TRC
-> OpenSim Tool 读取 TRC
-> 生成 MOT
-> OpenSim GUI / 其他工具查看
```

这里的主要特征是：

- 阶段之间靠文件交接
- 工具偏批处理
- 默认按整段序列思维工作

### 2.2 当前 realtime 管线的主通路

当前 `Pose2Sim/realtime/` 已经把主通路改成：

```text
videos/*.mp4
-> FramePacket
-> Pose2DPacket
-> MultiViewPosePacket
-> Pose3DPacket
-> MarkerWindow
-> OpenSimStatePacket
```

对应关系如下：

- 不再生成 `JSON` 作为 2D 主通路中间文件
- 不再生成 `TRC` 作为 IK 主通路中间文件
- 不再生成 `MOT` 才能可视化
- `MOT` 只保留为可选 recorder 输出

也就是说：

- 磁盘文件仍然存在，但只承担“输入源 / 配置 / 可选输出”的角色
- 阶段之间的数据交接，已经转为内存对象


## 3. 当前管线里哪些东西还来自磁盘

当前 realtime V1 不是“完全无磁盘”，而是“主通路不依赖中间文件”。

仍然从磁盘读取的东西有：

- 输入视频：`project_dir/videos/*.mp4`
- 标定文件：`project_dir/calibration/*.toml`
- OpenSim 模型：`.osim`
- OpenSim Geometry
- 配置文件：`Config.toml`

可选写回磁盘的东西有：

- `realtime.mot`
- `realtime_coordinates.tsv`

但这些输出不是主通路必需条件。


## 4. 主模块与职责分层

当前 `Pose2Sim/realtime/` 的主模块可以按下面理解：

### 4.1 入口层

- `run_file_replay.py`

职责：

- 读取 `Config.toml`
- 解析模型路径、视频源、标定文件
- 创建各个 realtime 模块
- 启动整条 replay 管线
- 打印 benchmark summary

### 4.2 编排层

- `pipeline.py`

职责：

- 串联各阶段
- 每步调用：
  - `pose2d`
  - `triangulate`
  - `filter`
  - `marker_buffer`
  - `ik`
- 记录阶段耗时与统计信息

### 4.3 数据契约层

- `packets.py`

职责：

- 定义内存传递的数据结构
- 统一每个阶段的输入输出格式

### 4.4 感知与几何层

- `capture.py`
- `pose2d.py`
- `triangulate_frame.py`
- `filter_realtime.py`

职责：

- 把多视频帧逐步变成滤波后的 3D marker

### 4.5 OpenSim 层

- `marker_buffer.py`
- `opensim_ik.py`
- `opensim_viz.py`
- `recorder.py`

职责：

- 管理小窗口
- 做滚动窗口 IK
- 更新 OpenSim 可视化
- 记录关节坐标与可选 MOT


## 5. 数据包与内存数据形状

当前 realtime 主通路依赖 [packets.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/packets.py) 中定义的 6 个核心 packet。

### 5.1 `FramePacket`

来源：

- `capture.py`

含义：

- 某一个逻辑时刻、某一个相机的一张原始图像帧

核心字段：

- `frame_id: int`
- `timestamp: float`
- `camera_id: str`
- `image: Any`
- `metadata: dict`

实际内存形状：

- `image` 通常是 OpenCV 读出的数组
- 常见形状：`(H, W, 3)`
- 常见 dtype：`uint8`

### 5.2 `Pose2DPacket`

来源：

- `pose2d.py`

含义：

- 某个相机、某一时刻、单人的 2D 关键点结果

核心字段：

- `frame_id`
- `timestamp`
- `camera_id`
- `keypoints`
- `scores`

实际内存形状：

- `keypoints.shape == (K, 2)`
- `scores.shape == (K,)`

其中：

- `K` = 当前 pose model 的关键点数
- 例如 `Body_with_feet -> HALPE_26` 时，`K = 26`

### 5.3 `MultiViewPosePacket`

来源：

- `pipeline.py` 中的默认单人聚合 `_associate()`

含义：

- 把同一逻辑时刻的 4 路 `Pose2DPacket` 聚合为一个多视角 observation

核心字段：

- `frame_id`
- `timestamp`
- `poses_by_camera: Mapping[str, Pose2DPacket]`

实际内存结构：

```python
{
    "cam01": Pose2DPacket,
    "cam02": Pose2DPacket,
    "cam03": Pose2DPacket,
    "cam04": Pose2DPacket,
}
```

### 5.4 `Pose3DPacket`

来源：

- `triangulate_frame.py`
- 后续可能被 `filter_realtime.py` 更新

含义：

- 单帧 3D marker 结果

核心字段：

- `marker_names`
- `markers_3d`
- `reprojection_error`

实际内存形状：

- `markers_3d.shape == (M, 3)`

其中：

- `M` = marker 数
- 当前第一版里，通常与 `K` 相同，基本可理解为“3D keypoints”

说明：

- 在当前 V1 中，没有 marker augmentation
- 所以这里的 `marker` 基本就是“用于 OpenSim IK 的 3D 关键点”
- 它不是离线 `.trc` 文件，而是内存中的 `numpy.ndarray`

### 5.5 `MarkerWindow`

来源：

- `marker_buffer.py`

含义：

- 最近若干帧 3D marker 组成的滑动窗口

核心字段：

- `frame_ids`
- `timestamps`
- `marker_names`
- `markers_3d`

实际内存形状：

- `markers_3d.shape == (T, M, 3)`

其中：

- `T` = 窗口长度，当前默认是 `10`
- `M` = marker 数

这是当前 realtime 管线中唯一明确的“滑动窗口对象”。

### 5.6 `OpenSimStatePacket`

来源：

- `opensim_ik.py`

含义：

- 当前窗口求解出的 OpenSim 输出

核心字段：

- `frame_id`
- `timestamp`
- `coordinate_values: Mapping[str, float]`
- `source_window: Tuple[int, int]`
- `latency_ms`
- `metadata`

实际内存结构：

```python
{
    "pelvis_tilt": ...,
    "hip_flexion_r": ...,
    "knee_angle_r": ...,
    ...
}
```

这就是当前 visualizer 和 recorder 的直接输入。


## 6. 当前 realtime 数据流是怎么走的

下面按实际执行顺序描述。

### 6.1 视频回放 -> `FramePacket`

文件：

- [capture.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/capture.py)

`VideoReplayFrameSource` 会：

1. 打开所有 `videos/*.mp4`
2. 每次从每路视频读取一帧
3. 给这一逻辑时刻的每路图像打包成一组 `FramePacket`

因此一轮 `read()` 的返回值是：

```python
Sequence[FramePacket]
```

也就是一批同一时刻、不同相机的图像。

### 6.2 `FramePacket` -> `Pose2DPacket`

文件：

- [pose2d.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pose2d.py)

`RealtimePoseEstimator` 会：

1. 为每个相机维持独立 `PoseTracker`
2. 跑 detector + pose
3. 做 NMS
4. 做单人 tracking
5. 只保留一个人

输出为每个相机一个 `Pose2DPacket`：

```python
List[Pose2DPacket]
```

### 6.3 `Pose2DPacket x N cameras` -> `MultiViewPosePacket`

文件：

- [pipeline.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pipeline.py)

当前 V1 不单独引入 `association.py`。  
单人场景下，`pipeline._associate()` 直接按 `camera_id` 聚合：

```python
List[Pose2DPacket] -> MultiViewPosePacket
```

### 6.4 `MultiViewPosePacket` -> `Pose3DPacket`

文件：

- [triangulate_frame.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/triangulate_frame.py)

这一层会：

1. 读取标定和投影矩阵
2. 对每个 marker 汇总所有相机的 `(x, y, confidence)`
3. 调用离线核心数学 `triangulation_from_best_cameras()`
4. 得到单帧 3D 点
5. 做 `Z-up -> Y-up` 转换，和离线 `.trc` 一致

输出：

```python
Pose3DPacket
```

这里是 realtime 路线里最重要的“2D -> 3D”转换点。

### 6.5 `Pose3DPacket` -> 滤波后的 `Pose3DPacket`

文件：

- [filter_realtime.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/filter_realtime.py)

当前默认是：

- `RealtimeKalmanFilter`

它不是离线那种整段双向滤波，而是：

- 单向
- 因果
- 每个 marker 独立维护状态

所以这里的输出仍然是：

```python
Pose3DPacket
```

只是 `markers_3d` 已经被因果滤波过。

### 6.6 `Pose3DPacket` -> `MarkerWindow`

文件：

- [marker_buffer.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/marker_buffer.py)

`SlidingMarkerBuffer` 会：

1. 每来一帧 `Pose3DPacket` 就 `push`
2. 当窗口攒满时，返回最近 `T` 帧

输出：

```python
MarkerWindow
```

当前窗口大小默认：

- `window_size = 10`

这是当前 realtime 路线里唯一显式使用滑动窗口的位置。

### 6.7 `MarkerWindow` -> `OpenSimStatePacket`

文件：

- [opensim_ik.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/opensim_ik.py)

`RealtimeIKSolver` 会：

1. 从窗口里选出当前模型实际能用的 marker
2. 在窗口内做缺失值填补
3. 把 `markers_3d` 组装成 `TimeSeriesTableVec3`
4. 构造 `MarkerWeightSet`
5. 构造 `MarkersReference`
6. 构造 `InverseKinematicsSolver`
7. 对整个窗口逐时刻调用 `assemble/track`
8. 取窗口末时刻的 OpenSim `state`
9. 把 coordinate 导出成 `OpenSimStatePacket`

换句话说：

```text
MarkerWindow(T, M, 3)
-> OpenSim 小窗口 IK
-> 当前 frame 的 coordinate_values
```

### 6.8 `OpenSimStatePacket` -> Visualizer / Recorder

文件：

- [opensim_viz.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/opensim_viz.py)
- [recorder.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/recorder.py)

`OpenSimVisualizer`：

- 根据 `coordinate_values` 更新本地 OpenSim state
- 调 `show(state)` 实时显示

`RealtimeRecorder`：

- 可选记录坐标值
- 可选写 `realtime.mot`


## 7. 这条 realtime 管线和 `.trc` 回放的本质差别

这是理解当前效果差异的关键。

### 7.1 `test_opensim_window_visualizer.py` 吃的是什么

该测试脚本默认读取的是离线文件，例如：

- `Demo_SinglePerson_1-96_filt_butterworth_LSTM.trc`

这意味着它吃到的是：

- 已经离线三角化过的 3D 点
- 已经离线滤波过的数据
- 可能已经过其他后处理

所以那条路径更像：

```text
高质量离线 marker
-> 小窗口 IK
-> Visualizer
```

### 7.2 `run_file_replay.py` 吃的是什么

production realtime 路线吃到的是：

```text
视频
-> 当前帧 2D
-> 当前帧 3D
-> 因果滤波
-> 小窗口 IK
```

所以它天然会：

- 更抖
- 更依赖前半段 2D/3D 质量
- 不可能直接等同于离线 `.trc` 效果

这不是因为 realtime 设计错了，而是因为：

- 两条链的输入质量不同
- 一条是“离线后处理结果”
- 一条是“在线生成结果”


## 8. benchmark 是在哪一层量的

当前 benchmark 是在 [run_file_replay.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/run_file_replay.py) 里按阶段统计的。

对应关系如下：

- `capture_avg_ms`
  - `capture.py`
- `pose2d_avg_ms`
  - `pose2d.py`
- `triangulate_avg_ms`
  - `triangulate_frame.py`
- `filter_avg_ms`
  - `filter_realtime.py`
- `ik_avg_ms`
  - `opensim_ik.py`

因此 benchmark 不是黑箱总时间，而是已经和模块结构对应起来了。


## 9. 当前 realtime V1 的关键工程判断

结合此前 `opus第三次.md` 与 `gpt第三次.md` 的共识，以及当前代码现状，可以把 realtime V1 理解成：

### 9.1 已经完成的改造

- 从文件中间件改为 packet 内存传递
- 建立了平行于离线流程的 `realtime/`
- 用 `MarkerWindow + RealtimeIKSolver` 替代离线 `TRC -> IK Tool`
- 用 `API Visualizer` 替代 `OpenSim GUI` 作为第一版在线显示后端

### 9.2 还没有做的事情

- 自动同步
- 多人关联
- marker augmentation
- 服务器到客户端传输
- 更强的固定延迟平滑 / smoother
- 更高质量的实时 2D/3D 前半段

### 9.3 当前最重要的现实边界

当前 realtime V1 是：

- 一条结构清晰的、可运行的、以内存为主通路的在线 OpenSim 管线

但它还不是：

- 一个已经达到离线质量的最终系统
- 一个多人、跨机器、强鲁棒的完整产品


## 10. 一句话总结

当前 `Pose2Sim/realtime/` 的本质是：

**用一组显式定义的内存 packet，把“多视频回放 -> 2D -> 3D -> 小窗口 IK -> OpenSim state” 这条链从离线文件流水线改造成了在线内存流水线。**

最关键的内存对象是：

```text
FramePacket
-> Pose2DPacket
-> MultiViewPosePacket
-> Pose3DPacket
-> MarkerWindow
-> OpenSimStatePacket
```

而最关键的改造点不是“把旧脚本提速”，而是：

**把数据契约从磁盘文件改成了模块之间直接传递的内存对象。**
