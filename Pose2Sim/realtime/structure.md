# Pose2Sim Realtime 管线结构说明

这份文档描述的是当前 `Pose2Sim/realtime/` 目录里**已经接入主链**的 realtime 管线，而不是最早的 V1 设想稿。

如果你现在想快速理解项目，最重要的结论先放前面：

- `realtime/` 已经不是“逐帧三角化后直接进 IK”的简化版
- 当前主链已经包含：
  - replay / live 两种输入入口
  - 2D 质量屏蔽
  - per-frame triangulation
  - offline-like quality 预清洗
  - realtime 3D filter
  - marker augmentation
  - augmentation 后轻量滤波
  - sliding-window IK
  - visualizer / recorder
- 阶段之间的数据契约仍然是**内存 packet**
- 离线 `JSON -> TRC -> MOT` 仍然存在，但已经不是 realtime 主通路


## 1. 当前 realtime 的目标

当前 realtime 管线的目标不是复用离线脚本做“伪实时”，而是建立一条与离线平行的、以内存对象串联的在线管线：

```text
视频回放 / 实时相机
-> FramePacket
-> Pose2DPacket
-> MultiViewPosePacket
-> Pose3DPacket
-> MarkerWindow
-> OpenSimStatePacket
```

其中：

- replay 和 live 共享同一条 runtime 主链
- 2D、3D、清洗、augmentation、IK 都在内存里连续完成
- `.trc` / `.mot` 不再是实时主链的必经中间文件


## 2. 当前主链的真实执行顺序

当前代码的主执行顺序以 [pipeline.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pipeline.py) 为准。

### 2.1 共享 runtime 主链

```text
capture
-> pose2d
-> associate(single-person)
-> optional pose2d quality mask
-> triangulate
-> optional offline-like quality processor
-> optional realtime pose filter
-> optional marker augmenter
-> optional post-augmentation filter
-> marker buffer
-> IK
-> visualizer / recorder / publisher
```

### 2.2 当前 Demo 配置下实际启用的阶段

按当前 [Config.toml](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/Demo_SinglePerson/Config.toml)，主链通常是：

```text
video replay
-> 2D pose
-> 2D quality mask
-> triangulation
-> offline-like quality
-> Kalman filter
-> LSTM marker augmentation
-> post-augmentation One Euro
-> IK buffer
-> OpenSim IK
-> visualizer / recorder
```

也就是说，当前项目里“最稳”的 realtime 路线，已经明显比早期 V1 复杂。


## 3. 与离线 Pose2Sim 的关系

### 3.1 离线主通路

离线更接近：

```text
video
-> 2D json
-> triangulation
-> 3D trc
-> filtering
-> marker augmentation
-> OpenSim tool / mot
```

### 3.2 realtime 主通路

realtime 则改成：

```text
frame batches
-> packets in memory
-> online cleaning / filtering / IK
-> optional visualization / recording
```

关键区别：

- realtime 不以磁盘中间文件为主通路
- realtime 不能直接照搬离线整段算法，所以很多地方做的是 fixed-lag 近似
- 当前 `offline_like_quality` 的目标就是“把 realtime 前处理尽量向离线靠拢”


## 4. 入口层与共享装配层

### 4.1 文件回放入口

文件：

- [run_file_replay.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/run_file_replay.py)

职责：

- 读取单 trial 的 `Config.toml`
- 自动发现 `videos/*.mp4`
- 解析 calibration、model path、frame rate
- 创建 replay frame source
- 调用共享的 runtime 装配逻辑
- 运行 benchmark loop

### 4.2 实时相机入口

文件：

- [run_live_capture.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/run_live_capture.py)

职责：

- 读取 `realtime.capture.source_type = "live_camera"`
- 打开本地 camera index 或流地址
- 复用同一条 runtime 主链

### 4.3 共享装配层

文件：

- [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py)

职责：

- 统一 replay / live 的 preflight
- 解析 realtime config
- 创建所有运行时模块
- 打印配置与 benchmark summary

这是当前 realtime 的真正“组装中心”。


## 5. 数据契约：packet 级别怎么传

文件：

- [packets.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/packets.py)

当前主链核心 packet 有 6 个。

### 5.1 `FramePacket`

表示某个逻辑时刻、某个相机的一张图像。

核心字段：

- `frame_id`
- `timestamp`
- `camera_id`
- `image`
- `metadata`

其中：

- `image` 通常是 OpenCV 读出的 `ndarray`
- 形状通常是 `(H, W, 3)`

### 5.2 `Pose2DPacket`

表示某一路相机在某一时刻的单人 2D 关键点结果。

核心字段：

- `frame_id`
- `timestamp`
- `camera_id`
- `keypoints`
- `scores`

典型形状：

- `keypoints.shape == (K, 2)`
- `scores.shape == (K,)`

当前默认 `Body_with_feet -> HALPE_26`，所以 `K = 26`。

### 5.3 `MultiViewPosePacket`

表示同一逻辑时刻的多相机单人 2D observation。

核心字段：

- `frame_id`
- `timestamp`
- `poses_by_camera`

典型结构：

```python
{
    "cam01": Pose2DPacket,
    "cam02": Pose2DPacket,
    "cam03": Pose2DPacket,
    "cam04": Pose2DPacket,
}
```

### 5.4 `Pose3DPacket`

表示单帧 3D marker 结果。

核心字段：

- `frame_id`
- `timestamp`
- `marker_names`
- `markers_3d`
- `reprojection_error`
- `source_camera_ids`
- `metadata`

典型形状：

- `markers_3d.shape == (M, 3)`

这里要注意：

- augmentation 关闭时，`M` 通常接近原始 pose keypoints 数
- augmentation 开启后，`M` 会变大，不再等于原始 keypoint 数

### 5.5 `MarkerWindow`

表示最近若干帧 3D marker 组成的 IK 滑动窗口。

核心字段：

- `frame_ids`
- `timestamps`
- `marker_names`
- `markers_3d`

典型形状：

- `markers_3d.shape == (T, M, 3)`

### 5.6 `OpenSimStatePacket`

表示当前窗口求得的 OpenSim 输出。

核心字段：

- `frame_id`
- `timestamp`
- `coordinate_values`
- `source_window`
- `latency_ms`
- `metadata`


## 6. 各阶段模块怎么分工

### 6.1 输入层

文件：

- [capture.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/capture.py)

当前支持两种 `FrameSource`：

- `VideoReplayFrameSource`
- `LiveCameraFrameSource`

这层只负责：

- 取帧
- 维护相机 id
- 产出一批时间对齐的 `FramePacket`

下游并不知道它来自文件还是相机。

### 6.2 2D 感知层

文件：

- [pose2d.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pose2d.py)

职责：

- 每个相机一套 `PoseTracker`
- detector + pose
- NMS
- 单人 tracking
- 每相机只保留一个人

输出：

- `List[Pose2DPacket]`

### 6.3 单人多视角聚合层

文件：

- [pipeline.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pipeline.py)

当前没有单独的 `association.py`。  
单人场景下，`pipeline._associate()` 直接按 `camera_id` 聚合成：

- `MultiViewPosePacket`

### 6.4 2D 质量屏蔽层

文件：

- [offline_like_quality.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/offline_like_quality.py)

类：

- `RealtimePose2DQualityMask`

职责：

- 在 triangulation 前，先按离线逻辑屏蔽低 confidence 的 2D 点
- 若 `score < likelihood_threshold_triangulation`，则对应 `(x, y, score)` 置为 `NaN`

这一步是把离线“低置信度 2D 先不参与三角化”的思想搬到了 realtime。

### 6.5 逐帧三角化层

文件：

- [triangulate_frame.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/triangulate_frame.py)

类：

- `RealtimeFrameTriangulator`

职责：

- 对每个 marker 汇总多相机 `(x, y, confidence)`
- 调离线核心数学 `triangulation_from_best_cameras()`
- 做 `Z-up -> Y-up` 坐标变换
- 输出 `Pose3DPacket`

当前还会在 `metadata` 里附带：

- `per_marker_reprojection_error`
- `per_marker_excluded_cameras`
- `per_marker_valid_after_triangulation`

这些信息后面会被 offline-like quality 继续利用。

### 6.6 offline-like quality 层

文件：

- [offline_like_quality.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/offline_like_quality.py)

类：

- `RealtimeOfflineLikeQualityProcessor`

这是当前 realtime 前处理里非常关键的一层。它不是完整复刻离线整段处理，而是 fixed-lag 近似版，顺序是：

1. 3D quality gate
2. short-gap interpolation
3. large-gap fill
4. Hampel-like outlier replacement
5. 输出中心帧

它的作用是：

- 在 augmentation 和 IK 之前，先把 raw 3D 尽量清理得更像离线输入

### 6.7 realtime 3D filter

文件：

- [filter_realtime.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/filter_realtime.py)

当前支持：

- `PassThroughFilter`
- `RealtimeKalmanFilter`
- `RealtimeOneEuroFilter`
- `RealtimeButterworthWindowFilter`
- `RealtimeKalmanRTSWindowFilter`

通过 [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py) 的 `build_pose_filter()` 装配。

注意：

- 这层是 raw / cleaned 3D filter
- 和 augmentation 后的 post-filter 不是同一层

### 6.8 marker augmentation 层

文件：

- [augmentation.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/augmentation.py)

类：

- `RealtimeMarkerAugmenter`

职责：

- 复用离线 LSTM augmenter 资产
- 对当前 `HALPE_26` 的 3D 点做 fixed-lag augmentation
- 输出 “原始 marker + LSTM response markers”

关键点：

- 当前只支持 `Body_with_feet / HALPE_26`
- 当前支持 `center` / `latest`
- 当前 demo 配置使用 `center`

### 6.9 augmentation 后滤波层

文件：

- [filter_realtime.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/filter_realtime.py)
- [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py)

概念上是：

- `post_augmentation_filter`

当前主要支持：

- post-augmentation One Euro

作用：

- 对 augmentation 后的新 marker 再做一层轻量平滑

### 6.10 滑动窗口层

文件：

- [marker_buffer.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/marker_buffer.py)

类：

- `SlidingMarkerBuffer`

职责：

- 按 marker layout 缓存最近若干帧 `Pose3DPacket`
- 在窗口 ready 时输出 `MarkerWindow`

### 6.11 OpenSim IK 层

文件：

- [opensim_ik.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/opensim_ik.py)

类：

- `RealtimeIKSolver`

职责：

- 根据当前 marker set 选择模型里实际能用的 markers
- 对窗口内 marker gap 做填补
- 组装 `TimeSeriesTableVec3`
- 创建 `MarkersReference`
- 在窗口内逐帧 `assemble / track`
- 输出 `OpenSimStatePacket`

这里的 window IK 是当前 realtime OpenSim 主链的核心。

### 6.12 输出层

文件：

- [opensim_viz.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/opensim_viz.py)
- [recorder.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/recorder.py)

职责：

- visualizer：把 `coordinate_values` 显示到 API visualizer
- recorder：可选记录 markers、coordinates、`realtime.mot`


## 7. 当前配置层怎么理解

文件：

- [config.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/config.py)

当前关键 realtime 配置块有：

- `[realtime.capture]`
- `[realtime.pose]`
- `[realtime.offline_like_quality]`
- `[realtime.filtering]`
- `[realtime.augmentation]`
- `[realtime.post_augmentation_filter]`
- `[realtime.ik]`
- `[realtime.visualizer]`
- `[realtime.recorder]`

### 7.1 当前真正接入主链的配置

这些配置块已经接入 runtime 主链：

- `capture`
- `pose`
- `offline_like_quality`
- `filtering`
- `augmentation`
- `post_augmentation_filter`
- `ik`
- `visualizer`
- `recorder`

### 7.2 当前保留但没有接入主链的旧实验配置

配置里还保留了：

- `pre_augmentation_cleanup`

但按当前 [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py) 的实际装配逻辑，这一层**没有进入当前主链**。  
也就是说：

- 它还在 config dataclass 里
- 日志也会识别它
- 但当前 pipeline 真实执行时，不再使用这一层

理解项目时，应该把它视为：

- **保留的旧实验配置**
而不是：
- 当前主链的一部分


## 8. 当前 fixed-lag 都来自哪里

当前 realtime 已经不只是一个 IK 窗口。

可能引入 fixed-lag 的层包括：

- `offline_like_quality.output_mode = center`
- `filtering.type = butterworth / kalman_rts`
- `augmentation.output_mode = center`
- `ik.window_size`

其中：

- quality / augmentation 的 `center` 会吃掉头尾帧，并引入显式延迟
- IK 窗口主要引入 warm-up，而不是同样意义上的中心帧延迟

当前 [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py) 也会在日志里打印 fixed-lag 估算。


## 9. 当前 Demo_SinglePerson 的默认理解方式

如果你现在只想用一句话记住 Demo 当前主链，可以记成：

```text
4 路同步视频
-> 单人 HALPE_26
-> 离线风格质量清洗
-> Kalman
-> LSTM marker augmentation
-> augmentation 后 One Euro
-> simple OpenSim model 的小窗口 IK
-> visualizer / recorder
```

这已经比最早的 realtime V1 设想更完整，也更接近离线工作流。


## 10. 当前已知边界

### 10.1 明确支持的方向

- 单人
- replay
- live camera
- API visualizer
- simple model realtime baseline
- offline-like quality + augmentation + post-filter 的组合

### 10.2 当前仍然要注意的边界

- realtime 仍然不是离线整段算法的完整复刻
- 三角化仍然是 per-frame 为主，后面再做 fixed-lag 清洗
- 当前 `RealtimeFrameTriangulator` 仍要求 runtime `camera_ids` 与 calibration 相机集合一致
- 当前主链更适合固定动作、固定 rig 的场景，而不是完全自由场景


## 11. 阅读代码的推荐顺序

如果你想从“能看懂这项目”出发，而不是一上来钻实现细节，最推荐按这个顺序读：

1. [run_file_replay.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/run_file_replay.py)  
   先看入口怎么启动

2. [runtime_support.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/runtime_support.py)  
   看模块是怎么装起来的

3. [pipeline.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/pipeline.py)  
   看一帧数据是怎么一步步往下走的

4. [packets.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/packets.py)  
   看每层输入输出是什么

5. [offline_like_quality.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/offline_like_quality.py)  
   看当前前处理为什么比早期版本复杂很多

6. [augmentation.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/augmentation.py)  
   看 marker augmentation 是怎么插进来的

7. [opensim_ik.py](/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/realtime/opensim_ik.py)  
   看 realtime OpenSim 小窗口 IK 的核心


## 12. 一句话总结

当前 `Pose2Sim/realtime/` 已经不是“简化版逐帧 IK demo”，而是一条共享 replay/live 输入、带 offline-like 质量清洗、marker augmentation、后处理滤波和 rolling-window IK 的在线 OpenSim 管线。
