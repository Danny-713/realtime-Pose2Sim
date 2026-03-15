#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime pipeline scaffolding for Pose2Sim.

The package now exposes a small set of runtime-facing modules that map directly
to the agreed implementation plan:

- capture
- triangulate_frame
- filter_realtime
- marker_buffer
- pipeline

Supporting types and config helpers remain available, but they are secondary to
those runtime modules.
"""

from Pose2Sim.realtime.capture import FrameSource, ReplayFrameSource, VideoReplayFrameSource
from Pose2Sim.realtime.filter_realtime import (
    PassThroughFilter,
    RealtimeKalmanFilter,
    RealtimePoseFilter,
)
from Pose2Sim.realtime.marker_buffer import SlidingMarkerBuffer
from Pose2Sim.realtime.packets import (
    FramePacket,
    MarkerWindow,
    MultiViewPosePacket,
    OpenSimStatePacket,
    Pose2DPacket,
    Pose3DPacket,
)
from Pose2Sim.realtime.pipeline import RealtimePipeline
from Pose2Sim.realtime.pose2d import RealtimePoseEstimator
from Pose2Sim.realtime.triangulate_frame import (
    CallableFrameTriangulator,
    FrameTriangulator,
    RealtimeFrameTriangulator,
)

__all__ = [
    "CallableFrameTriangulator",
    "FramePacket",
    "FrameSource",
    "FrameTriangulator",
    "MarkerWindow",
    "MultiViewPosePacket",
    "OpenSimStatePacket",
    "Pose2DPacket",
    "Pose3DPacket",
    "PassThroughFilter",
    "RealtimeCaptureConfig",
    "RealtimeConfig",
    "RealtimeFrameTriangulator",
    "RealtimeIKConfig",
    "RealtimeKalmanFilter",
    "RealtimePipeline",
    "RealtimePoseConfig",
    "RealtimePoseEstimator",
    "RealtimePoseFilter",
    "RealtimeRecorderConfig",
    "RealtimeVisualizerConfig",
    "ReplayFrameSource",
    "SlidingMarkerBuffer",
    "VideoReplayFrameSource",
    "load_realtime_config",
]


def __getattr__(name):
    _config_names = {
        "RealtimeCaptureConfig",
        "RealtimeConfig",
        "RealtimeIKConfig",
        "RealtimePoseConfig",
        "RealtimeRecorderConfig",
        "RealtimeVisualizerConfig",
        "load_realtime_config",
    }
    if name in _config_names:
        from Pose2Sim.realtime import config as realtime_config

        return getattr(realtime_config, name)
    raise AttributeError(f"module 'Pose2Sim.realtime' has no attribute {name!r}")
