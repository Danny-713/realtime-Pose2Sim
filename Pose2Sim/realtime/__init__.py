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
from Pose2Sim.realtime.triangulate_frame import (
    CallableFrameTriangulator,
    FrameTriangulator,
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
    "RealtimeKalmanFilter",
    "RealtimePoseFilter",
    "RealtimeCaptureConfig",
    "RealtimeConfig",
    "RealtimeIKConfig",
    "RealtimePipeline",
    "ReplayFrameSource",
    "RealtimeRecorderConfig",
    "RealtimeVisualizerConfig",
    "SlidingMarkerBuffer",
    "VideoReplayFrameSource",
    "load_realtime_config",
]


def __getattr__(name):
    if name in {
        "RealtimeCaptureConfig",
        "RealtimeConfig",
        "RealtimeIKConfig",
        "RealtimeRecorderConfig",
        "RealtimeVisualizerConfig",
        "VideoReplayFrameSource",
        "load_realtime_config",
    }:
        if name == "VideoReplayFrameSource":
            from Pose2Sim.realtime.capture import VideoReplayFrameSource

            return VideoReplayFrameSource

        from Pose2Sim.realtime import config as realtime_config

        return getattr(realtime_config, name)
    raise AttributeError(f"module 'Pose2Sim.realtime' has no attribute {name!r}")
