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

from Pose2Sim.realtime.capture import FrameSource, ReplayFrameSource
from Pose2Sim.realtime.filter_realtime import (
    ExponentialRealtimeFilter,
    PassThroughFilter,
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
from Pose2Sim.realtime.triangulate_frame import CallableFrameTriangulator, FrameTriangulator

__all__ = [
    "CallableFrameTriangulator",
    "ExponentialRealtimeFilter",
    "FramePacket",
    "FrameSource",
    "FrameTriangulator",
    "MarkerWindow",
    "MultiViewPosePacket",
    "OpenSimStatePacket",
    "Pose2DPacket",
    "Pose3DPacket",
    "PassThroughFilter",
    "RealtimePoseFilter",
    "RealtimeCaptureConfig",
    "RealtimeConfig",
    "RealtimeIKConfig",
    "RealtimePipeline",
    "ReplayFrameSource",
    "RealtimeRecorderConfig",
    "RealtimeTransportConfig",
    "SlidingMarkerBuffer",
    "load_realtime_config",
]


def __getattr__(name):
    if name in {
        "RealtimeCaptureConfig",
        "RealtimeConfig",
        "RealtimeIKConfig",
        "RealtimeRecorderConfig",
        "RealtimeTransportConfig",
        "load_realtime_config",
    }:
        from Pose2Sim.realtime import config as realtime_config

        return getattr(realtime_config, name)
    raise AttributeError(f"module 'Pose2Sim.realtime' has no attribute {name!r}")
