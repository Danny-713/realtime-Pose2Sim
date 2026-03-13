#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime pipeline scaffolding for Pose2Sim.

The realtime package is intentionally parallel to the existing offline modules.
It provides in-memory data structures and orchestration primitives without
changing the current file-based workflow.
"""

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

__all__ = [
    "FramePacket",
    "MarkerWindow",
    "MultiViewPosePacket",
    "OpenSimStatePacket",
    "Pose2DPacket",
    "Pose3DPacket",
    "RealtimeCaptureConfig",
    "RealtimeConfig",
    "RealtimeIKConfig",
    "RealtimePipeline",
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
