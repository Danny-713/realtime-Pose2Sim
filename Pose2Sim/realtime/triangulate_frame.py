#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Per-frame triangulation entry points.

The long-term goal is to extract and wrap the reusable math from
``Pose2Sim/triangulation.py`` so realtime code can stay in-memory. For now this
module defines the narrow interface the realtime pipeline expects.
"""

from __future__ import annotations

from typing import Callable, Protocol, runtime_checkable

from Pose2Sim.realtime.packets import MultiViewPosePacket, Pose3DPacket


@runtime_checkable
class FrameTriangulator(Protocol):
    """
    Realtime triangulator contract: one multi-view packet in, one 3D packet out.
    """

    def triangulate(self, packet: MultiViewPosePacket) -> Pose3DPacket:
        """
        Triangulate one synchronized multi-camera observation.
        """


class CallableFrameTriangulator:
    """
    Thin adapter around a plain callable.

    This keeps early experiments simple while still giving the pipeline a
    stable object-level interface.
    """

    def __init__(self, triangulate_fn: Callable[[MultiViewPosePacket], Pose3DPacket]):
        self._triangulate_fn = triangulate_fn

    def triangulate(self, packet: MultiViewPosePacket) -> Pose3DPacket:
        return self._triangulate_fn(packet)


__all__ = ["FrameTriangulator", "CallableFrameTriangulator"]
