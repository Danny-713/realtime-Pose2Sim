#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime-safe 3D marker filters.

These filters are causal by construction, which keeps them compatible with the
 future online pipeline. They are intentionally lightweight; the first goal is
 to provide a clean module boundary and a working default for early tests.
"""

from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

import numpy as np

from Pose2Sim.realtime.packets import Pose3DPacket


@runtime_checkable
class RealtimePoseFilter(Protocol):
    """
    Minimal contract for a causal 3D marker filter.
    """

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        """
        Return a filtered version of the latest 3D markers.
        """


class PassThroughFilter:
    """
    No-op filter used when we want to wire the pipeline before tuning smoothing.
    """

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        return pose3d


class ExponentialRealtimeFilter:
    """
    Simple causal smoother for bootstrap experiments.

    This is not meant to replace the final Kalman or single-pass OneEuro
    implementation. It gives the realtime package a small, understandable
    filter module that behaves like an actual online filter.
    """

    def __init__(self, alpha: float = 0.35):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in the interval (0, 1].")
        self.alpha = float(alpha)
        self._previous_markers: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._previous_markers = None

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        current = np.asarray(pose3d.markers_3d, dtype=float)
        if self._previous_markers is None:
            filtered = current.copy()
        else:
            filtered = self.alpha * current + (1.0 - self.alpha) * self._previous_markers
        self._previous_markers = filtered

        return Pose3DPacket(
            frame_id=pose3d.frame_id,
            timestamp=pose3d.timestamp,
            marker_names=tuple(pose3d.marker_names),
            markers_3d=filtered,
            reprojection_error=pose3d.reprojection_error,
            source_camera_ids=tuple(pose3d.source_camera_ids),
            metadata=dict(pose3d.metadata),
        )


__all__ = ["RealtimePoseFilter", "PassThroughFilter", "ExponentialRealtimeFilter"]
