#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Shared in-memory packet types for the realtime pipeline.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class FramePacket:
    """
    A single captured image frame.
    """

    frame_id: int
    timestamp: float
    camera_id: str
    image: Any
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Pose2DPacket:
    """
    2D keypoints detected for one camera at one instant.
    """

    frame_id: int
    timestamp: float
    camera_id: str
    keypoints: np.ndarray
    scores: Optional[np.ndarray] = None
    person_id: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MultiViewPosePacket:
    """
    Time-aligned multi-camera 2D detections for a single subject.
    """

    frame_id: int
    timestamp: float
    poses_by_camera: Mapping[str, Pose2DPacket]
    person_id: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def camera_ids(self) -> Tuple[str, ...]:
        return tuple(self.poses_by_camera.keys())


@dataclass(frozen=True)
class Pose3DPacket:
    """
    A single-frame 3D marker estimate.
    """

    frame_id: int
    timestamp: float
    marker_names: Sequence[str]
    markers_3d: np.ndarray
    reprojection_error: Optional[float] = None
    source_camera_ids: Tuple[str, ...] = tuple()
    metadata: Dict[str, Any] = field(default_factory=dict)

    def as_marker_dict(self) -> Dict[str, np.ndarray]:
        return {
            marker_name: self.markers_3d[idx]
            for idx, marker_name in enumerate(self.marker_names)
        }


@dataclass(frozen=True)
class MarkerWindow:
    """
    Sliding window of recent 3D markers for realtime IK.
    """

    frame_ids: Tuple[int, ...]
    timestamps: np.ndarray
    marker_names: Tuple[str, ...]
    markers_3d: np.ndarray

    @property
    def start_frame(self) -> int:
        return self.frame_ids[0]

    @property
    def end_frame(self) -> int:
        return self.frame_ids[-1]

    @property
    def num_frames(self) -> int:
        return len(self.frame_ids)


@dataclass(frozen=True)
class OpenSimStatePacket:
    """
    The latest realtime OpenSim output to be visualized or recorded.
    """

    frame_id: int
    timestamp: float
    coordinate_values: Mapping[str, float]
    source_window: Tuple[int, int]
    state: Optional[Any] = None
    latency_ms: Optional[float] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
