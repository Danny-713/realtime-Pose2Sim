#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Sliding marker window for realtime IK.
"""

from collections import deque
from typing import Deque, Optional, Sequence, Tuple

import numpy as np

from Pose2Sim.realtime.packets import MarkerWindow, Pose3DPacket


class SlidingMarkerBuffer:
    """
    Keep the latest N 3D marker packets in memory.
    """

    def __init__(self, window_size: int, expected_marker_names: Optional[Sequence[str]] = None):
        if window_size <= 0:
            raise ValueError("window_size must be a positive integer.")
        self.window_size = int(window_size)
        self.expected_marker_names = tuple(expected_marker_names) if expected_marker_names else None
        self._packets: Deque[Pose3DPacket] = deque(maxlen=self.window_size)

    def __len__(self) -> int:
        return len(self._packets)

    def clear(self) -> None:
        self._packets.clear()

    def ready(self, min_window_size: Optional[int] = None) -> bool:
        min_size = self.window_size if min_window_size is None else int(min_window_size)
        return len(self._packets) >= min_size

    def push(self, packet: Pose3DPacket) -> None:
        marker_names = tuple(packet.marker_names)
        markers_3d = np.asarray(packet.markers_3d, dtype=float)
        if markers_3d.ndim != 2 or markers_3d.shape[1] != 3:
            raise ValueError("Pose3DPacket.markers_3d must have shape (n_markers, 3).")
        if len(marker_names) != markers_3d.shape[0]:
            raise ValueError(
                f"Pose3DPacket marker name count {len(marker_names)} does not match markers array "
                f"shape {markers_3d.shape}."
            )
        if self.expected_marker_names is None:
            self.expected_marker_names = marker_names
        elif marker_names != self.expected_marker_names:
            raise ValueError(
                "Marker names do not match the current realtime buffer layout. "
                "Expected {}, got {}.".format(self.expected_marker_names, marker_names)
            )
        self._packets.append(packet)

    def latest_packet(self) -> Pose3DPacket:
        if not self._packets:
            raise RuntimeError("SlidingMarkerBuffer is empty.")
        return self._packets[-1]

    def latest_window(self, min_window_size: Optional[int] = None) -> MarkerWindow:
        if not self.ready(min_window_size=min_window_size):
            raise RuntimeError("SlidingMarkerBuffer does not contain enough frames yet.")

        packets = tuple(self._packets)
        marker_names = tuple(packets[-1].marker_names)
        frame_ids = tuple(packet.frame_id for packet in packets)
        timestamps = np.asarray([packet.timestamp for packet in packets], dtype=float)
        markers_3d = np.stack([packet.markers_3d for packet in packets], axis=0)
        if markers_3d.ndim != 3 or markers_3d.shape[2] != 3:
            raise RuntimeError(f"SlidingMarkerBuffer expected stacked marker data of shape (T, M, 3), got {markers_3d.shape}.")

        return MarkerWindow(
            frame_ids=frame_ids,
            timestamps=timestamps,
            marker_names=marker_names,
            markers_3d=markers_3d,
        )
