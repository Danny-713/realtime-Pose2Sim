#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime capture entry points.

This module provides file-replay capture for the first realtime milestone.
The interface stays compatible with future live camera sources.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol, Sequence, runtime_checkable

import cv2

from Pose2Sim.realtime.packets import FramePacket


@runtime_checkable
class FrameSource(Protocol):
    """
    Minimal contract for any realtime or replay capture backend.
    """

    def start(self) -> None:
        """
        Allocate resources and prepare streaming.
        """

    def read(self) -> Sequence[FramePacket]:
        """
        Return the next time-aligned batch of per-camera frames.
        """

    def stop(self) -> None:
        """
        Release resources.
        """


class ReplayFrameSource:
    """
    Deterministic in-memory source used for early pipeline bring-up.

    Each item in ``frame_batches`` is one logical timestep containing one
    ``FramePacket`` per camera.
    """

    def __init__(self, frame_batches: Sequence[Sequence[FramePacket]]):
        self._frame_batches = [tuple(batch) for batch in frame_batches]
        self._cursor = 0
        self._running = False

    def start(self) -> None:
        self._cursor = 0
        self._running = True

    def read(self) -> Sequence[FramePacket]:
        if not self._running or self._cursor >= len(self._frame_batches):
            return []
        batch = self._frame_batches[self._cursor]
        self._cursor += 1
        return batch

    def stop(self) -> None:
        self._running = False

    def remaining_batches(self) -> int:
        return max(0, len(self._frame_batches) - self._cursor)


class VideoReplayFrameSource:
    """
    Multi-camera file replay source.

    Video files are assumed to be pre-synchronized. Optional per-camera frame
    offsets can be used to skip leading frames when needed.
    """

    def __init__(
        self,
        sources: Sequence[str],
        camera_ids: Sequence[str],
        frame_rate: float = 0.0,
        frame_offsets: Sequence[int] | None = None,
        stop_on_shortest: bool = True,
    ):
        if not sources:
            raise ValueError("VideoReplayFrameSource requires at least one source video.")
        if len(sources) != len(camera_ids):
            raise ValueError("sources and camera_ids must have the same length.")

        self.sources = tuple(str(Path(source).resolve()) for source in sources)
        self.camera_ids = tuple(camera_ids)
        self.frame_rate = float(frame_rate)
        self.frame_offsets = tuple(int(offset) for offset in (frame_offsets or [0] * len(sources)))
        if len(self.frame_offsets) != len(self.sources):
            raise ValueError("frame_offsets must match the number of sources.")
        self.stop_on_shortest = bool(stop_on_shortest)

        self._captures: list[cv2.VideoCapture] = []
        self._running = False
        self._frame_id = 0
        self._source_frames = list(self.frame_offsets)

    def start(self) -> None:
        self.stop()
        self._captures = []
        self._frame_id = 0
        self._source_frames = list(self.frame_offsets)

        detected_fps = None
        for source, offset in zip(self.sources, self.frame_offsets):
            capture = cv2.VideoCapture(source)
            if not capture.isOpened():
                self.stop()
                raise RuntimeError(f"Could not open replay video: {source}")
            if offset > 0:
                capture.set(cv2.CAP_PROP_POS_FRAMES, int(offset))
            fps = capture.get(cv2.CAP_PROP_FPS)
            if detected_fps is None and fps and fps > 0:
                detected_fps = float(fps)
            self._captures.append(capture)

        if self.frame_rate <= 0 and detected_fps:
            self.frame_rate = detected_fps
        if self.frame_rate <= 0:
            self.frame_rate = 30.0

        self._running = True

    def read(self) -> Sequence[FramePacket]:
        if not self._running:
            return []

        frame_packets = []
        for index, (capture, camera_id, source) in enumerate(zip(self._captures, self.camera_ids, self.sources)):
            ok, frame = capture.read()
            if not ok:
                if self.stop_on_shortest:
                    self._running = False
                    return []
                continue

            source_frame_id = self._source_frames[index]
            timestamp = self._frame_id / self.frame_rate
            frame_packets.append(
                FramePacket(
                    frame_id=self._frame_id,
                    timestamp=timestamp,
                    camera_id=camera_id,
                    image=frame,
                    metadata={
                        "source": source,
                        "source_frame_id": source_frame_id,
                    },
                )
            )
            self._source_frames[index] += 1

        if not frame_packets:
            self._running = False
            return []

        self._frame_id += 1
        return tuple(frame_packets)

    def stop(self) -> None:
        for capture in self._captures:
            capture.release()
        self._captures = []
        self._running = False


__all__ = ["FrameSource", "ReplayFrameSource", "VideoReplayFrameSource"]
