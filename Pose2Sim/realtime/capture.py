#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime capture entry points.

This module intentionally stays small for now: it defines the frame-source
interface used by the realtime pipeline and provides a simple replay source for
bootstrap work. Concrete camera/RTSP backends can plug into the same contract
later without changing the pipeline orchestration.
"""

from __future__ import annotations

from typing import List, Protocol, Sequence, runtime_checkable

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


__all__ = ["FrameSource", "ReplayFrameSource"]
