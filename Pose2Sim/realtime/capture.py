#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime capture entry points.

This module provides both file-replay and live camera capture backends.
The rest of the realtime pipeline only depends on the ``FrameSource``
contract, so replay and live inputs can share the same downstream stages.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, Sequence, runtime_checkable

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


@dataclass(frozen=True)
class _CapturedFrame:
    source_frame_id: int
    timestamp: float
    image: Any


class _LiveCameraReader:
    """
    Dedicated per-camera reader that continuously keeps the latest frame.

    The public source assembles synchronized batches on top of these latest
    frames. This keeps live capture decoupled from downstream processing
    latency and avoids blocking all cameras on the slowest stage.
    """

    def __init__(
        self,
        *,
        source: str,
        camera_id: str,
        api_preference: int = 0,
        frame_width: int = 0,
        frame_height: int = 0,
        buffer_size: int = 1,
    ):
        self.source = source
        self.camera_id = camera_id
        self.api_preference = int(api_preference)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.buffer_size = int(buffer_size)

        self._capture: cv2.VideoCapture | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._frame_counter = -1
        self._latest_frame: _CapturedFrame | None = None
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)

    @staticmethod
    def _normalize_source(source: str) -> int | str:
        source_text = str(source).strip()
        if source_text.isdigit():
            return int(source_text)
        return source_text

    def start(self) -> None:
        self.stop()
        normalized_source = self._normalize_source(self.source)
        if self.api_preference > 0:
            capture = cv2.VideoCapture(normalized_source, self.api_preference)
        else:
            capture = cv2.VideoCapture(normalized_source)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open live camera source {self.source!r} for {self.camera_id}.")

        if self.frame_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.frame_width))
        if self.frame_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.frame_height))
        if self.buffer_size > 0:
            capture.set(cv2.CAP_PROP_BUFFERSIZE, float(self.buffer_size))

        self._capture = capture
        self._running = True
        self._frame_counter = -1
        self._latest_frame = None
        self._thread = threading.Thread(
            target=self._read_loop,
            name=f"live-camera-{self.camera_id}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._capture is not None:
            self._capture.release()
        self._capture = None

    def wait_for_latest(self, *, after_source_frame_id: int, timeout_s: float) -> _CapturedFrame | None:
        deadline = time.perf_counter() + max(0.0, timeout_s)
        with self._condition:
            while self._running:
                latest_frame = self._latest_frame
                if latest_frame is not None and latest_frame.source_frame_id > after_source_frame_id:
                    return latest_frame
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)
        with self._lock:
            latest_frame = self._latest_frame
            if latest_frame is not None and latest_frame.source_frame_id > after_source_frame_id:
                return latest_frame
        return None

    def _read_loop(self) -> None:
        assert self._capture is not None
        while self._running:
            ok, frame = self._capture.read()
            if not ok:
                time.sleep(0.01)
                continue
            captured_frame = _CapturedFrame(
                source_frame_id=self._frame_counter + 1,
                timestamp=time.perf_counter(),
                image=frame,
            )
            with self._condition:
                self._frame_counter = captured_frame.source_frame_id
                self._latest_frame = captured_frame
                self._condition.notify_all()


class LiveCameraFrameSource:
    """
    Multi-camera live capture source.

    Each camera is read on its own thread and the source returns the latest
    frame from each camera once all of them have advanced beyond the previous
    batch. This is a practical low-latency baseline for dual-view realtime
    capture without coupling the cameras to downstream processing speed.
    """

    def __init__(
        self,
        *,
        sources: Sequence[str],
        camera_ids: Sequence[str],
        frame_rate: float = 0.0,
        api_preference: int = 0,
        frame_width: int = 0,
        frame_height: int = 0,
        buffer_size: int = 1,
        read_timeout_ms: int = 1000,
        max_batch_skew_ms: float = 80.0,
    ):
        if not sources:
            raise ValueError("LiveCameraFrameSource requires at least one camera source.")
        if len(sources) != len(camera_ids):
            raise ValueError("sources and camera_ids must have the same length.")

        self.sources = tuple(str(source) for source in sources)
        self.camera_ids = tuple(str(camera_id) for camera_id in camera_ids)
        self.frame_rate = float(frame_rate)
        self.api_preference = int(api_preference)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.buffer_size = int(buffer_size)
        self.read_timeout_ms = int(read_timeout_ms)
        self.max_batch_skew_ms = float(max_batch_skew_ms)

        self._readers: list[_LiveCameraReader] = []
        self._running = False
        self._frame_id = 0
        self._last_source_frames: list[int] = [-1 for _ in self.sources]
        self._start_timestamp = 0.0

    def start(self) -> None:
        self.stop()
        self._frame_id = 0
        self._last_source_frames = [-1 for _ in self.sources]
        self._start_timestamp = time.perf_counter()
        self._readers = [
            _LiveCameraReader(
                source=source,
                camera_id=camera_id,
                api_preference=self.api_preference,
                frame_width=self.frame_width,
                frame_height=self.frame_height,
                buffer_size=self.buffer_size,
            )
            for source, camera_id in zip(self.sources, self.camera_ids)
        ]
        try:
            for reader in self._readers:
                reader.start()
        except Exception:
            self.stop()
            raise
        self._running = True

    def read(self) -> Sequence[FramePacket]:
        if not self._running:
            return []

        captured_frames: list[_CapturedFrame] = []
        timeout_s = max(0.001, self.read_timeout_ms / 1000.0)
        for index, reader in enumerate(self._readers):
            captured = reader.wait_for_latest(
                after_source_frame_id=self._last_source_frames[index],
                timeout_s=timeout_s,
            )
            if captured is None:
                self._running = False
                return []
            captured_frames.append(captured)

        timestamps = [frame.timestamp for frame in captured_frames]
        batch_timestamp = max(timestamps) - self._start_timestamp
        batch_skew_ms = (max(timestamps) - min(timestamps)) * 1000.0 if timestamps else 0.0

        frame_packets = []
        for index, (reader, captured) in enumerate(zip(self._readers, captured_frames)):
            frame_packets.append(
                FramePacket(
                    frame_id=self._frame_id,
                    timestamp=batch_timestamp,
                    camera_id=reader.camera_id,
                    image=captured.image,
                    metadata={
                        "source": reader.source,
                        "source_frame_id": captured.source_frame_id,
                        "capture_timestamp": captured.timestamp - self._start_timestamp,
                        "batch_skew_ms": batch_skew_ms,
                        "skew_exceeded": batch_skew_ms > self.max_batch_skew_ms,
                    },
                )
            )
            self._last_source_frames[index] = captured.source_frame_id

        self._frame_id += 1
        return tuple(frame_packets)

    def stop(self) -> None:
        for reader in self._readers:
            reader.stop()
        self._readers = []
        self._running = False


__all__ = [
    "FrameSource",
    "LiveCameraFrameSource",
    "ReplayFrameSource",
    "VideoReplayFrameSource",
]
