#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime pipeline orchestration.

This is the main entry point for the runtime-facing realtime package. The
capture backend, pose estimator, per-frame triangulator, optional causal
filter, rolling marker buffer, IK solver, and output sinks are wired together
here.
"""

from typing import Any, List, Optional, Sequence

from Pose2Sim.realtime.capture import FrameSource
from Pose2Sim.realtime.filter_realtime import RealtimePoseFilter
from Pose2Sim.realtime.marker_buffer import SlidingMarkerBuffer
from Pose2Sim.realtime.packets import (
    FramePacket,
    MultiViewPosePacket,
    OpenSimStatePacket,
    Pose2DPacket,
    Pose3DPacket,
)
from Pose2Sim.realtime.triangulate_frame import FrameTriangulator


class RealtimePipeline:
    """
    Glue logic for the realtime path.

    Each component is intentionally duck-typed so the first implementation can
    evolve quickly without introducing a heavy abstraction layer.
    """

    def __init__(
        self,
        frame_source: FrameSource,
        pose2d_estimator: Any,
        triangulator: FrameTriangulator,
        marker_buffer: SlidingMarkerBuffer,
        associator: Optional[Any] = None,
        pose3d_filter: Optional[RealtimePoseFilter] = None,
        ik_solver: Optional[Any] = None,
        visualizer: Optional[Any] = None,
        recorder: Optional[Any] = None,
        publisher: Optional[Any] = None,
        min_window_size: Optional[int] = None,
    ):
        self.frame_source = frame_source
        self.pose2d_estimator = pose2d_estimator
        self.associator = associator
        self.triangulator = triangulator
        self.pose3d_filter = pose3d_filter
        self.marker_buffer = marker_buffer
        self.ik_solver = ik_solver
        self.visualizer = visualizer
        self.recorder = recorder
        self.publisher = publisher
        self.min_window_size = min_window_size
        self._running = False

    def start(self) -> None:
        if hasattr(self.frame_source, "start"):
            self.frame_source.start()
        self._running = True

    def stop(self) -> None:
        self._running = False
        if hasattr(self.frame_source, "stop"):
            self.frame_source.stop()

    def run_forever(self) -> None:
        self.start()
        try:
            while self._running:
                frame_packets = self.frame_source.read()
                if not frame_packets:
                    continue
                self.run_step(frame_packets)
        finally:
            self.stop()

    def run_step(self, frame_packets: Sequence[FramePacket]) -> Optional[OpenSimStatePacket]:
        pose2d_packets = self._infer_2d(frame_packets)
        multiview_packet = self._associate(pose2d_packets)
        pose3d_packet = self.triangulator.triangulate(multiview_packet)

        if self.pose3d_filter is not None:
            pose3d_packet = self.pose3d_filter.update(pose3d_packet)

        self.marker_buffer.push(pose3d_packet)

        if self.recorder is not None and hasattr(self.recorder, "record_pose3d"):
            self.recorder.record_pose3d(pose3d_packet)

        if self.ik_solver is None or not self.marker_buffer.ready(self.min_window_size):
            return None

        marker_window = self.marker_buffer.latest_window(self.min_window_size)
        state_packet = self.ik_solver.solve_window(marker_window)
        self._fan_out_state(state_packet)
        return state_packet

    def _infer_2d(self, frame_packets: Sequence[FramePacket]) -> List[Pose2DPacket]:
        if hasattr(self.pose2d_estimator, "infer_batch"):
            return list(self.pose2d_estimator.infer_batch(list(frame_packets)))
        return [self.pose2d_estimator.infer(packet) for packet in frame_packets]

    def _associate(self, pose2d_packets: Sequence[Pose2DPacket]) -> MultiViewPosePacket:
        if self.associator is not None:
            return self.associator.associate(list(pose2d_packets))

        if not pose2d_packets:
            raise ValueError("run_step requires at least one Pose2DPacket.")

        first_packet = pose2d_packets[0]
        poses_by_camera = {packet.camera_id: packet for packet in pose2d_packets}
        return MultiViewPosePacket(
            frame_id=first_packet.frame_id,
            timestamp=first_packet.timestamp,
            poses_by_camera=poses_by_camera,
            person_id=first_packet.person_id,
        )

    def _fan_out_state(self, state_packet: OpenSimStatePacket) -> None:
        if self.recorder is not None and hasattr(self.recorder, "record_state"):
            self.recorder.record_state(state_packet)
        if self.publisher is not None and hasattr(self.publisher, "publish"):
            self.publisher.publish(state_packet)
        if self.visualizer is not None and hasattr(self.visualizer, "show"):
            self.visualizer.show(state_packet)
