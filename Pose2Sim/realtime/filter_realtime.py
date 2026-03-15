#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Causal 3D marker filters for the realtime pipeline.
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
    No-op filter used for debugging and bring-up.
    """

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        return pose3d


class _MarkerKalmanState:
    def __init__(self, frame_rate: float, measurement_noise: float, process_noise: float):
        from filterpy.common import Q_discrete_white_noise
        from filterpy.kalman import KalmanFilter

        self.frame_rate = float(frame_rate)
        self.measurement_noise = float(measurement_noise)
        self.process_noise = float(process_noise)
        self._KalmanFilter = KalmanFilter
        self._Q_discrete_white_noise = Q_discrete_white_noise
        self.filter = self._create_filter()
        self.initialized = False

    def _create_filter(self):
        dt = 1.0 / self.frame_rate
        kalman = self._KalmanFilter(dim_x=9, dim_z=3)
        kalman.F = np.array(
            [
                [1.0, dt, (dt**2) / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, dt, (dt**2) / 2.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt, (dt**2) / 2.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        kalman.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        kalman.P *= self.measurement_noise
        kalman.R = np.diag([self.measurement_noise**2] * 3)
        kalman.Q = self._Q_discrete_white_noise(3, dt=dt, var=self.process_noise**2, block_size=3)
        return kalman

    def update(self, measurement: np.ndarray) -> np.ndarray:
        measurement = np.asarray(measurement, dtype=float)
        if measurement.shape != (3,):
            raise ValueError("Kalman measurement must be a 3D vector.")

        finite_measurement = np.isfinite(measurement).all()
        if not self.initialized and finite_measurement:
            self.filter.x = np.array(
                [measurement[0], 0.0, 0.0, measurement[1], 0.0, 0.0, measurement[2], 0.0, 0.0],
                dtype=float,
            )
            self.initialized = True
            return measurement

        if not self.initialized:
            return np.full(3, np.nan, dtype=float)

        self.filter.predict()
        if finite_measurement:
            self.filter.update(measurement)
        return np.array([self.filter.x[0], self.filter.x[3], self.filter.x[6]], dtype=float)


class RealtimeKalmanFilter:
    """
    Causal per-marker Kalman filter for realtime 3D data.
    """

    def __init__(self, frame_rate: float, trust_ratio: float = 500.0):
        if frame_rate <= 0:
            raise ValueError("RealtimeKalmanFilter requires a positive frame rate.")
        self.frame_rate = float(frame_rate)
        self.trust_ratio = float(trust_ratio)
        self.measurement_noise = 20.0
        self.process_noise = self.measurement_noise * self.trust_ratio
        self._marker_states: list[_MarkerKalmanState] = []
        self._marker_names: Optional[tuple[str, ...]] = None

    def reset(self) -> None:
        self._marker_states = []
        self._marker_names = None

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        marker_names = tuple(pose3d.marker_names)
        markers = np.asarray(pose3d.markers_3d, dtype=float)
        if markers.ndim != 2 or markers.shape[1] != 3:
            raise ValueError("Pose3DPacket.markers_3d must have shape (n_markers, 3).")

        if self._marker_names is None:
            self._marker_names = marker_names
            self._marker_states = [
                _MarkerKalmanState(self.frame_rate, self.measurement_noise, self.process_noise)
                for _ in marker_names
            ]
        elif self._marker_names != marker_names:
            raise ValueError(
                f"RealtimeKalmanFilter marker layout changed from {self._marker_names} to {marker_names}."
            )

        filtered = np.vstack(
            [state.update(markers[idx]) for idx, state in enumerate(self._marker_states)]
        ).astype(float)

        return Pose3DPacket(
            frame_id=pose3d.frame_id,
            timestamp=pose3d.timestamp,
            marker_names=marker_names,
            markers_3d=filtered,
            reprojection_error=pose3d.reprojection_error,
            source_camera_ids=tuple(pose3d.source_camera_ids),
            metadata=dict(pose3d.metadata),
        )


__all__ = ["RealtimePoseFilter", "PassThroughFilter", "RealtimeKalmanFilter"]
