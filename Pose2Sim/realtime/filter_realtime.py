#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Causal 3D marker filters for the realtime pipeline.
"""

from __future__ import annotations

from collections import deque
from typing import Optional, Protocol, runtime_checkable

import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy import signal

from Pose2Sim.realtime.packets import Pose3DPacket


@runtime_checkable
class RealtimePoseFilter(Protocol):
    """
    Minimal contract for a causal 3D marker filter.
    """

    def update(self, pose3d: Pose3DPacket) -> Optional[Pose3DPacket]:
        """
        Return a filtered version of the latest 3D markers, or *None* if the
        filter is still accumulating its internal buffer (windowed filters).
        """


class PassThroughFilter:
    """
    No-op filter used for debugging and bring-up.
    """

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        return pose3d


class _MarkerKalmanState:
    def __init__(self, frame_rate: float, measurement_noise: float, process_noise: float):
        self.frame_rate = float(frame_rate)
        self.measurement_noise = float(measurement_noise)
        self.process_noise = float(process_noise)
        self.filter = self._create_filter()
        self.initialized = False

    def _create_filter(self):
        dt = 1.0 / self.frame_rate
        kalman = KalmanFilter(dim_x=9, dim_z=3)
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
        kalman.Q = Q_discrete_white_noise(3, dt=dt, var=self.process_noise**2, block_size=3)
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


# ---------------------------------------------------------------------------
# One-Euro filter
# ---------------------------------------------------------------------------


class _OneEuroAxis:
    """
    Single-axis causal One-Euro filter state.

    The One-Euro filter adaptively tunes its cutoff frequency based on signal
    velocity: slow movements are smoothed aggressively while fast movements
    pass through with minimal lag.
    """

    def __init__(self, dt: float, min_cutoff: float, beta: float, d_cutoff: float):
        self.dt = float(dt)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x_prev: Optional[float] = None
        self._dx_prev: float = 0.0

    @staticmethod
    def _smoothing_factor(dt: float, cutoff: float) -> float:
        r = 2.0 * np.pi * cutoff * dt
        return r / (r + 1.0)

    def update(self, x: float) -> float:
        if not np.isfinite(x):
            return self._x_prev if self._x_prev is not None else np.nan

        if self._x_prev is None:
            self._x_prev = x
            self._dx_prev = 0.0
            return x

        a_d = self._smoothing_factor(self.dt, self.d_cutoff)
        dx = (x - self._x_prev) / self.dt
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        alpha = self._smoothing_factor(self.dt, cutoff)
        x_hat = alpha * x + (1.0 - alpha) * self._x_prev

        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat


class RealtimeOneEuroFilter:
    """
    Causal per-marker One-Euro filter for realtime 3D data.

    Each marker axis (x, y, z) is filtered independently with an adaptive
    cutoff that increases with signal velocity, providing a good trade-off
    between smoothness during slow motions and responsiveness during fast ones.
    """

    def __init__(
        self,
        frame_rate: float,
        min_cutoff: float = 4.0,
        beta: float = 1.5,
        d_cutoff: float = 1.0,
    ):
        if frame_rate <= 0:
            raise ValueError("RealtimeOneEuroFilter requires a positive frame rate.")
        self.frame_rate = float(frame_rate)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._axes: list[list[_OneEuroAxis]] = []
        self._marker_names: Optional[tuple[str, ...]] = None

    def reset(self) -> None:
        self._axes = []
        self._marker_names = None

    def _init_axes(self, marker_names: tuple[str, ...]) -> None:
        dt = 1.0 / self.frame_rate
        self._marker_names = marker_names
        self._axes = [
            [_OneEuroAxis(dt, self.min_cutoff, self.beta, self.d_cutoff) for _ in range(3)]
            for _ in marker_names
        ]

    def update(self, pose3d: Pose3DPacket) -> Pose3DPacket:
        marker_names = tuple(pose3d.marker_names)
        markers = np.asarray(pose3d.markers_3d, dtype=float)
        if markers.ndim != 2 or markers.shape[1] != 3:
            raise ValueError("Pose3DPacket.markers_3d must have shape (n_markers, 3).")

        if self._marker_names is None:
            self._init_axes(marker_names)
        elif self._marker_names != marker_names:
            raise ValueError(
                f"RealtimeOneEuroFilter marker layout changed from {self._marker_names} to {marker_names}."
            )

        filtered = np.empty_like(markers)
        for marker_idx, axis_group in enumerate(self._axes):
            for axis_idx, axis_filter in enumerate(axis_group):
                filtered[marker_idx, axis_idx] = axis_filter.update(markers[marker_idx, axis_idx])

        return Pose3DPacket(
            frame_id=pose3d.frame_id,
            timestamp=pose3d.timestamp,
            marker_names=marker_names,
            markers_3d=filtered,
            reprojection_error=pose3d.reprojection_error,
            source_camera_ids=tuple(pose3d.source_camera_ids),
            metadata=dict(pose3d.metadata),
        )


# ---------------------------------------------------------------------------
# Windowed bidirectional Butterworth filter
# ---------------------------------------------------------------------------


class RealtimeButterworthWindowFilter:
    """
    Sliding-window zero-phase Butterworth filter for realtime 3D data.

    Accumulates frames in an internal ring buffer.  Once the buffer is full,
    ``scipy.signal.filtfilt`` is applied across the window for each marker
    axis and the **middle frame** is emitted, approximating the offline
    bidirectional Butterworth without requiring the full sequence.

    Introduces a fixed latency of ``window_size // 2`` frames.  Before the
    buffer fills, input frames are passed through unmodified.
    """

    def __init__(
        self,
        frame_rate: float,
        order: int = 4,
        cutoff: float = 6.0,
        window_size: int = 30,
    ):
        if frame_rate <= 0:
            raise ValueError("RealtimeButterworthWindowFilter requires a positive frame rate.")
        if window_size < 6:
            raise ValueError("RealtimeButterworthWindowFilter requires window_size >= 6.")
        self.frame_rate = float(frame_rate)
        self.order = int(order)
        self.cutoff = float(cutoff)
        self.window_size = int(window_size)

        nyquist = self.frame_rate / 2.0
        normalised_cutoff = min(self.cutoff / nyquist, 0.99)
        self._b, self._a = signal.butter(self.order // 2, normalised_cutoff, btype="low")
        self._padlen = 3 * max(len(self._a), len(self._b))
        self._buffer: deque[Pose3DPacket] = deque(maxlen=self.window_size)

    def reset(self) -> None:
        self._buffer.clear()

    def update(self, pose3d: Pose3DPacket) -> Optional[Pose3DPacket]:
        self._buffer.append(pose3d)

        if len(self._buffer) < self.window_size:
            return None

        packets = tuple(self._buffer)
        data = np.stack([np.asarray(p.markers_3d, dtype=float) for p in packets], axis=0)
        n_frames, n_markers, _ = data.shape
        filtered = data.copy()

        for m_idx in range(n_markers):
            for ax in range(3):
                col = data[:, m_idx, ax]
                if np.isfinite(col).all() and n_frames > self._padlen:
                    filtered[:, m_idx, ax] = signal.filtfilt(self._b, self._a, col)

        mid = self.window_size // 2
        mid_packet = packets[mid]
        return Pose3DPacket(
            frame_id=pose3d.frame_id,
            timestamp=pose3d.timestamp,
            marker_names=tuple(mid_packet.marker_names),
            markers_3d=filtered[mid],
            reprojection_error=mid_packet.reprojection_error,
            source_camera_ids=tuple(mid_packet.source_camera_ids),
            metadata=dict(mid_packet.metadata),
        )


# ---------------------------------------------------------------------------
# Windowed Kalman RTS smoother
# ---------------------------------------------------------------------------


def _make_kalman_model(frame_rate: float, measurement_noise: float, process_noise: float) -> KalmanFilter:
    """
    Build a constant-acceleration Kalman model for a single 3D marker.

    Shared by :class:`_MarkerKalmanState` (causal) and
    :class:`RealtimeKalmanRTSWindowFilter` (bidirectional).
    """
    dt = 1.0 / frame_rate
    kf = KalmanFilter(dim_x=9, dim_z=3)
    kf.F = np.array(
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
    kf.H = np.array(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    kf.P *= measurement_noise
    kf.R = np.diag([measurement_noise**2] * 3)
    kf.Q = Q_discrete_white_noise(3, dt=dt, var=process_noise**2, block_size=3)
    return kf


class RealtimeKalmanRTSWindowFilter:
    """
    Sliding-window Kalman forward-backward (RTS) smoother for realtime 3D data.

    For each marker, a fresh Kalman filter is run **forward** over the window
    (predict + update), then the Rauch-Tung-Striebel backward pass refines all
    estimates.  The **middle frame** of the smoothed window is emitted.

    Compared to the causal :class:`RealtimeKalmanFilter`, every frame benefits
    from both past and future observations within the window — the same
    algorithm the offline pipeline uses with ``smooth = true``, but limited to
    a sliding window for bounded latency.

    Missing (NaN) measurements are handled naturally: the forward pass simply
    predicts without updating, and the backward pass still propagates
    information through those gaps.
    """

    def __init__(
        self,
        frame_rate: float,
        trust_ratio: float = 5.0,
        window_size: int = 20,
    ):
        if frame_rate <= 0:
            raise ValueError("RealtimeKalmanRTSWindowFilter requires a positive frame rate.")
        if window_size < 4:
            raise ValueError("RealtimeKalmanRTSWindowFilter requires window_size >= 4.")
        self.frame_rate = float(frame_rate)
        self.trust_ratio = float(trust_ratio)
        self.window_size = int(window_size)
        self.measurement_noise = 20.0
        self.process_noise = self.measurement_noise * self.trust_ratio
        self._buffer: deque[Pose3DPacket] = deque(maxlen=self.window_size)

    def reset(self) -> None:
        self._buffer.clear()

    def _smooth_marker(self, measurements: np.ndarray) -> np.ndarray:
        """
        Run forward Kalman + backward RTS on shape ``(window_size, 3)`` and
        return the smoothed 3D position at the middle frame.
        """
        n = measurements.shape[0]
        kf = _make_kalman_model(self.frame_rate, self.measurement_noise, self.process_noise)

        means = np.zeros((n, 9), dtype=float)
        covariances = np.zeros((n, 9, 9), dtype=float)

        # Initialise state with first finite measurement.
        initialized = False
        for z in measurements:
            if np.isfinite(z).all():
                kf.x = np.array(
                    [z[0], 0.0, 0.0, z[1], 0.0, 0.0, z[2], 0.0, 0.0], dtype=float,
                )
                initialized = True
                break

        if not initialized:
            return np.full(3, np.nan, dtype=float)

        # Forward pass.
        for i, z in enumerate(measurements):
            kf.predict()
            if np.isfinite(z).all():
                kf.update(z)
            means[i] = kf.x.flatten()
            covariances[i] = kf.P.copy()

        # Backward RTS smoothing.
        smoothed_means, _, _, _ = kf.rts_smoother(means, covariances)
        mid = n // 2
        s = smoothed_means[mid]
        return np.array([s[0], s[3], s[6]], dtype=float)

    def update(self, pose3d: Pose3DPacket) -> Optional[Pose3DPacket]:
        self._buffer.append(pose3d)

        if len(self._buffer) < self.window_size:
            return None

        packets = tuple(self._buffer)
        data = np.stack([np.asarray(p.markers_3d, dtype=float) for p in packets], axis=0)
        n_markers = data.shape[1]

        mid = self.window_size // 2
        mid_packet = packets[mid]
        filtered_mid = np.empty((n_markers, 3), dtype=float)
        for m_idx in range(n_markers):
            filtered_mid[m_idx] = self._smooth_marker(data[:, m_idx, :])

        return Pose3DPacket(
            frame_id=pose3d.frame_id,
            timestamp=pose3d.timestamp,
            marker_names=tuple(mid_packet.marker_names),
            markers_3d=filtered_mid,
            reprojection_error=mid_packet.reprojection_error,
            source_camera_ids=tuple(mid_packet.source_camera_ids),
            metadata=dict(mid_packet.metadata),
        )


__all__ = [
    "RealtimePoseFilter",
    "PassThroughFilter",
    "RealtimeButterworthWindowFilter",
    "RealtimeKalmanFilter",
    "RealtimeKalmanRTSWindowFilter",
    "RealtimeOneEuroFilter",
]
