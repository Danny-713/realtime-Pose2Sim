#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Offline-like quality helpers for the realtime pipeline.

The goal is not to fully reproduce Pose2Sim's offline whole-sequence
processing, but to bring the realtime path closer to the same logic:

1. mask low-confidence 2D observations before triangulation
2. gate unreliable 3D markers after triangulation
3. interpolate short NaN gaps inside a fixed-lag window
4. apply Hampel-style center-sample replacement
5. emit the cleaned center frame downstream
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Sequence

import numpy as np

from Pose2Sim.realtime.packets import MultiViewPosePacket, Pose2DPacket, Pose3DPacket


SUPPORTED_POSE_MODELS = {"BODY_WITH_FEET", "HALPE_26"}
SUPPORTED_OUTPUT_MODES = {"center"}
SUPPORTED_TARGETS = {"all_markers"}
SUPPORTED_FILL_MODES = {"last_value", "nan", "zeros"}


def _normalize_pose_model_name(pose_model: str) -> str:
    pose_model = str(pose_model).upper()
    if pose_model == "BODY_WITH_FEET":
        return "HALPE_26"
    return pose_model


class RealtimePose2DQualityMask:
    """
    Apply offline-like likelihood masking on multi-view 2D packets.
    """

    def __init__(self, *, likelihood_threshold: float):
        self.likelihood_threshold = float(likelihood_threshold)

    def apply(self, packet: MultiViewPosePacket) -> MultiViewPosePacket:
        poses_by_camera: dict[str, Pose2DPacket] = {}
        masked_points = 0

        for camera_id, pose_packet in packet.poses_by_camera.items():
            if pose_packet.scores is None:
                poses_by_camera[camera_id] = pose_packet
                continue

            keypoints = np.asarray(pose_packet.keypoints, dtype=float).copy()
            scores = np.asarray(pose_packet.scores, dtype=float).copy()
            invalid = (~np.isfinite(scores)) | (scores < self.likelihood_threshold)
            if np.any(invalid):
                keypoints[invalid, :] = np.nan
                scores[invalid] = np.nan
                masked_points += int(np.sum(invalid))

            poses_by_camera[camera_id] = Pose2DPacket(
                frame_id=pose_packet.frame_id,
                timestamp=pose_packet.timestamp,
                camera_id=pose_packet.camera_id,
                keypoints=keypoints,
                scores=scores,
                person_id=pose_packet.person_id,
                metadata=dict(pose_packet.metadata),
            )

        metadata = dict(packet.metadata)
        metadata["quality_mode_masked_2d_points"] = masked_points
        metadata["quality_mode_likelihood_threshold"] = self.likelihood_threshold
        return MultiViewPosePacket(
            frame_id=packet.frame_id,
            timestamp=packet.timestamp,
            poses_by_camera=poses_by_camera,
            person_id=packet.person_id,
            metadata=metadata,
        )


class RealtimeOfflineLikeQualityProcessor:
    """
    Fixed-lag processor that approximates the offline 3D cleanup path.
    """

    def __init__(
        self,
        *,
        pose_model: str,
        raw_marker_names: Sequence[str] | None = None,
        window_size: int = 21,
        min_window_size: int = 21,
        output_mode: str = "center",
        gate_high_reproj_3d: bool = True,
        reproj_error_threshold_triangulation: float = 15.0,
        min_cameras_for_triangulation: int = 2,
        interp_short_gaps: bool = True,
        interp_max_gap: int = 20,
        fill_large_gaps_with: str = "last_value",
        reject_outliers: bool = True,
        hampel_window_size: int = 7,
        hampel_n_sigma: float = 2.0,
        target: str = "all_markers",
        parameter_source: str = "offline_defaults",
    ):
        normalized_pose_model = _normalize_pose_model_name(pose_model)
        if normalized_pose_model not in SUPPORTED_POSE_MODELS:
            raise ValueError(
                "Realtime offline-like quality mode currently supports only "
                f"Body_with_feet / HALPE_26, got {pose_model!r}."
            )

        self.pose_model = normalized_pose_model
        self.window_size = int(window_size)
        self.min_window_size = int(min_window_size)
        if self.window_size <= 0 or self.min_window_size <= 0:
            raise ValueError("offline-like quality window sizes must be > 0.")
        if self.min_window_size > self.window_size:
            raise ValueError(
                "realtime.offline_like_quality.min_window_size must be <= window_size."
            )

        self.output_mode = str(output_mode).lower()
        if self.output_mode not in SUPPORTED_OUTPUT_MODES:
            raise ValueError(
                "Realtime offline-like quality mode currently only supports output_mode='center'."
            )

        self.gate_high_reproj_3d = bool(gate_high_reproj_3d)
        self.reproj_error_threshold_triangulation = float(reproj_error_threshold_triangulation)
        self.min_cameras_for_triangulation = int(min_cameras_for_triangulation)
        self.interp_short_gaps = bool(interp_short_gaps)
        self.interp_max_gap = int(interp_max_gap)
        self.fill_large_gaps_with = str(fill_large_gaps_with).lower()
        if self.fill_large_gaps_with not in SUPPORTED_FILL_MODES:
            raise ValueError(
                "realtime.offline_like_quality.fill_large_gaps_with must be one of "
                f"{sorted(SUPPORTED_FILL_MODES)}, got {fill_large_gaps_with!r}."
            )
        self.reject_outliers = bool(reject_outliers)
        self.hampel_window_size = int(hampel_window_size)
        self.hampel_n_sigma = float(hampel_n_sigma)
        self.target = str(target).lower()
        if self.target not in SUPPORTED_TARGETS:
            raise ValueError(
                "Realtime offline-like quality mode currently only supports "
                f"target in {sorted(SUPPORTED_TARGETS)}, got {target!r}."
            )
        self.parameter_source = str(parameter_source)

        self._packets: Deque[Pose3DPacket] = deque(maxlen=self.window_size)
        self._raw_marker_names: Optional[tuple[str, ...]] = tuple(raw_marker_names) if raw_marker_names else None
        self._last_clean_values: dict[str, np.ndarray] = {}

    def update(self, pose3d: Pose3DPacket) -> Optional[Pose3DPacket]:
        marker_names = tuple(pose3d.marker_names)
        markers = np.asarray(pose3d.markers_3d, dtype=float)
        if markers.ndim != 2 or markers.shape[1] != 3:
            raise ValueError("Pose3DPacket.markers_3d must have shape (n_markers, 3).")
        if len(marker_names) != markers.shape[0]:
            raise ValueError("Pose3DPacket marker_names length must match markers_3d row count.")

        if self._raw_marker_names is None:
            self._raw_marker_names = marker_names
        elif marker_names != self._raw_marker_names:
            raise ValueError(
                "RealtimeOfflineLikeQualityProcessor marker layout changed from "
                f"{self._raw_marker_names} to {marker_names}."
            )

        self._packets.append(pose3d)
        if len(self._packets) < self._required_size():
            return None

        packets = tuple(self._packets)[-self.window_size :]
        center_index = len(packets) // 2
        window_data = np.stack([np.asarray(packet.markers_3d, dtype=float) for packet in packets], axis=0)

        gated_data, gated_markers = self._apply_quality_gate(window_data, packets, center_index)
        interpolated_data, interpolated_markers = self._interpolate_short_gaps(
            gated_data,
            center_index=center_index,
        )
        filled_data, filled_markers = self._fill_large_gaps(
            interpolated_data,
            marker_names=marker_names,
            center_index=center_index,
        )
        cleaned_data, outlier_replaced_markers = self._replace_center_outliers(
            filled_data,
            center_index=center_index,
        )

        center_packet = packets[center_index]
        cleaned_markers = np.asarray(cleaned_data[center_index], dtype=float)
        self._remember_clean_values(marker_names, cleaned_markers)

        metadata = dict(center_packet.metadata)
        metadata.update(
            {
                "quality_mode_enabled": True,
                "quality_mode_window_size": len(packets),
                "quality_mode_output_mode": self.output_mode,
                "quality_mode_parameter_source": self.parameter_source,
                "quality_mode_gated_markers": gated_markers,
                "quality_mode_interpolated_markers": interpolated_markers,
                "quality_mode_filled_markers": filled_markers,
                "quality_mode_outlier_replaced_markers": outlier_replaced_markers,
            }
        )

        return Pose3DPacket(
            frame_id=center_packet.frame_id,
            timestamp=center_packet.timestamp,
            marker_names=marker_names,
            markers_3d=cleaned_markers,
            reprojection_error=center_packet.reprojection_error,
            source_camera_ids=center_packet.source_camera_ids,
            metadata=metadata,
        )

    def _required_size(self) -> int:
        return self.min_window_size

    def _apply_quality_gate(
        self,
        window_data: np.ndarray,
        packets: Sequence[Pose3DPacket],
        center_index: int,
    ) -> tuple[np.ndarray, int]:
        gated = np.array(window_data, dtype=float, copy=True)
        if not self.gate_high_reproj_3d:
            return gated, 0

        marker_count = gated.shape[1]
        for frame_index, packet in enumerate(packets):
            reproj_errors = self._coerce_float_list(
                packet.metadata.get("per_marker_reprojection_error"),
                marker_count,
                fill_value=np.nan,
            )
            excluded_cameras = self._coerce_sequence_list(
                packet.metadata.get("per_marker_excluded_cameras"),
                marker_count,
            )
            valid_after = self._coerce_bool_list(
                packet.metadata.get("per_marker_valid_after_triangulation"),
                marker_count,
                fill_value=True,
            )
            total_cameras = len(packet.source_camera_ids)

            for marker_index in range(marker_count):
                usable_cameras = max(total_cameras - len(excluded_cameras[marker_index]), 0)
                should_mask = (
                    (not valid_after[marker_index])
                    or (not np.isfinite(reproj_errors[marker_index]))
                    or (reproj_errors[marker_index] > self.reproj_error_threshold_triangulation)
                    or (usable_cameras < self.min_cameras_for_triangulation)
                )
                if should_mask:
                    gated[frame_index, marker_index, :] = np.nan

        center_before = np.asarray(window_data[center_index], dtype=float)
        center_after = np.asarray(gated[center_index], dtype=float)
        gated_markers = 0
        for marker_index in range(center_before.shape[0]):
            if np.allclose(center_before[marker_index], center_after[marker_index], equal_nan=True):
                continue
            if np.isfinite(center_before[marker_index]).any():
                gated_markers += 1
        return gated, gated_markers

    def _interpolate_short_gaps(
        self,
        window_data: np.ndarray,
        *,
        center_index: int,
    ) -> tuple[np.ndarray, int]:
        if not self.interp_short_gaps:
            return np.array(window_data, dtype=float, copy=True), 0

        interpolated = np.array(window_data, dtype=float, copy=True)
        marker_count = interpolated.shape[1]
        for marker_index in range(marker_count):
            for axis in range(3):
                interpolated[:, marker_index, axis] = self._interpolate_short_gaps_1d(
                    interpolated[:, marker_index, axis],
                    max_gap=self.interp_max_gap,
                )

        center_before = np.asarray(window_data[center_index], dtype=float)
        center_after = np.asarray(interpolated[center_index], dtype=float)
        interpolated_markers = 0
        for marker_index in range(center_before.shape[0]):
            became_finite = np.any(~np.isfinite(center_before[marker_index]) & np.isfinite(center_after[marker_index]))
            if became_finite:
                interpolated_markers += 1
        return interpolated, interpolated_markers

    def _fill_large_gaps(
        self,
        window_data: np.ndarray,
        *,
        marker_names: Sequence[str],
        center_index: int,
    ) -> tuple[np.ndarray, int]:
        filled = np.array(window_data, dtype=float, copy=True)
        center_values = filled[center_index]
        filled_markers = 0

        for marker_index, marker_name in enumerate(marker_names):
            marker_value = np.asarray(center_values[marker_index], dtype=float)
            if np.isfinite(marker_value).all():
                continue

            repaired = marker_value.copy()
            if self.fill_large_gaps_with == "last_value":
                previous = self._last_clean_values.get(str(marker_name))
                if previous is not None:
                    invalid_mask = ~np.isfinite(repaired) & np.isfinite(previous)
                    if np.any(invalid_mask):
                        repaired[invalid_mask] = previous[invalid_mask]
            elif self.fill_large_gaps_with == "zeros":
                repaired[~np.isfinite(repaired)] = 0.0

            if not np.allclose(marker_value, repaired, equal_nan=True):
                center_values[marker_index] = repaired
                filled_markers += 1

        filled[center_index] = center_values
        return filled, filled_markers

    def _replace_center_outliers(
        self,
        window_data: np.ndarray,
        *,
        center_index: int,
    ) -> tuple[np.ndarray, int]:
        if not self.reject_outliers:
            return np.array(window_data, dtype=float, copy=True), 0

        cleaned = np.array(window_data, dtype=float, copy=True)
        marker_count = cleaned.shape[1]
        replaced_markers = 0

        for marker_index in range(marker_count):
            replaced = False
            for axis in range(3):
                replacement = self._hampel_center_value(
                    cleaned[:, marker_index, axis],
                    center_index=center_index,
                )
                if replacement is None:
                    continue
                if not np.isfinite(cleaned[center_index, marker_index, axis]):
                    continue
                cleaned[center_index, marker_index, axis] = replacement
                replaced = True
            if replaced:
                replaced_markers += 1

        return cleaned, replaced_markers

    def _hampel_center_value(self, values: np.ndarray, *, center_index: int) -> Optional[float]:
        values = np.asarray(values, dtype=float)
        if center_index >= len(values) or not np.isfinite(values[center_index]):
            return None

        half_window = max(self.hampel_window_size // 2, 1)
        start = max(center_index - half_window, 0)
        end = min(center_index + half_window + 1, len(values))
        window = values[start:end]
        window = window[np.isfinite(window)]
        if len(window) < 3:
            return None

        median = float(np.median(window))
        mad = float(np.median(np.abs(window - median)))
        if mad == 0.0:
            return None

        modified_z_score = 0.6745 * (float(values[center_index]) - median) / mad
        if abs(modified_z_score) > self.hampel_n_sigma:
            return median
        return None

    @staticmethod
    def _interpolate_short_gaps_1d(values: np.ndarray, *, max_gap: int) -> np.ndarray:
        values = np.asarray(values, dtype=float).copy()
        if len(values) == 0:
            return values

        index = 0
        while index < len(values):
            if np.isfinite(values[index]):
                index += 1
                continue

            gap_start = index
            while index < len(values) and not np.isfinite(values[index]):
                index += 1
            gap_end = index - 1
            gap_len = gap_end - gap_start + 1
            left = gap_start - 1
            right = index
            if (
                gap_len <= max_gap
                and left >= 0
                and right < len(values)
                and np.isfinite(values[left])
                and np.isfinite(values[right])
            ):
                interpolated = np.linspace(values[left], values[right], gap_len + 2)[1:-1]
                values[gap_start:right] = interpolated

        return values

    def _remember_clean_values(self, marker_names: Sequence[str], cleaned_markers: np.ndarray) -> None:
        for marker_name, marker_value in zip(marker_names, cleaned_markers):
            marker_value = np.asarray(marker_value, dtype=float)
            if np.isfinite(marker_value).any():
                self._last_clean_values[str(marker_name)] = marker_value.copy()

    @staticmethod
    def _coerce_float_list(values, expected_length: int, *, fill_value: float) -> list[float]:
        if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
            return [fill_value] * expected_length
        coerced = []
        for item in list(values)[:expected_length]:
            try:
                coerced.append(float(item))
            except (TypeError, ValueError):
                coerced.append(fill_value)
        if len(coerced) < expected_length:
            coerced.extend([fill_value] * (expected_length - len(coerced)))
        return coerced

    @staticmethod
    def _coerce_bool_list(values, expected_length: int, *, fill_value: bool) -> list[bool]:
        if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
            return [fill_value] * expected_length
        coerced = [bool(item) for item in list(values)[:expected_length]]
        if len(coerced) < expected_length:
            coerced.extend([fill_value] * (expected_length - len(coerced)))
        return coerced

    @staticmethod
    def _coerce_sequence_list(values, expected_length: int) -> list[tuple]:
        if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
            return [tuple() for _ in range(expected_length)]
        coerced = []
        for item in list(values)[:expected_length]:
            if isinstance(item, Sequence) and not isinstance(item, (str, bytes)):
                coerced.append(tuple(item))
            else:
                coerced.append(tuple())
        if len(coerced) < expected_length:
            coerced.extend([tuple() for _ in range(expected_length - len(coerced))])
        return coerced


__all__ = [
    "RealtimeOfflineLikeQualityProcessor",
    "RealtimePose2DQualityMask",
]
