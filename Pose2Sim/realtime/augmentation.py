#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime marker augmentation backed by the offline LSTM augmenter assets.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence as SequenceABC
from pathlib import Path
from typing import Deque, Optional, Sequence

import numpy as np
import onnxruntime as ort
import pandas as pd

from Pose2Sim.common import compute_height
from Pose2Sim.realtime.packets import Pose3DPacket


SUPPORTED_POSE_MODELS = {"BODY_WITH_FEET", "HALPE_26"}

LOWER_FEATURE_MARKERS = (
    "Neck",
    "RShoulder",
    "LShoulder",
    "RHip",
    "LHip",
    "RKnee",
    "LKnee",
    "RAnkle",
    "LAnkle",
    "RHeel",
    "LHeel",
    "RSmallToe",
    "LSmallToe",
    "RBigToe",
    "LBigToe",
)
LOWER_RESPONSE_MARKERS = (
    "r.ASIS_study",
    "L.ASIS_study",
    "r.PSIS_study",
    "L.PSIS_study",
    "r_knee_study",
    "r_mknee_study",
    "r_ankle_study",
    "r_mankle_study",
    "r_toe_study",
    "r_5meta_study",
    "r_calc_study",
    "L_knee_study",
    "L_mknee_study",
    "L_ankle_study",
    "L_mankle_study",
    "L_toe_study",
    "L_calc_study",
    "L_5meta_study",
    "r_shoulder_study",
    "L_shoulder_study",
    "C7_study",
    "r_thigh1_study",
    "r_thigh2_study",
    "r_thigh3_study",
    "L_thigh1_study",
    "L_thigh2_study",
    "L_thigh3_study",
    "r_sh1_study",
    "r_sh2_study",
    "r_sh3_study",
    "L_sh1_study",
    "L_sh2_study",
    "L_sh3_study",
    "RHJC_study",
    "LHJC_study",
)
UPPER_FEATURE_MARKERS = (
    "Neck",
    "RShoulder",
    "LShoulder",
    "RElbow",
    "LElbow",
    "RWrist",
    "LWrist",
)
UPPER_RESPONSE_MARKERS = (
    "r_lelbow_study",
    "r_melbow_study",
    "r_lwrist_study",
    "r_mwrist_study",
    "L_lelbow_study",
    "L_melbow_study",
    "L_lwrist_study",
    "L_mwrist_study",
)


def _normalize_pose_model_name(pose_model: str) -> str:
    pose_model = str(pose_model).upper()
    if pose_model == "BODY_WITH_FEET":
        return "HALPE_26"
    return pose_model


class _AugmenterSession:
    def __init__(self, model_dir: Path, feature_markers: Sequence[str], response_markers: Sequence[str]):
        self.model_dir = Path(model_dir).resolve()
        self.feature_markers = tuple(feature_markers)
        self.response_markers = tuple(response_markers)
        self.mean = np.load(self.model_dir / "mean.npy", allow_pickle=True).astype(np.float32)
        self.std = np.load(self.model_dir / "std.npy", allow_pickle=True).astype(np.float32)
        self.session = ort.InferenceSession(str(self.model_dir / "model.onnx"))
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def run(
        self,
        marker_lookup: dict[str, np.ndarray],
        height_m: float,
        mass_kg: float,
    ) -> np.ndarray:
        features = np.concatenate([marker_lookup[name] for name in self.feature_markers], axis=1)
        hip = marker_lookup["Hip"]
        normalized = features - np.tile(hip, (1, len(self.feature_markers)))
        normalized = normalized / float(height_m)
        enriched = np.concatenate(
            (
                normalized,
                np.full((normalized.shape[0], 1), float(height_m), dtype=float),
                np.full((normalized.shape[0], 1), float(mass_kg), dtype=float),
            ),
            axis=1,
        )
        standardized = (enriched - self.mean) / self.std
        inputs = np.reshape(standardized.astype(np.float32), (1, standardized.shape[0], standardized.shape[1]))
        outputs = self.session.run([self.output_name], {self.input_name: inputs})[0]
        outputs = np.reshape(outputs, (outputs.shape[1], outputs.shape[2]))
        unnormalized = outputs * float(height_m)
        return unnormalized + np.tile(hip, (1, outputs.shape[1] // 3))


class RealtimeMarkerAugmenter:
    """
    Fixed-lag realtime wrapper around the offline LSTM marker augmenter.
    """

    def __init__(
        self,
        *,
        config_dict,
        pose_model: str,
        raw_marker_names: Sequence[str] | None = None,
        model_name: str = "LSTM",
        model_version: str = "v0.3",
        window_size: int = 15,
        min_window_size: int = 15,
        output_mode: str = "center",
        feet_on_floor: bool = False,
        use_subject_stats: bool = True,
    ):
        normalized_pose_model = _normalize_pose_model_name(pose_model)
        if normalized_pose_model not in SUPPORTED_POSE_MODELS:
            raise ValueError(
                "Realtime marker augmentation currently supports only Body_with_feet / HALPE_26, "
                f"got {pose_model!r}."
            )
        if str(model_name).upper() != "LSTM":
            raise ValueError(f"Realtime marker augmentation only supports model_name='LSTM', got {model_name!r}.")
        if int(window_size) <= 0:
            raise ValueError("realtime.augmentation.window_size must be a positive integer.")
        if int(min_window_size) <= 0:
            raise ValueError("realtime.augmentation.min_window_size must be a positive integer.")

        self.config_dict = config_dict
        self.pose_model = normalized_pose_model
        self.model_name = str(model_name).upper()
        self.model_version = str(model_version)
        self.window_size = int(window_size)
        self.min_window_size = int(min_window_size)
        self.output_mode = str(output_mode).lower()
        if self.output_mode not in {"center", "latest"}:
            raise ValueError("realtime.augmentation.output_mode must be 'center' or 'latest'.")
        self.feet_on_floor = bool(feet_on_floor)
        self.use_subject_stats = bool(use_subject_stats)
        self._packets: Deque[Pose3DPacket] = deque(maxlen=self.window_size)
        self._raw_marker_names: Optional[tuple[str, ...]] = tuple(raw_marker_names) if raw_marker_names else None
        self._subject_height_m: Optional[float] = None

        augmenter_root = Path(__file__).resolve().parents[1] / "MarkerAugmenter" / self.model_name
        self._lower = _AugmenterSession(
            augmenter_root / f"{self.model_version}_lower",
            LOWER_FEATURE_MARKERS,
            LOWER_RESPONSE_MARKERS,
        )
        self._upper = _AugmenterSession(
            augmenter_root / f"{self.model_version}_upper",
            UPPER_FEATURE_MARKERS,
            UPPER_RESPONSE_MARKERS,
        )
        self.response_marker_names = tuple(self._lower.response_markers + self._upper.response_markers)
        self.output_marker_names: Optional[tuple[str, ...]] = None
        if self._raw_marker_names is not None:
            self.output_marker_names = tuple(self._raw_marker_names + self.response_marker_names)

    def update(self, pose3d: Pose3DPacket) -> Optional[Pose3DPacket]:
        marker_names = tuple(pose3d.marker_names)
        markers = np.asarray(pose3d.markers_3d, dtype=float)
        if markers.ndim != 2 or markers.shape[1] != 3:
            raise ValueError("Pose3DPacket.markers_3d must have shape (n_markers, 3).")
        if len(marker_names) != markers.shape[0]:
            raise ValueError("Pose3DPacket marker_names length must match markers_3d row count.")

        if self._raw_marker_names is None:
            self._raw_marker_names = marker_names
            self.output_marker_names = tuple(self._raw_marker_names + self.response_marker_names)
        elif marker_names != self._raw_marker_names:
            raise ValueError(
                "RealtimeMarkerAugmenter marker layout changed from "
                f"{self._raw_marker_names} to {marker_names}."
            )

        self._packets.append(pose3d)
        if len(self._packets) < self._required_size():
            return None

        window_packets = tuple(self._packets)[-self.window_size :]
        marker_lookup = self._build_marker_lookup(window_packets)
        self._ensure_required_markers(marker_lookup)
        height_m, mass_kg = self._resolve_subject_stats(window_packets)

        lower_output = self._lower.run(marker_lookup, height_m=height_m, mass_kg=mass_kg)
        upper_output = self._upper.run(marker_lookup, height_m=height_m, mass_kg=mass_kg)
        augmented_sequence = np.concatenate((lower_output, upper_output), axis=1)
        response_sequence = augmented_sequence.reshape(augmented_sequence.shape[0], len(self.response_marker_names), 3)

        output_index = self._select_output_index(len(window_packets))
        center_packet = window_packets[output_index]
        merged_markers = np.concatenate(
            (
                np.asarray(center_packet.markers_3d, dtype=float),
                response_sequence[output_index],
            ),
            axis=0,
        )
        if self.feet_on_floor:
            min_y = float(np.nanmin(response_sequence[output_index][:, 1]))
            if np.isfinite(min_y):
                merged_markers[:, 1] = merged_markers[:, 1] - (min_y - 0.01)

        metadata = dict(center_packet.metadata)
        metadata.update(
            {
                "augmentation_enabled": True,
                "augmentation_model": self.model_name,
                "augmentation_model_version": self.model_version,
                "augmentation_output_mode": self.output_mode,
                "augmentation_window_size": self.window_size,
                "augmentation_subject_height_m": float(height_m),
                "augmentation_subject_mass_kg": float(mass_kg),
                "augmentation_added_markers": len(self.response_marker_names),
            }
        )

        return Pose3DPacket(
            frame_id=center_packet.frame_id,
            timestamp=center_packet.timestamp,
            marker_names=self.output_marker_names or marker_names,
            markers_3d=merged_markers,
            reprojection_error=center_packet.reprojection_error,
            source_camera_ids=tuple(center_packet.source_camera_ids),
            metadata=metadata,
        )

    def _required_size(self) -> int:
        if self.output_mode == "center":
            return self.window_size
        return min(self.window_size, self.min_window_size)

    def _select_output_index(self, window_length: int) -> int:
        if self.output_mode == "center":
            return window_length // 2
        return window_length - 1

    def _build_marker_lookup(self, packets: Sequence[Pose3DPacket]) -> dict[str, np.ndarray]:
        marker_names = tuple(packets[0].marker_names)
        stacked = np.stack([np.asarray(packet.markers_3d, dtype=float) for packet in packets], axis=0)
        lookup = {
            name: np.asarray(stacked[:, index, :], dtype=float)
            for index, name in enumerate(marker_names)
        }

        if "Neck" not in lookup and {"RShoulder", "LShoulder"}.issubset(lookup):
            lookup["Neck"] = (lookup["RShoulder"] + lookup["LShoulder"]) / 2.0
        if "Hip" not in lookup and {"RHip", "LHip"}.issubset(lookup):
            lookup["Hip"] = (lookup["RHip"] + lookup["LHip"]) / 2.0
        return lookup

    def _ensure_required_markers(self, marker_lookup: dict[str, np.ndarray]) -> None:
        required = set(LOWER_FEATURE_MARKERS + UPPER_FEATURE_MARKERS)
        missing = sorted(name for name in required if name not in marker_lookup)
        if missing:
            raise ValueError(
                "Realtime marker augmentation requires feature markers "
                f"{missing}, but they are not present in the current HALPE_26 stream."
            )

    def _resolve_subject_stats(self, packets: Sequence[Pose3DPacket]) -> tuple[float, float]:
        project_cfg = self.config_dict.get("project", {})
        kinematics_cfg = self.config_dict.get("kinematics", {})
        participant_mass = project_cfg.get("participant_mass")
        participant_height = project_cfg.get("participant_height")
        default_height = float(kinematics_cfg.get("default_height", 1.70))

        if self.use_subject_stats:
            mass_kg = self._resolve_mass(participant_mass)
            height_m = self._resolve_height(packets, participant_height, default_height, kinematics_cfg)
        else:
            mass_kg = 70.0
            height_m = default_height
        return float(height_m), float(mass_kg)

    @staticmethod
    def _resolve_mass(participant_mass) -> float:
        if isinstance(participant_mass, (int, float)) and float(participant_mass) > 0:
            return float(participant_mass)
        if isinstance(participant_mass, SequenceABC) and not isinstance(participant_mass, (str, bytes)):
            for value in participant_mass:
                if isinstance(value, (int, float)) and float(value) > 0:
                    return float(value)
        return 70.0

    def _resolve_height(self, packets, participant_height, default_height: float, kinematics_cfg) -> float:
        if self._subject_height_m is not None and np.isfinite(self._subject_height_m):
            return float(self._subject_height_m)

        if isinstance(participant_height, (int, float)) and float(participant_height) > 0:
            self._subject_height_m = float(participant_height)
            return self._subject_height_m

        if isinstance(participant_height, SequenceABC) and not isinstance(participant_height, (str, bytes)):
            for value in participant_height:
                if isinstance(value, (int, float)) and float(value) > 0:
                    self._subject_height_m = float(value)
                    return self._subject_height_m

        try:
            keypoint_names = tuple(packets[0].marker_names)
            q_coords = self._window_to_dataframe(packets)
            self._subject_height_m = float(
                compute_height(
                    q_coords,
                    keypoint_names,
                    fastest_frames_to_remove_percent=float(kinematics_cfg.get("fastest_frames_to_remove_percent", 0.1)),
                    close_to_zero_speed=float(kinematics_cfg.get("close_to_zero_speed_m", 50)),
                    large_hip_knee_angles=float(kinematics_cfg.get("large_hip_knee_angles", 45)),
                    trimmed_extrema_percent=float(kinematics_cfg.get("trimmed_extrema_percent", 0.5)),
                )
            )
            if np.isfinite(self._subject_height_m) and self._subject_height_m > 0:
                return self._subject_height_m
        except Exception:
            pass

        self._subject_height_m = float(default_height)
        return self._subject_height_m

    @staticmethod
    def _window_to_dataframe(packets: Sequence[Pose3DPacket]) -> pd.DataFrame:
        marker_names = tuple(packets[0].marker_names)
        stacked = np.stack([np.asarray(packet.markers_3d, dtype=float) for packet in packets], axis=0)
        flattened = np.concatenate([stacked[:, index, :] for index in range(len(marker_names))], axis=1)
        columns = [name for name in marker_names for _ in range(3)]
        return pd.DataFrame(flattened, columns=columns)


__all__ = ["RealtimeMarkerAugmenter"]
