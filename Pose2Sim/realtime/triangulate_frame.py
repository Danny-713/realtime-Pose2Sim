#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Per-frame triangulation for the realtime pipeline.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Mapping, Protocol, Sequence, runtime_checkable

import numpy as np

from Pose2Sim.realtime.packets import MultiViewPosePacket, Pose3DPacket


@runtime_checkable
class FrameTriangulator(Protocol):#协议，定义了triangulate方法
    """
    Realtime triangulator contract: one multi-view packet in, one 3D packet out.
    """

    def triangulate(self, packet: MultiViewPosePacket) -> Pose3DPacket:
        """
        Triangulate one synchronized multi-camera observation.
        """


class CallableFrameTriangulator:#适配器，本身什么都没有做，为了适配协议
    """
    Thin adapter around a plain callable.
    """

    def __init__(self, triangulate_fn: Callable[[MultiViewPosePacket], Pose3DPacket]):#类型注释callable，输入：triangulate函数，函数也是类
        self._triangulate_fn = triangulate_fn#把函数存起来，相当于一个属性

    def triangulate(self, packet: MultiViewPosePacket) -> Pose3DPacket:
        return self._triangulate_fn(packet)


class RealtimeFrameTriangulator:
    """
    Single-person per-frame triangulator backed by the offline core math.
    """

    def __init__(
        self,
        config_dict: Mapping[str, object],
        calib_file: str,
        camera_ids: Sequence[str],
        marker_names: Sequence[str],
    ):
        if not calib_file:
            raise ValueError("RealtimeFrameTriangulator requires a calibration file.")

        self.config_dict = config_dict
        self.calib_file = str(Path(calib_file).resolve())
        self.marker_names = tuple(marker_names)

        import toml as _toml

        from Pose2Sim.common import computeP, retrieve_calib_params

        self.calib = _toml.load(self.calib_file)
        self.calib_camera_ids = tuple(
            key
            for key, value in self.calib.items()
            if key not in {"metadata", "capture_volume", "charuco", "checkerboard"} and isinstance(value, dict)
        )
        self.calib_params = retrieve_calib_params(self.calib_file)
        self.projection_matrices = computeP(
            self.calib_file,
            undistort=bool(config_dict.get("triangulation", {}).get("undistort_points", False)),
        )
        self.camera_ids = self._resolve_camera_order(camera_ids)
        self.camera_index_by_id = {camera_id: idx for idx, camera_id in enumerate(self.camera_ids)}
        self.swap_indices = self._build_swap_indices(self.marker_names)
        self.coordinate_transform = self._resolve_coordinate_transform(config_dict)

        from Pose2Sim.triangulation import triangulation_from_best_cameras

        self._triangulation_from_best_cameras = triangulation_from_best_cameras

    @staticmethod
    def _resolve_coordinate_transform(config_dict: Mapping[str, object]) -> str:
        realtime_cfg = config_dict.get("realtime", {})
        if isinstance(realtime_cfg, Mapping):
            triangulation_cfg = realtime_cfg.get("triangulation", {})
            if isinstance(triangulation_cfg, Mapping):
                transform = str(triangulation_cfg.get("coordinate_transform", "zup_to_yup")).lower()
                if transform in {"zup_to_yup", "identity"}:
                    return transform
                raise ValueError(
                    "Unsupported realtime.triangulation.coordinate_transform="
                    f"{transform!r}. Expected 'zup_to_yup' or 'identity'."
                )
        return "zup_to_yup"

    @staticmethod
    def _zup_to_yup(markers_3d: np.ndarray) -> np.ndarray:#坐标轴转换，Z-up to Y-up
        """
        Match the offline TRC export convention before handing markers to OpenSim.

        Pose2Sim's offline path writes TRC files after converting per-marker
        coordinates from Z-up to Y-up by reordering axes as (x, y, z) -> (y, z, x).
        Realtime OpenSim should consume the same convention, otherwise the model
        appears rotated/lying down compared with the validated TRC-based path.
        """

        markers_3d = np.asarray(markers_3d, dtype=float)
        if markers_3d.ndim != 2 or markers_3d.shape[1] != 3:
            raise ValueError(
                f"RealtimeFrameTriangulator expected marker data of shape (n_markers, 3), got {markers_3d.shape}."
            )
        return markers_3d[:, [1, 2, 0]]

    def _apply_coordinate_transform(self, markers_3d: np.ndarray) -> np.ndarray:
        markers_3d = np.asarray(markers_3d, dtype=float)
        if self.coordinate_transform == "identity":
            return markers_3d
        return self._zup_to_yup(markers_3d)

    def _resolve_camera_order(self, runtime_camera_ids: Sequence[str]) -> tuple[str, ...]:#相机顺序解析
        runtime_ids = {str(camera_id).lower(): str(camera_id) for camera_id in runtime_camera_ids}
        resolved = []
        for calib_id in self.calib_camera_ids:
            match = runtime_ids.get(calib_id.lower())
            if match is None:
                raise ValueError(
                    f"Calibration camera '{calib_id}' is missing from realtime capture ids {tuple(runtime_camera_ids)}."
                )
            resolved.append(match)
        return tuple(resolved)

    @staticmethod
    def _build_swap_indices(marker_names: Sequence[str]) -> tuple[int, ...]:#左右手交换索引构建
        index_by_name = {name: idx for idx, name in enumerate(marker_names)}
        swap_indices = []
        for name in marker_names:
            if name.startswith("R") and ("L" + name[1:]) in index_by_name:
                swap_indices.append(index_by_name["L" + name[1:]])
            elif name.startswith("L") and ("R" + name[1:]) in index_by_name:
                swap_indices.append(index_by_name["R" + name[1:]])
            else:
                swap_indices.append(index_by_name[name])
        return tuple(swap_indices)

    def triangulate(self, packet: MultiViewPosePacket) -> Pose3DPacket:#三角化，调用离线版的三角化函数
        marker_positions = []
        reprojection_errors = []
        excluded_camera_counts = []
        per_marker_reprojection_errors = []
        per_marker_excluded_cameras = []
        per_marker_valid_after_triangulation = []

        for marker_index, marker_name in enumerate(self.marker_names):
            coords = self._collect_camera_coords(packet, marker_index)
            swapped_coords = self._collect_camera_coords(packet, self.swap_indices[marker_index])

            x_all = coords[:, 0]
            y_all = coords[:, 1]
            likelihood_all = coords[:, 2]
            x_all_swapped = swapped_coords[:, 0]
            y_all_swapped = swapped_coords[:, 1]
            likelihood_all_swapped = swapped_coords[:, 2]

            q, error_min, nb_cams_excluded, excluded_camera_ids = self._triangulation_from_best_cameras(
                self.config_dict,
                (x_all, y_all, likelihood_all),
                (x_all_swapped, y_all_swapped, likelihood_all_swapped),
                self.projection_matrices,
                self.calib_params,
            )
            q_array = np.asarray(q, dtype=float)
            marker_positions.append(q_array)
            if np.isfinite(error_min):
                reprojection_errors.append(float(error_min))
            excluded_camera_counts.append(int(nb_cams_excluded))
            per_marker_reprojection_errors.append(float(error_min) if np.isfinite(error_min) else np.nan)
            excluded_camera_ids = np.atleast_1d(excluded_camera_ids).tolist()
            excluded_camera_names = []
            for camera_index in excluded_camera_ids:
                try:
                    camera_index_int = int(camera_index)
                except (TypeError, ValueError):
                    continue
                if 0 <= camera_index_int < len(self.camera_ids):
                    excluded_camera_names.append(self.camera_ids[camera_index_int])
            per_marker_excluded_cameras.append(excluded_camera_names)
            per_marker_valid_after_triangulation.append(bool(np.isfinite(q_array).all() and np.isfinite(error_min)))

        marker_array = np.asarray(marker_positions, dtype=float)
        marker_array = self._apply_coordinate_transform(marker_array)
        reprojection_error = float(np.mean(reprojection_errors)) if reprojection_errors else np.nan

        return Pose3DPacket(
            frame_id=packet.frame_id,
            timestamp=packet.timestamp,
            marker_names=self.marker_names,
            markers_3d=marker_array,
            reprojection_error=reprojection_error,
            source_camera_ids=tuple(self.camera_ids),
            metadata={
                "mean_excluded_cameras": float(np.mean(excluded_camera_counts)) if excluded_camera_counts else 0.0,
                "num_valid_markers": int(np.sum(np.isfinite(marker_array[:, 0]))),
                "per_marker_reprojection_error": per_marker_reprojection_errors,
                "per_marker_excluded_cameras": per_marker_excluded_cameras,
                "per_marker_valid_after_triangulation": per_marker_valid_after_triangulation,
            },
        )

    def _collect_camera_coords(self, packet: MultiViewPosePacket, marker_index: int) -> np.ndarray:
        coords = np.full((len(self.camera_ids), 3), np.nan, dtype=float)
        for camera_idx, camera_id in enumerate(self.camera_ids):
            pose_packet = packet.poses_by_camera.get(camera_id)
            if pose_packet is None:
                continue
            if marker_index >= pose_packet.keypoints.shape[0]:
                continue
            coords[camera_idx, 0] = float(pose_packet.keypoints[marker_index, 0])
            coords[camera_idx, 1] = float(pose_packet.keypoints[marker_index, 1])
            if pose_packet.scores is not None and marker_index < pose_packet.scores.shape[0]:
                coords[camera_idx, 2] = float(pose_packet.scores[marker_index])
        return coords


__all__ = ["FrameTriangulator", "CallableFrameTriangulator", "RealtimeFrameTriangulator"]
