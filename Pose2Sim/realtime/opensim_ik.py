#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime OpenSim IK solver based on rolling marker windows.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Sequence

import numpy as np
import opensim as osim

from Pose2Sim.realtime.packets import MarkerWindow, OpenSimStatePacket


def _normalize_pose_model_name(pose_model: str) -> str:
    pose_model = str(pose_model).upper()
    if pose_model == "BODY_WITH_FEET":
        return "HALPE_26"
    if pose_model == "WHOLE_BODY_WRIST":
        return "COCO_133_WRIST"
    if pose_model == "WHOLE_BODY":
        return "COCO_133"
    if pose_model == "BODY":
        return "COCO_17"
    if pose_model == "HAND":
        return "HAND_21"
    if pose_model == "FACE":
        return "FACE_106"
    if pose_model == "ANIMAL":
        return "ANIMAL2D_17"
    return pose_model


def _apply_pose_model_markers(
    model: osim.Model,
    pose_model: str,
    osim_setup_dir: Path,
    marker_set_name: str | None = None,
) -> None:
    from Pose2Sim.kinematics import get_markers_path

    resolved_name = marker_set_name or _normalize_pose_model_name(pose_model)
    markers_path = get_markers_path(resolved_name, osim_setup_dir)
    marker_set = osim.MarkerSet(str(markers_path))
    model.set_MarkerSet(marker_set)


def _set_column_labels(table: osim.TimeSeriesTableVec3, labels: Sequence[str]) -> None:
    try:
        vector = osim.StdVectorString()
        for label in labels:
            vector.append(str(label))
        table.setColumnLabels(vector)
    except Exception:
        table.setColumnLabels(list(labels))


def _build_marker_weights(labels: Sequence[str]) -> osim.SetMarkerWeights:
    weights = osim.SetMarkerWeights()
    for label in labels:
        weights.cloneAndAppend(osim.MarkerWeight(str(label), 1.0))
    return weights


def _build_markers_reference(
    table: osim.TimeSeriesTableVec3,
    weights: osim.SetMarkerWeights,
) -> osim.MarkersReference:
    try:
        return osim.MarkersReference(table, weights)
    except Exception:
        return osim.MarkersReference(table, weights, osim.Units(osim.Units.Meters))


def _build_solver(#ik调用
    model: osim.Model,
    markers_ref: osim.MarkersReference,
) -> osim.InverseKinematicsSolver:
    coord_refs = osim.SimTKArrayCoordinateReference()
    try:
        return osim.InverseKinematicsSolver(model, markers_ref, coord_refs)
    except Exception:
        try:
            return osim.InverseKinematicsSolver(model, markers_ref, coord_refs, 1.0)
        except Exception:
            return osim.InverseKinematicsSolver(model, markers_ref, coord_refs, False)


def _fill_marker_gaps(window_markers: np.ndarray, marker_names: Sequence[str]) -> tuple[np.ndarray, tuple[str, ...]]:
    """
    Fill missing samples causally within the current window and drop markers that
    are entirely missing.
    """

    keep_names = []
    filled_markers = []

    for marker_index, marker_name in enumerate(marker_names):
        series = np.asarray(window_markers[:, marker_index, :], dtype=float).copy()
        valid = np.isfinite(series).all(axis=1)
        if not valid.any():
            continue

        first_valid = series[valid][0].copy()
        carry = first_valid.copy()
        for frame_index in range(series.shape[0]):
            if np.isfinite(series[frame_index]).all():
                carry = series[frame_index].copy()
            else:
                series[frame_index] = carry

        keep_names.append(str(marker_name))
        filled_markers.append(series)

    if not filled_markers:
        return np.empty((window_markers.shape[0], 0, 3), dtype=float), tuple()

    return np.stack(filled_markers, axis=1), tuple(keep_names)


class RealtimeIKSolver:
    """
    Rolling-window OpenSim IK solver.
    """

    def __init__(
        self,
        model_path: str,
        pose_model: str,
        osim_setup_dir: Path,
        warm_start: bool = True,
        accuracy: float = 1e-3,
        marker_set_name: str | None = None,
    ):
        self.model_path = str(Path(model_path).resolve())
        self.pose_model = pose_model
        self.osim_setup_dir = Path(osim_setup_dir).resolve()
        self.warm_start = bool(warm_start)
        self.accuracy = float(accuracy)
        self.marker_set_name = marker_set_name

        geometry_dir = self.osim_setup_dir / "Geometry"
        if geometry_dir.exists():
            osim.ModelVisualizer.addDirToGeometrySearchPaths(str(geometry_dir))

        self.model = osim.Model(self.model_path)
        _apply_pose_model_markers(
            self.model,
            self.pose_model,
            self.osim_setup_dir,
            marker_set_name=self.marker_set_name,
        )
        self._model_marker_names = {
            self.model.getMarkerSet().get(index).getName()
            for index in range(self.model.getMarkerSet().getSize())
        }
        self._state = self.model.initSystem()
        self._initialized = False
        logging.info(
            "RealtimeIKSolver ready: model=%s pose_model=%s marker_set=%s marker_count=%d warm_start=%s",
            self.model_path,
            self.pose_model,
            self.marker_set_name or _normalize_pose_model_name(self.pose_model),
            len(self._model_marker_names),
            self.warm_start,
        )

    def solve_window(self, marker_window: MarkerWindow) -> OpenSimStatePacket:
        if marker_window.num_frames == 0:
            raise ValueError("MarkerWindow is empty.")

        filtered_markers, filtered_names = self._prepare_markers(marker_window)
        if len(filtered_names) == 0:
            if not self._initialized:
                raise RuntimeError("No valid markers available to initialize realtime IK.")
            logging.warning(
                "RealtimeIKSolver skipped window %s-%s because no marker columns matched the current model.",
                marker_window.start_frame,
                marker_window.end_frame,
            )
            return self._build_state_packet(marker_window, latency_ms=0.0, metadata={"ik_status": "skipped"})

        if not self.warm_start or not self._initialized:
            self._state = self.model.initSystem()

        window_table = self._build_window_table(marker_window.timestamps, filtered_markers, filtered_names)
        weights = _build_marker_weights(filtered_names)
        markers_ref = _build_markers_reference(window_table, weights)
        solver = _build_solver(self.model, markers_ref)
        if hasattr(solver, "setAccuracy"):
            solver.setAccuracy(self.accuracy)
        if hasattr(solver, "setAdvanceTimeFromReference"):
            solver.setAdvanceTimeFromReference(False)

        solve_start = time.perf_counter()
        for frame_index, timestamp in enumerate(marker_window.timestamps):
            self._state.setTime(float(timestamp))
            if frame_index == 0:
                solver.assemble(self._state)
            else:
                solver.track(self._state)
        latency_ms = (time.perf_counter() - solve_start) * 1000.0
        self._initialized = True

        metadata = {
            "ik_status": "solved",
            "num_markers_in_use": int(solver.getNumMarkersInUse()),
            "marker_columns": filtered_names,
            "marker_set_name": self.marker_set_name or _normalize_pose_model_name(self.pose_model),
        }
        if filtered_names and hasattr(solver, "computeCurrentMarkerError"):
            try:
                metadata["marker_error"] = float(solver.computeCurrentMarkerError(filtered_names[0]))
            except Exception:
                pass

        logging.info(
            "RealtimeIKSolver solved window %s-%s in %.2f ms with %d markers.",
            marker_window.start_frame,
            marker_window.end_frame,
            latency_ms,
            metadata["num_markers_in_use"],
        )

        return self._build_state_packet(marker_window, latency_ms=latency_ms, metadata=metadata)

    def _prepare_markers(self, marker_window: MarkerWindow) -> tuple[np.ndarray, tuple[str, ...]]:
        marker_indices = [
            index
            for index, name in enumerate(marker_window.marker_names)
            if str(name) in self._model_marker_names
        ]
        if not marker_indices:
            return np.empty((marker_window.num_frames, 0, 3), dtype=float), tuple()

        selected_markers = np.asarray(marker_window.markers_3d[:, marker_indices, :], dtype=float)
        selected_names = tuple(str(marker_window.marker_names[index]) for index in marker_indices)
        return _fill_marker_gaps(selected_markers, selected_names)

    def _build_window_table(
        self,
        timestamps: Sequence[float],
        markers_3d: np.ndarray,
        marker_names: Sequence[str],
    ) -> osim.TimeSeriesTableVec3:
        table = osim.TimeSeriesTableVec3()
        _set_column_labels(table, marker_names)

        for frame_index, timestamp in enumerate(timestamps):
            row = osim.RowVectorVec3(len(marker_names))
            for marker_index in range(len(marker_names)):
                x, y, z = markers_3d[frame_index, marker_index]
                row[marker_index] = osim.Vec3(float(x), float(y), float(z))
            table.appendRow(float(timestamp), row)

        return table

    def _build_state_packet(
        self,
        marker_window: MarkerWindow,
        latency_ms: float,
        metadata: dict,
    ) -> OpenSimStatePacket:
        coordinate_values = {}
        coordinate_set = self.model.getCoordinateSet()
        for coord_index in range(coordinate_set.getSize()):
            coordinate = coordinate_set.get(coord_index)
            coordinate_values[coordinate.getName()] = float(coordinate.getValue(self._state))

        return OpenSimStatePacket(
            frame_id=marker_window.end_frame,
            timestamp=float(marker_window.timestamps[-1]),
            coordinate_values=coordinate_values,
            source_window=(marker_window.start_frame, marker_window.end_frame),
            state=None,
            latency_ms=float(latency_ms),
            metadata=metadata,
        )
