#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Pseudo-realtime rolling-window IK replay with optional OpenSim API Visualizer.

This script is the first "full-chain" prototype for the validated OpenSim
backend:

    TRC replay -> rolling marker window -> MarkersReference -> IK -> state
    -> API Visualizer

It still uses a file-backed TRC for repeatability, but the OpenSim half of the
realtime architecture is the same one we want to use later with an in-memory
marker buffer.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Sequence

import opensim as osim


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay rolling-window IK and drive the OpenSim API Visualizer.")
    parser.add_argument("--model", type=Path, help="Path to an .osim model.")
    parser.add_argument("--trc", type=Path, help="Path to a .trc marker file.")
    parser.add_argument("--window-size", type=int, default=10, help="Number of frames per rolling IK window.")
    parser.add_argument("--max-windows", type=int, default=30, help="How many windows to replay. Use 0 for all.")
    parser.add_argument("--step", type=int, default=1, help="How many frames to slide forward per iteration.")
    parser.add_argument(
        "--replay-fps",
        type=float,
        default=15.0,
        help="Target playback rate for the visualizer. Use 0 to run as fast as possible.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=1.5,
        help="How long to keep the last pose on screen before exiting. Ignored with --no-visualizer.",
    )
    parser.add_argument(
        "--no-visualizer",
        action="store_true",
        help="Run the rolling-window IK replay without launching the visualizer window.",
    )
    return parser.parse_args()


def repo_pose2sim_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def ensure_simbody_visualizer_on_path() -> None:
    """
    Make the Simbody visualizer discoverable without requiring a manual export.
    """

    prefix = os.environ.get("CONDA_PREFIX") or sys.prefix
    candidates = [
        Path(prefix) / "libexec" / "simbody",
        Path(prefix) / "simbody" / "libexec" / "simbody",
    ]
    existing = [str(path) for path in candidates if path.exists()]
    if not existing:
        return

    current = os.environ.get("PATH", "")
    current_parts = current.split(os.pathsep) if current else []
    prepend = [path for path in existing if path not in current_parts]
    if prepend:
        os.environ["PATH"] = os.pathsep.join(prepend + current_parts)


def add_geometry_search_path(pose2sim_dir: Path) -> None:
    geometry_dir = pose2sim_dir / "OpenSim_Setup" / "Geometry"
    if geometry_dir.exists():
        osim.ModelVisualizer.addDirToGeometrySearchPaths(str(geometry_dir))


def pick_default_model(pose2sim_dir: Path) -> Path:
    demo_kinematics_dir = pose2sim_dir / "Demo_SinglePerson" / "kinematics"
    demo_models = sorted(demo_kinematics_dir.glob("*.osim"))
    if demo_models:
        return demo_models[0]
    return pose2sim_dir / "OpenSim_Setup" / "Model_Pose2Sim_simple.osim"


def pick_default_trc(pose2sim_dir: Path) -> Path:
    candidates = [
        pose2sim_dir / "Demo_SinglePerson" / "pose-3d" / "Demo_SinglePerson_1-96_filt_butterworth_LSTM.trc",
        pose2sim_dir / "Demo_SinglePerson" / "pose-3d" / "Demo_SinglePerson_1-96_filt_butterworth.trc",
        pose2sim_dir / "Demo_SinglePerson" / "pose-3d" / "Demo_SinglePerson_1-95.trc",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("Could not find a demo TRC file to replay.")


def preview_coordinates(model: osim.Model, limit: int = 8) -> list[str]:
    coord_set = model.getCoordinateSet()
    names = [coord_set.get(i).getName() for i in range(min(coord_set.getSize(), limit))]
    print(f"[info] Coordinate sample: {names}")
    return names


def resolve_probe_coordinate_name(model: osim.Model) -> str | None:
    preferred = ("knee_angle_r", "hip_flexion_r", "pelvis_tilt")
    coord_set = model.getCoordinateSet()
    coord_names = [coord_set.get(i).getName() for i in range(coord_set.getSize())]
    for name in preferred:
        if name in coord_names:
            return name
    return coord_names[0] if coord_names else None


def build_marker_weights(labels: Sequence[str]) -> osim.SetMarkerWeights:
    weights = osim.SetMarkerWeights()
    for label in labels:
        weights.cloneAndAppend(osim.MarkerWeight(str(label), 1.0))
    return weights


def build_markers_reference(
    table: osim.TimeSeriesTableVec3,
    weights: osim.SetMarkerWeights,
) -> tuple[osim.MarkersReference, str]:
    candidates = [
        ("MarkersReference(table, weights)", lambda: osim.MarkersReference(table, weights)),
        (
            "MarkersReference(table, weights, Units.Meters)",
            lambda: osim.MarkersReference(table, weights, osim.Units(osim.Units.Meters)),
        ),
    ]
    last_error = None
    for label, factory in candidates:
        try:
            return factory(), label
        except Exception as exc:
            last_error = exc
            print(f"[warn] {label} failed: {type(exc).__name__}: {exc}")
    raise RuntimeError(f"Could not construct MarkersReference: {last_error}")


def build_ik_solver(
    model: osim.Model,
    markers_ref: osim.MarkersReference,
) -> tuple[osim.InverseKinematicsSolver, str]:
    coord_refs = osim.SimTKArrayCoordinateReference()
    candidates = [
        (
            "InverseKinematicsSolver(model, markers_ref, coord_refs)",
            lambda: osim.InverseKinematicsSolver(model, markers_ref, coord_refs),
        ),
        (
            "InverseKinematicsSolver(model, markers_ref, coord_refs, 1.0)",
            lambda: osim.InverseKinematicsSolver(model, markers_ref, coord_refs, 1.0),
        ),
        (
            "InverseKinematicsSolver(model, markers_ref, coord_refs, False)",
            lambda: osim.InverseKinematicsSolver(model, markers_ref, coord_refs, False),
        ),
    ]
    last_error = None
    for label, factory in candidates:
        try:
            return factory(), label
        except Exception as exc:
            last_error = exc
            print(f"[warn] {label} failed: {type(exc).__name__}: {exc}")
    raise RuntimeError(f"Could not construct InverseKinematicsSolver: {last_error}")


def build_window_table(
    full_table: osim.TimeSeriesTableVec3,
    trc_path: Path,
    start_time: float,
    end_time: float,
) -> osim.TimeSeriesTableVec3:
    try:
        window_table = osim.TimeSeriesTableVec3(full_table)
    except Exception:
        window_table = osim.TimeSeriesTableVec3(str(trc_path))
    window_table.trim(float(start_time), float(end_time))
    return window_table


def maybe_show(viz, model: osim.Model, state) -> None:
    if viz is None:
        return
    model.realizePosition(state)
    viz.show(state)


def main() -> None:
    args = parse_args()
    if args.window_size < 2:
        raise ValueError("--window-size must be at least 2.")
    if args.step < 1:
        raise ValueError("--step must be at least 1.")

    pose2sim_dir = repo_pose2sim_dir()
    model_path = (args.model or pick_default_model(pose2sim_dir)).resolve()
    trc_path = (args.trc or pick_default_trc(pose2sim_dir)).resolve()

    ensure_simbody_visualizer_on_path()
    add_geometry_search_path(pose2sim_dir)

    print(f"[info] OpenSim: {osim.GetVersionAndDate()}")
    print(f"[info] Model path: {model_path}")
    print(f"[info] TRC path:   {trc_path}")
    print(f"[info] Window size: {args.window_size} frames, step={args.step}")
    print(f"[info] Visualizer enabled: {not args.no_visualizer}")

    model = osim.Model(str(model_path))
    if not args.no_visualizer:
        model.setUseVisualizer(True)
    state = model.initSystem()
    viz = model.getVisualizer() if not args.no_visualizer else None

    marker_set = model.getMarkerSet()
    probe_coord_name = resolve_probe_coordinate_name(model)

    print("[ok] Model loaded and system initialized.")
    print(f"[info] marker_count_in_model={marker_set.getSize()}")
    preview_coordinates(model)

    full_table = osim.TimeSeriesTableVec3(str(trc_path))
    labels = list(full_table.getColumnLabels())
    times = list(full_table.getIndependentColumn())

    print(f"[ok] Loaded full marker table: rows={full_table.getNumRows()} cols={len(labels)}")
    if len(times) < args.window_size:
        raise RuntimeError(
            f"TRC contains only {len(times)} frames, smaller than requested window size {args.window_size}."
        )

    weights = build_marker_weights(labels)
    first_marker_name = marker_set.get(0).getName() if marker_set.getSize() else None

    maybe_show(viz, model, state)

    end_indices = list(range(args.window_size - 1, len(times), args.step))
    if args.max_windows > 0:
        end_indices = end_indices[: args.max_windows]

    solve_times_ms = []
    marker_errors = []
    probe_values = []

    for window_idx, end_idx in enumerate(end_indices, start=1):
        start_idx = end_idx - args.window_size + 1
        start_time = float(times[start_idx])
        end_time = float(times[end_idx])

        window_table = build_window_table(full_table, trc_path, start_time, end_time)
        markers_ref, markers_ctor = build_markers_reference(window_table, weights)
        solver, solver_ctor = build_ik_solver(model, markers_ref)

        if window_idx == 1:
            print(f"[info] Using {markers_ctor}")
            print(f"[info] Using {solver_ctor}")

        if hasattr(solver, "setAccuracy"):
            solver.setAccuracy(1e-3)
        if hasattr(solver, "setAdvanceTimeFromReference"):
            solver.setAdvanceTimeFromReference(False)

        state.setTime(end_time)
        t0 = time.perf_counter()
        solver.assemble(state)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        solve_times_ms.append(solve_ms)

        marker_error = None
        if first_marker_name and hasattr(solver, "computeCurrentMarkerError"):
            try:
                marker_error = float(solver.computeCurrentMarkerError(first_marker_name))
                marker_errors.append(marker_error)
            except Exception:
                marker_error = None

        probe_value = None
        if probe_coord_name:
            probe_value = float(model.getCoordinateSet().get(probe_coord_name).getValue(state))
            probe_values.append(probe_value)

        maybe_show(viz, model, state)

        msg = (
            f"[window {window_idx:02d}] frames={start_idx + 1:03d}-{end_idx + 1:03d} "
            f"time=({start_time:.4f},{end_time:.4f}) "
            f"rows={window_table.getNumRows():02d} "
            f"solve_ms={solve_ms:7.2f} "
            f"markers={solver.getNumMarkersInUse():02d}"
        )
        if marker_error is not None:
            msg += f" marker_error={marker_error:.5f}"
        if probe_value is not None:
            msg += f" {probe_coord_name}={probe_value:.5f}"
        print(msg)

        if args.replay_fps > 0:
            time.sleep(1.0 / args.replay_fps)

    print()
    print(f"[summary] windows_run={len(solve_times_ms)}")
    print(
        "[summary] solve_ms avg={:.2f} min={:.2f} max={:.2f}".format(
            statistics.mean(solve_times_ms),
            min(solve_times_ms),
            max(solve_times_ms),
        )
    )
    if marker_errors:
        print(
            "[summary] marker_error avg={:.5f} min={:.5f} max={:.5f}".format(
                statistics.mean(marker_errors),
                min(marker_errors),
                max(marker_errors),
            )
        )
    if probe_values and len(probe_values) > 1:
        print(
            "[summary] {} range=({:.5f}, {:.5f})".format(
                probe_coord_name,
                min(probe_values),
                max(probe_values),
            )
        )

    if viz is not None and args.hold_seconds > 0:
        print(f"[info] Holding final pose for {args.hold_seconds:.1f}s before exit.")
        time.sleep(args.hold_seconds)

    print("\n[pass] Rolling-window IK to API Visualizer replay is working at the basic level.")


if __name__ == "__main__":
    main()
