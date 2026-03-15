#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Run the realtime V1 pipeline on synchronized multi-camera replay videos.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import tempfile
import time
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _ensure_runtime_dir() -> Path:
    runtime_dir = REPO_ROOT / ".codex_runtime"
    runtime_dir.mkdir(parents=True, exist_ok=True)
    return runtime_dir


def apply_runtime_preflight() -> Path:
    runtime_dir = _ensure_runtime_dir()
    mpl_dir = runtime_dir / "mplconfig"
    temp_dir = runtime_dir / "tmp"
    mpl_dir.mkdir(parents=True, exist_ok=True)
    temp_dir.mkdir(parents=True, exist_ok=True)

    os.environ.setdefault("MPLCONFIGDIR", str(mpl_dir))
    os.environ.setdefault("TMPDIR", str(temp_dir))
    os.environ.setdefault("TMP", str(temp_dir))
    os.environ.setdefault("TEMP", str(temp_dir))
    tempfile.tempdir = str(temp_dir)
    return runtime_dir


RUNTIME_DIR = apply_runtime_preflight()

from Pose2Sim.common import natural_sort_key
from Pose2Sim.kinematics import get_model_path, get_opensim_setup_dir
from Pose2Sim.realtime.capture import VideoReplayFrameSource
from Pose2Sim.realtime.config import load_realtime_config
from Pose2Sim.realtime.filter_realtime import PassThroughFilter, RealtimeKalmanFilter
from Pose2Sim.realtime.marker_buffer import SlidingMarkerBuffer
from Pose2Sim.realtime.opensim_ik import RealtimeIKSolver
from Pose2Sim.realtime.opensim_viz import OpenSimVisualizer
from Pose2Sim.realtime.pipeline import RealtimePipeline
from Pose2Sim.realtime.pose2d import RealtimePoseEstimator
from Pose2Sim.realtime.recorder import RealtimeRecorder
from Pose2Sim.realtime.triangulate_frame import RealtimeFrameTriangulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Pose2Sim realtime V1 file-replay pipeline.")
    parser.add_argument(
        "--config",
        type=str,
        default="Pose2Sim/Demo_SinglePerson/Config.toml",
        help="Path to a single-trial Config.toml or trial directory.",
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="Optional explicit OpenSim model path for realtime replay. Overrides realtime.ik.model_path.",
    )
    parser.add_argument(
        "--no-visualizer",
        action="store_true",
        help="Run the full realtime pipeline without launching the OpenSim API Visualizer.",
    )
    parser.add_argument(
        "--record-mot",
        action="store_true",
        help="Force recorder output and write a realtime.mot file.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Optional limit for debugging. Use 0 to process the full replay.",
    )
    parser.add_argument(
        "--playback-fps",
        type=float,
        default=None,
        help="Optional replay throttling. Defaults to realtime.visualizer.playback_fps.",
    )
    return parser.parse_args()


def resolve_model_path(
    project_dir: str,
    configured_model_path: str | None,
    use_simple_model: bool,
) -> tuple[str, str]:
    project_path = Path(project_dir).resolve()
    if configured_model_path:
        candidate = Path(configured_model_path)
        if not candidate.is_absolute():
            cwd_candidate = candidate.resolve()
            project_candidate = (project_path / candidate).resolve()
            candidate = cwd_candidate if cwd_candidate.exists() else project_candidate
        return str(candidate.resolve()), "explicit"

    if use_simple_model:
        return (
            str(get_model_path(True, get_opensim_setup_dir()).resolve()),
            "simple-default",
        )

    trial_models = sorted((project_path / "kinematics").glob("*.osim"), key=natural_sort_key)
    if trial_models:
        return str(trial_models[0].resolve()), "trial-default"

    return (
        str(get_model_path(True, get_opensim_setup_dir()).resolve()),
        "simple-fallback",
    )


def detect_frame_rate(sources: tuple[str, ...], configured_frame_rate: float) -> float:
    if configured_frame_rate and configured_frame_rate > 0:
        return float(configured_frame_rate)
    if not sources:
        return 30.0

    capture = cv2.VideoCapture(sources[0])
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
    finally:
        capture.release()
    return float(fps) if fps and fps > 0 else 30.0


def build_pose_filter(config_dict, frame_rate: float):
    filtering_cfg = config_dict.get("filtering", {})
    if not bool(filtering_cfg.get("filter", True)):
        return PassThroughFilter()

    kalman_cfg = filtering_cfg.get("kalman", {})
    trust_ratio = float(kalman_cfg.get("trust_ratio", 500))
    return RealtimeKalmanFilter(frame_rate=frame_rate, trust_ratio=trust_ratio)


def _mean_or_nan(values: list[float]) -> float:
    finite_values = [float(value) for value in values if value is not None and value == value]
    return float(sum(finite_values) / len(finite_values)) if finite_values else float("nan")


def _int_mean_or_zero(values: list[int]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def main() -> None:
    args = parse_args()
    realtime_config = load_realtime_config(args.config)
    if not realtime_config.capture.sources:
        raise FileNotFoundError(
            f"No replay videos were found for project_dir={realtime_config.project_dir}. "
            "Set realtime.capture.sources or provide videos/*.mp4."
        )
    if not realtime_config.calib_file:
        raise FileNotFoundError(
            f"No calibration .toml file was found under {Path(realtime_config.project_dir) / 'calibration'}."
        )

    frame_rate = detect_frame_rate(realtime_config.capture.sources, realtime_config.capture.frame_rate)
    model_path, model_source = resolve_model_path(
        realtime_config.project_dir,
        args.model_path or realtime_config.ik.model_path,
        realtime_config.ik.use_simple_model,
    )
    osim_setup_dir = get_opensim_setup_dir()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logging.info("Realtime V1 configuration")
    logging.info("  project_dir=%s", realtime_config.project_dir)
    logging.info("  calib_file=%s", realtime_config.calib_file)
    logging.info("  model_path=%s", model_path)
    logging.info("  model_source=%s", model_source)
    logging.info("  frame_rate=%.3f", frame_rate)
    logging.info("  camera_ids=%s", realtime_config.capture.camera_ids)
    logging.info("  runtime_dir=%s", RUNTIME_DIR)
    logging.info(
        "  playback_fps=%s",
        args.playback_fps if args.playback_fps is not None else realtime_config.visualizer.playback_fps,
    )

    frame_source = VideoReplayFrameSource(
        sources=realtime_config.capture.sources,
        camera_ids=realtime_config.capture.camera_ids,
        frame_rate=frame_rate,
        frame_offsets=realtime_config.capture.frame_offsets,
        stop_on_shortest=realtime_config.capture.stop_on_shortest,
    )
    pose2d_estimator = RealtimePoseEstimator(
        config_dict=realtime_config.raw_config,
        camera_ids=realtime_config.capture.camera_ids,
    )
    logging.info("  runtime_pose_backends=%s", pose2d_estimator.runtime_choices)
    triangulator = RealtimeFrameTriangulator(
        config_dict=realtime_config.raw_config,
        calib_file=realtime_config.calib_file,
        camera_ids=realtime_config.capture.camera_ids,
        marker_names=pose2d_estimator.keypoint_names,
    )
    pose_filter = build_pose_filter(realtime_config.raw_config, frame_rate)
    marker_buffer = SlidingMarkerBuffer(
        window_size=realtime_config.ik.window_size,
        expected_marker_names=pose2d_estimator.keypoint_names,
    )
    ik_solver = None
    if realtime_config.ik.enabled:
        ik_solver = RealtimeIKSolver(
            model_path=model_path,
            pose_model=realtime_config.pose_model,
            osim_setup_dir=osim_setup_dir,
            warm_start=realtime_config.ik.warm_start,
        )

    visualizer = None
    visualizer_enabled = realtime_config.visualizer.enabled and not args.no_visualizer
    if visualizer_enabled:
        visualizer = OpenSimVisualizer(model_path=model_path, osim_setup_dir=osim_setup_dir)

    record_mot = bool(args.record_mot or realtime_config.recorder.record_mot)
    recorder_enabled = bool(realtime_config.recorder.enabled or record_mot)
    recorder = None
    if recorder_enabled:
        recorder = RealtimeRecorder(
            output_dir=realtime_config.recorder.output_dir,
            record_markers=realtime_config.recorder.record_markers,
            record_mot=record_mot,
        )

    pipeline = RealtimePipeline(
        frame_source=frame_source,
        pose2d_estimator=pose2d_estimator,
        triangulator=triangulator,
        marker_buffer=marker_buffer,
        pose3d_filter=pose_filter,
        ik_solver=ik_solver,
        visualizer=visualizer,
        recorder=recorder,
        min_window_size=realtime_config.ik.min_window_size,
    )

    playback_fps = (
        float(args.playback_fps)
        if args.playback_fps is not None
        else float(realtime_config.visualizer.playback_fps)
    )

    processed_frames = 0
    produced_states = 0
    capture_times_ms: list[float] = []
    pose2d_times_ms: list[float] = []
    triangulate_times_ms: list[float] = []
    filter_times_ms: list[float] = []
    ik_times_ms: list[float] = []
    valid_markers: list[int] = []
    reprojection_errors: list[float] = []
    markers_in_use: list[int] = []
    start_time = time.perf_counter()

    pipeline.start()
    try:
        while True:
            t0 = time.perf_counter()
            frame_packets = frame_source.read()
            capture_ms = (time.perf_counter() - t0) * 1000.0
            if not frame_packets:
                break
            capture_times_ms.append(capture_ms)

            state_packet = pipeline.run_step(frame_packets)
            processed_frames += 1
            step_metrics = dict(pipeline.last_step_metrics)
            pose2d_times_ms.append(float(step_metrics.get("pose2d_ms", 0.0)))
            triangulate_times_ms.append(float(step_metrics.get("triangulate_ms", 0.0)))
            filter_times_ms.append(float(step_metrics.get("filter_ms", 0.0)))
            valid_markers.append(int(step_metrics.get("valid_markers", 0)))
            reproj = step_metrics.get("reprojection_error", float("nan"))
            if reproj is not None:
                reprojection_errors.append(float(reproj))
            if state_packet is not None:
                produced_states += 1
                ik_times_ms.append(float(step_metrics.get("ik_ms", 0.0)))
                markers_in_use.append(int(step_metrics.get("num_markers_in_use", 0)))

            if args.max_frames > 0 and processed_frames >= args.max_frames:
                break
            if playback_fps > 0:
                time.sleep(1.0 / playback_fps)
    finally:
        pipeline.stop()

    elapsed_s = time.perf_counter() - start_time
    logging.info("Realtime replay finished")
    logging.info("  processed_frames=%d", processed_frames)
    logging.info("  produced_states=%d", produced_states)
    logging.info("  elapsed_s=%.3f", elapsed_s)
    logging.info("Realtime stage summary (ms)")
    logging.info("  capture_avg_ms=%.2f", _mean_or_nan(capture_times_ms))
    logging.info("  pose2d_avg_ms=%.2f", _mean_or_nan(pose2d_times_ms))
    logging.info("  triangulate_avg_ms=%.2f", _mean_or_nan(triangulate_times_ms))
    logging.info("  filter_avg_ms=%.2f", _mean_or_nan(filter_times_ms))
    logging.info("  ik_avg_ms=%.2f", _mean_or_nan(ik_times_ms))
    logging.info("Realtime data summary")
    logging.info("  avg_valid_markers=%.2f", _int_mean_or_zero(valid_markers))
    logging.info("  avg_num_markers_in_use=%.2f", _int_mean_or_zero(markers_in_use))
    logging.info("  avg_reprojection_error=%.5f", _mean_or_nan(reprojection_errors))


if __name__ == "__main__":
    main()
