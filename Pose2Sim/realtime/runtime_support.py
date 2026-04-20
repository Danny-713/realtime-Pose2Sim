#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Shared runtime helpers for realtime entry points.

Replay and live capture intentionally share the same runtime preflight,
pipeline assembly, and benchmark reporting so the input source can vary
without changing the rest of the realtime stack.
"""

from __future__ import annotations

import logging
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

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
from Pose2Sim.realtime.augmentation import RealtimeMarkerAugmenter
from Pose2Sim.realtime.config import (
    RealtimeConfig,
    RealtimeFilteringConfig,
    RealtimeOfflineLikeQualityConfig,
    RealtimePostAugmentationFilterConfig,
)
from Pose2Sim.realtime.filter_realtime import (
    PassThroughFilter,
    RealtimeButterworthWindowFilter,
    RealtimeKalmanFilter,
    RealtimeKalmanRTSWindowFilter,
    RealtimeOneEuroFilter,
)
from Pose2Sim.realtime.marker_buffer import SlidingMarkerBuffer
from Pose2Sim.realtime.offline_like_quality import (
    RealtimeOfflineLikeQualityProcessor,
    RealtimePose2DQualityMask,
)
from Pose2Sim.realtime.opensim_ik import RealtimeIKSolver
from Pose2Sim.realtime.opensim_viz import OpenSimVisualizer
from Pose2Sim.realtime.pipeline import RealtimePipeline
from Pose2Sim.realtime.pose2d import RealtimePoseEstimator
from Pose2Sim.realtime.recorder import RealtimeRecorder
from Pose2Sim.realtime.triangulate_frame import RealtimeFrameTriangulator


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


def detect_replay_frame_rate(sources: tuple[str, ...], configured_frame_rate: float) -> float:
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


def resolve_live_frame_rate(configured_frame_rate: float) -> float:
    return float(configured_frame_rate) if configured_frame_rate and configured_frame_rate > 0 else 30.0


def build_pose_filter(filter_cfg: RealtimeFilteringConfig, frame_rate: float):
    filter_type = filter_cfg.type

    if filter_type == "none":
        return PassThroughFilter()

    if filter_type == "kalman":
        return RealtimeKalmanFilter(
            frame_rate=frame_rate,
            trust_ratio=filter_cfg.kalman_trust_ratio,
        )

    if filter_type == "one_euro":
        return RealtimeOneEuroFilter(
            frame_rate=frame_rate,
            min_cutoff=filter_cfg.one_euro_min_cutoff,
            beta=filter_cfg.one_euro_beta,
            d_cutoff=filter_cfg.one_euro_d_cutoff,
        )

    if filter_type == "butterworth":
        return RealtimeButterworthWindowFilter(
            frame_rate=frame_rate,
            order=filter_cfg.butterworth_order,
            cutoff=filter_cfg.butterworth_cutoff,
            window_size=filter_cfg.butterworth_window_size,
        )

    if filter_type == "kalman_rts":
        return RealtimeKalmanRTSWindowFilter(
            frame_rate=frame_rate,
            trust_ratio=filter_cfg.kalman_rts_trust_ratio,
            window_size=filter_cfg.kalman_rts_window_size,
        )

    raise ValueError(
        f"Unknown realtime filter type {filter_type!r}. "
        "Supported types: 'kalman', 'one_euro', 'butterworth', 'kalman_rts', 'none'."
    )


def build_post_augmentation_filter(
    filter_cfg: RealtimePostAugmentationFilterConfig,
    frame_rate: float,
):
    filter_type = filter_cfg.type

    if not filter_cfg.enabled or filter_type == "none":
        return None

    if filter_type == "one_euro":
        return RealtimeOneEuroFilter(
            frame_rate=frame_rate,
            min_cutoff=filter_cfg.one_euro_min_cutoff,
            beta=filter_cfg.one_euro_beta,
            d_cutoff=filter_cfg.one_euro_d_cutoff,
        )

    raise ValueError(
        f"Unknown realtime post-augmentation filter type {filter_type!r}. "
        "Supported types: 'one_euro', 'none'."
    )


def build_pose2d_quality_mask(quality_cfg: RealtimeOfflineLikeQualityConfig):
    if not quality_cfg.enabled or not quality_cfg.mask_low_likelihood_2d:
        return None

    return RealtimePose2DQualityMask(
        likelihood_threshold=quality_cfg.likelihood_threshold_triangulation,
    )


def build_offline_like_quality_processor(
    quality_cfg: RealtimeOfflineLikeQualityConfig,
    *,
    pose_model: str,
    raw_marker_names,
):
    if not quality_cfg.enabled:
        return None

    return RealtimeOfflineLikeQualityProcessor(
        pose_model=pose_model,
        raw_marker_names=raw_marker_names,
        window_size=quality_cfg.window_size,
        min_window_size=quality_cfg.min_window_size,
        output_mode=quality_cfg.output_mode,
        gate_high_reproj_3d=quality_cfg.gate_high_reproj_3d,
        reproj_error_threshold_triangulation=quality_cfg.reproj_error_threshold_triangulation,
        min_cameras_for_triangulation=quality_cfg.min_cameras_for_triangulation,
        interp_short_gaps=quality_cfg.interp_short_gaps,
        interp_max_gap=quality_cfg.interp_max_gap,
        fill_large_gaps_with=quality_cfg.fill_large_gaps_with,
        reject_outliers=quality_cfg.reject_outliers,
        hampel_window_size=quality_cfg.hampel_window_size,
        hampel_n_sigma=quality_cfg.hampel_n_sigma,
        target=quality_cfg.target,
        parameter_source=quality_cfg.parameter_source,
    )


def build_runtime_pipeline(
    realtime_config: RealtimeConfig,
    frame_source: Any,
    frame_rate: float,
    model_path: str,
    *,
    visualizer_enabled: bool,
    record_mot: bool,
) -> tuple[RealtimePipeline, dict[str, Any]]:
    osim_setup_dir = get_opensim_setup_dir()

    pose2d_estimator = RealtimePoseEstimator(
        config_dict=realtime_config.raw_config,
        camera_ids=realtime_config.capture.camera_ids,
        parallel=realtime_config.pose.parallel,
    )
    triangulator = RealtimeFrameTriangulator(
        config_dict=realtime_config.raw_config,
        calib_file=realtime_config.calib_file,
        camera_ids=realtime_config.capture.camera_ids,
        marker_names=pose2d_estimator.keypoint_names,
    )
    pose2d_quality_mask = build_pose2d_quality_mask(realtime_config.offline_like_quality)
    offline_like_quality_processor = build_offline_like_quality_processor(
        realtime_config.offline_like_quality,
        pose_model=realtime_config.pose_model,
        raw_marker_names=pose2d_estimator.keypoint_names,
    )
    pose_filter = build_pose_filter(realtime_config.filtering, frame_rate)
    if realtime_config.post_augmentation_filter.enabled and not realtime_config.augmentation.enabled:
        raise ValueError(
            "realtime.post_augmentation_filter.enabled=true requires "
            "realtime.augmentation.enabled=true."
        )
    post_augmentation_filter = build_post_augmentation_filter(
        realtime_config.post_augmentation_filter,
        frame_rate,
    )

    marker_augmenter = None
    expected_marker_names = tuple(pose2d_estimator.keypoint_names)
    if realtime_config.augmentation.enabled:
        marker_augmenter = RealtimeMarkerAugmenter(
            config_dict=realtime_config.raw_config,
            pose_model=realtime_config.pose_model,
            raw_marker_names=pose2d_estimator.keypoint_names,
            model_name=realtime_config.augmentation.model_name,
            model_version=realtime_config.augmentation.model_version,
            window_size=realtime_config.augmentation.window_size,
            min_window_size=realtime_config.augmentation.min_window_size,
            output_mode=realtime_config.augmentation.output_mode,
            feet_on_floor=realtime_config.augmentation.feet_on_floor,
            first_frame_feet_on_floor=realtime_config.augmentation.first_frame_feet_on_floor,
            use_subject_stats=realtime_config.augmentation.use_subject_stats,
        )
        expected_marker_names = marker_augmenter.output_marker_names

    marker_buffer = SlidingMarkerBuffer(
        window_size=realtime_config.ik.window_size,
        expected_marker_names=expected_marker_names,
    )

    ik_solver = None
    if realtime_config.ik.enabled:
        marker_set_name = realtime_config.ik.marker_set_name
        if realtime_config.augmentation.enabled and not marker_set_name:
            marker_set_name = "LSTM"
        ik_solver = RealtimeIKSolver(
            model_path=model_path,
            pose_model=realtime_config.pose_model,
            osim_setup_dir=osim_setup_dir,
            warm_start=realtime_config.ik.warm_start,
            marker_set_name=marker_set_name,
        )

    visualizer = None
    if visualizer_enabled:
        visualizer = OpenSimVisualizer(model_path=model_path, osim_setup_dir=osim_setup_dir)

    recorder = None
    recorder_enabled = bool(realtime_config.recorder.enabled or record_mot)
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
        pose2d_quality_mask=pose2d_quality_mask,
        offline_like_quality_processor=offline_like_quality_processor,
        pose3d_filter=pose_filter,
        marker_augmenter=marker_augmenter,
        post_augmentation_filter=post_augmentation_filter,
        ik_solver=ik_solver,
        visualizer=visualizer,
        recorder=recorder,
        min_window_size=realtime_config.ik.min_window_size,
    )
    return pipeline, {
        "pose2d_estimator": pose2d_estimator,
        "triangulator": triangulator,
        "pose2d_quality_mask": pose2d_quality_mask,
        "offline_like_quality_processor": offline_like_quality_processor,
        "pose_filter": pose_filter,
        "marker_augmenter": marker_augmenter,
        "post_augmentation_filter": post_augmentation_filter,
        "marker_buffer": marker_buffer,
        "ik_solver": ik_solver,
        "visualizer": visualizer,
        "recorder": recorder,
    }


def log_runtime_configuration(
    *,
    source_label: str,
    realtime_config: RealtimeConfig,
    model_path: str,
    model_source: str,
    frame_rate: float,
    playback_fps: float,
) -> None:
    logging.info("Realtime V1 configuration")
    logging.info("  source=%s", source_label)
    logging.info("  project_dir=%s", realtime_config.project_dir)
    logging.info("  calib_file=%s", realtime_config.calib_file)
    logging.info("  model_path=%s", model_path)
    logging.info("  model_source=%s", model_source)
    logging.info("  frame_rate=%.3f", frame_rate)
    logging.info("  camera_ids=%s", realtime_config.capture.camera_ids)
    logging.info("  runtime_dir=%s", RUNTIME_DIR)
    logging.info("  playback_fps=%s", playback_fps)
    logging.info(
        "  rt_pose_mode=%s  rt_det_frequency=%d  parallel=%s",
        realtime_config.pose.mode,
        realtime_config.pose.det_frequency,
        realtime_config.pose.parallel,
    )
    logging.info("  rt_filter_type=%s", realtime_config.filtering.type)
    if realtime_config.pre_augmentation_cleanup.enabled:
        logging.info(
            "  rt_pre_augmentation_cleanup_enabled=%s  window=%d  output_mode=%s  target=%s",
            realtime_config.pre_augmentation_cleanup.enabled,
            realtime_config.pre_augmentation_cleanup.window_size,
            realtime_config.pre_augmentation_cleanup.output_mode,
            realtime_config.pre_augmentation_cleanup.target,
        )
    logging.info(
        "  rt_offline_like_quality_enabled=%s  window=%d  output_mode=%s  params=%s",
        realtime_config.offline_like_quality.enabled,
        realtime_config.offline_like_quality.window_size,
        realtime_config.offline_like_quality.output_mode,
        realtime_config.offline_like_quality.parameter_source,
    )
    logging.info(
        "  rt_augmentation_enabled=%s  model=%s  version=%s  window=%d  output_mode=%s",
        realtime_config.augmentation.enabled,
        realtime_config.augmentation.model_name,
        realtime_config.augmentation.model_version,
        realtime_config.augmentation.window_size,
        realtime_config.augmentation.output_mode,
    )
    logging.info(
        "  rt_post_augmentation_filter_enabled=%s  type=%s",
        realtime_config.post_augmentation_filter.enabled,
        realtime_config.post_augmentation_filter.type,
    )
    quality_delay = (
        realtime_config.offline_like_quality.window_size // 2
        if realtime_config.offline_like_quality.enabled
        and realtime_config.offline_like_quality.output_mode == "center"
        else 0
    )
    filter_delay = 0
    if realtime_config.filtering.type == "butterworth":
        filter_delay = realtime_config.filtering.butterworth_window_size // 2
    elif realtime_config.filtering.type == "kalman_rts":
        filter_delay = realtime_config.filtering.kalman_rts_window_size // 2
    augmentation_delay = (
        realtime_config.augmentation.window_size // 2
        if realtime_config.augmentation.enabled and realtime_config.augmentation.output_mode == "center"
        else 0
    )
    if quality_delay > 0 and (filter_delay > 0 or augmentation_delay > 0):
        total_delay_frames = quality_delay + filter_delay + augmentation_delay
        logging.info(
            "  rt_fixed_lag_delay_frames=%d (~%.3fs): quality=%d filter=%d augmentation=%d",
            total_delay_frames,
            total_delay_frames / float(frame_rate) if frame_rate > 0 else float("nan"),
            quality_delay,
            filter_delay,
            augmentation_delay,
        )


def log_pipeline_components(details: dict[str, Any]) -> None:
    pose2d_estimator = details["pose2d_estimator"]
    logging.info("  runtime_pose_backends=%s", pose2d_estimator.runtime_choices)
    pose2d_quality_mask = details.get("pose2d_quality_mask")
    if pose2d_quality_mask is not None:
        logging.info(
            "  pose2d_quality_mask=%s",
            type(pose2d_quality_mask).__name__,
        )
    offline_like_quality_processor = details.get("offline_like_quality_processor")
    if offline_like_quality_processor is not None:
        logging.info(
            "  offline_like_quality_processor=%s",
            type(offline_like_quality_processor).__name__,
        )
    marker_augmenter = details.get("marker_augmenter")
    if marker_augmenter is not None:
        logging.info(
            "  augmentation_output_markers=%d  response_markers=%d",
            len(marker_augmenter.output_marker_names or ()),
            len(marker_augmenter.response_marker_names),
        )
    post_augmentation_filter = details.get("post_augmentation_filter")
    if post_augmentation_filter is not None:
        logging.info(
            "  post_augmentation_filter=%s",
            type(post_augmentation_filter).__name__,
        )


def _mean_or_nan(values: list[float]) -> float:
    finite_values = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(sum(finite_values) / len(finite_values)) if finite_values else float("nan")


def _int_mean_or_zero(values: list[int]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def run_pipeline_loop(
    *,
    pipeline: RealtimePipeline,
    frame_source: Any,
    max_frames: int,
    playback_fps: float,
) -> dict[str, float | int]:
    processed_frames = 0
    produced_states = 0
    capture_times_ms: list[float] = []
    pose2d_times_ms: list[float] = []
    triangulate_times_ms: list[float] = []
    quality_mode_times_ms: list[float] = []
    filter_times_ms: list[float] = []
    augmentation_times_ms: list[float] = []
    post_filter_times_ms: list[float] = []
    ik_times_ms: list[float] = []
    valid_markers: list[int] = []
    quality_mode_gated_markers: list[int] = []
    quality_mode_interpolated_markers: list[int] = []
    quality_mode_outlier_replaced_markers: list[int] = []
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
            quality_mode_times_ms.append(float(step_metrics.get("quality_mode_ms", 0.0)))
            filter_times_ms.append(float(step_metrics.get("filter_ms", 0.0)))
            augmentation_times_ms.append(float(step_metrics.get("augmentation_ms", 0.0)))
            post_filter_times_ms.append(float(step_metrics.get("post_filter_ms", 0.0)))
            valid_markers.append(int(step_metrics.get("valid_markers", 0)))
            quality_mode_gated_markers.append(int(step_metrics.get("quality_mode_gated_markers", 0)))
            quality_mode_interpolated_markers.append(
                int(step_metrics.get("quality_mode_interpolated_markers", 0))
            )
            quality_mode_outlier_replaced_markers.append(
                int(step_metrics.get("quality_mode_outlier_replaced_markers", 0))
            )
            reproj = step_metrics.get("reprojection_error", float("nan"))
            if reproj is not None:
                reprojection_errors.append(float(reproj))
            if state_packet is not None:
                produced_states += 1
                ik_times_ms.append(float(step_metrics.get("ik_ms", 0.0)))
                markers_in_use.append(int(step_metrics.get("num_markers_in_use", 0)))

            if max_frames > 0 and processed_frames >= max_frames:
                break
            if playback_fps > 0:
                time.sleep(1.0 / playback_fps)
    finally:
        pipeline.stop()

    elapsed_s = time.perf_counter() - start_time
    return {
        "processed_frames": processed_frames,
        "produced_states": produced_states,
        "elapsed_s": elapsed_s,
        "capture_avg_ms": _mean_or_nan(capture_times_ms),
        "pose2d_avg_ms": _mean_or_nan(pose2d_times_ms),
        "triangulate_avg_ms": _mean_or_nan(triangulate_times_ms),
        "quality_mode_avg_ms": _mean_or_nan(quality_mode_times_ms),
        "filter_avg_ms": _mean_or_nan(filter_times_ms),
        "augmentation_avg_ms": _mean_or_nan(augmentation_times_ms),
        "post_filter_avg_ms": _mean_or_nan(post_filter_times_ms),
        "ik_avg_ms": _mean_or_nan(ik_times_ms),
        "avg_valid_markers": _int_mean_or_zero(valid_markers),
        "quality_mode_gated_markers_avg": _int_mean_or_zero(quality_mode_gated_markers),
        "quality_mode_interpolated_markers_avg": _int_mean_or_zero(quality_mode_interpolated_markers),
        "quality_mode_outlier_replaced_markers_avg": _int_mean_or_zero(
            quality_mode_outlier_replaced_markers
        ),
        "avg_num_markers_in_use": _int_mean_or_zero(markers_in_use),
        "avg_reprojection_error": _mean_or_nan(reprojection_errors),
    }


def log_pipeline_summary(summary: dict[str, float | int], *, finish_label: str) -> None:
    logging.info(finish_label)
    logging.info("  processed_frames=%d", int(summary["processed_frames"]))
    logging.info("  produced_states=%d", int(summary["produced_states"]))
    logging.info("  elapsed_s=%.3f", float(summary["elapsed_s"]))
    logging.info("Realtime stage summary (ms)")
    logging.info("  capture_avg_ms=%.2f", float(summary["capture_avg_ms"]))
    logging.info("  pose2d_avg_ms=%.2f", float(summary["pose2d_avg_ms"]))
    logging.info("  triangulate_avg_ms=%.2f", float(summary["triangulate_avg_ms"]))
    logging.info("  quality_mode_avg_ms=%.2f", float(summary["quality_mode_avg_ms"]))
    logging.info("  filter_avg_ms=%.2f", float(summary["filter_avg_ms"]))
    logging.info("  augmentation_avg_ms=%.2f", float(summary["augmentation_avg_ms"]))
    logging.info("  post_filter_avg_ms=%.2f", float(summary["post_filter_avg_ms"]))
    logging.info("  ik_avg_ms=%.2f", float(summary["ik_avg_ms"]))
    logging.info("Realtime data summary")
    logging.info("  avg_valid_markers=%.2f", float(summary["avg_valid_markers"]))
    logging.info(
        "  quality_mode_gated_markers_avg=%.2f",
        float(summary["quality_mode_gated_markers_avg"]),
    )
    logging.info(
        "  quality_mode_interpolated_markers_avg=%.2f",
        float(summary["quality_mode_interpolated_markers_avg"]),
    )
    logging.info(
        "  quality_mode_outlier_replaced_markers_avg=%.2f",
        float(summary["quality_mode_outlier_replaced_markers_avg"]),
    )
    logging.info("  avg_num_markers_in_use=%.2f", float(summary["avg_num_markers_in_use"]))
    logging.info("  avg_reprojection_error=%.5f", float(summary["avg_reprojection_error"]))
