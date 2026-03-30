#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Run the realtime V1 pipeline on live multi-camera input.

This entry point is intentionally separate from file replay so live capture can
evolve without complicating the replay baseline.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Pose2Sim.realtime.capture import LiveCameraFrameSource
from Pose2Sim.realtime.config import load_realtime_config
from Pose2Sim.realtime.runtime_support import (
    build_runtime_pipeline,
    log_pipeline_components,
    log_pipeline_summary,
    log_runtime_configuration,
    resolve_live_frame_rate,
    resolve_model_path,
    run_pipeline_loop,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Pose2Sim realtime V1 pipeline on live cameras.")
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
        help="Optional explicit OpenSim model path. Overrides realtime.ik.model_path.",
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
        help="Optional limit for debugging. Use 0 to keep running until capture stops.",
    )
    parser.add_argument(
        "--playback-fps",
        type=float,
        default=None,
        help="Optional throttling for downstream processing. Defaults to realtime.visualizer.playback_fps.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    realtime_config = load_realtime_config(args.config)

    if realtime_config.capture.source_type != "live_camera":
        raise ValueError(
            "run_live_capture.py expects realtime.capture.source_type = 'live_camera'. "
            f"Current value: {realtime_config.capture.source_type!r}."
        )
    if not realtime_config.capture.sources:
        raise FileNotFoundError(
            "No live camera sources were configured. "
            "Set realtime.capture.sources to camera indices or stream URLs."
        )
    if not realtime_config.calib_file:
        raise FileNotFoundError(
            f"No calibration .toml file was found under {Path(realtime_config.project_dir) / 'calibration'}."
        )

    frame_rate = resolve_live_frame_rate(realtime_config.capture.frame_rate)
    model_path, model_source = resolve_model_path(
        realtime_config.project_dir,
        args.model_path or realtime_config.ik.model_path,
        realtime_config.ik.use_simple_model,
    )
    playback_fps = (
        float(args.playback_fps)
        if args.playback_fps is not None
        else float(realtime_config.visualizer.playback_fps)
    )

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    log_runtime_configuration(
        source_label="live_camera",
        realtime_config=realtime_config,
        model_path=model_path,
        model_source=model_source,
        frame_rate=frame_rate,
        playback_fps=playback_fps,
    )

    frame_source = LiveCameraFrameSource(
        sources=realtime_config.capture.sources,
        camera_ids=realtime_config.capture.camera_ids,
        frame_rate=frame_rate,
        api_preference=realtime_config.capture.api_preference,
        frame_width=realtime_config.capture.frame_width,
        frame_height=realtime_config.capture.frame_height,
        buffer_size=realtime_config.capture.buffer_size,
        read_timeout_ms=realtime_config.capture.read_timeout_ms,
        max_batch_skew_ms=realtime_config.capture.max_batch_skew_ms,
    )
    visualizer_enabled = realtime_config.visualizer.enabled and not args.no_visualizer
    record_mot = bool(args.record_mot or realtime_config.recorder.record_mot)
    pipeline, details = build_runtime_pipeline(
        realtime_config,
        frame_source,
        frame_rate,
        model_path,
        visualizer_enabled=visualizer_enabled,
        record_mot=record_mot,
    )
    log_pipeline_components(details)

    summary = run_pipeline_loop(
        pipeline=pipeline,
        frame_source=frame_source,
        max_frames=args.max_frames,
        playback_fps=playback_fps,
    )
    log_pipeline_summary(summary, finish_label="Realtime live capture finished")


if __name__ == "__main__":
    main()
