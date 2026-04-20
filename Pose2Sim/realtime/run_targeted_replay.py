#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Run the realtime replay pipeline on a manually selected target person.

This entry point is intentionally isolated from the main realtime replay path.
It adds a lightweight target-selection layer in front of the existing single-
person realtime pipeline so we can validate multi-person scenes without
changing the production-facing replay runner.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Pose2Sim.realtime.runtime_support import (
    build_offline_like_quality_processor,
    build_pose2d_quality_mask,
    build_pose_filter,
    build_post_augmentation_filter,
    detect_replay_frame_rate,
    log_pipeline_components,
    log_pipeline_summary,
    log_runtime_configuration,
    resolve_model_path,
    run_pipeline_loop,
)
from Pose2Sim.common import bbox_xyxy_compute, sort_people_sports2d
from Pose2Sim.kinematics import get_opensim_setup_dir
from Pose2Sim.realtime.augmentation import RealtimeMarkerAugmenter
from Pose2Sim.realtime.capture import VideoReplayFrameSource
from Pose2Sim.realtime.config import RealtimeConfig, load_realtime_config
from Pose2Sim.realtime.marker_buffer import SlidingMarkerBuffer
from Pose2Sim.realtime.offline_like_quality import (
    RealtimeOfflineLikeQualityProcessor,
    RealtimePose2DQualityMask,
)
from Pose2Sim.realtime.opensim_ik import RealtimeIKSolver
from Pose2Sim.realtime.opensim_viz import OpenSimVisualizer
from Pose2Sim.realtime.packets import FramePacket, Pose2DPacket
from Pose2Sim.realtime.pipeline import RealtimePipeline
from Pose2Sim.realtime.pose2d import RealtimePoseEstimator
from Pose2Sim.realtime.recorder import RealtimeRecorder
from Pose2Sim.realtime.triangulate_frame import RealtimeFrameTriangulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Pose2Sim realtime replay pipeline with a manually selected target person.",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="Pose2Sim/demo_tiaosan_4cam/Config.toml",
        help="Path to a single-trial Config.toml or trial directory.",
    )
    parser.add_argument(
        "--frame-range",
        nargs=2,
        type=int,
        metavar=("START", "END"),
        required=True,
        help="Anchor-camera source frame range to process, inclusive.",
    )
    parser.add_argument(
        "--selection-source-frame",
        type=int,
        required=True,
        help="Anchor-camera source frame used for the manual target initialization.",
    )
    parser.add_argument(
        "--init-target",
        nargs="+",
        required=True,
        metavar="CAMERA:INDEX",
        help="Per-camera selected candidate index on the selection frame, e.g. cam01:0 cam03:2.",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=30,
        help="Frames to process before the selection frame so candidate ordering can stabilize.",
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
        help="Run the full targeted pipeline without launching the OpenSim API Visualizer.",
    )
    parser.add_argument(
        "--record-mot",
        action="store_true",
        help="Force recorder output and write a realtime.mot file.",
    )
    parser.add_argument(
        "--playback-fps",
        type=float,
        default=None,
        help="Optional replay throttling. Defaults to realtime.visualizer.playback_fps.",
    )
    return parser.parse_args()


def _parse_init_targets(values: Sequence[str]) -> dict[str, int]:
    init_targets: dict[str, int] = {}
    for value in values:
        if ":" not in value:
            raise ValueError(
                f"Invalid --init-target value {value!r}. Expected CAMERA:INDEX, for example cam01:0."
            )
        camera_id, candidate_text = value.split(":", 1)
        camera_id = camera_id.strip()
        if not camera_id:
            raise ValueError(f"Invalid --init-target value {value!r}: missing camera id.")
        try:
            candidate_index = int(candidate_text)
        except ValueError as exc:
            raise ValueError(
                f"Invalid --init-target value {value!r}: candidate index must be an integer."
            ) from exc
        if candidate_index < 0:
            raise ValueError(
                f"Invalid --init-target value {value!r}: candidate index must be >= 0."
            )
        if camera_id in init_targets:
            raise ValueError(
                f"Duplicate --init-target entry for {camera_id!r}. Pass each camera only once."
            )
        init_targets[camera_id] = candidate_index
    return init_targets


def _validate_targeted_args(
    *,
    realtime_config: RealtimeConfig,
    frame_range: tuple[int, int],
    selection_source_frame: int,
    warmup: int,
    init_targets: dict[str, int],
) -> None:
    frame_start, frame_end = frame_range
    if frame_start < 0 or frame_end < 0:
        raise ValueError("--frame-range values must be non-negative.")
    if frame_end < frame_start:
        raise ValueError("--frame-range END must be >= START.")
    if selection_source_frame < 0:
        raise ValueError("--selection-source-frame must be non-negative.")
    if selection_source_frame > frame_start:
        raise ValueError(
            "--selection-source-frame must be <= frame-range start so the target exists before main processing begins."
        )
    if warmup < 0:
        raise ValueError("--warmup must be >= 0.")
    if realtime_config.capture.source_type != "video_replay":
        raise ValueError(
            "run_targeted_replay.py currently supports only realtime.capture.source_type='video_replay'. "
            f"Current value: {realtime_config.capture.source_type!r}."
        )
    expected_camera_ids = set(realtime_config.capture.camera_ids)
    if set(init_targets) != expected_camera_ids:
        missing = sorted(expected_camera_ids - set(init_targets))
        extra = sorted(set(init_targets) - expected_camera_ids)
        parts = []
        if missing:
            parts.append(f"missing cameras={missing}")
        if extra:
            parts.append(f"unknown cameras={extra}")
        raise ValueError(
            "--init-target must cover exactly the configured realtime cameras. " + ", ".join(parts)
        )


def _ordered_packets(
    frame_packets: Sequence[FramePacket],
    camera_ids: Sequence[str],
) -> list[FramePacket]:
    by_camera = {packet.camera_id: packet for packet in frame_packets}
    missing = [camera_id for camera_id in camera_ids if camera_id not in by_camera]
    if missing:
        raise RuntimeError(
            f"Targeted replay expected packets for cameras {tuple(camera_ids)}, but {missing} were missing."
        )
    return [by_camera[camera_id] for camera_id in camera_ids]


class TargetedReplayPoseSelector:
    """
    Wrap RealtimePoseEstimator with manual one-time target initialization.

    Before the selection frame, we only warm up candidate ordering.  At the
    selection frame we choose the manually provided candidate for each camera.
    Afterwards each camera independently tracks that chosen person with the same
    keypoint-distance matching idea as the current single-person path.
    """

    def __init__(
        self,
        *,
        config_dict,
        camera_ids: Sequence[str],
        init_targets: dict[str, int],
        selection_source_frame: int,
    ):
        self.camera_ids = tuple(camera_ids)
        self.anchor_camera = self.camera_ids[0]
        self.selection_source_frame = int(selection_source_frame)
        self.init_targets = dict(init_targets)
        self._estimator = RealtimePoseEstimator(
            config_dict=config_dict,
            camera_ids=self.camera_ids,
            parallel=False,
        )
        self.keypoint_names = tuple(self._estimator.keypoint_names)
        self.max_distance_px = float(self._estimator.max_distance_px)
        self._target_prev: dict[str, np.ndarray] = {}
        self._selection_done = False
        self._last_processed_anchor_frame: int | None = None
        self._consecutive_missing: dict[str, int] = {camera_id: 0 for camera_id in self.camera_ids}

    @property
    def runtime_choices(self):
        return self._estimator.runtime_choices

    def shutdown(self) -> None:
        self._estimator.shutdown()

    def prepass_batch(self, frame_packets: Sequence[FramePacket]) -> None:
        self._process_batch(frame_packets, emit_packets=False)

    def infer_batch(self, frame_packets: Sequence[FramePacket]) -> list[Pose2DPacket]:
        packets = self._process_batch(frame_packets, emit_packets=True)
        return packets or []

    def _process_batch(
        self,
        frame_packets: Sequence[FramePacket],
        *,
        emit_packets: bool,
    ) -> list[Pose2DPacket] | None:
        ordered_packets = _ordered_packets(frame_packets, self.camera_ids)
        anchor_source_frame = self._anchor_source_frame(ordered_packets)
        self._last_processed_anchor_frame = anchor_source_frame

        if not self._selection_done:
            candidate_batches = self._detect_and_sort_candidates(ordered_packets)
            if anchor_source_frame < self.selection_source_frame:
                return None
            if anchor_source_frame > self.selection_source_frame:
                raise RuntimeError(
                    "Selection frame was skipped before manual target initialization. "
                    f"Expected anchor source frame {self.selection_source_frame}, got {anchor_source_frame}."
                )
            return self._initialize_targets(
                ordered_packets,
                candidate_batches,
                anchor_source_frame=anchor_source_frame,
                emit_packets=emit_packets,
            )

        return self._track_targets(
            ordered_packets,
            anchor_source_frame=anchor_source_frame,
            emit_packets=emit_packets,
        )

    def _detect_and_sort_candidates(
        self,
        ordered_packets: Sequence[FramePacket],
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        candidates: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for packet in ordered_packets:
            tracker = self._estimator._trackers[packet.camera_id]
            keypoints, scores = self._estimator._detect_people(tracker, packet.image)
            if keypoints.size == 0 or len(keypoints) == 0:
                candidates[packet.camera_id] = (keypoints, scores)
                continue
            keypoints, scores = self._estimator._track_single_person(packet.camera_id, keypoints, scores)
            candidates[packet.camera_id] = (keypoints, scores)
        return candidates

    def _initialize_targets(
        self,
        ordered_packets: Sequence[FramePacket],
        candidate_batches: dict[str, tuple[np.ndarray, np.ndarray]],
        *,
        anchor_source_frame: int,
        emit_packets: bool,
    ) -> list[Pose2DPacket] | None:
        output_packets: list[Pose2DPacket] = []
        summary_parts: list[str] = []

        for packet in ordered_packets:
            keypoints, scores = candidate_batches[packet.camera_id]
            candidate_count = int(len(keypoints))
            selected_index = self.init_targets[packet.camera_id]
            if selected_index >= candidate_count:
                raise ValueError(
                    "Selected candidate index is out of range at the initialization frame. "
                    f"camera={packet.camera_id} selected={selected_index} candidates={candidate_count} "
                    f"anchor_source_frame={anchor_source_frame}"
                )

            selected_keypoints = np.asarray(keypoints[selected_index], dtype=float)
            selected_scores = np.asarray(scores[selected_index], dtype=float)
            self._target_prev[packet.camera_id] = selected_keypoints.copy()
            self._consecutive_missing[packet.camera_id] = 0

            boxes = bbox_xyxy_compute(packet.image.shape, keypoints, padding=0)
            bbox = (
                np.asarray(boxes[selected_index], dtype=float)
                if len(boxes) > selected_index
                else np.full(4, np.nan, dtype=float)
            )
            mean_score = float(np.nanmean(selected_scores)) if selected_scores.size else float("nan")
            logging.info(
                "Target initialization: anchor_source_frame=%d camera=%s selected_candidate=%d candidates=%d "
                "mean_score=%.3f bbox=%s",
                anchor_source_frame,
                packet.camera_id,
                selected_index,
                candidate_count,
                mean_score,
                np.round(bbox, 1).tolist(),
            )
            summary_parts.append(f"{packet.camera_id}=selected#{selected_index}")

            if emit_packets:
                output_packets.append(
                    Pose2DPacket(
                        frame_id=packet.frame_id,
                        timestamp=packet.timestamp,
                        camera_id=packet.camera_id,
                        keypoints=selected_keypoints,
                        scores=selected_scores,
                        person_id=0,
                        metadata={
                            "detected_people": candidate_count,
                            "target_status": "selected",
                            "target_candidate_id": selected_index,
                            "target_consecutive_missing": 0,
                            "anchor_source_frame": anchor_source_frame,
                        },
                    )
                )

        self._selection_done = True
        logging.info(
            "Target selection frame %d locked: %s",
            anchor_source_frame,
            " ".join(summary_parts),
        )
        return output_packets if emit_packets else None

    def _track_targets(
        self,
        ordered_packets: Sequence[FramePacket],
        *,
        anchor_source_frame: int,
        emit_packets: bool,
    ) -> list[Pose2DPacket] | None:
        output_packets: list[Pose2DPacket] = []
        summary_parts: list[str] = []

        for packet in ordered_packets:
            tracker = self._estimator._trackers[packet.camera_id]
            keypoints, scores = self._estimator._detect_people(tracker, packet.image)
            detected_people = int(len(keypoints))
            matched_keypoints, matched_scores, matched_id, status = self._match_target_candidate(
                packet.camera_id,
                keypoints,
                scores,
            )

            if status == "matched":
                self._target_prev[packet.camera_id] = matched_keypoints.copy()
                self._consecutive_missing[packet.camera_id] = 0
                summary_parts.append(f"{packet.camera_id}=matched#{matched_id}")
            else:
                self._consecutive_missing[packet.camera_id] += 1
                summary_parts.append(
                    f"{packet.camera_id}=missing({self._consecutive_missing[packet.camera_id]})"
                )

            if emit_packets:
                output_packets.append(
                    Pose2DPacket(
                        frame_id=packet.frame_id,
                        timestamp=packet.timestamp,
                        camera_id=packet.camera_id,
                        keypoints=matched_keypoints,
                        scores=matched_scores,
                        person_id=0,
                        metadata={
                            "detected_people": detected_people,
                            "target_status": status,
                            "target_candidate_id": matched_id,
                            "target_consecutive_missing": self._consecutive_missing[packet.camera_id],
                            "anchor_source_frame": anchor_source_frame,
                        },
                    )
                )

        logging.info(
            "Target tracking: anchor_source_frame=%d %s",
            anchor_source_frame,
            " ".join(summary_parts),
        )
        return output_packets if emit_packets else None

    def _match_target_candidate(
        self,
        camera_id: str,
        keypoints: np.ndarray,
        scores: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, int, str]:
        previous = self._target_prev.get(camera_id)
        if previous is None or keypoints.size == 0 or len(keypoints) == 0:
            return self._empty_person_packet_components()

        _, _, sorted_ids = sort_people_sports2d(
            np.expand_dims(previous, axis=0),
            keypoints,
            max_dist=self.max_distance_px,
        )
        if len(sorted_ids) == 0:
            return self._empty_person_packet_components()

        matched_id = int(sorted_ids[0])
        if matched_id < 0 or matched_id >= len(keypoints):
            return self._empty_person_packet_components()

        matched_keypoints = np.asarray(keypoints[matched_id], dtype=float)
        if not np.isfinite(matched_keypoints).any():
            return self._empty_person_packet_components()

        if matched_id < len(scores):
            matched_scores = np.asarray(scores[matched_id], dtype=float)
        else:
            matched_scores = np.full((len(self.keypoint_names),), np.nan, dtype=float)
        return matched_keypoints, matched_scores, matched_id, "matched"

    def _empty_person_packet_components(self) -> tuple[np.ndarray, np.ndarray, int, str]:
        keypoints, scores = self._estimator._empty_person()
        return keypoints, scores, -1, "missing"

    def _anchor_source_frame(self, ordered_packets: Sequence[FramePacket]) -> int:
        for packet in ordered_packets:
            if packet.camera_id == self.anchor_camera:
                return int(packet.metadata.get("source_frame_id", -1))
        raise RuntimeError(f"Anchor camera {self.anchor_camera!r} was missing from the current frame batch.")


def _run_prepass(
    selector: TargetedReplayPoseSelector,
    realtime_config: RealtimeConfig,
    *,
    frame_rate: float,
    selection_source_frame: int,
    frame_range_start: int,
    warmup: int,
) -> None:
    start_reference = max(0, selection_source_frame - warmup)
    prepass_source = VideoReplayFrameSource(
        sources=realtime_config.capture.sources,
        camera_ids=realtime_config.capture.camera_ids,
        frame_rate=frame_rate,
        frame_offsets=tuple(offset + start_reference for offset in realtime_config.capture.frame_offsets),
        stop_on_shortest=realtime_config.capture.stop_on_shortest,
    )

    logging.info(
        "Targeted prepass: selection_source_frame=%d frame_range_start=%d warmup=%d start_reference=%d",
        selection_source_frame,
        frame_range_start,
        warmup,
        start_reference,
    )

    saw_selection = False
    prepass_source.start()
    try:
        while True:
            frame_packets = prepass_source.read()
            if not frame_packets:
                break
            anchor_source_frame = int(frame_packets[0].metadata.get("source_frame_id", -1))
            if anchor_source_frame >= frame_range_start:
                break
            selector.prepass_batch(frame_packets)
            if anchor_source_frame >= selection_source_frame:
                saw_selection = True
    finally:
        prepass_source.stop()

    if selection_source_frame < frame_range_start and not saw_selection:
        raise RuntimeError(
            "Targeted prepass did not reach the requested selection frame before the main frame range start. "
            f"selection_source_frame={selection_source_frame} frame_range_start={frame_range_start}"
        )


def _build_targeted_pipeline(
    *,
    realtime_config: RealtimeConfig,
    frame_source: VideoReplayFrameSource,
    frame_rate: float,
    model_path: str,
    target_selector: TargetedReplayPoseSelector,
    visualizer_enabled: bool,
    record_mot: bool,
) -> tuple[RealtimePipeline, dict[str, Any]]:
    osim_setup_dir = get_opensim_setup_dir()

    triangulator = RealtimeFrameTriangulator(
        config_dict=realtime_config.raw_config,
        calib_file=realtime_config.calib_file,
        camera_ids=realtime_config.capture.camera_ids,
        marker_names=target_selector.keypoint_names,
    )
    pose2d_quality_mask = build_pose2d_quality_mask(realtime_config.offline_like_quality)
    offline_like_quality_processor = build_offline_like_quality_processor(
        realtime_config.offline_like_quality,
        pose_model=realtime_config.pose_model,
        raw_marker_names=target_selector.keypoint_names,
    )
    pose_filter = build_pose_filter(realtime_config.filtering, frame_rate)
    if realtime_config.post_augmentation_filter.enabled and not realtime_config.augmentation.enabled:
        raise ValueError(
            "realtime.post_augmentation_filter.enabled=true requires realtime.augmentation.enabled=true."
        )
    post_augmentation_filter = build_post_augmentation_filter(
        realtime_config.post_augmentation_filter,
        frame_rate,
    )

    marker_augmenter = None
    expected_marker_names = tuple(target_selector.keypoint_names)
    if realtime_config.augmentation.enabled:
        marker_augmenter = RealtimeMarkerAugmenter(
            config_dict=realtime_config.raw_config,
            pose_model=realtime_config.pose_model,
            raw_marker_names=target_selector.keypoint_names,
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
        pose2d_estimator=target_selector,
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
        "pose2d_estimator": target_selector,
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


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    realtime_config = load_realtime_config(args.config)
    frame_range = (int(args.frame_range[0]), int(args.frame_range[1]))
    selection_source_frame = int(args.selection_source_frame)
    init_targets = _parse_init_targets(args.init_target)
    _validate_targeted_args(
        realtime_config=realtime_config,
        frame_range=frame_range,
        selection_source_frame=selection_source_frame,
        warmup=int(args.warmup),
        init_targets=init_targets,
    )

    if not realtime_config.capture.sources:
        raise FileNotFoundError(
            f"No replay videos were found for project_dir={realtime_config.project_dir}. "
            "Set realtime.capture.sources or provide videos/*.mp4."
        )
    if not realtime_config.calib_file:
        raise FileNotFoundError(
            f"No calibration .toml file was found under {Path(realtime_config.project_dir) / 'calibration'}."
        )

    frame_rate = detect_replay_frame_rate(
        realtime_config.capture.sources,
        realtime_config.capture.frame_rate,
    )
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

    log_runtime_configuration(
        source_label="video_replay_targeted",
        realtime_config=realtime_config,
        model_path=model_path,
        model_source=model_source,
        frame_rate=frame_rate,
        playback_fps=playback_fps,
    )
    logging.info(
        "Targeted replay configuration: frame_range=%s selection_source_frame=%d warmup=%d init_targets=%s",
        frame_range,
        selection_source_frame,
        int(args.warmup),
        init_targets,
    )

    target_selector = TargetedReplayPoseSelector(
        config_dict=realtime_config.raw_config,
        camera_ids=realtime_config.capture.camera_ids,
        init_targets=init_targets,
        selection_source_frame=selection_source_frame,
    )

    try:
        frame_start, frame_end = frame_range
        _run_prepass(
            target_selector,
            realtime_config,
            frame_rate=frame_rate,
            selection_source_frame=selection_source_frame,
            frame_range_start=frame_start,
            warmup=int(args.warmup),
        )

        main_source = VideoReplayFrameSource(
            sources=realtime_config.capture.sources,
            camera_ids=realtime_config.capture.camera_ids,
            frame_rate=frame_rate,
            frame_offsets=tuple(offset + frame_start for offset in realtime_config.capture.frame_offsets),
            stop_on_shortest=realtime_config.capture.stop_on_shortest,
        )
        visualizer_enabled = realtime_config.visualizer.enabled and not args.no_visualizer
        record_mot = bool(args.record_mot or realtime_config.recorder.record_mot)
        pipeline, details = _build_targeted_pipeline(
            realtime_config=realtime_config,
            frame_source=main_source,
            frame_rate=frame_rate,
            model_path=model_path,
            target_selector=target_selector,
            visualizer_enabled=visualizer_enabled,
            record_mot=record_mot,
        )
        log_pipeline_components(details)

        summary = run_pipeline_loop(
            pipeline=pipeline,
            frame_source=main_source,
            max_frames=(frame_end - frame_start + 1),
            playback_fps=playback_fps,
        )
        log_pipeline_summary(summary, finish_label="Targeted realtime replay finished")
    finally:
        target_selector.shutdown()


if __name__ == "__main__":
    main()
