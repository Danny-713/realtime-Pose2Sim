#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Frame-oriented 2D pose estimation for the realtime pipeline.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Sequence, Tuple

import numpy as np
from anytree import RenderTree
from rtmlib.tools.object_detection.post_processings import nms

from Pose2Sim.common import bbox_xyxy_compute, sort_people_sports2d
from Pose2Sim.poseEstimation import setup_model_class_mode, setup_pose_tracker
from Pose2Sim.realtime.packets import FramePacket, Pose2DPacket


class RealtimePoseEstimator:
    """
    Single-person 2D estimator for replayed multi-camera frames.

    Each camera gets its own PoseTracker instance and tracking state. When
    multiple people are detected, the first tracker-sorted person is retained.
    """

    def __init__(
        self,
        config_dict,
        camera_ids: Sequence[str],
    ):
        pose_cfg = config_dict.get("pose", {})

        self.pose_model_name = pose_cfg.get("pose_model", "Body_with_feet")
        self.tracking_mode = str(pose_cfg.get("tracking_mode", "sports2d")).lower()
        self.max_distance_px = float(pose_cfg.get("max_distance_px", 100))
        self.backend = pose_cfg.get("backend", "auto")
        self.device = pose_cfg.get("device", "auto")
        self.det_frequency = int(pose_cfg.get("det_frequency", 1))
        self.mode = pose_cfg.get("mode", "balanced")

        self.pose_model, model_class, resolved_mode = setup_model_class_mode(
            self.pose_model_name,
            self.mode,
            config_dict,
        )
        self._resolved_mode = resolved_mode
        self.num_keypoints, self.keypoint_names = self._build_keypoint_layout(self.pose_model)

        if self.tracking_mode not in {"none", "sports2d"}:
            logging.warning(
                "RealtimePoseEstimator currently supports tracking_mode 'none' and 'sports2d'. "
                "Falling back to 'sports2d'."
            )
            self.tracking_mode = "sports2d"

        self._trackers = {}
        self._runtime_choices: Dict[str, Tuple[str, str]] = {}
        fallback_candidates = self._build_fallback_candidates()
        logging.info(
            "RealtimePoseEstimator setup: pose_model=%s mode=%s tracking=%s fallback_chain=%s",
            self.pose_model_name,
            resolved_mode,
            self.tracking_mode,
            fallback_candidates,
        )
        for camera_id in camera_ids:
            tracker, selected_backend, selected_device = self._initialize_tracker(
                camera_id,
                model_class,
                resolved_mode,
                fallback_candidates,
            )
            self._trackers[camera_id] = tracker
            self._runtime_choices[camera_id] = (selected_backend, selected_device)
            logging.info(
                "RealtimePoseEstimator ready for %s with backend=%s device=%s.",
                camera_id,
                selected_backend,
                selected_device,
            )
        self._prev_keypoints: Dict[str, np.ndarray] = {}

    def _build_fallback_candidates(self) -> List[Tuple[str, str]]:
        fallback_candidates = [
            (self.backend, self.device),
            ("onnxruntime", "cpu"),
            ("openvino", "cpu"),
        ]
        deduped: List[Tuple[str, str]] = []
        for candidate in fallback_candidates:
            if candidate not in deduped:
                deduped.append(candidate)
        return deduped

    def _initialize_tracker(
        self,
        camera_id: str,
        model_class,
        resolved_mode: str,
        fallback_candidates: Sequence[Tuple[str, str]],
    ):
        last_error = None
        for backend, device in fallback_candidates:
            try:
                tracker = setup_pose_tracker(
                    model_class,
                    self.det_frequency,
                    resolved_mode,
                    False,
                    backend,
                    device,
                )
                return tracker, backend, device
            except Exception as exc:
                last_error = exc
                logging.warning(
                    "Failed to initialize realtime pose tracker with backend=%s device=%s for %s: %s.",
                    backend,
                    device,
                    camera_id,
                    exc,
                )
        raise RuntimeError(f"Could not initialize realtime pose tracker for {camera_id}: {last_error}")

    @staticmethod
    def _build_keypoint_layout(pose_model) -> Tuple[int, Tuple[str, ...]]:
        keypoint_ids = [node.id for _, _, node in RenderTree(pose_model) if node.id is not None]
        if not keypoint_ids:
            raise ValueError("Pose model does not expose any keypoint ids.")
        num_keypoints = max(keypoint_ids) + 1
        keypoint_names = [f"kpt_{idx:02d}" for idx in range(num_keypoints)]
        for _, _, node in RenderTree(pose_model):
            if node.id is not None:
                keypoint_names[node.id] = str(node.name)
        return num_keypoints, tuple(keypoint_names)

    def infer(self, frame_packet: FramePacket) -> Pose2DPacket:
        tracker = self._trackers[frame_packet.camera_id]
        keypoints, scores = self._detect_people(tracker, frame_packet.image)
        keypoints, scores = self._track_single_person(frame_packet.camera_id, keypoints, scores)
        pose_keypoints, pose_scores = self._select_single_person(keypoints, scores)

        return Pose2DPacket(
            frame_id=frame_packet.frame_id,
            timestamp=frame_packet.timestamp,
            camera_id=frame_packet.camera_id,
            keypoints=pose_keypoints,
            scores=pose_scores,
            person_id=0,
            metadata={"detected_people": int(len(keypoints))},
        )

    def infer_batch(self, frame_packets: Sequence[FramePacket]) -> List[Pose2DPacket]:
        return [self.infer(packet) for packet in frame_packets]

    @property
    def runtime_choices(self) -> Dict[str, Tuple[str, str]]:
        return dict(self._runtime_choices)

    def _detect_people(self, tracker, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            return self._empty_detections()
        try:
            keypoints, scores = tracker(frame)
            keypoints = np.asarray(keypoints, dtype=float)
            scores = np.asarray(scores, dtype=float)
        except Exception:
            return self._empty_detections()

        if keypoints.size == 0 or scores.size == 0:
            return self._empty_detections()
        if keypoints.ndim != 3 or scores.ndim != 2:
            return self._empty_detections()
        if keypoints.shape[0] != scores.shape[0]:
            return self._empty_detections()

        mask_scores = np.nanmean(scores, axis=1) > 0.2
        likely_keypoints = np.where(mask_scores[:, np.newaxis, np.newaxis], keypoints, np.nan)
        likely_scores = np.where(mask_scores[:, np.newaxis], scores, np.nan)

        frame_shape = frame.shape
        likely_bboxes = bbox_xyxy_compute(frame_shape, likely_keypoints, padding=0)
        score_likely_bboxes = np.nanmean(likely_scores, axis=1)

        valid_indices = np.where(~np.isnan(score_likely_bboxes))[0]
        if len(valid_indices) == 0:
            return self._empty_detections()

        valid_bboxes = likely_bboxes[valid_indices]
        valid_scores = score_likely_bboxes[valid_indices]
        keep_valid = nms(valid_bboxes, valid_scores, nms_thr=0.45)
        keep = valid_indices[keep_valid]
        return likely_keypoints[keep], likely_scores[keep]

    def _track_single_person(
        self,
        camera_id: str,
        keypoints: np.ndarray,
        scores: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if self.tracking_mode == "none":
            return keypoints, scores

        previous = self._prev_keypoints.get(camera_id)
        if previous is None:
            previous = keypoints
        previous, keypoints, scores = sort_people_sports2d(
            previous,
            keypoints,
            scores=scores,
            max_dist=self.max_distance_px,
        )
        self._prev_keypoints[camera_id] = previous
        return keypoints, scores

    def _select_single_person(
        self,
        keypoints: np.ndarray,
        scores: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if keypoints.size == 0 or len(keypoints) == 0:
            return self._empty_person()

        selected_keypoints = np.asarray(keypoints[0], dtype=float)
        selected_scores = np.asarray(scores[0], dtype=float)

        if selected_keypoints.shape != (self.num_keypoints, 2):
            padded_keypoints, padded_scores = self._empty_person()
            limit = min(self.num_keypoints, selected_keypoints.shape[0])
            padded_keypoints[:limit] = selected_keypoints[:limit]
            padded_scores[:limit] = selected_scores[:limit]
            return padded_keypoints, padded_scores
        if selected_scores.shape != (self.num_keypoints,):
            padded_keypoints, padded_scores = self._empty_person()
            limit = min(self.num_keypoints, selected_keypoints.shape[0], selected_scores.shape[0])
            padded_keypoints[:limit] = selected_keypoints[:limit]
            padded_scores[:limit] = selected_scores[:limit]
            return padded_keypoints, padded_scores

        return selected_keypoints, selected_scores

    def _empty_detections(self) -> Tuple[np.ndarray, np.ndarray]:
        return (
            np.empty((0, self.num_keypoints, 2), dtype=float),
            np.empty((0, self.num_keypoints), dtype=float),
        )

    def _empty_person(self) -> Tuple[np.ndarray, np.ndarray]:
        return (
            np.full((self.num_keypoints, 2), np.nan, dtype=float),
            np.full((self.num_keypoints,), np.nan, dtype=float),
        )
