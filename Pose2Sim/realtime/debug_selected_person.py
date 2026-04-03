#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Export annotated multi-camera frames showing which 2D person candidate the
current realtime pipeline would keep in each camera.

This is intended for diagnosing multi-person scenes where the single-person
realtime path may pick different people in different views.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

_runtime_dir = REPO_ROOT / ".codex_runtime"
_runtime_dir.mkdir(parents=True, exist_ok=True)
_mpl_dir = _runtime_dir / "mplconfig"
_mpl_dir.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_mpl_dir))

from Pose2Sim.common import bbox_xyxy_compute
from Pose2Sim.realtime.capture import VideoReplayFrameSource
from Pose2Sim.realtime.config import load_realtime_config
from Pose2Sim.realtime.pose2d import RealtimePoseEstimator
from Pose2Sim.realtime.runtime_support import RUNTIME_DIR


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Debug realtime single-person selection.")
    parser.add_argument("--config", required=True, help="Trial Config.toml or trial directory.")
    parser.add_argument(
        "--frames",
        nargs="+",
        type=int,
        default=[800, 900, 1000, 1100],
        help="Source frame ids to export, referenced to the anchor camera.",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=30,
        help="Number of frames to process before the first requested frame so per-camera tracking can settle.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Optional output directory. Defaults to .codex_runtime/debug_selected_person/<trial_name>/",
    )
    parser.add_argument(
        "--panel-width",
        type=int,
        default=960,
        help="Maximum width of each camera panel in the collage.",
    )
    parser.add_argument(
        "--panel-height",
        type=int,
        default=540,
        help="Maximum height of each camera panel in the collage.",
    )
    return parser.parse_args()


def _annotate_candidates(
    image: np.ndarray,
    camera_id: str,
    source_frame_id: int,
    keypoints: np.ndarray,
    scores: np.ndarray,
) -> np.ndarray:
    canvas = image.copy()
    header = f"{camera_id}  src={source_frame_id}"
    cv2.putText(canvas, header, (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(canvas, header, (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (20, 20, 20), 1, cv2.LINE_AA)

    if keypoints.size == 0 or len(keypoints) == 0:
        cv2.putText(
            canvas,
            "No detections",
            (18, 78),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return canvas

    boxes = bbox_xyxy_compute(canvas.shape, keypoints, padding=0)
    count_text = f"candidates={len(keypoints)}  selected=#0"
    cv2.putText(canvas, count_text, (18, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, count_text, (18, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (30, 30, 30), 1, cv2.LINE_AA)

    for idx, (bbox, person_keypoints, person_scores) in enumerate(zip(boxes, keypoints, scores)):
        if np.isnan(bbox).any():
            continue
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        color = (0, 0, 255) if idx == 0 else (0, 215, 255)
        thickness = 4 if idx == 0 else 2
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)

        label = f"#{idx} score={float(np.nanmean(person_scores)):.2f}"
        label_y = max(24, y1 - 10)
        cv2.putText(canvas, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        point_color = color if idx == 0 else (0, 255, 255)
        for x, y in person_keypoints:
            if np.isfinite(x) and np.isfinite(y):
                cv2.circle(canvas, (int(round(x)), int(round(y))), 3 if idx == 0 else 2, point_color, -1)

    return canvas


def _fit_panel(image: np.ndarray, max_width: int, max_height: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height)
    resized = cv2.resize(image, (max(1, int(round(width * scale))), max(1, int(round(height * scale)))))

    panel = np.zeros((max_height, max_width, 3), dtype=np.uint8)
    panel[:, :] = (18, 18, 18)
    y = (max_height - resized.shape[0]) // 2
    x = (max_width - resized.shape[1]) // 2
    panel[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return panel


def _compose_grid(images: Sequence[np.ndarray], panel_width: int, panel_height: int) -> np.ndarray:
    panels = [_fit_panel(image, panel_width, panel_height) for image in images]
    if len(panels) % 2 == 1:
        blank = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
        blank[:, :] = (18, 18, 18)
        panels = list(panels) + [blank]

    rows = []
    for start in range(0, len(panels), 2):
        rows.append(cv2.hconcat(panels[start : start + 2]))
    return cv2.vconcat(rows)


def _default_output_dir(project_dir: str) -> Path:
    return RUNTIME_DIR / "debug_selected_person" / Path(project_dir).name


def _sorted_unique_frames(frames: Iterable[int]) -> list[int]:
    requested = sorted({int(frame) for frame in frames})
    if not requested:
        raise ValueError("At least one frame id must be provided.")
    if requested[0] < 0:
        raise ValueError("Frame ids must be non-negative.")
    return requested


def main() -> None:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    realtime_config = load_realtime_config(args.config)
    requested_frames = _sorted_unique_frames(args.frames)
    anchor_camera = realtime_config.capture.camera_ids[0]
    start_reference = max(0, requested_frames[0] - int(args.warmup))
    output_dir = Path(args.output_dir) if args.output_dir else _default_output_dir(realtime_config.project_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    estimator = RealtimePoseEstimator(
        config_dict=realtime_config.raw_config,
        camera_ids=realtime_config.capture.camera_ids,
        parallel=False,
    )
    source = VideoReplayFrameSource(
        sources=realtime_config.capture.sources,
        camera_ids=realtime_config.capture.camera_ids,
        frame_rate=realtime_config.capture.frame_rate,
        frame_offsets=tuple(offset + start_reference for offset in realtime_config.capture.frame_offsets),
        stop_on_shortest=True,
    )

    logging.info(
        "Debugging selected person for %s. Requested anchor-camera frames=%s, warmup=%s, output_dir=%s",
        realtime_config.project_dir,
        requested_frames,
        args.warmup,
        output_dir,
    )

    saved = 0
    requested_set = set(requested_frames)
    try:
        source.start()
        while True:
            frame_packets = source.read()
            if not frame_packets:
                break

            by_camera = {}
            anchor_source_frame = None
            for packet in frame_packets:
                tracker = estimator._trackers[packet.camera_id]
                keypoints, scores = estimator._detect_people(tracker, packet.image)
                keypoints, scores = estimator._track_single_person(packet.camera_id, keypoints, scores)
                by_camera[packet.camera_id] = (packet, keypoints, scores)
                if packet.camera_id == anchor_camera:
                    anchor_source_frame = int(packet.metadata.get("source_frame_id", -1))

            if anchor_source_frame is None or anchor_source_frame not in requested_set:
                continue

            annotated = []
            for camera_id in realtime_config.capture.camera_ids:
                packet, keypoints, scores = by_camera[camera_id]
                source_frame_id = int(packet.metadata.get("source_frame_id", -1))
                annotated.append(_annotate_candidates(packet.image, camera_id, source_frame_id, keypoints, scores))

            grid = _compose_grid(annotated, panel_width=args.panel_width, panel_height=args.panel_height)
            output_path = output_dir / f"selected_person_frame_{anchor_source_frame:04d}.png"
            cv2.imwrite(str(output_path), grid)
            logging.info("Saved %s", output_path)
            saved += 1

            if saved >= len(requested_set):
                break
    finally:
        source.stop()
        estimator.shutdown()

    if saved == 0:
        raise RuntimeError(
            f"No requested frames were exported. Anchor camera={anchor_camera}, requested={requested_frames}."
        )
    logging.info("Done. Exported %d frame collage(s).", saved)


if __name__ == "__main__":
    main()
