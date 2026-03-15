#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime configuration helpers.

These helpers intentionally reuse Pose2Sim's existing configuration loading so
the realtime pipeline can start from the same project folder and Config.toml.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple, Union

from Pose2Sim.Pose2Sim import read_config_files
from Pose2Sim.common import natural_sort_key


@dataclass(frozen=True)
class RealtimeCaptureConfig:
    source_type: str = "video_replay"
    camera_ids: Tuple[str, ...] = tuple()
    sources: Tuple[str, ...] = tuple()
    frame_rate: float = 0.0
    frame_offsets: Tuple[int, ...] = tuple()
    stop_on_shortest: bool = True


@dataclass(frozen=True)
class RealtimeIKConfig:
    enabled: bool = True
    window_size: int = 10
    min_window_size: int = 10
    warm_start: bool = True
    model_path: Optional[str] = None
    use_simple_model: bool = True
    marker_set_name: Optional[str] = None


@dataclass(frozen=True)
class RealtimeVisualizerConfig:
    enabled: bool = True
    playback_fps: float = 0.0


@dataclass(frozen=True)
class RealtimePoseConfig:
    mode: Optional[str] = "lightweight"
    det_frequency: int = 10
    backend: Optional[str] = None
    device: Optional[str] = None
    parallel: bool = True


@dataclass(frozen=True)
class RealtimeRecorderConfig:
    enabled: bool = False
    record_markers: bool = False
    record_mot: bool = False
    output_dir: Optional[str] = None


@dataclass(frozen=True)
class RealtimeConfig:
    project_dir: str
    calib_file: Optional[str]
    pose_model: str
    capture: RealtimeCaptureConfig
    pose: RealtimePoseConfig
    ik: RealtimeIKConfig
    visualizer: RealtimeVisualizerConfig
    recorder: RealtimeRecorderConfig
    raw_config: Mapping[str, Any] = field(repr=False)


def _ensure_single_trial_config(config: Union[None, str, Mapping[str, Any]]) -> Mapping[str, Any]:
    """
    Reuse Pose2Sim's config loading but restrict realtime startup to one trial.
    """

    config_arg = config
    if isinstance(config, str):
        config_path = Path(config)
        if config_path.is_file():
            config_arg = str(config_path.parent)

    _, config_dicts = read_config_files(config_arg)
    if len(config_dicts) != 1:
        raise ValueError(
            "Realtime startup currently expects a single-trial config. "
            "Please point to one trial directory or pass a single config dict."
        )
    return config_dicts[0]


def _tuple_from_value(value: Any) -> Tuple[str, ...]:
    if value in (None, "", []):
        return tuple()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence):
        return tuple(str(item) for item in value)
    return (str(value),)


def _int_tuple_from_value(value: Any) -> Tuple[int, ...]:
    if value in (None, "", []):
        return tuple()
    if isinstance(value, (int, float)):
        return (int(value),)
    if isinstance(value, Sequence) and not isinstance(value, str):
        return tuple(int(item) for item in value)
    return (int(value),)


def _discover_video_sources(project_dir: str, extension: str) -> Tuple[str, ...]:
    videos_dir = Path(project_dir) / "videos"
    if not videos_dir.exists():
        return tuple()
    pattern = f"*.{str(extension).lstrip('.')}"
    sources = sorted((str(path.resolve()) for path in videos_dir.glob(pattern)), key=natural_sort_key)
    return tuple(sources)


def _discover_calibration_file(project_dir: str) -> Optional[str]:
    calibration_dir = Path(project_dir) / "calibration"
    if not calibration_dir.exists():
        return None
    candidates = sorted((path for path in calibration_dir.glob("*.toml")), key=lambda path: path.name.lower())
    return str(candidates[0].resolve()) if candidates else None


def _default_output_dir(project_dir: str) -> str:
    return str((Path(project_dir) / "realtime_output").resolve())


def load_realtime_config(config: Union[None, str, Mapping[str, Any]] = None) -> RealtimeConfig:
    """
    Build a realtime-focused config view from an existing Pose2Sim config.
    """

    config_dict = _ensure_single_trial_config(config)
    project_cfg = config_dict.get("project", {})
    pose_cfg = config_dict.get("pose", {})
    kinematics_cfg = config_dict.get("kinematics", {})
    realtime_cfg = config_dict.get("realtime", {})

    capture_cfg = realtime_cfg.get("capture", {})
    rt_pose_cfg = realtime_cfg.get("pose", {})
    ik_cfg = realtime_cfg.get("ik", {})
    visualizer_cfg = realtime_cfg.get("visualizer", {})
    recorder_cfg = realtime_cfg.get("recorder", {})

    project_dir = str(Path(project_cfg.get("project_dir", ".")).resolve())

    frame_rate = project_cfg.get("frame_rate", 0)
    if frame_rate in (None, "auto"):
        frame_rate = 0

    vid_img_extension = pose_cfg.get("vid_img_extension", "mp4")

    sources = _tuple_from_value(capture_cfg.get("sources"))
    if not sources:
        sources = _discover_video_sources(project_dir, vid_img_extension)
    camera_ids = _tuple_from_value(capture_cfg.get("camera_ids"))
    if not camera_ids and sources:
        camera_ids = tuple(Path(source).stem for source in sources)
    frame_offsets = _int_tuple_from_value(capture_cfg.get("frame_offsets"))
    if not frame_offsets and camera_ids:
        frame_offsets = tuple(0 for _ in camera_ids)
    if frame_offsets and camera_ids and len(frame_offsets) != len(camera_ids):
        raise ValueError("realtime.capture.frame_offsets must match realtime.capture.camera_ids length.")

    calib_file = realtime_cfg.get("calib_file") or _discover_calibration_file(project_dir)
    recorder_output_dir = recorder_cfg.get("output_dir") or _default_output_dir(project_dir)

    rt_pose = RealtimePoseConfig(
        mode=rt_pose_cfg.get("mode", "lightweight"),
        det_frequency=int(rt_pose_cfg.get("det_frequency", 10)),
        backend=rt_pose_cfg.get("backend") or None,
        device=rt_pose_cfg.get("device") or None,
        parallel=bool(rt_pose_cfg.get("parallel", True)),
    )

    # Inject resolved realtime.pose defaults into raw_config so that
    # downstream consumers (e.g. RealtimePoseEstimator) see them even when
    # the user did not explicitly write a [realtime.pose] section.
    enriched_config: Dict[str, Any] = dict(config_dict)
    enriched_rt: Dict[str, Any] = dict(enriched_config.get("realtime", {}))
    enriched_rt_pose: Dict[str, Any] = dict(enriched_rt.get("pose", {}))
    enriched_rt_pose.setdefault("mode", rt_pose.mode)
    enriched_rt_pose.setdefault("det_frequency", rt_pose.det_frequency)
    if rt_pose.backend:
        enriched_rt_pose.setdefault("backend", rt_pose.backend)
    if rt_pose.device:
        enriched_rt_pose.setdefault("device", rt_pose.device)
    enriched_rt_pose.setdefault("parallel", rt_pose.parallel)
    enriched_rt["pose"] = enriched_rt_pose
    enriched_config["realtime"] = enriched_rt

    return RealtimeConfig(
        project_dir=project_dir,
        calib_file=calib_file,
        pose_model=pose_cfg.get("pose_model", "HALPE_26"),
        capture=RealtimeCaptureConfig(
            source_type=capture_cfg.get("source_type", "video_replay"),
            camera_ids=camera_ids,
            sources=sources,
            frame_rate=float(capture_cfg.get("frame_rate", frame_rate)),
            frame_offsets=frame_offsets,
            stop_on_shortest=bool(capture_cfg.get("stop_on_shortest", True)),
        ),
        pose=rt_pose,
        ik=RealtimeIKConfig(
            enabled=bool(ik_cfg.get("enabled", True)),
            window_size=int(ik_cfg.get("window_size", 10)),
            min_window_size=int(ik_cfg.get("min_window_size", ik_cfg.get("window_size", 10))),
            warm_start=bool(ik_cfg.get("warm_start", True)),
            model_path=ik_cfg.get("model_path") or None,
            use_simple_model=bool(
                ik_cfg.get("use_simple_model", kinematics_cfg.get("use_simple_model", True))
            ),
            marker_set_name=ik_cfg.get("marker_set_name"),
        ),
        visualizer=RealtimeVisualizerConfig(
            enabled=bool(visualizer_cfg.get("enabled", True)),
            playback_fps=float(visualizer_cfg.get("playback_fps", 0)),
        ),
        recorder=RealtimeRecorderConfig(
            enabled=bool(recorder_cfg.get("enabled", False)),
            record_markers=bool(recorder_cfg.get("record_markers", False)),
            record_mot=bool(recorder_cfg.get("record_mot", False)),
            output_dir=recorder_output_dir,
        ),
        raw_config=enriched_config,
    )
