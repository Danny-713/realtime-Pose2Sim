#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Realtime configuration helpers.

These helpers intentionally reuse Pose2Sim's existing configuration loading so
the realtime pipeline can start from the same project folder and Config.toml.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple, Union

from Pose2Sim.Pose2Sim import read_config_files


@dataclass(frozen=True)
class RealtimeCaptureConfig:
    source_type: str = "video"
    camera_ids: Tuple[str, ...] = tuple()
    sources: Tuple[str, ...] = tuple()
    frame_rate: float = 30.0
    batch_size: int = 1


@dataclass(frozen=True)
class RealtimeIKConfig:
    enabled: bool = True
    window_size: int = 10
    min_window_size: int = 5
    warm_start: bool = True
    model_path: Optional[str] = None
    use_simple_model: bool = True
    marker_set_name: Optional[str] = None


@dataclass(frozen=True)
class RealtimeTransportConfig:
    enabled: bool = False
    backend: str = "zmq"
    endpoint: str = "tcp://127.0.0.1:5555"
    topic: str = "opensim_state"


@dataclass(frozen=True)
class RealtimeRecorderConfig:
    enabled: bool = False
    record_markers: bool = False
    record_mot: bool = False
    output_dir: Optional[str] = None


@dataclass(frozen=True)
class RealtimeConfig:
    project_dir: str
    pose_model: str
    capture: RealtimeCaptureConfig
    ik: RealtimeIKConfig
    transport: RealtimeTransportConfig
    recorder: RealtimeRecorderConfig
    raw_config: Mapping[str, Any] = field(repr=False)


def _ensure_single_trial_config(config: Union[None, str, Mapping[str, Any]]) -> Mapping[str, Any]:
    """
    Reuse Pose2Sim's config loading but restrict realtime startup to one trial.
    """

    _, config_dicts = read_config_files(config)
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
    ik_cfg = realtime_cfg.get("ik", {})
    transport_cfg = realtime_cfg.get("transport", {})
    recorder_cfg = realtime_cfg.get("recorder", {})

    frame_rate = project_cfg.get("frame_rate", 30)
    if frame_rate in (None, "auto"):
        frame_rate = 30

    sources = _tuple_from_value(capture_cfg.get("sources"))
    camera_ids = _tuple_from_value(capture_cfg.get("camera_ids"))
    if not camera_ids and sources:
        camera_ids = tuple("cam{:02d}".format(idx + 1) for idx, _ in enumerate(sources))

    return RealtimeConfig(
        project_dir=project_cfg.get("project_dir", "."),
        pose_model=pose_cfg.get("pose_model", "HALPE_26"),
        capture=RealtimeCaptureConfig(
            source_type=capture_cfg.get("source_type", "video"),
            camera_ids=camera_ids,
            sources=sources,
            frame_rate=float(capture_cfg.get("frame_rate", frame_rate)),
            batch_size=int(capture_cfg.get("batch_size", 1)),
        ),
        ik=RealtimeIKConfig(
            enabled=bool(ik_cfg.get("enabled", True)),
            window_size=int(ik_cfg.get("window_size", 10)),
            min_window_size=int(ik_cfg.get("min_window_size", 5)),
            warm_start=bool(ik_cfg.get("warm_start", True)),
            model_path=ik_cfg.get("model_path"),
            use_simple_model=bool(
                ik_cfg.get("use_simple_model", kinematics_cfg.get("use_simple_model", True))
            ),
            marker_set_name=ik_cfg.get("marker_set_name"),
        ),
        transport=RealtimeTransportConfig(
            enabled=bool(transport_cfg.get("enabled", False)),
            backend=transport_cfg.get("backend", "zmq"),
            endpoint=transport_cfg.get("endpoint", "tcp://127.0.0.1:5555"),
            topic=transport_cfg.get("topic", "opensim_state"),
        ),
        recorder=RealtimeRecorderConfig(
            enabled=bool(recorder_cfg.get("enabled", False)),
            record_markers=bool(recorder_cfg.get("record_markers", False)),
            record_mot=bool(recorder_cfg.get("record_mot", False)),
            output_dir=recorder_cfg.get("output_dir"),
        ),
        raw_config=config_dict,
    )
