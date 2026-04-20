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
    frame_width: int = 0
    frame_height: int = 0
    buffer_size: int = 1
    read_timeout_ms: int = 1000
    max_batch_skew_ms: float = 80.0
    api_preference: int = 0


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
class RealtimeAugmentationConfig:
    enabled: bool = False
    model_name: str = "LSTM"
    model_version: str = "v0.3"
    window_size: int = 15
    min_window_size: int = 15
    output_mode: str = "center"
    feet_on_floor: bool = False
    first_frame_feet_on_floor: bool = False
    use_subject_stats: bool = True


@dataclass(frozen=True)
class RealtimeOfflineLikeQualityConfig:
    enabled: bool = False
    window_size: int = 21
    min_window_size: int = 21
    output_mode: str = "center"
    mask_low_likelihood_2d: bool = True
    gate_high_reproj_3d: bool = True
    likelihood_threshold_triangulation: float = 0.3
    reproj_error_threshold_triangulation: float = 15.0
    min_cameras_for_triangulation: int = 2
    interp_short_gaps: bool = True
    interp_max_gap: int = 20
    fill_large_gaps_with: str = "last_value"
    reject_outliers: bool = True
    hampel_window_size: int = 7
    hampel_n_sigma: float = 2.0
    target: str = "all_markers"
    parameter_source: str = "offline_defaults"


@dataclass(frozen=True)
class RealtimePostAugmentationFilterConfig:
    enabled: bool = False
    type: str = "one_euro"
    one_euro_min_cutoff: float = 2.0
    one_euro_beta: float = 0.5
    one_euro_d_cutoff: float = 1.0


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
class RealtimeFilteringConfig:
    type: str = "one_euro"
    kalman_trust_ratio: float = 5.0
    one_euro_min_cutoff: float = 4.0
    one_euro_beta: float = 1.5
    one_euro_d_cutoff: float = 1.0
    butterworth_order: int = 4
    butterworth_cutoff: float = 6.0
    butterworth_window_size: int = 30
    kalman_rts_trust_ratio: float = 5.0
    kalman_rts_window_size: int = 20


@dataclass(frozen=True)
class RealtimePreAugmentationCleanupConfig:
    enabled: bool = False
    window_size: int = 9
    min_window_size: int = 9
    output_mode: str = "center"
    max_marker_deviation_m: float = 0.10
    max_marker_velocity_m_s: float = 6.0
    max_segment_deviation_ratio: float = 0.20
    replacement: str = "interpolate"
    target: str = "limbs"


@dataclass(frozen=True)
class RealtimeConfig:
    project_dir: str
    calib_file: Optional[str]
    pose_model: str
    capture: RealtimeCaptureConfig
    pose: RealtimePoseConfig
    pre_augmentation_cleanup: RealtimePreAugmentationCleanupConfig
    offline_like_quality: RealtimeOfflineLikeQualityConfig
    filtering: RealtimeFilteringConfig
    augmentation: RealtimeAugmentationConfig
    post_augmentation_filter: RealtimePostAugmentationFilterConfig
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
    filtering_cfg = realtime_cfg.get("filtering", {})
    pre_cleanup_cfg = realtime_cfg.get("pre_augmentation_cleanup", {})
    offline_like_quality_cfg = realtime_cfg.get("offline_like_quality", {})
    augmentation_cfg = realtime_cfg.get("augmentation", {})
    post_augmentation_filter_cfg = realtime_cfg.get("post_augmentation_filter", {})
    ik_cfg = realtime_cfg.get("ik", {})
    visualizer_cfg = realtime_cfg.get("visualizer", {})
    recorder_cfg = realtime_cfg.get("recorder", {})
    triangulation_cfg = config_dict.get("triangulation", {})
    offline_filtering_cfg = config_dict.get("filtering", {})

    project_dir = str(Path(project_cfg.get("project_dir", ".")).resolve())

    frame_rate = project_cfg.get("frame_rate", 0)
    if frame_rate in (None, "auto"):
        frame_rate = 0

    vid_img_extension = pose_cfg.get("vid_img_extension", "mp4")

    source_type = str(capture_cfg.get("source_type", "video_replay")).lower()

    sources = _tuple_from_value(capture_cfg.get("sources"))
    if not sources and source_type == "video_replay":
        sources = _discover_video_sources(project_dir, vid_img_extension)
    camera_ids = _tuple_from_value(capture_cfg.get("camera_ids"))
    if not camera_ids and sources:
        if source_type == "live_camera":
            camera_ids = tuple(f"cam{index + 1:02d}" for index in range(len(sources)))
        else:
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

    rt_pre_cleanup = RealtimePreAugmentationCleanupConfig(
        enabled=bool(pre_cleanup_cfg.get("enabled", False)),
        window_size=int(pre_cleanup_cfg.get("window_size", 9)),
        min_window_size=int(pre_cleanup_cfg.get("min_window_size", 9)),
        output_mode=str(pre_cleanup_cfg.get("output_mode", "center")).lower(),
        max_marker_deviation_m=float(pre_cleanup_cfg.get("max_marker_deviation_m", 0.10)),
        max_marker_velocity_m_s=float(pre_cleanup_cfg.get("max_marker_velocity_m_s", 6.0)),
        max_segment_deviation_ratio=float(pre_cleanup_cfg.get("max_segment_deviation_ratio", 0.20)),
        replacement=str(pre_cleanup_cfg.get("replacement", "interpolate")).lower(),
        target=str(pre_cleanup_cfg.get("target", "limbs")).lower(),
    )
    enriched_rt_pre_cleanup: Dict[str, Any] = dict(enriched_rt.get("pre_augmentation_cleanup", {}))
    enriched_rt_pre_cleanup.setdefault("enabled", rt_pre_cleanup.enabled)
    enriched_rt_pre_cleanup.setdefault("window_size", rt_pre_cleanup.window_size)
    enriched_rt_pre_cleanup.setdefault("min_window_size", rt_pre_cleanup.min_window_size)
    enriched_rt_pre_cleanup.setdefault("output_mode", rt_pre_cleanup.output_mode)
    enriched_rt_pre_cleanup.setdefault("max_marker_deviation_m", rt_pre_cleanup.max_marker_deviation_m)
    enriched_rt_pre_cleanup.setdefault("max_marker_velocity_m_s", rt_pre_cleanup.max_marker_velocity_m_s)
    enriched_rt_pre_cleanup.setdefault("max_segment_deviation_ratio", rt_pre_cleanup.max_segment_deviation_ratio)
    enriched_rt_pre_cleanup.setdefault("replacement", rt_pre_cleanup.replacement)
    enriched_rt_pre_cleanup.setdefault("target", rt_pre_cleanup.target)
    enriched_rt["pre_augmentation_cleanup"] = enriched_rt_pre_cleanup

    quality_source_keys = {
        "likelihood_threshold_triangulation",
        "reproj_error_threshold_triangulation",
        "min_cameras_for_triangulation",
        "interp_max_gap",
        "fill_large_gaps_with",
        "reject_outliers",
        "hampel_window_size",
        "hampel_n_sigma",
    }
    rt_offline_like_quality = RealtimeOfflineLikeQualityConfig(
        enabled=bool(offline_like_quality_cfg.get("enabled", False)),
        window_size=int(offline_like_quality_cfg.get("window_size", 21)),
        min_window_size=int(offline_like_quality_cfg.get("min_window_size", 21)),
        output_mode=str(offline_like_quality_cfg.get("output_mode", "center")).lower(),
        mask_low_likelihood_2d=bool(offline_like_quality_cfg.get("mask_low_likelihood_2d", True)),
        gate_high_reproj_3d=bool(offline_like_quality_cfg.get("gate_high_reproj_3d", True)),
        likelihood_threshold_triangulation=float(
            offline_like_quality_cfg.get(
                "likelihood_threshold_triangulation",
                triangulation_cfg.get("likelihood_threshold_triangulation", 0.3),
            )
        ),
        reproj_error_threshold_triangulation=float(
            offline_like_quality_cfg.get(
                "reproj_error_threshold_triangulation",
                triangulation_cfg.get("reproj_error_threshold_triangulation", 15.0),
            )
        ),
        min_cameras_for_triangulation=int(
            offline_like_quality_cfg.get(
                "min_cameras_for_triangulation",
                triangulation_cfg.get("min_cameras_for_triangulation", 2),
            )
        ),
        interp_short_gaps=bool(offline_like_quality_cfg.get("interp_short_gaps", True)),
        interp_max_gap=int(
            offline_like_quality_cfg.get(
                "interp_max_gap",
                triangulation_cfg.get("interp_if_gap_smaller_than", 20),
            )
        ),
        fill_large_gaps_with=str(
            offline_like_quality_cfg.get(
                "fill_large_gaps_with",
                triangulation_cfg.get("fill_large_gaps_with", "last_value"),
            )
        ).lower(),
        reject_outliers=bool(
            offline_like_quality_cfg.get(
                "reject_outliers",
                offline_filtering_cfg.get("reject_outliers", True),
            )
        ),
        hampel_window_size=int(offline_like_quality_cfg.get("hampel_window_size", 7)),
        hampel_n_sigma=float(offline_like_quality_cfg.get("hampel_n_sigma", 2.0)),
        target=str(offline_like_quality_cfg.get("target", "all_markers")).lower(),
        parameter_source=(
            "realtime_overrides"
            if any(key in offline_like_quality_cfg for key in quality_source_keys)
            else "offline_defaults"
        ),
    )
    enriched_rt_offline_like_quality: Dict[str, Any] = dict(enriched_rt.get("offline_like_quality", {}))
    enriched_rt_offline_like_quality.setdefault("enabled", rt_offline_like_quality.enabled)
    enriched_rt_offline_like_quality.setdefault("window_size", rt_offline_like_quality.window_size)
    enriched_rt_offline_like_quality.setdefault("min_window_size", rt_offline_like_quality.min_window_size)
    enriched_rt_offline_like_quality.setdefault("output_mode", rt_offline_like_quality.output_mode)
    enriched_rt_offline_like_quality.setdefault(
        "mask_low_likelihood_2d", rt_offline_like_quality.mask_low_likelihood_2d
    )
    enriched_rt_offline_like_quality.setdefault(
        "gate_high_reproj_3d", rt_offline_like_quality.gate_high_reproj_3d
    )
    enriched_rt_offline_like_quality.setdefault(
        "likelihood_threshold_triangulation",
        rt_offline_like_quality.likelihood_threshold_triangulation,
    )
    enriched_rt_offline_like_quality.setdefault(
        "reproj_error_threshold_triangulation",
        rt_offline_like_quality.reproj_error_threshold_triangulation,
    )
    enriched_rt_offline_like_quality.setdefault(
        "min_cameras_for_triangulation",
        rt_offline_like_quality.min_cameras_for_triangulation,
    )
    enriched_rt_offline_like_quality.setdefault("interp_short_gaps", rt_offline_like_quality.interp_short_gaps)
    enriched_rt_offline_like_quality.setdefault("interp_max_gap", rt_offline_like_quality.interp_max_gap)
    enriched_rt_offline_like_quality.setdefault(
        "fill_large_gaps_with", rt_offline_like_quality.fill_large_gaps_with
    )
    enriched_rt_offline_like_quality.setdefault("reject_outliers", rt_offline_like_quality.reject_outliers)
    enriched_rt_offline_like_quality.setdefault(
        "hampel_window_size", rt_offline_like_quality.hampel_window_size
    )
    enriched_rt_offline_like_quality.setdefault("hampel_n_sigma", rt_offline_like_quality.hampel_n_sigma)
    enriched_rt_offline_like_quality.setdefault("target", rt_offline_like_quality.target)
    enriched_rt["offline_like_quality"] = enriched_rt_offline_like_quality

    rt_augmentation = RealtimeAugmentationConfig(
        enabled=bool(augmentation_cfg.get("enabled", False)),
        model_name=str(augmentation_cfg.get("model_name", "LSTM")),
        model_version=str(augmentation_cfg.get("model_version", "v0.3")),
        window_size=int(augmentation_cfg.get("window_size", 15)),
        min_window_size=int(augmentation_cfg.get("min_window_size", 15)),
        output_mode=str(augmentation_cfg.get("output_mode", "center")).lower(),
        feet_on_floor=bool(augmentation_cfg.get("feet_on_floor", False)),
        first_frame_feet_on_floor=bool(augmentation_cfg.get("first_frame_feet_on_floor", False)),
        use_subject_stats=bool(augmentation_cfg.get("use_subject_stats", True)),
    )
    enriched_rt_augmentation: Dict[str, Any] = dict(enriched_rt.get("augmentation", {}))
    enriched_rt_augmentation.setdefault("enabled", rt_augmentation.enabled)
    enriched_rt_augmentation.setdefault("model_name", rt_augmentation.model_name)
    enriched_rt_augmentation.setdefault("model_version", rt_augmentation.model_version)
    enriched_rt_augmentation.setdefault("window_size", rt_augmentation.window_size)
    enriched_rt_augmentation.setdefault("min_window_size", rt_augmentation.min_window_size)
    enriched_rt_augmentation.setdefault("output_mode", rt_augmentation.output_mode)
    enriched_rt_augmentation.setdefault("feet_on_floor", rt_augmentation.feet_on_floor)
    enriched_rt_augmentation.setdefault(
        "first_frame_feet_on_floor", rt_augmentation.first_frame_feet_on_floor
    )
    enriched_rt_augmentation.setdefault("use_subject_stats", rt_augmentation.use_subject_stats)
    enriched_rt["augmentation"] = enriched_rt_augmentation

    rt_post_augmentation_filter_one_euro = post_augmentation_filter_cfg.get("one_euro", {})
    rt_post_augmentation_filter = RealtimePostAugmentationFilterConfig(
        enabled=bool(post_augmentation_filter_cfg.get("enabled", False)),
        type=str(post_augmentation_filter_cfg.get("type", "one_euro")).lower(),
        one_euro_min_cutoff=float(
            rt_post_augmentation_filter_one_euro.get("cut_off_frequency", 2.0)
        ),
        one_euro_beta=float(rt_post_augmentation_filter_one_euro.get("beta", 0.5)),
        one_euro_d_cutoff=float(
            rt_post_augmentation_filter_one_euro.get("d_cut_off_frequency", 1.0)
        ),
    )
    enriched_rt_post_augmentation_filter: Dict[str, Any] = dict(
        enriched_rt.get("post_augmentation_filter", {})
    )
    enriched_rt_post_augmentation_filter.setdefault("enabled", rt_post_augmentation_filter.enabled)
    enriched_rt_post_augmentation_filter.setdefault("type", rt_post_augmentation_filter.type)
    enriched_rt_post_augmentation_filter_one_euro: Dict[str, Any] = dict(
        enriched_rt_post_augmentation_filter.get("one_euro", {})
    )
    enriched_rt_post_augmentation_filter_one_euro.setdefault(
        "cut_off_frequency", rt_post_augmentation_filter.one_euro_min_cutoff
    )
    enriched_rt_post_augmentation_filter_one_euro.setdefault(
        "beta", rt_post_augmentation_filter.one_euro_beta
    )
    enriched_rt_post_augmentation_filter_one_euro.setdefault(
        "d_cut_off_frequency", rt_post_augmentation_filter.one_euro_d_cutoff
    )
    enriched_rt_post_augmentation_filter["one_euro"] = enriched_rt_post_augmentation_filter_one_euro
    enriched_rt["post_augmentation_filter"] = enriched_rt_post_augmentation_filter
    enriched_config["realtime"] = enriched_rt

    # Build filtering config from [realtime.filtering] with per-type sub-tables.
    rt_filtering_kalman = filtering_cfg.get("kalman", {})
    rt_filtering_one_euro = filtering_cfg.get("one_euro", {})
    rt_filtering_butterworth = filtering_cfg.get("butterworth", {})
    rt_filtering_kalman_rts = filtering_cfg.get("kalman_rts", {})
    rt_filtering = RealtimeFilteringConfig(
        type=str(filtering_cfg.get("type", "one_euro")).lower(),
        kalman_trust_ratio=float(rt_filtering_kalman.get("trust_ratio", 5.0)),
        one_euro_min_cutoff=float(rt_filtering_one_euro.get("cut_off_frequency", 4.0)),
        one_euro_beta=float(rt_filtering_one_euro.get("beta", 1.5)),
        one_euro_d_cutoff=float(rt_filtering_one_euro.get("d_cut_off_frequency", 1.0)),
        butterworth_order=int(rt_filtering_butterworth.get("order", 4)),
        butterworth_cutoff=float(rt_filtering_butterworth.get("cut_off_frequency", 6.0)),
        butterworth_window_size=int(rt_filtering_butterworth.get("window_size", 30)),
        kalman_rts_trust_ratio=float(rt_filtering_kalman_rts.get("trust_ratio", 5.0)),
        kalman_rts_window_size=int(rt_filtering_kalman_rts.get("window_size", 20)),
    )

    if rt_pre_cleanup.enabled and rt_offline_like_quality.enabled:
        raise ValueError(
            "realtime.pre_augmentation_cleanup.enabled=true cannot be combined with "
            "realtime.offline_like_quality.enabled=true. Please enable only one pre-cleaning route."
        )

    return RealtimeConfig(
        project_dir=project_dir,
        calib_file=calib_file,
        pose_model=pose_cfg.get("pose_model", "HALPE_26"),
        capture=RealtimeCaptureConfig(
            source_type=source_type,
            camera_ids=camera_ids,
            sources=sources,
            frame_rate=float(capture_cfg.get("frame_rate", frame_rate)),
            frame_offsets=frame_offsets,
            stop_on_shortest=bool(capture_cfg.get("stop_on_shortest", True)),
            frame_width=int(capture_cfg.get("frame_width", 0)),
            frame_height=int(capture_cfg.get("frame_height", 0)),
            buffer_size=int(capture_cfg.get("buffer_size", 1)),
            read_timeout_ms=int(capture_cfg.get("read_timeout_ms", 1000)),
            max_batch_skew_ms=float(capture_cfg.get("max_batch_skew_ms", 80.0)),
            api_preference=int(capture_cfg.get("api_preference", 0)),
        ),
        pose=rt_pose,
        pre_augmentation_cleanup=rt_pre_cleanup,
        offline_like_quality=rt_offline_like_quality,
        filtering=rt_filtering,
        augmentation=rt_augmentation,
        post_augmentation_filter=rt_post_augmentation_filter,
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
