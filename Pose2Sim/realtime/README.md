# Pose2Sim Realtime V1

`Pose2Sim/realtime/` is the first production-facing realtime path that runs in
parallel to the existing offline pipeline.

Current V1 scope is intentionally narrow:

- single person
- synchronized multi-video replay
- live multi-camera capture
- same-machine OpenSim API Visualizer
- in-memory data flow from replay frames to OpenSim state packets

Out of scope for this stage:

- automatic synchronization
- OpenSim GUI integration
- server-to-client transport
- multi-person association

## Main entry point

Use `run_file_replay.py` as the V1 runtime entry point.

Replay without GUI:

```bash
conda activate Pose2Sim
python Pose2Sim/realtime/run_file_replay.py --config Pose2Sim/Demo_SinglePerson/Config.toml --no-visualizer
```

Replay with an explicit trial model for comparison:

```bash
conda activate Pose2Sim
python Pose2Sim/realtime/run_file_replay.py --config Pose2Sim/Demo_SinglePerson/Config.toml --no-visualizer --model-path Pose2Sim/Demo_SinglePerson/kinematics/Demo_SinglePerson_1-96_filt_butterworth_LSTM.osim
```

Replay with API Visualizer:

```bash
conda activate Pose2Sim
python Pose2Sim/realtime/run_file_replay.py --config Pose2Sim/Demo_SinglePerson/Config.toml
```

Force MOT recording:

```bash
conda activate Pose2Sim
python Pose2Sim/realtime/run_file_replay.py --config Pose2Sim/Demo_SinglePerson/Config.toml --no-visualizer --record-mot
```

Live capture without GUI:

```bash
conda activate Pose2Sim
python Pose2Sim/realtime/run_live_capture.py --config Pose2Sim/Demo_SinglePerson/Config.toml --no-visualizer
```

## Runtime path

The current V1 replay path is:

`videos/*.mp4 -> pose2d -> single-person multiview packet -> per-frame triangulation -> causal realtime filter -> sliding marker window -> realtime IK -> OpenSimStatePacket -> visualizer/recorder`

When `realtime.offline_like_quality.enabled = true`, realtime inserts a shared
offline-like quality mode before the current realtime filter:

`videos/*.mp4 -> pose2d -> 2D quality mask -> triangulation -> raw 3D quality gate -> short-gap fill -> Hampel-like outlier rejection -> causal realtime filter -> sliding marker window -> realtime IK`

When `realtime.augmentation.enabled = true`, the pipeline inserts a fixed-lag
LSTM augmenter between filtering and the IK buffer:

`videos/*.mp4 -> pose2d -> 2D quality mask -> triangulation -> offline-like quality processing -> causal realtime filter -> marker augmentation -> sliding marker window -> realtime IK`

When `realtime.post_augmentation_filter.enabled = true`, a lightweight causal
One Euro filter is inserted after augmentation and before the IK buffer:

`videos/*.mp4 -> pose2d -> 2D quality mask -> triangulation -> offline-like quality processing -> causal realtime filter -> marker augmentation -> post-augmentation One Euro filter -> sliding marker window -> realtime IK`

The live capture path keeps the same downstream stack and only swaps the frame
source:

`live cameras -> pose2d -> single-person multiview packet -> per-frame triangulation -> causal realtime filter -> sliding marker window -> realtime IK -> OpenSimStatePacket -> visualizer/recorder`

The main runtime modules are:

- `run_file_replay.py`: V1 replay runner
- `run_live_capture.py`: V1 live-camera runner
- `pipeline.py`: stage orchestration
- `capture.py`: replay and live multi-camera capture backends
- `pose2d.py`: single-person per-camera 2D estimation
- `triangulate_frame.py`: per-frame 3D triangulation
- `offline_like_quality.py`: optional offline-like 2D/3D quality mode before realtime filtering
- `filter_realtime.py`: causal realtime filters
- `augmentation.py`: optional fixed-lag LSTM marker augmentation
- `marker_buffer.py`: sliding window for small-window IK
- `opensim_ik.py`: rolling-window OpenSim IK
- `opensim_viz.py`: API Visualizer backend
- `recorder.py`: optional coordinate/MOT output

Support modules:

- `packets.py`: in-memory packet contracts
- `config.py`: realtime config view over `Config.toml`

## Defaults and behavior

- Replay input is auto-discovered from `project_dir/videos/*.mp4` when `realtime.capture.sources` is empty.
- Live capture reads `realtime.capture.sources` as camera indices or stream URLs when `realtime.capture.source_type = "live_camera"`.
- Calibration is auto-discovered from `project_dir/calibration/*.toml` when `realtime.calib_file` is empty.
- OpenSim model resolution is:
  - use `realtime.ik.model_path` or `--model-path` when explicitly provided
  - otherwise, if `realtime.ik.use_simple_model=true`, use the simple setup model
  - otherwise fall back to an existing `project_dir/kinematics/*.osim`
- The current V1 baseline is the simple model path; trial/complex models are kept as explicit comparison paths.
- When realtime augmentation is enabled, the body model remains the current simple `.osim`, but the IK marker set automatically switches from `HALPE_26` to `LSTM`.
- `playback_fps <= 0` means full-speed replay; no artificial slow-down is added.
- For live capture, `playback_fps <= 0` means process frames as fast as the pipeline can consume them.
- `--record-mot` forces coordinate recording and writes `realtime.mot`.
- Recorder outputs go to `project_dir/realtime_output/` by default.
- The runner prints a stage summary with capture, pose2d, triangulate, quality-mode, filter, augmentation, post-filter, and ik average timings.
- The runner also reports average valid 3D markers, average quality-gated/interpolated/outlier-replaced markers, average markers used by IK, and average reprojection error.

## Offline-like quality mode config

If realtime raw 3D differs noticeably from the offline `filt`/`LSTM` route, you
can enable a shared offline-like quality mode immediately after triangulation:

```toml
[realtime.offline_like_quality]
enabled = true
window_size = 21
min_window_size = 21
output_mode = "center"
mask_low_likelihood_2d = true
gate_high_reproj_3d = true
interp_short_gaps = true
interp_max_gap = 20
fill_large_gaps_with = "last_value"
reject_outliers = true
hampel_window_size = 7
hampel_n_sigma = 2.0
target = "all_markers"
```

Notes:

- The current implementation only supports `Body_with_feet / HALPE_26`.
- `output_mode = "center"` is currently the only supported mode.
- The quality mode masks low-confidence 2D observations before triangulation, then applies per-marker 3D gating, short-gap interpolation, and Hampel-like 1D outlier replacement on the raw 3D window.
- The goal is to bring realtime closer to the offline `triangulation -> fill -> Hampel -> filter` logic without requiring the full sequence.
- Do not enable this at the same time as the legacy `realtime.pre_augmentation_cleanup` route.

## Realtime augmentation config

Realtime marker augmentation is optional and disabled by default. The current
implementation only supports the default `Body_with_feet / HALPE_26` route and
reuses the offline LSTM augmenter assets.

Example:

```toml
[realtime.augmentation]
enabled = true
model_name = "LSTM"
model_version = "v0.3"
window_size = 15
min_window_size = 15
output_mode = "center"
feet_on_floor = false
use_subject_stats = true
```

Notes:

- `output_mode = "center"` is the default and adds fixed delay to improve stability.
- Height is reused from `project.participant_height` when numeric.
- If height is `auto`, realtime estimates it once from the first valid augmentation window and then keeps it fixed.
- The augmented packet keeps the original 3D keypoints and appends the LSTM response markers before IK.

## Post-augmentation smoothing config

If augmentation makes the richer marker set steadier overall but still leaves
small high-frequency jitter, you can insert a lightweight One Euro filter
before the IK window:

```toml
[realtime.post_augmentation_filter]
enabled = true
type = "one_euro"

  [realtime.post_augmentation_filter.one_euro]
  cut_off_frequency = 2.0
  beta = 0.5
  d_cut_off_frequency = 1.0
```

Notes:

- This stage is intentionally separate from `realtime.filtering`, which still applies to the raw triangulated 3D markers.
- The post-augmentation filter currently requires `realtime.augmentation.enabled = true`.
- The goal is lightweight de-jittering, not large additional smoothing windows.

## Stage-0 regression scripts

These scripts are kept as OpenSim/backend regression checks:

- `test_api_visualizer.py`
- `test_opensim_api_ik.py`
- `test_opensim_window_ik.py`
- `test_opensim_window_visualizer.py`

They are not the V1 production entry point.

## Common notes

- The runner creates `.codex_runtime/` under the repo root for temporary files and Matplotlib config to avoid noisy startup issues.
- If `pose.backend=auto` fails on the current machine, the realtime estimator will fall back to CPU backends and log the selected runtime backend per camera.
- GUI mode is intended for a local desktop session. Headless deployment is not part of this stage.
- Headless live benchmarking is supported via `run_live_capture.py --no-visualizer`.
- Current performance work treats simple model as the default realtime baseline; trial models may still run, but are expected to be slower.
