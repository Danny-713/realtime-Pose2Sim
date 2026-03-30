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
- marker augmentation
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
- `filter_realtime.py`: causal realtime filters
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
- `playback_fps <= 0` means full-speed replay; no artificial slow-down is added.
- For live capture, `playback_fps <= 0` means process frames as fast as the pipeline can consume them.
- `--record-mot` forces coordinate recording and writes `realtime.mot`.
- Recorder outputs go to `project_dir/realtime_output/` by default.
- The runner prints a stage summary with capture, pose2d, triangulate, filter, and ik average timings.
- The runner also reports average valid 3D markers, average markers used by IK, and average reprojection error.

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
