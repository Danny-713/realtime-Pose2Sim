# Pose2Sim Realtime Layout

This folder is the realtime branch that stays parallel to the existing offline
Pose2Sim workflow.

## Runtime-facing modules

These are the files that define the future stage-1 realtime path and should be
the first place to look:

- `pipeline.py`: realtime pipeline main entry point
- `capture.py`: frame-source contract and replay bootstrap source
- `triangulate_frame.py`: per-frame triangulation contract
- `filter_realtime.py`: causal realtime-safe filters
- `marker_buffer.py`: sliding 3D marker window for small-window IK

## Support modules

These files are still useful, but they support the runtime modules rather than
define the main architecture:

- `packets.py`: in-memory packet/data types passed between stages
- `config.py`: realtime-oriented config view built on top of `Config.toml`

## Stage-0 validation scripts

These scripts exist to validate the OpenSim backend before the full realtime
front-half is wired up:

- `test_api_visualizer.py`
- `test_opensim_api_ik.py`
- `test_opensim_window_ik.py`
- `test_opensim_window_visualizer.py`

## Current boundaries

- No changes to the existing offline pipeline
- No camera-specific backend committed yet
- No direct network transport implementation yet
- OpenSim validation is script-first; production modules come next

## Recommended next implementation order

1. Expand `capture.py` to real multi-camera sources.
2. Add `pose2d.py` and `association.py` wrappers for frame-oriented inference.
3. Replace the callable adapter in `triangulate_frame.py` with extracted offline core logic.
4. Replace the bootstrap filter in `filter_realtime.py` with Kalman or single-pass OneEuro.
5. Add `opensim_ik.py` and `visualizer.py` production modules around the validated stage-0 scripts.
