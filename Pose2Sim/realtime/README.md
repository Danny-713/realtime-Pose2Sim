# Pose2Sim Realtime Bootstrap

This folder is the starting point for a realtime pipeline that stays parallel to
the existing offline Pose2Sim workflow.

Current scope:

- `types.py`: in-memory packet types passed between realtime stages
- `config.py`: a realtime-focused config view built on top of the existing
  `Config.toml` loading logic
- `marker_buffer.py`: rolling 3D marker buffer for small-window IK
- `pipeline.py`: orchestration skeleton that wires realtime stages together

Intentional non-goals in this first step:

- No change to the existing offline pipeline
- No hard dependency on a specific camera backend
- No direct OpenSim API binding logic yet
- No network transport implementation yet

Recommended next implementation order:

1. Add `capture.py` for live or file-backed multi-camera sources.
2. Add `pose2d.py` with a frame-oriented wrapper around the existing RTMLib setup.
3. Add `association.py` and `triangulation.py` wrappers that reuse offline core
   math but avoid file IO.
4. Add a realtime-safe filter in `filtering.py`.
5. Add `opensim_model.py` and `opensim_ik.py` for small-window IK.
6. Add `transport.py` and a local viewer client if server-side execution stays
   headless.
