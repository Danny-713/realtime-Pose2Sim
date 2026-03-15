#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
OpenSim API Visualizer backend for realtime state display.
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

import opensim as osim

from Pose2Sim.realtime.packets import OpenSimStatePacket


def ensure_simbody_visualizer_on_path() -> None:
    prefix = os.environ.get("CONDA_PREFIX") or sys.prefix
    candidates = [
        Path(prefix) / "libexec" / "simbody",
        Path(prefix) / "simbody" / "libexec" / "simbody",
    ]
    existing = [str(path) for path in candidates if path.exists()]
    if not existing:
        return

    current = os.environ.get("PATH", "")
    current_parts = current.split(os.pathsep) if current else []
    prepend = [path for path in existing if path not in current_parts]
    if prepend:
        os.environ["PATH"] = os.pathsep.join(prepend + current_parts)


class OpenSimVisualizer:
    """
    API Visualizer wrapper driven by coordinate values in OpenSimStatePacket.
    """

    def __init__(self, model_path: str, osim_setup_dir: Path):
        ensure_simbody_visualizer_on_path()

        self.model_path = str(Path(model_path).resolve())
        self.osim_setup_dir = Path(osim_setup_dir).resolve()
        geometry_dir = self.osim_setup_dir / "Geometry"
        if geometry_dir.exists():
            osim.ModelVisualizer.addDirToGeometrySearchPaths(str(geometry_dir))

        self.model = osim.Model(self.model_path)
        self.model.setUseVisualizer(True)
        self.state = self.model.initSystem()
        self.visualizer = self.model.getVisualizer()
        self.model.realizePosition(self.state)
        self.visualizer.show(self.state)
        logging.info("OpenSimVisualizer ready for model=%s", self.model_path)

    def show(self, state_packet: OpenSimStatePacket) -> None:
        coordinate_set = self.model.getCoordinateSet()
        self.state.setTime(float(state_packet.timestamp))
        for coord_index in range(coordinate_set.getSize()):
            coordinate = coordinate_set.get(coord_index)
            if coordinate.getName() in state_packet.coordinate_values:
                coordinate.setValue(self.state, float(state_packet.coordinate_values[coordinate.getName()]))
        self.model.realizePosition(self.state)
        self.visualizer.show(self.state)

    def close(self) -> None:
        """
        The Simbody visualizer is managed by OpenSim; no explicit teardown is required.
        """
