#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Optional realtime recorder utilities.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np

from Pose2Sim.realtime.packets import OpenSimStatePacket, Pose3DPacket


class RealtimeRecorder:
    """
    Collect realtime outputs and optionally flush them to disk on close.
    """

    def __init__(
        self,
        output_dir: str,
        record_markers: bool = False,
        record_mot: bool = False,
    ):
        self.output_dir = Path(output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.record_markers = bool(record_markers)
        self.record_mot = bool(record_mot)

        self._pose3d_packets: List[Pose3DPacket] = []
        self._state_packets: List[OpenSimStatePacket] = []
        self._flushed = False

    def start(self) -> None:
        """No-op, kept for interface consistency with other pipeline components."""

    def record_pose3d(self, pose3d_packet: Pose3DPacket) -> None:
        if self.record_markers:
            self._pose3d_packets.append(pose3d_packet)

    def record_state(self, state_packet: OpenSimStatePacket) -> None:
        self._state_packets.append(state_packet)

    def close(self) -> None:
        if self._flushed:
            return
        self._flushed = True

        if self._state_packets:
            self._write_coordinate_log()
            if self.record_mot:
                self._write_mot()
        if self.record_markers and self._pose3d_packets:
            self._write_marker_npz()

    def _write_coordinate_log(self) -> None:
        coordinate_names = list(self._state_packets[0].coordinate_values.keys())
        output_path = self.output_dir / "realtime_coordinates.tsv"
        with output_path.open("w", encoding="utf-8") as output_file:
            output_file.write("\t".join(["time", *coordinate_names]) + "\n")
            for packet in self._state_packets:
                row = [f"{packet.timestamp:.8f}"]
                row.extend(f"{packet.coordinate_values.get(name, np.nan):.8f}" for name in coordinate_names)
                output_file.write("\t".join(row) + "\n")

    def _write_mot(self) -> None:
        coordinate_names = list(self._state_packets[0].coordinate_values.keys())
        output_path = self.output_dir / "realtime.mot"
        with output_path.open("w", encoding="utf-8") as output_file:
            output_file.write("Coordinates\n")
            output_file.write("version=1\n")
            output_file.write(f"nRows={len(self._state_packets)}\n")
            output_file.write(f"nColumns={len(coordinate_names) + 1}\n")
            output_file.write("inDegrees=no\n\n")
            output_file.write("Units are S.I. units (second, meters, radians, ...)\n")
            output_file.write("endheader\n")
            output_file.write("\t".join(["time", *coordinate_names]) + "\n")
            for packet in self._state_packets:
                row = [f"{packet.timestamp:.8f}"]
                row.extend(f"{packet.coordinate_values.get(name, np.nan):.8f}" for name in coordinate_names)
                output_file.write("\t".join(row) + "\n")

    def _write_marker_npz(self) -> None:
        marker_names = tuple(self._pose3d_packets[0].marker_names)
        timestamps = np.asarray([packet.timestamp for packet in self._pose3d_packets], dtype=float)
        markers_3d = np.stack([packet.markers_3d for packet in self._pose3d_packets], axis=0)
        output_path = self.output_dir / "realtime_markers.npz"
        np.savez(
            output_path,
            marker_names=np.asarray(marker_names, dtype=object),
            timestamps=timestamps,
            markers_3d=markers_3d,
        )
