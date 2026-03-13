#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Version helper that keeps source-tree imports working even when the package has
not been installed into the current Python environment.
"""

from importlib.metadata import PackageNotFoundError, version


def get_pose2sim_version() -> str:
    try:
        return version("pose2sim")
    except PackageNotFoundError:
        return "0+local"
