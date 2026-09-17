"""Bundled camera presets.

Every calibration file in ``presets/`` (``*.yaml``, ``*.yml``, ``*.json``) is
listed by name in the *Add Camera* / *Load Preset* operators.  Drop your own
files in that folder (or import a calibration and export it there) to make them
show up - no code changes needed.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

from . import paths
from .calibration_io import Calibration, load_calibration

PRESETS_DIR = os.path.join(paths.ADDON_ROOT, "presets")

#: fake entry used by the UI to mean "use the add-on defaults / current values"
CURRENT = "__current__"


def list_preset_paths() -> List[str]:
    """Absolute paths of the bundled presets, sorted."""
    if not os.path.isdir(PRESETS_DIR):
        return []
    names = [
        name for name in sorted(os.listdir(PRESETS_DIR))
        if name.lower().endswith((".yaml", ".yml", ".json"))
    ]
    return [os.path.join(PRESETS_DIR, name) for name in names]


def list_presets() -> List[Tuple[str, str]]:
    """``(identifier, label)`` pairs for UI enum items."""
    return [(os.path.splitext(os.path.basename(path))[0],
             os.path.splitext(os.path.basename(path))[0]) for path in list_preset_paths()]


def preset_path(identifier: str) -> Optional[str]:
    for path in list_preset_paths():
        if os.path.splitext(os.path.basename(path))[0] == identifier:
            return path
    return None


def load_preset(identifier: str, camera_name: Optional[str] = None) -> Calibration:
    """Load a bundled preset by identifier (file name without extension)."""
    path = preset_path(identifier)
    if path is None:
        raise ValueError(
            f"preset {identifier!r} not found (available: "
            + ", ".join(name for name, _ in list_presets()) + ")"
        )
    return load_calibration(path, camera_name=camera_name)