"""Filesystem locations inside the add-on package.

Never build paths from ``__package__`` (an add-on is imported as
``bl_ext.<repo>.<id>`` when installed as an extension and as the plain package
name when installed as a legacy add-on); ``__file__`` is stable in both cases.
"""

from __future__ import annotations

import os
from typing import List

ADDON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHADERS_DIR = os.path.join(ADDON_ROOT, "shaders")
#: bundled camera calibration presets (scanned by :mod:`core.presets`)
PRESETS_DIR = os.path.join(ADDON_ROOT, "presets")


def scene_preset_file(scene_id: str, name: str = "default") -> str:
    """Absolute path of a scene default preset.

    Scene presets live in a *sub-folder* of ``presets/`` so that
    :func:`core.presets.list_preset_paths` (which only lists the top level)
    never offers them as camera calibrations.
    """
    return os.path.join(PRESETS_DIR, scene_id, f"{name}.json")


def shader_file(name: str) -> str:
    """Absolute path of a bundled ``.osl`` file."""
    return os.path.join(SHADERS_DIR, name)


def bundled_shaders() -> List[str]:
    """Names of the bundled ``.osl`` shaders (sorted)."""
    try:
        return sorted(n for n in os.listdir(SHADERS_DIR) if n.endswith(".osl"))
    except OSError:
        return []


def read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()