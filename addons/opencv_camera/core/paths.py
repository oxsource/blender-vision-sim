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
LOGOS_DIR = os.path.join(ADDON_ROOT, "logos")
MODELS_DIR = os.path.join(ADDON_ROOT, "models")
#: bundled camera calibration presets (scanned by :mod:`core.presets`)
PRESETS_DIR = os.path.join(ADDON_ROOT, "presets")

#: logo shipped with the add-on, used by the AVM Scene ground decal when no
#: custom image is chosen (so it survives a Blender restart and ships in the zip)
DEFAULT_LOGO = "avm_logo"

#: the real AVM bowl ground shipped with the add-on, used by the AVM Scene when
#: no custom ground model is chosen (it is the mesh the Falcon app projects the
#: four camera images onto, so the simulated cameras see the same environment)
DEFAULT_GROUND_MODEL = "unlit_round_bowls"


def logo_file(name: str = DEFAULT_LOGO) -> str:
    """Absolute path of a bundled logo image."""
    return os.path.join(LOGOS_DIR, f"{name}.png")


def bundled_logos() -> List[str]:
    """Names of the bundled logos (sorted, without extension)."""
    try:
        return sorted(n[:-4] for n in os.listdir(LOGOS_DIR) if n.endswith(".png"))
    except OSError:
        return []


def ground_model_file(name: str = DEFAULT_GROUND_MODEL) -> str:
    """Absolute path of a bundled ground model (``.glb``)."""
    return os.path.join(MODELS_DIR, f"{name}.glb")


def bundled_ground_models() -> List[str]:
    """Names of the bundled ground models (sorted, without extension)."""
    try:
        return sorted(n[:-4] for n in os.listdir(MODELS_DIR) if n.endswith(".glb"))
    except OSError:
        return []


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