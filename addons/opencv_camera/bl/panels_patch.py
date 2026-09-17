"""Hide the raw parameter list that Cycles draws for custom cameras.

Cycles draws one row per OSL shader parameter
(``CYCLES_CAMERA_PT_lens_custom_parameters``) as plain numbers, with ``0``/``1``
for the boolean ones.  The add-on has curated panels for the same values, so the
raw list is hidden unless the user turns on *Show Cycles Raw Parameters*.

Implementation note: this replaces the poll of another add-on's panel at runtime
(Blender looks the Python ``poll`` up on every draw, so the replacement takes
effect) and restores the original on unregister.  If Cycles re-registers its
panels afterwards (engine switch, re-enabling Cycles) the patch is gone until the
add-on is reloaded; hiding the panel is cosmetic, so that is acceptable.
"""

from __future__ import annotations

from typing import Optional

import bpy

PANEL_NAME = "CYCLES_CAMERA_PT_lens_custom_parameters"

_original_poll: Optional[object] = None


def _hide_for(context) -> bool:
    """True when the raw list should not be drawn for the current camera."""
    obj = getattr(context, "camera", None)
    if obj is None:
        obj = getattr(getattr(context, "scene", None), "camera", None)
    if obj is None:
        return False
    data = getattr(obj, "data", obj)
    settings = getattr(data, "opencv_cam", None)
    if settings is None:
        return False
    return not settings.show_raw_params


def _poll(cls, context):
    if _hide_for(context):
        return False
    return bool(_original_poll and _original_poll(cls, context))


def is_patched() -> bool:
    return _original_poll is not None


def register() -> None:
    global _original_poll
    panel = getattr(bpy.types, PANEL_NAME, None)
    if panel is None or _original_poll is not None:
        return  # Cycles disabled, or already patched
    poll = panel.poll
    _original_poll = poll.__func__ if hasattr(poll, "__func__") else poll
    panel.poll = classmethod(_poll)


def unregister() -> None:
    global _original_poll
    panel = getattr(bpy.types, PANEL_NAME, None)
    if panel is not None and _original_poll is not None:
        panel.poll = classmethod(_original_poll)
    _original_poll = None