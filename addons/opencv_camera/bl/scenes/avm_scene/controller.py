"""AVM Scene controller: debounced rebuilds, preset loading, active camera.

The property ``update`` callbacks land here.  Rebuilding straight from an update
callback would run once per mouse move while a slider is dragged, so the work is
debounced through :mod:`bl.scenes.debounce`.
"""

from __future__ import annotations

from typing import Optional

import bpy

from ....core.scenes import avm_layout
from .. import debounce
from . import builder

#: debounce key (one pending rebuild per scene)
KEY = "avm_scene"


def settings_of(context) -> Optional[object]:
    scene = getattr(context, "scene", None)
    return getattr(scene, "avm_scene", None) if scene is not None else None


def schedule_rebuild(context=None) -> None:
    """Ask for a rebuild once the user stops editing."""
    context = context or bpy.context
    debounce.schedule(KEY, lambda: rebuild_now(context))


def rebuild_now(context=None):
    """Rebuild immediately (also used by the ``avm_rebuild`` operator)."""
    context = context or bpy.context
    scene = getattr(context, "scene", None)
    settings = settings_of(context)
    if scene is None or settings is None or settings.root is None:
        return None
    return builder.rebuild(scene, settings)


def apply_active_camera(context=None) -> None:
    """Give the active camera ownership of the render resolution."""
    context = context or bpy.context
    scene = getattr(context, "scene", None)
    settings = settings_of(context)
    if scene is None or settings is None:
        return
    for name in avm_layout.CAMERAS:
        camera = bpy.data.objects.get(f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX[name]}")
        if camera is not None:
            camera.data.opencv_cam.output.lock_scene_resolution = (
                name == settings.active_camera)
    builder._apply_active_resolution(scene, settings)


def load_preset_into(settings, preset) -> None:
    """Copy a preset's field and camera records into the scene settings."""
    field = avm_layout.field_from_preset(preset)
    settings.border_w = field.border_w
    settings.border_h = field.border_h
    settings.corner = field.corner
    settings.inner_w = field.inner_w
    settings.inner_h = field.inner_h
    settings.core_w = field.core_w
    settings.core_h = field.core_h
    settings.car_follow_core = True

    settings.cameras.clear()
    for record in avm_layout.cameras_from_preset(preset):
        entry = settings.cameras.add()
        entry.name = record["name"]
        entry.enable = record["enable"]
        entry.location = record["location"]
        entry.rotation = record["rotation"]
    settings.active_camera = "front"
