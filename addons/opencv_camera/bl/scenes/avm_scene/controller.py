"""AVM Scene controller: debounced rebuilds, preset loading, active camera.

The property ``update`` callbacks land here.  Rebuilding straight from an update
callback would run once per mouse move while a slider is dragged, so the work is
debounced through :mod:`bl.scenes.debounce`.
"""

from __future__ import annotations

import math
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
    """Make the active camera the render camera (F12 uses ``scene.camera``)."""
    context = context or bpy.context
    scene = getattr(context, "scene", None)
    settings = settings_of(context)
    if scene is None or settings is None:
        return
    builder._apply_active_camera(scene, settings)


def set_field(settings, field) -> None:
    """Copy a :class:`FieldSpec` into the scene settings."""
    settings.border_w = field.border_w
    settings.border_h = field.border_h
    settings.corner = field.corner
    settings.inner_w = field.inner_w
    settings.inner_h = field.inner_h
    settings.core_w = field.core_w
    settings.core_h = field.core_h


def load_preset_into(settings, preset) -> None:
    """Copy a preset's field and camera records into the scene settings.

    The mount pose itself is written straight onto the camera objects (the pose
    has a single source: the object transform), so the AVM panel and
    ``CV Extrinsics`` can never disagree.
    """
    set_field(settings, avm_layout.field_from_preset(preset))
    settings.car_follow_core = True

    records = avm_layout.cameras_from_preset(preset)
    settings.cameras.clear()
    for record in records:
        entry = settings.cameras.add()
        entry.name = record["name"]
        entry.enable = record["enable"]
    settings.active_camera = "front"

    # the body has to be tall enough for the cameras to sit on it
    mount_heights = [record["location"][2] for record in records]
    if mount_heights:
        settings.car_height = round(max(mount_heights) + 0.05, 2)

    apply_preset_poses(records)


def apply_preset_poses(records) -> None:
    """Write preset mount poses onto the camera objects that already exist."""
    for record in records:
        object_name = (f"{builder.CAMERA_PREFIX}"
                       f"{builder.CAMERA_SUFFIX.get(record['name'], '')}")
        camera = bpy.data.objects.get(object_name)
        if camera is None:
            continue  # not built yet; _ensure_cameras applies it on creation
        builder.apply_camera_pose(
            camera, record["location"],
            [math.radians(value) for value in record["rotation"]])
