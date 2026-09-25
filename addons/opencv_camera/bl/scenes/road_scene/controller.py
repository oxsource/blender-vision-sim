"""Road Scene controller: the debounced rebuild."""

from __future__ import annotations

from typing import Optional

import bpy

from .. import debounce
from . import builder

#: debounce key (one pending rebuild per scene)
KEY = "road_scene"


def settings_of(context) -> Optional[object]:
    scene = getattr(context, "scene", None)
    return getattr(scene, "road_scene", None) if scene is not None else None


def schedule_rebuild(context=None) -> None:
    """Ask for a rebuild once the user stops editing."""
    context = context or bpy.context
    debounce.schedule(KEY, lambda: rebuild_now(context))


def rebuild_now(context=None):
    """Rebuild immediately (also used by the ``road_rebuild`` operator)."""
    context = context or bpy.context
    scene = getattr(context, "scene", None)
    settings = settings_of(context)
    if scene is None or settings is None or settings.root is None:
        return None
    return builder.rebuild(scene, settings)
