"""The scene registry.

Every algorithm scene is one module here, exposing a
:class:`~.base.SceneDefinition` plus ``register()`` / ``unregister()``.  Adding a
scene means adding a module and one entry to :data:`_MODULES` - nothing else in
the add-on changes (the Add menu and the panels iterate this registry).

See ``docs/avm-scene.md`` §17 for the full design.
"""

from __future__ import annotations

from typing import List, Optional

from . import camera_scene, debounce
from .base import (
    SceneDefinition,
    ScenePanel,
    collection,
    has_scene,
    link_to_collection,
    remove_collection_objects,
    root_pointer,
    scene_settings,
)

__all__ = [
    "SceneDefinition", "ScenePanel", "debounce",
    "collection", "has_scene", "link_to_collection", "remove_collection_objects",
    "root_pointer", "scene_settings",
    "definitions", "definition", "register", "unregister",
]

#: scene modules, in registration order (a scene added later goes last)
_MODULES = (camera_scene,)


def definitions() -> List[SceneDefinition]:
    """All registered scenes, sorted for the menu."""
    return sorted((module.DEFINITION for module in _MODULES), key=lambda d: d.order)


def definition(scene_id: str) -> Optional[SceneDefinition]:
    """The definition with this id, or ``None``."""
    for candidate in definitions():
        if candidate.id == scene_id:
            return candidate
    return None


def register() -> None:
    for module in _MODULES:
        module.register()


def unregister() -> None:
    for module in reversed(_MODULES):
        module.unregister()
