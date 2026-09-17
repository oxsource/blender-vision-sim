"""The scene registry.

Every algorithm scene is one module here, exposing a
:class:`~.base.SceneDefinition` plus ``register()`` / ``unregister()``.  Adding a
scene means adding a module and one entry to :data:`_MODULES` - nothing else in
the add-on changes (the Add menu and the panels iterate this registry).

See ``docs/avm-scene.md`` §17 for the full design.
"""

from __future__ import annotations

from typing import List, Optional

from . import avm_scene, camera_scene, debounce, view
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
from .view import ViewSpec, scene_objects

__all__ = [
    "SceneDefinition", "ScenePanel", "ViewSpec", "debounce", "view",
    "collection", "has_scene", "link_to_collection", "remove_collection_objects",
    "root_pointer", "scene_settings", "scene_objects",
    "definitions", "definition", "register", "unregister",
]

#: scene modules, in registration order (a scene added later goes last)
_MODULES = (camera_scene, avm_scene)


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
    view.register()
    for module in _MODULES:
        module.register()


def unregister() -> None:
    for module in reversed(_MODULES):
        module.unregister()
    view.unregister()
