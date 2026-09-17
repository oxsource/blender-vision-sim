"""Shared scene framework.

An algorithm scene (Camera Scene, AVM Scene, later DMS, ...) is *one
self-contained module + one registry entry*.  This module holds the pieces every
scene shares:

* :class:`SceneDefinition` - the metadata the menu and the panels are built from;
* :func:`has_scene` - the "is it built?" test (a root empty pointer that Blender
  clears when the object is deleted, so panels cannot go stale);
* :func:`collection` / :func:`link_to_collection` - the object grouping helpers;
* :class:`ScenePanel` - a panel mixin that only shows while its scene exists.

The default 3/4 orbit a freshly built scene is framed from lives in
:mod:`.view` and is carried on the definition (:attr:`SceneDefinition.view`).

The naming convention (see ``docs/avm-scene.md`` §17.6): scene id
``<name>_scene``, collection ``<Name> Scene``, root empty ``<NAME>_Root``,
properties ``scene.<id>``, panel ``OPENCV_CAM_PT_<id>`` and operators
``opencv_cam.<id>_<action>``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import bpy

from .view import ViewSpec

#: a scene's "what should the default view frame?" hook: it gets the scene and
#: returns the *subject* objects (see :func:`bl.scenes.view.targets`)
ViewTargets = Callable[[bpy.types.Scene], List[bpy.types.Object]]


@dataclass(frozen=True)
class SceneDefinition:
    """Everything the menu and the panel scaffolding need to know."""

    id: str                 #: "avm_scene" (also the Scene property name)
    label: str              #: "AVM Scene" (menu / panel label)
    icon: str               #: icons/<icon>.png (with a built-in fallback)
    order: int              #: menu order (lower first)
    add_operator: str       #: "opencv_cam.avm_add_scene"
    root_name: str = ""     #: "AVM_Root"; empty = no root / no panel
    collection_name: str = ""  #: "AVM Scene"
    view: ViewSpec = field(default_factory=ViewSpec)  #: orbit of a fresh scene
    view_targets: Optional[ViewTargets] = None  #: default: the whole collection

    @property
    def has_panel(self) -> bool:
        return bool(self.root_name)


def scene_settings(context, scene_id: str):
    """The scene's settings property group (``scene.<scene_id>``), or ``None``."""
    scene = getattr(context, "scene", None)
    if scene is None:
        return None
    return getattr(scene, scene_id, None)


def root_pointer(context, scene_id: str) -> Optional[bpy.types.Object]:
    """The scene's root empty, if the property group exposes one."""
    settings = scene_settings(context, scene_id)
    return getattr(settings, "root", None) if settings is not None else None


def has_scene(context, definition: SceneDefinition) -> bool:
    """True while the scene is built.

    ``root`` is a ``PointerProperty(type=Object)``: Blender clears it when the
    empty is deleted, so this cannot report a stale scene.  The name check is a
    belt-and-braces guard for pointer edge cases.
    """
    root = root_pointer(context, definition.id)
    return root is not None and root.name in bpy.data.objects


def collection(name: str, scene: Optional[bpy.types.Scene] = None,
               create: bool = False) -> Optional[bpy.types.Collection]:
    """Find (and optionally create) a named collection in ``scene``."""
    found = bpy.data.collections.get(name)
    if found is None and create and scene is not None:
        found = bpy.data.collections.new(name)
        scene.collection.children.link(found)
    return found


def link_to_collection(obj: bpy.types.Object, target: bpy.types.Collection) -> None:
    """Move ``obj`` into ``target`` (unlink it from every other collection)."""
    target.objects.link(obj)
    for other in list(obj.users_collection):
        if other is not target:
            other.objects.unlink(obj)


def remove_collection_objects(name: str) -> int:
    """Delete every object of a collection; returns how many were removed."""
    found = bpy.data.collections.get(name)
    if found is None:
        return 0
    objects = list(found.objects)
    for obj in objects:
        bpy.data.objects.remove(obj, do_unlink=True)
    return len(objects)


class ScenePanel:
    """Panel mixin: the panel appears only while its scene is built.

    Subclasses set :attr:`definition` plus the usual ``bl_idname`` / ``bl_label``
    and ``draw``; ``bl_space_type`` / ``bl_region_type`` / ``bl_context`` default
    to the Scene Properties tab (the 3D viewport N panel overrides them).
    """

    definition: Optional[SceneDefinition] = None
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

    @classmethod
    def poll(cls, context):
        return cls.definition is not None and has_scene(context, cls.definition)
