"""AVM Scene operators: add / rebuild / reset / remove.

Import/export, the four-camera render and the coverage tools are added by their
own modules (``io.py`` / ``coverage.py``) so this file stays a thin action layer.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty

from ....core.scenes import avm_layout
from ..base import has_scene
from . import DEFINITION, builder, controller


def _scene(context):
    return getattr(context, "scene", None)


def _settings(context):
    return controller.settings_of(context)


class _AVMOperator:
    @classmethod
    def poll(cls, context):
        return _scene(context) is not None


class _AVMSceneOperator(_AVMOperator):
    """Base for operators that need an already-built AVM Scene."""

    @classmethod
    def poll(cls, context):
        return _scene(context) is not None and has_scene(context, DEFINITION)


class OPENCV_CAM_OT_avm_add_scene(_AVMOperator, bpy.types.Operator):
    """Create the AVM plane scene (ground, car, four blocks, four fisheye cameras)"""

    bl_idname = "opencv_cam.avm_add_scene"
    bl_label = "AVM Scene"
    bl_description = (
        "Create the AVM scene: a ground plane, a cube car, four solid black "
        "calibration blocks and four OpenCV fisheye cameras, using the bundled "
        "default parameters"
    )
    bl_options = {"REGISTER", "UNDO"}

    reset: BoolProperty(
        name="Reset", default=False,
        description="Reload the bundled defaults even if the scene already exists")

    def execute(self, context):
        scene = _scene(context)
        settings = _settings(context)
        if settings is None:
            self.report({"ERROR"}, "scene.avm_scene is not registered")
            return {"CANCELLED"}
        if settings.root is not None and not self.reset:
            self.report({"WARNING"}, "an AVM Scene already exists (use Rebuild or Reset)")
            return {"CANCELLED"}
        if settings.root is not None and self.reset:
            builder.remove(scene, settings)
        try:
            preset = avm_layout.load_preset()
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        created = builder.build(scene, settings, preset=preset)
        self.report(
            {"INFO"},
            f"AVM Scene ready: 1 ground, 1 car, {len(created['blocks'])} blocks, "
            f"{len(created['cameras'])} cameras",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_rebuild(_AVMSceneOperator, bpy.types.Operator):
    """Rebuild the AVM scene geometry from the current parameters"""

    bl_idname = "opencv_cam.avm_rebuild"
    bl_label = "Rebuild"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        created = controller.rebuild_now(context)
        if created is None:
            self.report({"WARNING"}, "no AVM Scene to rebuild")
            return {"CANCELLED"}
        self.report({"INFO"}, "AVM Scene rebuilt")
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_reset_defaults(_AVMSceneOperator, bpy.types.Operator):
    """Reload the bundled default parameters (field, car and camera poses)"""

    bl_idname = "opencv_cam.avm_reset_defaults"
    bl_label = "Reset Defaults"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        try:
            preset = avm_layout.load_preset()
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        controller.load_preset_into(settings, preset)
        controller.rebuild_now(context)
        self.report({"INFO"}, "AVM Scene defaults restored")
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_remove_scene(_AVMSceneOperator, bpy.types.Operator):
    """Remove the AVM Scene (deletes its objects; the panels disappear)"""

    bl_idname = "opencv_cam.avm_remove_scene"
    bl_label = "Remove AVM Scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = _scene(context)
        settings = _settings(context)
        removed = builder.remove(scene, settings)
        self.report({"INFO"}, f"AVM Scene removed ({removed} objects)")
        return {"FINISHED"}


_CLASSES = (
    OPENCV_CAM_OT_avm_add_scene,
    OPENCV_CAM_OT_avm_rebuild,
    OPENCV_CAM_OT_avm_reset_defaults,
    OPENCV_CAM_OT_avm_remove_scene,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
