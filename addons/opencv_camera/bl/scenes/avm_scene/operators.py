"""AVM Scene operators: add / rebuild / reset / remove.

Import/export, the four-camera render and the coverage tools are added by their
own modules (``io.py`` / ``coverage.py``) so this file stays a thin action layer.
"""

from __future__ import annotations

import os

import bpy
from bpy.props import BoolProperty, EnumProperty, StringProperty
from bpy_extras.io_utils import ExportHelper, ImportHelper

from ....core.scenes import avm_layout
from ..base import has_scene
from .. import view as view_mod
from . import DEFINITION, builder, controller, io as io_mod, properties


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
        view_mod.frame(view_mod.scene_objects(DEFINITION), DEFINITION.view)
        messages = created.get("messages") or []
        for message in messages:
            self.report({"WARNING"}, message)
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
        for message in created.get("messages") or []:
            self.report({"WARNING"}, message)
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


class OPENCV_CAM_OT_avm_select_camera(_AVMSceneOperator, bpy.types.Operator):
    """Select a camera, make it the render camera (F12) and focus its intrinsics"""

    bl_idname = "opencv_cam.avm_select_camera"
    bl_label = "Select Camera"
    bl_options = {"REGISTER", "UNDO"}

    name: EnumProperty(name="Camera", items=properties.CAMERA_ITEMS)

    def execute(self, context):
        camera = bpy.data.objects.get(f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX[self.name]}")
        if camera is None:
            self.report({"WARNING"}, f"camera {self.name} is missing")
            return {"CANCELLED"}
        view_layer = context.view_layer
        for obj in view_layer.objects:
            obj.select_set(False)
        camera.select_set(True)
        view_layer.objects.active = camera
        # F12 renders scene.camera, not the selection: make this camera active so
        # the render and the resolution follow the button
        settings = _settings(context)
        if settings is not None:
            settings.active_camera = self.name
            builder._apply_active_resolution(context.scene, settings)
        self.report(
            {"INFO"},
            f"{camera.name} is the render camera now - edit its intrinsics in CV Intrinsics",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_apply_preset(_AVMSceneOperator, bpy.types.Operator):
    """Apply one of the quick field presets (sizes only, like the HTML tool)"""

    bl_idname = "opencv_cam.avm_apply_preset"
    bl_label = "Quick Preset"
    bl_options = {"REGISTER", "UNDO"}

    preset: EnumProperty(
        name="Preset",
        items=[(key, value[0], "") for key, value in avm_layout.PRESETS.items()],
        default="default",
    )

    def execute(self, context):
        settings = _settings(context)
        _, field = avm_layout.PRESETS[self.preset]
        controller.set_field(settings, field)
        controller.rebuild_now(context)
        self.report({"INFO"}, f"preset applied: {avm_layout.PRESETS[self.preset][0]}")
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_export_json(_AVMSceneOperator, bpy.types.Operator):
    """Write the current parameters into the panel's text box (full format)"""

    bl_idname = "opencv_cam.avm_export_json"
    bl_label = "Generate JSON"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        settings.io_text = io_mod.dumps(io_mod.to_full(settings))
        settings.io_status = "generated the full avm_scene JSON"
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_apply_json(_AVMSceneOperator, bpy.types.Operator):
    """Apply the parameters from the panel's text box"""

    bl_idname = "opencv_cam.avm_apply_json"
    bl_label = "Apply JSON"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        try:
            data = io_mod.loads(settings.io_text)
            fmt = io_mod.apply(settings, data)
        except Exception as exc:
            settings.io_status = f"error: {exc}"
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        controller.rebuild_now(context)
        settings.io_status = f"applied the {fmt} parameters"
        self.report({"INFO"}, settings.io_status)
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_export_params(_AVMSceneOperator, bpy.types.Operator, ExportHelper):
    """Export the parameters to a JSON file (full or compact)"""

    bl_idname = "opencv_cam.avm_export_params"
    bl_label = "Export Parameters"
    bl_options = {"REGISTER"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})
    format: EnumProperty(
        name="Format",
        items=[("full", "Full (avm_scene)", "Field, car, ground and the four cameras"),
               ("plane", "Compact (plane_scene)", "The four HTML/App Store keys only")],
        default="full",
    )

    def execute(self, context):
        settings = _settings(context)
        data = (io_mod.to_full(settings) if self.format == "full"
                else io_mod.to_plane(settings))
        try:
            path = io_mod.write(self.filepath, data)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.io_status = f"exported {self.format} parameters"
        self.report({"INFO"}, f"written {path}")
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_import_params(_AVMSceneOperator, bpy.types.Operator, ImportHelper):
    """Import parameters from a JSON/YAML file and rebuild"""

    bl_idname = "opencv_cam.avm_import_params"
    bl_label = "Import Parameters"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json;*.yaml;*.yml", options={"HIDDEN"})

    def execute(self, context):
        settings = _settings(context)
        try:
            data = io_mod.read(self.filepath)
            fmt = io_mod.apply(settings, data)
        except Exception as exc:
            settings.io_status = f"error: {exc}"
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        controller.rebuild_now(context)
        settings.io_status = f"imported {fmt} parameters"
        self.report({"INFO"}, settings.io_status)
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_render_cameras(_AVMSceneOperator, bpy.types.Operator, ExportHelper):
    """Render every enabled camera to a PNG in the chosen folder"""

    bl_idname = "opencv_cam.avm_render_cameras"
    bl_label = "Export 4 Camera Views"
    bl_options = {"REGISTER"}

    filename_ext = ".png"
    filter_glob: StringProperty(default="*.png", options={"HIDDEN"})
    samples: bpy.props.IntProperty(name="Samples", default=64, min=1, max=4096)

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = "avm.png"
        return super().invoke(context, event)

    def execute(self, context):
        settings = _settings(context)
        directory = os.path.dirname(os.path.abspath(self.filepath))
        try:
            written = io_mod.render_cameras(context, settings, directory,
                                            samples=self.samples)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.io_status = f"rendered {len(written)} cameras"
        self.report({"INFO"}, f"{len(written)} images written to {directory}")
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_analyze_coverage(_AVMSceneOperator, bpy.types.Operator):
    """Compute the ground coverage, blind spots and block visibility"""

    bl_idname = "opencv_cam.avm_analyze_coverage"
    bl_label = "Analyze Coverage"
    bl_options = {"REGISTER", "UNDO"}

    step: bpy.props.FloatProperty(
        name="Grid Step", default=0.05, min=0.005, max=1.0, unit="LENGTH",
        description="Grid resolution of the area statistics")

    def execute(self, context):
        from . import coverage
        settings = _settings(context)
        report = coverage.analyze(settings, step=self.step)
        coverage.ensure_curves(context.scene, settings, report)
        settings.show_coverage = True
        self.report({"INFO"}, settings.coverage_status)
        return {"FINISHED"}


class OPENCV_CAM_OT_avm_export_materials(_AVMSceneOperator, bpy.types.Operator, ExportHelper):
    """Render the four cameras and write the parameters + coverage report"""

    bl_idname = "opencv_cam.avm_export_materials"
    bl_label = "Export Materials"
    bl_options = {"REGISTER"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})
    samples: bpy.props.IntProperty(name="Samples", default=64, min=1, max=4096)

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = "avm_scene.json"
        return super().invoke(context, event)

    def execute(self, context):
        from . import coverage
        settings = _settings(context)
        directory = os.path.dirname(os.path.abspath(self.filepath))
        try:
            written = coverage.export_materials(context, settings, directory,
                                                samples=self.samples)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"{len(written)} files written to {directory}")
        return {"FINISHED"}


_CLASSES = (
    OPENCV_CAM_OT_avm_add_scene,
    OPENCV_CAM_OT_avm_rebuild,
    OPENCV_CAM_OT_avm_reset_defaults,
    OPENCV_CAM_OT_avm_remove_scene,
    OPENCV_CAM_OT_avm_select_camera,
    OPENCV_CAM_OT_avm_apply_preset,
    OPENCV_CAM_OT_avm_export_json,
    OPENCV_CAM_OT_avm_apply_json,
    OPENCV_CAM_OT_avm_export_params,
    OPENCV_CAM_OT_avm_import_params,
    OPENCV_CAM_OT_avm_render_cameras,
    OPENCV_CAM_OT_avm_analyze_coverage,
    OPENCV_CAM_OT_avm_export_materials,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
