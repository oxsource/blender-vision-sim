"""Operators: the only place where user-visible actions are defined.

Operators stay thin: they resolve the context, call the ``bl`` helpers and turn
their return values into reports.
"""

from __future__ import annotations

import os

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper, ImportHelper

from ..core import calibration_io, presets
from . import apply as apply_mod
from . import camera_factory, preview, selftest, shader


def _camera_object(context):
    """Resolve the camera to work on (see :func:`camera_factory.resolve_camera`)."""
    return camera_factory.resolve_camera(context)


class _CameraOperator:
    """Mixin providing shared context handling and reporting."""

    @classmethod
    def poll(cls, context):
        return _camera_object(context) is not None

    def camera(self, context):
        obj = _camera_object(context)
        if obj is None:
            raise RuntimeError("no camera in context")
        return obj, obj.data


#: enum items for the Add Camera operator.  A *list* (not a callable) so the
#: default can be a string identifier; with callable items bpy requires an index.
MODEL_ITEMS = [(key, value[0], "") for key, value in camera_factory.MODELS.items()]


def _preset_items(self, context):
    """Enum items: the add-on defaults, the active camera, then bundled presets.

    The first item is what the Add menu uses (no dialog is shown), so it must be
    the predictable "Add-on Defaults" rather than silently copying whatever camera
    happens to be active.
    """
    items = [
        (presets.DEFAULTS, "Add-on Defaults", "Use the add-on's default camera values"),
        (presets.CURRENT, "Current Camera Settings",
         "Copy the intrinsics/distortion of the active camera"),
    ]
    for identifier, label in presets.list_presets():
        items.append((identifier, label, f"presets/{identifier}"))
    return items


def _apply_defaults(settings) -> None:
    """Restore the bundled default camera values on ``settings``."""
    from . import properties as props
    intrinsics = props.DEFAULT_INTRINSICS
    coefficients = props.DEFAULT_DISTORTION
    settings.intrinsics.auto_center = False
    settings.intrinsics.fx = intrinsics["fx"]
    settings.intrinsics.fy = intrinsics["fy"]
    settings.intrinsics.cx = intrinsics["cx"]
    settings.intrinsics.cy = intrinsics["cy"]
    settings.intrinsics.image_width = intrinsics["image_width"]
    settings.intrinsics.image_height = intrinsics["image_height"]
    settings.intrinsics.scale_to_render = True
    settings.distortion.model = "fisheye"
    settings.distortion.enabled = True
    (settings.distortion.k1, settings.distortion.k2,
     settings.distortion.k3, settings.distortion.k4) = coefficients
    settings.distortion.k5 = settings.distortion.k6 = 0.0
    settings.distortion.p1 = settings.distortion.p2 = 0.0


def _report_messages(operator, messages, level="INFO"):
    for message in messages:
        operator.report({level}, message)


# ---------------------------------------------------------------------------
class OPENCV_CAM_OT_apply(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.apply_settings"
    bl_label = "Apply"
    bl_description = (
        "Install/compile the OSL shader and copy the intrinsics and distortion "
        "into the camera's Cycles custom parameters"
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        settings.status = "ok" if ok else "error"
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"OpenCV camera applied (fx={settings.intrinsics.fx:.2f}, "
            f"distortion {'on' if settings.distortion.enabled else 'off'})",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_install_shader(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.install_shader"
    bl_label = "Install / Refresh Shader"
    bl_description = "Write the bundled OSL shader into a Text data-block and compile it"
    bl_options = {"REGISTER", "UNDO"}

    force: BoolProperty(name="Force", default=False, description="Overwrite local edits")

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        text, changed = shader.install_shader(settings, force=self.force)
        shader.attach(cam_data, settings)
        ok, messages = shader.ensure_compiled(cam_data)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, f"shader '{text.name}' {'updated' if changed else 'unchanged'} and compiled")
        return {"FINISHED"}


class OPENCV_CAM_OT_import_calibration(_CameraOperator, bpy.types.Operator, ImportHelper):
    bl_idname = "opencv_cam.import_calibration"
    bl_label = "Import"
    bl_description = "Load intrinsics and distortion from an OpenCV/ROS/Kalibr calibration file"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ""
    filter_glob: StringProperty(default="*.yaml;*.yml;*.json;*.xml", options={"HIDDEN"})
    camera_name: StringProperty(
        name="Camera",
        description="Camera to pick from a multi-camera config (empty = first enabled)",
        default="",
    )

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        try:
            calibration = calibration_io.load_calibration(
                self.filepath, camera_name=self.camera_name or None
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.calibration.filepath = self.filepath
        settings.calibration.last_import = os.path.basename(self.filepath)
        note = ""
        if not calibration.model_name:
            note = (" (no distortion_model in the file: coefficients were read as "
                    f"{calibration.distortion.model} - pick Fisheye in the panel if this "
                    "is a wide angle / AVM lens)")
        self.report(
            {"INFO"},
            f"{os.path.basename(self.filepath)}: {calibration.width}x{calibration.height}, "
            f"fx={calibration.intrinsics.fx:.2f}, model={calibration.distortion.model}{note}",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_export_calibration(_CameraOperator, bpy.types.Operator, ExportHelper):
    bl_idname = "opencv_cam.export_calibration"
    bl_label = "Export"
    bl_description = "Write the current intrinsics and distortion to a calibration file"
    bl_options = {"REGISTER"}

    filename_ext = ".yaml"
    filter_glob: StringProperty(default="*.yaml;*.yml;*.json", options={"HIDDEN"})

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        calibration = calibration_io.Calibration(
            intrinsics=apply_mod.effective_intrinsics(
                settings, settings.intrinsics.image_width, settings.intrinsics.image_height
            ),
            distortion=settings.core_distortion(),
        )
        try:
            path = calibration_io.save_calibration(self.filepath, calibration)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"written {path}")
        return {"FINISHED"}


class OPENCV_CAM_OT_sync_from_lens(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.sync_from_lens"
    bl_label = "From Blender Lens"
    bl_description = "Derive fx, fy and the principal point from lens/sensor/shift"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        messages = apply_mod.sync_intrinsics_from_lens(cam_data, settings, context.scene)
        _report_messages(self, messages)
        return {"FINISHED"}


class OPENCV_CAM_OT_add_camera(bpy.types.Operator):
    """Create a camera that already uses the OpenCV lens model"""

    bl_idname = "opencv_cam.add_camera"
    bl_label = "OpenCV Camera"
    bl_description = (
        "Add a camera configured as Cycles Custom with an OpenCV lens model "
        "(fisheye / Brown-Conrady / rational / pinhole)"
    )
    bl_options = {"REGISTER", "UNDO"}

    model: EnumProperty(name="Model", items=MODEL_ITEMS, default="fisheye")
    preset: EnumProperty(
        name="Preset",
        description="Start from the add-on defaults, a bundled calibration, or copy "
        "the active camera",
        items=_preset_items,
    )
    use_rig: BoolProperty(
        name="Add Rig Empty",
        description="Parent the camera to an empty, handy for extrinsics/multi-camera setups",
        default=False,
    )
    at_cursor: BoolProperty(
        name="At 3D Cursor",
        description="Place the camera at the 3D cursor instead of the world origin",
        default=True,
    )

    def execute(self, context):
        scene = context.scene
        location = tuple(scene.cursor.location) if self.at_cursor else (0.0, 0.0, 0.0)
        try:
            # pass the identifier straight through: add_camera knows DEFAULTS,
            # CURRENT and the bundled preset files
            camera, messages = camera_factory.add_camera(
                scene, model=self.model, preset=self.preset,
                use_rig=self.use_rig, location=location,
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        _report_messages(self, messages)
        self.report({"INFO"}, f"added {camera.name} ({camera_factory.MODELS[self.model][0]})")
        return {"FINISHED"}


class OPENCV_CAM_OT_load_preset(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.load_preset"
    bl_label = "Load Preset"
    bl_description = "Load a bundled preset calibration into this camera"
    bl_options = {"REGISTER", "UNDO"}

    preset: EnumProperty(name="Preset", items=_preset_items)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        if self.preset == presets.CURRENT:
            self.report({"WARNING"}, "pick a preset, or use Reset Defaults")
            return {"CANCELLED"}
        if self.preset == presets.DEFAULTS:
            _apply_defaults(settings)
            ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
            _report_messages(self, messages, "INFO" if ok else "ERROR")
            if not ok:
                return {"CANCELLED"}
            self.report({"INFO"}, "add-on defaults restored")
            return {"FINISHED"}
        try:
            calibration = presets.load_preset(self.preset)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.calibration.last_import = self.preset
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, f"preset {self.preset} loaded ({calibration.intrinsics.fx:.2f} px)")
        return {"FINISHED"}


class OPENCV_CAM_OT_reset_defaults(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.reset_defaults"
    bl_label = "Reset Defaults"
    bl_description = "Restore the add-on default intrinsics and distortion coefficients"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        _apply_defaults(settings)
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, "defaults restored (bundled reference camera)")
        return {"FINISHED"}


class OPENCV_CAM_OT_recompile(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.recompile"
    bl_label = "Recompile Shader"
    bl_description = "Refresh the bundled OSL shader text and recompile it"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        _, changed = shader.install_shader(settings, force=False)
        cam_data.custom_bytecode = ""
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"{shader.shader_filename(settings.distortion.model)} recompiled "
            f"({'text updated' if changed else 'text unchanged'}, "
            f"{len(cam_data.custom_bytecode)} chars)",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_preview(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.preview"
    bl_label = "Preview"
    bl_description = (
        "Render a quick preview with the current camera and show it in the Image Editor "
        "(render settings are restored afterwards)"
    )
    bl_options = {"REGISTER"}

    size: EnumProperty(
        name="Size",
        items=[("256", "256 px", ""), ("384", "384 px", ""), ("512", "512 px", ""), ("720", "720 px", "")],
        default="384",
    )
    samples: IntProperty(name="Samples", default=16, min=1, max=512)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        settings.preview.size = self.size
        settings.preview.samples = self.samples
        result = preview.render_preview(
            cam_data, settings, context.scene,
            size_key=self.size, samples=self.samples, denoise=settings.preview.denoise,
        )
        _report_messages(self, result["messages"], "INFO" if result["ok"] else "ERROR")
        if not result["ok"]:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"preview rendered at {result['resolution'][0]}x{result['resolution'][1]} "
            f"({result['samples']} samples)",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_save_preview(_CameraOperator, bpy.types.Operator, ExportHelper):
    bl_idname = "opencv_cam.save_preview"
    bl_label = "Save Preview Image"
    bl_description = "Render a preview and write it to a PNG file"
    bl_options = {"REGISTER"}

    filename_ext = ".png"
    filter_glob: StringProperty(default="*.png", options={"HIDDEN"})

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        try:
            path = preview.save_preview_image(
                cam_data, settings, context.scene,
                size_key=settings.preview.size, samples=max(settings.preview.samples, 16),
                filepath=self.filepath,
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"written {path}")
        return {"FINISHED"}


class OPENCV_CAM_OT_selftest(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.self_test"
    bl_label = "Run Self Test"
    bl_description = (
        "Render a test target and verify the image position against the OpenCV "
        "model (catches stale or broken shaders)"
    )
    bl_options = {"REGISTER"}

    resolution: IntProperty(name="Resolution", default=256, min=32, max=2048)
    samples: IntProperty(name="Samples", default=4, min=1, max=1024)
    tolerance_px: FloatProperty(name="Tolerance (px)", default=1.0, min=0.01, max=50.0)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        try:
            result = selftest.run(
                cam_data,
                settings,
                context.scene,
                resolution=self.resolution,
                samples=self.samples,
                tolerance_px=self.tolerance_px,
            )
        except (selftest.SelfTestError, RuntimeError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        status = "PASSED" if result["passed"] else "FAILED"
        settings.status = f"selftest {status.lower()} ({result['error_px']:.3f} px)"
        self.report(
            {"INFO"} if result["passed"] else {"WARNING"},
            f"self test {status}: predicted {result['predicted'][0]:.2f},{result['predicted'][1]:.2f} "
            f"measured {result['measured'][0]:.3f},{result['measured'][1]:.3f} "
            f"error {result['error_px']:.3f} px (tolerance {result['tolerance_px']:.2f})",
        )
        return {"FINISHED"} if result["passed"] else {"CANCELLED"}


_CLASSES = (
    OPENCV_CAM_OT_apply,
    OPENCV_CAM_OT_add_camera,
    OPENCV_CAM_OT_load_preset,
    OPENCV_CAM_OT_reset_defaults,
    OPENCV_CAM_OT_recompile,
    OPENCV_CAM_OT_preview,
    OPENCV_CAM_OT_save_preview,
    OPENCV_CAM_OT_install_shader,
    OPENCV_CAM_OT_import_calibration,
    OPENCV_CAM_OT_export_calibration,
    OPENCV_CAM_OT_sync_from_lens,
    OPENCV_CAM_OT_selftest,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)