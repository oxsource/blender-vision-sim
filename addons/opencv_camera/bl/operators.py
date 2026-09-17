"""Operators: the only place where user-visible actions are defined.

Operators stay thin: they resolve the context, call the ``bl`` helpers and turn
their return values into reports.
"""

from __future__ import annotations

import os

import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper, ImportHelper

from ..core import calibration_io
from . import apply as apply_mod
from . import selftest, shader


def _camera_data(context):
    obj = context.active_object or context.camera
    if obj is None or obj.type != "CAMERA":
        return None
    return obj.data


class _CameraOperator:
    """Mixin providing shared context handling and reporting."""

    @classmethod
    def poll(cls, context):
        return _camera_data(context) is not None

    def camera(self, context):
        obj = context.active_object or context.camera
        return obj, obj.data


def _report_messages(operator, messages, level="INFO"):
    for message in messages:
        operator.report({level}, message)


# ---------------------------------------------------------------------------
class OPENCV_CAM_OT_apply(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.apply_settings"
    bl_label = "Apply to Camera"
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
    bl_label = "Import Calibration"
    bl_description = "Load intrinsics and distortion from an OpenCV/ROS/Kalibr calibration file"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ""
    filter_glob: StringProperty(default="*.yaml;*.yml;*.json;*.xml", options={"HIDDEN"})

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        try:
            calibration = calibration_io.load_calibration(self.filepath)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.calibration.filepath = self.filepath
        settings.calibration.last_import = os.path.basename(self.filepath)
        self.report(
            {"INFO"},
            f"{os.path.basename(self.filepath)}: {calibration.width}x{calibration.height}, "
            f"fx={calibration.intrinsics.fx:.2f}, model={calibration.distortion.model}",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_export_calibration(_CameraOperator, bpy.types.Operator, ExportHelper):
    bl_idname = "opencv_cam.export_calibration"
    bl_label = "Export Calibration"
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


class OPENCV_CAM_OT_apply_pose(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.apply_pose"
    bl_label = "Apply Pose to Object"
    bl_description = "Set the camera object transform from the OpenCV world-to-camera pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, cam_data = self.camera(context)
        apply_mod.apply_opencv_pose(obj, cam_data.opencv_cam)
        self.report({"INFO"}, f"{obj.name}: transform set from OpenCV R/t")
        return {"FINISHED"}


class OPENCV_CAM_OT_read_pose(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.read_pose"
    bl_label = "Read Pose from Object"
    bl_description = "Store the current camera transform as an OpenCV world-to-camera pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        rotation, translation = apply_mod.read_opencv_pose(obj, settings)
        settings.pose.rotation = rotation
        settings.pose.translation = translation
        self.report({"INFO"}, "pose read from the camera object")
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
    OPENCV_CAM_OT_install_shader,
    OPENCV_CAM_OT_import_calibration,
    OPENCV_CAM_OT_export_calibration,
    OPENCV_CAM_OT_sync_from_lens,
    OPENCV_CAM_OT_apply_pose,
    OPENCV_CAM_OT_read_pose,
    OPENCV_CAM_OT_selftest,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)