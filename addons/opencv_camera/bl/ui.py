"""Panels: ``Object Data Properties ▸ Lens ▸ OpenCV Camera``."""

from __future__ import annotations

import bpy

from . import apply as apply_mod
from . import shader


class _CameraPanel:
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.camera is not None


class OPENCV_CAM_PT_main(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_main"
    bl_label = "OpenCV Camera"
    bl_parent_id = "DATA_PT_lens"

    def draw(self, context):
        layout = self.layout
        cam_data = context.camera
        settings = cam_data.opencv_cam
        scene = context.scene

        column = layout.column(align=True)
        column.operator("opencv_cam.apply_settings", icon="CHECKMARK")
        column.operator("opencv_cam.install_shader")

        box = layout.box()
        box.label(
            text=f"bytecode: {len(cam_data.custom_bytecode)} chars"
            if cam_data.type == "CUSTOM"
            else "camera is not in Custom mode yet",
            icon="INFO" if shader.is_compiled(cam_data) else "ERROR",
        )
        if settings.status:
            box.label(text=f"status: {settings.status}")
        for note in apply_mod.resolution_notes(settings, scene):
            box.label(text=note, icon="INFO")

        layout.operator("opencv_cam.self_test", icon="RESTRICT_RENDER_OFF")
        layout.prop(settings, "shader_text_name")


class OPENCV_CAM_PT_intrinsics(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_intrinsics"
    bl_label = "Intrinsics"
    bl_parent_id = "OPENCV_CAM_PT_main"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        intrinsics = settings.intrinsics
        scene = context.scene

        column = layout.column()
        column.prop(intrinsics, "fx")
        column.prop(intrinsics, "fy")
        column.prop(intrinsics, "auto_center")
        row = column.row()
        row.enabled = not intrinsics.auto_center
        row.prop(intrinsics, "cx")
        row = column.row()
        row.enabled = not intrinsics.auto_center
        row.prop(intrinsics, "cy")
        column.prop(intrinsics, "image_width")
        column.prop(intrinsics, "image_height")
        column.prop(intrinsics, "scale_to_render")

        effective = apply_mod.effective_intrinsics(
            settings, scene.render.resolution_x, scene.render.resolution_y
        )
        box = layout.box()
        box.label(text=f"render: fx={effective.fx:.2f} fy={effective.fy:.2f}")
        box.label(text=f"cx={effective.cx:.2f} cy={effective.cy:.2f}")
        box.label(text=f"hfov={effective.hfov_deg():.2f}° vfov={effective.vfov_deg():.2f}°")
        layout.operator("opencv_cam.sync_from_lens", icon="DRIVER_ROTATIONAL_DIFFERENCE")


class OPENCV_CAM_PT_distortion(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_distortion"
    bl_label = "Distortion"
    bl_parent_id = "OPENCV_CAM_PT_main"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        distortion = context.camera.opencv_cam.distortion

        layout.prop(distortion, "model")
        layout.prop(distortion, "enabled")
        column = layout.column()
        column.enabled = distortion.enabled
        column.prop(distortion, "k1")
        column.prop(distortion, "k2")
        column.prop(distortion, "p1")
        column.prop(distortion, "p2")
        column.prop(distortion, "k3")
        if distortion.model == "rational" or distortion.k4 or distortion.k5 or distortion.k6:
            column.prop(distortion, "k4")
            column.prop(distortion, "k5")
            column.prop(distortion, "k6")
        column.prop(distortion, "iterations")


class OPENCV_CAM_PT_calibration(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_calibration"
    bl_label = "Calibration File"
    bl_parent_id = "OPENCV_CAM_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        layout.prop(settings.calibration, "filepath")
        row = layout.row(align=True)
        row.operator("opencv_cam.import_calibration", icon="IMPORT")
        row.operator("opencv_cam.export_calibration", icon="EXPORT")
        if settings.calibration.last_import:
            layout.label(text=f"imported: {settings.calibration.last_import}")


class OPENCV_CAM_PT_pose(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_pose"
    bl_label = "Extrinsics (OpenCV)"
    bl_parent_id = "OPENCV_CAM_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        pose = context.camera.opencv_cam.pose

        column = layout.column()
        column.prop(pose, "rotation")
        column.prop(pose, "translation")
        column.prop(pose, "use_world_transform")
        row = layout.row()
        row.enabled = pose.use_world_transform
        row.prop(pose, "world_matrix")

        row = layout.row(align=True)
        row.operator("opencv_cam.apply_pose", icon="OBJECT_ORIGIN")
        row.operator("opencv_cam.read_pose", icon="TRACKER")


_CLASSES = (
    OPENCV_CAM_PT_main,
    OPENCV_CAM_PT_intrinsics,
    OPENCV_CAM_PT_distortion,
    OPENCV_CAM_PT_calibration,
    OPENCV_CAM_PT_pose,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)