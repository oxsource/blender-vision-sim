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
        column.operator("opencv_cam.preview", icon="RENDER_STILL")
        column.operator("opencv_cam.add_test_scene", icon="MESH_CUBE")

        row = layout.row(align=True)
        row.prop(settings, "auto_apply", toggle=True)
        row.operator("opencv_cam.recompile", icon="FILE_REFRESH")

        box = layout.box()
        compiled = shader.is_compiled(cam_data) and cam_data.type == "CUSTOM"
        box.label(
            text=(f"{shader.shader_filename(settings.distortion.model)} "
                  f"({len(cam_data.custom_bytecode)} chars bytecode)") if compiled
            else "shader not compiled yet - press Apply to Camera",
            icon="CHECKMARK" if compiled else "ERROR",
        )
        if settings.status:
            box.label(text=f"status: {settings.status}")
        for note in apply_mod.resolution_notes(settings, scene):
            box.label(text=note, icon="INFO")


class OPENCV_CAM_PT_preview(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_preview"
    bl_label = "Preview"
    bl_parent_id = "OPENCV_CAM_PT_main"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        preview = settings.preview

        row = layout.row(align=True)
        row.prop(preview, "size", text="")
        row.prop(preview, "samples", text="")

        column = layout.column(align=True)
        column.operator("opencv_cam.preview", icon="RENDER_STILL")
        column.operator("opencv_cam.save_preview", icon="FILE_IMAGE")
        column.operator("opencv_cam.self_test", icon="RESTRICT_RENDER_OFF")

        layout.prop(preview, "denoise")
        layout.prop(preview, "on_change")
        layout.label(
            text="Preview renders into Blender's Image Editor (F12 puts it there too)",
            icon="INFO",
        )
        layout.label(
            text="Viewport preview depends on Cycles supporting custom cameras there",
            icon="INFO",
        )


class OPENCV_CAM_PT_presets(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_presets"
    bl_label = "Presets"
    bl_parent_id = "OPENCV_CAM_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        settings = context.camera.opencv_cam
        column = layout.column(align=True)
        column.operator("opencv_cam.load_preset", icon="PRESET")
        column.operator("opencv_cam.reset_defaults", icon="LOOP_BACK")
        if settings.calibration.last_import:
            layout.label(text=f"last: {settings.calibration.last_import}")
        layout.label(text="Drop .yaml/.json files into the presets/ folder", icon="INFO")


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


class OPENCV_CAM_PT_output(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_output"
    bl_label = "Output Image"
    bl_parent_id = "OPENCV_CAM_PT_main"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        output = settings.output
        scene = context.scene

        layout.prop(output, "mode")
        if output.mode == "custom":
            layout.prop(output, "preset")
            column = layout.column(align=True)
            column.prop(output, "width")
            column.prop(output, "height")
        if output.mode != "scene":
            layout.prop(output, "lock_scene_resolution")
            row = layout.row(align=True)
            row.operator("opencv_cam.set_render_resolution", icon="FULLSCREEN_ENTER")
            row.operator("opencv_cam.read_scene_resolution", icon="FULLSCREEN_EXIT")

        width, height = apply_mod.output_resolution(settings, scene)
        intrinsics = settings.intrinsics
        box = layout.box()
        box.label(
            text=f"output {width}x{height}  |  scene "
                 f"{scene.render.resolution_x}x{scene.render.resolution_y}"
        )
        box.label(
            text=f"calibrated {intrinsics.image_width}x{intrinsics.image_height}"
        )
        effective = apply_mod.effective_intrinsics(settings, width, height)
        box.label(text=f"effective fx={effective.fx:.2f} fy={effective.fy:.2f}")
        if (width, height) != (intrinsics.image_width, intrinsics.image_height):
            same_aspect = (height > 0 and intrinsics.image_height > 0
                           and abs(width / height - intrinsics.image_width / intrinsics.image_height) < 1e-3)
            box.label(
                text="rescaled: same field of view" if same_aspect
                else "centre crop at the original pixel pitch",
                icon="INFO",
            )


class OPENCV_CAM_PT_distortion(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_distortion"
    bl_label = "Distortion"
    bl_parent_id = "OPENCV_CAM_PT_main"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        distortion = context.camera.opencv_cam.distortion
        model = distortion.model

        layout.prop(distortion, "model")
        layout.prop(distortion, "enabled")
        column = layout.column()
        column.enabled = distortion.enabled
        if model == "fisheye":
            layout.label(text="theta_d = t (1 + k1 t² + k2 t⁴ + k3 t⁶ + k4 t⁸)")
            column.prop(distortion, "k1")
            column.prop(distortion, "k2")
            column.prop(distortion, "k3")
            column.prop(distortion, "k4")
        else:
            column.prop(distortion, "k1")
            column.prop(distortion, "k2")
            column.prop(distortion, "p1")
            column.prop(distortion, "p2")
            column.prop(distortion, "k3")
            if model == "rational" or distortion.k4 or distortion.k5 or distortion.k6:
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
    OPENCV_CAM_PT_preview,
    OPENCV_CAM_PT_presets,
    OPENCV_CAM_PT_intrinsics,
    OPENCV_CAM_PT_output,
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