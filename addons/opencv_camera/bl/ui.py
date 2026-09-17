"""Panels.

Every block of the add-on is a **top level** panel in the camera's Object Data
Properties (independent of Blender's *Lens* panel), carries a ``CV `` prefix and
sorts *before* Blender's own panels (negative ``bl_order``):

* ``CV Intrinsics``  - lens model, fx, fy, cx, cy, distortion coefficients,
                       calibration size, then Apply / Preview / Live Apply /
                       Recompile and the status box
* ``CV Extrinsics``  - Euler / R / t, plus a collapsible *World Frame* sub-block
* ``CV Presets``     - calibration files, presets, defaults
* ``CV Preview``     - preview render settings and tools
* ``CV Output``      - the image size the render should produce
"""

from __future__ import annotations

import bpy

from . import apply as apply_mod
from . import icons, panels_patch, preview as preview_mod, shader

#: negative orders put our panels at the top of the camera data properties
BASE_ORDER = -50


class _CameraPanel:
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.camera is not None


class OPENCV_CAM_PT_main(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_main"
    bl_label = "CV Intrinsics"
    bl_order = BASE_ORDER

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        cam_data = context.camera
        settings = cam_data.opencv_cam
        scene = context.scene
        intrinsics = settings.intrinsics
        distortion = settings.distortion

        # ---- lens model + intrinsics --------------------------------------
        layout.prop(distortion, "model")
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

        # ---- distortion ----------------------------------------------------
        layout.prop(distortion, "enabled")
        column = layout.column()
        column.enabled = distortion.enabled
        if distortion.model == "fisheye":
            column.label(text="theta_d = t (1 + k1 t² + k2 t⁴ + k3 t⁶ + k4 t⁸)")
            for name in ("k1", "k2", "k3", "k4"):
                column.prop(distortion, name)
        else:
            for name in ("k1", "k2", "p1", "p2", "k3"):
                column.prop(distortion, name)
            if distortion.model == "rational" or distortion.k4 or distortion.k5 or distortion.k6:
                for name in ("k4", "k5", "k6"):
                    column.prop(distortion, name)
        column.prop(distortion, "iterations")
        column.prop(distortion, "discard_invalid_rays")

        # ---- calibration size ----------------------------------------------
        column = layout.column()
        column.prop(intrinsics, "image_width")
        column.prop(intrinsics, "image_height")
        column.prop(intrinsics, "scale_to_render")
        layout.operator("opencv_cam.sync_from_lens", icon="DRIVER_ROTATIONAL_DIFFERENCE")

        effective = apply_mod.effective_intrinsics(
            settings, scene.render.resolution_x, scene.render.resolution_y
        )
        box = layout.box()
        box.label(text=f"render: fx={effective.fx:.2f} fy={effective.fy:.2f} "
                       f"cx={effective.cx:.2f} cy={effective.cy:.2f}")
        box.label(text=f"hfov={effective.hfov_deg():.2f}°  vfov={effective.vfov_deg():.2f}°")

        # ---- actions, after the parameters ---------------------------------
        layout.separator()
        column = layout.column(align=True)
        icons.operator(column, "opencv_cam.apply_settings", "Apply", name="visionsim")
        column.operator("opencv_cam.preview", icon="RENDER_STILL")
        row = layout.row(align=True)
        row.prop(settings, "auto_apply", toggle=True)
        row.operator("opencv_cam.recompile", icon="FILE_REFRESH")

        # ---- status --------------------------------------------------------
        box = layout.box()
        compiled = shader.is_compiled(cam_data) and cam_data.type == "CUSTOM"
        box.label(
            text=(f"{shader.shader_filename(distortion.model)} "
                  f"({len(cam_data.custom_bytecode)} chars bytecode)") if compiled
            else "shader not compiled yet - press Apply",
            icon="CHECKMARK" if compiled else "ERROR",
        )
        if settings.status:
            box.label(text=f"status: {settings.status}")
        for note in apply_mod.resolution_notes(settings, scene):
            box.label(text=note, icon="INFO")
        box.prop(settings, "show_raw_params")
        if not panels_patch.is_patched():
            box.label(text="Cycles parameter list is shown (patch inactive)", icon="INFO")


class OPENCV_CAM_PT_extrinsics(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_extrinsics"
    bl_label = "CV Extrinsics"
    bl_order = BASE_ORDER + 1

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        pose = context.camera.opencv_cam.pose

        # three angles first: the friendly way to dial in an orientation
        box = layout.box()
        box.label(text="Euler (XYZ, Blender world)")
        box.prop(pose, "euler")

        # the OpenCV-native representation, visually separated from the angles
        layout.separator()
        layout.label(text="OpenCV Pose")
        column = layout.column()
        column.prop(pose, "rotation")
        column.prop(pose, "translation")

        row = layout.row(align=True)
        row.operator("opencv_cam.apply_pose", icon="OBJECT_ORIGIN")
        row.operator("opencv_cam.read_pose", icon="TRACKER")
        layout.label(text="Euler and R describe the same rotation", icon="INFO")


class OPENCV_CAM_PT_world_frame(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_world_frame"
    bl_label = "World Frame"
    bl_parent_id = "OPENCV_CAM_PT_extrinsics"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        pose = context.camera.opencv_cam.pose

        layout.prop(pose, "use_world_transform")
        row = layout.row()
        row.enabled = pose.use_world_transform
        row.prop(pose, "world_matrix")
        layout.label(text="Maps the calibration world frame to the Blender world", icon="INFO")


class OPENCV_CAM_PT_output(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_output"
    bl_label = "CV Output"
    bl_order = BASE_ORDER + 4

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        intrinsics = settings.intrinsics
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
        box = layout.box()
        box.label(text=f"output {width}x{height}  |  scene "
                       f"{scene.render.resolution_x}x{scene.render.resolution_y}")
        box.label(text=f"calibrated {intrinsics.image_width}x{intrinsics.image_height}")
        effective = apply_mod.effective_intrinsics(settings, width, height)
        box.label(text=f"effective fx={effective.fx:.2f} fy={effective.fy:.2f}")
        if (width, height) != (intrinsics.image_width, intrinsics.image_height):
            same_aspect = (height > 0 and intrinsics.image_height > 0
                           and abs(width / height - intrinsics.image_width / intrinsics.image_height) < 1e-3)
            box.label(text="rescaled: same field of view" if same_aspect
                      else "centre crop at the original pixel pitch", icon="INFO")


class OPENCV_CAM_PT_io(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_io"
    bl_label = "CV Presets"
    bl_order = BASE_ORDER + 2
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

        layout.separator()
        column = layout.column(align=True)
        column.operator("opencv_cam.load_preset", icon="PRESET")
        column.operator("opencv_cam.reset_defaults", icon="LOOP_BACK")
        layout.label(text="Presets come from the add-on's presets/ folder", icon="INFO")


class OPENCV_CAM_PT_preview(_CameraPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_preview"
    bl_label = "CV Preview"
    bl_order = BASE_ORDER + 3
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.camera.opencv_cam
        preview = settings.preview

        column = layout.column(align=True)
        column.prop(preview, "size")
        column.prop(preview, "samples")
        column.prop(preview, "denoise")
        column.prop(preview, "on_change")

        width, height = preview_mod.preview_resolution(settings, preview.size)
        layout.label(
            text=f"preview {width}x{height} ({preview.samples} samples) - F12 uses "
                 f"{context.scene.render.resolution_x}x{context.scene.render.resolution_y}",
            icon="INFO",
        )
        column = layout.column(align=True)
        column.operator("opencv_cam.preview", icon="RENDER_STILL")
        column.operator("opencv_cam.save_preview", icon="FILE_IMAGE")
        column.operator("opencv_cam.self_test", icon="RESTRICT_RENDER_OFF")


#: registration order does not matter (bl_order decides), sorted here for reading
_CLASSES = (
    OPENCV_CAM_PT_main,
    OPENCV_CAM_PT_extrinsics,
    OPENCV_CAM_PT_world_frame,
    OPENCV_CAM_PT_io,
    OPENCV_CAM_PT_preview,
    OPENCV_CAM_PT_output,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)