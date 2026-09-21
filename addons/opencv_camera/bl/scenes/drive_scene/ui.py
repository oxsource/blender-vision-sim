"""Drive Scene panels.

Two panels, both shown **only while the scene is built** (``has_scene``):

* ``Drive Scene`` in the Scene Properties - everything, including the recording;
* ``Drive Scene`` in the 3D viewport N panel - **layout only**, plus the record
  button, so the driver's seat workflow (tweak, scrub, record) needs no tab
  switching.

Camera intrinsics are deliberately absent: they stay on ``camera.data.opencv_cam``
and are edited in ``CV Intrinsics`` / ``CV Presets``, like in every other scene.
The only camera setting this scene owns is *which ones a clip records*.
"""

from __future__ import annotations

import bpy

from ....core.scenes import avm_cameras, drive_path
from ..base import ScenePanel
from . import DEFINITION

CATEGORY = "Drive Scene"


def _overview(layout, settings) -> None:
    length, width = settings.lot_size()
    plan = settings.plan()
    box = layout.box()
    box.label(text=f"Car park {length:.0f} x {width:.0f} m", icon="MESH_GRID")
    box.label(text=drive_path.summary(plan))
    per_frame = settings.drive_speed / max(1, settings.drive_fps)
    box.label(text=f"{per_frame:.3f} m per frame at {settings.drive_speed:.2f} m/s")


def _lot(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Car park")
    for name in ("aisle_length", "aisle_width", "bay_depth", "bay_width"):
        column.prop(settings, name)
    column.prop(settings, "ground_texture")
    column.prop(settings, "show_bays")
    column.prop(settings, "bay_numbers")
    column.prop(settings, "parked_cars")
    column.prop(settings, "pillar_count")
    column.prop(settings, "light_energy")
    column.prop(settings, "shadows")


def _car(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Vehicle")
    for name in ("car_length", "car_width", "car_height", "car_clearance"):
        column.prop(settings, name)


def _drive(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Drive")
    for name in ("drive_distance", "drive_speed", "drive_profile"):
        column.prop(settings, name)
    if settings.drive_profile == "trapezoid":
        column.prop(settings, "drive_accel")
    for name in ("drive_fps", "drive_heading"):
        column.prop(settings, name)


def _layers(layout, settings) -> None:
    box = layout.box()
    box.label(text="Show / Hide", icon="HIDE_OFF")
    row = box.row(align=True)
    for name in ("show_ground", "show_walls", "show_parked", "show_car"):
        row.prop(settings, name, toggle=True)


def _cameras(layout, settings) -> None:
    """The four cameras: which one the viewport shows, which ones a clip records.

    The switches are the only camera setting this scene owns - K / D / output and
    the mount pose come from the AVM Scene's calibration preset, so they are
    edited in ``CV Intrinsics`` / ``CV Presets``, not here.
    """
    box = layout.box()
    box.label(text="Cameras", icon="CAMERA_DATA")
    box.use_property_split = True
    box.prop(settings, "active_camera")
    recorded = settings.recorded_cameras()
    row = box.row(align=True)
    for key in avm_cameras.CAMERAS:
        entry = settings.camera(key)
        if entry is not None:
            row.prop(entry, "enable", toggle=True, text=avm_cameras.LABEL[key])
    box.label(text=f"{len(recorded)} of {len(avm_cameras.CAMERAS)} recorded",
              icon="RENDER_STILL" if recorded else "ERROR")
    box.label(text="K / D / mount come from the AVM calibration", icon="INFO")
    box.operator("opencv_cam.drive_sync_cameras", icon="IMPORT")


def _record(layout, settings) -> None:
    box = layout.box()
    box.label(text="Record", icon="RENDER_ANIMATION")
    column = box.column(align=True)
    column.use_property_split = True
    column.prop(settings, "clip_quality")
    column.prop(settings, "clip_device")
    column.prop(settings, "clip_keep_frames")
    box.operator("opencv_cam.drive_export_zip", text="Export Clip…", icon="EXPORT")
    if settings.clip_status:
        running = settings.clip_status.startswith("frame ")
        box.label(text=settings.clip_status,
                  icon="TIME" if running else "CHECKMARK")
        if running:
            box.label(text="press ESC to cancel", icon="INFO")


class _DrivePanel(ScenePanel):
    definition = DEFINITION

    def draw_layout(self, layout, context):
        settings = context.scene.drive_scene
        _overview(layout, settings)
        _lot(layout, settings)
        _car(layout, settings)
        _drive(layout, settings)
        _layers(layout, settings)


class OPENCV_CAM_PT_drive_scene(_DrivePanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_drive_scene"
    bl_label = "Drive Scene"
    bl_order = -30

    def draw(self, context):
        layout = self.layout
        settings = context.scene.drive_scene
        self.draw_layout(layout, context)
        _record(layout, settings)
        row = layout.row(align=True)
        row.operator("opencv_cam.drive_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.drive_reset_defaults", icon="LOOP_BACK")
        layout.operator("opencv_cam.frame_view", text="Frame View",
                        icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id
        layout.operator("opencv_cam.drive_remove_scene", icon="TRASH")


class OPENCV_CAM_PT_drive_scene_view3d(_DrivePanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_drive_scene_view3d"
    bl_label = "Drive Scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY
    bl_context = ""  # bl_context is a PROPERTIES concept; leave it unset here
    bl_order = -10

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        _record(layout, context.scene.drive_scene)
        row = layout.row(align=True)
        row.operator("opencv_cam.drive_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.drive_reset_defaults", icon="LOOP_BACK")
        layout.operator("opencv_cam.frame_view", text="Frame View",
                        icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id


_CLASSES = (OPENCV_CAM_PT_drive_scene, OPENCV_CAM_PT_drive_scene_view3d)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
