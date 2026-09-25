"""Road Scene panels.

Two panels, both shown only while the scene is built (``has_scene``): one in the
Scene Properties and a layout-only twin in the 3D viewport N panel.  Camera
intrinsics are deliberately absent - they stay on ``camera.data.opencv_cam`` and
are edited in ``CV Intrinsics`` / ``CV Presets``.
"""

from __future__ import annotations

import bpy

from ....core.scenes import avm_cameras, road_path
from ..base import ScenePanel
from . import DEFINITION

CATEGORY = "Road Scene"


def _overview(layout, settings) -> None:
    box = layout.box()
    track = settings.track()
    plan = settings.plan()
    box.label(text=f"Loop {track.length:.1f} m, {len(track.segments)} segments",
              icon="MESH_GRID")
    box.label(text=road_path.summary(plan))
    recorded = max(1, len(settings.recorded_cameras()))
    box.label(text=f"{len(plan.frames) * recorded} stills to render",
              icon="RENDER_STILL")


def _track(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Track")
    column.prop(settings, "track_preset")
    for name in ("straight_length", "curve_radius", "curve_bank_deg", "ramp_rise", "ramp_length",
                 "road_width", "shoulder_width"):
        column.prop(settings, name)
    column.prop(settings, "ground_texture")
    column.prop(settings, "parking_bays")
    column.prop(settings, "parking_speed")


def _props(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Roadside")
    for name in ("pedestrians", "trees", "lamps", "signs"):
        column.prop(settings, name)
    column.prop(settings, "animate_pedestrians")
    if settings.animate_pedestrians:
        column.prop(settings, "pedestrian_speed")


def _lighting(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Lighting")
    column.prop(settings, "light_energy")


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
    column.prop(settings, "drive_profile")
    column.prop(settings, "drive_speed")
    if settings.drive_profile == "scenario":
        column.prop(settings, "slow_speed")
        column.prop(settings, "drive_accel")
        column.prop(settings, "drive_decel")
    elif settings.drive_profile == "trapezoid":
        column.prop(settings, "drive_accel")
    for name in ("drive_fps", "drive_direction", "drive_segment"):
        column.prop(settings, name)
    parking_lap = (settings.drive_profile == "scenario" and settings.parking
                   and settings.drive_segment == "loop")
    if settings.drive_segment == "loop" and not parking_lap:
        column.prop(settings, "drive_loops")


def _cameras(layout, settings) -> None:
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
    box.operator("opencv_cam.road_sync_cameras", icon="IMPORT")


def _record(layout, settings) -> None:
    box = layout.box()
    box.label(text="Record", icon="RENDER_ANIMATION")
    column = box.column(align=True)
    column.use_property_split = True
    column.prop(settings, "clip_quality")
    column.prop(settings, "clip_device")
    column.prop(settings, "clip_keep_frames")
    box.operator("opencv_cam.road_export_zip", text="Export Clip…", icon="EXPORT")
    if settings.clip_status:
        running = settings.clip_status.startswith("frame ")
        box.label(text=settings.clip_status,
                  icon="TIME" if running else "CHECKMARK")
        if running:
            box.label(text="press ESC to cancel", icon="INFO")


class _RoadPanel(ScenePanel):
    definition = DEFINITION

    def draw_layout(self, layout, context):
        settings = context.scene.road_scene
        _overview(layout, settings)
        _track(layout, settings)
        _props(layout, settings)
        _lighting(layout, settings)
        _car(layout, settings)
        _drive(layout, settings)


class OPENCV_CAM_PT_road_scene(_RoadPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_road_scene"
    bl_label = "Road Scene"
    bl_order = -40

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        _cameras(layout, context.scene.road_scene)
        _record(layout, context.scene.road_scene)
        row = layout.row(align=True)
        row.operator("opencv_cam.road_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.road_reset_defaults", icon="LOOP_BACK")
        layout.operator("opencv_cam.frame_view", text="Frame View",
                        icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id
        layout.operator("opencv_cam.road_remove_scene", icon="TRASH")


class OPENCV_CAM_PT_road_scene_view3d(_RoadPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_road_scene_view3d"
    bl_label = "Road Scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY
    bl_context = ""
    bl_order = -20

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        _cameras(layout, context.scene.road_scene)
        _record(layout, context.scene.road_scene)
        row = layout.row(align=True)
        row.operator("opencv_cam.road_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.road_reset_defaults", icon="LOOP_BACK")
        layout.operator("opencv_cam.frame_view", text="Frame View",
                        icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id


_CLASSES = (OPENCV_CAM_PT_road_scene, OPENCV_CAM_PT_road_scene_view3d)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
