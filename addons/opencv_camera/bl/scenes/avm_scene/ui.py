"""AVM Scene panels.

Two panels, both shown **only while the scene is built** (``has_scene``, i.e. the
``AVM_Root`` pointer exists):

* ``AVM Scene`` in the Scene Properties - everything, including the JSON import /
  export (P4) and the coverage tools (P5);
* ``AVM Scene`` in the 3D viewport N panel - **layout only**: sizes, positions,
  poses and layer toggles.  No camera intrinsics (those stay in ``CV Intrinsics``)
  and no import/export (see ``docs/avm-scene.md`` §5.2).
"""

from __future__ import annotations

import bpy

from ....core.scenes import avm_layout
from ..base import ScenePanel
from . import DEFINITION

CATEGORY = "VisionSim"


def _overview(layout, settings) -> None:
    field = settings.field_spec()
    width, height = avm_layout.scene_size(field)
    block_area = 4.0 * (field.corner * avm_layout.CM_TO_M) ** 2
    box = layout.box()
    box.label(text=f"Field {width:.0f} x {height:.0f} cm", icon="MESH_GRID")
    box.label(text=f"Blocks {block_area:.2f} m2  |  revision {settings.revision}")


def _field(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Field (cm)")
    for name in ("border_w", "border_h", "corner", "inner_w", "inner_h",
                 "core_w", "core_h"):
        column.prop(settings, name)


def _car(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Car")
    column.prop(settings, "car_follow_core")
    if not settings.car_follow_core:
        column.prop(settings, "car_length")
        column.prop(settings, "car_width")
    column.prop(settings, "car_height")
    column.prop(settings, "car_clearance")


def _ground_and_blocks(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Ground / Blocks")
    column.prop(settings, "ground_w")
    column.prop(settings, "ground_d")
    column.prop(settings, "block_lift")


def _cameras(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Cameras (mount pose)")
    column.prop(settings, "active_camera")
    for record in settings.cameras:
        box = layout.box()
        row = box.row(align=True)
        row.prop(record, "enable", text="")
        row.label(text=record.name.capitalize())
        select = row.operator("opencv_cam.avm_select_camera", text="", icon="RESTRICT_SELECT_OFF")
        select.name = record.name
        column = box.column(align=True)
        column.use_property_split = True
        column.prop(record, "location")
        column.prop(record, "rotation")
    layout.label(text="Intrinsics live in CV Intrinsics", icon="INFO")


def _layers(layout, settings) -> None:
    column = layout.column(align=True)
    column.label(text="Layers")
    row = column.row(align=True)
    for name in ("show_ground", "show_car", "show_blocks"):
        row.prop(settings, name, toggle=True)
    row = column.row(align=True)
    row.prop(settings, "show_cameras", toggle=True)
    row.prop(settings, "show_coverage", toggle=True)


def _actions(layout, context) -> None:
    row = layout.row(align=True)
    row.operator("opencv_cam.avm_rebuild", icon="FILE_REFRESH")
    row.operator("opencv_cam.avm_reset_defaults", icon="LOOP_BACK")
    layout.operator_menu_enum("opencv_cam.avm_apply_preset", "preset",
                              text="Quick Preset", icon="PRESET")
    layout.operator("opencv_cam.avm_remove_scene", icon="TRASH")


def _io_section(layout, settings) -> None:
    box = layout.box()
    box.label(text="Export / Import", icon="FILE_TEXT")
    box.prop(settings, "io_text", text="")
    row = box.row(align=True)
    row.operator("opencv_cam.avm_export_json", icon="EXPORT")
    row.operator("opencv_cam.avm_apply_json", icon="IMPORT")
    row = box.row(align=True)
    row.operator("opencv_cam.avm_export_params", icon="FILE_TICK")
    row.operator("opencv_cam.avm_import_params", icon="FILEBROWSER")
    box.operator("opencv_cam.avm_render_cameras", icon="RENDER_STILL")
    if settings.io_status:
        box.label(text=settings.io_status)


class _AVMPanel(ScenePanel):
    definition = DEFINITION

    def draw_layout(self, layout, context):
        settings = context.scene.avm_scene
        _overview(layout, settings)
        _field(layout, settings)
        _car(layout, settings)
        _ground_and_blocks(layout, settings)
        _cameras(layout, settings)
        _layers(layout, settings)


class OPENCV_CAM_PT_avm_scene(_AVMPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_avm_scene"
    bl_label = "AVM Scene"
    bl_order = -20

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        _actions(layout, context)
        _io_section(layout, context.scene.avm_scene)


class OPENCV_CAM_PT_avm_scene_view3d(_AVMPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_avm_scene_view3d"
    bl_label = "AVM Scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        row = layout.row(align=True)
        row.operator("opencv_cam.avm_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.avm_reset_defaults", icon="LOOP_BACK")


_CLASSES = (OPENCV_CAM_PT_avm_scene, OPENCV_CAM_PT_avm_scene_view3d)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
