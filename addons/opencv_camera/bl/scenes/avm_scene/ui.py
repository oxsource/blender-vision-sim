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

CATEGORY = "AVM Scene"


def _overview(layout, settings) -> None:
    field = settings.field_spec()
    width, height = avm_layout.scene_size(field)
    block_area = 4.0 * (field.corner * avm_layout.CM_TO_M) ** 2
    box = layout.box()
    box.label(text=f"Field {width:.0f} x {height:.0f} cm", icon="MESH_GRID")
    box.label(text=f"      {width / 100:.2f} x {height / 100:.2f} m")
    box.label(text=f"Blocks {block_area:.2f} m2  |  revision {settings.revision}")


def _field(layout, settings) -> None:
    """The seven field sliders, in the order of PlaneSceneActivity / the HTML tool."""
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Field (cm)")
    for name in ("border_w", "border_h", "corner", "core_w", "core_h",
                 "inner_w", "inner_h"):
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
    column.label(text="Ground / Blocks / Sun")
    column.prop(settings, "use_ground_model")
    model = column.column(align=True)
    model.enabled = settings.use_ground_model
    model.prop(settings, "ground_model", text="Model")
    if settings.use_ground_model and not (settings.ground_model or "").strip():
        model.label(text="Bundled: unlit_round_bowls.glb", icon="MESH_DATA")
    model.prop(settings, "ground_radius")
    model.prop(settings, "ground_rim_height")
    plane = column.column(align=True)
    plane.enabled = not settings.use_ground_model
    plane.prop(settings, "ground_w")
    plane.prop(settings, "ground_d")
    column.prop(settings, "block_lift")
    column.prop(settings, "sun_energy")
    column.prop(settings, "sun_shadow")


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
        detect = box.row(align=True)
        button = detect.operator("opencv_cam.avm_detect_camera",
                                 text="Detect Corners", icon="TRACKER")
        button.name = record.name
        view = detect.operator("opencv_cam.avm_show_corners", text="", icon="IMAGE_DATA")
        view.name = record.name
        clear = detect.operator("opencv_cam.avm_clear_camera", text="", icon="X")
        clear.name = record.name
        if record.points_2d_ok:
            detect.label(text=f"{record.points_2d_error:.2f} px", icon="CHECKMARK")
            values = list(record.points_2d)
            grid = box.grid_flow(columns=2, align=True, even_columns=True)
            for index in range(8):
                grid.label(text=f"P{index} {values[2 * index]:.1f},{values[2 * index + 1]:.1f}")
    layout.label(text="Intrinsics live in CV Intrinsics", icon="INFO")


def _props(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Props around the field")
    column.prop(settings, "prop_pedestrians")
    column.prop(settings, "prop_boxes")
    column.prop(settings, "prop_carts")


def _ground_text(layout, settings) -> None:
    column = layout.column(align=True)
    column.use_property_split = True
    column.label(text="Ground text")
    column.prop(settings, "logo_enabled")
    row = column.row()
    row.enabled = settings.logo_enabled
    row.prop(settings, "logo_size")


def _layers(layout, settings) -> None:
    box = layout.box()
    box.label(text="Show / Hide", icon="HIDE_OFF")
    column = box.column(align=True)
    row = column.row(align=True)
    for name in ("show_ground", "show_car", "show_blocks", "show_props"):
        row.prop(settings, name, toggle=True)
    row = column.row(align=True)
    for name in ("show_cameras", "show_labels", "show_coverage", "show_sun"):
        row.prop(settings, name, toggle=True)


def _actions(layout, context) -> None:
    row = layout.row(align=True)
    row.operator("opencv_cam.avm_rebuild", icon="FILE_REFRESH")
    row.operator("opencv_cam.avm_reset_defaults", icon="LOOP_BACK")
    layout.operator("opencv_cam.frame_view", text="Frame View",
                    icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id
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
    box.operator("opencv_cam.avm_export_falcon", text="Export Falcon", icon="EXPORT")
    if settings.io_status:
        box.label(text=settings.io_status)


def _coverage_section(layout, settings) -> None:
    box = layout.box()
    box.label(text="Coverage", icon="MESH_GRID")
    box.operator("opencv_cam.avm_analyze_coverage", icon="DRIVER_DISTANCE")
    if settings.coverage_status:
        box.label(text=settings.coverage_status)
    if settings.coverage_matrix:
        for part in settings.coverage_matrix.split(" | "):
            box.label(text=part, icon="CHECKMARK")
    box.operator("opencv_cam.avm_export_materials", icon="PACKAGE")


class _AVMPanel(ScenePanel):
    definition = DEFINITION

    def draw_layout(self, layout, context):
        settings = context.scene.avm_scene
        _overview(layout, settings)
        _field(layout, settings)
        _car(layout, settings)
        _ground_and_blocks(layout, settings)
        _props(layout, settings)
        _ground_text(layout, settings)
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
        _coverage_section(layout, context.scene.avm_scene)
        _io_section(layout, context.scene.avm_scene)


class OPENCV_CAM_PT_avm_scene_view3d(_AVMPanel, bpy.types.Panel):
    bl_idname = "OPENCV_CAM_PT_avm_scene_view3d"
    bl_label = "AVM Scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY
    bl_context = ""  # bl_context is a PROPERTIES concept; leave it unset here
    bl_order = -10

    def draw(self, context):
        layout = self.layout
        self.draw_layout(layout, context)
        row = layout.row(align=True)
        row.operator("opencv_cam.avm_rebuild", icon="FILE_REFRESH")
        row.operator("opencv_cam.avm_reset_defaults", icon="LOOP_BACK")
        layout.operator("opencv_cam.frame_view", text="Frame View",
                        icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id
        row = layout.row(align=True)
        row.operator("opencv_cam.avm_export_falcon",
                     text="Export Falcon", icon="EXPORT")
        row.operator("opencv_cam.avm_export_bowl",
                     text="Export Bowl", icon="MESH_DATA")


_CLASSES = (OPENCV_CAM_PT_avm_scene, OPENCV_CAM_PT_avm_scene_view3d)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
