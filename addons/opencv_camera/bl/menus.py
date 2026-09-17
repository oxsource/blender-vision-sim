"""Menus.

Everything the add-on adds lives under :menuselection:`Add ▸ vision-sim`:

* ``Camera ▸ ...`` - create a camera configured with an OpenCV lens model
* ``Test Scene`` - checker cube/ground/lights for a quick distortion check

The camera entries are *not* also appended to :menuselection:`Add ▸ Camera`:
Blender does not let add-ons extend ``Camera.type``, so an ``Add ▸ Camera`` entry
would only ever create a *Custom* lens camera anyway - one place to look is
clearer.
"""

from __future__ import annotations

import bpy

from . import camera_factory

MENU_ID = "OPENCV_CAM_MT_vision_sim"
MENU_CAMERA_ID = "OPENCV_CAM_MT_camera"


class OPENCV_CAM_MT_camera(bpy.types.Menu):
    bl_idname = MENU_CAMERA_ID
    bl_label = "Camera"

    def draw(self, context):
        layout = self.layout
        for model, (label, _, _) in camera_factory.MODELS.items():
            layout.operator("opencv_cam.add_camera", text=label, icon="CAMERA_DATA").model = model


class OPENCV_CAM_MT_vision_sim(bpy.types.Menu):
    bl_idname = MENU_ID
    bl_label = "vision-sim"

    def draw(self, context):
        layout = self.layout
        layout.menu(MENU_CAMERA_ID, icon="CAMERA_DATA")
        layout.separator()
        layout.operator("opencv_cam.add_test_scene", icon="MESH_CUBE")
        layout.operator("opencv_cam.add_rig_empty", icon="EMPTY_AXIS")


def _menu_add(self, context):
    """Draw the vision-sim submenu inside Add."""
    self.layout.menu(MENU_ID, icon="CAMERA_DATA")


_CLASSES = (OPENCV_CAM_MT_camera, OPENCV_CAM_MT_vision_sim)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_add.append(_menu_add)


def unregister() -> None:
    bpy.types.VIEW3D_MT_add.remove(_menu_add)
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)