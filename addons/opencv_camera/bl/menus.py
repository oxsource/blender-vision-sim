"""Menus.

Everything the add-on adds lives under :menuselection:`Add ▸ VisionSim`:

* ``Camera ▸ ...`` - create a camera configured with an OpenCV lens model
* ``Test Scene`` - checker cube/ground/lights for a quick distortion check
* ``Camera Rig`` - empty to parent cameras to (extrinsics / multi-camera)

The camera entries are *not* also appended to :menuselection:`Add ▸ Camera`:
Blender does not let add-ons extend ``Camera.type``, so an entry there could only
ever create a *Custom* lens camera anyway - one place to look is clearer.
"""

from __future__ import annotations

import bpy

from . import camera_factory, icons

MENU_ID = "OPENCV_CAM_MT_vision_sim"
MENU_CAMERA_ID = "OPENCV_CAM_MT_camera"
MENU_LABEL = "VisionSim"


class OPENCV_CAM_MT_camera(bpy.types.Menu):
    bl_idname = MENU_CAMERA_ID
    bl_label = "Camera"

    def draw(self, context):
        layout = self.layout
        for model, (label, _, _) in camera_factory.MODELS.items():
            icons.operator(layout, "opencv_cam.add_camera", label, model=model)


class OPENCV_CAM_MT_vision_sim(bpy.types.Menu):
    bl_idname = MENU_ID
    bl_label = MENU_LABEL

    def draw(self, context):
        layout = self.layout
        layout.menu(MENU_CAMERA_ID, **_submenu_kwargs("Camera"))
        layout.separator()
        icons.operator(layout, "opencv_cam.add_test_scene", "Test Scene", fallback_icon="MESH_CUBE")
        icons.operator(layout, "opencv_cam.add_rig_empty", "Camera Rig", fallback_icon="EMPTY_AXIS")


def _submenu_kwargs(text: str = MENU_LABEL) -> dict:
    """``layout.menu`` arguments with our own icon (built-in name as fallback)."""
    value = icons.icon_id()
    if value > 0:
        return {"text": text, "icon_value": value}
    return {"text": text, "icon": "TRACKING"}  # crosshair, not another camera icon


def _menu_add(self, context):
    """Draw the VisionSim submenu inside Add."""
    self.layout.menu(MENU_ID, **_submenu_kwargs())


_CLASSES = (OPENCV_CAM_MT_camera, OPENCV_CAM_MT_vision_sim)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_add.append(_menu_add)


def unregister() -> None:
    bpy.types.VIEW3D_MT_add.remove(_menu_add)
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)