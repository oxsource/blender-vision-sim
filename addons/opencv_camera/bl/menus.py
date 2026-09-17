"""Menus.

Everything the add-on adds lives under :menuselection:`Add ▸ VisionSim`, and every
entry carries its own line icon (see ``scripts/make_icon.py``):

* ``Camera ▸ ...`` - create a camera configured with an OpenCV lens model
* ``Camera Scene`` - checker cube/ground/lights for a quick distortion check

The camera entries are *not* also appended to :menuselection:`Add  Camera`:
Blender does not let add-ons extend ``Camera.type``, so an entry there could only
ever create a *Custom* lens camera anyway - one place to look is clearer.
"""

from __future__ import annotations

import bpy

from . import camera_factory, icons

MENU_ID = "OPENCV_CAM_MT_vision_sim"
MENU_CAMERA_ID = "OPENCV_CAM_MT_camera"
MENU_LABEL = "VisionSim"

#: distortion model -> icon name
MODEL_ICONS = {
    "fisheye": "fisheye",
    "brown_conrady": "brown_conrady",
    "rational": "rational",
    "pinhole": "pinhole",
}


class OPENCV_CAM_MT_camera(bpy.types.Menu):
    bl_idname = MENU_CAMERA_ID
    bl_label = "Camera"

    def draw(self, context):
        layout = self.layout
        for model, (label, _, _) in camera_factory.MODELS.items():
            icons.operator(layout, "opencv_cam.add_camera", label,
                           name=MODEL_ICONS.get(model, "camera"), model=model)


class OPENCV_CAM_MT_vision_sim(bpy.types.Menu):
    bl_idname = MENU_ID
    bl_label = MENU_LABEL

    def draw(self, context):
        layout = self.layout
        layout.menu(MENU_CAMERA_ID, **icons.kwargs("camera", "Camera"))
        layout.separator()
        icons.operator(layout, "opencv_cam.add_camera_scene", "Camera Scene", name="camera_scene")


def _menu_add(self, context):
    """Draw the VisionSim submenu inside Add."""
    self.layout.menu(MENU_ID, **icons.kwargs("visionsim", MENU_LABEL))


_CLASSES = (OPENCV_CAM_MT_camera, OPENCV_CAM_MT_vision_sim)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_add.append(_menu_add)


def unregister() -> None:
    bpy.types.VIEW3D_MT_add.remove(_menu_add)
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)