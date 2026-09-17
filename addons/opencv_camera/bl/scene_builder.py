"""Deprecated shim: the camera scene moved to :mod:`bl.scenes.camera_scene`.

Kept for one release so existing scripts and tests importing
``opencv_camera.bl.scene_builder`` keep working; import from
``opencv_camera.bl.scenes.camera_scene`` instead.
"""

from __future__ import annotations

from .scenes.camera_scene import (  # noqa: F401
    COLLECTION_NAME,
    OPENCV_CAM_OT_add_camera_scene,
    apply_and_build,
    build,
    prepare_render,
)

__all__ = [
    "COLLECTION_NAME",
    "OPENCV_CAM_OT_add_camera_scene",
    "apply_and_build",
    "build",
    "prepare_render",
]
