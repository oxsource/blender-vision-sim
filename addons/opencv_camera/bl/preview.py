"""Parameter preview.

Blender 4.x has no in-panel image widget for add-ons (``template_preview`` and
``Image.preview`` were removed), so the preview is a *real render* at reduced
settings that is displayed in Blender's Image Editor (``render.view_show``), i.e.
the same place F12 puts its result - just fast and without touching the scene.

The render settings are saved and restored, so a preview never changes the user's
setup.
"""

from __future__ import annotations

import os
import tempfile
import time
from typing import Dict, Optional

import bpy

from . import apply as apply_mod

#: debounce state for "Preview On Change" (module level, one request at a time)
_PENDING: Dict[str, object] = {"time": 0.0, "camera": None, "settings": None, "scene": None}
_DEBOUNCE_SECONDS = 0.6

#: long side of the preview render per UI option
PREVIEW_SIZES = {
    "256": 256,
    "384": 384,
    "512": 512,
    "720": 720,
}


def preview_resolution(settings, size_key: str = "384") -> tuple:
    """Aspect ratio of the output image, long side from the UI option."""
    long_side = PREVIEW_SIZES.get(str(size_key), 384)
    try:  # what the final render will produce
        width, height = apply_mod.output_resolution(settings)
    except Exception:
        width = height = 0
    if width <= 0 or height <= 0:
        intrinsics = settings.intrinsics
        width, height = int(intrinsics.image_width), int(intrinsics.image_height)
    if width <= 0 or height <= 0:
        return long_side, long_side
    if width >= height:
        return long_side, max(8, int(round(long_side * height / width)))
    return max(8, int(round(long_side * width / height))), long_side


def render_preview(
    cam_data,
    settings,
    scene: Optional[bpy.types.Scene] = None,
    size_key: str = "384",
    samples: int = 16,
    denoise: bool = True,
    save_to: str = "",
    show: bool = True,
) -> Dict:
    """Render a preview with the current camera and show it in the Image Editor.

    Returns a small result dict (``ok``, ``resolution``, ``samples``, messages).
    """
    scene = scene or bpy.context.scene
    messages = []
    if scene.render.engine != "CYCLES":
        scene.render.engine = "CYCLES"
        messages.append("switched the render engine to Cycles (custom cameras need it)")

    width, height = preview_resolution(settings, size_key)
    render = scene.render
    saved = {
        "resolution_x": render.resolution_x,
        "resolution_y": render.resolution_y,
        "resolution_percentage": render.resolution_percentage,
        "filepath": render.filepath,
        "file_format": render.image_settings.file_format,
        "samples": scene.cycles.samples,
        "preview_samples": scene.cycles.preview_samples,
        "denoising": scene.cycles.use_denoising,
        "film_transparent": render.film_transparent,
    }
    ok = True
    try:
        # the render resolution has to be set *before* the intrinsics are pushed:
        # they are scaled to the render resolution, so applying them afterwards
        # would use the previous resolution and change the field of view
        render.resolution_x = width
        render.resolution_y = height
        render.resolution_percentage = 100
        render.image_settings.file_format = "PNG"
        scene.cycles.samples = int(samples)
        scene.cycles.use_denoising = bool(denoise)
        ok, apply_messages = apply_mod.apply_settings(cam_data, settings, scene)
        messages.extend(apply_messages)
        if ok:
            if save_to:
                render.filepath = save_to
            bpy.ops.render.render(write_still=bool(save_to))
            if show and not bpy.app.background:
                try:
                    bpy.ops.render.view_show()
                except RuntimeError as exc:  # no image editor available
                    messages.append(f"could not open the render view: {exc}")
    finally:
        render.resolution_x = saved["resolution_x"]
        render.resolution_y = saved["resolution_y"]
        render.resolution_percentage = saved["resolution_percentage"]
        render.filepath = saved["filepath"]
        render.image_settings.file_format = saved["file_format"]
        scene.cycles.samples = saved["samples"]
        scene.cycles.preview_samples = saved["preview_samples"]
        scene.cycles.use_denoising = saved["denoising"]
        render.film_transparent = saved["film_transparent"]
        try:  # push the intrinsics back for the restored resolution
            apply_mod.apply_settings(cam_data, settings, scene)
        except Exception:
            pass

    return {
        "ok": bool(ok),
        "resolution": (width, height) if ok else (0, 0),
        "samples": int(samples) if ok else 0,
        "messages": messages,
    }


def schedule_preview(cam_data, settings, scene=None, size_key: str = "", samples: int = 0) -> None:
    """Ask for a preview shortly after the last parameter change (debounced).

    Called from property update callbacks, hence the debounce: dragging a slider
    would otherwise start a render per pixel of mouse movement.  Only runs in a UI
    session (timers do not fire in background mode).
    """
    if bpy.app.background:
        return
    _PENDING["time"] = time.time()
    _PENDING["camera"] = cam_data
    _PENDING["settings"] = settings
    _PENDING["scene"] = scene
    if not _PENDING.get("registered"):
        _PENDING["registered"] = True
        bpy.app.timers.register(_preview_timer, first_interval=0.25)


def cancel_preview() -> None:
    _PENDING["time"] = 0.0
    _PENDING["camera"] = None
    _PENDING["settings"] = None
    _PENDING["scene"] = None


def _preview_timer():
    """Timer callback: render once the user stopped changing parameters."""
    if _PENDING["camera"] is None:
        _PENDING["registered"] = False
        return None
    if time.time() - float(_PENDING["time"]) < _DEBOUNCE_SECONDS:
        return 0.25  # still editing, check again shortly
    cam_data = _PENDING["camera"]
    settings = _PENDING["settings"]
    scene = _PENDING["scene"] or bpy.context.scene
    cancel_preview()
    try:
        if cam_data.name in bpy.data.cameras:
            render_preview(cam_data, settings, scene,
                           size_key=settings.preview.size,
                           samples=settings.preview.samples,
                           denoise=settings.preview.denoise,
                           show=False)
    except Exception:
        pass
    _PENDING["registered"] = False
    return None


def save_preview_image(cam_data, settings, scene=None, size_key: str = "720",
                       samples: int = 32, filepath: str = "") -> str:
    """Render a preview to a file (used by the *Save Preview* operator)."""
    if not filepath:
        filepath = os.path.join(tempfile.mkdtemp(prefix="opencv_cam_preview_"), "preview.png")
    result = render_preview(cam_data, settings, scene, size_key=size_key,
                            samples=samples, save_to=filepath, show=False)
    if not result["ok"]:
        raise RuntimeError("; ".join(result["messages"]))
    return filepath