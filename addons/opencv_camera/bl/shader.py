"""OSL camera shader handling.

Responsibilities:

* keep Text data-blocks in sync with the bundled ``.osl`` files (one per model);
* attach the one matching the selected distortion model to a camera
  (Lens Type = Custom / Internal);
* make sure the shader got compiled, because **Cycles silently keeps the old
  bytecode when a recompile fails** and renders with a stale shader otherwise.
"""

from __future__ import annotations

import importlib
import sys
from typing import Callable, Dict, List, Optional, Tuple

import bpy

from ..core import camera_model, paths

#: distortion model -> bundled shader file
MODEL_SHADERS: Dict[str, str] = {
    camera_model.MODEL_BROWN_CONRADY: "opencv_camera.osl",
    camera_model.MODEL_RATIONAL: "opencv_camera.osl",
    camera_model.MODEL_FISHEYE: "opencv_fisheye.osl",
}


def shader_filename(model: str) -> str:
    """Bundled shader that implements ``model``."""
    return MODEL_SHADERS.get(model, MODEL_SHADERS[camera_model.MODEL_BROWN_CONRADY])


def shader_source(filename: str) -> str:
    """Read a bundled shader from disk (kept as the authoritative copy)."""
    return paths.read_text(paths.shader_file(filename))


def installed_text(model: str) -> Optional[bpy.types.Text]:
    """Text data-block for a model, if it was installed before."""
    return bpy.data.texts.get(shader_filename(model))


def install_shader(settings, force: bool = False) -> Tuple[bpy.types.Text, bool]:
    """Create or refresh the Text data-block for the selected distortion model.

    Returns ``(text, changed)``.  The text is pinned with a fake user so it is
    saved with the ``.blend`` (the compiled bytecode lives on the camera).
    """
    filename = shader_filename(settings.distortion.model)
    source = shader_source(filename)
    text = bpy.data.texts.get(filename)
    changed = False
    if text is None:
        text = bpy.data.texts.new(filename)
        text.write(source)
        changed = True
    elif force or text.as_string() != source:
        text.clear()
        text.write(source)
        changed = True
    text.use_fake_user = True
    return text, changed


def attach(cam_data, settings) -> bpy.types.Text:
    """Point ``cam_data`` at the shader matching the selected model."""
    text, _ = install_shader(settings)
    cam_data.type = "CUSTOM"
    if cam_data.custom_mode != "INTERNAL":
        cam_data.custom_mode = "INTERNAL"
    if cam_data.custom_shader != text:
        cam_data.custom_shader = text
    return text


def is_compiled(cam_data) -> bool:
    """True when the camera carries OSL bytecode (i.e. a usable shader)."""
    return bool(getattr(cam_data, "custom_bytecode", ""))


def cycles_custom_params(cam_data):
    """The Cycles custom-camera parameter group, or ``None`` without Cycles."""
    return getattr(cam_data, "cycles_custom", None)


def report_callback(messages: List[str]) -> Callable:
    """Adapter matching the ``report(flags, message)`` signature Cycles expects."""

    def _report(flags, message):
        messages.append(f"{sorted(flags)}: {message}")

    return _report


def cycles_addon_module():
    """Best-effort lookup of the Cycles add-on module (``None`` when disabled)."""
    module = sys.modules.get("cycles")
    if module is not None:
        return module
    for addon in bpy.context.preferences.addons:
        name = addon.module if isinstance(addon.module, str) else addon.module.__name__
        if name.split(".")[-1] == "cycles":
            try:
                return importlib.import_module(name)
            except ImportError:
                continue
    return None


def force_compile(cam_data) -> List[str]:
    """Ask the Cycles add-on to (re)compile the attached shader.

    ``Camera.custom_shader`` is only compiled by an RNA update callback that
    Cycles provides; that callback does not fire in every context, so the
    add-on drives it explicitly as a fallback.
    """
    messages: List[str] = []
    module = cycles_addon_module()
    if module is None or not hasattr(module, "osl"):
        messages.append("Cycles add-on unavailable: cannot compile the OSL camera shader")
        return messages
    try:
        module.osl.update_custom_camera_shader(cam_data, report_callback(messages))
    except Exception as exc:  # defensive: never break an operator on a Cycles change
        messages.append(f"{type(exc).__name__}: {exc}")
    return messages


def ensure_compiled(cam_data) -> Tuple[bool, List[str]]:
    """Make sure compiled bytecode is present. Returns ``(ok, messages)``."""
    if is_compiled(cam_data):
        return True, []
    messages = force_compile(cam_data)
    if is_compiled(cam_data):
        return True, messages
    messages.append(
        "OSL camera shader was not compiled - check the Blender console for oslc "
        "errors (previous bytecode, if any, is kept silently)"
    )
    return False, messages