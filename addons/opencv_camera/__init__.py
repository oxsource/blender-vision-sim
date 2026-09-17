"""OpenCV Camera - render with OpenCV intrinsics and distortion coefficients.

A Cycles "Lens Type = Custom" camera backed by an OSL shader that implements the
OpenCV pinhole model with fisheye (Kannala-Brandt) / Brown-Conrady / rational
distortion, plus the plumbing to import calibration files, keep Blender's render
size in sync with the camera's real output size and verify the result against the
OpenCV model.

Module layout
-------------
``core``  pure Python camera model, transforms, presets, calibration IO (no ``bpy``)
``bl``    Blender integration: properties, shader handling, camera factory,
          preview, operators, menus, UI
``shaders``  the bundled ``.osl`` camera shaders (authoritative copies)

Only relative imports are used: an add-on is imported as ``bl_ext.<repo>.<id>``
when installed as an extension and as its plain package name as a legacy add-on.
"""

from .bl import menus, operators, panels_patch, properties, ui

__all__ = ["menus", "operators", "panels_patch", "properties", "ui"]


def register():
    properties.register()
    menus.register()
    operators.register()
    ui.register()
    panels_patch.register()


def unregister():
    panels_patch.unregister()
    ui.unregister()
    operators.unregister()
    menus.unregister()
    properties.unregister()