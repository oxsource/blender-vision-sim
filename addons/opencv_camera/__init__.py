"""OpenCV Camera - render with OpenCV intrinsics and distortion coefficients.

A Cycles "Lens Type = Custom" camera backed by an OSL shader that implements the
OpenCV pinhole model with Brown-Conrady / rational distortion, plus the plumbing
to import calibration files and to verify the result against the OpenCV model.

Module layout
-------------
``core``  pure Python camera model, transforms and calibration IO (no ``bpy``)
``bl``    Blender integration: properties, shader handling, operators, UI
``shaders``  the bundled ``.osl`` camera shader (authoritative copy)

Only relative imports are used: an add-on is imported as ``bl_ext.<repo>.<id>``
when installed as an extension and as its plain package name as a legacy add-on.
"""

from .bl import operators, properties, ui

__all__ = ["operators", "properties", "ui"]


def register():
    properties.register()
    operators.register()
    ui.register()


def unregister():
    ui.unregister()
    operators.unregister()
    properties.unregister()