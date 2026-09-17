"""AVM Scene: a plane-scene layout for around-view-monitor evaluation.

One ground plane, a cube car, four identical solid black calibration blocks and
four OpenCV fisheye cameras, all placed with the real mount poses.  The scene
**does not solve poses**: its contract is "place the components + set the camera
intrinsics in, per-camera preview images + an export of every element's size and
pose out" (``docs/avm-scene.md`` §1.1).

Layout
------
``properties.py``  Scene level settings (field, car, ground, blocks, camera poses)
``controller.py``  debounced rebuild, preset loading, active camera
``builder.py``     creates / updates the objects, materials and cameras
``operators.py``   add / rebuild / reset / remove
``io.py``          parameter import/export (P4)
``coverage.py``    footprints, visibility matrix, material export (P5)
``ui.py``          Scene Properties panel + 3D viewport N panel (P3)
"""

from __future__ import annotations

from ..base import SceneDefinition

DEFINITION = SceneDefinition(
    id="avm_scene",
    label="AVM Scene",
    icon="avm_scene",
    order=20,
    add_operator="opencv_cam.avm_add_scene",
    root_name="AVM_Root",
    collection_name="AVM Scene",
)

from . import builder, controller, operators, properties  # noqa: E402

__all__ = ["DEFINITION", "builder", "controller", "operators", "properties"]


def register() -> None:
    properties.register()
    operators.register()


def unregister() -> None:
    operators.unregister()
    properties.unregister()
