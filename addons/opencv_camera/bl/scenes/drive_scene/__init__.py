"""Drive Scene: an indoor car-park drive, recorded frame by frame.

A self-contained algorithm scene (see ``docs/drive-scene.md``): a parking aisle
with bay markings, round columns and a concrete floor; a minibus with the
add-on's **OpenCV front camera** at the real mount pose; and a planned drive
(distance + speed profile) that is keyframed on ``DRIVE_Vehicle``.  Recording
writes one PNG per frame plus ``frames.csv`` (per-frame speed and pose) and
``clip.json`` - the material a transparent-chassis algorithm is developed on.

Layout
------
``properties.py``  Scene level settings (car park, vehicle, drive)
``controller.py``  debounced rebuild
``builder.py``     creates / updates the lot, the vehicle, the camera, the keyframes
``recording.py``   renders the clip: PNG sequence + frames.csv + clip.json
``operators.py``   add / rebuild / reset / remove / render clip
``ui.py``          Scene Properties panel + 3D viewport N panel
"""

from __future__ import annotations

from ..base import SceneDefinition
from ..view import ViewSpec
from . import builder

DEFINITION = SceneDefinition(
    id="drive_scene",
    label="Drive Scene",
    icon="drive_scene",
    order=30,
    add_operator="opencv_cam.drive_add_scene",
    root_name=builder.ROOT_NAME,
    collection_name=builder.COLLECTION_NAME,
    # the nose is +Y and the car drives up the aisle: front-right-above shows the
    # car, the markings under it and the bay rows on both sides
    view=ViewSpec(azimuth=135.0, elevation=30.0),
    view_targets=builder.view_targets,
)

from . import (controller, operators, properties, recording, ui)  # noqa: E402,F401

__all__ = ["DEFINITION", "builder", "controller", "operators", "properties",
           "recording", "ui"]


def register() -> None:
    properties.register()
    operators.register()
    ui.register()


def unregister() -> None:
    ui.unregister()
    operators.unregister()
    properties.unregister()
