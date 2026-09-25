"""Road Scene: a closed test loop, recorded frame by frame.

A self-contained algorithm scene (see ``docs/road-scene.md``): a closed road with
straights, constant-radius curves and up/down ramps, roadside props (parked cars,
pedestrians, trees, lamps, signs), and a minibus carrying the add-on's four
OpenCV cameras.  Recording writes one PNG per frame plus ``frames.csv`` (per-frame
speed, the **3D** vehicle pose and each camera's world pose, with the road-type
label) and ``clip.json`` - the material the transparent-chassis "road type x
speed x direction" matrix is measured on (``filament_avm`` M5 / CH-018).

Layout
------
``properties.py``  Scene level settings (track, props, lighting, vehicle, drive)
``controller.py``  debounced rebuild
``builder.py``     creates / updates the track, props, vehicle, cameras, keyframes
``recording.py``   the Road Scene's profile of the shared clip recorder
``operators.py``   add / rebuild / reset / remove / export clip
``ui.py``          Scene Properties panel + 3D viewport N panel
"""

from __future__ import annotations

from ..base import SceneDefinition
from ..view import ViewSpec
from . import builder

DEFINITION = SceneDefinition(
    id="road_scene",
    label="Road Scene",
    icon="road_scene",
    order=40,
    add_operator="opencv_cam.road_add_scene",
    root_name=builder.ROOT_NAME,
    collection_name=builder.COLLECTION_NAME,
    # the default view frames the car, not the 250 m loop
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
