"""The ego vehicle's geometry - the one place its numbers live.

Pure Python (no ``bpy``).  The AVM Scene's car mesh, the Drive Scene's car mesh,
the ``vehicle`` block of ``clip.json`` and the wheel geometry the Falcon
steering-line config ships all read this module, so "the car that was rendered"
and "the car that was described" cannot disagree
(``docs/drive-scene-multicam.md`` section 5.6.3).

Section 5.6.1 found that the numbers already agreed with each other
(``avm_falcon.STEERING["body_width"] == 2.4 == car_width``) - what was missing
was a single source *and* an export.  This module is the source; :func:`block`
is the export.

All values are metres in the **vehicle frame**: origin at the vehicle centre on
the ground, ``+X`` right, ``+Y`` forward (the nose), ``+Z`` up - the same frame
the cameras are mounted in, which is why ``clip.json``'s camera mounts and its
vehicle block are directly comparable.

``body`` is the nominal body box the meshes are built from.  The generated cars
add two things proud of it - the wheels (outer faces 0.11 m beyond each side) and
the lamp boxes (0.02 m beyond each end) - so the object's ``dimensions`` is
slightly larger than :data:`BODY_LENGTH_M` / :data:`BODY_WIDTH_M`.  For an ego
occlusion mask the body box is the quantity that matters; that the mesh really
was built at this size is asserted against the mesh's own vertices in
``tests/run_blender_tests.py::test_drive_scene``.
"""

from __future__ import annotations

from typing import Dict, List, Optional

__all__ = [
    "FRAME", "BODY_LENGTH_M", "BODY_WIDTH_M", "BODY_HEIGHT_M", "GROUND_CLEARANCE_M",
    "WHEEL_BASE_M", "REAR_TRACK_M", "REAR_CENTER_OFFSET_M", "AXLE_FRACTION",
    "body", "axles", "center_to_rear_axle", "block",
]

#: the frame every number here is expressed in
FRAME = "vehicle"

#: the nominal body box [m]
BODY_LENGTH_M = 4.8
BODY_WIDTH_M = 2.4
BODY_HEIGHT_M = 2.88

#: how far the body sits above the ground [m]
GROUND_CLEARANCE_M = 0.0

#: the axle geometry [m] - the anchors a 3D vehicle projection needs.  The
#: defaults are the app's ``SteeringPerfs`` (``avm_falcon.STEERING``), which now
#: reads them from here instead of carrying its own copy.
WHEEL_BASE_M = 3.2
REAR_TRACK_M = 1.8
REAR_CENTER_OFFSET_M = 2.8

#: where the generated car meshes actually put the wheels: each axle sits
#: ``AXLE_FRACTION`` of the body length ahead of / behind the centre.  The meshes
#: in ``bl/scenes/*/builder.py`` read this, so the rendered axle and the exported
#: :func:`center_to_rear_axle` cannot drift apart.  Note this is *not* the
#: steering ``axles`` block above: ``SteeringPerfs`` is a projection anchor, while
#: this is the physical wheel position in the vehicle frame.
AXLE_FRACTION = 0.31


def body(length: Optional[float] = None, width: Optional[float] = None,
         height: Optional[float] = None) -> Dict[str, float]:
    """The body box as ``{"length", "width", "height"}`` (metres)."""
    return {
        "length": _m(BODY_LENGTH_M if length is None else length),
        "width": _m(BODY_WIDTH_M if width is None else width),
        "height": _m(BODY_HEIGHT_M if height is None else height),
    }


def axles(wheel_base: Optional[float] = None, rear_track: Optional[float] = None,
          rear_center_offset: Optional[float] = None) -> Dict[str, float]:
    """The axle geometry as a ``clip.json``-shaped dict (metres)."""
    return {
        "wheel_base": _m(WHEEL_BASE_M if wheel_base is None else wheel_base),
        "rear_track": _m(REAR_TRACK_M if rear_track is None else rear_track),
        "rear_center_offset": _m(
            REAR_CENTER_OFFSET_M if rear_center_offset is None else rear_center_offset),
    }


def center_to_rear_axle(length: Optional[float] = None) -> List[float]:
    """The rear-axle centre in the vehicle frame, ``[x, y, z]`` metres.

    The vehicle frame origin is the body centre on the ground (see the module
    docstring), so the rear axle is :data:`AXLE_FRACTION` of the body length
    behind it: ``y = -length * AXLE_FRACTION``.  A transparent-chassis consumer
    anchored on the rear axle needs this explicitly; it must not be inferred
    from the steering ``axles`` block, whose reference point is a different one.
    """
    body_length = BODY_LENGTH_M if length is None else length
    return [0.0, _m(-body_length * AXLE_FRACTION), 0.0]


def block(length: Optional[float] = None, width: Optional[float] = None,
          height: Optional[float] = None,
          clearance: Optional[float] = None) -> Dict:
    """The ``vehicle`` block of a clip's ``clip.json`` (section 5.6.2).

    ``frame`` is written out explicitly: a geometry block whose frame is only
    implied is the same class of hazard as a pose whose frame is only implied
    (section 4.7).  The block describes the vehicle, so there is exactly **one**
    of it per clip - it does not repeat per camera.

    ``center_to_rear_axle`` is the vehicle-centre -> rear-axle transform a
    rear-axle-anchored consumer (the transparent chassis) needs; it is exported
    rather than left implicit so a missing calibration cannot silently become
    zero.  Every length in the block is in metres, so the keys carry no unit
    suffix; the frame is named once.
    """
    return {
        "frame": FRAME,
        "body": body(length, width, height),
        "ground_clearance": _m(GROUND_CLEARANCE_M if clearance is None else clearance),
        "center_to_rear_axle": center_to_rear_axle(length),
        "axles": axles(),
    }


def _m(value: float) -> float:
    """A metres value at a readable precision.

    The callers hand over Blender ``FloatProperty`` values, which are 32 bit:
    4.8 arrives as 4.800000190734863.  Six decimals keeps a micron and makes the
    file say what a person wrote.
    """
    return round(float(value), 6)
