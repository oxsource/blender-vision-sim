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

import copy
from typing import Dict, List, Optional

__all__ = [
    "FRAME", "BODY_LENGTH_M", "BODY_WIDTH_M", "BODY_HEIGHT_M", "GROUND_CLEARANCE_M",
    "WHEEL_BASE_M", "REAR_TRACK_M", "REAR_CENTER_OFFSET_M", "AXLE_FRACTION",
    "WHEEL_RADIUS_M", "DOOR_SLOTS", "DOOR_SLOTS_3ROW", "ABSENT_OK",
    "ABSENT_OK_3ROW", "STYLE_2ROW", "STYLE_3ROW", "APPEARANCE",
    "body", "axles", "center_to_rear_axle", "block",
    "wheel_radius", "parts", "manifest", "to_blender",
]

#: the frame every number here is expressed in
FRAME = "vehicle"

#: the nominal body box [m] - the Skyworth "Metro Elf" (NJL6541EVF):
#: 5445 x 2085 x 2655 mm, low floor, 3680 mm wheelbase.
BODY_LENGTH_M = 5.445
BODY_WIDTH_M = 2.085
BODY_HEIGHT_M = 2.655

#: how far the body sits above the ground [m]
GROUND_CLEARANCE_M = 0.0

#: the axle geometry [m] - the anchors a 3D vehicle projection needs.  The
#: defaults are the app's ``SteeringPerfs`` (``avm_falcon.STEERING``), which now
#: reads them from here instead of carrying its own copy.
WHEEL_BASE_M = 3.68
REAR_TRACK_M = 1.56
REAR_CENTER_OFFSET_M = 3.12

#: tyre size 215/75R17.5 -> ~0.38 m radius; the M2 contract keeps 0.36.
FRONT_OVERHANG_M = 0.995
REAR_OVERHANG_M = 0.770

#: where the generated car meshes actually put the wheels, as a fraction of the
#: body length from the centre.  The bus axles are asymmetric (front overhang
#: 0.995 m, rear 0.770 m), unlike the old minibus.  The meshes in
#: ``bl/scenes/*/`` read :func:`axle_offsets`, so the rendered axle and the
#: exported :func:`center_to_rear_axle` cannot drift apart.  Note this is *not*
#: the steering ``axles`` block above: ``SteeringPerfs`` is a projection anchor,
#: while this is the physical wheel position in the vehicle frame.
FRONT_AXLE_FRACTION = (BODY_LENGTH_M / 2.0 - FRONT_OVERHANG_M) / BODY_LENGTH_M
REAR_AXLE_FRACTION = (BODY_LENGTH_M / 2.0 - REAR_OVERHANG_M) / BODY_LENGTH_M
AXLE_FRACTION = REAR_AXLE_FRACTION

#: the default wheel radius [m]; the M2 contract fixes it at 0.36 for the
#: 2.88 m tall minibus (a wheel outside 0.30..0.40 would break the style table).
WHEEL_RADIUS_M = 0.36

#: door slot -> part id (slot 7 reserved); ``None`` means the slot is invalid.
DOOR_SLOTS = ["door_r1l", "door_r1r", "door_r2l", "door_r2r",
              None, None, "door_tail", None]
DOOR_SLOTS_3ROW = ["door_r1l", "door_r1r", "door_r2l", "door_r2r",
                   "door_r3l", "door_r3r", "door_tail", None]

#: parts a style may legitimately omit (N3: whole-part no-op).
ABSENT_OK = ["door_r3l", "door_r3r"]
ABSENT_OK_3ROW: List[str] = []

#: the 3-row style adds the third pair of side doors; everything else follows
#: the same part cards / channels.
STYLE_2ROW = "minibus_2row"
STYLE_3ROW = "minibus_3row"

#: The default appearance set (M2).  ``materials`` use the slot names in
#: ``part_contract.json``; ``glass`` is a fixed material and never varies.
#: There is deliberately no opacity: the app sets that through a future C API.
APPEARANCE: Dict = {
    "default": "sky_blue",
    "fallback": {"unknown_variant": "sky_blue", "missing_slot": "keep_glb"},
    "fixed": {"glass": {"base_color": [0.025, 0.035, 0.045, 1.0], "roughness": 0.15}},
    "variants": [
        {
            # The reference bus: light cyan body (linear ~#67D2EA after sRGB).
            "id": "sky_blue",
            "materials": {
                "paint_body": {"base_color": [0.13, 0.62, 0.82, 1.0], "roughness": 0.35},
                "paint_door": {"base_color": [0.13, 0.62, 0.82, 1.0], "roughness": 0.35},
                "trim_door_inner": {"base_color": [0.52, 0.53, 0.55, 1.0], "roughness": 0.70},
                "paint_mirror": {"base_color": [0.10, 0.50, 0.68, 1.0], "roughness": 0.35},
                "tire": {"base_color": [0.03, 0.03, 0.03, 1.0], "roughness": 0.85},
                "rim": {"base_color": [0.55, 0.57, 0.60, 1.0], "roughness": 0.35},
                "lamp_lens_front": {"base_color": [0.95, 0.95, 0.90, 1.0], "roughness": 0.20},
                "lamp_lens_rear": {"base_color": [0.55, 0.05, 0.05, 1.0], "roughness": 0.30},
            },
        },
        {
            # The ride-hailing orange: linear RGB ~ #FF7000 after sRGB encode.
            "id": "taxi_orange",
            "materials": {
                "paint_body": {"base_color": [1.00, 0.16, 0.00, 1.0], "roughness": 0.45},
                "paint_door": {"base_color": [1.00, 0.16, 0.00, 1.0], "roughness": 0.45},
                "paint_mirror": {"base_color": [0.86, 0.12, 0.00, 1.0], "roughness": 0.45},
            },
        },
        {
            "id": "exec_black",
            "materials": {
                "paint_body": {"base_color": [0.05, 0.05, 0.06, 1.0], "roughness": 0.35},
                "paint_door": {"base_color": [0.05, 0.05, 0.06, 1.0], "roughness": 0.35},
                "paint_mirror": {"base_color": [0.08, 0.08, 0.09, 1.0], "roughness": 0.35},
            },
        },
    ],
}


def wheel_radius(height: Optional[float] = None) -> float:
    """The style's wheel radius, clamped to the M1 range (0.30..0.40 m)."""
    body_height = BODY_HEIGHT_M if height is None else float(height)
    return round(min(0.40, max(0.30, body_height * (WHEEL_RADIUS_M / BODY_HEIGHT_M))), 4)


def _scaled_pivot(pos, sx: float, sy: float, sz: float) -> List[float]:
    """Scale a glTF-frame pivot with the body box (x width, y height, z length)."""
    return [round(pos[0] * sx, 4), round(pos[1] * sy, 4), round(pos[2] * sz, 4)]


def parts(length: Optional[float] = None, width: Optional[float] = None,
          height: Optional[float] = None,
          style: str = STYLE_2ROW) -> List[Dict]:
    """The M2 part cards for the default 2-row minibus.

    Pivot positions are in the glTF vehicle frame (+X right, +Y up, nose -Z).
    They scale with the body box so a custom AVM car stays consistent; the
    default 4.8 x 2.4 x 2.88 m box reproduces the frozen M2 numbers exactly.
    """
    body_length = BODY_LENGTH_M if length is None else float(length)
    body_width = BODY_WIDTH_M if width is None else float(width)
    body_height = BODY_HEIGHT_M if height is None else float(height)
    sx = body_width / BODY_WIDTH_M
    sy = body_height / BODY_HEIGHT_M
    sz = body_length / BODY_LENGTH_M
    radius = wheel_radius(body_height)

    def card(part_id, node, pivot, parent, slots, optional, state, motion):
        return {
            "id": part_id,
            "node": node,
            "pivot": pivot,
            "parent": parent,
            "slots": list(slots),
            "optional": optional,
            "state": state,
            "motion": None if motion is None else copy.deepcopy(motion),
        }

    def rotating(part_id, pivot_name, slots, channel, axis, travel, direction,
                 optional=False, state=None):
        motion = {"channel": channel, "form": "rotate", "axis": list(axis),
                  "pivot_pos": None, "travel": travel, "unit": "deg",
                  "direction": direction}
        return card(part_id, f"part_{part_id}", pivot_name, pivot_name, slots,
                    optional, state, motion)

    def sliding(part_id, pivot_name, channel, travel, direction):
        motion = {"channel": channel, "form": "translate", "axis": [0.0, 0.0, 1.0],
                  "pivot_pos": None, "travel": travel, "unit": "m",
                  "direction": direction}
        return card(part_id, f"part_{part_id}", pivot_name, pivot_name,
                    ["paint_door", "trim_door_inner"], False, None, motion)

    #: base pivots for the 5.445 x 2.085 x 2.655 m Metro Elf box.  The door
    #: hinges sit on the front edge of their span (glTF z = -y_forward), so the
    #: hinge above a swinging front door and the handle edge of a sliding door
    #: both stay on the door's leading edge.
    half_width = BODY_WIDTH_M / 2.0
    front_axle, rear_axle = axle_offsets()
    spans = door_spans()
    base = {
        "hood": [0.0, 0.84, -2.66],
        "wheel_fl": [-half_width + 0.02, WHEEL_RADIUS_M, -front_axle],
        "wheel_fr": [half_width - 0.02, WHEEL_RADIUS_M, -front_axle],
        "wheel_rl": [-half_width + 0.02, WHEEL_RADIUS_M, rear_axle],
        "wheel_rr": [half_width - 0.02, WHEEL_RADIUS_M, rear_axle],
        "mirror_l": [-half_width, 1.98, -1.70],
        "mirror_r": [half_width, 1.98, -1.70],
        "door_r1l": [-half_width, 0.30, -spans["r1"][1]],
        "door_r1r": [half_width, 0.30, -spans["r1"][1]],
        "door_r2l": [-half_width, 0.30, -spans["r2"][1]],
        "door_r2r": [half_width, 0.30, -spans["r2"][1]],
        "door_r3l": [-half_width, 0.30, -spans["r3"][1]],
        "door_r3r": [half_width, 0.30, -spans["r3"][1]],
        "door_tail": [0.0, 2.35, 2.56],
    }

    result = [
        card("body", "part_body", None, FRAME, ["paint_body"], False, None, None),
        card("interior", "part_interior", None, FRAME,
             ["trim_door_inner"], True, None, None),
        rotating("hood", "pivot_hood", ["paint_body"], "hood_ratio",
                 (1.0, 0.0, 0.0), 45.0, -1),
        card("lamp_head_l", "part_lamp_head_l", None, FRAME,
             ["lamp_lens_front"], False, "lamp_state", None),
        card("lamp_head_r", "part_lamp_head_r", None, FRAME,
             ["lamp_lens_front"], False, "lamp_state", None),
        card("lamp_tail_l", "part_lamp_tail_l", None, "pivot_door_tail",
             ["lamp_lens_rear"], False, "lamp_state", None),
        card("lamp_tail_r", "part_lamp_tail_r", None, "pivot_door_tail",
             ["lamp_lens_rear"], False, "lamp_state", None),
    ]
    for key, side in (("fl", "l"), ("fr", "r"), ("rl", "l"), ("rr", "r")):
        result.append(rotating(f"wheel_{key}", f"pivot_wheel_{key}",
                               ["tire", "rim"], "wheel_spin",
                               (1.0, 0.0, 0.0), 360.0, -1))
    result.extend([
        rotating("mirror_l", "pivot_mirror_l", ["paint_mirror"], "mirror_fold",
                 (0.0, 1.0, 0.0), 90.0, 1, optional=True),
        rotating("mirror_r", "pivot_mirror_r", ["paint_mirror"], "mirror_fold",
                 (0.0, 1.0, 0.0), 90.0, -1, optional=True),
        # The Metro Elf's passenger doors are sliders, not swing doors: both
        # leaves translate rearward outside the body skin (a plug door pops out
        # and runs back).  Travel is the leaf width, so the opening is clear.
        sliding("door_r1l", "pivot_door_r1l", "slide_offset", 0.95, 1),
        sliding("door_r1r", "pivot_door_r1r", "slide_offset", 0.95, 1),
        sliding("door_r2l", "pivot_door_r2l", "slide_offset", 0.95, 1),
        sliding("door_r2r", "pivot_door_r2r", "slide_offset", 0.95, 1),
    ])
    if style == STYLE_3ROW:
        result.extend([
            rotating("door_r3l", "pivot_door_r3l",
                     ["paint_door", "trim_door_inner"], "open_ratio",
                     (0.0, 1.0, 0.0), 70.0, -1),
            rotating("door_r3r", "pivot_door_r3r",
                     ["paint_door", "trim_door_inner"], "open_ratio",
                     (0.0, 1.0, 0.0), 70.0, 1),
        ])
    result.append(rotating("door_tail", "pivot_door_tail",
                           ["paint_door", "trim_door_inner"], "tailgate_ratio",
                           (1.0, 0.0, 0.0), 75.0, -1))

    for item in result:
        motion = item["motion"]
        if motion is None:
            continue
        pivot = base[item["id"]]
        if item["id"].startswith("wheel_"):
            scaled = [pivot[0] * sx, radius, pivot[2] * sz]
        else:
            scaled = _scaled_pivot(pivot, sx, sy, sz)
        motion["pivot_pos"] = [round(value, 4) for value in scaled]
        if motion["unit"] == "m":
            motion["travel"] = round(motion["travel"] * sz, 4)
    return result


def manifest(style_id: str = "minibus_2row", style_name: str = "2-row minibus",
             length: Optional[float] = None, width: Optional[float] = None,
             height: Optional[float] = None,
             clearance: Optional[float] = None) -> Dict:
    """The single ``vehicle_model.json`` object for the current style."""
    body_length = BODY_LENGTH_M if length is None else float(length)
    body_width = BODY_WIDTH_M if width is None else float(width)
    body_height = BODY_HEIGHT_M if height is None else float(height)
    three_row = style_id == STYLE_3ROW
    dimensions = body(body_length, body_width, body_height)
    dimensions["ground_clearance"] = _m(
        GROUND_CLEARANCE_M if clearance is None else clearance)
    # The anchors describe THIS model.  Every wheel pivot is scaled with the body
    # (see _scaled_pivot), so the axle geometry scales with it too; quoting the
    # reference car's absolute values on a shorter body would claim a 3.68 m
    # wheelbase under a 4.8 m box.  At the reference dimensions this is identity.
    anchors = axles(wheel_base=WHEEL_BASE_M * (body_length / BODY_LENGTH_M),
                    rear_track=REAR_TRACK_M * (body_width / BODY_WIDTH_M),
                    rear_center_offset=REAR_CENTER_OFFSET_M * (body_length / BODY_LENGTH_M))
    anchors["axle_fraction"] = AXLE_FRACTION
    anchors["wheel_radius"] = wheel_radius(body_height)
    return {
        "schema_version": 1,
        "model": {
            "style_id": style_id,
            "style_name": style_name,
            "frame": FRAME,
            "root": FRAME,
            "dimensions": dimensions,
            "anchors": anchors,
            "door_slots": list(DOOR_SLOTS_3ROW if three_row else DOOR_SLOTS),
            "absent_ok": list(ABSENT_OK_3ROW if three_row else ABSENT_OK),
        },
        "parts": parts(body_length, body_width, body_height,
                       STYLE_3ROW if three_row else STYLE_2ROW),
        "appearance": copy.deepcopy(APPEARANCE),
    }


def to_blender(position) -> List[float]:
    """glTF vehicle frame -> Blender frame (X right, Y forward, Z up)."""
    x, y, z = (float(value) for value in position)
    return [x, -z, y]



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


def axle_offsets(length: Optional[float] = None) -> Tuple[float, float]:
    """Physical front / rear axle offsets [m] from the body centre.

    Both are positive distances in the vehicle frame (+Y forward): the front
    axle is ahead of the centre, the rear axle behind it.  The bus axle layout
    is asymmetric (front overhang 0.995 m, rear 0.770 m).
    """
    body_length = BODY_LENGTH_M if length is None else float(length)
    return (body_length * FRONT_AXLE_FRACTION,
            body_length * REAR_AXLE_FRACTION)


def door_spans(length: Optional[float] = None) -> Dict[str, Tuple[float, float]]:
    """Side-door spans in the Blender frame (+Y forward), metres.

    r1 is the front door, r2 the sliding door; r3 only exists in the 3-row
    style and is kept inside the rear bay.  The mesh builder and the pivots
    both read this table, so a seam cannot drift away from its hinge.
    """
    body_length = BODY_LENGTH_M if length is None else float(length)
    scale = body_length / BODY_LENGTH_M
    return {
        "r1": (0.35 * scale, 1.35 * scale),
        "r2": (-1.25 * scale, -0.25 * scale),
        "r3": (-2.20 * scale, -1.30 * scale),
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
