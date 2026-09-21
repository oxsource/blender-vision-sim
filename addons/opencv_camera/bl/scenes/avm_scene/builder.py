"""AVM Scene builder: create and update the ground, car, blocks and cameras.

The objects are created once and then **updated in place** (idempotent rebuild),
so a slider drag never destroys the user's selection, materials or parenting.
Every mesh is generated at its real size (object scale stays 1), which keeps the
exported GLB/FBX and the coverage maths honest.
"""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional

import bmesh
import bpy
from mathutils import Matrix

from ....core import paths
from ....core.scenes import avm_cameras, avm_coverage, avm_layout
from ... import apply as apply_mod
from ... import camera_factory, compat, shader
from ..base import collection, link_to_collection, remove_collection_objects

ROOT_NAME = "AVM_Root"
COLLECTION_NAME = "AVM Scene"

GROUND_NAME = "AVM_Ground"
CAR_NAME = "AVM_Car"
SUN_NAME = "AVM_Sun"
#: the fill light shares the SUN_NAME prefix on purpose: the ``show_sun`` layer
#: toggle matches by prefix, so one flag hides both
FILL_NAME = "AVM_Sun_Fill"
BLOCK_PREFIX = "AVM_Block_"
CAMERA_PREFIX = "AVM_Cam_"
PROP_PREFIX = "AVM_Prop_"
LABEL_PREFIX = "AVM_Label_"

#: ground text: "front/back/left/right" markers plus a title decal
LABEL_TEXT = {"Front": "前", "Back": "后", "Left": "左", "Right": "右"}
LABEL_SIZE = 0.9
TITLE_SIZE = 0.7
LABEL_Z = 0.002  #: above the ground (-2 mm) and the blocks (+1 mm)

#: fonts that carry CJK glyphs, tried in order when ``label_font`` is empty
#: (Blender's built-in font has no CJK, so 前后左右 would come out blank)
CJK_FONT_CANDIDATES = (
    "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
    "/System/Library/Fonts/PingFang.ttc",
    "/System/Library/Fonts/STHeiti Light.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
)

#: block key -> object name suffix
BLOCK_SUFFIX = {
    avm_layout.BLOCK_FRONT_LEFT: "FrontLeft",
    avm_layout.BLOCK_FRONT_RIGHT: "FrontRight",
    avm_layout.BLOCK_BACK_LEFT: "BackLeft",
    avm_layout.BLOCK_BACK_RIGHT: "BackRight",
}
CAMERA_SUFFIX = avm_cameras.SUFFIX

#: The rendered ground sits this far below the nominal ``z = 0`` plane.  The
#: blocks (and every calibration quantity: ``points_3d``, the coverage maths)
#: stay at ``z ~ 0``, so the two never share a plane and cannot z-fight.
GROUND_DROP = 0.002

#: White frame drawn around each black calibration block [m].  The black face
#: keeps the nominal ``corner`` size (the ``points_3d`` contract depends on it);
#: the frame only extends outward, so the calibration corners never move.
BLOCK_BORDER = 0.2


# ---------------------------------------------------------------------------
# meshes (generated at real size; object scale stays 1)
# ---------------------------------------------------------------------------
def _quad_mesh(name: str, width: float, height: float) -> bpy.types.Mesh:
    mesh = bpy.data.meshes.new(name)
    hw, hh = width * 0.5, height * 0.5
    mesh.from_pydata([(-hw, -hh, 0.0), (hw, -hh, 0.0),
                      (hw, hh, 0.0), (-hw, hh, 0.0)], [], [(0, 1, 2, 3)])
    mesh.update()
    return mesh


def _block_mesh(name: str, inner_w: float, inner_h: float,
                border: float) -> bpy.types.Mesh:
    """A black calibration square with a white frame around it.

    The black face keeps the block's nominal size (the ``points_3d`` contract
    and the corner detector depend on it); the frame extends ``border`` metres
    outward on all four sides, so the calibration corners never move.  The two
    regions meet edge to edge and never overlap, so they cannot z-fight.
    """
    mesh = bpy.data.meshes.new(name)
    hw, hh = inner_w * 0.5, inner_h * 0.5
    ow, oh = hw + border, hh + border
    verts = [(-hw, -hh, 0.0), (hw, -hh, 0.0), (hw, hh, 0.0), (-hw, hh, 0.0),
             (-ow, -oh, 0.0), (ow, -oh, 0.0), (ow, oh, 0.0), (-ow, oh, 0.0)]
    faces = [(0, 1, 2, 3),
             (4, 5, 1, 0), (5, 6, 2, 1), (6, 7, 3, 2), (7, 4, 0, 3)]
    mesh.from_pydata(verts, [], faces)
    for polygon in mesh.polygons[1:]:
        polygon.material_index = 1
    mesh.update()
    return mesh


def _box_mesh(name: str, size_x: float, size_y: float, size_z: float,
              base_at_zero: bool = True) -> bpy.types.Mesh:
    """A box; with ``base_at_zero`` its origin sits on the bottom face."""
    mesh = bpy.data.meshes.new(name)
    hx, hy, hz = size_x * 0.5, size_y * 0.5, size_z * 0.5
    z0, z1 = (0.0, size_z) if base_at_zero else (-hz, hz)
    verts = [(-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
             (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1)]
    faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    return mesh


def _replace_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh) -> None:
    old = obj.data
    obj.data = mesh
    if old is not None and old.users == 0:
        bpy.data.meshes.remove(old)


# ---------------------------------------------------------------------------
# real ground mesh: the bowl the Falcon app projects the cameras onto
# ---------------------------------------------------------------------------
#: marker stored on the merged ground mesh: the model path plus the scale it was
#: built with, so a rebuild can tell a loaded model from the fallback quad and
#: skip re-importing the file on every slider drag
GROUND_MODEL_KEY = "avm_ground_model"

#: base (unscaled) geometry per model path, so changing the bowl size / rim
#: height rescales from the cache instead of re-importing the file
_MODEL_GEOMETRY: Dict[str, tuple] = {}


def ground_model_path(settings) -> str:
    """The ground model to load, or ``""`` for the flat plane.

    ``use_ground_model`` off means the flat plane; an empty ``ground_model``
    means the bowl bundled with the add-on (so a fresh scene works offline and
    survives a Blender restart).
    """
    if not settings.use_ground_model:
        return ""
    return (settings.ground_model or "").strip() or paths.ground_model_file()


def ground_model_key(settings) -> str:
    """Cache key of the merged mesh: the file plus the two scale settings."""
    path = ground_model_path(settings)
    if not path:
        return ""
    return (f"{path}\x00{settings.ground_radius:g}\x00{settings.ground_rim_height:g}")


def _collect_meshes(objects) -> Optional[tuple]:
    """Merge mesh objects into ``(verts, faces)`` with world transforms baked."""
    verts: List = []
    faces: List = []
    for obj in objects:
        if obj.type != "MESH" or obj.data is None:
            continue
        matrix = obj.matrix_world
        base = len(verts)
        verts.extend(tuple(matrix @ vertex.co) for vertex in obj.data.vertices)
        faces.extend(tuple(base + index for index in polygon.vertices)
                     for polygon in obj.data.polygons)
    if not faces:
        return None
    return verts, faces


def _model_geometry(path: str) -> Optional[tuple]:
    """Import ``path`` once and return its cached ``(verts, faces)``.

    The importer links its objects into the active collection; they are merged
    (world transform baked) and removed again, so nothing but the geometry is
    left behind.  The user's selection is restored: a rebuild must never move it.
    """
    cached = _MODEL_GEOMETRY.get(path)
    if cached is not None:
        return cached
    view_layer = bpy.context.view_layer
    selected = list(bpy.context.selected_objects)
    active = view_layer.objects.active
    before = {obj.name for obj in bpy.data.objects}
    imported: List = []
    geometry: Optional[tuple] = None
    try:
        compat.import_model(path)
        imported = [obj for obj in bpy.data.objects if obj.name not in before]
        geometry = _collect_meshes(imported)
    except Exception:
        geometry = None
    for obj in imported:
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if data is not None and data.users == 0:
            bpy.data.meshes.remove(data)
    try:
        for obj in selected:
            if obj.name in bpy.data.objects:
                obj.select_set(True)
        view_layer.objects.active = active
    except Exception:
        pass
    if geometry is not None:
        _MODEL_GEOMETRY[path] = geometry
    return geometry


def _load_ground_model_mesh(path: str, radius: float,
                            rim_height: float) -> Optional[bpy.types.Mesh]:
    """Build the ground mesh from ``path``, scaled to ``radius`` x ``rim_height``.

    The real bowl is a flat floor whose raised rim follows a rounded-square
    boundary (~15 m radius on the axes, ~17.1 m at the corners) and climbs ~5 m
    over the last ~4 m.  Scaling the whole mesh horizontally to ``radius`` (its
    nominal half-extent) and vertically so the rim reaches ``rim_height`` keeps
    that shape while letting the user grow/shrink the bowl and its wall height
    independently.  The geometry is centred on the vehicle and its floor sits on
    ``z = 0``, so the calibration blocks stay on the flat part.
    """
    geometry = _model_geometry(path)
    if geometry is None:
        return None
    verts, faces = geometry
    xs = [vertex[0] for vertex in verts]
    ys = [vertex[1] for vertex in verts]
    zs = [vertex[2] for vertex in verts]
    width = max(xs) - min(xs)
    depth = max(ys) - min(ys)
    height = max(zs) - min(zs)
    base_radius = 0.5 * max(width, depth)
    scale_h = radius / base_radius if base_radius > 1e-9 else 1.0
    scale_z = rim_height / height if height > 1e-9 else 1.0
    centre_x = 0.5 * (max(xs) + min(xs))
    centre_y = 0.5 * (max(ys) + min(ys))
    floor = min(zs)
    mesh = bpy.data.meshes.new("AVM_Ground")
    mesh.from_pydata(
        [((x - centre_x) * scale_h, (y - centre_y) * scale_h,
          (z - floor) * scale_z) for x, y, z in verts], [], faces)
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    mesh.update()
    return mesh


#: object name prefix -> the setting that shows/hides it (one place to rule them
#: all: the panels only flip the flags, :func:`apply_visibility` does the work)
VISIBILITY = (
    (GROUND_NAME, "show_ground"),
    (CAR_NAME, "show_car"),
    (BLOCK_PREFIX, "show_blocks"),
    (CAMERA_PREFIX, "show_cameras"),
    (PROP_PREFIX, "show_props"),
    (LABEL_PREFIX, "show_labels"),
    ("AVM_Coverage_", "show_coverage"),
    (SUN_NAME, "show_sun"),
)


def apply_visibility(settings) -> None:
    """Show/hide every AVM object according to the layer flags.

    Runs on a flag change without a rebuild, so toggling a layer never recreates
    geometry.  Both ``hide_render`` (F12 / exports) and ``hide_set`` (viewport)
    are set, so the toggle reads the same everywhere.
    """
    for prefix, attribute in VISIBILITY:
        visible = bool(getattr(settings, attribute, True))
        for obj in bpy.data.objects:
            if obj.name == prefix or obj.name.startswith(prefix):
                obj.hide_render = not visible
                obj.hide_set(not visible)


# ---------------------------------------------------------------------------
# the car: a small minibus built from primitives (one mesh, several materials)
# ---------------------------------------------------------------------------
#: material slot indices of the car mesh
CAR_BODY, CAR_GLASS, CAR_TIRE, CAR_HEAD, CAR_TAIL = range(5)


def _add_box(bm, x0, x1, y0, y1, z0, z1, material: int) -> None:
    verts = [bm.verts.new(position) for position in (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
    for face in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        bm.faces.new([verts[i] for i in face]).material_index = material


def _add_quad(bm, p0, p1, p2, p3, material: int) -> None:
    verts = [bm.verts.new(position) for position in (p0, p1, p2, p3)]
    bm.faces.new(verts).material_index = material


def _add_prism(bm, bottom, top, material: int) -> None:
    """A closed solid from two matching rings of four points (bottom, top)."""
    lower = [bm.verts.new(position) for position in bottom]
    upper = [bm.verts.new(position) for position in top]
    bm.faces.new(lower).material_index = material
    bm.faces.new(upper).material_index = material
    for index in range(4):
        nxt = (index + 1) % 4
        bm.faces.new((lower[index], lower[nxt], upper[nxt], upper[index])).material_index = material


def _add_cylinder(bm, centre, radius: float, half_len: float, axis: str,
                  segments: int, material: int) -> None:
    """A closed cylinder centred on ``centre``; ``axis`` is "X", "Y" or "Z"."""
    other = {"X": (1, 2), "Y": (0, 2), "Z": (0, 1)}[axis]
    index = {"X": 0, "Y": 1, "Z": 2}[axis]
    rings = []
    for side in (-half_len, half_len):
        ring = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            position = [centre[0], centre[1], centre[2]]
            position[index] += side
            position[other[0]] += radius * math.cos(angle)
            position[other[1]] += radius * math.sin(angle)
            ring.append(bm.verts.new(tuple(position)))
        rings.append(ring)
    inner, outer = rings
    for i in range(segments):
        nxt = (i + 1) % segments
        bm.faces.new((inner[i], inner[nxt], outer[nxt], outer[i])).material_index = material
    bm.faces.new(list(reversed(inner))).material_index = material
    bm.faces.new(outer).material_index = material


def _add_wheel(bm, centre_x: float, centre_y: float, radius: float,
               half_width: float, segments: int, material: int) -> None:
    """A wheel: a cylinder along X with its bottom resting on ``z = 0``."""
    _add_cylinder(bm, (centre_x, centre_y, radius), radius, half_width,
                  "X", segments, material)


def _minibus_mesh(name: str, length: float, width: float, height: float) -> bpy.types.Mesh:
    """A rough minibus silhouette: body, raked windshield, roof, windows, wheels.

    Everything is generated at real size with the origin on the ground under the
    car centre, ``+Y`` = front, so the object scale stays 1.  The wheels reach
    ``z = 0`` and stick out of the body sides; the front is readable from the
    windshield rake, the roof cap and the head/tail lights.
    """
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    length = max(1.0, float(length))
    width = max(0.5, float(width))
    height = max(0.5, float(height))

    wheel_radius = min(0.50, max(0.28, height * 0.16))
    floor = wheel_radius * 0.7
    beltline = floor + (height - floor) * 0.55

    # lower body (full length) and the cabin above the beltline
    _add_box(bm, -width / 2, width / 2, -length / 2, length / 2, floor, beltline, CAR_BODY)

    cabin_half = width * 0.47
    cabin_front = length * 0.46
    cabin_back = -length * 0.47
    rake = min(0.45, length * 0.09)
    rear_rake = 0.12
    bottom = [(-cabin_half, cabin_back, beltline), (cabin_half, cabin_back, beltline),
              (cabin_half, cabin_front, beltline), (-cabin_half, cabin_front, beltline)]
    top = [(-cabin_half, cabin_back + rear_rake, height),
           (cabin_half, cabin_back + rear_rake, height),
           (cabin_half, cabin_front - rake, height),
           (-cabin_half, cabin_front - rake, height)]
    _add_prism(bm, bottom, top, CAR_BODY)

    # roof cap, slightly proud of the cabin so the silhouette reads as a bus
    _add_box(bm, -width * 0.48, width * 0.48,
             -length * 0.46, length * 0.44, height, height + 0.06, CAR_BODY)

    # windshield: on the raked front plane, pushed out along its normal
    rise = height - beltline
    normal = math.hypot(rise, rake) or 1.0
    offset_y, offset_z = rise / normal * 0.015, rake / normal * 0.015
    glass_half = cabin_half - 0.07
    _add_quad(bm,
              (-glass_half, cabin_front + offset_y, beltline + offset_z),
              (glass_half, cabin_front + offset_y, beltline + offset_z),
              (glass_half, cabin_front - rake + offset_y, height + offset_z),
              (-glass_half, cabin_front - rake + offset_y, height + offset_z), CAR_GLASS)

    # side windows, one quad per side (just outside the cabin wall)
    for side in (-1.0, 1.0):
        x = side * (cabin_half + 0.015)
        y0, y1 = cabin_back + 0.40, cabin_front - 0.50
        z0, z1 = beltline + 0.16, height - 0.14
        points = [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]
        _add_quad(bm, *(points if side > 0 else list(reversed(points))), CAR_GLASS)

    # rear window, on the raked back plane
    _add_quad(bm,
              (-glass_half, cabin_back - 0.015, beltline + 0.20),
              (glass_half, cabin_back - 0.015, beltline + 0.20),
              (glass_half, cabin_back + rear_rake - 0.015, height - 0.14),
              (-glass_half, cabin_back + rear_rake - 0.015, height - 0.14), CAR_GLASS)

    # head and tail lights
    lamp_half = width * 0.13
    for side in (-1.0, 1.0):
        centre = side * width * 0.30
        _add_box(bm, centre - lamp_half, centre + lamp_half,
                 length / 2, length / 2 + 0.02,
                 floor + height * 0.16, floor + height * 0.16 + 0.18, CAR_HEAD)
        _add_box(bm, centre - lamp_half, centre + lamp_half,
                 -length / 2 - 0.02, -length / 2,
                 floor + height * 0.30, floor + height * 0.30 + 0.30, CAR_TAIL)

    # wheels: bottoms on the ground, outer faces proud of the body sides
    wheelbase = length * 0.62
    wheel_x = width / 2 - 0.02
    for side in (-1.0, 1.0):
        for front in (-1.0, 1.0):
            _add_wheel(bm, side * wheel_x, front * wheelbase / 2,
                       wheel_radius, 0.13, 16, CAR_TIRE)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


# ---------------------------------------------------------------------------
# props: pedestrians, crates and a pallet cart, like the real scene
# ---------------------------------------------------------------------------
#: material slot indices of the prop meshes
PED_JACKET, PED_HEAD, PED_LEGS = range(3)
CRATE_BOX, CRATE_RIM = range(2)
CART_WOOD, CART_TIRE, CART_METAL = range(3)


def _add_sphere(bm, centre, radius: float, segments: int, rings: int,
                material: int) -> None:
    top = bm.verts.new((centre[0], centre[1], centre[2] + radius))
    bottom = bm.verts.new((centre[0], centre[1], centre[2] - radius))
    bands = []
    for ring in range(1, rings):
        phi = math.pi * ring / rings
        bands.append([
            bm.verts.new((centre[0] + radius * math.sin(phi) * math.cos(2 * math.pi * s / segments),
                          centre[1] + radius * math.sin(phi) * math.sin(2 * math.pi * s / segments),
                          centre[2] + radius * math.cos(phi)))
            for s in range(segments)])
    for s in range(segments):
        nxt = (s + 1) % segments
        bm.faces.new((top, bands[0][s], bands[0][nxt])).material_index = material
        bm.faces.new((bottom, bands[-1][nxt], bands[-1][s])).material_index = material
    for ring in range(len(bands) - 1):
        for s in range(segments):
            nxt = (s + 1) % segments
            bm.faces.new((bands[ring][s], bands[ring][nxt],
                          bands[ring + 1][nxt], bands[ring + 1][s])).material_index = material


def _pedestrian_mesh(name: str, height: float = 1.70) -> bpy.types.Mesh:
    """A blocky person: legs, jacket, arms and a head."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    hip = height * 0.47
    shoulder = height * 0.83
    _add_box(bm, -0.17, 0.17, -0.11, 0.11, 0.0, hip, PED_LEGS)
    _add_box(bm, -0.21, 0.21, -0.13, 0.13, hip, shoulder, PED_JACKET)
    for side in (-1.0, 1.0):
        x = side * 0.27
        _add_box(bm, x - 0.06, x + 0.06, -0.10, 0.10, hip + 0.04, shoulder, PED_JACKET)
    _add_sphere(bm, (0.0, 0.0, height - 0.12), 0.115, 10, 6, PED_HEAD)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def _crate_mesh(name: str, length: float = 0.52, width: float = 0.36,
                height: float = 0.32) -> bpy.types.Mesh:
    """A plastic crate: a tapered box with a proud rim."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    hx, hy = length / 2.0, width / 2.0
    _add_box(bm, -hx, hx, -hy, hy, 0.0, height, CRATE_BOX)
    _add_box(bm, -hx - 0.015, hx + 0.015, -hy - 0.015, hy + 0.015,
             height - 0.05, height, CRATE_RIM)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def _cart_mesh(name: str, length: float = 1.15, width: float = 0.72) -> bpy.types.Mesh:
    """A small pallet cart: platform, four casters and a push handle."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    hx, hy = length / 2.0, width / 2.0
    deck = 0.20
    _add_box(bm, -hx, hx, -hy, hy, deck, deck + 0.09, CART_WOOD)
    for side in (-1.0, 1.0):
        for front in (-1.0, 1.0):
            _add_cylinder(bm, (side * (hx - 0.14), front * (hy - 0.12), 0.08),
                          0.08, 0.045, "X", 10, CART_TIRE)
    handle_y = -hy + 0.03
    _add_box(bm, -0.03, 0.03, handle_y - 0.03, handle_y + 0.03,
             deck + 0.09, deck + 0.85, CART_METAL)
    _add_box(bm, -0.17, 0.17, handle_y - 0.03, handle_y + 0.03,
             deck + 0.80, deck + 0.86, CART_METAL)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def _prop_slots(field) -> List:
    """Deterministic spots around the field (outside it, on the ground)."""
    geo = avm_layout.geometry(field)
    margin = 1.7
    slots = []
    for y in (-4.2, -1.6, 1.6, 4.2):
        slots.append((geo.half_x + margin, y))
        slots.append((-(geo.half_x + margin), y))
    for x in (-1.5, 0.0, 1.5):
        slots.append((x, geo.half_y + margin))
        slots.append((x, -(geo.half_y + margin)))
    return slots


def _ensure_props(scene: bpy.types.Scene, settings, target: bpy.types.Collection,
                  root: bpy.types.Object) -> List[bpy.types.Object]:
    """Create / update the pedestrians, crates and carts around the field."""
    # the desired list, in a fixed order so positions are stable across rebuilds
    wanted: List = []
    for index in range(max(0, int(settings.prop_carts))):
        wanted.append(("Cart", index))
    for index in range(max(0, int(settings.prop_boxes))):
        wanted.append(("Crate", index))
    for index in range(max(0, int(settings.prop_pedestrians))):
        wanted.append(("Pedestrian", index))

    materials = {
        "Pedestrian": (
            _principled("AVM_Ped_Jacket_Mat", (0.05, 0.08, 0.30, 1.0), 0.6),
            _principled("AVM_Ped_Head_Mat", (0.62, 0.45, 0.34, 1.0), 0.7),
            _principled("AVM_Ped_Legs_Mat", (0.10, 0.10, 0.12, 1.0), 0.7),
        ),
        "Crate": (
            _principled("AVM_Crate_Mat", (0.06, 0.20, 0.55, 1.0), 0.5),
            _principled("AVM_Crate_Rim_Mat", (0.03, 0.11, 0.34, 1.0), 0.5),
        ),
        "Cart": (
            _principled("AVM_Cart_Wood_Mat", (0.38, 0.24, 0.12, 1.0), 0.75),
            _principled("AVM_Cart_Tire_Mat", (0.04, 0.04, 0.04, 1.0), 0.85),
            _principled("AVM_Cart_Metal_Mat", (0.45, 0.47, 0.50, 1.0), 0.4),
        ),
    }
    meshes = {
        "Pedestrian": _pedestrian_mesh,
        "Crate": _crate_mesh,
        "Cart": _cart_mesh,
    }

    slots = _prop_slots(settings.field_spec())
    objects: List[bpy.types.Object] = []
    for order, (kind, index) in enumerate(wanted):
        name = f"{PROP_PREFIX}{kind}_{index:02d}"
        mesh = meshes[kind](name)
        obj = bpy.data.objects.get(name)
        if obj is None:
            obj = bpy.data.objects.new(name, mesh)
            target.objects.link(obj)
        _assign_mesh(obj, mesh, materials[kind])
        # spread the props over the slots and vary the heading deterministically
        x, y = slots[(order * 5) % len(slots)]
        obj.location = (x, y, 0.0)
        obj.rotation_euler = (0.0, 0.0, math.radians((order * 47) % 360))
        _parent(obj, root)
        hidden = not settings.show_props
        obj.hide_render = hidden
        obj.hide_set(hidden)
        objects.append(obj)

    # remove props that are no longer wanted
    wanted_names = {f"{PROP_PREFIX}{kind}_{index:02d}" for kind, index in wanted}
    for obj in list(target.objects):
        if obj.name.startswith(PROP_PREFIX) and obj.name not in wanted_names:
            bpy.data.objects.remove(obj, do_unlink=True)
    return objects


# ---------------------------------------------------------------------------
# ground text: 前 / 后 / 左 / 右 and the field title
# ---------------------------------------------------------------------------
def label_font(settings):
    """A CJK-capable font for the ground text, or ``None``.

    ``label_font`` wins when set; otherwise the first existing entry of
    :data:`CJK_FONT_CANDIDATES` is loaded.  Blender's built-in font has no CJK
    glyphs, so without one the Chinese labels would render blank.
    """
    if settings.label_font:
        try:
            return bpy.data.fonts.load(settings.label_font)
        except Exception:
            pass
    for path in CJK_FONT_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            return bpy.data.fonts.load(path)
        except Exception:
            continue
    return None


def _text_object(name: str, body: str, size: float, font, material,
                 target: bpy.types.Collection) -> bpy.types.Object:
    """Create / update a flat FONT object (readable from the top view)."""
    curve = bpy.data.curves.get(name)
    if curve is None:
        curve = bpy.data.curves.new(name, type="FONT")
    curve.body = body
    curve.size = size
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    curve.extrude = 0.002
    if font is not None:
        curve.font = font
    curve.materials.clear()
    curve.materials.append(material)

    obj = bpy.data.objects.get(name)
    if obj is None:
        obj = bpy.data.objects.new(name, curve)
        target.objects.link(obj)
    elif obj.data is not curve:
        obj.data = curve
    return obj


def _load_logo_image(path: str):
    """Load (and refresh) the logo image; ``None`` when it cannot be read."""
    try:
        image = bpy.data.images.load(path, check_existing=True)
    except Exception:
        return None
    # Blender caches loaded images: without a reload, replacing the file on disk
    # would keep showing the old logo after a rebuild
    try:
        image.reload()
    except Exception:
        pass
    return image


def _logo_material(image) -> bpy.types.Material:
    """A flat, alpha-aware material showing the logo image."""
    material = bpy.data.materials.get("AVM_Logo_Mat")
    if material is None:
        material = bpy.data.materials.new("AVM_Logo_Mat")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    texture = nodes.new("ShaderNodeTexImage")
    coords = nodes.new("ShaderNodeTexCoord")
    texture.image = image
    principled.inputs["Roughness"].default_value = 0.9
    links = material.node_tree.links
    # the quad has no UV map, so feed the texture Generated coordinates
    # (the object bounding box mapped to 0..1); without this the texture
    # samples a single texel and the decal is invisible
    links.new(coords.outputs["Generated"], texture.inputs["Vector"])
    links.new(texture.outputs["Color"], principled.inputs["Base Color"])
    if "Alpha" in principled.inputs:
        links.new(texture.outputs["Alpha"], principled.inputs["Alpha"])
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    material.diffuse_color = (1.0, 1.0, 1.0, 1.0)
    compat.set_material_blend(material)
    return material


def _logo_path(settings) -> Optional[str]:
    """Which logo image to draw: a custom one, the bundled one, or none.

    An empty ``logo_image`` means "use the bundled logo" - that file lives in the
    add-on (``logos/``) so it ships in the package and survives a restart, unlike
    a path into the repository.
    """
    if not settings.logo_enabled:
        return None
    custom = (settings.logo_image or "").strip()
    if custom:
        return bpy.path.abspath(custom)
    bundled = paths.logo_file()
    return bundled if os.path.exists(bundled) else None


def _ensure_logo(settings, target: bpy.types.Collection, y: float,
                 size: float) -> Optional[bpy.types.Object]:
    """The ground logo plane; removed again when the logo is turned off.

    ``logo_size`` is the logo **width**; the height follows the image aspect
    ratio, so a non-square logo is never stretched.
    """
    name = f"{LABEL_PREFIX}Logo"
    path = _logo_path(settings)
    if not path:
        existing = bpy.data.objects.get(name)
        if existing is not None:
            bpy.data.objects.remove(existing, do_unlink=True)
        return None
    image = _load_logo_image(path)
    if image is None:
        return None
    width = float(size)
    height = width * image.size[1] / image.size[0] if image.size[0] else width
    mesh = _quad_mesh(name, width, height)
    obj = bpy.data.objects.get(name)
    if obj is None:
        obj = bpy.data.objects.new(name, mesh)
        target.objects.link(obj)
    _assign_mesh(obj, mesh, [_logo_material(image)])
    obj.location = (0.0, y, LABEL_Z)
    obj.rotation_euler = (0.0, 0.0, 0.0)
    return obj


def _ensure_labels(scene: bpy.types.Scene, settings, target: bpy.types.Collection,
                   root: bpy.types.Object) -> List[bpy.types.Object]:
    """Create / update the ground text and the logo, laid out side by side."""
    font = label_font(settings)
    material = _principled("AVM_Label_Mat", (0.12, 0.12, 0.13, 1.0), 0.9)
    geo = avm_layout.geometry(settings.field_spec())
    offset = 0.9
    title_y = -(geo.half_y + 3.4)

    specs = [
        (f"{LABEL_PREFIX}Front", LABEL_TEXT["Front"], LABEL_SIZE, (0.0, geo.half_y + offset)),
        (f"{LABEL_PREFIX}Back", LABEL_TEXT["Back"], LABEL_SIZE, (0.0, -(geo.half_y + offset))),
        (f"{LABEL_PREFIX}Left", LABEL_TEXT["Left"], LABEL_SIZE, (-(geo.half_x + offset), 0.0)),
        (f"{LABEL_PREFIX}Right", LABEL_TEXT["Right"], LABEL_SIZE, (geo.half_x + offset, 0.0)),
    ]
    title = (settings.ground_title or "").strip()
    if title:
        specs.append((f"{LABEL_PREFIX}Title", title, TITLE_SIZE, (0.0, title_y)))

    wanted = {name for name, _, _, _ in specs} | {f"{LABEL_PREFIX}Logo"}
    objects: List[bpy.types.Object] = []
    for name, body, size, (x, y) in specs:
        obj = _text_object(name, body, size, font, material, target)
        obj.location = (x, y, LABEL_Z)
        obj.rotation_euler = (0.0, 0.0, 0.0)
        _parent(obj, root)
        objects.append(obj)

    logo = _ensure_logo(settings, target, title_y, settings.logo_size)
    title_object = bpy.data.objects.get(f"{LABEL_PREFIX}Title")
    if logo is not None:
        _parent(logo, root)
        objects.append(logo)
        if title_object is not None:
            # put the logo immediately before the text and shift the text right
            bpy.context.view_layer.update()
            width = float(title_object.dimensions.x) or 1.0
            size = float(settings.logo_size)
            gap = 0.45
            total = size + gap + width
            logo.location = (-total / 2.0 + size / 2.0, title_y, LABEL_Z)
            title_object.location = (-total / 2.0 + size + gap + width / 2.0, title_y, LABEL_Z)

    for obj in list(target.objects):
        if obj.name.startswith(LABEL_PREFIX) and obj.name not in wanted:
            bpy.data.objects.remove(obj, do_unlink=True)
    return objects


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------
def _principled(name: str, color, roughness: float = 0.7) -> bpy.types.Material:
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
    # viewport (solid) shading uses diffuse_color, not the shader nodes
    material.diffuse_color = color
    material.roughness = roughness
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Roughness"].default_value = roughness
    material.node_tree.links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


def _assign(obj: bpy.types.Object, material: bpy.types.Material) -> None:
    obj.data.materials.clear()
    obj.data.materials.append(material)


def _assign_many(obj: bpy.types.Object, materials) -> None:
    """Set the material slots in order (face ``material_index`` refers to these).

    Note: ``mesh.materials.clear()`` clamps every polygon's ``material_index`` to
    0, so this must never run on a mesh that already carries indices.
    """
    obj.data.materials.clear()
    for material in materials:
        obj.data.materials.append(material)


def _assign_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh, materials) -> None:
    """Put ``mesh`` on ``obj`` with its material slots.

    The slots are attached to the *mesh* before it becomes the object's data:
    clearing the slots of a mesh that already has per-face ``material_index``
    values would clamp them all to 0 (which silently dropped the car's glass,
    tires and lights).
    """
    for material in materials:
        mesh.materials.append(material)
    _replace_mesh(obj, mesh)


# ---------------------------------------------------------------------------
# object creation
# ---------------------------------------------------------------------------
def _new_mesh_object(name: str, mesh: bpy.types.Mesh,
                     target: bpy.types.Collection) -> bpy.types.Object:
    obj = bpy.data.objects.new(name, mesh)
    link_to_collection(obj, target)
    return obj


def _ensure_light(scene: bpy.types.Scene, target: bpy.types.Collection, name: str,
                  light_type: str, pitch: float, yaw: float) -> bpy.types.Object:
    """Create / reuse a light of the scene, aimed by a (pitch, yaw) rotation."""
    light = bpy.data.objects.get(name)
    if light is None:
        light_data = bpy.data.lights.new(name, type=light_type)
        light = bpy.data.objects.new(name, light_data)
        target.objects.link(light)
    light.rotation_euler = (pitch, 0.0, yaw)
    return light


def _ensure_root(scene: bpy.types.Scene, target: bpy.types.Collection) -> bpy.types.Object:
    root = bpy.data.objects.get(ROOT_NAME)
    if root is None:
        root = bpy.data.objects.new(ROOT_NAME, None)
        root.empty_display_type = "PLAIN_AXES"
        root.empty_display_size = 1.0
        target.objects.link(root)
    elif target not in root.users_collection:
        link_to_collection(root, target)
    return root


def _parent(obj: bpy.types.Object, root: bpy.types.Object) -> None:
    if obj.parent is not root:
        obj.parent = root
        obj.matrix_parent_inverse = root.matrix_world.inverted()


def view_targets(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    """The subject of the default view: the blocks, the car and the cameras.

    Deliberately *not* the 30 m ground, the props (1.7 m outside the field), the
    ground text or the lights - framing those would shrink the 4.8 x 8.4 m field
    to a few percent of the viewport.  The blocks are the outermost field
    objects, so they alone decide the frame.
    """
    target = bpy.data.collections.get(COLLECTION_NAME)
    if target is None:
        return []
    return [obj for obj in target.objects
            if obj.name == CAR_NAME
            or obj.name.startswith((BLOCK_PREFIX, CAMERA_PREFIX))]


def _ensure_cameras(scene: bpy.types.Scene, settings, target: bpy.types.Collection,
                    preset_cameras: Dict[str, Dict]):
    """Create the four fisheye cameras, configured from the preset records."""
    cameras: Dict[str, bpy.types.Object] = {}
    messages: List[str] = []
    for name in avm_layout.CAMERAS:
        object_name = avm_cameras.object_name(CAMERA_PREFIX, name)
        camera = bpy.data.objects.get(object_name)
        if camera is None:
            record = preset_cameras.get(name, {})
            camera, _ = camera_factory.add_camera(
                scene, model="fisheye", preset=None, name=object_name,
                location=tuple(record.get("location", (0.0, 0.0, 0.0))),
            )
            # add_camera names the object after the camera data-block, which
            # Blender may suffix (.001) when an old data-block lingers; the AVM
            # scene owns these names, so force them
            camera.name = object_name
            camera.data.name = object_name
            ok, apply_messages = camera_factory.configure_from_record(camera, record, scene)
            if not ok:
                messages.append(f"{name}: " + "; ".join(apply_messages))
            apply_camera_pose(camera, record.get("location", (0.0, 0.0, 0.0)),
                              [math.radians(value)
                               for value in record.get("rotation", (0.0, 0.0, 0.0))])
        link_to_collection(camera, target)
        cameras[name] = camera
    return cameras, messages


def apply_camera_pose(camera: bpy.types.Object, location, rotation) -> None:
    """Set the mount pose from the AVM record.

    ``rotation`` is the record's XYZ Euler in **radians** (the unit Blender's
    Euler property stores); the coverage maths and :func:`object_matrix` work in
    degrees, so it is converted here.  Writing ``matrix_world`` also updates the
    object's own ``location`` / ``rotation_euler``, so ``CV Extrinsics`` (which
    shows the object transform) stays in step.
    """
    rotation_deg = [math.degrees(value) for value in rotation]
    matrix = avm_coverage.object_matrix(location, rotation_deg)
    camera.matrix_world = Matrix((matrix[0:4], matrix[4:8],
                                  matrix[8:12], matrix[12:16]))


# ---------------------------------------------------------------------------
# build / rebuild
# ---------------------------------------------------------------------------
def build(scene: bpy.types.Scene, settings, preset: Optional[Dict] = None) -> Dict:
    """Create the scene objects and return them (also stored on ``settings``)."""
    from . import controller

    preset = preset or avm_layout.load_preset()
    controller.load_preset_into(settings, preset)

    target = collection(COLLECTION_NAME, scene, create=True)
    root = _ensure_root(scene, target)
    settings.root = root
    preset_cameras = {record["name"]: record
                      for record in avm_layout.cameras_from_preset(preset)}
    _, messages = _ensure_cameras(scene, settings, target, preset_cameras)
    created = rebuild(scene, settings)
    created["messages"] = messages + (created.get("messages") or [])
    return created


def _ensure_render_setup(scene: bpy.types.Scene, messages: List[str]) -> None:
    """Make the scene renderable the way the AVM material needs it.

    * **Cycles**: the custom OSL camera only exists there; a fresh scene (e.g.
      after a Blender restart) defaults to EEVEE and F12 comes out empty.
    * **Standard view transform**: Blender 4.x defaults to AgX, which tone-maps
      and desaturates the image, so the blocks, the ground and the car no longer
      match their albedo.  The Camera Scene builder does the same.
    """
    if scene.render.engine != "CYCLES":
        scene.render.engine = "CYCLES"
        messages.append("switched the render engine to Cycles (custom cameras need it)")
    if scene.view_settings.view_transform != "Standard":
        scene.view_settings.view_transform = "Standard"
        messages.append("set the view transform to Standard (AgX would tone-map the images)")


def rebuild(scene: bpy.types.Scene, settings) -> Dict[str, List]:
    """Update every object in place; creates anything that is missing."""
    messages: List[str] = []
    _ensure_render_setup(scene, messages)
    if settings.root is None:
        target = collection(COLLECTION_NAME, scene, create=True)
        settings.root = _ensure_root(scene, target)
    root = settings.root
    target = collection(COLLECTION_NAME, scene, create=True)
    for obj in [root] + [child for child in bpy.data.objects if child.parent is root]:
        if target not in obj.users_collection:
            link_to_collection(obj, target)

    field = settings.field_spec()
    geo = avm_layout.geometry(field)
    # the ground is a plain light grey: any printed grid would be picked up by
    # the black-region corner detector, and black blocks need contrast
    ground_material = _principled("AVM_Ground_Mat", (0.62, 0.62, 0.60, 1.0), 0.9)
    block_material = _principled("AVM_Block_Mat", (0.02, 0.02, 0.02, 1.0), 0.9)
    block_border_material = _principled(
        "AVM_Block_Border_Mat", (0.95, 0.95, 0.95, 1.0), 0.9)

    # ground ---------------------------------------------------------------
    # The real Falcon ground is a bowl the app projects the four camera images
    # onto, so by default the simulated cameras see the same mesh; a flat plane
    # remains the fallback (toggle off, or a model that will not load).
    ground = bpy.data.objects.get(GROUND_NAME)
    if ground is None:
        ground = _new_mesh_object(GROUND_NAME, _quad_mesh("AVM_Ground", 1.0, 1.0), target)
    model_path = ground_model_path(settings)
    model_key = ground_model_key(settings)
    loaded = ground.data.get(GROUND_MODEL_KEY) if ground.data is not None else None
    if not model_key:
        loaded = None
    elif loaded != model_key:
        if not os.path.isfile(model_path):
            messages.append(f"ground model {os.path.basename(model_path)!r} not found; "
                            "using the flat plane")
            loaded = None
        else:
            mesh = _load_ground_model_mesh(
                model_path, settings.ground_radius, settings.ground_rim_height)
            if mesh is not None:
                mesh[GROUND_MODEL_KEY] = model_key
                _replace_mesh(ground, mesh)
                loaded = model_key
            else:
                messages.append(f"ground model {os.path.basename(model_path)!r} could not "
                                "be imported; using the flat plane")
                loaded = None
    if loaded is not None:
        ground.location = (0.0, 0.0, 0.0)
    else:
        _replace_mesh(ground, _quad_mesh("AVM_Ground", settings.ground_w, settings.ground_d))
        ground.location = (0.0, 0.0, -GROUND_DROP)
    _assign(ground, ground_material)
    _parent(ground, root)
    ground.hide_render = not settings.show_ground

    # car ------------------------------------------------------------------
    car_length, car_width = settings.car_size()
    car = bpy.data.objects.get(CAR_NAME)
    if car is None:
        car = _new_mesh_object(CAR_NAME, _quad_mesh("AVM_Car", 1.0, 1.0), target)
    car_mesh = _minibus_mesh("AVM_Car", car_length, car_width, settings.car_height)
    _assign_mesh(car, car_mesh, (
        _principled("AVM_Car_Mat", *avm_layout.MINIBUS_MATERIALS["body"]),
        _principled("AVM_Car_Glass_Mat", *avm_layout.MINIBUS_MATERIALS["glass"]),
        _principled("AVM_Car_Tire_Mat", *avm_layout.MINIBUS_MATERIALS["tire"]),
        _principled("AVM_Car_Head_Mat", *avm_layout.MINIBUS_MATERIALS["head"]),
        _principled("AVM_Car_Tail_Mat", *avm_layout.MINIBUS_MATERIALS["tail"]),
    ))
    car.location = (0.0, 0.0, settings.car_clearance)
    _parent(car, root)
    car.hide_render = not settings.show_car

    # calibration blocks ---------------------------------------------------
    blocks = avm_layout.block_rects(field)
    block_objects = []
    for key, (x0, y0, x1, y1) in blocks.items():
        name = f"{BLOCK_PREFIX}{BLOCK_SUFFIX[key]}"
        block = bpy.data.objects.get(name)
        if block is None:
            block = _new_mesh_object(name, _quad_mesh(name, 1.0, 1.0), target)
        mesh = _block_mesh(name, x1 - x0, y1 - y0, BLOCK_BORDER)
        _assign_mesh(block, mesh, (block_material, block_border_material))
        block.location = (0.5 * (x0 + x1), 0.5 * (y0 + y1), settings.block_lift)
        _parent(block, root)
        block.hide_render = not settings.show_blocks
        block_objects.append(block)

    # cameras --------------------------------------------------------------
    # The camera object's own transform is the mount pose (so CV Extrinsics and
    # the AVM panels show the same thing); a rebuild never resets it, only the
    # preset / import do.
    camera_objects = []
    for index, name in enumerate(avm_layout.CAMERAS):
        record = settings.camera(name)
        camera = bpy.data.objects.get(f"{CAMERA_PREFIX}{CAMERA_SUFFIX[name]}")
        if camera is None:
            continue
        camera.hide_render = not (record.enable if record else True) or not settings.show_cameras
        # repair a camera without bytecode (e.g. a compile that failed when it
        # was created, or a .blend from before the Cycles osl fix)
        if not shader.is_compiled(camera.data):
            ok, apply_messages = apply_mod.apply_settings(
                camera.data, camera.data.opencv_cam, scene)
            if not ok:
                messages.append(f"{name}: " + "; ".join(apply_messages))
        _parent(camera, root)
        camera_objects.append(camera)

    # A key sun plus a dim fill from the opposite side, both shadowless, so the
    # scene reads the same whatever other lights the surrounding scene has (a
    # fresh scene after a restart only has these).  Shadows stay off: a cast
    # shadow is a dark ground patch the corner detector can mistake for a block.
    sun = _ensure_light(scene, target, SUN_NAME, "SUN", math.radians(35.0), math.radians(-40.0))
    sun.data.energy = float(settings.sun_energy)
    sun.data.use_shadow = bool(settings.sun_shadow)
    fill = _ensure_light(scene, target, FILL_NAME, "SUN", math.radians(50.0), math.radians(140.0))
    fill.data.energy = float(settings.sun_energy) * 0.4
    fill.data.use_shadow = False
    for light in (sun, fill):
        _parent(light, root)

    # props (pedestrians, crates, a pallet cart) ---------------------------
    props = _ensure_props(scene, settings, target, root)

    # ground text (前 / 后 / 左 / 右 + the field title) ----------------------
    labels = _ensure_labels(scene, settings, target, root)

    _apply_active_camera(scene, settings)
    settings.revision += 1
    apply_visibility(settings)
    return {
        "root": root,
        "ground": ground,
        "car": car,
        "blocks": block_objects,
        "cameras": camera_objects,
        "props": props,
        "labels": labels,
        "sun": sun,
        "messages": messages,
    }


def _apply_active_camera(scene: bpy.types.Scene, settings) -> None:
    """Make the active camera the render camera (F12 uses ``scene.camera``)."""
    camera = bpy.data.objects.get(
        f"{CAMERA_PREFIX}{CAMERA_SUFFIX[settings.active_camera]}")
    if camera is None:
        return
    scene.camera = camera


def remove(scene: bpy.types.Scene, settings) -> int:
    """Delete the whole AVM Scene and clear the root pointer."""
    removed = remove_collection_objects(COLLECTION_NAME)
    root = settings.root
    if root is not None and root.name in bpy.data.objects:
        bpy.data.objects.remove(root, do_unlink=True)
        removed += 1
    settings.root = None
    for name in avm_layout.CAMERAS:
        camera = bpy.data.objects.get(f"{CAMERA_PREFIX}{CAMERA_SUFFIX[name]}")
        if camera is not None:
            bpy.data.objects.remove(camera, do_unlink=True)
            removed += 1
    target = bpy.data.collections.get(COLLECTION_NAME)
    if target is not None and not target.objects and not target.children:
        bpy.data.collections.remove(target)
    return removed
