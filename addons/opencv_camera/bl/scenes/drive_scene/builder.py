"""Drive Scene builder: the car park, the vehicle, the camera and the keyframes.

The objects are created once and then **updated in place** (idempotent rebuild),
so a slider drag never destroys the user's selection or their camera tweaks.
Every mesh is generated at its real size (object scale stays 1).

World convention (shared with the AVM Scene): metres, Z up, the aisle runs along
``+Y`` and the vehicle's nose is ``+Y``.  ``DRIVE_Vehicle`` is the moving frame -
the car and the front camera hang off it, so the camera keeps its **vehicle
frame** mount pose while the empty carries the world pose that
``frames.csv`` records.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import bmesh
import bpy
from mathutils import Matrix

from ....core.scenes import avm_layout, drive_lot
from ... import camera_factory, compat
from ..base import collection, link_to_collection, remove_collection_objects

ROOT_NAME = "DRIVE_Root"
VEHICLE_NAME = "DRIVE_Vehicle"
COLLECTION_NAME = "Drive Scene"

GROUND_NAME = "DRIVE_Ground"
CAR_NAME = "DRIVE_Car"
CAMERA_NAME = "DRIVE_Cam_Front"
PILLAR_PREFIX = "DRIVE_Pillar_"
WALL_PREFIX = "DRIVE_Wall_"
LIGHT_PREFIX = "DRIVE_Light_"
PARKED_PREFIX = "DRIVE_Parked_"
NUMBER_PREFIX = "DRIVE_Number_"

#: the painted lines sit on the nominal ``z = 0`` plane (the car's wheels rest on
#: them) while the slab is dropped by this much, so the two never z-fight
SLAB_DROP = 0.002
#: painted line width [m]
LINE_WIDTH = 0.12
#: round pillars and walls of the car park [m]
PILLAR_SIZE = 0.6
WALL_HEIGHT = 3.0
WALL_THICKNESS = 0.2
#: the ceiling light: one large soft panel just above the wall tops (a row of
#: small lamps burned bright pools into the floor)
CEILING_LIFT = 0.2
#: painted bay numbers [m] and the height they lie at (above the paint, which is
#: at z = 0, and below anything the car drives on)
NUMBER_SIZE = 0.5
NUMBER_EXTRUDE = 0.002
NUMBER_Z = 0.004

#: material slot indices of the lot mesh
LOT_FLOOR, LOT_WHITE, LOT_YELLOW = range(3)
#: material slot indices of the car mesh
CAR_BODY, CAR_GLASS, CAR_TIRE, CAR_LAMP = range(4)


# ---------------------------------------------------------------------------
# meshes
# ---------------------------------------------------------------------------
def _add_quad(bm, p0, p1, p2, p3, material: int) -> None:
    verts = [bm.verts.new(position) for position in (p0, p1, p2, p3)]
    bm.faces.new(verts).material_index = material


def _add_box(bm, x0, x1, y0, y1, z0, z1, material: int) -> None:
    verts = [bm.verts.new(position) for position in (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
    for face in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        bm.faces.new([verts[i] for i in face]).material_index = material


def _add_cylinder(bm, centre, radius: float, half_len: float, axis: str,
                  segments: int, material: int) -> None:
    other = {"X": (1, 2), "Y": (0, 2), "Z": (0, 1)}[axis]
    index = {"X": 0, "Y": 1, "Z": 2}[axis]
    rings = []
    for side in (-half_len, half_len):
        ring = []
        for step in range(segments):
            angle = 2.0 * math.pi * step / segments
            position = [centre[0], centre[1], centre[2]]
            position[index] += side
            position[other[0]] += radius * math.cos(angle)
            position[other[1]] += radius * math.sin(angle)
            ring.append(bm.verts.new(tuple(position)))
        rings.append(ring)
    inner, outer = rings
    for step in range(segments):
        nxt = (step + 1) % segments
        bm.faces.new((inner[step], inner[nxt], outer[nxt], outer[step])).material_index = material
    bm.faces.new(list(reversed(inner))).material_index = material
    bm.faces.new(outer).material_index = material


def _paint(bm, x0: float, x1: float, y0: float, y1: float, material: int,
           z: float = 0.0) -> None:
    """A painted marking: a flat quad on the floor plane."""
    left, right = min(x0, x1), max(x0, x1)
    back, front = min(y0, y1), max(y0, y1)
    _add_quad(bm, (left, back, z), (right, back, z),
              (right, front, z), (left, front, z), material)


def _strip(bm, x: float, y0: float, y1: float, material: int,
           width: float = LINE_WIDTH, z: float = 0.0) -> None:
    """A line along Y centred on ``x``."""
    _paint(bm, x - width / 2, x + width / 2, y0, y1, material, z)


def _band(bm, y: float, x0: float, x1: float, material: int,
          width: float = LINE_WIDTH, z: float = 0.0) -> None:
    """A line along X centred on ``y``."""
    _paint(bm, x0, x1, y - width / 2, y + width / 2, material, z)


def _rect_outline(bm, x0: float, x1: float, y0: float, y1: float, material: int,
                  width: float = LINE_WIDTH, z: float = 0.0) -> None:
    _strip(bm, x0, y0, y1, material, width, z)
    _strip(bm, x1, y0, y1, material, width, z)
    _band(bm, y0, x0, x1, material, width, z)
    _band(bm, y1, x0, x1, material, width, z)


def _lot_mesh(name: str, lot_length: float, lot_width: float, aisle_width: float,
              bay_depth: float, bay_width: float, show_bays: bool,
              keep_clear_y: float) -> bpy.types.Mesh:
    """The car park floor: one slab plus the painted markings as flat quads.

    Markings are real geometry with their own material slot (the AVM Scene's
    calibration blocks are built the same way): they stay sharp at any camera
    resolution and cost nothing to texture.  The dashed centre line runs right
    under the car - that is the marking a transparent-chassis algorithm has to
    reconstruct from the frames it *can* see.
    """
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    half_l, half_w = lot_length / 2.0, lot_width / 2.0
    half_aisle = aisle_width / 2.0
    _paint(bm, -half_w, half_w, -half_l, half_l, LOT_FLOOR, z=-SLAB_DROP)

    if show_bays:
        # aisle edge lines, one row of bay dividers per side (the dividers come
        # from the shared layout, so a bay number or a parked car always lines up
        # with the paint)
        dividers = drive_lot.divider_positions(lot_length, bay_width)
        for side in (-1.0, 1.0):
            _strip(bm, side * half_aisle, -half_l, half_l, LOT_WHITE)
            for divider in dividers:
                # the end dividers are pulled in by half a line, so no paint
                # sticks out past the slab
                y = min(max(divider, -half_l + LINE_WIDTH / 2.0),
                        half_l - LINE_WIDTH / 2.0)
                _band(bm, y, side * half_aisle, side * (half_aisle + bay_depth),
                      LOT_WHITE)
        # dashed centre line: 3 m of paint, 3 m of gap
        dash, gap = 3.0, 3.0
        y = -half_l + dash
        while y <= half_l:
            _strip(bm, 0.0, y - dash, y, LOT_WHITE)
            y += dash + gap
        # "keep clear" box at the end of the aisle (the only yellow marking)
        _rect_outline(bm, -half_aisle + 0.4, half_aisle - 0.4,
                      keep_clear_y, keep_clear_y + 3.0, LOT_YELLOW)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def _car_mesh(name: str, length: float, width: float, height: float) -> bpy.types.Mesh:
    """A compact minibus silhouette: body, cabin, windows, wheels, lamps.

    Generated at real size with the origin on the ground under the car centre and
    ``+Y`` = front, so the object scale stays 1 and the keyframed vehicle empty
    can carry it directly.
    """
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    length = max(1.0, float(length))
    width = max(0.5, float(width))
    height = max(0.5, float(height))

    wheel_radius = min(0.50, max(0.28, height * 0.16))
    floor = wheel_radius * 0.7
    beltline = floor + (height - floor) * 0.55
    half_w, half_l = width / 2.0, length / 2.0

    # body up to the beltline, then the cabin (inset, with raked front and rear)
    _add_box(bm, -half_w, half_w, -half_l, half_l, floor, beltline, CAR_BODY)
    cabin_half = width * 0.47
    cabin_front, cabin_back = length * 0.46, -length * 0.47
    rake = min(0.45, length * 0.09)
    rear_rake = 0.12
    bottom = [(-cabin_half, cabin_back, beltline), (cabin_half, cabin_back, beltline),
              (cabin_half, cabin_front, beltline), (-cabin_half, cabin_front, beltline)]
    top = [(-cabin_half, cabin_back + rear_rake, height),
           (cabin_half, cabin_back + rear_rake, height),
           (cabin_half, cabin_front - rake, height),
           (-cabin_half, cabin_front - rake, height)]
    lower = [bm.verts.new(position) for position in bottom]
    upper = [bm.verts.new(position) for position in top]
    bm.faces.new(lower).material_index = CAR_BODY
    bm.faces.new(upper).material_index = CAR_BODY
    for index in range(4):
        nxt = (index + 1) % 4
        bm.faces.new((lower[index], lower[nxt], upper[nxt], upper[index])).material_index = CAR_BODY

    # windshield (on the raked plane), side windows, rear window
    rise = height - beltline
    normal = math.hypot(rise, rake) or 1.0
    offset_y, offset_z = rise / normal * 0.015, rake / normal * 0.015
    glass_half = cabin_half - 0.07
    _add_quad(bm,
              (-glass_half, cabin_front + offset_y, beltline + offset_z),
              (glass_half, cabin_front + offset_y, beltline + offset_z),
              (glass_half, cabin_front - rake + offset_y, height + offset_z),
              (-glass_half, cabin_front - rake + offset_y, height + offset_z), CAR_GLASS)
    for side in (-1.0, 1.0):
        x = side * (cabin_half + 0.015)
        points = [(x, cabin_back + 0.40, beltline + 0.16),
                  (x, cabin_front - 0.50, beltline + 0.16),
                  (x, cabin_front - 0.50, height - 0.14),
                  (x, cabin_back + 0.40, height - 0.14)]
        _add_quad(bm, *(points if side > 0 else list(reversed(points))), CAR_GLASS)
    _add_quad(bm,
              (-glass_half, cabin_back - 0.015, beltline + 0.20),
              (glass_half, cabin_back - 0.015, beltline + 0.20),
              (glass_half, cabin_back + rear_rake - 0.015, height - 0.14),
              (-glass_half, cabin_back + rear_rake - 0.015, height - 0.14), CAR_GLASS)

    # head and tail lamps, proud of the front / rear face
    lamp_half = width * 0.13
    for side in (-1.0, 1.0):
        centre = side * width * 0.30
        _add_box(bm, centre - lamp_half, centre + lamp_half,
                 half_l, half_l + 0.02,
                 floor + height * 0.16, floor + height * 0.16 + 0.18, CAR_LAMP)
        _add_box(bm, centre - lamp_half, centre + lamp_half,
                 -half_l - 0.02, -half_l,
                 floor + height * 0.30, floor + height * 0.30 + 0.30, CAR_LAMP)

    # wheels: bottoms on the ground, outer faces proud of the body sides
    for side in (-1.0, 1.0):
        for end in (-1.0, 1.0):
            _add_cylinder(bm, (side * (half_w - 0.02), end * length * 0.31, wheel_radius),
                          wheel_radius, 0.13, "X", 16, CAR_TIRE)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------
def _principled(name: str, color, roughness: float = 0.7) -> bpy.types.Material:
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
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


def _mottled_material(name: str, color, roughness: float,
                      stains: Tuple[float, float, float, float],
                      grain: Tuple[float, float, float], grain_weight: float,
                      stops) -> bpy.types.Material:
    """A procedurally mottled floor: broad stains + a fine grain, no texture file.

    A flat floor leaves a transparent-chassis algorithm nothing to align against,
    but a loud pattern (cracks, high-contrast tiles) does not look like the real
    thing either.  Two low-contrast noise scales give every patch of floor its own
    signature while staying quiet; they are anchored to the ground object, so they
    do not swim as the car moves (see ``docs/drive-scene.md``).

    ``stains`` is ``(scale, detail, roughness, distortion)``, ``grain`` is
    ``(scale, detail, roughness)`` and ``stops`` is the colour ramp as
    ``(position, brightness, rgb offset)`` tuples.
    """
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
    material.diffuse_color = color
    material.roughness = roughness
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    coords = nodes.new("ShaderNodeTexCoord")

    stains_node = nodes.new("ShaderNodeTexNoise")
    stains_node.inputs["Scale"].default_value = stains[0]
    stains_node.inputs["Detail"].default_value = stains[1]
    stains_node.inputs["Roughness"].default_value = stains[2]
    stains_node.inputs["Distortion"].default_value = stains[3]

    grain_node = nodes.new("ShaderNodeTexNoise")
    grain_node.inputs["Scale"].default_value = grain[0]
    grain_node.inputs["Detail"].default_value = grain[1]
    grain_node.inputs["Roughness"].default_value = grain[2]

    grain_scale = nodes.new("ShaderNodeMath")
    grain_scale.operation = "MULTIPLY"
    grain_scale.inputs[1].default_value = grain_weight

    combine = nodes.new("ShaderNodeMath")   # stains * (1 - w) + grain * w
    combine.operation = "MULTIPLY_ADD"
    combine.inputs[1].default_value = 1.0 - grain_weight

    ramp = nodes.new("ShaderNodeValToRGB")

    def shade(level: float, offset) -> tuple:
        return tuple(min(1.0, max(0.0, value * level + shift))
                     for value, shift in zip(color[:3], offset)) + (1.0,)

    ramp.color_ramp.elements[0].position = stops[0][0]
    ramp.color_ramp.elements[0].color = shade(stops[0][1], stops[0][2])
    ramp.color_ramp.elements[1].position = stops[-1][0]
    ramp.color_ramp.elements[1].color = shade(stops[-1][1], stops[-1][2])
    for position, level, offset in stops[1:-1]:
        element = ramp.color_ramp.elements.new(position)
        element.color = shade(level, offset)

    principled.inputs["Roughness"].default_value = roughness
    links = material.node_tree.links
    for texture in (stains_node, grain_node):
        links.new(coords.outputs["Object"], texture.inputs["Vector"])
    links.new(grain_node.outputs["Fac"], grain_scale.inputs[0])
    links.new(stains_node.outputs["Fac"], combine.inputs[0])
    links.new(grain_scale.outputs["Value"], combine.inputs[2])
    links.new(combine.outputs["Value"], ramp.inputs["Fac"])
    links.new(ramp.outputs["Color"], principled.inputs["Base Color"])
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


def _concrete_material(name: str) -> bpy.types.Material:
    """Poured concrete: mid grey, matte, gentle blotchiness."""
    return _mottled_material(
        name, (0.42, 0.43, 0.44, 1.0), 0.80,
        stains=(1.1, 6.0, 0.5, 0.9), grain=(30.0, 3.0, 0.6), grain_weight=0.2,
        stops=((0.00, 0.72, (0.030, 0.000, -0.030)),
               (0.40, 0.90, (0.0, 0.0, 0.0)),
               (0.70, 1.04, (0.0, 0.0, 0.0)),
               (1.00, 1.18, (0.0, 0.0, 0.0))))


def _asphalt_material(name: str) -> bpy.types.Material:
    """Asphalt: near black, matte, with the light aggregate speckle."""
    return _mottled_material(
        name, (0.085, 0.085, 0.09, 1.0), 0.92,
        stains=(2.4, 5.0, 0.55, 1.2), grain=(90.0, 4.0, 0.65), grain_weight=0.35,
        stops=((0.00, 0.55, (0.0, 0.0, 0.0)),
               (0.40, 0.85, (0.0, 0.0, 0.0)),
               (0.72, 1.25, (0.0, 0.0, 0.0)),
               (0.90, 1.90, (0.030, 0.030, 0.030)),
               (1.00, 2.60, (0.060, 0.060, 0.060))))


def _epoxy_material(name: str) -> bpy.types.Material:
    """Epoxy coating: light grey, semi-gloss, almost even."""
    return _mottled_material(
        name, (0.50, 0.51, 0.53, 1.0), 0.30,
        stains=(0.7, 4.0, 0.5, 0.6), grain=(70.0, 2.0, 0.5), grain_weight=0.1,
        stops=((0.00, 0.88, (0.0, 0.0, 0.010)),
               (0.50, 0.98, (0.0, 0.0, 0.0)),
               (1.00, 1.08, (0.0, 0.0, 0.0))))


def _checker_material(name: str) -> bpy.types.Material:
    """High contrast tiles - the strongest possible floor texture."""
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    checker = nodes.new("ShaderNodeTexChecker")
    coords = nodes.new("ShaderNodeTexCoord")
    checker.inputs["Scale"].default_value = 24.0
    checker.inputs["Color1"].default_value = (0.72, 0.72, 0.70, 1.0)
    checker.inputs["Color2"].default_value = (0.16, 0.17, 0.18, 1.0)
    principled.inputs["Roughness"].default_value = 0.75
    links = material.node_tree.links
    links.new(coords.outputs["Object"], checker.inputs["Vector"])
    links.new(checker.outputs["Color"], principled.inputs["Base Color"])
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


#: floor preset -> the material builder that paints it
FLOOR_MATERIALS = {
    "concrete": lambda: _concrete_material("DRIVE_Floor_Concrete_Mat"),
    "asphalt": lambda: _asphalt_material("DRIVE_Floor_Asphalt_Mat"),
    "epoxy": lambda: _epoxy_material("DRIVE_Floor_Epoxy_Mat"),
    "checker": lambda: _checker_material("DRIVE_Floor_Checker_Mat"),
    "plain": lambda: _principled("DRIVE_Floor_Plain_Mat", (0.42, 0.43, 0.44, 1.0), 0.8),
}


def _floor_material(settings) -> bpy.types.Material:
    builder = FLOOR_MATERIALS.get(settings.ground_texture, FLOOR_MATERIALS["concrete"])
    return builder()


def _lot_materials(settings) -> Tuple[bpy.types.Material, ...]:
    return (
        _floor_material(settings),
        _principled("DRIVE_Paint_White_Mat", (0.86, 0.86, 0.84, 1.0), 0.55),
        _principled("DRIVE_Paint_Yellow_Mat", (0.78, 0.62, 0.10, 1.0), 0.55),
    )


def _car_materials(body: Optional[bpy.types.Material] = None) -> Tuple[bpy.types.Material, ...]:
    """The car's slots: body (the ego car's paint, or a parked car's), glass, etc.

    The ego car paints from the shared minibus palette, so it looks exactly like
    the AVM Scene's car; a parked car passes its own body colour.
    """
    return (
        body or _principled("DRIVE_Car_Mat", *avm_layout.MINIBUS_MATERIALS["body"]),
        _principled("DRIVE_Car_Glass_Mat", *avm_layout.MINIBUS_MATERIALS["glass"]),
        _principled("DRIVE_Car_Tire_Mat", *avm_layout.MINIBUS_MATERIALS["tire"]),
        _principled("DRIVE_Car_Lamp_Mat", *avm_layout.MINIBUS_MATERIALS["head"]),
    )


def _parked_material(paint: int) -> bpy.types.Material:
    """One of the parked cars' paints (shared by every car that uses it)."""
    return _principled(f"DRIVE_Parked_Paint_{paint:02d}_Mat",
                       drive_lot.PARKED_COLORS[paint % len(drive_lot.PARKED_COLORS)], 0.32)


# ---------------------------------------------------------------------------
# objects
# ---------------------------------------------------------------------------
def _replace_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh) -> None:
    old = obj.data
    obj.data = mesh
    if old is not None and old.users == 0:
        bpy.data.meshes.remove(old)


def _assign_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh, materials) -> None:
    """Put ``mesh`` on ``obj`` with its slots (slots first: clearing them would
    clamp every polygon's material_index to 0)."""
    for material in materials:
        mesh.materials.append(material)
    _replace_mesh(obj, mesh)


def _new_mesh_object(name: str, mesh: bpy.types.Mesh,
                     target: bpy.types.Collection) -> bpy.types.Object:
    obj = bpy.data.objects.new(name, mesh)
    link_to_collection(obj, target)
    return obj


def _ensure_mesh_object(name: str, target: bpy.types.Collection,
                        mesh: Optional[bpy.types.Mesh] = None) -> bpy.types.Object:
    obj = bpy.data.objects.get(name)
    if obj is None:
        obj = _new_mesh_object(name, mesh or bpy.data.meshes.new(name), target)
    elif target not in obj.users_collection:
        link_to_collection(obj, target)
    return obj


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


def _parent_local(obj: bpy.types.Object, parent: bpy.types.Object) -> None:
    """Parent ``obj`` so its **local** transform is the one that is kept.

    The AVM Scene parents with ``matrix_world.inverted()`` (the world pose is
    what matters there); here the child pose *is* a vehicle-frame quantity, so
    the inverse has to stay the identity.
    """
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)


def _ensure_vehicle(root: bpy.types.Object, target: bpy.types.Collection) -> bpy.types.Object:
    vehicle = bpy.data.objects.get(VEHICLE_NAME)
    if vehicle is None:
        vehicle = bpy.data.objects.new(VEHICLE_NAME, None)
        vehicle.empty_display_type = "PLAIN_AXES"
        vehicle.empty_display_size = 1.5
        target.objects.link(vehicle)
    elif target not in vehicle.users_collection:
        link_to_collection(vehicle, target)
    vehicle.rotation_mode = "XYZ"
    _parent_local(vehicle, root)
    return vehicle


def _ensure_walls(target: bpy.types.Collection, lot_length: float, lot_width: float,
                  material: bpy.types.Material) -> List[bpy.types.Object]:
    """Four walls around the slab (an open-top box: the ceiling would hide the
    scene from the default 3/4 view, and the footage only needs the floor)."""
    half_l, half_w = lot_length / 2.0, lot_width / 2.0
    specs = (
        ("Left", -half_w, -half_w + WALL_THICKNESS, -half_l, half_l),
        ("Right", half_w - WALL_THICKNESS, half_w, -half_l, half_l),
        ("Back", -half_w, half_w, -half_l, -half_l + WALL_THICKNESS),
        ("Front", -half_w, half_w, half_l - WALL_THICKNESS, half_l),
    )
    walls: List[bpy.types.Object] = []
    for suffix, x0, x1, y0, y1 in specs:
        name = f"{WALL_PREFIX}{suffix}"
        wall = _ensure_mesh_object(name, target)
        mesh = bpy.data.meshes.new(name)
        bm = bmesh.new()
        _add_box(bm, x0, x1, y0, y1, 0.0, WALL_HEIGHT, 0)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()
        _assign_mesh(wall, mesh, (material,))
        wall.location = (0.0, 0.0, 0.0)
        walls.append(wall)
    return walls


def _ensure_pillars(target: bpy.types.Collection, lot_length: float,
                    aisle_width: float, count: int,
                    material: bpy.types.Material) -> List[bpy.types.Object]:
    """Square columns along both sides of the aisle, evenly spaced."""
    pillars: List[bpy.types.Object] = []
    positions = drive_lot.pillar_positions(lot_length, aisle_width, count)
    for index, (x, y) in enumerate(positions):
        name = f"{PILLAR_PREFIX}{'L' if x < 0 else 'R'}{index % count:02d}"
        pillar = _ensure_mesh_object(name, target)
        mesh = bpy.data.meshes.new(name)
        bm = bmesh.new()
        _add_box(bm, -PILLAR_SIZE / 2, PILLAR_SIZE / 2, -PILLAR_SIZE / 2, PILLAR_SIZE / 2,
                 0.0, WALL_HEIGHT, 0)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()
        _assign_mesh(pillar, mesh, (material,))
        pillar.location = (x, y, 0.0)
        pillars.append(pillar)
    # drop the pillars the user removed from the count
    for obj in list(target.objects):
        if obj.name.startswith(PILLAR_PREFIX) and obj not in pillars:
            bpy.data.objects.remove(obj, do_unlink=True)
    return pillars


def _ensure_numbers(target: bpy.types.Collection, settings,
                    lot_bays: List[drive_lot.Bay]) -> List[bpy.types.Object]:
    """The bay numbers: text objects lying flat in the aisle, one per bay.

    Text rather than painted geometry (a number is a glyph) and digits exist in
    Blender's built-in font, so unlike the AVM ground text this needs no CJK
    font hunt.  Each number sits in the aisle **in front of its bay**: a parked
    car cannot cover it, and the camera keeps it in view while driving past -
    which is what makes the clip comparable frame by frame.
    """
    material = _principled("DRIVE_Paint_White_Mat", (0.86, 0.86, 0.84, 1.0), 0.55)
    numbers: List[bpy.types.Object] = []
    wanted = []
    if settings.bay_numbers and settings.show_bays:
        for bay in lot_bays:
            x, y = drive_lot.label_position(bay, settings.aisle_width)
            wanted.append((f"{NUMBER_PREFIX}{bay.label}", bay.label, x, y))
    for name, body, x, y in wanted:
        obj = bpy.data.objects.get(name)
        if obj is None or obj.type != "FONT":
            if obj is not None:
                bpy.data.objects.remove(obj, do_unlink=True)
            obj = bpy.data.objects.new(name, bpy.data.curves.new(name, type="FONT"))
            target.objects.link(obj)
        curve = obj.data
        curve.body = body
        curve.size = NUMBER_SIZE
        curve.extrude = NUMBER_EXTRUDE
        curve.align_x = "CENTER"
        curve.align_y = "CENTER"
        obj.location = (x, y, NUMBER_Z)
        obj.rotation_euler = (0.0, 0.0, 0.0)  # a FONT curve already lies flat
        curve.materials.clear()
        curve.materials.append(material)
        numbers.append(obj)
    for obj in list(target.objects):
        if obj.name.startswith(NUMBER_PREFIX) and obj not in numbers:
            bpy.data.objects.remove(obj, do_unlink=True)
    return numbers


def _ensure_parked_cars(target: bpy.types.Collection, root: bpy.types.Object,
                        settings, lot_bays: List[drive_lot.Bay],
                        pillars: List[Tuple[float, float]]) -> List[bpy.types.Object]:
    """The parked vehicles: nose-in, one form per bay, from the shared layout."""
    cars: List[bpy.types.Object] = []
    wanted = drive_lot.parked_cars(lot_bays, pillars, int(settings.parked_cars))
    for parked in wanted:
        name = f"{PARKED_PREFIX}{parked.bay.label}"
        car = _ensure_mesh_object(name, target)
        _assign_mesh(car, _car_mesh(name, parked.length, parked.width, parked.height),
                     _car_materials(_parked_material(parked.paint)))
        car.location = (parked.bay.x, parked.bay.y, 0.0)
        car.rotation_mode = "XYZ"
        car.rotation_euler = (0.0, 0.0, math.radians(parked.yaw))
        _parent_local(car, root)
        cars.append(car)
    for obj in list(target.objects):
        if obj.name.startswith(PARKED_PREFIX) and obj not in cars:
            bpy.data.objects.remove(obj, do_unlink=True)
    return cars


def _ensure_lights(scene: bpy.types.Scene, target: bpy.types.Collection,
                   lot_length: float, lot_width: float, settings) -> List[bpy.types.Object]:
    """One soft panel over the whole car park.

    A row of small lamps gave the floor bright pools and dead zones: the same
    bay was lit differently at different y, so frames of one clip did not
    compare.  A single emitter the size of the lot, at ceiling height, lights it
    evenly - which is what a drive clip needs.
    """
    name = f"{LIGHT_PREFIX}Ceiling"
    light = bpy.data.objects.get(name)
    if light is None:
        light_data = bpy.data.lights.new(name, type="AREA")
        light_data.shape = "RECTANGLE"
        light = bpy.data.objects.new(name, light_data)
        target.objects.link(light)
    elif target not in light.users_collection:
        link_to_collection(light, target)
    light.data.size = max(1.0, lot_width * 0.9)   # the panel's X is the lot width
    light.data.size_y = max(1.0, lot_length * 0.9)
    light.data.energy = float(settings.light_energy)
    light.data.use_shadow = bool(settings.shadows)
    light.location = (0.0, 0.0, WALL_HEIGHT + CEILING_LIFT)
    light.rotation_euler = (0.0, 0.0, 0.0)  # an area lamp points down -Z
    for obj in list(target.objects):
        if obj.name.startswith(LIGHT_PREFIX) and obj is not light:
            bpy.data.objects.remove(obj, do_unlink=True)
    return [light]


# ---------------------------------------------------------------------------
# camera (the add-on's own OpenCV camera, mounted in the vehicle frame)
# ---------------------------------------------------------------------------
def preset_camera(name: str = "front") -> Dict:
    """One camera record of the bundled minibus calibration.

    The Drive Scene has no calibration of its own: it drives the same vehicle as
    the AVM Scene, so its camera is the same fisheye (K / D / output) at the same
    mount pose - that is what makes the footage comparable to the real car.
    """
    from ....core.scenes import avm_layout
    for record in avm_layout.cameras_from_preset(avm_layout.load_preset()):
        if record["name"] == name:
            return record
    return {}


def _apply_camera_pose(camera: bpy.types.Object, record: Dict) -> None:
    """Set the camera's **vehicle frame** mount pose (degrees in the record)."""
    camera.location = tuple(record.get("location", (0.0, 0.0, 0.0)))
    camera.rotation_mode = "XYZ"
    camera.rotation_euler = tuple(math.radians(value)
                                  for value in record.get("rotation", (0.0, 0.0, 0.0)))


def _ensure_camera(scene: bpy.types.Scene, target: bpy.types.Collection,
                   vehicle: bpy.types.Object, messages: List[str]) -> bpy.types.Object:
    """Create the front camera once and keep the user's later tweaks."""
    camera = bpy.data.objects.get(CAMERA_NAME)
    if camera is None:
        record = preset_camera("front")
        camera, _ = camera_factory.add_camera(
            scene, model="fisheye", preset=None, name=CAMERA_NAME,
            location=tuple(record.get("location", (0.0, 0.0, 0.0))))
        camera.name = CAMERA_NAME  # a lingering data-block could suffix it
        camera.data.name = CAMERA_NAME
        ok, apply_messages = camera_factory.configure_from_record(camera, record, scene)
        if not ok:
            messages.append("front camera: " + "; ".join(apply_messages))
        _apply_camera_pose(camera, record)
    if target not in camera.users_collection:
        link_to_collection(camera, target)
    _parent_local(camera, vehicle)
    return camera


# ---------------------------------------------------------------------------
# the drive
# ---------------------------------------------------------------------------
#: object name / prefix -> the setting that shows it (one place to rule them all:
#: the panels only flip the flags, this table does the work). The ceiling panel is
#: deliberately not here: hiding the walls to look inside the lot must not darken it.
VISIBILITY = (
    (GROUND_NAME, "show_ground"),
    (NUMBER_PREFIX, "show_ground"),   # the bay numbers are painted markings
    (WALL_PREFIX, "show_walls"),
    (PILLAR_PREFIX, "show_walls"),    # the columns belong to the structure
    (PARKED_PREFIX, "show_parked"),
    (CAR_NAME, "show_car"),
)


def apply_visibility(settings) -> None:
    """Show/hide the layers without a rebuild (both viewport and render)."""
    for prefix, attribute in VISIBILITY:
        visible = bool(getattr(settings, attribute, True))
        for obj in bpy.data.objects:
            if obj.name == prefix or obj.name.startswith(prefix):
                obj.hide_render = not visible
                obj.hide_set(not visible)


def apply_drive(scene: bpy.types.Scene, vehicle: bpy.types.Object, settings) -> None:
    """Key the vehicle along the motion plan: one key per frame, linear."""
    plan = settings.plan()
    if vehicle.animation_data is not None:
        vehicle.animation_data_clear()
    for frame in plan.frames:
        vehicle.location = (frame.x, frame.y, 0.0)
        vehicle.rotation_euler = (0.0, 0.0, math.radians(frame.yaw))
        vehicle.keyframe_insert("location", frame=frame.index)
        vehicle.keyframe_insert("rotation_euler", frame=frame.index)
    animation = vehicle.animation_data
    if animation is not None and animation.action is not None:
        for curve in compat.action_fcurves(animation.action):
            for key in curve.keyframe_points:
                key.interpolation = "LINEAR"
    scene.frame_start = plan.frames[0].index
    scene.frame_end = plan.frames[-1].index
    scene.render.fps = max(1, int(round(settings.drive_fps)))
    scene.render.fps_base = 1.0
    scene.frame_set(scene.frame_start)


def _ensure_render_setup(scene: bpy.types.Scene, messages: List[str]) -> None:
    """Cycles + Standard view transform, like every other scene of the add-on.

    The custom OSL camera only exists in Cycles, and AgX would tone-map the
    colours the algorithm is supposed to see.
    """
    if scene.render.engine != "CYCLES":
        scene.render.engine = "CYCLES"
        messages.append("switched the render engine to Cycles (custom cameras need it)")
    if scene.view_settings.view_transform != "Standard":
        scene.view_settings.view_transform = "Standard"
        messages.append("set the view transform to Standard")
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.use_nodes = True
    background = compat.node_of_type(scene.world.node_tree, "BACKGROUND")
    if background is not None:
        # a dim ambient: the car park is lit by its own ceiling lamps
        background.inputs[0].default_value = (0.045, 0.047, 0.05, 1.0)
        background.inputs[1].default_value = 1.0


def view_targets(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    """The default view subject: the vehicle and its camera.

    Not the whole slab: a 48 m car park would shrink the car to a few percent of
    the viewport, and the interesting part of this scene is the vehicle.
    """
    return [obj for obj in (bpy.data.objects.get(CAR_NAME),
                            bpy.data.objects.get(CAMERA_NAME))
            if obj is not None]


# ---------------------------------------------------------------------------
# build / rebuild
# ---------------------------------------------------------------------------
def build(scene: bpy.types.Scene, settings) -> Dict:
    """Create the scene objects and return them."""
    created = rebuild(scene, settings)
    messages = created.get("messages") or []
    if settings.root is None:
        messages.append("the scene could not be created")
    return created


def rebuild(scene: bpy.types.Scene, settings) -> Dict:
    """Update every object in place; creates anything that is missing."""
    messages: List[str] = []
    _ensure_render_setup(scene, messages)

    target = collection(COLLECTION_NAME, scene, create=True)
    root = _ensure_root(scene, target)
    settings.root = root
    vehicle = _ensure_vehicle(root, target)

    lot_length, lot_width = settings.lot_size()
    plan = settings.plan()
    keep_clear_y = max(-lot_length / 2.0 + 1.0,
                       plan.frames[-1].y + 1.5 if plan.frames else 0.0)

    # floor + markings -----------------------------------------------------
    ground = _ensure_mesh_object(GROUND_NAME, target)
    _assign_mesh(ground, _lot_mesh("DRIVE_Ground", lot_length, lot_width,
                                   settings.aisle_width, settings.bay_depth,
                                   settings.bay_width, settings.show_bays,
                                   keep_clear_y), _lot_materials(settings))
    ground.location = (0.0, 0.0, 0.0)
    _parent_local(ground, root)
    ground.hide_render = not settings.show_ground

    # walls and pillars ----------------------------------------------------
    structure_material = _principled("DRIVE_Structure_Mat", (0.52, 0.53, 0.55, 1.0), 0.85)
    walls = _ensure_walls(target, lot_length, lot_width, structure_material)
    pillars = _ensure_pillars(target, lot_length, settings.aisle_width,
                              int(settings.pillar_count), structure_material)
    for obj in walls + pillars:
        _parent_local(obj, root)

    # bays: the numbers in the aisle and the parked cars (the same layout the
    # painted dividers come from)
    lot_bays = drive_lot.bays(lot_length, settings.aisle_width, settings.bay_depth,
                              settings.bay_width)
    numbers = _ensure_numbers(target, settings, lot_bays)
    for number in numbers:
        _parent_local(number, root)
    parked = _ensure_parked_cars(
        target, root, settings, lot_bays,
        drive_lot.pillar_positions(lot_length, settings.aisle_width,
                                   int(settings.pillar_count)))

    # the vehicle and its camera ------------------------------------------
    car = _ensure_mesh_object(CAR_NAME, target)
    _assign_mesh(car, _car_mesh("DRIVE_Car", settings.car_length, settings.car_width,
                                settings.car_height), _car_materials())
    car.location = (0.0, 0.0, settings.car_clearance)
    _parent_local(car, vehicle)
    car.hide_render = not settings.show_car

    camera = _ensure_camera(scene, target, vehicle, messages)
    _parent_local(camera, vehicle)
    scene.camera = camera

    lights = _ensure_lights(scene, target, lot_length, lot_width, settings)
    for light in lights:
        _parent_local(light, root)

    apply_drive(scene, vehicle, settings)
    settings.revision += 1
    apply_visibility(settings)
    return {
        "root": root,
        "vehicle": vehicle,
        "ground": ground,
        "walls": walls,
        "pillars": pillars,
        "numbers": numbers,
        "parked": parked,
        "bays": lot_bays,
        "car": car,
        "camera": camera,
        "lights": lights,
        "plan": plan,
        "messages": messages,
    }


def remove(scene: bpy.types.Scene, settings) -> int:
    """Delete the whole Drive Scene and clear the root pointer."""
    removed = remove_collection_objects(COLLECTION_NAME)
    root = settings.root
    if root is not None and root.name in bpy.data.objects:
        bpy.data.objects.remove(root, do_unlink=True)
        removed += 1
    settings.root = None
    camera = bpy.data.objects.get(CAMERA_NAME)
    if camera is not None:
        bpy.data.objects.remove(camera, do_unlink=True)
        removed += 1
    target = bpy.data.collections.get(COLLECTION_NAME)
    if target is not None and not target.objects and not target.children:
        bpy.data.collections.remove(target)
    return removed
