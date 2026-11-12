"""Part meshes for the AVM ego car (M3 skeleton).

The scene keeps one object per ``part_*`` and one empty per ``pivot_*`` so the
exported GLB matches the M2 contract (``docs/specs/011_vehicle_parts/model_spec.md``).
Every mesh is generated at its real size with the object transform at the pivot
(or at the vehicle origin for static parts).  Materials are unlit (a Background
node, which is what makes Blender write ``KHR_materials_unlit``) and named after
the contract slots.

This is the *skeleton*: proportions are deliberately simple boxes/cylinders.
M3 refines the silhouette, seams and UVs; the hierarchy, pivots and slot names
are already the frozen contract.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import bmesh
import bpy

from ....core.scenes import vehicle

#: the material of each part, in mesh material-index order.  ``glass`` is a
#: fixed material (not a variant slot), exactly as the contract says.
PART_MATERIALS = {
    "body": ("paint_body", "glass"),
    "interior": ("trim_door_inner",),
    "hood": ("paint_body",),
    "door": ("paint_door", "trim_door_inner", "glass"),
    "wheel": ("tire", "rim"),
    "mirror": ("paint_mirror",),
    "lamp_head": ("lamp_lens_front",),
    "lamp_tail": ("lamp_lens_rear",),
}

#: material index group of each part kind
PAINT, TRIM, GLASS, RIM = 0, 1, 2, 1
#: the body mesh has only two slots (paint, glass)
BODY_GLASS = 1


def _color_map():
    """Slot -> RGBA from the default variant plus the fixed materials."""
    appearance = vehicle.APPEARANCE
    default_id = appearance["default"]
    variant = next(item for item in appearance["variants"] if item["id"] == default_id)
    colors = {slot: tuple(entry["base_color"])
              for slot, entry in variant["materials"].items()}
    colors.update({slot: tuple(entry["base_color"])
                   for slot, entry in appearance["fixed"].items()})
    return colors


def material_names(part_id: str) -> Tuple[str, ...]:
    """The material slots a part mesh uses, in index order."""
    if part_id == "body":
        return PART_MATERIALS["body"]
    if part_id == "interior":
        return PART_MATERIALS["interior"]
    if part_id == "hood":
        return PART_MATERIALS["hood"]
    if part_id.startswith("door_"):
        return PART_MATERIALS["door"]
    if part_id.startswith("wheel_"):
        return PART_MATERIALS["wheel"]
    if part_id.startswith("mirror_"):
        return PART_MATERIALS["mirror"]
    if part_id.startswith("lamp_head_"):
        return PART_MATERIALS["lamp_head"]
    if part_id.startswith("lamp_tail_"):
        return PART_MATERIALS["lamp_tail"]
    raise KeyError(f"unknown part {part_id!r}")


def _unlit(name: str, color: Sequence[float]):
    """A Background material: Blender exports it as ``KHR_materials_unlit``."""
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    background = nodes.new("ShaderNodeBackground")
    background.inputs["Color"].default_value = tuple(color)
    background.inputs["Strength"].default_value = 1.0
    material.node_tree.links.new(background.outputs["Background"], output.inputs["Surface"])
    material.diffuse_color = tuple(color)
    material.use_backface_culling = False
    return material


def materials_for(names: Sequence[str]) -> List[bpy.types.Material]:
    colors = _color_map()
    return [_unlit(name, colors.get(name, (0.5, 0.5, 0.5, 1.0))) for name in names]


def _box(bm, x0, x1, y0, y1, z0, z1, material: int):
    """A closed box; returns its six faces in a fixed order (see below)."""
    verts = [bm.verts.new(p) for p in (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
    faces = []
    for indices in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                    (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        face = bm.faces.new([verts[i] for i in indices])
        face.material_index = material
        faces.append(face)
    # 0 bottom, 1 top, 2 -Y, 3 +X, 4 +Y, 5 -X
    return faces


def _quad(bm, points, material: int):
    verts = [bm.verts.new(point) for point in points]
    face = bm.faces.new(verts)
    face.material_index = material
    return face


def _prism(bm, bottom, top, material: int):
    lower = [bm.verts.new(point) for point in bottom]
    upper = [bm.verts.new(point) for point in top]
    faces = [bm.faces.new(lower), bm.faces.new(upper)]
    for index in range(4):
        nxt = (index + 1) % 4
        faces.append(bm.faces.new((lower[index], lower[nxt], upper[nxt], upper[index])))
    for face in faces:
        face.material_index = material
    return faces


def _cylinder(bm, centre, radius: float, half_len: float, axis: str,
              segments: int, material: int):
    other = {"X": (1, 2), "Y": (0, 2), "Z": (0, 1)}[axis]
    index = {"X": 0, "Y": 1, "Z": 2}[axis]
    rings = []
    for side in (-half_len, half_len):
        ring = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            position = list(centre)
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


def _ring(bm, centre, outer: float, inner: float, half_len: float,
          segments: int, material: int) -> None:
    """A steering-wheel-like ring in the XZ plane, thickness along Y."""
    cx, cy, cz = centre
    rings = []
    for y in (cy - half_len, cy + half_len):
        outer_ring, inner_ring = [], []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            outer_ring.append(bm.verts.new(
                (cx + outer * math.cos(angle), y, cz + outer * math.sin(angle))))
            inner_ring.append(bm.verts.new(
                (cx + inner * math.cos(angle), y, cz + inner * math.sin(angle))))
        rings.append((outer_ring, inner_ring))
    (front_outer, front_inner), (back_outer, back_inner) = rings
    for i in range(segments):
        nxt = (i + 1) % segments
        bm.faces.new((front_outer[i], front_outer[nxt],
                      back_outer[nxt], back_outer[i])).material_index = material
        bm.faces.new((front_inner[nxt], front_inner[i],
                      back_inner[i], back_inner[nxt])).material_index = material
        bm.faces.new((front_outer[i], front_inner[i],
                      front_inner[nxt], front_outer[nxt])).material_index = material
        bm.faces.new((back_outer[nxt], back_inner[nxt],
                      back_inner[i], back_outer[i])).material_index = material


def _offset(bm, pivot) -> None:
    if any(abs(value) > 1e-9 for value in pivot):
        bmesh.ops.translate(bm, verts=bm.verts, vec=tuple(-value for value in pivot))


def _finish(name: str, bm: bmesh.types.BMesh) -> bpy.types.Mesh:
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    # A tiny per-face atlas: every face gets its own cell, projected along its
    # dominant axis.  It is deliberately simple (the M3 skeleton has no
    # textures yet) but it is a non-overlapping 0..1 UV map, so a later texture
    # pass does not have to change the mesh structure.
    uv_layer = mesh.uv_layers.new(name="UVMap")
    count = max(1, len(mesh.polygons))
    grid = int(math.ceil(math.sqrt(count)))
    cell = 1.0 / grid
    pad = cell * 0.02
    for index, polygon in enumerate(mesh.polygons):
        axis = max(range(3), key=lambda i: abs(polygon.normal[i]))
        if axis == 0:
            a, b = 1, 2
        elif axis == 1:
            a, b = 0, 2
        else:
            a, b = 0, 1
        coords = [mesh.vertices[mesh.loops[loop].vertex_index].co
                  for loop in polygon.loop_indices]
        min_a = min(co[a] for co in coords)
        max_a = max(co[a] for co in coords)
        min_b = min(co[b] for co in coords)
        max_b = max(co[b] for co in coords)
        span_a = (max_a - min_a) or 1.0
        span_b = (max_b - min_b) or 1.0
        column, row = index % grid, index // grid
        left, bottom = column * cell, row * cell
        for loop, co in zip(polygon.loop_indices, coords):
            u = (co[a] - min_a) / span_a
            v = (co[b] - min_b) / span_b
            uv_layer.data[loop].uv = (
                left + pad + u * (cell - 2.0 * pad),
                bottom + pad + v * (cell - 2.0 * pad),
            )
    return mesh


def _body_mesh(name: str, length: float, width: float, height: float,
               radius: float, style: str) -> bpy.types.Mesh:
    """The static shell: floor, door-cut side walls, pillars and the roof.

    The side walls stop at the door openings (the doors cover them when closed),
    so an open door really reveals the interior instead of a solid body side.
    """
    bm = bmesh.new()
    floor = radius * 0.7
    beltline = floor + (height - floor) * 0.55
    scale = length / vehicle.BODY_LENGTH_M
    cabin_half = width * 0.47
    cabin_front = length * 0.46
    cabin_back = -length * 0.47
    rake = min(0.45, length * 0.09)
    rear_rake = 0.12

    # door spans (Blender +Y = forward), matching the pivot cards
    r1_front, r1_rear = 1.55 * scale, 0.35 * scale
    r2_front, r2_rear = 0.25 * scale, -0.95 * scale
    r3_front, r3_rear = -0.95 * scale, -1.90 * scale
    three_row = style == vehicle.STYLE_3ROW

    _box(bm, -width / 2, width / 2, -length / 2, length / 2,
         floor, floor + 0.06, PAINT)
    # front fascia: the head lamps mount on this panel instead of floating
    _box(bm, -width / 2, width / 2, length / 2 - 0.06, length / 2,
         floor, beltline, PAINT)
    # front / rear bumpers (still part of the static body shell)
    _box(bm, -width * 0.49, width * 0.49, length / 2 - 0.10, length / 2 + 0.03,
         floor, floor + 0.26, PAINT)
    _box(bm, -width * 0.49, width * 0.49, -length / 2 - 0.03, -length / 2 + 0.10,
         floor, floor + 0.26, PAINT)
    # head-lamp housings, proud of the fascia
    lamp_half = width * 0.145
    for side in (-1.0, 1.0):
        centre = side * width * 0.30
        _box(bm, centre - lamp_half, centre + lamp_half,
             length / 2 - 0.03, length / 2 + 0.005,
             height * 0.34, height * 0.42, PAINT)

    lower_walls = [(r1_front, length / 2), (r1_rear, r2_front)]
    upper_pillars = [(r1_front, length / 2), (r1_front - 0.05, r1_front),
                     (r2_front, r2_front + 0.05)]
    if three_row:
        lower_walls += [(r2_rear, r3_front + 0.05), (-length / 2, r3_rear)]
        upper_pillars += [(r3_front, r3_front + 0.05), (-length / 2, r3_rear)]
    else:
        lower_walls.append((-length / 2, r2_rear))
        upper_pillars.append((-length / 2, r2_rear))
    for side in (-1.0, 1.0):
        outer = side * width / 2
        inner = outer - side * 0.07
        for y0, y1 in lower_walls:
            _box(bm, min(outer, inner), max(outer, inner),
                 min(y0, y1), max(y0, y1), floor, beltline, PAINT)
        outer = side * cabin_half
        inner = outer - side * 0.06
        for y0, y1 in upper_pillars:
            _box(bm, min(outer, inner), max(outer, inner),
                 min(y0, y1), max(y0, y1), beltline, height - 0.06, PAINT)
    # firewall between the front seats and the bonnet
    _box(bm, -cabin_half, cabin_half, r1_front, r1_front + 0.06,
         floor, beltline, PAINT)
    # roof
    _box(bm, -width / 2, width / 2, cabin_back, cabin_front,
         height - 0.06, height, PAINT)

    rise = height - beltline
    normal = math.hypot(rise, rake) or 1.0
    offset_y, offset_z = rise / normal * 0.015, rake / normal * 0.015
    glass_half = cabin_half - 0.07
    _quad(bm, [
        (-glass_half, cabin_front + offset_y, beltline + offset_z),
        (glass_half, cabin_front + offset_y, beltline + offset_z),
        (glass_half, cabin_front - rake + offset_y, height + offset_z),
        (-glass_half, cabin_front - rake + offset_y, height + offset_z),
    ], BODY_GLASS)
    _quad(bm, [
        (-glass_half, cabin_back - 0.015, beltline + 0.20),
        (glass_half, cabin_back - 0.015, beltline + 0.20),
        (glass_half, cabin_back + rear_rake - 0.015, height - 0.14),
        (-glass_half, cabin_back + rear_rake - 0.015, height - 0.14),
    ], BODY_GLASS)
    return _finish(name, bm)


def _interior_mesh(name: str, length: float, width: float, height: float,
                   radius: float, style: str) -> bpy.types.Mesh:
    """Simple cabin furniture: floor mat, seats, dash, console, wheel."""
    bm = bmesh.new()
    floor = radius * 0.7
    beltline = floor + (height - floor) * 0.55
    cabin_half = width * 0.47

    _box(bm, -width * 0.42, width * 0.42, -length * 0.38, length * 0.36,
         floor + 0.06, floor + 0.08, PAINT)

    def seat(cx: float, cy: float) -> None:
        _box(bm, cx - 0.25, cx + 0.25, cy - 0.25, cy + 0.25,
             floor + 0.08, floor + 0.38, PAINT)
        _box(bm, cx - 0.25, cx + 0.25, cy - 0.25, cy - 0.15,
             floor + 0.38, floor + 0.92, PAINT)
        _box(bm, cx - 0.12, cx + 0.12, cy - 0.23, cy - 0.11,
             floor + 0.92, floor + 1.08, PAINT)

    seat(-0.55, 1.05)
    seat(0.55, 1.05)

    def bench(cy: float) -> None:
        _box(bm, -0.80, 0.80, cy - 0.25, cy + 0.25,
             floor + 0.08, floor + 0.38, PAINT)
        _box(bm, -0.80, 0.80, cy - 0.25, cy - 0.15,
             floor + 0.38, floor + 0.95, PAINT)
        for x in (-0.50, 0.0, 0.50):
            _box(bm, x - 0.10, x + 0.10, cy - 0.23, cy - 0.11,
                 floor + 0.95, floor + 1.08, PAINT)

    bench(-0.35)
    if style == vehicle.STYLE_3ROW:
        bench(-1.30)

    # dashboard + centre console
    _box(bm, -cabin_half + 0.05, cabin_half - 0.05, 1.38, 1.52,
         beltline - 0.32, beltline + 0.04, PAINT)
    _box(bm, -0.18, 0.18, 0.75, 1.38, floor + 0.35, floor + 0.62, PAINT)
    # steering wheel (left-hand drive) + column
    _ring(bm, (-0.55, 1.30, beltline - 0.16), 0.18, 0.14, 0.025, 16, PAINT)
    _cylinder(bm, (-0.55, 1.42, beltline - 0.16), 0.035, 0.10, "Y", 8, PAINT)
    return _finish(name, bm)


def _hood_mesh(name: str, length: float, width: float, height: float,
               radius: float, pivot) -> bpy.types.Mesh:
    bm = bmesh.new()
    floor = radius * 0.7
    beltline = floor + (height - floor) * 0.55
    # A bonnet surface on the beltline, hinged at its front edge.
    _box(bm, -width * 0.42, width * 0.42,
         length / 2 - 0.85, length / 2 - 0.02,
         beltline, beltline + 0.06, PAINT)
    _offset(bm, pivot)
    return _finish(name, bm)


def _door_mesh(name: str, length: float, width: float, height: float,
               radius: float, y0: float, y1: float, side: str,
               pivot) -> bpy.types.Mesh:
    """A side door: outer paint, inner trim and a tall glass window."""
    bm = bmesh.new()
    thickness = 0.05
    if side == "l":
        x_outer, x_inner = -width / 2, -width / 2 + thickness
    else:
        x_outer, x_inner = width / 2, width / 2 - thickness
    floor = radius * 0.7
    beltline = floor + (height - floor) * 0.55
    z0, z1 = 0.30 * (height / vehicle.BODY_HEIGHT_M), height - 0.02
    faces = _box(bm, min(x_outer, x_inner), max(x_outer, x_inner),
                 y0, y1, z0, z1, PAINT)
    inner_face = faces[3] if side == "l" else faces[5]
    inner_face.material_index = TRIM
    glass_x = x_outer + (-0.002 if side == "l" else 0.002)
    _quad(bm, [
        (glass_x, y0 + 0.12, beltline + 0.02),
        (glass_x, y1 - 0.12, beltline + 0.02),
        (glass_x, y1 - 0.12, z1 - 0.08),
        (glass_x, y0 + 0.12, z1 - 0.08),
    ], GLASS)
    # handle, just below the window
    handle_x = x_outer + (-0.03 if side == "l" else 0.03)
    _box(bm, min(x_outer, handle_x), max(x_outer, handle_x),
         y1 - 0.38, y1 - 0.20, beltline - 0.24, beltline - 0.18, PAINT)
    _offset(bm, pivot)
    return _finish(name, bm)


def _tail_mesh(name: str, length: float, width: float, height: float,
               pivot) -> bpy.types.Mesh:
    bm = bmesh.new()
    z0, z1 = height * 0.28, height * 0.73
    y0, y1 = -length / 2 - 0.03, -length / 2 + 0.03
    faces = _box(bm, -width * 0.42, width * 0.42, y0, y1, z0, z1, PAINT)
    faces[4].material_index = TRIM  # +Y (inner) face
    _quad(bm, [
        (-width * 0.34, y0 - 0.002, z0 + 0.35),
        (width * 0.34, y0 - 0.002, z0 + 0.35),
        (width * 0.34, y0 - 0.002, z1 - 0.08),
        (-width * 0.34, y0 - 0.002, z1 - 0.08),
    ], GLASS)
    _offset(bm, pivot)
    return _finish(name, bm)


def _wheel_mesh(name: str, radius: float) -> bpy.types.Mesh:
    bm = bmesh.new()
    _cylinder(bm, (0.0, 0.0, 0.0), radius, 0.13, "X", 20, PAINT)
    _cylinder(bm, (0.0, 0.0, 0.0), radius * 0.58, 0.14, "X", 20, RIM)
    return _finish(name, bm)


def _mirror_mesh(name: str, side: str) -> bpy.types.Mesh:
    bm = bmesh.new()
    sign = -1.0 if side == "l" else 1.0
    # arm from the pivot to the housing, then the housing itself
    _box(bm, min(0.0, sign * 0.07), max(0.0, sign * 0.07),
         -0.025, 0.025, -0.025, 0.025, PAINT)
    if side == "l":
        x0, x1 = -0.18, -0.06
    else:
        x0, x1 = 0.06, 0.18
    _box(bm, x0, x1, -0.06, 0.06, -0.09, 0.09, PAINT)
    return _finish(name, bm)


def _lamp_mesh(name: str, length: float, width: float, height: float,
               front: bool, side: str, pivot=None) -> bpy.types.Mesh:
    bm = bmesh.new()
    half = width * 0.13
    centre = (-1.0 if side == "l" else 1.0) * width * 0.30
    if front:
        y0, y1 = length / 2 - 0.01, length / 2 + 0.02
        z0, z1 = height * 0.35, height * 0.41
    else:
        y0, y1 = -length / 2 - 0.02, -length / 2 + 0.01
        z0, z1 = height * 0.42, height * 0.52
    _box(bm, centre - half, centre + half, y0, y1, z0, z1, PAINT)
    if pivot is not None:
        _offset(bm, pivot)
    return _finish(name, bm)


def build(part: dict, length: float, width: float, height: float,
          radius: float, style: str = vehicle.STYLE_2ROW,
          parent_pos=None) -> bpy.types.Mesh:
    """Build the mesh for one part card.

    A moving part uses its own pivot (``motion.pivot_pos``); a static part
    mounted on another pivot (the tail lamps on the tailgate) receives that
    parent's position in ``parent_pos`` so its vertices are local to it.
    """
    part_id = part["id"]
    name = part["node"]
    motion = part.get("motion")
    pivot = vehicle.to_blender(motion["pivot_pos"]) if motion else (0.0, 0.0, 0.0)

    if part_id == "body":
        return _body_mesh(name, length, width, height, radius, style)
    if part_id == "interior":
        return _interior_mesh(name, length, width, height, radius, style)
    if part_id == "hood":
        return _hood_mesh(name, length, width, height, radius, pivot)
    if part_id.startswith("door_r1"):
        side = part_id[-1]
        y0, y1 = 0.35 * length / vehicle.BODY_LENGTH_M, 1.55 * length / vehicle.BODY_LENGTH_M
        return _door_mesh(name, length, width, height, radius, y0, y1, side, pivot)
    if part_id.startswith("door_r2"):
        side = part_id[-1]
        y0, y1 = -0.95 * length / vehicle.BODY_LENGTH_M, 0.25 * length / vehicle.BODY_LENGTH_M
        return _door_mesh(name, length, width, height, radius, y0, y1, side, pivot)
    if part_id.startswith("door_r3"):
        side = part_id[-1]
        y0, y1 = -1.90 * length / vehicle.BODY_LENGTH_M, -0.95 * length / vehicle.BODY_LENGTH_M
        return _door_mesh(name, length, width, height, radius, y0, y1, side, pivot)
    if part_id == "door_tail":
        return _tail_mesh(name, length, width, height, pivot)
    if part_id.startswith("wheel_"):
        return _wheel_mesh(name, radius)
    if part_id.startswith("mirror_"):
        return _mirror_mesh(name, part_id[-1])
    if part_id.startswith("lamp_head_"):
        return _lamp_mesh(name, length, width, height, True, part_id[-1])
    if part_id.startswith("lamp_tail_"):
        return _lamp_mesh(name, length, width, height, False, part_id[-1],
                          parent_pos)
    raise KeyError(f"unknown part {part_id!r}")
