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
    """The static shell of the Metro Elf: low-floor pod with a wrap-around
    glass band, black bumper/skirt, arched wheel openings and a crowned roof.

    The side walls stop at the door openings (the doors cover them when closed),
    so an open door really reveals the interior instead of a solid body side.
    """
    bm = bmesh.new()
    w2 = width / 2.0
    l2 = length / 2.0
    bottom = 0.20
    floor = radius * 0.9
    belt = 0.82
    glass_top = height - 0.14
    roof_side = height - 0.06
    roof_half = w2 - 0.10
    scale = length / vehicle.BODY_LENGTH_M
    spans = vehicle.door_spans(length)
    # door_spans returns (rear edge, front edge) in Blender +Y forward.
    r1_rear, r1_front = spans["r1"]
    r2_rear, r2_front = spans["r2"]
    three_row = style == vehicle.STYLE_3ROW
    r3_rear, r3_front = spans["r3"]
    skin = 0.07

    # Floor, roof and the structural rails.  The roof gets a chamfer strip on
    # each side so the silhouette reads as a rounded pod, not a box.
    _box(bm, -w2, w2, -l2, l2, bottom, bottom + 0.06, PAINT)
    _box(bm, -roof_half, roof_half, -l2 + 0.06, l2 - 0.06,
         roof_side, height, PAINT)
    for side in (-1.0, 1.0):
        outer = side * (w2 - 0.005)
        inner = side * roof_half
        _quad(bm, [
            (inner, -l2 + 0.06, roof_side),
            (outer, -l2 + 0.06, glass_top),
            (outer, l2 - 0.06, glass_top),
            (inner, l2 - 0.06, roof_side),
        ] if side > 0 else [
            (outer, -l2 + 0.06, glass_top),
            (inner, -l2 + 0.06, roof_side),
            (inner, l2 - 0.06, roof_side),
            (outer, l2 - 0.06, glass_top),
        ], PAINT)
    # Front and rear roof caps close the chamfer.
    _quad(bm, [(-roof_half, l2 - 0.06, roof_side), (roof_half, l2 - 0.06, roof_side),
               (w2 - 0.005, l2 - 0.06, glass_top), (-(w2 - 0.005), l2 - 0.06, glass_top)],
          PAINT)
    _quad(bm, [(-(w2 - 0.005), -l2 + 0.06, glass_top), (w2 - 0.005, -l2 + 0.06, glass_top),
               (roof_half, -l2 + 0.06, roof_side), (-roof_half, -l2 + 0.06, roof_side)],
          PAINT)

    # Front fascia below the windshield: low cyan nose, black bumper band and
    # the recess the light bar sits in.
    _box(bm, -w2, w2, l2 - 0.06, l2, bottom, belt, PAINT)
    _box(bm, -w2 + 0.03, w2 - 0.03, l2 - 0.03, l2 + 0.015, bottom, 0.40, BODY_GLASS)
    # Black centre grille with the light-bar recess above it; the cyan bumper
    # corners stay visible around it, as on the reference.
    _box(bm, -w2 * 0.55, w2 * 0.55, l2 - 0.02, l2 + 0.008, 0.42, 0.70, BODY_GLASS)
    _box(bm, -w2 + 0.05, w2 - 0.05, l2 - 0.02, l2 + 0.005, 0.71, 0.80, BODY_GLASS)
    # Rear fascia below the rear window.
    _box(bm, -w2, w2, -l2, -l2 + 0.06, bottom, belt, PAINT)
    _box(bm, -w2 + 0.03, w2 - 0.03, -l2 - 0.015, -l2 + 0.03, bottom, 0.58, BODY_GLASS)
    # Black skirt strip along both sides.
    for side in (-1.0, 1.0):
        x = side * (w2 + 0.002)
        _quad(bm, [(x, -l2 + 0.08, bottom), (x, l2 - 0.08, bottom),
                   (x, l2 - 0.08, 0.36), (x, -l2 + 0.08, 0.36)]
              if side > 0 else
              [(x, l2 - 0.08, bottom), (x, -l2 + 0.08, bottom),
               (x, -l2 + 0.08, 0.36), (x, l2 - 0.08, 0.36)], BODY_GLASS)

    # Side walls: lower panels between the doors and the upper pillars between
    # the glass bays.  The front/rear returns close the corners.
    lower = [(r1_front, l2 - 0.06)]
    pillars = [(r1_front - 0.05, r1_front + 0.05), (r2_front - 0.05, r2_front + 0.05),
               (r2_rear - 0.05, r2_rear + 0.05)]
    if three_row:
        lower += [(r2_rear, r3_front), (-l2 + 0.06, r3_rear)]
        pillars += [(r3_front - 0.05, r3_front + 0.05), (r3_rear - 0.05, r3_rear + 0.05)]
    else:
        lower.append((-l2 + 0.06, r2_rear))
    for side in (-1.0, 1.0):
        outer = side * w2
        inner = outer - side * skin
        for y0, y1 in lower:
            _box(bm, min(outer, inner), max(outer, inner),
                 min(y0, y1), max(y0, y1), bottom, belt, PAINT)
        for y0, y1 in pillars:
            _box(bm, min(outer, inner), max(outer, inner),
                 min(y0, y1), max(y0, y1), belt, glass_top, BODY_GLASS)

    # Glass band: one pane per bay between the doors/pillars, inset 3 mm.
    x_glass = w2 - 0.003
    bays = [(r1_front + 0.06, l2 - 0.09),
            (r2_front + 0.06, r1_rear - 0.06),
            (-l2 + 0.09, r2_rear - 0.06)]
    if three_row:
        bays.append((r3_front + 0.06, r2_rear - 0.06))
    for side in (-1.0, 1.0):
        x = side * x_glass
        for y0, y1 in bays:
            if y1 - y0 < 0.04:
                continue
            _quad(bm, [(x, y0, belt), (x, y1, belt), (x, y1, glass_top), (x, y0, glass_top)]
                  if side > 0 else
                  [(x, y1, belt), (x, y0, belt), (x, y0, glass_top), (x, y1, glass_top)],
                  BODY_GLASS)

    # Windshield: nearly vertical, raked 0.32 m over its height.
    _quad(bm, [
        (-(w2 - 0.07), l2 - 0.02, belt),
        (w2 - 0.07, l2 - 0.02, belt),
        (w2 - 0.09, l2 - 0.34, glass_top),
        (-(w2 - 0.09), l2 - 0.34, glass_top),
    ], BODY_GLASS)
    # Rear window, 8 cm narrower than the body.
    _quad(bm, [
        (-(w2 - 0.10), -l2 + 0.005, belt),
        (w2 - 0.10, -l2 + 0.005, belt),
        (w2 - 0.12, -l2 + 0.03, glass_top - 0.02),
        (-(w2 - 0.12), -l2 + 0.03, glass_top - 0.02),
    ], BODY_GLASS)

    # Wheel arches: a black flat ring on each side around the physical axles.
    front_y, rear_y = vehicle.axle_offsets(length)
    for side in (-1.0, 1.0):
        x = side * (w2 + 0.003)
        for cy in (front_y, -rear_y):
            segments = 10
            for i in range(segments):
                a0 = math.pi * i / segments
                a1 = math.pi * (i + 1) / segments
                outer, inner = radius + 0.10, radius + 0.035
                points = [
                    (x, cy + outer * math.cos(a0), radius + outer * math.sin(a0)),
                    (x, cy + outer * math.cos(a1), radius + outer * math.sin(a1)),
                    (x, cy + inner * math.cos(a1), radius + inner * math.sin(a1)),
                    (x, cy + inner * math.cos(a0), radius + inner * math.sin(a0)),
                ]
                _quad(bm, points if side > 0 else list(reversed(points)), BODY_GLASS)

    # Roof sensor blister, flush with the roof (the reference's front pod).
    _box(bm, -0.18, 0.18, l2 - 1.05, l2 - 0.60, height - 0.05, height, BODY_GLASS)
    return _finish(name, bm)


def _interior_mesh(name: str, length: float, width: float, height: float,
                   radius: float, style: str) -> bpy.types.Mesh:
    """Simple cabin furniture: floor mat, two front seats, bench rows, dash."""
    bm = bmesh.new()
    l2 = length / 2.0
    floor = radius * 0.9
    _box(bm, -width * 0.42, width * 0.42, -l2 + 0.35, l2 - 0.45,
         floor, floor + 0.04, PAINT)

    def seat(cx: float, cy: float) -> None:
        _box(bm, cx - 0.24, cx + 0.24, cy - 0.22, cy + 0.22,
             floor + 0.06, floor + 0.36, PAINT)
        _box(bm, cx - 0.24, cx + 0.24, cy - 0.24, cy - 0.12,
             floor + 0.36, floor + 0.92, PAINT)
        _box(bm, cx - 0.11, cx + 0.11, cy - 0.23, cy - 0.11,
             floor + 0.92, floor + 1.08, PAINT)

    def bench(cy: float) -> None:
        _box(bm, -0.78, 0.78, cy - 0.24, cy + 0.24,
             floor + 0.06, floor + 0.36, PAINT)
        _box(bm, -0.78, 0.78, cy - 0.26, cy - 0.14,
             floor + 0.36, floor + 0.98, PAINT)
        for x in (-0.48, 0.0, 0.48):
            _box(bm, x - 0.10, x + 0.10, cy - 0.24, cy - 0.12,
                 floor + 0.98, floor + 1.10, PAINT)

    seat(-0.52, l2 - 1.05)
    seat(0.52, l2 - 1.05)
    bench(0.45)
    bench(-0.95)
    if style == vehicle.STYLE_3ROW:
        bench(-1.90)

    # Boarding steps at the side doors: a light tread just inside the skin so
    # an open sliding leaf shows a readable entrance, not a black hole.
    spans = vehicle.door_spans(length)
    for key in ("r1", "r2"):
        y0, y1 = spans[key]
        for side in (-1.0, 1.0):
            x0, x1 = side * (width * 0.34), side * (width * 0.48)
            _box(bm, min(x0, x1), max(x0, x1), y0 + 0.06, y1 - 0.06,
                 0.18, 0.34, PAINT)

    # Dashboard + centre console.
    _box(bm, -width * 0.45, width * 0.45, l2 - 0.45, l2 - 0.30,
         0.92, 1.18, PAINT)
    _box(bm, -0.16, 0.16, l2 - 1.10, l2 - 0.45, floor + 0.30, floor + 0.58, PAINT)
    _ring(bm, (-0.52, l2 - 0.62, 1.00), 0.17, 0.13, 0.025, 16, PAINT)
    _cylinder(bm, (-0.52, l2 - 0.50, 1.00), 0.03, 0.10, "Y", 8, PAINT)
    return _finish(name, bm)


def _hood_mesh(name: str, length: float, width: float, height: float,
               radius: float, pivot) -> bpy.types.Mesh:
    """The front service flap, flush with the nose below the light bar."""
    bm = bmesh.new()
    l2 = length / 2.0
    _box(bm, -width * 0.30, width * 0.30, l2 - 0.72, l2 - 0.06,
         0.82, 0.85, PAINT)
    _offset(bm, pivot)
    return _finish(name, bm)


def _door_mesh(name: str, length: float, width: float, height: float,
               radius: float, y0: float, y1: float, side: str,
               pivot) -> bpy.types.Mesh:
    """A flush side door: paint below the beltline, one tall glass pane above."""
    bm = bmesh.new()
    thickness = 0.05
    sign = -1.0 if side == "l" else 1.0
    # 1 cm proud of the body skin: a sliding leaf runs outside the body when
    # it opens, so it must not z-fight with the rear quarter it passes over.
    x_outer = sign * (width / 2.0 + 0.01)
    x_inner = x_outer - sign * thickness
    bottom = 0.20
    belt = 0.82
    glass_top = height - 0.14
    faces = _box(bm, min(x_outer, x_inner), max(x_outer, x_inner),
                 y0 + 0.008, y1 - 0.008, bottom, belt, PAINT)
    inner_face = faces[3] if side == "l" else faces[5]
    inner_face.material_index = TRIM
    # The upper half is glass edge to edge: the reference's side window band is
    # continuous, with only the door seams interrupting it.
    _box(bm, min(x_outer, x_inner), max(x_outer, x_inner),
         y0 + 0.008, y1 - 0.008, belt, glass_top, GLASS)
    _offset(bm, pivot)
    return _finish(name, bm)


def _tail_mesh(name: str, length: float, width: float, height: float,
               pivot) -> bpy.types.Mesh:
    """The rear door: paint below the beltline, one large rear window above."""
    bm = bmesh.new()
    l2 = length / 2.0
    bottom = 0.20
    belt = 0.82
    glass_top = height - 0.14
    faces = _box(bm, -width * 0.49, width * 0.49, -l2 - 0.02, -l2 + 0.03,
                 bottom, glass_top, PAINT)
    faces[4].material_index = TRIM  # +Y (inner) face
    _quad(bm, [
        (-width * 0.39, -l2 - 0.022, belt),
        (width * 0.39, -l2 - 0.022, belt),
        (width * 0.39, -l2 - 0.022, glass_top - 0.05),
        (-width * 0.39, -l2 - 0.022, glass_top - 0.05),
    ], GLASS)
    _offset(bm, pivot)
    return _finish(name, bm)


def _mirror_mesh(name: str, side: str) -> bpy.types.Mesh:
    """A long stalk with a vertical paddle housing (the reference's mirrors)."""
    bm = bmesh.new()
    sign = -1.0 if side == "l" else 1.0
    x0, x1 = sign * 1.02, sign * 1.34
    _box(bm, min(x0, x1), max(x0, x1), 1.92, 1.97, 1.94, 1.98, PAINT)
    px0, px1 = sign * 1.28, sign * 1.40
    _box(bm, min(px0, px1), max(px0, px1), 1.55, 1.80, 1.78, 2.10, PAINT)
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
    l2 = length / 2.0
    sign = -1.0 if side == "l" else 1.0
    if front:
        # A horizontal light-bar segment on each side of the nose.
        centre = sign * width * 0.27
        half = width * 0.17
        _box(bm, centre - half, centre + half, l2 + 0.012, l2 + 0.045,
             0.725, 0.785, PAINT)
    else:
        # Vertical C-shape at the rear corners: outer bar plus two short arms
        # pointing inward, which is what reads as "tail lamp" from behind.
        inner = sign * (width * 0.34)
        outer = sign * (width * 0.46)
        _box(bm, min(inner, outer), max(inner, outer), -l2 - 0.045, -l2 - 0.012,
             0.86, 1.54, PAINT)
        _box(bm, min(inner, outer), max(inner, outer), -l2 - 0.045, -l2 - 0.012,
             1.46, 1.54, PAINT)
        _box(bm, min(inner, outer), max(inner, outer), -l2 - 0.045, -l2 - 0.012,
             0.86, 0.94, PAINT)
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
    if part_id.startswith("door_r"):
        side = part_id[-1]
        key = part_id[len("door_"):len("door_r") + 1]
        y0, y1 = vehicle.door_spans(length)[key]
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
