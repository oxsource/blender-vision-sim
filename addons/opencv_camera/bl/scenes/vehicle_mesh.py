"""The ego vehicle's mesh - one source for every driving scene.

The Drive Scene and the Road Scene must render the *same* minibus at the *same*
size, because both feed the same transparent-chassis pipeline: a reconstruction
verified on one scene has to be valid on the other.  The mesh used to live in
``drive_scene/builder.py``; it now lives here so the Road Scene's ego car and its
parked cars are the identical geometry instead of a look-alike.

Generated at real size (object scale stays 1), origin on the ground under the
body centre, ``+Y`` = front, with faces tagged by material slot
(:data:`CAR_BODY` / :data:`CAR_GLASS` / :data:`CAR_TIRE` / :data:`CAR_LAMP`).
"""

from __future__ import annotations

import math

import bmesh
import bpy

from ...core.scenes import vehicle

__all__ = ["car_mesh", "CAR_BODY", "CAR_GLASS", "CAR_TIRE", "CAR_LAMP"]

#: material slot indices of the car mesh
CAR_BODY, CAR_GLASS, CAR_TIRE, CAR_LAMP = range(4)


def _add_quad(bm, p0, p1, p2, p3, material: int) -> None:
    verts = [bm.verts.new(position) for position in (p0, p1, p2, p3)]
    bm.faces.new(verts).material_index = material


def _add_box(bm, x0, x1, y0, y1, z0, z1, material: int) -> None:
    verts = [bm.verts.new(position) for position in (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
    for face in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        bm.faces.new([verts[index] for index in face]).material_index = material


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


def car_mesh(name: str, length: float, width: float, height: float) -> bpy.types.Mesh:
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
            _add_cylinder(bm, (side * (half_w - 0.02), end * length * vehicle.AXLE_FRACTION,
                               wheel_radius),
                          wheel_radius, 0.13, "X", 16, CAR_TIRE)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh
