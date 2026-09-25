"""Small procedural meshes for the Road Scene's roadside props.

Every mesh is generated at real size (object scale stays 1) with its origin on
the ground, ``+Y`` forward - the same convention the Drive Scene's car uses, so
an object placed at a track pose needs no correction.  Faces carry a **material
slot index**; the builder owns the materials, so a tree and a parked car can
share one :func:`~addons.opencv_camera.bl.scenes.prop_mesh.tree_mesh` while each
object still gets its own paints.

Only bmesh primitives written here are used (no ``bmesh.ops`` beyond normal
recalculation): the ``create_*`` operators changed their keyword names across
Blender releases, and a prop mesh is not worth a version shim.
"""

from __future__ import annotations

import math

import bmesh
import bpy

__all__ = ["pedestrian_mesh", "tree_mesh", "lamp_mesh", "sign_mesh"]

#: pedestrian slots
PED_SKIN, PED_TORSO, PED_LEGS = range(3)
#: tree slots
TREE_TRUNK, TREE_LEAF = range(2)
#: lamp slots
LAMP_METAL, LAMP_GLASS = range(2)
#: sign slots
SIGN_POST, SIGN_BOARD = range(2)


def _add_quad(bm, points, material: int) -> None:
    verts = [bm.verts.new(position) for position in points]
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


def _add_cone(bm, centre, radius: float, height: float, segments: int,
              material: int) -> None:
    base = [bm.verts.new((centre[0] + radius * math.cos(2.0 * math.pi * step / segments),
                          centre[1] + radius * math.sin(2.0 * math.pi * step / segments),
                          centre[2]))
            for step in range(segments)]
    tip = bm.verts.new((centre[0], centre[1], centre[2] + height))
    for step in range(segments):
        nxt = (step + 1) % segments
        bm.faces.new((base[step], base[nxt], tip)).material_index = material
    bm.faces.new(list(reversed(base))).material_index = material


def _finish(mesh: bpy.types.Mesh, bm) -> bpy.types.Mesh:
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def pedestrian_mesh(name: str, height: float = 1.75,
                    slot: int = PED_SKIN) -> bpy.types.Mesh:
    """A simple standing person: head, torso, arms and legs (material slots)."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    height = max(0.8, float(height))
    skin, torso, legs = slot + 0, slot + 1, slot + 2
    head_r = height * 0.075
    neck = height * 0.80
    hip = height * 0.46
    _add_cylinder(bm, (0.0, 0.0, neck + head_r * 0.9), head_r, head_r, "Z", 12, skin)
    _add_box(bm, -0.13, 0.13, -0.10, 0.10, hip, neck, torso)
    for side in (-1.0, 1.0):
        _add_cylinder(bm, (side * 0.22, 0.0, hip + (neck - hip) * 0.55),
                      0.055, (neck - hip) * 0.48, "Z", 8, torso)
        _add_cylinder(bm, (side * 0.09, 0.0, hip * 0.5), 0.075, hip * 0.48, "Z", 8, legs)
        _add_box(bm, side * 0.09 - 0.08, side * 0.09 + 0.08, -0.12, 0.14, 0.0, 0.06, legs)
    return _finish(mesh, bm)


def tree_mesh(name: str, height: float = 4.0, slot: int = TREE_TRUNK) -> bpy.types.Mesh:
    """A trunk with a conical canopy."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    height = max(1.5, float(height))
    trunk_h = height * 0.42
    trunk_r = height * 0.045
    _add_cylinder(bm, (0.0, 0.0, trunk_h), trunk_r, trunk_h, "Z", 8, slot + 0)
    _add_cone(bm, (0.0, 0.0, trunk_h * 0.75), height * 0.24, height * 0.55, 10, slot + 1)
    return _finish(mesh, bm)


def lamp_mesh(name: str, height: float = 6.0, reach: float = 1.2,
              slot: int = LAMP_METAL) -> bpy.types.Mesh:
    """A street lamp: pole, arm and a head box."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    height = max(2.0, float(height))
    _add_box(bm, -0.07, 0.07, -0.07, 0.07, 0.0, height, slot + 0)
    _add_box(bm, -0.06, reach, -0.06, 0.06, height - 0.08, height, slot + 0)
    _add_box(bm, reach - 0.30, reach, -0.15, 0.15, height - 0.20, height - 0.06, slot + 1)
    return _finish(mesh, bm)


def sign_mesh(name: str, height: float = 2.2, slot: int = SIGN_POST) -> bpy.types.Mesh:
    """A roadside sign: a post and a board."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    height = max(1.0, float(height))
    _add_box(bm, -0.04, 0.04, -0.04, 0.04, 0.0, height, slot + 0)
    _add_box(bm, -0.42, 0.42, -0.03, 0.03, height - 0.55, height, slot + 1)
    return _finish(mesh, bm)
