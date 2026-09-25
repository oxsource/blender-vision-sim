"""Road Scene builder: the closed test track, its props, the vehicle, the cameras.

The road is a **ribbon along the track centreline** (:mod:`core.scenes.road_track`)
with lane markings, shoulders and grass embankments down to a base plane.  Because
the ribbon follows the centreline's ``z`` and ``pitch``, an up/down slope is real
geometry, not a camera trick - which is exactly what the transparent-chassis M5
slope cases have to be measured on.

The scene is otherwise the Drive Scene's twin: the same vehicle empty carrying
the car and the four OpenCV cameras, the same per-frame keyframing from a pure
Python plan.  Objects are updated **in place** so a slider drag never destroys the
user's selection, and every mesh is generated at real size (object scale 1).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import bmesh
import bpy
from mathutils import Matrix

from ....core.scenes import avm_cameras, avm_layout, drive_lot, road_track, vehicle
from ... import apply as apply_mod, camera_factory, compat
from .. import prop_mesh
from ..base import collection, link_to_collection, remove_collection_objects
#: the ego car (and the parked ones) is the Drive Scene's minibus mesh, so the
#: two scenes render the same vehicle at the same size
from ..vehicle_mesh import car_mesh as _car_mesh

ROOT_NAME = "ROAD_Root"
VEHICLE_NAME = "ROAD_Vehicle"
COLLECTION_NAME = "Road Scene"

GROUND_NAME = "ROAD_Ground"
CAR_NAME = "ROAD_Car"
CAMERA_PREFIX = "ROAD_Cam_"
TREE_PREFIX = "ROAD_Tree_"
LAMP_PREFIX = "ROAD_Lamp_"
PED_PREFIX = "ROAD_Ped_"
BAY_CAR_PREFIX = "ROAD_BayCar_"
SIGN_PREFIX = "ROAD_Sign_"
LIGHT_PREFIX = "ROAD_Light_"

#: how finely the ribbon follows the centreline [m]
MESH_STEP = 1.0
#: lane marking width and where the edge lines sit inside the road [m]
LINE_WIDTH = 0.12
EDGE_INSET = 0.30
#: a marking floats this far above the road so the two never z-fight [m]
MARK_Z = 0.004
#: dashed centre line: 3 m of paint inside a 6 m period
DASH_ON = 3.0
DASH_PERIOD = 6.0
#: zebra crossing geometry lives in core (``road_track``), so the paint here and
#: the speed field that slows the car for it read the same rule
CROSSWALK_DEPTH = road_track.CROSSWALK_DEPTH
CROSSWALK_BAR = road_track.CROSSWALK_BAR
CROSSWALK_GAP = road_track.CROSSWALK_GAP
CROSSWALK_MARGIN = road_track.CROSSWALK_MARGIN
#: the bars float a hair above the other markings, so the centre line cannot
#: z-fight through the crossing
CROSSWALK_LIFT = 0.002
#: the ground under everything; the road is slightly above it, so a flat road
#: section cannot z-fight with its own base plane
BASE_Z = -0.02
#: an embankment rises outwards this many metres per metre of height
BANK_SLOPE = 1.5
#: how far off the shoulder a tree stands [m]
TREE_CLEAR = 1.5

#: material slot indices of the track mesh
G_ROAD, G_SHOULDER, G_GRASS, G_WHITE, G_YELLOW = range(5)


# ---------------------------------------------------------------------------
# small mesh helpers
# ---------------------------------------------------------------------------
def _face(bm, points, material: int) -> None:
    verts = [bm.verts.new(position) for position in points]
    bm.faces.new(verts).material_index = material


def _quad(bm, a, b, c, d, material: int) -> None:
    """A quad wound counter-clockwise seen from above (normal up)."""
    _face(bm, (a, b, c, d), material)


def _box(bm, x0, x1, y0, y1, z0, z1, material: int) -> None:
    verts = [bm.verts.new(position) for position in (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
    for face in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        bm.faces.new([verts[index] for index in face]).material_index = material


def _right(pose: road_track.Pose) -> Tuple[float, float]:
    angle = math.radians(pose.yaw)
    return (math.cos(angle), math.sin(angle))


def _right3d(pose: road_track.Pose) -> Tuple[float, float, float]:
    yaw, roll = math.radians(pose.yaw), math.radians(pose.roll)
    return (math.cos(yaw) * math.cos(roll),
            math.sin(yaw) * math.cos(roll), -math.sin(roll))


def _offset_point(pose: road_track.Pose, lateral: float, z: Optional[float] = None):
    right = _right3d(pose)
    centre_z = pose.z if z is None else z
    return (pose.x + right[0] * lateral, pose.y + right[1] * lateral,
            centre_z + right[2] * lateral)


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------
def _principled(name: str, color, roughness: float = 0.7,
                emission: Optional[Tuple] = None) -> bpy.types.Material:
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
    if emission is not None:
        if "Emission Color" in principled.inputs:
            principled.inputs["Emission Color"].default_value = emission
            principled.inputs["Emission Strength"].default_value = emission[3]
        elif "Emission" in principled.inputs:
            principled.inputs["Emission"].default_value = emission
    material.node_tree.links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


def _mottled_material(name: str, color, roughness: float,
                      stains: Tuple[float, float, float, float],
                      grain: Tuple[float, float, float], grain_weight: float,
                      stops) -> bpy.types.Material:
    """A procedurally mottled surface: broad stains + a fine grain, no texture."""
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
    combine = nodes.new("ShaderNodeMath")
    combine.operation = "MULTIPLY_ADD"
    combine.inputs[1].default_value = 1.0 - grain_weight
    ramp = nodes.new("ShaderNodeValToRGB")

    def shade(level, offset):
        return tuple(min(1.0, max(0.0, value * level + shift))
                     for value, shift in zip(color[:3], offset)) + (1.0,)

    ramp.color_ramp.elements[0].position = stops[0][0]
    ramp.color_ramp.elements[0].color = shade(stops[0][1], stops[0][2])
    ramp.color_ramp.elements[-1].position = stops[-1][0]
    ramp.color_ramp.elements[-1].color = shade(stops[-1][1], stops[-1][2])
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


def _asphalt_material(name: str) -> bpy.types.Material:
    """Asphalt with a strong aggregate speckle: dark base, bright chips.

    The grain is weighted above the broad stains and the ramp reaches further
    both down (dark binder) and up (light aggregate), so the surface keeps a
    visible, per-patch texture at the BEV's distance - a flat near-black road
    leaves a transparent-chassis algorithm almost nothing to align against.
    """
    return _mottled_material(
        name, (0.085, 0.085, 0.09, 1.0), 0.92,
        stains=(2.4, 5.0, 0.55, 1.2), grain=(200.0, 6.0, 0.78), grain_weight=0.72,
        stops=((0.00, 0.28, (0.0, 0.0, 0.0)),
               (0.26, 0.58, (0.0, 0.0, 0.0)),
               (0.50, 1.00, (0.0, 0.0, 0.0)),
               (0.72, 1.85, (0.035, 0.035, 0.035)),
               (0.88, 3.30, (0.090, 0.090, 0.090)),
               (1.00, 5.00, (0.140, 0.140, 0.140))))


def _concrete_material(name: str) -> bpy.types.Material:
    return _mottled_material(
        name, (0.42, 0.43, 0.44, 1.0), 0.80,
        stains=(1.1, 6.0, 0.5, 0.9), grain=(30.0, 3.0, 0.6), grain_weight=0.2,
        stops=((0.00, 0.72, (0.030, 0.000, -0.030)),
               (0.40, 0.90, (0.0, 0.0, 0.0)),
               (0.70, 1.04, (0.0, 0.0, 0.0)),
               (1.00, 1.18, (0.0, 0.0, 0.0))))


def _epoxy_material(name: str) -> bpy.types.Material:
    return _mottled_material(
        name, (0.50, 0.51, 0.53, 1.0), 0.30,
        stains=(0.7, 4.0, 0.5, 0.6), grain=(70.0, 2.0, 0.5), grain_weight=0.1,
        stops=((0.00, 0.88, (0.0, 0.0, 0.010)),
               (0.50, 0.98, (0.0, 0.0, 0.0)),
               (1.00, 1.08, (0.0, 0.0, 0.0))))


def _checker_material(name: str) -> bpy.types.Material:
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


def _grass_material(name: str) -> bpy.types.Material:
    return _mottled_material(
        name, (0.20, 0.30, 0.13, 1.0), 0.95,
        stains=(0.9, 4.0, 0.6, 0.7), grain=(40.0, 3.0, 0.6), grain_weight=0.5,
        stops=((0.00, 0.55, (0.020, 0.010, 0.0)),
               (0.45, 0.95, (0.0, 0.0, 0.0)),
               (0.80, 1.30, (0.0, 0.020, 0.0)),
               (1.00, 1.80, (0.010, 0.030, 0.0))))


ROAD_MATERIALS = {
    "asphalt": lambda: _asphalt_material("ROAD_Surface_Asphalt_Mat"),
    "concrete": lambda: _concrete_material("ROAD_Surface_Concrete_Mat"),
    "epoxy": lambda: _epoxy_material("ROAD_Surface_Epoxy_Mat"),
    "checker": lambda: _checker_material("ROAD_Surface_Checker_Mat"),
    "plain": lambda: _principled("ROAD_Surface_Plain_Mat", (0.10, 0.10, 0.11, 1.0), 0.9),
}


def _track_materials(settings) -> Tuple[bpy.types.Material, ...]:
    road = ROAD_MATERIALS.get(settings.ground_texture, ROAD_MATERIALS["asphalt"])()
    return (
        road,
        _principled("ROAD_Shoulder_Mat", (0.46, 0.46, 0.44, 1.0), 0.85),
        _grass_material("ROAD_Grass_Mat"),
        _principled("ROAD_Paint_White_Mat", (0.86, 0.86, 0.84, 1.0), 0.55),
        _principled("ROAD_Paint_Yellow_Mat", (0.78, 0.62, 0.10, 1.0), 0.55),
    )


def _car_materials(body: Optional[bpy.types.Material] = None) -> Tuple[bpy.types.Material, ...]:
    return (
        body or _principled("ROAD_Car_Mat", *avm_layout.MINIBUS_MATERIALS["body"]),
        _principled("ROAD_Car_Glass_Mat", *avm_layout.MINIBUS_MATERIALS["glass"]),
        _principled("ROAD_Car_Tire_Mat", *avm_layout.MINIBUS_MATERIALS["tire"]),
        _principled("ROAD_Car_Lamp_Mat", *avm_layout.MINIBUS_MATERIALS["head"]),
    )


def _parked_material(paint: int) -> bpy.types.Material:
    return _principled(f"ROAD_Parked_Paint_{paint:02d}_Mat",
                       drive_lot.PARKED_COLORS[paint % len(drive_lot.PARKED_COLORS)], 0.32)


#: the pedestrians' clothes (shared, deterministic)
PED_COLORS = (
    ((0.85, 0.72, 0.62, 1.0), (0.18, 0.22, 0.38, 1.0), (0.12, 0.12, 0.14, 1.0)),
    ((0.72, 0.58, 0.46, 1.0), (0.62, 0.16, 0.14, 1.0), (0.14, 0.16, 0.20, 1.0)),
    ((0.88, 0.78, 0.68, 1.0), (0.16, 0.38, 0.24, 1.0), (0.20, 0.20, 0.22, 1.0)),
    ((0.66, 0.52, 0.40, 1.0), (0.30, 0.30, 0.34, 1.0), (0.10, 0.12, 0.16, 1.0)),
)

def _ped_materials(index: int) -> Tuple[bpy.types.Material, ...]:
    skin, torso, legs = PED_COLORS[index % len(PED_COLORS)]
    return (
        _principled(f"ROAD_Ped_Skin_{index:02d}_Mat", skin, 0.6),
        _principled(f"ROAD_Ped_Torso_{index:02d}_Mat", torso, 0.7),
        _principled(f"ROAD_Ped_Legs_{index:02d}_Mat", legs, 0.7),
    )


def _tree_materials() -> Tuple[bpy.types.Material, ...]:
    return (
        _principled("ROAD_Tree_Trunk_Mat", (0.28, 0.20, 0.12, 1.0), 0.9),
        _principled("ROAD_Tree_Leaf_Mat", (0.12, 0.30, 0.10, 1.0), 0.9),
    )


def _lamp_materials() -> Tuple[bpy.types.Material, ...]:
    return (
        _principled("ROAD_Lamp_Metal_Mat", (0.16, 0.16, 0.18, 1.0), 0.4),
        _principled("ROAD_Lamp_Glass_Mat", (1.0, 0.95, 0.8, 1.0), 0.3,
                    emission=(1.0, 0.95, 0.8, 8.0)),
    )


def _sign_materials() -> Tuple[bpy.types.Material, ...]:
    return (
        _principled("ROAD_Sign_Post_Mat", (0.35, 0.35, 0.37, 1.0), 0.5),
        _principled("ROAD_Sign_Board_Mat", (0.86, 0.86, 0.82, 1.0), 0.5),
    )


# ---------------------------------------------------------------------------
# objects / collections
# ---------------------------------------------------------------------------
def _replace_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh) -> None:
    old = obj.data
    obj.data = mesh
    if old is not None and old.users == 0:
        bpy.data.meshes.remove(old)


def _assign_mesh(obj: bpy.types.Object, mesh: bpy.types.Mesh, materials) -> None:
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
        root.empty_display_size = 2.0
        target.objects.link(root)
    elif target not in root.users_collection:
        link_to_collection(root, target)
    return root


def _parent_local(obj: bpy.types.Object, parent: bpy.types.Object) -> None:
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)


def _ensure_vehicle(root: bpy.types.Object, target: bpy.types.Collection) -> bpy.types.Object:
    vehicle_obj = bpy.data.objects.get(VEHICLE_NAME)
    if vehicle_obj is None:
        vehicle_obj = bpy.data.objects.new(VEHICLE_NAME, None)
        vehicle_obj.empty_display_type = "PLAIN_AXES"
        vehicle_obj.empty_display_size = 2.0
        target.objects.link(vehicle_obj)
    elif target not in vehicle_obj.users_collection:
        link_to_collection(vehicle_obj, target)
    vehicle_obj.rotation_mode = "XYZ"
    _parent_local(vehicle_obj, root)
    return vehicle_obj


# ---------------------------------------------------------------------------
# the track mesh
# ---------------------------------------------------------------------------
def _edge(pose: road_track.Pose, half: float, shoulder: float, side: float,
          z: float):
    inner = _offset_point(pose, side * half, z)
    outer = _offset_point(pose, side * (half + shoulder), z)
    return inner, outer


def _bank(pose: road_track.Pose, half: float, shoulder: float, side: float):
    right = _right(pose)
    run = max(0.0, pose.z - BASE_Z) * BANK_SLOPE
    top = _offset_point(pose, side * (half + shoulder))
    bottom = (top[0] + side * run * right[0], top[1] + side * run * right[1], BASE_Z)
    return top, bottom


def _ring(track: road_track.RoadTrack, step: float = MESH_STEP
          ) -> List[road_track.Pose]:
    poses = track.samples(step)
    return poses


def _dash_intervals(length: float) -> List[Tuple[float, float]]:
    intervals: List[Tuple[float, float]] = []
    start = 0.0
    while start < length - 1e-9:
        intervals.append((start, min(start + DASH_ON, length)))
        start += DASH_PERIOD
    return intervals


#: the crossing's arc length is pure geometry, shared with the speed field
crosswalk_distance = road_track.crosswalk_distance


def _add_crosswalk(bm, track: road_track.RoadTrack, half: float,
                   centre: float) -> None:
    """Paint the zebra bars: each bar spans the crossing's depth along travel,
    and the bars repeat across the lane (the classic zebra pattern)."""
    back = track.pose_at(centre - CROSSWALK_DEPTH / 2.0, wrap=False)
    front = track.pose_at(centre + CROSSWALK_DEPTH / 2.0, wrap=False)
    usable = max(0.0, half - CROSSWALK_MARGIN)
    lateral = -usable
    while lateral < usable - 1e-9:
        edge = min(lateral + CROSSWALK_BAR, usable)
        if edge - lateral > 1e-3:
            back_l = _offset_point(back, lateral, back.z + MARK_Z + CROSSWALK_LIFT)
            back_r = _offset_point(back, edge, back.z + MARK_Z + CROSSWALK_LIFT)
            front_r = _offset_point(front, edge, front.z + MARK_Z + CROSSWALK_LIFT)
            front_l = _offset_point(front, lateral, front.z + MARK_Z + CROSSWALK_LIFT)
            _quad(bm, back_l, back_r, front_r, front_l, G_WHITE)
        lateral += CROSSWALK_BAR + CROSSWALK_GAP


def _track_mesh(name: str, track: road_track.RoadTrack, settings) -> bpy.types.Mesh:
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    half = max(0.5, settings.road_width) / 2.0
    shoulder = max(0.0, settings.shoulder_width)
    poses = _ring(track)
    count = len(poses)
    steps = count if track.closed else count - 1

    for index in range(steps):
        a = poses[index]
        b = poses[(index + 1) % count]
        # road: the full driving surface, edge to edge
        _quad(bm, _offset_point(a, -half, a.z), _offset_point(a, half, a.z),
              _offset_point(b, half, b.z), _offset_point(b, -half, b.z), G_ROAD)
        # shoulders (outer point first, then the road edge)
        for side in (-1.0, 1.0):
            pa_in, pa_out = _edge(a, half, shoulder, side, a.z)
            pb_in, pb_out = _edge(b, half, shoulder, side, b.z)
            if side < 0:
                _quad(bm, pa_out, pa_in, pb_in, pb_out, G_SHOULDER)
            else:
                _quad(bm, pa_in, pa_out, pb_out, pb_in, G_SHOULDER)
        # embankments
        for side in (-1.0, 1.0):
            top_a, bottom_a = _bank(a, half, shoulder, side)
            top_b, bottom_b = _bank(b, half, shoulder, side)
            if side < 0:
                _quad(bm, top_a, top_b, bottom_b, bottom_a, G_GRASS)
            else:
                _quad(bm, top_a, bottom_a, bottom_b, top_b, G_GRASS)

    # base plane under everything
    min_x, max_x, min_y, max_y, _, _ = track.bounds()
    margin = 24.0
    _quad(bm, (min_x - margin, min_y - margin, BASE_Z),
          (max_x + margin, min_y - margin, BASE_Z),
          (max_x + margin, max_y + margin, BASE_Z),
          (min_x - margin, max_y + margin, BASE_Z), G_GRASS)

    if settings.show_markings:
        _add_markings(bm, track, half)
        if settings.show_crosswalk:
            crossing = crosswalk_distance(track)
            if crossing is not None:
                _add_crosswalk(bm, track, half, crossing)
    if settings.parking:
        for pose in _bay_poses(track, settings):
            _bay_outline(bm, pose, settings.car_length + 2.0 * BAY_MARGIN,
                         settings.car_width + 2.0 * BAY_MARGIN, G_WHITE)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    return mesh


def _add_markings(bm, track: road_track.RoadTrack, half: float) -> None:
    poses = _ring(track)
    count = len(poses)
    steps = count if track.closed else count - 1
    edge_offset = max(0.0, half - EDGE_INSET)
    for index in range(steps):
        a = poses[index]
        b = poses[(index + 1) % count]
        for side, material in ((-1.0, G_WHITE), (1.0, G_WHITE)):
            offset = side * edge_offset
            a_in = _offset_point(a, offset - side * LINE_WIDTH / 2.0, a.z + MARK_Z)
            a_out = _offset_point(a, offset + side * LINE_WIDTH / 2.0, a.z + MARK_Z)
            b_in = _offset_point(b, offset - side * LINE_WIDTH / 2.0, b.z + MARK_Z)
            b_out = _offset_point(b, offset + side * LINE_WIDTH / 2.0, b.z + MARK_Z)
            if side < 0:
                _quad(bm, a_out, a_in, b_in, b_out, material)
            else:
                _quad(bm, a_in, a_out, b_out, b_in, material)
    # dashed centre line
    for start, end in _dash_intervals(track.length):
        samples = max(2, int(math.ceil((end - start) / MESH_STEP)) + 1)
        for index in range(samples - 1):
            s0 = start + (end - start) * index / (samples - 1)
            s1 = start + (end - start) * (index + 1) / (samples - 1)
            a = track.pose_at(s0, wrap=False)
            b = track.pose_at(s1, wrap=False)
            a_l = _offset_point(a, -LINE_WIDTH / 2.0, a.z + MARK_Z)
            a_r = _offset_point(a, LINE_WIDTH / 2.0, a.z + MARK_Z)
            b_l = _offset_point(b, -LINE_WIDTH / 2.0, b.z + MARK_Z)
            b_r = _offset_point(b, LINE_WIDTH / 2.0, b.z + MARK_Z)
            _quad(bm, a_l, a_r, b_r, b_l, G_YELLOW)


# ---------------------------------------------------------------------------
# props
# ---------------------------------------------------------------------------
def _even_distances(track: road_track.RoadTrack, count: int,
                    offset: float = 0.0) -> List[float]:
    if count <= 0:
        return []
    return [(offset + index / count) % 1.0 * track.length for index in range(count)]


#: perpendicular parking bays at the start of the road [m]
BAY_PAINT_Z = 0.005
#: the painted bay is the car plus this margin on every side, and adjacent bays
#: are separated by the gap - so three bays do not read as one solid block
BAY_MARGIN = 0.30
BAY_GAP = 0.60


def _bay_poses(track: road_track.RoadTrack, settings) -> List[road_track.Pose]:
    """Bay centres, arrayed along the road from the ego bay forward."""
    arc = settings.parking_arc(track)
    if arc is None:
        return []
    angle = math.radians(arc.entry.yaw)
    forward = (-math.sin(angle), math.cos(angle))
    pitch = settings.car_width + 2.0 * BAY_MARGIN + BAY_GAP
    return [road_track.Pose(x=arc.bay.x + forward[0] * pitch * index,
                            y=arc.bay.y + forward[1] * pitch * index,
                            z=arc.bay.z, yaw=arc.bay.yaw)
            for index in range(int(settings.parking_bays))]


def _parking_gaps(track: road_track.RoadTrack, settings):
    """Arc lengths of the gaps **between** the parking bays, and their span.

    A lamp on the right shoulder that lands on a bay would stand in the parked
    car's nose; snapping it to the nearest gap puts it between two cars instead.
    """
    arc = settings.parking_arc(track)
    if arc is None:
        return None, None
    pitch = settings.car_width + 2.0 * BAY_MARGIN + BAY_GAP
    count = max(0, int(settings.parking_bays))
    bay0 = settings.parking_entry_s() - settings.parking_radius()
    gaps = [bay0 + (index + 0.5) * pitch for index in range(count - 1)]
    zone = (bay0 - 0.5 * pitch, bay0 + (count - 0.5) * pitch) if count else None
    return gaps, zone


def _quad_up(bm, points, material: int) -> None:
    """A flat quad wound so its normal points up (parking paint)."""
    area = 0.0
    count = len(points)
    for index in range(count):
        x0, y0, _ = points[index]
        x1, y1, _ = points[(index + 1) % count]
        area += x0 * y1 - x1 * y0
    _face(bm, list(reversed(points)) if area < 0.0 else list(points), material)


def _bay_outline(bm, pose: road_track.Pose, depth: float, width: float,
                 material: int) -> None:
    angle = math.radians(pose.yaw)
    forward = (-math.sin(angle), math.cos(angle))
    across = (math.cos(angle), math.sin(angle))
    z = BAY_PAINT_Z
    half_d, half_w = depth / 2.0, width / 2.0

    def point(along, side):
        return (pose.x + forward[0] * along + across[0] * side,
                pose.y + forward[1] * along + across[1] * side, z)

    half_line = LINE_WIDTH / 2.0
    for side in (-1.0, 1.0):                       # the two side lines
        start = point(-half_d, side * half_w)
        end = point(half_d, side * half_w)
        _quad_up(bm, [
            (start[0] - across[0] * half_line, start[1] - across[1] * half_line, z),
            (end[0] - across[0] * half_line, end[1] - across[1] * half_line, z),
            (end[0] + across[0] * half_line, end[1] + across[1] * half_line, z),
            (start[0] + across[0] * half_line, start[1] + across[1] * half_line, z),
        ], material)
    back = point(-half_d, 0.0)                     # the closed end (kerb side)
    _quad_up(bm, [
        (back[0] - forward[0] * half_line, back[1] - forward[1] * half_line, z),
        (back[0] - forward[0] * half_line + across[0] * half_w,
         back[1] - forward[1] * half_line + across[1] * half_w, z),
        (back[0] + forward[0] * half_line + across[0] * half_w,
         back[1] + forward[1] * half_line + across[1] * half_w, z),
        (back[0] + forward[0] * half_line, back[1] + forward[1] * half_line, z),
    ], material)


def _ensure_bay_cars(target: bpy.types.Collection, root: bpy.types.Object,
                     track: road_track.RoadTrack, settings) -> List[bpy.types.Object]:
    """Parked cars in the bays other than the ego's (the ego drives out)."""
    cars: List[bpy.types.Object] = []
    poses = _bay_poses(track, settings)
    for index, pose in enumerate(poses[1:], start=1):
        name = f"{BAY_CAR_PREFIX}{index:02d}"
        car = _ensure_mesh_object(name, target)
        _assign_mesh(car, _car_mesh(name, settings.car_length,
                                            settings.car_width, settings.car_height),
                     _car_materials(_parked_material(index)))
        car.location = (pose.x, pose.y, pose.z)
        car.rotation_mode = "XYZ"
        car.rotation_euler = (0.0, 0.0, math.radians(pose.yaw))
        _parent_local(car, root)
        cars.append(car)
    for obj in list(target.objects):
        if obj.name.startswith(BAY_CAR_PREFIX) and obj not in cars:
            bpy.data.objects.remove(obj, do_unlink=True)
    return cars


def _pedestrian_layout(track: road_track.RoadTrack, settings):
    """``(distance, side, waiting)`` per pedestrian.

    Two pedestrians wait at the zebra crossing's ends (when there is a crossing
    and at least two people); the rest are spread along both shoulders.  The
    layout is deterministic, so a rebuild always looks the same.
    """
    count = int(settings.pedestrians)
    crossing = crosswalk_distance(track) if settings.show_crosswalk else None
    if crossing is not None and count >= 2:
        layout = [(crossing, -1.0, True), (crossing, 1.0, True)]
        for index, distance in enumerate(_even_distances(track, count - 2, offset=0.13)):
            layout.append((distance, -1.0 if index % 2 == 0 else 1.0, False))
        return layout
    return [(distance, -1.0 if index % 2 == 0 else 1.0, False)
            for index, distance in enumerate(_even_distances(track, count, offset=0.13))]


def _ensure_pedestrians(target: bpy.types.Collection, root: bpy.types.Object,
                        track: road_track.RoadTrack, settings) -> List[bpy.types.Object]:
    peds: List[bpy.types.Object] = []
    half = settings.road_width / 2.0
    for index, (distance, side, waiting) in enumerate(_pedestrian_layout(track, settings)):
        pose = track.pose_at(distance)
        name = f"{PED_PREFIX}{index:02d}"
        obj = _ensure_mesh_object(name, target)
        height = 1.6 + 0.12 * (index % 3)
        _assign_mesh(obj, prop_mesh.pedestrian_mesh(name, height),
                     _ped_materials(index))
        lateral = side * (half + settings.shoulder_width * 0.5)
        obj.location = _offset_point(pose, lateral)
        obj.rotation_mode = "XYZ"
        # a person at the crossing faces across the road; the others face the
        # road (and, when walking, along the track)
        if waiting:
            facing = pose.yaw + side * 90.0
        else:
            facing = pose.yaw + (90.0 if side < 0 else -90.0)
        obj.rotation_euler = (0.0, 0.0, math.radians(facing))
        # a reuse must not keep last rebuild's walk keys unless asked again
        if obj.animation_data is not None:
            obj.animation_data_clear()
        obj["road_ped_distance"] = float(distance)
        obj["road_ped_lateral"] = float(lateral)
        obj["road_ped_speed"] = float(settings.pedestrian_speed)
        _parent_local(obj, root)
        peds.append(obj)
    for obj in list(target.objects):
        if obj.name.startswith(PED_PREFIX) and obj not in peds:
            bpy.data.objects.remove(obj, do_unlink=True)
    return peds


def _ensure_trees(target: bpy.types.Collection, root: bpy.types.Object,
                  track: road_track.RoadTrack, settings) -> List[bpy.types.Object]:
    trees: List[bpy.types.Object] = []
    count = int(settings.trees)
    half = settings.road_width / 2.0
    for index, distance in enumerate(_even_distances(track, count, offset=0.05)):
        pose = track.pose_at(distance)
        side = -1.0 if index % 2 == 0 else 1.0
        name = f"{TREE_PREFIX}{index:02d}"
        obj = _ensure_mesh_object(name, target)
        height = 3.4 + 0.9 * (index % 4)
        _assign_mesh(obj, prop_mesh.tree_mesh(name, height), _tree_materials())
        run = max(0.0, pose.z - BASE_Z) * BANK_SLOPE
        lateral = side * (half + settings.shoulder_width + run + TREE_CLEAR)
        tree_x, tree_y, _ = _offset_point(pose, lateral)
        obj.location = (tree_x, tree_y, BASE_Z)
        obj.rotation_mode = "XYZ"
        obj.rotation_euler = (0.0, 0.0, math.radians(pose.yaw + 37.0 * index))
        _parent_local(obj, root)
        trees.append(obj)
    for obj in list(target.objects):
        if obj.name.startswith(TREE_PREFIX) and obj not in trees:
            bpy.data.objects.remove(obj, do_unlink=True)
    return trees


def _ensure_lamps(target: bpy.types.Collection, root: bpy.types.Object,
                  track: road_track.RoadTrack, settings) -> List[bpy.types.Object]:
    lamps: List[bpy.types.Object] = []
    count = int(settings.lamps)
    half = settings.road_width / 2.0
    gaps, bay_zone = _parking_gaps(track, settings)
    for index, distance in enumerate(_even_distances(track, count, offset=0.30)):
        side = 1.0 if index % 2 == 0 else -1.0
        # a right-side lamp that lands on a parking bay moves to the gap between
        # two bays, so it never stands in a parked car's nose
        if side > 0 and gaps and bay_zone is not None and bay_zone[0] <= distance <= bay_zone[1]:
            distance = min(gaps, key=lambda gap: abs(gap - distance))
        pose = track.pose_at(distance)
        name = f"{LAMP_PREFIX}{index:02d}"
        obj = _ensure_mesh_object(name, target)
        _assign_mesh(obj, prop_mesh.lamp_mesh(name, 6.0, 1.4), _lamp_materials())
        lateral = side * (half + settings.shoulder_width + 0.1)
        obj.location = _offset_point(pose, lateral)
        obj.rotation_mode = "XYZ"
        # the lamp arm leaves the mesh along +X; turn it in over the road
        obj.rotation_euler = (0.0, 0.0, math.radians(pose.yaw + (0.0 if side < 0 else 180.0)))
        _parent_local(obj, root)
        lamps.append(obj)
    for obj in list(target.objects):
        if obj.name.startswith(LAMP_PREFIX) and obj not in lamps:
            bpy.data.objects.remove(obj, do_unlink=True)
    return lamps


def _ensure_signs(target: bpy.types.Collection, root: bpy.types.Object,
                  track: road_track.RoadTrack, settings) -> List[bpy.types.Object]:
    signs: List[bpy.types.Object] = []
    count = int(settings.signs)
    half = settings.road_width / 2.0
    for index, distance in enumerate(_even_distances(track, count, offset=0.68)):
        pose = track.pose_at(distance)
        side = 1.0 if index % 2 == 0 else -1.0
        name = f"{SIGN_PREFIX}{index:02d}"
        obj = _ensure_mesh_object(name, target)
        _assign_mesh(obj, prop_mesh.sign_mesh(name, 2.2), _sign_materials())
        lateral = side * (half + settings.shoulder_width + 0.2)
        obj.location = _offset_point(pose, lateral)
        obj.rotation_mode = "XYZ"
        obj.rotation_euler = (0.0, 0.0, math.radians(pose.yaw + (180.0 if side < 0 else 0.0)))
        _parent_local(obj, root)
        signs.append(obj)
    for obj in list(target.objects):
        if obj.name.startswith(SIGN_PREFIX) and obj not in signs:
            bpy.data.objects.remove(obj, do_unlink=True)
    return signs


# ---------------------------------------------------------------------------
# cameras (the add-on's own OpenCV cameras, mounted in the vehicle frame)
# ---------------------------------------------------------------------------
def camera_name(key: str) -> str:
    return avm_cameras.object_name(CAMERA_PREFIX, key)


def preset_camera(key: str = avm_cameras.FRONT) -> Dict:
    for record in avm_layout.cameras_from_preset(avm_layout.load_preset()):
        if record["name"] == key:
            return record
    return {}


def _apply_camera_pose(camera: bpy.types.Object, record: Dict) -> None:
    camera.location = tuple(record.get("location", (0.0, 0.0, 0.0)))
    camera.rotation_mode = "XYZ"
    camera.rotation_euler = tuple(math.radians(value)
                                  for value in record.get("rotation", (0.0, 0.0, 0.0)))


def _ensure_cameras(scene: bpy.types.Scene, target: bpy.types.Collection,
                    vehicle_obj: bpy.types.Object,
                    messages: List[str]) -> Dict[str, bpy.types.Object]:
    cameras: Dict[str, bpy.types.Object] = {}
    for key in avm_cameras.CAMERAS:
        name = camera_name(key)
        camera = bpy.data.objects.get(name)
        if camera is None:
            record = preset_camera(key)
            camera, _ = camera_factory.add_camera(
                scene, model="fisheye", preset=None, name=name,
                location=tuple(record.get("location", (0.0, 0.0, 0.0))))
            camera.name = name
            camera.data.name = name
            ok, apply_messages = camera_factory.configure_from_record(camera, record, scene)
            if not ok:
                messages.append(f"{key} camera: " + "; ".join(apply_messages))
            _apply_camera_pose(camera, record)
        if target not in camera.users_collection:
            link_to_collection(camera, target)
        _parent_local(camera, vehicle_obj)
        cameras[key] = camera
    return cameras


def apply_active_camera(scene: bpy.types.Scene, settings) -> Optional[bpy.types.Object]:
    camera = bpy.data.objects.get(camera_name(settings.active_camera))
    if camera is not None and scene is not None:
        scene.camera = camera
        intrinsics = camera.data.opencv_cam.intrinsics
        width, height = int(intrinsics.image_width), int(intrinsics.image_height)
        if width > 0 and height > 0:
            scene.render.resolution_x = width
            scene.render.resolution_y = height
            scene.render.resolution_percentage = 100
            apply_mod.apply_settings(camera.data, camera.data.opencv_cam, scene,
                                     resolution=(width, height))
    return camera


def sync_cameras_from_avm(settings, scene: bpy.types.Scene) -> List[str]:
    messages: List[str] = []
    for key in avm_cameras.CAMERAS:
        source = bpy.data.objects.get(avm_cameras.object_name("AVM_Cam_", key))
        camera = bpy.data.objects.get(camera_name(key))
        if source is None or camera is None:
            messages.append(f"{key}: no AVM camera to copy from")
            continue
        record = {
            "location": tuple(float(value) for value in source.location),
            "rotation": tuple(math.degrees(float(value)) for value in source.rotation_euler),
            "K": [float(source.data.opencv_cam.intrinsics.fx),
                  float(source.data.opencv_cam.intrinsics.fy),
                  float(source.data.opencv_cam.intrinsics.cx),
                  float(source.data.opencv_cam.intrinsics.cy)],
            "D": [float(source.data.opencv_cam.distortion.k1),
                  float(source.data.opencv_cam.distortion.k2),
                  float(source.data.opencv_cam.distortion.k3),
                  float(source.data.opencv_cam.distortion.k4)],
            "output": [int(source.data.opencv_cam.intrinsics.image_width),
                       int(source.data.opencv_cam.intrinsics.image_height)],
        }
        ok, apply_messages = camera_factory.configure_from_record(camera, record, scene)
        _apply_camera_pose(camera, record)
        messages.append(f"{key}: copied from {source.name}"
                        + ("" if ok else " (" + "; ".join(apply_messages) + ")"))
    return messages


# ---------------------------------------------------------------------------
# lights / render setup
# ---------------------------------------------------------------------------
def _ensure_lights(scene: bpy.types.Scene, target: bpy.types.Collection,
                   settings, track: road_track.RoadTrack) -> List[bpy.types.Object]:
    """One even light with a dim world, like the Drive Scene's flat look.

    An outdoor sky plus a sun washed the road out; a large close panel burned a
    hotspot into the loop interior.  A single **sun** (directional, so its
    irradiance is the same everywhere on the loop) with a dim world gives the
    even, low-contrast lighting the Drive Scene has, so a reconstruction verified
    on one scene holds on the other.
    """
    name = f"{LIGHT_PREFIX}Sun"
    light = bpy.data.objects.get(name)
    if light is None:
        light_data = bpy.data.lights.new(name, type="SUN")
        light = bpy.data.objects.new(name, light_data)
        target.objects.link(light)
    elif target not in light.users_collection:
        link_to_collection(light, target)
    light.data.energy = float(settings.light_energy)
    light.data.angle = math.radians(3.0)
    light.data.use_shadow = False
    light.location = (0.0, 0.0, 40.0)
    light.rotation_mode = "XYZ"
    light.rotation_euler = (math.radians(45.0), 0.0, math.radians(35.0))
    for obj in list(target.objects):
        if obj.name.startswith(LIGHT_PREFIX) and obj is not light:
            bpy.data.objects.remove(obj, do_unlink=True)
    return [light]


def _ensure_render_setup(scene: bpy.types.Scene, messages: List[str]) -> None:
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
        # dim ambient, same as the Drive Scene: the loop is lit by its own panel
        background.inputs[0].default_value = (0.045, 0.047, 0.05, 1.0)
        background.inputs[1].default_value = 1.0


# ---------------------------------------------------------------------------
# the drive
# ---------------------------------------------------------------------------
def apply_drive(scene: bpy.types.Scene, vehicle_obj: bpy.types.Object,
                settings, plan) -> None:
    """Key the vehicle along the plan: one key per frame, linear."""
    if vehicle_obj.animation_data is not None:
        vehicle_obj.animation_data_clear()
    for frame in plan.frames:
        vehicle_obj.location = (frame.x, frame.y, frame.z)
        vehicle_obj.rotation_euler = (math.radians(frame.pitch),
                                      math.radians(frame.roll),
                                      math.radians(frame.yaw))
        vehicle_obj.keyframe_insert("location", frame=frame.index)
        vehicle_obj.keyframe_insert("rotation_euler", frame=frame.index)
    animation = vehicle_obj.animation_data
    if animation is not None and animation.action is not None:
        for curve in compat.action_fcurves(animation.action):
            for key in curve.keyframe_points:
                key.interpolation = "LINEAR"
    scene.frame_start = plan.frames[0].index
    scene.frame_end = plan.frames[-1].index
    scene.render.fps = max(1, int(round(settings.drive_fps)))
    scene.render.fps_base = 1.0
    scene.frame_set(scene.frame_start)


def _animate_pedestrians(vehicle_root: bpy.types.Object, peds: List[bpy.types.Object],
                         track: road_track.RoadTrack, settings, plan) -> None:
    """Walk each pedestrian forward along the shoulder, one key per frame."""
    for obj in peds:
        if obj.animation_data is not None:
            obj.animation_data_clear()
        start = float(obj.get("road_ped_distance", 0.0))
        lateral = float(obj.get("road_ped_lateral", 0.0))
        speed = float(obj.get("road_ped_speed", 1.2))
        for frame in plan.frames:
            distance = (start + speed * frame.time) % track.length
            pose = track.pose_at(distance)
            obj.location = _offset_point(pose, lateral)
            obj.rotation_euler = (0.0, 0.0, math.radians(pose.yaw))
            obj.keyframe_insert("location", frame=frame.index)
            obj.keyframe_insert("rotation_euler", frame=frame.index)
        animation = obj.animation_data
        if animation is not None and animation.action is not None:
            for curve in compat.action_fcurves(animation.action):
                for key in curve.keyframe_points:
                    key.interpolation = "LINEAR"


def view_targets(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    names = [CAR_NAME] + [camera_name(key) for key in avm_cameras.CAMERAS]
    return [obj for obj in (bpy.data.objects.get(name) for name in names)
            if obj is not None]


# ---------------------------------------------------------------------------
# build / rebuild / remove
# ---------------------------------------------------------------------------
def build(scene: bpy.types.Scene, settings) -> Dict:
    return rebuild(scene, settings)


def rebuild(scene: bpy.types.Scene, settings) -> Dict:
    messages: List[str] = []
    _ensure_render_setup(scene, messages)
    settings.ensure_cameras()

    target = collection(COLLECTION_NAME, scene, create=True)
    root = _ensure_root(scene, target)
    settings.root = root
    vehicle_obj = _ensure_vehicle(root, target)

    track = settings.track()
    plan = settings.plan()

    # the track ------------------------------------------------------------
    ground = _ensure_mesh_object(GROUND_NAME, target)
    _assign_mesh(ground, _track_mesh("ROAD_Ground", track, settings),
                 _track_materials(settings))
    ground.location = (0.0, 0.0, 0.0)
    _parent_local(ground, root)

    # props ----------------------------------------------------------------
    trees = _ensure_trees(target, root, track, settings)
    lamps = _ensure_lamps(target, root, track, settings)
    signs = _ensure_signs(target, root, track, settings)
    peds = _ensure_pedestrians(target, root, track, settings)
    bay_cars = _ensure_bay_cars(target, root, track, settings)

    # the vehicle and its cameras ------------------------------------------
    car = _ensure_mesh_object(CAR_NAME, target)
    _assign_mesh(car, _car_mesh("ROAD_Car", settings.car_length,
                                         settings.car_width, settings.car_height),
                 _car_materials())
    car.location = (0.0, 0.0, settings.car_clearance)
    _parent_local(car, vehicle_obj)

    cameras = _ensure_cameras(scene, target, vehicle_obj, messages)
    apply_active_camera(scene, settings)

    lights = _ensure_lights(scene, target, settings, track)
    for light in lights:
        _parent_local(light, root)

    apply_drive(scene, vehicle_obj, settings, plan)
    if settings.animate_pedestrians:
        _animate_pedestrians(root, peds, track, settings, plan)
    settings.revision += 1
    return {
        "root": root,
        "vehicle": vehicle_obj,
        "ground": ground,
        "trees": trees,
        "lamps": lamps,
        "signs": signs,
        "pedestrians": peds,
        "bay_cars": bay_cars,
        "car": car,
        "cameras": cameras,
        "lights": lights,
        "track": track,
        "plan": plan,
        "messages": messages,
    }


def remove(scene: bpy.types.Scene, settings) -> int:
    removed = remove_collection_objects(COLLECTION_NAME)
    root = settings.root
    if root is not None and root.name in bpy.data.objects:
        bpy.data.objects.remove(root, do_unlink=True)
        removed += 1
    settings.root = None
    for key in avm_cameras.CAMERAS:
        camera = bpy.data.objects.get(camera_name(key))
        if camera is not None:
            bpy.data.objects.remove(camera, do_unlink=True)
            removed += 1
    target = bpy.data.collections.get(COLLECTION_NAME)
    if target is not None and not target.objects and not target.children:
        bpy.data.collections.remove(target)
    return removed
