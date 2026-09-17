"""AVM Scene builder: create and update the ground, car, blocks and cameras.

The objects are created once and then **updated in place** (idempotent rebuild),
so a slider drag never destroys the user's selection, materials or parenting.
Every mesh is generated at its real size (object scale stays 1), which keeps the
exported GLB/FBX and the coverage maths honest.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import bmesh
import bpy
from mathutils import Matrix

from ....core.scenes import avm_coverage, avm_layout
from ... import apply as apply_mod
from ... import camera_factory
from ..base import collection, link_to_collection, remove_collection_objects

ROOT_NAME = "AVM_Root"
COLLECTION_NAME = "AVM Scene"

GROUND_NAME = "AVM_Ground"
CAR_NAME = "AVM_Car"
SUN_NAME = "AVM_Sun"
BLOCK_PREFIX = "AVM_Block_"
CAMERA_PREFIX = "AVM_Cam_"

#: block key -> object name suffix
BLOCK_SUFFIX = {
    avm_layout.BLOCK_FRONT_LEFT: "FrontLeft",
    avm_layout.BLOCK_FRONT_RIGHT: "FrontRight",
    avm_layout.BLOCK_BACK_LEFT: "BackLeft",
    avm_layout.BLOCK_BACK_RIGHT: "BackRight",
}
CAMERA_SUFFIX = {"front": "Front", "back": "Back", "left": "Left", "right": "Right"}


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


# ---------------------------------------------------------------------------
# object creation
# ---------------------------------------------------------------------------
def _new_mesh_object(name: str, mesh: bpy.types.Mesh,
                     target: bpy.types.Collection) -> bpy.types.Object:
    obj = bpy.data.objects.new(name, mesh)
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


def _parent(obj: bpy.types.Object, root: bpy.types.Object) -> None:
    if obj.parent is not root:
        obj.parent = root
        obj.matrix_parent_inverse = root.matrix_world.inverted()


def _ensure_cameras(scene: bpy.types.Scene, settings, target: bpy.types.Collection,
                    preset_cameras: Dict[str, Dict]) -> Dict[str, bpy.types.Object]:
    """Create the four fisheye cameras, configured from the preset records."""
    cameras: Dict[str, bpy.types.Object] = {}
    for name in avm_layout.CAMERAS:
        object_name = f"{CAMERA_PREFIX}{CAMERA_SUFFIX[name]}"
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
            _configure_camera(camera, record, scene)
        link_to_collection(camera, target)
        cameras[name] = camera
    return cameras


def _configure_camera(camera: bpy.types.Object, record: Dict,
                      scene: bpy.types.Scene) -> None:
    """Write the preset's K / D / output into the camera's own settings."""
    settings = camera.data.opencv_cam
    intrinsics = settings.intrinsics
    fx, fy, cx, cy = record.get("K", (0.0, 0.0, 0.0, 0.0))
    intrinsics.fx, intrinsics.fy = float(fx), float(fy)
    intrinsics.auto_center = False
    intrinsics.cx, intrinsics.cy = float(cx), float(cy)
    width, height = record.get("output", (1280, 960))
    intrinsics.image_width, intrinsics.image_height = int(width), int(height)
    intrinsics.scale_to_render = True

    distortion = settings.distortion
    distortion.model = "fisheye"
    distortion.enabled = True
    coefficients = list(record.get("D", (0.0, 0.0, 0.0, 0.0))) + [0.0] * 4
    (distortion.k1, distortion.k2, distortion.k3,
     distortion.k4) = (float(value) for value in coefficients[:4])
    settings.output.mode = "calibration"
    apply_mod.apply_settings(camera.data, settings, scene)


def apply_camera_pose(camera: bpy.types.Object, location, rotation_deg) -> None:
    """Set the mount pose, then sync ``CV Extrinsics`` (R/t) from the object.

    The object matrix is written in one go and R/t are read *back* from it: going
    through ``pose.euler`` first would read a stale ``matrix_world`` (the
    depsgraph has not caught up yet) and collapse the mount position to the
    origin.
    """
    matrix = avm_coverage.object_matrix(location, rotation_deg)
    camera.matrix_world = Matrix((matrix[0:4], matrix[4:8],
                                  matrix[8:12], matrix[12:16]))
    settings = camera.data.opencv_cam
    rotation, translation = apply_mod.read_opencv_pose(camera, settings)
    settings.pose.rotation = rotation
    settings.pose.translation = translation


# ---------------------------------------------------------------------------
# build / rebuild
# ---------------------------------------------------------------------------
def build(scene: bpy.types.Scene, settings, preset: Optional[Dict] = None) -> Dict[str, List]:
    """Create the scene objects and return them (also stored on ``settings``)."""
    from . import controller

    preset = preset or avm_layout.load_preset()
    controller.load_preset_into(settings, preset)

    target = collection(COLLECTION_NAME, scene, create=True)
    root = _ensure_root(scene, target)
    settings.root = root
    preset_cameras = {record["name"]: record
                      for record in avm_layout.cameras_from_preset(preset)}
    _ensure_cameras(scene, settings, target, preset_cameras)
    return rebuild(scene, settings)


def rebuild(scene: bpy.types.Scene, settings) -> Dict[str, List]:
    """Update every object in place; creates anything that is missing."""
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
    car_material = _principled("AVM_Car_Mat", (0.78, 0.79, 0.80, 1.0), 0.5)
    block_material = _principled("AVM_Block_Mat", (0.02, 0.02, 0.02, 1.0), 0.9)

    # ground ---------------------------------------------------------------
    ground = bpy.data.objects.get(GROUND_NAME)
    if ground is None:
        ground = _new_mesh_object(GROUND_NAME, _quad_mesh("AVM_Ground", 1.0, 1.0), target)
    _replace_mesh(ground, _quad_mesh("AVM_Ground", settings.ground_w, settings.ground_d))
    ground.location = (0.0, 0.0, 0.0)
    _assign(ground, ground_material)
    _parent(ground, root)
    ground.hide_render = not settings.show_ground

    # car ------------------------------------------------------------------
    car_length, car_width = settings.car_size()
    car = bpy.data.objects.get(CAR_NAME)
    if car is None:
        car = _new_mesh_object(CAR_NAME, _box_mesh("AVM_Car", 1.0, 1.0, 1.0), target)
    _replace_mesh(car, _box_mesh("AVM_Car", car_width, car_length, settings.car_height))
    car.location = (0.0, 0.0, settings.car_clearance)
    _assign(car, car_material)
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
        _replace_mesh(block, _quad_mesh(name, x1 - x0, y1 - y0))
        block.location = (0.5 * (x0 + x1), 0.5 * (y0 + y1), settings.block_lift)
        _assign(block, block_material)
        _parent(block, root)
        block.hide_render = not settings.show_blocks
        block_objects.append(block)

    # cameras --------------------------------------------------------------
    camera_objects = []
    for index, name in enumerate(avm_layout.CAMERAS):
        record = settings.camera(name)
        camera = bpy.data.objects.get(f"{CAMERA_PREFIX}{CAMERA_SUFFIX[name]}")
        if camera is None:
            continue
        camera.hide_render = not (record.enable if record else True) or not settings.show_cameras
        if record is not None:
            apply_camera_pose(camera, record.location, record.rotation)
        # only the active camera drives the scene resolution
        camera.data.opencv_cam.output.lock_scene_resolution = (name == settings.active_camera)
        _parent(camera, root)
        camera_objects.append(camera)

    # a sun so the scene reads out of the box (it is removed with the scene)
    sun = bpy.data.objects.get(SUN_NAME)
    if sun is None:
        sun_data = bpy.data.lights.new(SUN_NAME, type="SUN")
        sun = bpy.data.objects.new(SUN_NAME, sun_data)
        target.objects.link(sun)
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(35.0), 0.0, math.radians(-40.0))
    _parent(sun, root)

    _apply_active_resolution(scene, settings)
    settings.revision += 1
    from . import coverage  # lazy: coverage imports this module for the names
    coverage.apply_visibility(settings)
    return {
        "root": root,
        "ground": ground,
        "car": car,
        "blocks": block_objects,
        "cameras": camera_objects,
        "sun": sun,
    }


def _apply_active_resolution(scene: bpy.types.Scene, settings) -> None:
    """Let the active camera write the render resolution."""
    active = settings.active_camera_settings()
    if active is None:
        return
    camera = bpy.data.objects.get(f"{CAMERA_PREFIX}{CAMERA_SUFFIX[settings.active_camera]}")
    if camera is None:
        return
    apply_mod.apply_render_resolution(scene, camera.data.opencv_cam)


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
