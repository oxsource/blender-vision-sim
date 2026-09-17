"""Build a throw-away test scene for eyeballing the projection.

Creates a checker cube, a ground grid and a light in front of the active camera
so the distortion is visible without any modelling work.
"""

from __future__ import annotations

import math
from typing import List, Optional

import bpy
from mathutils import Vector

from . import apply as apply_mod

COLLECTION_NAME = "OpenCV Camera Test"


def _collection(scene: bpy.types.Scene) -> bpy.types.Collection:
    collection = bpy.data.collections.get(COLLECTION_NAME)
    if collection is None:
        collection = bpy.data.collections.new(COLLECTION_NAME)
        scene.collection.children.link(collection)
    return collection


def _checker_material(name: str, scale: float, color_a=(0.85, 0.85, 0.85, 1.0),
                      color_b=(0.05, 0.05, 0.05, 1.0)) -> bpy.types.Material:
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    checker = nodes.new("ShaderNodeTexChecker")
    coords = nodes.new("ShaderNodeTexCoord")
    checker.inputs["Scale"].default_value = scale
    checker.inputs["Color1"].default_value = color_a
    checker.inputs["Color2"].default_value = color_b
    principled.inputs["Roughness"].default_value = 0.6
    material.node_tree.links.new(coords.outputs["Object"], checker.inputs["Vector"])
    material.node_tree.links.new(checker.outputs["Color"], principled.inputs["Base Color"])
    material.node_tree.links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


def _link(scene: bpy.types.Scene, obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    collection.objects.link(obj)
    for coll in list(obj.users_collection):
        if coll is not collection:
            coll.objects.unlink(obj)


def build(
    camera_object: bpy.types.Object,
    scene: Optional[bpy.types.Scene] = None,
    distance: float = 4.0,
    cube_size: float = 1.2,
    ground_offset: float = 2.0,
    clean: bool = True,
) -> List[bpy.types.Object]:
    """Create a checker cube + ground grid + sun light in front of the camera.

    Returns the created objects.  Everything is placed in the camera's local
    frame, so it works with any camera transform.
    """
    scene = scene or bpy.context.scene
    collection = _collection(scene)
    matrix = camera_object.matrix_world.copy()

    if clean:
        for obj in list(collection.objects):
            bpy.data.objects.remove(obj, do_unlink=True)

    created: List[bpy.types.Object] = []

    # checker cube straight ahead
    bpy.ops.mesh.primitive_cube_add(size=cube_size, location=(0.0, 0.0, 0.0))
    cube = bpy.context.active_object
    cube.name = "TestCube"
    cube.matrix_world = matrix @ _local_translation(0.0, 0.0, -distance)
    cube.data.materials.clear()
    cube.data.materials.append(_checker_material("OpenCVTestCube", 6.0))
    _link(scene, cube, collection)
    created.append(cube)

    # ground grid perpendicular to the camera's up axis
    bpy.ops.mesh.primitive_plane_add(size=distance * 6.0)
    ground = bpy.context.active_object
    ground.name = "TestGround"
    ground.matrix_world = (
        matrix
        @ _local_translation(0.0, -ground_offset, -distance)
        @ _local_rotation(math.radians(90.0), 0.0, 0.0)
    )
    ground.data.materials.clear()
    ground.data.materials.append(
        _checker_material("OpenCVTestGround", 8.0, (0.30, 0.31, 0.33, 1.0), (0.72, 0.73, 0.75, 1.0))
    )
    _link(scene, ground, collection)
    created.append(ground)

    # a few coloured blocks at the frame edges, useful to see the warp
    for index, (x, y, z, color) in enumerate((
        (-0.85, 0.35, -distance * 0.75, (0.9, 0.2, 0.15, 1.0)),
        (0.85, 0.35, -distance * 0.75, (0.15, 0.5, 0.9, 1.0)),
        (-0.85, -0.35, -distance * 1.25, (0.9, 0.75, 0.1, 1.0)),
        (0.85, -0.35, -distance * 1.25, (0.2, 0.75, 0.35, 1.0)),
    )):
        bpy.ops.mesh.primitive_cube_add(size=cube_size * 0.35)
        block = bpy.context.active_object
        block.name = f"TestBlock{index}"
        block.matrix_world = matrix @ _local_translation(x * distance * 0.8, y * distance * 0.8, z)
        block.data.materials.clear()
        material = bpy.data.materials.new(f"OpenCVTestBlock{index}")
        material.use_nodes = True
        principled = material.node_tree.nodes["Principled BSDF"]
        principled.inputs["Base Color"].default_value = color
        block.data.materials.append(material)
        _link(scene, block, collection)
        created.append(block)

    # headlight style point light at the camera, plus a sun for shape.
    # A sun alone leaves the subjects much too dark for a quick visual check.
    light_data = bpy.data.lights.new("OpenCVTestLight", type="POINT")
    light_data.energy = 200.0 * (distance / 4.0) ** 2 * math.pi
    light_data.shadow_soft_size = 0.4 * distance / 4.0
    light = bpy.data.objects.new("OpenCVTestLight", light_data)
    light.matrix_world = matrix @ _local_translation(0.0, 0.5, 0.2)
    _link(scene, light, collection)
    created.append(light)

    sun_data = bpy.data.lights.new("OpenCVTestSun", type="SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("OpenCVTestSun", sun_data)
    sun.matrix_world = matrix @ _local_rotation(math.radians(20.0), 0.0, math.radians(-35.0))
    _link(scene, sun, collection)
    created.append(sun)
    return created


def _local_translation(x: float, y: float, z: float):
    from mathutils import Matrix
    return Matrix.Translation(Vector((x, y, z)))


def _local_rotation(rx: float, ry: float, rz: float):
    from mathutils import Euler
    return Euler((rx, ry, rz), "XYZ").to_matrix().to_4x4()


def prepare_render(scene: bpy.types.Scene, settings, samples: int = 64) -> None:
    """Set up sane Cycles settings for a distortion check."""
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    # "Standard" shows the geometry without tone mapping surprises
    scene.view_settings.view_transform = "Standard"
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.use_nodes = True
    background = scene.world.node_tree.nodes.get("Background")
    if background is None:
        background = scene.world.node_tree.nodes.new("ShaderNodeBackground")
        scene.world.node_tree.links.new(
            background.outputs["Background"],
            scene.world.node_tree.nodes["World Output"].inputs["Surface"],
        )
    background.inputs[0].default_value = (0.055, 0.06, 0.075, 1.0)  # dim sky, not pure black
    background.inputs[1].default_value = 1.0
    intrinsics = settings.intrinsics
    if intrinsics.image_width and intrinsics.image_height:
        scene.render.resolution_x = int(intrinsics.image_width)
        scene.render.resolution_y = int(intrinsics.image_height)


def apply_and_build(camera_object: bpy.types.Object, scene: Optional[bpy.types.Scene] = None,
                    samples: int = 64):
    """Convenience for operators: prepare render settings, apply, build the scene."""
    scene = scene or bpy.context.scene
    settings = camera_object.data.opencv_cam
    prepare_render(scene, settings, samples=samples)
    ok, messages = apply_mod.apply_settings(camera_object.data, settings, scene)
    if not ok:
        return False, messages, []
    created = build(camera_object, scene)
    scene.camera = camera_object
    try:  # keep the camera selected so the panel operators stay valid
        view_layer = bpy.context.view_layer
        for obj in view_layer.objects:
            obj.select_set(False)
        camera_object.select_set(True)
        view_layer.objects.active = camera_object
    except Exception:
        pass
    return True, messages, created