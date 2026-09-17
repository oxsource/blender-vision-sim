"""Camera Scene: a checker cube, ground grid and lights in front of the camera.

The quick "look at the distortion" scene.  It is a one-shot builder (no settings
panel, no root empty), but it goes through the same scene registry and menu as
every other scene.

The scene is **world aligned** (see ``docs/avm-scene.md`` §17.5): the ground is
a horizontal plane and the subjects stand on it, the way the AVM Scene is laid
out.  Only the camera's pose decides *where* in front of it the subject goes, so
a camera pitched down 45 deg no longer builds a tilted floor.

Originally ``bl/scene_builder.py``; the object names are unchanged so existing
``.blend`` files and scripts keep working.
"""

from __future__ import annotations

import math
from typing import List, Optional

import bpy
from bpy.props import FloatProperty, IntProperty
from mathutils import Vector

from .. import apply as apply_mod
from .. import camera_factory
from . import view
from .base import SceneDefinition, collection, link_to_collection
from .view import ViewSpec

COLLECTION_NAME = "OpenCV Camera Scene"
GROUND_NAME = "CheckerGround"
#: the ground sits this far below the cube's base, so the two coplanar faces
#: cannot z-fight.  1 mm: above the AVM Scene's 2 mm drop, so when both scenes
#: are built their floors never land on exactly the same plane
GROUND_DROP = 0.001


def view_targets(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    """The checker cube and the colour blocks - and nothing else.

    The checker ground is ``6 x distance`` across (24 m at the default distance)
    while the cube is 1.2 m, and the two lights sit on the camera, so framing
    any of them would either shrink the cube to a few percent of the viewport or
    drag the centre back to the eye.  The ground still fills the frame behind.
    """
    return [obj for obj in view.scene_objects(DEFINITION)
            if obj.type == "MESH" and obj.name != GROUND_NAME]


DEFINITION = SceneDefinition(
    id="camera_scene",
    label="Camera Scene",
    icon="camera_scene",
    order=10,
    add_operator="opencv_cam.add_camera_scene",
    collection_name=COLLECTION_NAME,
    # the ground is the world floor now, so the standard 3/4 orbit reads it as
    # a floor: you look down at the cube, the ground it stands on and the
    # colour blocks around it
    view=ViewSpec(azimuth=135.0, elevation=30.0),
    view_targets=view_targets,
)


def _collection(scene: bpy.types.Scene) -> bpy.types.Collection:
    return collection(COLLECTION_NAME, scene, create=True)


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


def layout(camera_object: bpy.types.Object, distance: float = 4.0,
           cube_size: float = 1.2):
    """``(subject, floor)``: the cube's centre and the plane it stands on.

    A camera above the ground that pitches down meets ``z = 0`` along its central
    ray, so the cube goes **exactly where the camera is looking**: it is centred
    in the frame *and* it stands on the same world floor as the AVM Scene, which
    keeps the two scenes consistent (``docs/avm-scene.md`` §17.5).

    A camera aimed at or above the horizon - or one sitting *in* the ground
    plane, like a freshly added camera - never meets it, so the cube goes
    ``distance`` straight ahead and the floor drops to its feet instead.
    """
    matrix = camera_object.matrix_world
    location = matrix.translation
    forward = -(matrix.to_3x3() @ Vector((0.0, 0.0, 1.0)))  # a camera looks down -Z
    if forward.z < -1e-3 and location.z > 1e-3:
        hit = location + forward * (location.z / -forward.z)
        return Vector((hit.x, hit.y, cube_size * 0.5)), 0.0
    subject = location + forward * distance
    return subject, subject.z - cube_size * 0.5


def build(
    camera_object: bpy.types.Object,
    scene: Optional[bpy.types.Scene] = None,
    distance: float = 4.0,
    cube_size: float = 1.2,
    clean: bool = True,
) -> List[bpy.types.Object]:
    """Create a checker cube + ground grid + lights in front of the camera.

    The scene is **world aligned**: the ground is a horizontal plane and
    everything stands on it, exactly like the AVM Scene.  The camera's own up
    axis no longer tilts the floor, so a camera that pitches down (an AVM
    fisheye, or a top-down default camera) still gets a level ground instead of
    a wall.

    Returns the created objects.  The object names are unchanged.
    """
    scene = scene or bpy.context.scene
    target = _collection(scene)
    matrix = camera_object.matrix_world.copy()
    subject, floor = layout(camera_object, distance, cube_size)

    if clean:
        for obj in list(target.objects):
            bpy.data.objects.remove(obj, do_unlink=True)

    created: List[bpy.types.Object] = []

    # checker cube straight ahead, sitting on the ground
    bpy.ops.mesh.primitive_cube_add(size=cube_size, location=(0.0, 0.0, 0.0))
    cube = bpy.context.active_object
    cube.name = "CheckerCube"
    cube.location = subject
    cube.data.materials.clear()
    cube.data.materials.append(_checker_material("CheckerCube", 6.0))
    link_to_collection(cube, target)
    created.append(cube)

    # the ground: a horizontal plane under the subject, dropped by 2 mm so it
    # never shares a plane with the cube's bottom face (the AVM Scene's
    # GROUND_DROP does the same)
    bpy.ops.mesh.primitive_plane_add(size=distance * 6.0)
    ground = bpy.context.active_object
    ground.name = GROUND_NAME
    ground.location = (subject.x, subject.y, floor - GROUND_DROP)
    ground.data.materials.clear()
    ground.data.materials.append(
        _checker_material("CheckerGround", 8.0, (0.30, 0.31, 0.33, 1.0), (0.72, 0.73, 0.75, 1.0))
    )
    link_to_collection(ground, target)
    created.append(ground)

    # a few coloured blocks on the ground around the cube, useful to see the warp
    block_size = cube_size * 0.35
    for index, (dx, dy, color) in enumerate((
        (-0.85, 0.35, (0.9, 0.2, 0.15, 1.0)),
        (0.85, 0.35, (0.15, 0.5, 0.9, 1.0)),
        (-0.85, -0.35, (0.9, 0.75, 0.1, 1.0)),
        (0.85, -0.35, (0.2, 0.75, 0.35, 1.0)),
    )):
        bpy.ops.mesh.primitive_cube_add(size=block_size)
        block = bpy.context.active_object
        block.name = f"ColorBlock{index}"
        block.location = (subject.x + dx * distance * 0.8,
                          subject.y + dy * distance * 0.8,
                          floor + block_size * 0.5)
        block.data.materials.clear()
        material = bpy.data.materials.new(f"ColorBlock{index}")
        material.use_nodes = True
        principled = material.node_tree.nodes["Principled BSDF"]
        principled.inputs["Base Color"].default_value = color
        block.data.materials.append(material)
        link_to_collection(block, target)
        created.append(block)

    # headlight style point light at the camera, plus a sun for shape.
    # A sun alone leaves the subjects much too dark for a quick visual check.
    light_data = bpy.data.lights.new("KeyLight", type="POINT")
    light_data.energy = 200.0 * (distance / 4.0) ** 2 * math.pi
    light_data.shadow_soft_size = 0.4 * distance / 4.0
    light = bpy.data.objects.new("KeyLight", light_data)
    light.matrix_world = matrix @ _local_translation(0.0, 0.5, 0.2)
    link_to_collection(light, target)
    created.append(light)

    # a world-fixed sun, like the AVM Scene's: a camera-relative one would swing
    # around (and point sideways) as soon as the camera is pitched
    sun_data = bpy.data.lights.new("SunLight", type="SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("SunLight", sun_data)
    sun.rotation_euler = (math.radians(35.0), 0.0, math.radians(-40.0))
    link_to_collection(sun, target)
    created.append(sun)
    return created


def _local_translation(x: float, y: float, z: float):
    from mathutils import Matrix
    return Matrix.Translation(Vector((x, y, z)))


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
    apply_mod.apply_render_resolution(scene, settings)


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
    view.frame(created, DEFINITION.view)  # show what was just built (no-op headless)
    try:  # keep the camera selected so the panel operators stay valid
        view_layer = bpy.context.view_layer
        for obj in view_layer.objects:
            obj.select_set(False)
        camera_object.select_set(True)
        view_layer.objects.active = camera_object
    except Exception:
        pass
    return True, messages, created


# ---------------------------------------------------------------------------
class OPENCV_CAM_OT_add_camera_scene(bpy.types.Operator):
    """Create a checker cube/ground/lights on a level ground in front of the camera"""

    bl_idname = "opencv_cam.add_camera_scene"
    bl_label = "Camera Scene"
    bl_description = (
        "Create a checker cube, a level ground grid and lights in front of the "
        "camera, apply the current intrinsics and set up Cycles - then press F12 "
        "to look at the distortion. Adds a fisheye camera first when the scene "
        "has none"
    )
    bl_options = {"REGISTER", "UNDO"}

    distance: FloatProperty(name="Distance", default=4.0, min=0.5, max=100.0)
    samples: IntProperty(name="Samples", default=64, min=1, max=4096)

    @classmethod
    def poll(cls, context):
        return getattr(context, "scene", None) is not None

    def execute(self, context):
        obj = camera_factory.resolve_camera(context)
        if obj is None:
            try:
                obj, messages = camera_factory.add_camera(context.scene, model="fisheye")
            except Exception as exc:
                self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
                return {"CANCELLED"}
            for message in messages:
                self.report({"INFO"}, message)
            self.report({"INFO"}, f"no camera in the scene: added {obj.name} first")
        ok, messages, created = apply_and_build(obj, context.scene, samples=self.samples)
        for message in messages:
            self.report({"INFO"} if ok else {"ERROR"}, message)
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"camera scene ready ({len(created)} objects, "
            f"{context.scene.render.resolution_x}x{context.scene.render.resolution_y}) - press F12",
        )
        return {"FINISHED"}


class OPENCV_CAM_PT_camera_scene(bpy.types.Panel):
    """3D viewport sidebar entry: the scene has no settings, only its view.

    The Camera Scene has no root empty, so it cannot use :class:`ScenePanel`
    (whose poll is the root pointer); the collection's presence is the
    "is it built?" test instead.
    """

    bl_idname = "OPENCV_CAM_PT_camera_scene"
    bl_label = "Camera Scene"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Camera Scene"

    @classmethod
    def poll(cls, context):
        return collection(COLLECTION_NAME) is not None

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator("opencv_cam.frame_view", text="Frame View",
                     icon="VIEW_PERSPECTIVE").scene_id = DEFINITION.id
        layout.label(text="Default view: 45 deg azimuth, 30 deg elevation", icon="INFO")


_CLASSES = (OPENCV_CAM_OT_add_camera_scene, OPENCV_CAM_PT_camera_scene)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
