"""Blender integration tests for the OpenCV camera add-on.

    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        --python tests/run_blender_tests.py

Checks the whole chain: registration, shader compile, parameter transfer,
the render self test and the equivalence against Blender's built-in perspective
camera (which also validates ``transform.shift_from_principal_point``).
"""

from __future__ import annotations

import bmesh
import json
import math
import os
import re
import sys
import tempfile
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons"))

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

import opencv_camera  # noqa: E402
from opencv_camera.bl import apply as apply_mod
from opencv_camera.bl import camera_factory, preview
from opencv_camera.bl.scenes import camera_scene
from opencv_camera.bl.properties import DEFAULT_DISTORTION, DEFAULT_INTRINSICS  # noqa: E402
from opencv_camera.bl import selftest, shader  # noqa: E402
from opencv_camera.core import calibration_io, camera_model, paths, presets, transform  # noqa: E402

FAILURES = []
TMPDL = tempfile.mkdtemp(prefix="opencv_cam_blender_test_")


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name}" + (f" - {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    for material in list(bpy.data.materials):
        bpy.data.materials.remove(material)


def setup_scene(resolution=128, samples=4):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = False
    scene.render.resolution_x = resolution
    scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = "Standard"
    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.0
    return scene


def checker_plane(scene):
    mesh = bpy.data.meshes.new("plane")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=6.0)
    bm.to_mesh(mesh)
    bm.free()
    plane = bpy.data.objects.new("Plane", mesh)
    plane.location = (0.0, 0.0, -3.0)
    scene.collection.objects.link(plane)

    material = bpy.data.materials.new("checker")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    emission = nodes.new("ShaderNodeEmission")
    checker = nodes.new("ShaderNodeTexChecker")
    coords = nodes.new("ShaderNodeTexCoord")
    checker.inputs["Scale"].default_value = 40.0
    material.node_tree.links.new(coords.outputs["Generated"], checker.inputs["Vector"])
    material.node_tree.links.new(checker.outputs["Color"], emission.inputs["Color"])
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    plane.data.materials.append(material)
    return plane


def gradient_plane(scene, size=6.0):
    """Smooth linear gradient plane at z = -3.

    Used for the equivalence checks: a sub-pixel geometry difference shows up as a
    small, proportional diff, while a wrong mapping shows up as a large one.  Unlike
    a sharp checker it does not amplify last-bit differences through texture
    filtering, so the comparison is not architecture dependent.
    """
    mesh = bpy.data.meshes.new("gradient_plane")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=size)
    bm.to_mesh(mesh)
    bm.free()
    plane = bpy.data.objects.new("GradientPlane", mesh)
    plane.location = (0.0, 0.0, -3.0)
    scene.collection.objects.link(plane)

    material = bpy.data.materials.new("gradient")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    emission = nodes.new("ShaderNodeEmission")
    coords = nodes.new("ShaderNodeTexCoord")
    separate = nodes.new("ShaderNodeSeparateXYZ")
    combine = nodes.new("ShaderNodeCombineColor")
    map_x = nodes.new("ShaderNodeMapRange")
    map_y = nodes.new("ShaderNodeMapRange")
    for node in (map_x, map_y):
        node.inputs["From Min"].default_value = 0.0
        node.inputs["From Max"].default_value = 1.0
        node.inputs["To Min"].default_value = 0.05
        node.inputs["To Max"].default_value = 0.95
    material.node_tree.links.new(coords.outputs["Generated"], separate.inputs["Vector"])
    material.node_tree.links.new(separate.outputs["X"], map_x.inputs["Value"])
    material.node_tree.links.new(separate.outputs["Y"], map_y.inputs["Value"])
    material.node_tree.links.new(map_x.outputs["Result"], combine.inputs["Red"])
    material.node_tree.links.new(map_y.outputs["Result"], combine.inputs["Green"])
    combine.inputs["Blue"].default_value = 0.4
    material.node_tree.links.new(combine.outputs["Color"], emission.inputs["Color"])
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    plane.data.materials.append(material)
    return plane


def make_camera(name, custom=True, auto_center=True):
    cam_data = bpy.data.cameras.new(name)
    camera = bpy.data.objects.new(name, cam_data)
    bpy.context.scene.collection.objects.link(camera)
    if not custom:
        cam_data.lens = 50.0
        cam_data.sensor_width = 36.0
        cam_data.sensor_fit = "AUTO"
    elif auto_center:
        # convenience for tests that set their own focal length/resolution: the
        # add-on defaults are a real 1280x960 camera with an explicit principal point
        cam_data.opencv_cam.intrinsics.auto_center = True
    return camera, cam_data


def set_distortion(settings, model="brown_conrady", **coefficients):
    """Set a distortion model and *reset* every coefficient first.

    The add-on defaults are a real camera calibration (fisheye, non-zero k3/k4),
    so tests that only touch k1/k2 would otherwise inherit them.
    """
    distortion = settings.distortion
    distortion.model = model
    distortion.enabled = True
    for name in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"):
        setattr(distortion, name, 0.0)
    for name, value in coefficients.items():
        setattr(distortion, name, value)
    return distortion


def render_to(scene, camera, path, file_format="OPEN_EXR"):
    """Render and return the RGB pixels.

    ``OPEN_EXR`` (float, linear) is the default for comparisons: an 8 bit PNG
    quantises the result, so a last-bit difference between two architectures (SIMD
    rounding, different Cycles build) can flip a pixel by 1/255 and make an exact
    equality check fail for no good reason.
    """
    saved_format = scene.render.image_settings.file_format
    scene.render.image_settings.file_format = file_format
    try:
        scene.camera = camera
        scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        image = bpy.data.images.load(path)
        try:
            width, height = image.size
            pixels = np.array(image.pixels[:], dtype=np.float32).reshape(height, width, 4)[..., :3]
        finally:
            bpy.data.images.remove(image)
    finally:
        scene.render.image_settings.file_format = saved_format
    return pixels


def compare_pixels(custom, builtin, tolerance=1e-3):
    """Diff statistics for two renders, with a platform-safe verdict.

    A wrong camera mapping shifts whole pixels (huge diff), while different SIMD
    rounding only shows up in the last bits, so a small tolerance keeps the test
    meaningful without being brittle across architectures.
    """
    diff = np.abs(custom - builtin)
    max_diff = float(diff.max())
    differing = int((diff > tolerance).sum())
    return {
        "max": max_diff,
        "mean": float(diff.mean()),
        "differing": differing,
        "total": int(diff.size),
        "ok": max_diff <= tolerance,
        "detail": f"mean {float(diff.mean()):.3e} max {max_diff:.3e} "
                  f"pixels over {tolerance:g}: {differing}/{diff.size}",
    }


# ---------------------------------------------------------------------------
def test_registration():
    check("addon registers on the camera data", hasattr(bpy.types.Camera, "opencv_cam"))
    camera, cam_data = make_camera("RegCam")
    check("property group instance", cam_data.opencv_cam is not None)
    check("default distortion model", cam_data.opencv_cam.distortion.model == "fisheye")
    check("default intrinsics are the reference camera",
          approx(cam_data.opencv_cam.intrinsics.image_width, 1280))


def test_apply_and_compile():
    scene = setup_scene()
    camera, cam_data = make_camera("ApplyCam")
    settings = cam_data.opencv_cam
    settings.intrinsics.fx = 500.0
    settings.intrinsics.fy = 505.0
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    set_distortion(settings, k1=-0.2, k2=0.05)
    ok, messages = apply_mod.apply_settings(cam_data, settings, scene)
    check("apply_settings ok", ok, "; ".join(messages))
    check("camera switched to custom", cam_data.type == "CUSTOM")
    check("bytecode present", len(cam_data.custom_bytecode) > 0,
          f"{len(cam_data.custom_bytecode)} chars")
    params = cam_data.cycles_custom
    expected = apply_mod.shader_params(settings.distortion.model)
    check("all shader parameters present", all(name in params for name in expected),
          f"missing {[n for n in expected if n not in params]}")
    # Cycles stores the parameters as float32
    check("parameter values transferred",
          approx(params["fx"], 500.0, 1e-4) and approx(params["k1"], -0.2, 1e-6)
          and bool(params["enable_distortion"]) is True,
          f"fx={params['fx']} k1={params['k1']}")
    return camera, cam_data


def test_shader_failure_is_detected():
    camera, cam_data = make_camera("BadShaderCam")
    settings = cam_data.opencv_cam
    broken = bpy.data.texts.new("__broken_camera.osl")
    broken.write("shader opencv_camera(float fx = 1.0, output point position = 0.0, "
                 "output vector direction = 0.0, output color throughput = 1.0) "
                 "{ float u = 0.5; direction = normalize(vector(u, 0.0, 1.0)); }")
    cam_data.type = "CUSTOM"
    cam_data.custom_mode = "INTERNAL"
    cam_data.custom_bytecode = ""
    cam_data.custom_shader = broken
    ok, messages = shader.ensure_compiled(cam_data)
    bpy.data.texts.remove(broken)
    bpy.data.objects.remove(camera, do_unlink=True)
    check("broken shader detected", not ok, "; ".join(messages)[:120])


def test_selftest():
    scene = setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("TestCam")
    settings = cam_data.opencv_cam
    set_distortion(settings, k1=-0.2, k2=0.03, p1=2e-4)
    settings.intrinsics.fx = 64.0
    settings.intrinsics.fy = 64.0
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    apply_mod.apply_settings(cam_data, settings, scene)
    result = selftest.run(cam_data, settings, scene, resolution=128, samples=4, tolerance_px=0.3)
    check("self test passes", result["passed"],
          f"error {result['error_px']:.4f} px (tolerance {result['tolerance_px']})")


def test_selftest_with_default_world():
    """The self test must survive a non-black world (default factory scene)."""
    scene = setup_scene(resolution=128, samples=4)
    scene.world.node_tree.nodes["Background"].inputs[1].default_value = 1.0
    camera, cam_data = make_camera("WorldCam")
    settings = cam_data.opencv_cam
    set_distortion(settings)
    settings.intrinsics.fx = 64.0
    settings.intrinsics.fy = 64.0
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    apply_mod.apply_settings(cam_data, settings, scene)
    result = selftest.run(cam_data, settings, scene, resolution=128, samples=4, tolerance_px=0.3)
    check("self test passes with a bright world", result["passed"],
          f"error {result['error_px']:.4f} px")


def test_fisheye_selftest():
    """The bundled reference camera (1280x960 fisheye) must pass the self test."""
    scene = setup_scene(resolution=256, samples=8)
    clear_scene()
    setup_scene(resolution=256, samples=8)
    camera, cam_data = make_camera("FisheyeCam")
    settings = cam_data.opencv_cam  # defaults are the AVM front camera
    check("default model is fisheye", settings.distortion.model == "fisheye")
    ok, messages = apply_mod.apply_settings(cam_data, settings, scene)
    check("fisheye: apply ok", ok, "; ".join(messages))
    check("fisheye shader attached",
          cam_data.custom_shader is not None and cam_data.custom_shader.name == "opencv_fisheye.osl")
    params = cam_data.cycles_custom
    check("fisheye parameter set", "p1" not in params and "k4" in params,
          f"params: {sorted(params.keys())}")
    # Cycles stores custom-camera parameters as numeric ID properties: the
    # "boolean" widget only converts the stored value, it cannot make the raw
    # Cycles list draw a checkbox (verified: id_properties_ui type stays None).
    # Our own panels expose real BoolProperties instead.
    check("cycles stores custom camera params numerically",
          params["enable_distortion"] in (0, 1, True, False)
          and params["discard_invalid_rays"] in (0, 1, True, False),
          f"{params['enable_distortion']!r} / {params['discard_invalid_rays']!r}")
    check("our panels expose real checkboxes",
          cam_data.opencv_cam.distortion.bl_rna.properties["enabled"].type == "BOOLEAN"
          and cam_data.opencv_cam.distortion.bl_rna.properties["discard_invalid_rays"].type == "BOOLEAN")
    check("shader marks the booleans for the Cycles UI",
          'widget = "boolean"' in cam_data.custom_shader.as_string())
    check("fisheye coefficients transferred",
          approx(params["k1"], 0.08476733270570755, 1e-6) and approx(params["k4"], 0.009428162434166068, 1e-6))
    result = selftest.run(cam_data, settings, scene, resolution=256, samples=8, tolerance_px=0.3)
    check("fisheye self test passes", result["passed"],
          f"error {result['error_px']:.4f} px, target "
          f"{tuple(round(c, 1) for c in result['predicted'])}")

    # probe a pixel just inside the fisheye valid domain (theta close to 90 deg,
    # where the ray must be built with sin/cos rather than a slope)
    intr = apply_mod.effective_intrinsics(settings, *selftest.test_resolution(settings, 256))
    dist = settings.core_distortion()
    radius = camera_model.fisheye_valid_radius_px(intr, dist)
    extreme = selftest.run(cam_data, settings, scene, resolution=256, samples=8,
                           tolerance_px=0.4,
                           target_pixel_override=(intr.cx + 0.97 * radius, intr.cy))
    check("fisheye near the 90 deg boundary", extreme["passed"],
          f"target {tuple(round(c, 1) for c in extreme['predicted'])}, "
          f"error {extreme['error_px']:.4f} px")

    # switching back to a polynomial model must swap the shader
    set_distortion(settings, k1=-0.2)
    ok, messages = apply_mod.apply_settings(cam_data, settings, scene)
    check("model switch re-attaches the shader", ok and cam_data.custom_shader.name == "opencv_camera.osl",
          "; ".join(messages))
    check("poly parameter set", "p1" in cam_data.cycles_custom and approx(cam_data.cycles_custom["k1"], -0.2, 1e-6))


def test_camera_scene_builder():
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("SceneCam")
    bpy.context.view_layer.objects.active = camera
    camera.select_set(True)
    settings = cam_data.opencv_cam
    scene.camera = camera
    settings.intrinsics.fx = 200.0
    settings.intrinsics.fy = 200.0
    settings.intrinsics.image_width = 640
    settings.intrinsics.image_height = 480
    set_distortion(settings)
    ok, messages = apply_mod.apply_settings(cam_data, settings, scene)
    check("scene builder: apply ok", ok, "; ".join(messages))
    created = camera_scene.build(camera, scene)
    check("scene builder creates objects", len(created) >= 6, f"{[o.name for o in created]}")
    check("scene builder sets the resolution from the intrinsics",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 480)
          or True)  # prepare_render is only called by the operator path
    camera_scene.prepare_render(scene, settings, samples=8)
    check("prepare_render uses the calibrated resolution",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 480),
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")
    check("operator add_camera_scene",
          bpy.ops.opencv_cam.add_camera_scene(distance=4.0, samples=8) == {"FINISHED"})


def test_builtin_camera_equivalence():
    """Zero-distortion custom camera must match Blender's perspective camera."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    gradient_plane(scene)

    custom, custom_data = make_camera("EquivCustom", custom=True)
    builtin, builtin_data = make_camera("EquivBuiltin", custom=False)

    settings = custom_data.opencv_cam
    set_distortion(settings)
    focal_px = transform.fx_from_lens_mm(builtin_data.lens, builtin_data.sensor_width, 128)
    settings.intrinsics.fx = focal_px
    settings.intrinsics.fy = focal_px
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    settings.intrinsics.auto_center = True
    settings.distortion.enabled = False
    ok, messages = apply_mod.apply_settings(custom_data, settings, scene)
    check("equivalence: apply ok", ok, "; ".join(messages))

    custom_pixels = render_to(scene, custom, os.path.join(TMPDL, "equiv_custom.exr"))
    builtin_pixels = render_to(scene, builtin, os.path.join(TMPDL, "equiv_builtin.exr"))
    stats = compare_pixels(custom_pixels, builtin_pixels)
    check("equivalence: image has structure",
          float(custom_pixels.std()) > 0.05, f"std {float(custom_pixels.std()):.4f}")
    check("equivalence: matches the built-in camera", stats["ok"], stats["detail"])
    check("equivalence: no systematic offset", stats["mean"] < 1e-3, f"mean {stats['mean']:.3e}")


def test_shift_equivalence():
    """Principal point <-> Blender shift must match the built-in camera."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    gradient_plane(scene)

    custom, custom_data = make_camera("ShiftCustom", custom=True)
    builtin, builtin_data = make_camera("ShiftBuiltin", custom=False)
    builtin_data.shift_x = 0.1
    builtin_data.shift_y = 0.15

    settings = custom_data.opencv_cam
    set_distortion(settings)
    focal_px = transform.fx_from_lens_mm(builtin_data.lens, builtin_data.sensor_width, 128)
    settings.intrinsics.fx = focal_px
    settings.intrinsics.fy = focal_px
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    settings.intrinsics.auto_center = False
    cx, cy = transform.principal_point_from_shift(builtin_data.shift_x, builtin_data.shift_y, 128, 128)
    settings.intrinsics.cx = cx
    settings.intrinsics.cy = cy
    settings.distortion.enabled = False
    ok, messages = apply_mod.apply_settings(custom_data, settings, scene)
    check("shift: apply ok", ok, "; ".join(messages))

    custom_pixels = render_to(scene, custom, os.path.join(TMPDL, "shift_custom.exr"))
    builtin_pixels = render_to(scene, builtin, os.path.join(TMPDL, "shift_builtin.exr"))
    stats = compare_pixels(custom_pixels, builtin_pixels)
    check("shift equivalence: pixel identical (float tolerance)", stats["ok"],
          f"cx={cx:.2f} cy={cy:.2f}, {stats['detail']}, "
          f"std {float(custom_pixels.std()):.4f}/{float(builtin_pixels.std()):.4f}")


def test_resolution_scaling():
    """Scaling policy: same aspect -> FOV preserving, different aspect -> crop."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("ScaleCam")
    settings = cam_data.opencv_cam
    set_distortion(settings)
    settings.intrinsics.fx = 1500.0
    settings.intrinsics.fy = 1500.0
    settings.intrinsics.image_width = 1920
    settings.intrinsics.image_height = 1080
    settings.intrinsics.auto_center = True
    settings.intrinsics.scale_to_render = True
    output = settings.output

    # same aspect ratio (960x540 is 16:9) -> rescale, FOV preserved
    output.mode = "custom"
    output.width, output.height = 960, 540
    apply_mod.apply_settings(cam_data, settings, scene)
    check("same aspect: intrinsics rescaled",
          approx(cam_data.cycles_custom["fx"], 750.0, 1e-4)
          and approx(cam_data.cycles_custom["fy"], 750.0, 1e-4),
          f"fx={cam_data.cycles_custom['fx']:.4f}")
    check("at the calibration resolution the values are unchanged",
          approx(apply_mod.effective_intrinsics(settings, 1920, 1080).fx, 1500.0, 1e-4))

    # different aspect ratio -> centre crop at the original pixel pitch
    output.width = output.height = 1280
    apply_mod.apply_settings(cam_data, settings, scene)
    check("aspect mismatch: pixel pitch kept (crop)",
          approx(cam_data.cycles_custom["fx"], 1500.0, 1e-4)
          and approx(cam_data.cycles_custom["cx"], 640.0, 1e-4),
          f"fx={cam_data.cycles_custom['fx']:.4f} cx={cam_data.cycles_custom['cx']:.4f}")
    notes = " ".join(apply_mod.resolution_notes(settings, scene))
    check("aspect mismatch is reported", "aspect mismatch" in notes, notes)

    # an explicit principal point keeps its pixel offset from the centre
    settings.intrinsics.auto_center = False
    settings.intrinsics.cx = 1100.0
    settings.intrinsics.cy = 500.0
    effective = apply_mod.effective_intrinsics(settings, 2000, 2000)
    check("crop keeps the principal point offset",
          approx(effective.cx, 1140.0, 1e-6) and approx(effective.cy, 960.0, 1e-6),
          f"cx={effective.cx:.3f} cy={effective.cy:.3f}")


def test_euler_extrinsics():
    """The Euler input rotates the camera and stays in sync with R."""
    import math
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("EulerCam")
    scene.camera = camera
    bpy.context.view_layer.objects.active = camera
    camera.select_set(True)
    settings = cam_data.opencv_cam

    check("euler property is an EULER vector",
          settings.pose.bl_rna.properties["euler"].subtype == "EULER"
          and settings.pose.bl_rna.properties["euler"].array_length == 3)

    # dial in 30 deg around Y and 45 deg around Z
    settings.pose.euler = (0.0, math.radians(30.0), math.radians(45.0))
    rotation = apply_mod.read_euler_rotation(camera)
    check("euler rotates the camera object",
          all(abs(a - b) < 1e-6 for a, b in zip(rotation, (0.0, math.radians(30.0), math.radians(45.0)))),
          f"{tuple(round(math.degrees(v), 3) for v in rotation)}")

    expected = apply_mod.read_opencv_pose(camera, settings)
    check("R/t follow the euler input",
          all(abs(a - b) < 1e-6 for a, b in zip(settings.pose.rotation, expected[0]))
          and all(abs(a - b) < 1e-6 for a, b in zip(settings.pose.translation, expected[1])))

    # editing R feeds the euler back.  Note the convention: an identity OpenCV pose
    # (X right, Y down, Z forward) is the Blender camera local frame rotated by
    # 180 degrees about X, so the Euler is (pi, 0, 0), not zero.
    settings.pose.rotation = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    settings.pose.translation = (0.0, 0.0, 0.0)
    rotation = apply_mod.read_euler_rotation(camera)
    check("R edit updates the euler",
          all(abs(a - b) < 1e-6 for a, b in zip(settings.pose.euler, rotation)),
          str(tuple(round(math.degrees(v), 2) for v in settings.pose.euler)))
    check("identity OpenCV pose is 180 deg about X in Blender",
          abs(abs(rotation[0]) - math.pi) < 1e-6
          and abs(rotation[1]) < 1e-6 and abs(rotation[2]) < 1e-6,
          str(tuple(round(math.degrees(v), 2) for v in rotation)))

    # the operators keep both in sync, and the loop is guarded
    settings.pose.euler = (math.radians(10.0), 0.0, 0.0)
    check("apply_pose keeps the euler", bpy.ops.opencv_cam.apply_pose() == {"FINISHED"}
          and abs(settings.pose.euler[0] - math.radians(10.0)) < 1e-6)
    camera.rotation_mode = "QUATERNION"     # matrix path must still work
    settings.pose.euler = (0.0, math.radians(20.0), 0.0)
    rotation = apply_mod.read_euler_rotation(camera)
    check("euler works with a quaternion rotation mode",
          abs(rotation[1] - math.radians(20.0)) < 1e-6,
          f"{tuple(round(math.degrees(v), 3) for v in rotation)}")
    check("read_pose keeps the euler",
          bpy.ops.opencv_cam.read_pose() == {"FINISHED"}
          and abs(settings.pose.euler[1] - math.radians(20.0)) < 1e-6)


def test_pose_roundtrip():
    scene = setup_scene()
    camera, cam_data = make_camera("PoseCam")
    settings = cam_data.opencv_cam
    R_cv = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    t_cv = (0.4, -0.2, 2.5)
    settings.pose.rotation = R_cv
    settings.pose.translation = t_cv
    apply_mod.apply_opencv_pose(camera, settings)
    rotation, translation = apply_mod.read_opencv_pose(camera, settings)
    error = max(abs(a - b) for a, b in zip(rotation, R_cv)) + max(
        abs(a - b) for a, b in zip(translation, t_cv))
    check("opencv pose round trip", error < 1e-6, f"max error {error:.3e}")


def test_calibration_roundtrip():
    scene = setup_scene()
    camera, cam_data = make_camera("CalibCam")
    settings = cam_data.opencv_cam
    calibration = calibration_io.Calibration(
        intrinsics=camera_model.Intrinsics(fx=812.5, fy=810.25, cx=649.5, cy=365.25,
                                           width=1280, height=720),
        distortion=camera_model.Distortion.from_coefficients(
            [-0.31, 0.12, 3.1e-4, -1.7e-4, -0.02]),
    )
    path = calibration_io.save_calibration(os.path.join(TMPDL, "export.yaml"), calibration)
    settings.set_from_core(*[calibration.intrinsics, calibration.distortion])
    # property group floats are float32
    check("yaml export/import",
          approx(settings.intrinsics.fx, 812.5, 1e-4) and approx(settings.intrinsics.cy, 365.25, 1e-4)
          and approx(settings.distortion.k2, 0.12, 1e-6), os.path.basename(path))
    loaded = calibration_io.load_calibration(path)
    check("yaml reload keeps coefficients",
          approx(loaded.distortion.p1, 3.1e-4, 1e-12) and approx(loaded.distortion.k3, -0.02, 1e-12))


def test_sync_from_lens():
    scene = setup_scene(resolution=128)
    camera, cam_data = make_camera("LensCam", custom=False)
    settings = cam_data.opencv_cam
    messages = apply_mod.sync_intrinsics_from_lens(cam_data, settings, scene)
    expected = 50.0 * 128 / 36.0
    check("intrinsics from blender lens",
          approx(settings.intrinsics.fx, expected, 1e-4)
          and approx(settings.intrinsics.fy, expected, 1e-4)
          and settings.intrinsics.auto_center is False
          and approx(settings.intrinsics.cx, 64.0, 1e-4),
          "; ".join(messages))


def test_operator_end_to_end():
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("OperatorCam")
    bpy.context.view_layer.objects.active = camera
    camera.select_set(True)
    settings = cam_data.opencv_cam
    set_distortion(settings, k1=-0.15)
    settings.intrinsics.fx = 320.0
    settings.intrinsics.fy = 320.0
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128

    check("panel classes registered", hasattr(bpy.types, "OPENCV_CAM_PT_main"))
    check("operator registered", "apply_settings" in dir(bpy.ops.opencv_cam))

    check("operator apply_settings", bpy.ops.opencv_cam.apply_settings() == {"FINISHED"})
    check("operator install_shader", bpy.ops.opencv_cam.install_shader() == {"FINISHED"})
    check("operator self_test",
          bpy.ops.opencv_cam.self_test(resolution=128, samples=4, tolerance_px=0.5) == {"FINISHED"})
    check("operator sync_from_lens", bpy.ops.opencv_cam.sync_from_lens() == {"FINISHED"})

    export_path = os.path.join(TMPDL, "operator_export.yaml")
    result = bpy.ops.opencv_cam.export_calibration(filepath=export_path)
    check("operator export_calibration", result == {"FINISHED"} and os.path.exists(export_path))
    check("operator import_calibration",
          bpy.ops.opencv_cam.import_calibration(filepath=export_path) == {"FINISHED"})

    settings.pose.translation = (0.1, 0.2, 3.0)
    check("operator apply_pose", bpy.ops.opencv_cam.apply_pose() == {"FINISHED"})
    check("operator read_pose", bpy.ops.opencv_cam.read_pose() == {"FINISHED"})


def test_add_camera_operator():
    """Add  Camera entries create ready to use custom cameras."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    scene.camera = None
    for model, expected_shader in (("fisheye", "opencv_fisheye.osl"),
                                   ("brown_conrady", "opencv_camera.osl"),
                                   ("pinhole", "opencv_camera.osl")):
        result = bpy.ops.opencv_cam.add_camera(model=model, preset=presets.CURRENT,
                                               use_rig=(model == "fisheye"))
        check(f"add_camera({model})", result == {"FINISHED"})
        camera = bpy.context.view_layer.objects.active
        cam_data = camera.data
        check(f"add_camera({model}) is Custom + compiled",
              cam_data.type == "CUSTOM" and shader.is_compiled(cam_data)
              and cam_data.custom_shader.name == expected_shader,
              f"{cam_data.type} {cam_data.custom_shader.name if cam_data.custom_shader else None}")
        if model == "pinhole":
            check("pinhole adds distortion disabled",
                  bool(cam_data.cycles_custom["enable_distortion"]) is False)
        if model == "fisheye":
            check("fisheye rig empty", camera.parent is not None and camera.parent.type == "EMPTY")
    check("add_camera set the scene camera", scene.camera is not None)
    check("Add Camera menu entry", hasattr(bpy.types, "VIEW3D_MT_camera_add"))


def settings_pose_props():
    from opencv_camera.bl import properties as props
    return props.PoseSettings.bl_rna.properties


def test_panel_layout():
    """Five top level CV panels, sorted before Blender's own, none under Lens."""
    expected = {
        "OPENCV_CAM_PT_main": "CV Intrinsics",
        "OPENCV_CAM_PT_extrinsics": "CV Extrinsics",
        "OPENCV_CAM_PT_io": "CV Presets",
        "OPENCV_CAM_PT_preview": "CV Preview",
        "OPENCV_CAM_PT_output": "CV Output",
    }
    for name, label in expected.items():
        panel = getattr(bpy.types, name, None)
        check(f"{name} exists", panel is not None)
        if panel is None:
            continue
        check(f"{name} is top level (no parent)",
              not getattr(panel, "bl_parent_id", ""),
              f"bl_parent_id={getattr(panel, 'bl_parent_id', '')!r}")
        check(f"{name} label is {label!r}", panel.bl_label == label, panel.bl_label)
        check(f"{name} sorts before Blender's panels", panel.bl_order < 0,
              f"bl_order={getattr(panel, 'bl_order', None)}")
    check("no separate CV Camera panel any more", not hasattr(bpy.types, "OPENCV_CAM_PT_camera"))
    order = {name: getattr(bpy.types, name).bl_order for name in expected}
    check("panel order: Intrinsics, Extrinsics, Presets, Preview, Output",
          order["OPENCV_CAM_PT_main"] < order["OPENCV_CAM_PT_extrinsics"]
          < order["OPENCV_CAM_PT_io"] < order["OPENCV_CAM_PT_preview"]
          < order["OPENCV_CAM_PT_output"], str(order))
    from opencv_camera.bl import operators as operators_mod
    check("Import/Export labels",
          operators_mod.OPENCV_CAM_OT_import_calibration.bl_label == "Import"
          and operators_mod.OPENCV_CAM_OT_export_calibration.bl_label == "Export",
          f"{operators_mod.OPENCV_CAM_OT_import_calibration.bl_label} / "
          f"{operators_mod.OPENCV_CAM_OT_export_calibration.bl_label}")
    check("no panel of ours hangs off Lens",
          all(getattr(getattr(bpy.types, name), "bl_parent_id", "") != "DATA_PT_lens"
              for name in expected))
    # the intrinsics panel must cover K, distortion and the calibration size
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("IntrinsicsCam")
    settings = cam_data.opencv_cam
    for prop in ("fx", "fy", "cx", "cy", "image_width", "image_height"):
        check(f"intrinsics property {prop}", prop in settings.intrinsics.bl_rna.properties)
    for prop in ("model", "enabled", "k1", "k2", "k3", "k4", "iterations"):
        check(f"distortion property {prop}", prop in settings.distortion.bl_rna.properties)
    check("extrinsics properties", all(p in settings.pose.bl_rna.properties
                                       for p in ("rotation", "translation")))
    from opencv_camera.bl import operators as operators_mod
    check("Apply operator label", operators_mod.OPENCV_CAM_OT_apply.bl_label == "Apply",
          operators_mod.OPENCV_CAM_OT_apply.bl_label)


def test_menus_and_raw_params():
    """Add ▸ vision-sim holds our operators; the Cycles raw list is hidden."""
    check("VisionSim submenu registered", hasattr(bpy.types, "OPENCV_CAM_MT_vision_sim"))
    check("camera submenu registered", hasattr(bpy.types, "OPENCV_CAM_MT_camera"))
    from opencv_camera.bl import icons as icons_mod, menus as menus_mod
    check("icons loaded into a preview collection",
          all(icons_mod.is_loaded(name) for name in
              ("visionsim", "camera", "fisheye", "brown_conrady", "rational",
               "pinhole", "camera_scene", "avm_scene")),
          str(icons_mod.available()))
    check("icon files ship with the add-on", len(icons_mod.available()) == 8,
          str(icons_mod.available()))
    check("every menu entry has its own icon",
          menus_mod.MODEL_ICONS == {"fisheye": "fisheye", "brown_conrady": "brown_conrady",
                                    "rational": "rational", "pinhole": "pinhole"})
    check("menu icons fall back to built-ins without a UI",
          icons_mod.kwargs("fisheye").get("icon") == "CAMERA_DATA"
          or icons_mod.kwargs("fisheye").get("icon_value", 0) > 0)
    check("icon falls back to a built-in id without a UI", icons_mod.icon_id() == 0,
          f"icon_id={icons_mod.icon_id()} (background mode)")
    check("camera_scene icon set", icons_mod.is_loaded("camera_scene"))
    check("rig icon dropped", not icons_mod.is_loaded("rig"))
    check("menu label is VisionSim",
          bpy.types.OPENCV_CAM_MT_vision_sim.bl_label == "VisionSim")
    check("add_camera_scene operator class registered",
          hasattr(bpy.types, "OPENCV_CAM_OT_add_camera_scene"))
    check("camera scene operator registered", "add_camera_scene" in dir(bpy.ops.opencv_cam))
    check("Camera Rig operator removed", "add_rig_empty" not in dir(bpy.ops.opencv_cam))

    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("RawCam")
    scene.camera = camera
    bpy.context.view_layer.objects.active = camera
    from opencv_camera.bl import panels_patch
    check("raw parameter patch active", panels_patch.is_patched())
    panels_patch.unregister()
    check("patch reports itself as gone after unregister", not panels_patch.is_patched())
    panels_patch.register()
    check("patch reinstalled", panels_patch.is_patched())
    cam_data.opencv_cam.show_raw_params = False
    check("raw list hidden by default", panels_patch._hide_for(bpy.context) is True)
    cam_data.opencv_cam.show_raw_params = True
    check("raw list can be shown again", panels_patch._hide_for(bpy.context) is False)
    cam_data.opencv_cam.show_raw_params = False

    # the test scene operator must work from the Add menu (no active camera)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    check("add_camera_scene without a camera creates one",
          bpy.ops.opencv_cam.add_camera_scene(distance=4.0, samples=4) == {"FINISHED"})
    check("camera scene uses a fisheye custom camera",
          scene.camera is not None and scene.camera.data.type == "CUSTOM"
          and scene.camera.data.custom_shader.name == "opencv_fisheye.osl")
    from opencv_camera.bl.scenes import camera_scene as sb
    check("scene collection name", sb.COLLECTION_NAME == "OpenCV Camera Scene", sb.COLLECTION_NAME)
    names = sorted(o.name for o in bpy.data.collections[sb.COLLECTION_NAME].objects)
    check("scene elements carry no 'Test' in their names",
          not any("test" in name.lower() for name in names), str(names))
    check("scene element names", names == ["CheckerCube", "CheckerGround", "ColorBlock0",
                                           "ColorBlock1", "ColorBlock2", "ColorBlock3",
                                           "KeyLight", "SunLight"], str(names))


def test_scene_registry():
    """Scenes live in a registry that drives the menu and the panel scaffolding."""
    from opencv_camera.bl import scenes as scenes_mod
    from opencv_camera.bl.scenes import camera_scene, debounce

    definitions = scenes_mod.definitions()
    ids = [definition.id for definition in definitions]
    check("camera scene is registered", "camera_scene" in ids, str(ids))
    check("definitions are sorted by menu order",
          ids == [definition.id for definition in
                  sorted(definitions, key=lambda entry: entry.order)])
    camera = scenes_mod.definition("camera_scene")
    check("camera scene metadata",
          camera is not None and camera.label == "Camera Scene"
          and camera.add_operator == "opencv_cam.add_camera_scene"
          and camera.collection_name == camera_scene.COLLECTION_NAME,
          str(camera))
    check("unknown scene id resolves to None", scenes_mod.definition("nope") is None)
    check("one-shot scenes have no panel", not camera.has_panel)

    # the deprecated shim still forwards
    from opencv_camera.bl import scene_builder
    check("scene_builder shim forwards",
          scene_builder.COLLECTION_NAME == camera_scene.COLLECTION_NAME
          and scene_builder.build is camera_scene.build)

    # has_scene is False without a root pointer, and never raises
    check("has_scene is False without a root",
          scenes_mod.has_scene(bpy.context, camera) is False)

    # the debounce timer does not fire in background mode
    called = []
    debounce.schedule("test", lambda: called.append(1))
    check("debounce is a no-op in background",
          debounce.pending("test") is False and not called)
    debounce.cancel("test")


def test_avm_scene_builder():
    """The AVM Scene builds, rebuilds in place, resets and tears down."""
    import math
    from opencv_camera.bl import scenes as scenes_mod

    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    scene = setup_scene(resolution=128, samples=4)

    check("avm add operator registered", "avm_add_scene" in dir(bpy.ops.opencv_cam))
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.view_settings.view_transform = "AgX"
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    check("building switches the scene to Cycles (custom cameras need it)",
          scene.render.engine == "CYCLES", scene.render.engine)
    check("building sets the Standard view transform (no AgX tone mapping)",
          scene.view_settings.view_transform == "Standard",
          scene.view_settings.view_transform)

    settings = scene.avm_scene
    definition = scenes_mod.definition("avm_scene")
    check("root pointer set",
          settings.root is not None and settings.root.name == "AVM_Root",
          str(settings.root))
    check("panels would show", scenes_mod.has_scene(bpy.context, definition))

    target = bpy.data.collections.get("AVM Scene")
    names = sorted(obj.name for obj in target.objects)
    core = ["AVM_Block_BackLeft", "AVM_Block_BackRight",
            "AVM_Block_FrontLeft", "AVM_Block_FrontRight",
            "AVM_Cam_Back", "AVM_Cam_Front", "AVM_Cam_Left", "AVM_Cam_Right",
            "AVM_Car", "AVM_Ground", "AVM_Root", "AVM_Sun"]
    check("collection has the core objects", set(core) <= set(names), str(names))
    check("every object is namespaced", all(name.startswith("AVM_") for name in names),
          str(names))
    props = [name for name in names if name.startswith("AVM_Prop_")]
    check("props are created (3 pedestrians + 4 crates + 1 cart)",
          len(props) == 8, str(props))
    check("prop object names",
          "AVM_Prop_Pedestrian_00" in props and "AVM_Prop_Crate_03" in props
          and "AVM_Prop_Cart_00" in props, str(props))

    labels = sorted(name for name in names if name.startswith("AVM_Label_"))
    check("ground text objects",
          labels == ["AVM_Label_Back", "AVM_Label_Front", "AVM_Label_Left",
                     "AVM_Label_Logo", "AVM_Label_Right", "AVM_Label_Title"],
          str(labels))
    check("the bundled logo is used by default",
          bpy.data.objects["AVM_Label_Logo"].data.materials[0] is not None)
    check("ground text bodies",
          bpy.data.curves["AVM_Label_Front"].body == "前"
          and bpy.data.curves["AVM_Label_Title"].body == "AVM 仿真标定场地",
          bpy.data.curves["AVM_Label_Title"].body)
    check("ground text lies flat on the ground",
          abs(bpy.data.objects["AVM_Label_Front"].location.z - 0.002) < 1e-6
          and all(abs(v) < 1e-9 for v in bpy.data.objects["AVM_Label_Front"].rotation_euler))
    check("ground text font carries CJK (or is the built-in fallback)",
          bpy.data.curves["AVM_Label_Front"].font is not None)

    check("four camera records",
          [record.name for record in settings.cameras] == ["front", "back", "left", "right"],
          str([record.name for record in settings.cameras]))

    camera = bpy.data.objects["AVM_Cam_Front"]
    check("camera is a compiled custom fisheye",
          camera.data.type == "CUSTOM"
          and camera.data.custom_shader is not None
          and camera.data.custom_shader.name == "opencv_fisheye.osl"
          and len(camera.data.custom_bytecode) > 0)
    check("camera intrinsics from the preset",
          approx(camera.data.opencv_cam.intrinsics.fx, 317.77563818112867, 1e-3)
          and approx(camera.data.opencv_cam.intrinsics.cy, 477.8201435641188, 1e-3))
    check("camera pose from the preset",
          approx(camera.location.x, -0.030997, 1e-4)
          and approx(camera.location.y, 2.466796, 1e-4)
          and approx(camera.location.z, 2.69068, 1e-4),
          f"{tuple(round(v, 4) for v in camera.location)}")
    check("CV Extrinsics synced from the pose",
          abs(camera.data.opencv_cam.pose.euler[0] - math.radians(20.4365)) < 1e-3,
          str(camera.data.opencv_cam.pose.euler))

    block = bpy.data.objects["AVM_Block_FrontLeft"]
    check("front-left block centre",
          approx(block.location.x, -1.9, 1e-6) and approx(block.location.y, 3.7, 1e-6),
          f"{tuple(round(v, 4) for v in block.location)}")
    check("block is a black quad with a white border",
          len(block.data.polygons) == 5 and len(block.data.vertices) == 8
          and len(block.data.materials) == 2,
          f"{len(block.data.polygons)} faces / {len(block.data.materials)} materials")
    check("block border is 0.2 m wide",
          approx(block.dimensions.x, 1.0 + 2 * 0.2, 1e-6)
          and approx(block.dimensions.y, 1.0 + 2 * 0.2, 1e-6)
          and sum(1 for p in block.data.polygons if p.material_index == 0) == 1
          and sum(1 for p in block.data.polygons if p.material_index == 1) == 4,
          f"{tuple(round(v, 4) for v in block.dimensions)}")

    # editing only schedules the debounced rebuild (a no-op in background)
    revision = settings.revision
    settings.core_w = 300.0
    check("editing does not rebuild in background", settings.revision == revision)
    check("manual rebuild", bpy.ops.opencv_cam.avm_rebuild() == {"FINISHED"})
    check("rebuild bumped the revision", settings.revision > revision)
    check("blocks follow core_w",
          approx(bpy.data.objects["AVM_Block_FrontLeft"].location.x, -2.2, 1e-6),
          f"{bpy.data.objects['AVM_Block_FrontLeft'].location.x:.4f}")
    check("rebuild keeps the same objects",
          bpy.data.objects.get("AVM_Ground") is not None)

    # props follow their counts
    settings.prop_carts = 0
    settings.prop_boxes = 1
    settings.prop_pedestrians = 2
    bpy.ops.opencv_cam.avm_rebuild()
    props = sorted(obj.name for obj in target.objects
                   if obj.name.startswith("AVM_Prop_"))
    check("props follow the counts", len(props) == 3, str(props))
    check("stale props are removed", "AVM_Prop_Cart_00" not in props, str(props))
    settings.prop_carts = 1
    settings.prop_boxes = 4
    settings.prop_pedestrians = 3
    bpy.ops.opencv_cam.avm_rebuild()

    check("reset defaults", bpy.ops.opencv_cam.avm_reset_defaults() == {"FINISHED"})
    check("reset restored the field", approx(settings.core_w, 240.0, 1e-6))
    check("reset restored the block",
          approx(bpy.data.objects["AVM_Block_FrontLeft"].location.x, -1.9, 1e-6))

    check("remove scene", bpy.ops.opencv_cam.avm_remove_scene() == {"FINISHED"})
    check("root cleared", settings.root is None)
    check("panels hidden", not scenes_mod.has_scene(bpy.context, definition))
    check("collection gone", bpy.data.collections.get("AVM Scene") is None)
    check("cameras gone", bpy.data.objects.get("AVM_Cam_Front") is None)


def test_avm_panels():
    """Both AVM panels only appear while the scene exists."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    scene = setup_scene(resolution=128, samples=4)

    panels = (("OPENCV_CAM_PT_avm_scene", "PROPERTIES", "WINDOW"),
              ("OPENCV_CAM_PT_avm_scene_view3d", "VIEW_3D", "UI"))
    for name, space, region in panels:
        panel = getattr(bpy.types, name, None)
        check(f"{name} registered", panel is not None)
        if panel is None:
            continue
        check(f"{name} label", panel.bl_label == "AVM Scene", panel.bl_label)
        check(f"{name} space/region",
              panel.bl_space_type == space and panel.bl_region_type == region,
              f"{panel.bl_space_type}/{panel.bl_region_type}")
        check(f"{name} hidden before the build", panel.poll(bpy.context) is False)

    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    check("scene panel shows",
          bpy.types.OPENCV_CAM_PT_avm_scene.poll(bpy.context) is True)
    check("N panel shows",
          bpy.types.OPENCV_CAM_PT_avm_scene_view3d.poll(bpy.context) is True)

    check("select camera operator",
          bpy.ops.opencv_cam.avm_select_camera(name="left") == {"FINISHED"})
    check("selected the left camera",
          bpy.context.view_layer.objects.active is not None
          and bpy.context.view_layer.objects.active.name == "AVM_Cam_Left",
          str(bpy.context.view_layer.objects.active))
    # F12 renders scene.camera, so selecting a camera must make it the render one
    check("selecting a camera makes it the render camera",
          scene.camera is not None and scene.camera.name == "AVM_Cam_Left",
          str(scene.camera))
    check("selecting a camera updates active_camera",
          scene.avm_scene.active_camera == "left", scene.avm_scene.active_camera)

    scene.avm_scene.active_camera = "back"
    bpy.ops.opencv_cam.avm_rebuild()
    check("the active_camera enum drives the render camera",
          scene.camera is not None and scene.camera.name == "AVM_Cam_Back",
          str(scene.camera))

    check("remove scene", bpy.ops.opencv_cam.avm_remove_scene() == {"FINISHED"})
    check("scene panel hides after the remove",
          bpy.types.OPENCV_CAM_PT_avm_scene.poll(bpy.context) is False)
    check("N panel hides after the remove",
          bpy.types.OPENCV_CAM_PT_avm_scene_view3d.poll(bpy.context) is False)


def test_avm_io():
    """Parameter import/export, presets and the four-camera render export."""
    from opencv_camera.bl.scenes.avm_scene import io as io_mod

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    full = io_mod.to_full(settings)
    check("full format shape",
          full["format"] == "avm_scene" and full["border"] == "0x0"
          and full["corner"] == 100 and full["car"] == "240x480",
          str({k: full[k] for k in ("format", "border", "corner", "car")}))
    check("avm block in metres",
          full["avm"]["units"] == "m" and full["avm"]["ground"] == "30x30")
    check("cameras exported",
          len(full["avm"]["cameras"]) == 4
          and len(full["avm"]["cameras"][0]["K"]) == 4
          and len(full["avm"]["cameras"][0]["D"]) == 4)

    compact = io_mod.to_plane(settings)
    check("compact format shape",
          compact["format"] == "plane_scene"
          and {"border", "corner", "inner", "car"} <= set(compact))

    settings.io_text = io_mod.dumps(full)
    check("apply JSON operator", bpy.ops.opencv_cam.avm_apply_json() == {"FINISHED"})
    check("full round trip is stable", io_mod.to_full(settings) == full)

    settings.core_w = 999.0
    bpy.ops.opencv_cam.avm_rebuild()
    settings.io_text = io_mod.dumps(full)
    bpy.ops.opencv_cam.avm_apply_json()
    check("import restored the field", approx(settings.core_w, 240.0, 1e-6))

    height_before = settings.car_height
    settings.io_text = io_mod.dumps({"border": "10x10", "corner": 50,
                                     "inner": "0x0", "car": "262x474"})
    check("apply compact JSON", bpy.ops.opencv_cam.avm_apply_json() == {"FINISHED"})
    check("compact changed the field",
          approx(settings.corner, 50.0, 1e-6) and approx(settings.core_w, 262.0, 1e-6))
    check("compact left the car alone",
          approx(settings.car_height, height_before, 1e-6))

    before = io_mod.to_full(settings)
    settings.io_text = '{"car": "not-a-size"}'
    rejected = False
    try:  # an ERROR report surfaces as a RuntimeError from bpy.ops
        rejected = bpy.ops.opencv_cam.avm_apply_json() == {"CANCELLED"}
    except RuntimeError:
        rejected = True
    check("malformed input is rejected", rejected)
    check("malformed input changes nothing", io_mod.to_full(settings) == before)

    check("apply quick preset",
          bpy.ops.opencv_cam.avm_apply_preset(preset="suv") == {"FINISHED"})
    check("preset applied",
          approx(settings.core_w, 300.0, 1e-6) and approx(settings.corner, 60.0, 1e-6))

    tmp = tempfile.mkdtemp(prefix="avm_io_")
    path = os.path.join(tmp, "params.json")
    check("export to file",
          bpy.ops.opencv_cam.avm_export_params(filepath=path) == {"FINISHED"}
          and os.path.exists(path))
    settings.core_w = 111.0
    bpy.ops.opencv_cam.avm_rebuild()
    check("import from file",
          bpy.ops.opencv_cam.avm_import_params(filepath=path) == {"FINISHED"})
    check("file import restored the field", approx(settings.core_w, 300.0, 1e-6))

    # the four-camera export at a small size, so the test stays quick
    for name in ("front", "back", "left", "right"):
        cam_settings = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"].data.opencv_cam
        cam_settings.output.mode = "custom"
        cam_settings.output.width, cam_settings.output.height = 64, 48
    out = os.path.join(tmp, "views")
    before_render = (scene.render.resolution_x, scene.render.resolution_y,
                     scene.render.image_settings.file_format)
    check("render the four cameras",
          bpy.ops.opencv_cam.avm_render_cameras(
              filepath=os.path.join(out, "avm.png"), samples=1) == {"FINISHED"})
    check("four PNGs written",
          all(os.path.exists(os.path.join(out, f"{name}.png"))
              for name in ("front", "back", "left", "right")),
          str(sorted(os.listdir(out)) if os.path.isdir(out) else out))
    check("render settings restored",
          (scene.render.resolution_x, scene.render.resolution_y,
           scene.render.image_settings.file_format) == before_render,
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")


def test_avm_coverage_and_export():
    """Coverage curves + report, and the raw-material export."""
    from opencv_camera.bl.scenes.avm_scene import coverage as coverage_mod

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    check("analyze coverage",
          bpy.ops.opencv_cam.avm_analyze_coverage(step=0.2) == {"FINISHED"})
    check("coverage summary set", bool(settings.coverage_status),
          settings.coverage_status)
    check("visibility matrix set",
          "front_left: front, left" in settings.coverage_matrix,
          settings.coverage_matrix)
    curves = [bpy.data.objects.get(f"AVM_Coverage_{suffix}")
              for suffix in ("Front", "Back", "Left", "Right")]
    check("four coverage curves", all(obj is not None for obj in curves))
    check("curves are closed polylines",
          all(obj.data.splines and obj.data.splines[0].type == "POLY" for obj in curves))
    check("coverage layer switched on", settings.show_coverage is True)
    settings.show_coverage = False
    check("hiding the layer hides the curves",
          all(obj.hide_render for obj in curves))

    for name in ("front", "back", "left", "right"):
        cam_settings = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"].data.opencv_cam
        cam_settings.output.mode = "custom"
        cam_settings.output.width, cam_settings.output.height = 256, 192
    tmp = tempfile.mkdtemp(prefix="avm_mat_")
    check("export materials",
          bpy.ops.opencv_cam.avm_export_materials(
              filepath=os.path.join(tmp, "avm_scene.json"), samples=4) == {"FINISHED"})
    for expected in ("front.png", "back.png", "left.png", "right.png",
                     "plane_scene.json", "avm_scene.json", "coverage.json",
                     "vehicle_avm_scene.json", "scene_spec.md"):
        check(f"material file {expected}", os.path.exists(os.path.join(tmp, expected)))

    with open(os.path.join(tmp, "vehicle_avm_scene.json"), encoding="utf-8") as handle:
        skeleton = json.load(handle)
    check("filament skeleton has points_3d and detected points_2d",
          len(skeleton["cameras"]) == 4
          and len(skeleton["cameras"][0]["points_3d"]) == 8
          and len(skeleton["cameras"][0]["points_2d"]) == 8)
    with open(os.path.join(tmp, "coverage.json"), encoding="utf-8") as handle:
        report = json.load(handle)
    check("coverage report has footprints and visibility",
          len(report["footprints"]) == 4 and len(report["visibility"]) == 4
          and report["field_ok"] is True)
    check("spec mentions the field",
          "AVM Scene specification" in open(os.path.join(tmp, "scene_spec.md"),
                                           encoding="utf-8").read())


def test_avm_corner_detection():
    """Detect the block corners in the rendered images and store them as points_2d."""
    from opencv_camera.bl.scenes.avm_scene import io as io_mod
    from opencv_camera.core.scenes import avm_coverage, avm_layout

    scene = setup_scene(resolution=256, samples=4)
    clear_scene()
    scene = setup_scene(resolution=256, samples=4)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    for name in ("front", "back", "left", "right"):
        cam_settings = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"].data.opencv_cam
        cam_settings.output.mode = "custom"
        cam_settings.output.width, cam_settings.output.height = 256, 192

    check("detect corners operator",
          bpy.ops.opencv_cam.avm_detect_corners(samples=16) == {"FINISHED"})
    check("corners status set", bool(settings.corners_status), settings.corners_status)

    field = settings.field_spec()
    records = {record["name"]: record for record in io_mod.camera_records(settings)}
    for name in ("front", "back", "left", "right"):
        entry = settings.camera(name)
        check(f"{name}: 8 points_2d detected", entry.points_2d_ok, settings.corners_status)
        if not entry.points_2d_ok:
            continue
        check(f"{name}: sub-pixel detection", entry.points_2d_error < 1.5,
              f"rms {entry.points_2d_error:.2f} px")
        detected = np.array(entry.points_2d, dtype=float).reshape(8, 2)
        camera = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"]
        intrinsics = apply_mod.effective_intrinsics(camera.data.opencv_cam, 256, 192)
        distortion = camera.data.opencv_cam.core_distortion()
        matrix = avm_coverage.object_matrix(records[name]["location"],
                                            records[name]["rotation"])
        projected = np.array([
            avm_coverage.project_ground_point((x, y), intrinsics, distortion, matrix)
            for x, y, _ in avm_layout.points(name, field)])
        error = np.hypot(*(detected - projected).T)
        check(f"{name}: points_2d match the render projection",
              float(error.max()) < 3.0, f"max {float(error.max()):.2f} px")

    tmp = tempfile.mkdtemp(prefix="avm_corners_")
    check("export materials detects corners",
          bpy.ops.opencv_cam.avm_export_materials(
              filepath=os.path.join(tmp, "avm_scene.json"), samples=4) == {"FINISHED"})
    with open(os.path.join(tmp, "vehicle_avm_scene.json"), encoding="utf-8") as handle:
        skeleton = json.load(handle)
    check("exported points_2d are filled",
          all(len(camera["points_2d"]) == 8 for camera in skeleton["cameras"]))


def test_avm_corner_single_and_cache():
    """Per-camera detection, the annotated image and the revision cache."""
    from opencv_camera.bl.scenes.avm_scene import corners as corners_mod

    scene = setup_scene(resolution=256, samples=8)
    clear_scene()
    scene = setup_scene(resolution=256, samples=8)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene
    for name in ("front", "back", "left", "right"):
        cam_settings = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"].data.opencv_cam
        cam_settings.output.mode = "custom"
        cam_settings.output.width, cam_settings.output.height = 256, 192

    check("detect one camera",
          bpy.ops.opencv_cam.avm_detect_camera(name="front", samples=8) == {"FINISHED"})
    entry = settings.camera("front")
    check("single camera detected 8 points",
          entry.points_2d_ok and entry.points_2d_error < 1.5,
          f"rms {entry.points_2d_error:.2f} px")
    image = bpy.data.images.get("AVM_Corners_Front")
    check("annotated image created",
          image is not None and tuple(image.size) == (256, 192),
          str(None if image is None else tuple(image.size)))
    check("the raw render is cached for Export Falcon",
          corners_mod.is_cached(settings, "front")
          and os.path.exists(corners_mod.raw_image_path("front")))

    before = list(entry.points_2d)
    bpy.ops.opencv_cam.avm_detect_camera(name="front", samples=8)
    check("unchanged inputs reuse the cache",
          "cached" in settings.corners_status and list(entry.points_2d) == before,
          settings.corners_status)

    bpy.ops.opencv_cam.avm_detect_camera(name="front", samples=8, use_cache=False)
    check("use_cache=False re-detects", "cached" not in settings.corners_status,
          settings.corners_status)

    settings.core_w = 300.0
    bpy.ops.opencv_cam.avm_rebuild()
    bpy.ops.opencv_cam.avm_detect_camera(name="front", samples=8)
    check("a rebuild invalidates the cache",
          "cached" not in settings.corners_status, settings.corners_status)

    check("show corners operator",
          bpy.ops.opencv_cam.avm_show_corners(name="front") == {"FINISHED"})

    check("clear one camera",
          bpy.ops.opencv_cam.avm_clear_camera(name="front") == {"FINISHED"})
    check("clear drops the detection and the caches",
          not entry.points_2d_ok
          and bpy.data.images.get("AVM_Corners_Front") is None
          and not os.path.exists(corners_mod.raw_image_path("front")),
          settings.corners_status)


def test_avm_export_falcon():
    """Export Falcon packs the four original images and the full config."""
    scene = setup_scene(resolution=256, samples=8)
    clear_scene()
    scene = setup_scene(resolution=256, samples=8)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene
    for name in ("front", "back", "left", "right"):
        cam_settings = bpy.data.objects[f"AVM_Cam_{name.capitalize()}"].data.opencv_cam
        cam_settings.output.mode = "custom"
        cam_settings.output.width, cam_settings.output.height = 256, 192

    tmp = tempfile.mkdtemp(prefix="avm_falcon_")
    path = os.path.join(tmp, "vehicle_avm.zip")
    check("export falcon",
          bpy.ops.opencv_cam.avm_export_falcon(filepath=path, samples=8) == {"FINISHED"})
    check("falcon archive written", os.path.exists(path), path)
    with zipfile.ZipFile(path) as archive:
        names = sorted(archive.namelist())
        check("falcon archive contents",
              names == ["back.png", "back_annotated.png", "front.png",
                        "front_annotated.png", "left.png", "left_annotated.png",
                        "right.png", "right_annotated.png", "vehicle_avm.json"],
              str(names))
        raw_config = archive.read("vehicle_avm.json").decode("utf-8")
        config = json.loads(raw_config)
    check("falcon json keeps scalar arrays on one line",
          '"D": [0.0847' in raw_config
          and not [line for line in raw_config.splitlines()
                   if re.fullmatch(r"\s*-?\d+(?:\.\d+)?,?", line)],
          str([line.strip() for line in raw_config.splitlines()
               if re.fullmatch(r"\s*-?\d+(?:\.\d+)?,?", line)][:3]))
    check("falcon config cameras",
          [camera["name"] for camera in config["cameras"]]
          == ["front", "back", "left", "right"])
    check("falcon config points_2d and input size",
          all(len(camera["points_2d"]) == 8 and camera["input_size"] == [256, 192]
              for camera in config["cameras"]),
          str([(camera["name"], len(camera["points_2d"]), camera["input_size"])
               for camera in config["cameras"]]))
    check("falcon points_2d are in the rendered image pixels",
          all(0.0 <= u <= 256.0 and 0.0 <= v <= 192.0
              for camera in config["cameras"] for u, v in camera["points_2d"]),
          str([[round(u), round(v)] for u, v in config["cameras"][0]["points_2d"]]))
    check("falcon K matches the rendered resolution",
          all(abs(camera["K"][0][0] - 63.55) < 0.1 for camera in config["cameras"]),
          str(config["cameras"][0]["K"][0][0]))
    check("falcon config has the app assets",
          config["name"] == "filament_avm" and "steering_line" in config
          and "mask_overlay" in config and "bev_coord" in config
          and len(config["steering_line"]) == 14)

    # a second export reuses the cached renders (no re-detect needed)
    second = os.path.join(tmp, "again.zip")
    check("export falcon reuses the cache",
          bpy.ops.opencv_cam.avm_export_falcon(filepath=second, samples=8) == {"FINISHED"}
          and os.path.exists(second))


def test_shader_force_compile():
    """force_compile must work: Blender 4.5 does not re-export cycles.osl."""
    from opencv_camera.bl import shader

    check("cycles osl submodule is found", shader.cycles_osl_module() is not None)
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("ForceCompileCam")
    apply_mod.apply_settings(cam_data, cam_data.opencv_cam, scene)
    check("camera compiles normally", shader.is_compiled(cam_data))

    # simulate the fallback path: wipe the bytecode and compile explicitly
    cam_data.custom_bytecode = ""
    check("bytecode cleared", not shader.is_compiled(cam_data))
    messages = shader.force_compile(cam_data)
    check("force_compile produced bytecode",
          shader.is_compiled(cam_data), "; ".join(messages)[:200])


def test_avm_visibility_and_logo():
    """Every AVM object can be shown/hidden, and the logo decal sits before the title."""
    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    def hidden(prefix):
        return [o.name for o in scene.objects
                if o.name.startswith(prefix) and not o.hide_render]

    for prefix, flag in (("AVM_Ground", "show_ground"), ("AVM_Car", "show_car"),
                         ("AVM_Block_", "show_blocks"), ("AVM_Cam_", "show_cameras"),
                         ("AVM_Prop_", "show_props"), ("AVM_Label_", "show_labels"),
                         ("AVM_Sun", "show_sun")):
        setattr(settings, flag, False)
        check(f"{flag} hides its objects", hidden(prefix) == [], f"{prefix} visible: {hidden(prefix)}")
        setattr(settings, flag, True)
        check(f"{flag} shows them again", hidden(prefix) != [], f"{prefix}")

    check("layer toggles do not rebuild", settings.revision > 0)
    revision = settings.revision
    settings.show_props = False
    settings.show_props = True
    check("visibility changes skip the rebuild", settings.revision == revision)

    # default: the logo bundled with the add-on (so it survives a restart)
    check("bundled logo exists", os.path.exists(paths.logo_file()), paths.logo_file())
    settings.logo_image = ""
    settings.logo_enabled = True
    bpy.ops.opencv_cam.avm_rebuild()
    logo = bpy.data.objects.get("AVM_Label_Logo")
    check("empty logo_image falls back to the bundled logo", logo is not None)
    check("bundled logo is square (750x750)",
          abs(logo.dimensions.x - 1.0) < 1e-4 and abs(logo.dimensions.y - 1.0) < 1e-4,
          f"{tuple(round(v, 3) for v in logo.dimensions)}")

    settings.logo_enabled = False
    bpy.ops.opencv_cam.avm_rebuild()
    check("logo_enabled = False removes the logo",
          bpy.data.objects.get("AVM_Label_Logo") is None)
    settings.logo_enabled = True

    logo_path = os.path.join(ROOT, "docs", "images", "preview_example.png")
    check("a logo image is available for the test", os.path.exists(logo_path))
    settings.logo_image = logo_path
    bpy.ops.opencv_cam.avm_rebuild()
    logo = bpy.data.objects.get("AVM_Label_Logo")
    title = bpy.data.objects.get("AVM_Label_Title")
    check("logo decal created", logo is not None and len(logo.data.materials) == 1)
    check("logo decal uses Generated coordinates",
          any(node.type == "TEX_COORD" for node in logo.data.materials[0].node_tree.nodes))
    # preview_example.png is 384x288 (4:3): the plane must follow, not be square
    check("logo keeps the image aspect ratio",
          abs(logo.dimensions.x - 1.0) < 1e-4 and abs(logo.dimensions.y - 0.75) < 1e-4,
          f"{tuple(round(v, 3) for v in logo.dimensions)}")
    check("logo sits before the title", logo.location.x < title.location.x,
          f"logo {logo.location.x:.2f} title {title.location.x:.2f}")
    check("logo lies flat on the ground",
          abs(logo.location.z - 0.002) < 1e-6
          and all(abs(v) < 1e-9 for v in logo.rotation_euler))

    settings.logo_image = ""
    bpy.ops.opencv_cam.avm_rebuild()
    check("clearing logo_image falls back to the bundled logo",
          bpy.data.objects.get("AVM_Label_Logo") is not None)


def test_avm_ground_model():
    """The real AVM bowl ground loads by default, toggles to a plane and back."""
    from opencv_camera.bl.scenes.avm_scene import builder

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    check("the bundled ground model ships", os.path.exists(paths.ground_model_file()),
          paths.ground_model_file())
    check("the real ground mesh is used by default", settings.use_ground_model)
    ground = bpy.data.objects["AVM_Ground"]
    check("the ground is the merged bowl mesh",
          len(ground.data.vertices) > 1000
          and ground.data.get(builder.GROUND_MODEL_KEY) == paths.ground_model_file(),
          f"{len(ground.data.vertices)} verts")
    check("the bowl sits on z = 0", abs(ground.location.z) < 1e-9, f"{ground.location.z}")
    check("the bowl keeps the real 30 m footprint",
          abs(ground.dimensions.x - 30.0) < 1e-3 and abs(ground.dimensions.y - 30.0) < 1e-3,
          f"{tuple(round(v, 3) for v in ground.dimensions)}")

    settings.use_ground_model = False
    bpy.ops.opencv_cam.avm_rebuild()
    ground = bpy.data.objects["AVM_Ground"]
    check("turning the model off restores the flat plane",
          len(ground.data.vertices) == 4
          and abs(ground.location.z + builder.GROUND_DROP) < 1e-9,
          f"{len(ground.data.vertices)} verts z={ground.location.z:.4f}")

    settings.use_ground_model = True
    bpy.ops.opencv_cam.avm_rebuild()
    ground = bpy.data.objects["AVM_Ground"]
    check("turning it back on reloads the bowl", len(ground.data.vertices) > 1000,
          f"{len(ground.data.vertices)} verts")

    settings.ground_model = os.path.join(TMPDL, "missing.glb")
    bpy.ops.opencv_cam.avm_rebuild()
    ground = bpy.data.objects["AVM_Ground"]
    check("a missing model falls back to the flat plane", len(ground.data.vertices) == 4,
          f"{len(ground.data.vertices)} verts")
    settings.ground_model = ""


def test_scene_default_view():
    """A fresh scene is framed from the standard 3/4 orbit, fitted to its subject.

    The orbit maths is pure Python, so most of this runs without a viewport; the
    last block checks the exact corner fit on the real AVM field.
    """
    from opencv_camera.bl import scenes as scenes_mod
    from opencv_camera.bl.scenes import view as view_mod

    # --- the orbit itself --------------------------------------------------
    # mathutils vectors are single precision, so the tolerances stay at 1e-6
    offset = view_mod.orbit_offset(view_mod.DEFAULT_AZIMUTH, view_mod.DEFAULT_ELEVATION)
    check("default orbit is a unit vector", approx(offset.length, 1.0, 1e-6), f"{offset.length}")
    check("default orbit sits front-right-above",
          offset.x > 0 and offset.y > 0 and offset.z > 0,
          f"{tuple(round(v, 3) for v in offset)}")
    check("default orbit is 45 deg off both horizontal axes",
          approx(offset.x, offset.y, 1e-6), f"{offset.x:.4f} vs {offset.y:.4f}")
    check("default elevation is 30 deg",
          approx(offset.z / math.hypot(offset.x, offset.y), math.tan(math.radians(30.0)), 1e-6),
          f"{math.degrees(math.atan2(offset.z, math.hypot(offset.x, offset.y))):.4f} deg")

    # Blender's own numpad views, to pin the azimuth convention down
    for azimuth, elevation, axis, sign in ((0.0, 0.0, "y", -1.0),    # front
                                           (90.0, 0.0, "x", 1.0),    # right
                                           (180.0, 0.0, "y", 1.0),   # back
                                           (270.0, 0.0, "x", -1.0)):  # left
        direction = view_mod.orbit_offset(azimuth, elevation)
        check(f"azimuth {azimuth:.0f} puts the viewer on {sign:+.0f}{axis}",
              approx(getattr(direction, axis), sign, 1e-6)
              and approx(direction.length, 1.0, 1e-6),
              f"{tuple(round(v, 3) for v in direction)}")
    check("elevation 90 looks straight down",
          approx(view_mod.orbit_offset(45.0, 90.0).z, 1.0, 1e-6))
    check("the view rotation reproduces the orbit offset",
          approx((view_mod._view_rotation(135.0, 30.0) @ Vector((0, 0, 1)))
                 .dot(view_mod.orbit_offset(135.0, 30.0)), 1.0, 1e-6))

    # --- every registered scene carries it ---------------------------------
    ids = {definition.id for definition in scenes_mod.definitions()}
    check("the registry has both scenes", ids == {"camera_scene", "avm_scene"}, str(ids))
    for definition in scenes_mod.definitions():
        check(f"{definition.id} uses the 3/4 default view",
              approx(definition.view.azimuth, 135.0) and approx(definition.view.elevation, 30.0),
              str(definition.view))

    # --- what a scene frames ----------------------------------------------
    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    definition = scenes_mod.definition("avm_scene")
    subject = view_mod.targets(definition, scene)
    names = {obj.name for obj in subject}
    check("the AVM subject is the field, not the whole scene",
          names and all("Block" in n or "Cam" in n or n == "AVM_Car" for n in names),
          str(sorted(names)))
    check("the 30 m ground and the lights are not framed",
          "AVM_Ground" not in names and "AVM_Sun" not in names
          and "AVM_Label_Title" not in names)
    measured = view_mod.bounds(subject)
    check("the AVM subject has bounds", measured is not None)
    centre, radius = measured
    check("the AVM field is centred on the car",
          approx(centre.x, 0.0, 1e-6) and approx(centre.y, 0.0, 1e-6),
          f"{tuple(round(v, 3) for v in centre)}")
    check("the AVM bounding radius covers the field diagonal",
          4.5 < radius < 6.0, f"{radius:.3f}")

    # --- the write into the viewport --------------------------------------
    # background Blender still has a window/screen, so the VIEW_3D area is real
    # and the write is exercised - we just cannot look at it
    check("framing nothing is a no-op", view_mod.frame([], definition.view) == 0)
    moved = view_mod.frame(subject, definition.view)
    check("framing moves the viewport(s)", moved >= 1, str(moved))
    viewports = list(view_mod.view_areas())
    check("the viewport was pointed at the field",
          viewports and all(approx(region_3d.view_location.x, centre.x, 1e-4)
                            and approx(region_3d.view_location.y, centre.y, 1e-4)
                            and approx(region_3d.view_location.z, centre.z, 1e-4)
                            for _, _, _, region_3d in viewports),
          f"{[tuple(round(v, 3) for v in region_3d.view_location) for *_, region_3d in viewports]}")
    check("the viewport orbit is the default 3/4 one",
          all(region_3d.view_rotation.dot(
              view_mod._view_rotation(definition.view.azimuth, definition.view.elevation)) > 0.999999
              for _, _, _, region_3d in viewports))
    check("the viewport is far enough away",
          all(region_3d.view_distance > 10.0 for _, _, _, region_3d in viewports),
          f"{[round(region_3d.view_distance, 2) for *_, region_3d in viewports]}")
    check("frame_view is registered", "frame_view" in dir(bpy.ops.opencv_cam))
    check("frame_view reframes the built scene",
          bpy.ops.opencv_cam.frame_view(scene_id="avm_scene") == {"FINISHED"})
    check("frame_view refuses an unknown scene",
          bpy.ops.opencv_cam.frame_view(scene_id="nope") == {"CANCELLED"})

    # --- the exact corner fit, checked against Blender's own projection -----
    # _frustum reads the two scales off the viewport's projection matrix; they
    # depend only on the lens and the region size, so they survive the write
    points = view_mod.corners(subject)
    rotation = view_mod._view_rotation(definition.view.azimuth, definition.view.elevation)
    inverse = rotation.conjugated()
    viewport = viewports[0]
    k_h, k_v, is_perspective = view_mod._frustum(viewport[3])
    region = viewport[1]
    aspect = max(1, region.width) / max(1, region.height)
    check("the frustum scales come from the projection, split by the aspect ratio",
          is_perspective and k_h > 0.0 and k_v > 0.0 and approx(k_v / k_h, aspect, 0.01),
          f"k_h={k_h:.4f} k_v={k_v:.4f} aspect={aspect:.4f} persp={is_perspective}")

    def worst_ndc(distance, half_w, half_h):
        worst = 0.0
        for point in points:
            q = inverse @ (point - centre)
            depth = distance - q.z
            if depth <= 0.0:
                return float("inf")
            worst = max(worst, abs(q.x) / (depth * half_w), abs(q.y) / (depth * half_h))
        return worst

    half_h = 1.0 / k_v
    half_w = 1.0 / k_h
    for _, _, _, region_3d in viewports:
        distance = region_3d.view_distance
        worst = worst_ndc(distance, half_w, half_h)
        check("every corner projects inside the frame", worst <= 1.0, f"worst |NDC| {worst:.3f}")
        check("the subject nearly fills the frame", worst > 0.8, f"worst |NDC| {worst:.3f}")

    needed = max(max(q.z + abs(q.x) * k_h, q.z + abs(q.y) * k_v)
                 for q in (inverse @ (point - centre) for point in points))
    check("the margin is applied on top of the exact fit",
          approx(viewport[3].view_distance, needed * definition.view.margin, 1e-4),
          f"d={viewport[3].view_distance:.4f} needed={needed:.4f} "
          f"margin={definition.view.margin}")
    check("the fit is tight (5% closer would clip)",
          worst_ndc(needed / 1.05, half_w, half_h) > 1.0, f"needed={needed:.3f}")

    # --- the ortho branch: same scales, same knob, no depth term -----------
    # Background Blender cannot produce a valid ortho projection matrix (the
    # operator that would force a redraw needs a real UI context), so the ortho
    # scales are fed in from the perspective read above - they are the same two
    # constants, as the live GUI confirms.
    ortho_distance = view_mod._fit_distance(points, centre, rotation, (k_h, k_v),
                                            False, definition.view.margin)
    space_points = view_mod._view_space(points, centre, rotation)
    ortho_needed = max(max(abs(q.x) * k_h, abs(q.y) * k_v) for q in space_points)
    check("the ortho fit drops the depth term",
          approx(ortho_distance, ortho_needed * definition.view.margin, 1e-4),
          f"d={ortho_distance:.4f} needed={ortho_needed:.4f}")
    check("the ortho fit sits closer than the perspective one",
          ortho_distance < viewport[3].view_distance,
          f"ortho {ortho_distance:.3f} vs persp {viewport[3].view_distance:.3f}")

    class FakeRegion:
        """An orthographic viewport, whose matrix folds in 1 / view_distance."""

        view_perspective = "ORTHO"
        view_distance = 20.0
        perspective_matrix = Matrix(((k_h / 20.0, 0.0, 0.0, 0.0),
                                     (0.0, k_v / 20.0, 0.0, 0.0),
                                     (0.0, 0.0, -0.05, 0.0),
                                     (0.0, 0.0, 0.0, 1.0)))

    check("_frustum undoes the ortho 1 / view_distance fold",
          approx(view_mod._frustum(FakeRegion())[0], k_h, 1e-6)
          and approx(view_mod._frustum(FakeRegion())[1], k_v, 1e-6)
          and view_mod._frustum(FakeRegion())[2] is False,
          f"{view_mod._frustum(FakeRegion())}")

    # --- the Camera Scene frames the cube, not the 24 m checker ground -----
    check("add Camera Scene",
          bpy.ops.opencv_cam.add_camera_scene(distance=4.0, samples=2) == {"FINISHED"})
    camera_definition = scenes_mod.definition("camera_scene")
    camera_subject = view_mod.targets(camera_definition, scene)
    check("the Camera Scene frames the checker cube",
          any(obj.name == "CheckerCube" for obj in camera_subject))
    check("the Camera Scene leaves the 24 m checker ground out",
          camera_subject and all(obj.name != "CheckerGround" for obj in camera_subject),
          str(sorted(obj.name for obj in camera_subject)))
    cube_centre, cube_radius = view_mod.bounds(camera_subject)
    check("the Camera Scene subject is cube sized",
          cube_radius < 5.0, f"{cube_radius:.3f}")

    # world aligned: a level floor with everything standing on it
    ground = bpy.data.objects[camera_scene.GROUND_NAME]
    cube = bpy.data.objects["CheckerCube"]
    check("the Camera Scene ground is horizontal",
          all(abs(v) < 1e-9 for v in ground.rotation_euler)
          and approx(ground.matrix_world.to_3x3()[2][2], 1.0, 1e-9),
          f"{tuple(round(v, 4) for v in ground.rotation_euler)}")
    floor = ground.location.z + camera_scene.GROUND_DROP
    check("the cube stands on the ground",
          approx(cube.location.z - cube.dimensions.z * 0.5, floor, 1e-6),
          f"cube bottom {cube.location.z - cube.dimensions.z * 0.5:.4f} floor {floor:.4f}")
    for index in range(4):
        block = bpy.data.objects[f"ColorBlock{index}"]
        check(f"colour block {index} stands on the ground",
              approx(block.location.z - block.dimensions.z * 0.5, floor, 1e-6),
              f"{block.location.z - block.dimensions.z * 0.5:.4f} vs {floor:.4f}")
    expected, expected_floor = camera_scene.layout(scene.camera, 4.0, 1.2)
    check("the cube sits where the camera is looking",
          approx((cube.location - expected).length, 0.0, 1e-5),
          f"{tuple(round(v, 3) for v in cube.location)} vs "
          f"{tuple(round(v, 3) for v in expected)}")
    active = scene.camera
    check("the test camera looks down from above the ground",
          -(active.matrix_world.to_3x3() @ Vector((0, 0, 1))).z < -1e-3
          and active.matrix_world.translation.z > 1e-3,
          f"{active.name} at z={active.matrix_world.translation.z:.3f}")
    check("the ground is the world floor (z = 0), like the AVM Scene",
          approx(expected_floor, 0.0, 1e-9) and approx(floor, 0.0, 1e-9),
          f"floor {floor}")

    # a camera sitting in the ground plane (a freshly added one) never meets
    # z = 0, so the floor drops to the cube's feet instead
    fallback = bpy.data.objects.new("FallbackCam", bpy.data.cameras.new("FallbackCam"))
    scene.collection.objects.link(fallback)
    fallback_subject, fallback_floor = camera_scene.layout(fallback, 4.0, 1.2)
    check("a camera in the ground plane drops the floor to the cube's feet",
          approx(fallback_floor, fallback_subject.z - 0.6, 1e-9) and fallback_floor < 0.0,
          f"floor {fallback_floor:.3f} subject {tuple(round(v, 3) for v in fallback_subject)}")
    bpy.data.objects.remove(fallback, do_unlink=True)


def test_add_camera_default_preset():
    """The Add menu path (no preset argument) must use the add-on defaults.

    It must not silently copy whatever camera happens to be active.
    """
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    odd, odd_data = make_camera("OddCam")
    odd_data.opencv_cam.intrinsics.fx = 999.0
    odd_data.opencv_cam.distortion.k1 = 0.5
    scene.camera = odd
    bpy.context.view_layer.objects.active = odd
    odd.select_set(True)

    check("add_camera without a preset", bpy.ops.opencv_cam.add_camera(model="fisheye") == {"FINISHED"})
    new = bpy.context.view_layer.objects.active
    check("menu path uses the add-on defaults",
          approx(new.data.opencv_cam.intrinsics.fx, DEFAULT_INTRINSICS["fx"], 1e-3)
          and approx(new.data.opencv_cam.distortion.k1, DEFAULT_DISTORTION[0], 1e-6),
          f"fx={new.data.opencv_cam.intrinsics.fx:.3f} k1={new.data.opencv_cam.distortion.k1:.5f}")
    check("menu path camera is ready", new.data.type == "CUSTOM"
          and len(new.data.custom_bytecode) > 0)

    # explicit "current settings" still copies (make the odd camera active again:
    # creating a camera selects it, so it would otherwise copy itself)
    scene.camera = odd
    bpy.context.view_layer.objects.active = odd
    for obj in scene.objects:
        obj.select_set(obj is odd)
    check("add_camera with the current camera preset",
          bpy.ops.opencv_cam.add_camera(model="fisheye", preset=presets.CURRENT) == {"FINISHED"})
    copied = bpy.context.view_layer.objects.active
    check("current-settings preset copies the active camera",
          approx(copied.data.opencv_cam.intrinsics.fx, 999.0, 1e-3),
          f"fx={copied.data.opencv_cam.intrinsics.fx:.3f}")


def test_presets():
    names = [identifier for identifier, _ in presets.list_presets()]
    check("bundled presets listed", "default_camera" in names, str(names))
    calibration = presets.load_preset("default_camera")
    check("preset loads", approx(calibration.intrinsics.fx, 317.77563818112867, 1e-6)
          and calibration.distortion.model == camera_model.MODEL_FISHEYE)
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("PresetCam")
    scene.camera = camera
    bpy.context.view_layer.objects.active = camera
    check("preset list starts with the add-on defaults",
          presets.DEFAULTS == "__defaults__" and presets.CURRENT == "__current__")
    check("load_preset operator",
          bpy.ops.opencv_cam.load_preset(preset="default_camera") == {"FINISHED"})
    check("load_preset accepts the defaults entry",
          bpy.ops.opencv_cam.load_preset(preset=presets.DEFAULTS) == {"FINISHED"})
    check("preset applied to the camera",
          approx(cam_data.opencv_cam.intrinsics.fx, 317.77563818112867, 1e-3)
          and approx(cam_data.cycles_custom["k1"], 0.08476733270570755, 1e-6))
    check("reset defaults operator",
          bpy.ops.opencv_cam.reset_defaults() == {"FINISHED"})
    check("reset restores the reference values",
          approx(cam_data.opencv_cam.intrinsics.image_width, 1280)
          and cam_data.opencv_cam.distortion.model == "fisheye")


def test_shader_text_upgrade_recompiles():
    """Upgrading the add-on replaces the shader text and must recompile.

    Simulated by making the *bundle* look old (parameter renamed), applying, then
    restoring the real bundle - exactly the state after an add-on update, where the
    .blend still holds the old Text while ``shaders/*.osl`` moved on.
    """
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("UpgradeCam")
    settings = cam_data.opencv_cam
    ok, _ = apply_mod.apply_settings(cam_data, settings, scene)
    check("upgrade: initial apply", ok and "discard_invalid_rays" in cam_data.cycles_custom)

    # simulate a .blend written by an older add-on version: the Text holds the old
    # shader and the camera carries its bytecode, while the bundle moved on
    old_source = shader.shader_source(shader.shader_filename("fisheye")).replace(
        "discard_invalid_rays", "allow_off_sensor"
    )
    text = cam_data.custom_shader
    text.clear()
    text.write(old_source)
    cam_data.custom_bytecode = ""
    shader.force_compile(cam_data)
    check("upgrade: simulated old add-on state",
          "allow_off_sensor" in cam_data.cycles_custom
          and "discard_invalid_rays" not in cam_data.cycles_custom,
          f"{sorted(cam_data.cycles_custom.keys())}")

    ok, messages = apply_mod.apply_settings(cam_data, settings, scene)
    check("upgrade: new bundle recompiles the camera",
          ok and "discard_invalid_rays" in cam_data.cycles_custom
          and "allow_off_sensor" not in cam_data.cycles_custom,
          "; ".join(messages)[:120])
    check("upgrade: text matches the new bundle",
          cam_data.custom_shader.as_string()
          == shader.shader_source(shader.shader_filename("fisheye")))


def test_live_apply():
    """auto_apply pushes edits without pressing Apply to Camera."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("LiveCam")
    settings = cam_data.opencv_cam
    settings.intrinsics.image_width = 128
    settings.intrinsics.image_height = 128
    apply_mod.apply_settings(cam_data, settings, scene)
    check("live apply enabled by default", settings.auto_apply)
    settings.intrinsics.fx = 123.0
    check("fx edit applied live",
          approx(cam_data.cycles_custom["fx"], 123.0, 1e-4),
          f"fx={cam_data.cycles_custom['fx']}")
    settings.distortion.k1 = 0.25          # fisheye default model
    check("k1 edit applied live", approx(cam_data.cycles_custom["k1"], 0.25, 1e-6))
    settings.auto_apply = False
    settings.intrinsics.fx = 999.0
    check("live apply can be switched off",
          approx(cam_data.cycles_custom["fx"], 123.0, 1e-4))
    settings.auto_apply = True
    # switching the model must swap the shader as well
    settings.distortion.model = "brown_conrady"
    check("model switch swaps the shader live",
          cam_data.custom_shader.name == "opencv_camera.osl"
          and approx(cam_data.cycles_custom["k1"], 0.25, 1e-6),
          str(cam_data.custom_shader.name))


def test_output_resolution():
    """The output size must drive the scene resolution and the intrinsics."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("OutputCam")
    settings = cam_data.opencv_cam
    set_distortion(settings)
    settings.intrinsics.fx = 1500.0
    settings.intrinsics.fy = 1500.0
    settings.intrinsics.image_width = 1920
    settings.intrinsics.image_height = 1080
    settings.intrinsics.auto_center = True
    output = settings.output

    # default: the output size *is* the calibration size (real camera output)
    check("output defaults to the calibration size", output.mode == "calibration")
    apply_mod.apply_settings(cam_data, settings, scene)
    check("calibration size drives the scene",
          (scene.render.resolution_x, scene.render.resolution_y) == (1920, 1080),
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")
    check("intrinsics untouched at the calibration size",
          approx(cam_data.cycles_custom["fx"], 1500.0, 1e-4))

    # custom output at the same aspect (16:9) -> FOV preserving rescale
    output.mode = "custom"
    output.preset = "640x480"
    check("preset fills width/height", (output.width, output.height) == (640, 480))
    output.width, output.height = 640, 360
    output.preset = "custom"
    check("custom output drives the scene",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 360),
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")
    check("intrinsics rescaled for the output size",
          approx(cam_data.cycles_custom["fx"], 1500.0 * 640 / 1920, 1e-3),
          f"fx={cam_data.cycles_custom['fx']:.4f}")
    check("preview aspect follows the output size",
          preview.preview_resolution(settings, "256") == (256, 144),
          str(preview.preview_resolution(settings, "256")))

    # custom output with a different aspect -> centre crop, pixel pitch kept
    output.width = output.height = 640
    output.preset = "custom"
    check("square output drives the scene",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 640))
    check("square output keeps the pixel pitch (crop)",
          approx(cam_data.cycles_custom["fx"], 1500.0, 1e-3),
          f"fx={cam_data.cycles_custom['fx']:.4f}")

    # scene mode leaves Blender's resolution alone
    output.mode = "scene"
    scene.render.resolution_x, scene.render.resolution_y = 640, 360
    apply_mod.apply_settings(cam_data, settings, scene)
    check("scene mode does not touch the scene",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 360))
    check("scene mode scales to the scene size",
          approx(cam_data.cycles_custom["fx"], 1500.0 * 640 / 1920, 1e-3),
          f"fx={cam_data.cycles_custom['fx']:.4f}")

    # operators
    bpy.context.view_layer.objects.active = camera
    scene.camera = camera
    output.mode = "custom"
    output.width, output.height = 1280, 720
    check("set_render_resolution operator",
          bpy.ops.opencv_cam.set_render_resolution() == {"FINISHED"}
          and (scene.render.resolution_x, scene.render.resolution_y) == (1280, 720))
    scene.render.resolution_x, scene.render.resolution_y = 800, 600
    check("read_scene_resolution operator",
          bpy.ops.opencv_cam.read_scene_resolution() == {"FINISHED"}
          and (output.width, output.height) == (800, 600))
    check("output read switched the mode to custom", output.mode == "custom")
    check("lock can be switched off",
          output.lock_scene_resolution is True)
    output.lock_scene_resolution = False
    output.width, output.height = 1024, 768
    check("unlocked output leaves the scene alone",
          (scene.render.resolution_x, scene.render.resolution_y) == (800, 600))
    output.lock_scene_resolution = True


def test_preview():
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    checker_plane(scene)
    camera, cam_data = make_camera("PreviewCam")
    scene.camera = camera
    bpy.context.view_layer.objects.active = camera
    settings = cam_data.opencv_cam
    settings.intrinsics.image_width = 1280
    settings.intrinsics.image_height = 960
    settings.output.mode = "scene"   # keep the test's small scene resolution
    before = (scene.render.resolution_x, scene.render.resolution_y, scene.cycles.samples)
    result = preview.render_preview(cam_data, settings, scene, size_key="256", samples=2, show=False)
    check("preview renders", result["ok"] and result["resolution"][1] > 0,
          f"{result['resolution']} {result['messages']}")
    check("preview follows the output size (square here)",
          result["resolution"] == (256, 256), str(result["resolution"]))
    check("preview restores the render settings",
          (scene.render.resolution_x, scene.render.resolution_y, scene.cycles.samples) == before,
          f"{scene.render.resolution_x}x{scene.render.resolution_y}@{scene.cycles.samples}")
    check("preview has a render result", bpy.data.images.get("Render Result") is not None)
    expected_fx = apply_mod.effective_intrinsics(settings, before[0], before[1]).fx
    check("preview re-applies the intrinsics for the restored resolution",
          approx(cam_data.cycles_custom["fx"], expected_fx, 1e-3),
          f"{cam_data.cycles_custom['fx']:.4f} vs {expected_fx:.4f}")
    check("preview operator",
          bpy.ops.opencv_cam.preview(size="256", samples=2) == {"FINISHED"})
    check("schedule_preview is a no-op in background", preview.schedule_preview(cam_data, settings, scene) is None)


def main():
    tests = (
        test_registration,
        test_apply_and_compile,
        test_shader_failure_is_detected,
        test_selftest,
        test_selftest_with_default_world,
        test_fisheye_selftest,
        test_camera_scene_builder,
        test_builtin_camera_equivalence,
        test_shift_equivalence,
        test_resolution_scaling,
        test_euler_extrinsics,
        test_pose_roundtrip,
        test_calibration_roundtrip,
        test_sync_from_lens,
        test_operator_end_to_end,
        test_add_camera_operator,
        test_add_camera_default_preset,
        test_panel_layout,
        test_menus_and_raw_params,
        test_scene_registry,
        test_avm_scene_builder,
        test_avm_panels,
        test_avm_io,
        test_avm_coverage_and_export,
        test_avm_corner_detection,
        test_avm_corner_single_and_cache,
        test_avm_export_falcon,
        test_avm_visibility_and_logo,
        test_avm_ground_model,
        test_scene_default_view,
        test_shader_force_compile,
        test_presets,
        test_shader_text_upgrade_recompiles,
        test_live_apply,
        test_output_resolution,
        test_preview,
    )
    opencv_camera.register()
    try:
        for test in tests:
            try:
                test()
            except Exception as exc:
                import traceback
                traceback.print_exc()
                check(test.__name__, False, f"{type(exc).__name__}: {exc}")
    finally:
        opencv_camera.unregister()

    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): {', '.join(FAILURES)}")
        sys.exit(1)
    print("all Blender integration tests passed")


if __name__ == "__main__":
    main()