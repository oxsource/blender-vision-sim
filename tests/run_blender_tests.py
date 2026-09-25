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
from opencv_camera.bl import camera_factory, compat, preview
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
    camera_scene.prepare_render(scene, settings, samples=8)
    check("prepare_render leaves the scene resolution alone",
          (scene.render.resolution_x, scene.render.resolution_y) == (128, 128),
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

    # same aspect ratio (960x540 is 16:9) -> rescale, FOV preserved
    scene.render.resolution_x, scene.render.resolution_y = 960, 540
    apply_mod.apply_settings(cam_data, settings, scene)
    check("same aspect: intrinsics rescaled",
          approx(cam_data.cycles_custom["fx"], 750.0, 1e-4)
          and approx(cam_data.cycles_custom["fy"], 750.0, 1e-4),
          f"fx={cam_data.cycles_custom['fx']:.4f}")
    check("at the calibration resolution the values are unchanged",
          approx(apply_mod.effective_intrinsics(settings, 1920, 1080).fx, 1500.0, 1e-4))

    # different aspect ratio -> centre crop at the original pixel pitch
    scene.render.resolution_x = scene.render.resolution_y = 1280
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


def test_extrinsics_is_object_transform():
    """The camera object transform IS the pose - CV Extrinsics has no copy."""
    import math
    from opencv_camera.bl.scenes.avm_scene import builder

    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    camera, cam_data = make_camera("ExtrCam")
    scene.camera = camera
    bpy.context.view_layer.objects.active = camera
    camera.select_set(True)
    settings = cam_data.opencv_cam

    check("opencv_cam has no separate pose group", not hasattr(settings, "pose"))

    # the AVM builder writes the object transform, so the object Location/Rotation
    # (what CV Extrinsics edits) matches the record exactly
    location = [-0.031, 2.4668, 2.6907]
    rotation_deg = (20.4365, -1.1319, 1.7446)
    builder.apply_camera_pose(camera, location, [math.radians(v) for v in rotation_deg])
    check("pose lands on the object location",
          all(abs(a - b) < 1e-4 for a, b in zip(camera.location, location)),
          str(tuple(round(v, 4) for v in camera.location)))
    check("pose lands on the object rotation_euler",
          all(abs(math.degrees(a) - b) < 1e-3
              for a, b in zip(camera.rotation_euler, rotation_deg)),
          str(tuple(round(math.degrees(v), 3) for v in camera.rotation_euler)))

    # editing the object directly (Item tab / gizmo) is the same pose
    camera.rotation_euler = (math.radians(10.0), math.radians(20.0), math.radians(30.0))
    check("the object rotation is the pose",
          tuple(round(math.degrees(v), 3) for v in camera.rotation_euler)
          == (10.0, 20.0, 30.0),
          str(tuple(round(math.degrees(v), 3) for v in camera.rotation_euler)))


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


def test_panel_layout():
    """Four top level CV panels, sorted before Blender's own, none under Lens."""
    expected = {
        "OPENCV_CAM_PT_main": "CV Intrinsics",
        "OPENCV_CAM_PT_extrinsics": "CV Extrinsics",
        "OPENCV_CAM_PT_io": "CV Presets",
        "OPENCV_CAM_PT_preview": "CV Preview",
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
    check("no CV Output panel any more", not hasattr(bpy.types, "OPENCV_CAM_PT_output"))
    order = {name: getattr(bpy.types, name).bl_order for name in expected}
    check("panel order: Intrinsics, Extrinsics, Presets, Preview",
          order["OPENCV_CAM_PT_main"] < order["OPENCV_CAM_PT_extrinsics"]
          < order["OPENCV_CAM_PT_io"] < order["OPENCV_CAM_PT_preview"], str(order))
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
    check("extrinsics live on the object (no pose group on opencv_cam)",
          not hasattr(settings, "pose"))
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
               "pinhole", "camera_scene", "avm_scene", "drive_scene", "road_scene")),
          str(icons_mod.available()))
    check("icon files ship with the add-on", len(icons_mod.available()) == 10,
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
    scene.render.engine = compat.eevee_engine()
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
    check("the AVM rotation lands on the object (CV Extrinsics)",
          abs(math.degrees(camera.rotation_euler[0]) - 20.4365) < 1e-2,
          str(tuple(round(math.degrees(v), 3) for v in camera.rotation_euler)))

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
    """Parameter import/export and the four-camera render helper."""
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

    tmp = tempfile.mkdtemp(prefix="avm_io_")
    path = os.path.join(tmp, "params.json")
    check("export full parameters to file",
          bpy.ops.opencv_cam.avm_export_params(filepath=path) == {"FINISHED"}
          and os.path.exists(path))
    check("full round trip is stable", io_mod.read(path) == full)

    settings.core_w = 999.0
    bpy.ops.opencv_cam.avm_rebuild()
    check("import full parameters from file",
          bpy.ops.opencv_cam.avm_import_params(filepath=path) == {"FINISHED"})
    check("file import restored the field", approx(settings.core_w, 240.0, 1e-6))

    # the compact (plane_scene) format keeps the car alone
    compact_path = os.path.join(tmp, "plane.json")
    check("export compact parameters",
          bpy.ops.opencv_cam.avm_export_params(
              filepath=compact_path, format="plane") == {"FINISHED"})
    height_before = settings.car_height
    settings.corner = 123.0
    bpy.ops.opencv_cam.avm_rebuild()
    check("import compact parameters",
          bpy.ops.opencv_cam.avm_import_params(filepath=compact_path) == {"FINISHED"})
    check("compact changed the field",
          approx(settings.corner, 100.0, 1e-6) and approx(settings.core_w, 240.0, 1e-6))
    check("compact left the car alone", approx(settings.car_height, height_before, 1e-6))

    before = io_mod.to_full(settings)
    bad_path = os.path.join(tmp, "bad.json")
    with open(bad_path, "w", encoding="utf-8") as handle:
        handle.write('{"car": "not-a-size"}')
    rejected = False
    try:  # an ERROR report surfaces as a RuntimeError from bpy.ops
        rejected = bpy.ops.opencv_cam.avm_import_params(filepath=bad_path) == {"CANCELLED"}
    except RuntimeError:
        rejected = True
    check("malformed input is rejected", rejected)
    check("malformed input changes nothing", io_mod.to_full(settings) == before)

    # the four-camera render helper (used by Export Falcon) at a small size
    scene.render.resolution_x, scene.render.resolution_y = 64, 48
    out = os.path.join(tmp, "views")
    before_render = (scene.render.resolution_x, scene.render.resolution_y,
                     scene.render.image_settings.file_format)
    written = io_mod.render_cameras(bpy.context, settings, out, samples=1)
    check("render the four cameras", len(written) == 4, str(written))
    check("four PNGs written",
          all(os.path.exists(os.path.join(out, f"{name}.png"))
              for name in ("front", "back", "left", "right")),
          str(sorted(os.listdir(out)) if os.path.isdir(out) else out))
    check("render settings restored",
          (scene.render.resolution_x, scene.render.resolution_y,
           scene.render.image_settings.file_format) == before_render,
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")


def test_avm_io_covers_every_setting():
    """The full JSON round-trips every editable AVM Scene setting (no silent drops)."""
    from opencv_camera.bl.scenes.avm_scene import io as io_mod

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    # PropertyGroup's own `name` plus the state / UI / detection fields are not
    # parameters, so they are expected to stay out of the JSON
    skip = {"name", "root", "revision", "io_status",
            "coverage_status", "coverage_matrix", "corners_status",
            "bowl_include_vehicle",
            "points_2d", "points_2d_ok", "points_2d_error",
            "points_2d_revision", "points_2d_signature"}
    props = [prop for prop in settings.bl_rna.properties
             if prop.identifier not in skip and not prop.is_readonly]

    def is_array(prop):
        return bool(getattr(prop, "is_array", False))

    def target(prop, current):
        if prop.identifier == "corner":  # the HTML key is an integer (cm)
            return float(int(round(current)) + 1)
        if prop.type == "BOOLEAN":
            return not bool(current)
        if prop.type == "INT":
            value = int(current) + 3
            if prop.hard_max is not None:
                value = min(value, int(prop.hard_max))
            if prop.hard_min is not None:
                value = max(value, int(prop.hard_min))
            return value
        if prop.type == "FLOAT":
            if is_array(prop):
                return [float(value) + 0.5 for value in current]
            value = float(current) + 0.5
            if prop.hard_max is not None:
                value = min(value, prop.hard_max)
            if prop.hard_min is not None:
                value = max(value, prop.hard_min)
            return value
        if prop.type == "STRING":
            return f"probe-{prop.identifier}"
        if prop.type == "ENUM":
            ids = [item.identifier for item in prop.enum_items]
            return next((item for item in ids if item != current), current)
        return current

    # the scene settings live on the Scene and survive clear_scene(), so capture
    # the pre-test values and restore them afterwards, or every later test
    # inherits the probe values
    original = {prop.identifier: (list(getattr(settings, prop.identifier))
                                 if is_array(prop) else getattr(settings, prop.identifier),
                                 is_array(prop))
                for prop in props}

    wanted = {}
    for prop in props:
        current = getattr(settings, prop.identifier)
        if prop.type in ("FLOAT", "INT") and is_array(prop):
            current = list(current)
        value = target(prop, current)
        wanted[prop.identifier] = (value, prop.type, is_array(prop))
        setattr(settings, prop.identifier, value)

    try:
        full = io_mod.to_full(settings)

        # disturb every setting, then apply the JSON and compare
        for prop in props:
            if prop.type == "BOOLEAN":
                setattr(settings, prop.identifier, False)
            elif prop.type == "INT":
                setattr(settings, prop.identifier, int(prop.hard_min or 0))
            elif prop.type == "FLOAT":
                if is_array(prop):
                    setattr(settings, prop.identifier,
                            [0.0] * len(wanted[prop.identifier][0]))
                else:
                    setattr(settings, prop.identifier, float(prop.hard_min or 0.0))
            elif prop.type == "STRING":
                setattr(settings, prop.identifier, "")
            elif prop.type == "ENUM":
                ids = [item.identifier for item in prop.enum_items]
                setattr(settings, prop.identifier, ids[-1])
        io_mod.apply(settings, full)

        dropped = []
        for name, (expected, prop_type, array) in wanted.items():
            got = getattr(settings, name)
            if prop_type in ("FLOAT", "INT") and array:
                ok = all(abs(float(a) - float(b)) < 1e-6 for a, b in zip(got, expected))
            elif prop_type == "FLOAT":
                ok = abs(float(got) - float(expected)) < 1e-6
            else:
                ok = got == expected
            if not ok:
                dropped.append(f"{name}={got!r} (want {expected!r})")
        check(f"all {len(wanted)} editable settings round-trip through the full JSON",
              not dropped, "; ".join(dropped))

        # the camera records carry the mount pose, K, D and the OpenCV output
        # size; the pose is read from / written to the camera object itself (the
        # single source), so the round trip goes through the object transform
        record = full["avm"]["cameras"][0]
        check("camera records carry K/D/output", {"K", "D", "output"} <= set(record),
              str(sorted(record)))
        camera_object = bpy.data.objects["AVM_Cam_Front"]
        camera_object.location = [1.0, 2.0, 3.0]
        camera_object.rotation_euler = [math.radians(4.0), math.radians(5.0), math.radians(6.0)]
        cam_data = camera_object.data.opencv_cam
        cam_data.intrinsics.fx = 111.0
        cam_data.intrinsics.image_width, cam_data.intrinsics.image_height = 111, 222
        full = io_mod.to_full(settings)
        camera_object.location = [0.0, 0.0, 0.0]
        camera_object.rotation_euler = [0.0, 0.0, 0.0]
        cam_data.intrinsics.fx = 0.0
        cam_data.intrinsics.image_width, cam_data.intrinsics.image_height = 1, 1
        io_mod.apply(settings, full)
        camera_object = bpy.data.objects["AVM_Cam_Front"]
        cam_data = camera_object.data.opencv_cam
        check("camera pose / K / output round-trip",
              all(abs(a - b) < 1e-4 for a, b in zip(camera_object.location, (1.0, 2.0, 3.0)))
              and all(abs(math.degrees(a) - b) < 1e-3
                      for a, b in zip(camera_object.rotation_euler, (4.0, 5.0, 6.0)))
              and abs(cam_data.intrinsics.fx - 111.0) < 1e-6
              and (cam_data.intrinsics.image_width, cam_data.intrinsics.image_height) == (111, 222),
              f"loc={list(camera_object.location)} "
              f"rot={[round(math.degrees(v), 3) for v in camera_object.rotation_euler]} "
              f"fx={cam_data.intrinsics.fx} "
              f"out={cam_data.intrinsics.image_width}x{cam_data.intrinsics.image_height}")
    finally:
        for name, (value, array) in original.items():
            setattr(settings, name, list(value) if array else value)


def test_avm_coverage():
    """Coverage curves and the report (footprints, blind/overlap, visibility)."""
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

    report = coverage_mod.analyze(settings, step=0.2)
    check("report has footprints and visibility",
          len(report.footprints) == 4 and len(report.visibility) == 4
          and report.field_ok is True,
          f"footprints={len(report.footprints)} field_ok={report.field_ok}")


def test_avm_corner_detection():
    """Detect the block corners in the rendered images and store them as points_2d."""
    from opencv_camera.bl.scenes.avm_scene import io as io_mod
    from opencv_camera.core.scenes import avm_coverage, avm_layout

    scene = setup_scene(resolution=256, samples=4)
    clear_scene()
    scene = setup_scene(resolution=256, samples=4)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene

    scene.render.resolution_x, scene.render.resolution_y = 256, 192

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

    from opencv_camera.bl.scenes.avm_scene import falcon
    config = falcon.build(settings)
    check("the Falcon config carries the detected points_2d",
          all(len(camera["points_2d"]) == 8 for camera in config["cameras"]))


def test_avm_corner_single_and_cache():
    """Per-camera detection, the annotated image and the revision cache."""
    from opencv_camera.bl.scenes.avm_scene import corners as corners_mod

    scene = setup_scene(resolution=256, samples=8)
    clear_scene()
    scene = setup_scene(resolution=256, samples=8)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene
    scene.render.resolution_x, scene.render.resolution_y = 256, 192

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
    scene.render.resolution_x, scene.render.resolution_y = 256, 192

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
    """The real AVM bowl loads by default, scales by radius/height, and toggles."""
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
          and ground.data.get(builder.GROUND_MODEL_KEY) == builder.ground_model_key(settings),
          f"{len(ground.data.vertices)} verts")
    check("the bowl sits on z = 0", abs(ground.location.z) < 1e-9, f"{ground.location.z}")
    check("the bowl keeps the real 30 m footprint",
          abs(ground.dimensions.x - 30.0) < 1e-3 and abs(ground.dimensions.y - 30.0) < 1e-3,
          f"{tuple(round(v, 3) for v in ground.dimensions)}")
    check("the rim reaches the real 5 m",
          abs(max(v.co.z for v in ground.data.vertices) - 5.0) < 1e-3,
          f"{max(v.co.z for v in ground.data.vertices):.3f}")

    # radius: half the footprint; the floor stays flat at z = 0
    settings.ground_radius = 10.0
    bpy.ops.opencv_cam.avm_rebuild()
    ground = bpy.data.objects["AVM_Ground"]
    check("Bowl Radius rescales the footprint",
          abs(ground.dimensions.x - 20.0) < 1e-3 and abs(ground.dimensions.y - 20.0) < 1e-3,
          f"{tuple(round(v, 3) for v in ground.dimensions)}")
    check("the floor still sits on z = 0",
          abs(min(v.co.z for v in ground.data.vertices)) < 1e-6,
          f"{min(v.co.z for v in ground.data.vertices):.5f}")
    check("the rim is unchanged by the radius",
          abs(max(v.co.z for v in ground.data.vertices) - 5.0) < 1e-3)

    # rim height: the wall, independent of the radius
    settings.ground_rim_height = 2.0
    bpy.ops.opencv_cam.avm_rebuild()
    ground = bpy.data.objects["AVM_Ground"]
    check("Rim Height rescales the wall",
          abs(max(v.co.z for v in ground.data.vertices) - 2.0) < 1e-3,
          f"{max(v.co.z for v in ground.data.vertices):.3f}")
    check("the radius survives the rim change",
          abs(ground.dimensions.x - 20.0) < 1e-3, f"{ground.dimensions.x:.3f}")
    settings.ground_radius = 15.0
    settings.ground_rim_height = 5.0

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


def test_avm_export_bowl():
    """Export Bowl writes the scaled ground mesh as a standalone GLB."""
    import struct

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("add AVM scene", bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    settings = scene.avm_scene
    settings.ground_radius = 8.0
    settings.ground_rim_height = 3.0
    bpy.ops.opencv_cam.avm_rebuild()

    path = os.path.join(TMPDL, "bowl.glb")
    check("export bowl operator",
          bpy.ops.opencv_cam.avm_export_bowl(filepath=path) == {"FINISHED"})
    check("bowl glb written", os.path.exists(path) and os.path.getsize(path) > 1000,
          f"{os.path.getsize(path) if os.path.exists(path) else 0} bytes")

    data = open(path, "rb").read()
    _, _, length = struct.unpack_from("<III", data, 0)
    offset, document = 12, None
    while offset < length:
        chunk_len, = struct.unpack_from("<I", data, offset)
        tag = data[offset + 4:offset + 8]
        if tag == b"JSON":
            document = json.loads(data[offset + 8:offset + 8 + chunk_len])
        offset += 8 + chunk_len
    check("bowl glb parses", document is not None)
    names = [node.get("name") for node in document.get("nodes", [])]
    check("bowl glb has the four ground faces",
          sorted(names) == ["NurbsPath.001", "NurbsPath.002",
                            "NurbsPath.003", "NurbsPath.004"], str(names))
    positions = [document["accessors"][primitive["attributes"]["POSITION"]]
                 for mesh in document["meshes"] for primitive in mesh["primitives"]]
    low = [min(a["min"][k] for a in positions) for k in range(3)]
    high = [max(a["max"][k] for a in positions) for k in range(3)]
    # glTF is Y-up: the bowl radius is X/Z, the rim height is Y
    check("bowl glb keeps the radius and rim height",
          abs(low[0] + 8.0) < 1e-3 and abs(high[0] - 8.0) < 1e-3
          and abs(low[1]) < 1e-3 and abs(high[1] - 3.0) < 1e-3,
          f"x[{low[0]:.2f},{high[0]:.2f}] y[{low[1]:.2f},{high[1]:.2f}]")

    # with the vehicle included, the car is exported as a node named "vehicle"
    # and the scene object keeps its AVM_Car name
    settings.bowl_include_vehicle = True
    with_vehicle = os.path.join(TMPDL, "bowl_vehicle.glb")
    check("export bowl with vehicle operator",
          bpy.ops.opencv_cam.avm_export_bowl(filepath=with_vehicle) == {"FINISHED"})
    check("the scene car keeps its name",
          bpy.data.objects.get("AVM_Car") is not None
          and bpy.data.objects.get("vehicle") is None)
    vehicle_data = open(with_vehicle, "rb").read()
    _, _, vehicle_length = struct.unpack_from("<III", vehicle_data, 0)
    offset, vehicle_document = 12, None
    while offset < vehicle_length:
        chunk_len, = struct.unpack_from("<I", vehicle_data, offset)
        tag = vehicle_data[offset + 4:offset + 8]
        if tag == b"JSON":
            vehicle_document = json.loads(vehicle_data[offset + 8:offset + 8 + chunk_len])
        offset += 8 + chunk_len
    vehicle_names = [node.get("name") for node in vehicle_document.get("nodes", [])]
    check("bowl glb with vehicle has the four ground faces and a 'vehicle' node",
          set(vehicle_names) == {"NurbsPath.001", "NurbsPath.002", "NurbsPath.003",
                                 "NurbsPath.004", "vehicle"}, str(vehicle_names))


def test_drive_scene():
    """The Drive Scene: car park, four OpenCV cameras, keyframed drive and a clip."""
    from opencv_camera.bl import scenes as scenes_mod
    from opencv_camera.bl.scenes.drive_scene import builder, properties, recording
    from opencv_camera.core.scenes import avm_cameras, avm_falcon, drive_lot, drive_path, vehicle

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)

    check("drive add operator registered", "drive_add_scene" in dir(bpy.ops.opencv_cam))
    check("add Drive scene", bpy.ops.opencv_cam.drive_add_scene() == {"FINISHED"})
    settings = scene.drive_scene
    definition = scenes_mod.definition("drive_scene")
    check("the registry knows the scene",
          definition is not None and definition.label == "Drive Scene", str(definition))
    check("root pointer set",
          settings.root is not None and settings.root.name == "DRIVE_Root",
          str(settings.root))
    check("panels would show", scenes_mod.has_scene(bpy.context, definition))

    # the clip quality presets: only the render cost changes, so they must get
    # cheaper from high to draft while the output size stays untouched
    draft = properties.clip_quality_preset("draft")
    balanced = properties.clip_quality_preset("balanced")
    high = properties.clip_quality_preset("high")
    check("the clip defaults to the draft quality",
          settings.clip_quality == "draft", settings.clip_quality)
    check("the clip quality presets get cheaper towards draft",
          draft["samples"] < balanced["samples"] < high["samples"],
          f"{draft['samples']} / {balanced['samples']} / {high['samples']}")
    check("the draft / balanced presets denoise and cap the bounces",
          draft["denoise"] and balanced["denoise"]
          and draft["max_bounces"] < balanced["max_bounces"])
    check("the draft / balanced presets raise the adaptive threshold and drop caustics",
          draft["adaptive_threshold"] > balanced["adaptive_threshold"]
          and draft["caustics"] is False and balanced["caustics"] is False)
    check("the high preset keeps the historical sample count",
          high == {"samples": 64}, str(high))
    check("an unknown quality falls back to balanced",
          properties.clip_quality_preset("nonsense") == balanced)
    check("the export default file name is stable, not the .blend's",
          recording.default_filename() == "drive_scene.zip",
          recording.default_filename())
    check("the export defaults to the CPU device",
          settings.clip_device == "cpu", settings.clip_device)
    check("the export keeps the PNG frames unless asked otherwise",
          settings.clip_keep_frames is True, settings.clip_keep_frames)
    check("the drive defaults to a constant speed (no acceleration)",
          settings.drive_profile == "constant", settings.drive_profile)
    check("only OptiX may render the OSL camera on a GPU",
          recording.gpu_can_render_osl_camera("OPTIX") is True
          and recording.gpu_can_render_osl_camera("METAL") is False
          and recording.gpu_can_render_osl_camera("CUDA") is False)
    device_messages = []
    check("asking for CPU never warns",
          recording.resolve_device("cpu", device_messages) == "CPU"
          and device_messages == [])
    device_messages = []
    device_type = recording.compute_device_type()
    requested = recording.resolve_device("gpu", device_messages)
    expected_device = ("GPU" if (recording.gpu_can_render_osl_camera(device_type)
                                 and recording.enabled_osl_gpu_available())
                       else "CPU")
    check("a GPU request resolves against the running backend",
          requested == expected_device, f"{device_type} -> {requested}")
    check("a GPU request that cannot run the OSL camera warns",
          expected_device == "GPU" or bool(device_messages),
          str(device_messages))
    check("format_duration formats an ETA",
          recording.format_duration(0) == "0:00"
          and recording.format_duration(65) == "1:05"
          and recording.format_duration(3725) == "1:02:05"
          and recording.format_duration(None) == "--:--",
          recording.format_duration(65))
    check("progress_text shows the frame, percent and ETA",
          recording.progress_text(5, 10, 61) == "frame 5/10 · 50% · ETA 1:01",
          recording.progress_text(5, 10, 61))

    target = bpy.data.collections.get("Drive Scene")
    names = sorted(obj.name for obj in target.objects)
    check("collection has the core objects",
          {"DRIVE_Car", "DRIVE_Ground", "DRIVE_Root", "DRIVE_Vehicle",
           "DRIVE_Cam_Front", "DRIVE_Cam_Back", "DRIVE_Cam_Left",
           "DRIVE_Cam_Right"} <= set(names), str(names))
    check("every object is namespaced",
          all(name.startswith("DRIVE_") for name in names), str(names))
    check("the four cameras are the same four rig roles the AVM Scene uses",
          sorted(name for name in names if name.startswith("DRIVE_Cam_"))
          == sorted(avm_cameras.object_name("DRIVE_Cam_", key)
                    for key in avm_cameras.CAMERAS),
          str(sorted(name for name in names if name.startswith("DRIVE_Cam_"))))
    cameras = {key: bpy.data.objects[avm_cameras.object_name("DRIVE_Cam_", key)]
               for key in avm_cameras.CAMERAS}
    for key, camera in cameras.items():
        check(f"the {key} camera is a compiled custom fisheye",
              camera.data.type == "CUSTOM" and len(camera.data.custom_bytecode) > 0
              and camera.data.opencv_cam.distortion.model == "fisheye",
              f"{camera.data.type}, {len(camera.data.custom_bytecode)} bytes")
        check(f"the {key} camera hangs off DRIVE_Vehicle with a vehicle-frame pose",
              camera.parent is not None and camera.parent.name == "DRIVE_Vehicle",
              str(camera.parent))
    check("all four cameras carry the same minibus calibration",
          len({round(c.data.opencv_cam.intrinsics.fx, 6) for c in cameras.values()}) == 1
          and approx(cameras["front"].data.opencv_cam.intrinsics.fx,
                     317.77563818112867, 1e-3),
          str([round(c.data.opencv_cam.intrinsics.fx, 4) for c in cameras.values()]))
    check("the front camera keeps its vehicle-frame mount pose",
          approx(cameras["front"].location.y, 2.466796, 1e-4)
          and approx(cameras["front"].location.z, 2.69068, 1e-4),
          str(tuple(round(v, 4) for v in cameras["front"].location)))
    check("every camera sits somewhere different (no copied pose)",
          len({tuple(round(v, 4) for v in c.location) for c in cameras.values()}) == 4,
          str([tuple(round(v, 3) for v in c.location) for c in cameras.values()]))
    check("the car and the cameras all hang off the vehicle empty",
          bpy.data.objects["DRIVE_Car"].parent.name == "DRIVE_Vehicle")

    # The clip renders at each camera's own calibration size and the Drive Scene
    # syncs the Render tab to it, so shrink the calibration to keep this test
    # fast: 96x72 instead of the 1280x960 default.
    for key in avm_cameras.CAMERAS:
        intrinsics = bpy.data.objects[
            avm_cameras.object_name("DRIVE_Cam_", key)].data.opencv_cam.intrinsics
        intrinsics.image_width, intrinsics.image_height = 96, 72

    # the camera record switches: four roles, all on by default, and the active
    # one drives the render camera
    check("the record switches mirror the four rig roles",
          [entry.name for entry in settings.cameras] == list(avm_cameras.CAMERAS),
          str([entry.name for entry in settings.cameras]))
    check("every camera is recorded by default",
          settings.recorded_cameras() == list(avm_cameras.CAMERAS),
          str(settings.recorded_cameras()))
    check("the active camera defaults to the front one and drives scene.camera",
          settings.active_camera == "front" and scene.camera.name == "DRIVE_Cam_Front",
          f"{settings.active_camera} -> {scene.camera.name}")
    settings.active_camera = "left"
    check("switching the active camera switches the render camera",
          scene.camera.name == "DRIVE_Cam_Left", scene.camera.name)
    check("the render resolution follows the active camera's calibration size",
          (scene.render.resolution_x, scene.render.resolution_y)
          == (cameras["left"].data.opencv_cam.intrinsics.image_width,
              cameras["left"].data.opencv_cam.intrinsics.image_height),
          f"{scene.render.resolution_x}x{scene.render.resolution_y}")
    settings.active_camera = "front"
    check("the default view frames the car and all four cameras",
          len(builder.view_targets(scene)) == 1 + len(avm_cameras.CAMERAS),
          str([obj.name for obj in builder.view_targets(scene)]))

    check("the car park is built (walls / pillars / one soft ceiling panel)",
          len([n for n in names if n.startswith("DRIVE_Wall_")]) == 4
          and len([n for n in names if n.startswith("DRIVE_Pillar_")]) == 4
          and [n for n in names if n.startswith("DRIVE_Light_")] == ["DRIVE_Light_Ceiling"],
          str(names))
    panel = bpy.data.objects["DRIVE_Light_Ceiling"]
    lot_length, lot_width = settings.lot_size()
    check("the ceiling panel spans the lot instead of lighting it in pools",
          panel.data.type == "AREA"
          and approx(panel.data.size, lot_width * 0.9, 1e-3)
          and approx(panel.data.size_y, lot_length * 0.9, 1e-3),
          f"{panel.data.size:.1f} x {panel.data.size_y:.1f} m")

    # the floor: real geometry for the markings, a procedural grain (a flat grey
    # floor would give a transparent-chassis algorithm nothing to align against)
    ground = bpy.data.objects["DRIVE_Ground"]
    slots = [material.name for material in ground.data.materials]
    check("the floor carries floor / white / yellow slots",
          slots == ["DRIVE_Floor_Concrete_Mat", "DRIVE_Paint_White_Mat",
                    "DRIVE_Paint_Yellow_Mat"], str(slots))
    check("the concrete floor has a procedural grain",
          any(node.type == "TEX_NOISE" for node in ground.data.materials[0].node_tree.nodes),
          str([node.type for node in ground.data.materials[0].node_tree.nodes]))
    floor_nodes = [node.type for node in ground.data.materials[0].node_tree.nodes]
    check("the floor has multi-scale mottling to align a reconstruction against",
          floor_nodes.count("TEX_NOISE") >= 2, str(floor_nodes))

    # every floor preset builds its own material (and the textured ones keep the
    # two noise scales); the default is restored afterwards
    for kind, expected in (("concrete", "DRIVE_Floor_Concrete_Mat"),
                           ("asphalt", "DRIVE_Floor_Asphalt_Mat"),
                           ("epoxy", "DRIVE_Floor_Epoxy_Mat"),
                           ("checker", "DRIVE_Floor_Checker_Mat"),
                           ("plain", "DRIVE_Floor_Plain_Mat")):
        settings.ground_texture = kind
        bpy.ops.opencv_cam.drive_rebuild()
        floor = bpy.data.objects["DRIVE_Ground"].data.materials[0]
        check(f"the {kind} floor preset builds its material",
              floor.name == expected, floor.name)
        if kind in ("concrete", "asphalt", "epoxy"):
            types = [node.type for node in floor.node_tree.nodes]
            check(f"the {kind} floor keeps two noise scales",
                  types.count("TEX_NOISE") == 2, str(types))
    settings.ground_texture = "concrete"
    bpy.ops.opencv_cam.drive_rebuild()
    ground = bpy.data.objects["DRIVE_Ground"]
    check("floor and both paint colours are used",
          {polygon.material_index for polygon in ground.data.polygons} == {0, 1, 2},
          str({polygon.material_index for polygon in ground.data.polygons}))
    check("the lane markings include the centre line",
          any(polygon.material_index == 1 and abs(polygon.center.x) < 0.1
              for polygon in ground.data.polygons),
          "no white marking on x = 0")
    length, width = settings.lot_size()
    check("the slab covers the whole drive",
          approx(ground.dimensions.y, length, 1e-3) and approx(ground.dimensions.x, width, 1e-3),
          f"{tuple(round(v, 2) for v in ground.dimensions)} vs {length:.2f} x {width:.2f}")

    # the bays: parked cars and their numbers
    parked = sorted(name for name in names if name.startswith("DRIVE_Parked_"))
    check("parked cars are built (4 per row)", len(parked) == 8, str(parked))
    first = bpy.data.objects[parked[0]]
    check("a parked car stands in its bay, nose to the wall",
          first.parent is not None and first.parent.name == "DRIVE_Root"
          and approx(abs(math.degrees(first.rotation_euler[2])), 90.0, 1e-4)
          and abs(first.location.x) > settings.aisle_width / 2,
          f"{first.name}: {math.degrees(first.rotation_euler[2]):.2f} deg, "
          f"x={first.location.x:.2f}")
    paints = {bpy.data.objects[name].data.materials[0].name for name in parked}
    check("the parked cars share a small paint palette", 1 < len(paints) <= 5, str(paints))

    number_count = 2 * drive_lot.bay_count(length, settings.bay_width)
    numbers = sorted(name for name in names if name.startswith("DRIVE_Number_"))
    check("every bay has its number", len(numbers) == number_count,
          f"{len(numbers)} of {number_count}")
    number = bpy.data.objects["DRIVE_Number_A01"]
    check("a number is flat text lying in the aisle",
          number.type == "FONT" and number.data.body == "A01"
          and abs(number.location.z - builder.NUMBER_Z) < 1e-9
          and all(abs(value) < 1e-9 for value in number.rotation_euler)
          and abs(number.location.x) < settings.aisle_width / 2,
          f"{number.type}, z={number.location.z:.3f}, x={number.location.x:.2f}")

    settings.show_parked = False
    check("hiding the parked cars works",
          all(bpy.data.objects[name].hide_render for name in parked))
    settings.show_parked = True
    check("showing them again works",
          not any(bpy.data.objects[name].hide_render for name in parked))
    structure = [name for name in names if name.startswith(("DRIVE_Wall_", "DRIVE_Pillar_"))]
    settings.show_walls = False
    check("hiding the structure leaves the ceiling panel on",
          all(bpy.data.objects[name].hide_render for name in structure)
          and not panel.hide_render, str(structure[:2]))
    settings.show_walls = True
    settings.bay_numbers = False
    bpy.ops.opencv_cam.drive_rebuild()
    check("turning the numbers off drops the objects",
          not [obj for obj in target.objects if obj.name.startswith("DRIVE_Number_")])
    settings.bay_numbers = True
    bpy.ops.opencv_cam.drive_rebuild()
    check("turning them back on rebuilds them",
          len([obj for obj in target.objects
               if obj.name.startswith("DRIVE_Number_")]) == number_count)

    # the add-on's own cameras, checked above; from here on just the front one
    camera = cameras["front"]

    # the keyframes are the plan, frame for frame
    plan = settings.plan()
    vehicle_empty = bpy.data.objects["DRIVE_Vehicle"]
    curves = compat.action_fcurves(vehicle_empty.animation_data.action)
    check("one key per frame on every curve",
          curves and all(len(curve.keyframe_points) == len(plan.frames) for curve in curves),
          str([len(curve.keyframe_points) for curve in curves]))
    check("the keys interpolate linearly",
          all(key.interpolation == "LINEAR" for curve in curves
              for key in curve.keyframe_points))
    worst = 0.0
    for frame in plan.frames[::max(1, len(plan.frames) // 7)]:
        scene.frame_set(frame.index)
        location = vehicle_empty.matrix_world.translation
        worst = max(worst, abs(location.x - frame.x), abs(location.y - frame.y))
    check("the keyframed vehicle follows the pure plan", worst < 1e-4, f"{worst:.2e}")
    check("the timeline is the clip",
          scene.frame_start == 0 and scene.frame_end == len(plan.frames) - 1
          and scene.render.fps == settings.drive_fps,
          f"{scene.frame_start}..{scene.frame_end} @ {scene.render.fps}")
    check("a rebuild keeps the objects",
          bpy.ops.opencv_cam.drive_rebuild() == {"FINISHED"}
          and bpy.data.objects.get("DRIVE_Car") is not None)

    # recording: a short clip at a tiny size, through the real operator path
    settings.drive_distance = 6.0
    settings.drive_speed = 3.0
    settings.drive_profile = "constant"
    settings.drive_fps = 4
    bpy.ops.opencv_cam.drive_rebuild()
    plan = settings.plan()
    check("the short clip is 9 frames", len(plan.frames) == 9, str(len(plan.frames)))

    scene.render.filepath = "//keep-me"
    # deliberately non-default render / timeline settings, to prove the recording
    # restores every knob it touches (device, adaptive, caustics, threads, frames)
    scene.frame_start, scene.frame_end = 3, 5
    scene.frame_step = 2
    scene.cycles.use_adaptive_sampling = False
    scene.cycles.adaptive_threshold = 0.42
    scene.cycles.caustics_reflective = True
    scene.cycles.caustics_refractive = True
    scene.render.threads_mode = "FIXED"
    scene.render.threads = 3
    before = (scene.render.filepath, scene.camera, scene.cycles.samples,
              scene.render.image_settings.file_format,
              scene.render.resolution_x, scene.render.resolution_y,
              scene.render.resolution_percentage,
              scene.cycles.device, scene.cycles.use_denoising,
              scene.cycles.max_bounces, scene.cycles.diffuse_bounces,
              scene.cycles.glossy_bounces, scene.render.use_persistent_data,
              scene.cycles.use_adaptive_sampling, scene.cycles.adaptive_threshold,
              scene.cycles.caustics_reflective,
              scene.cycles.caustics_refractive,
              scene.render.threads_mode, scene.render.threads,
              scene.frame_start, scene.frame_end, scene.frame_step)
    settings.clip_device = "gpu"
    directory = tempfile.mkdtemp(prefix="opencv_cam_drive_")
    report = recording.render_clip(bpy.context, settings, directory, samples=2)
    settings.clip_device = "cpu"
    expected_device = ("GPU" if (recording.gpu_can_render_osl_camera(
        recording.compute_device_type()) and recording.enabled_osl_gpu_available())
        else "CPU")
    check("the render clip resolves the requested device",
          report["device"] == expected_device,
          f"{recording.compute_device_type()} -> {report['device']}")
    check("render clip writes every frame", report["frames"] == len(plan.frames),
          str(report["frames"]))
    recorded = settings.recorded_cameras()
    check("the clip records the four rig roles",
          report["cameras"] == list(avm_cameras.CAMERAS),
          str(report["cameras"]))
    check("every recorded camera renders a png in every frame",
          all(os.path.exists(os.path.join(directory,
                                          recording.frame_name(frame.index, key)))
              for frame in plan.frames for key in recorded),
          str(sorted(os.listdir(directory))[:4]))
    check("the per-frame render names the pngs exactly like frame_name()",
          sorted(name for name in os.listdir(directory) if name.endswith(".png"))
          == sorted(recording.frame_name(frame.index, key)
                    for frame in plan.frames for key in recorded),
          str(sorted(os.listdir(directory))[:4]))
    check("the still count is frames x cameras, not frames",
          len([name for name in os.listdir(directory) if name.endswith(".png")])
          == len(plan.frames) * len(recorded),
          f"{len([n for n in os.listdir(directory) if n.endswith('.png')])} "
          f"vs {len(plan.frames)} x {len(recorded)}")
    rows = open(os.path.join(directory, "frames.csv"), encoding="utf-8").read().rstrip("\n").split("\n")
    check("frames.csv: header plus one row per frame",
          rows[0] == ",".join(drive_path.csv_header(recorded))
          and len(rows) == len(plan.frames) + 1,
          f"{len(rows)} rows")
    check("frames.csv carries one named group of six columns per camera",
          all(len(row.split(",")) == 7 + 6 * len(recorded) for row in rows[1:])
          and all(f"cam_{key}_x_m" in rows[0] and f"cam_{key}_yaw_deg" in rows[0]
                  for key in recorded),
          rows[0])
    check("frames.csv carries the speed column",
          all(float(row.split(",")[3]) > 0.0 for row in rows[1:]),
          rows[1] if len(rows) > 1 else "")
    mounts = recording.camera_mounts(recorded)
    first = rows[1].split(",")
    for index, key in enumerate(recorded):
        expected = drive_path.camera_world_pose(plan.frames[0], mounts[key])
        start = 7 + 6 * index
        check(f"frames.csv carries the {key} camera world pose",
              all(approx(float(first[start + i]), expected[i], 1e-3) for i in range(6)),
              first[start:start + 6])
    check("the four cameras' poses really do differ (no shared column group)",
          len({tuple(round(v, 3) for v in mounts[key].location) for key in recorded}) == 4,
          str({key: tuple(round(v, 3) for v in mounts[key].location)
               for key in recorded}))
    meta = json.load(open(os.path.join(directory, "clip.json"), encoding="utf-8"))
    check("clip.json describes the clip",
          meta["format"] == "drive_clip" and meta["frames"] == len(plan.frames)
          and meta["drive"]["profile"] == "constant"
          and approx(meta["drive"]["cruise_speed_mps"], 3.0, 1e-9),
          str({k: meta[k] for k in ("format", "frames")}))
    check("clip.json is version 2 and lists one entry per recorded camera",
          meta["version"] == 2 and len(meta["cameras"]) == len(recorded)
          and [entry["camera"] for entry in meta["cameras"]] == recorded,
          str([entry.get("camera") for entry in meta["cameras"]]))
    check("each camera entry names its object and carries K / D / mount",
          all(entry["name"] == avm_cameras.object_name("DRIVE_Cam_", entry["camera"])
              and entry["mount"]["frame"] == "vehicle"
              and len(entry["K"]) == 4 and len(entry["D"]) == 4
              and len(entry["mount"]["location"]) == 3
              and len(entry["mount"]["rotation_deg"]) == 3
              for entry in meta["cameras"]),
          str(meta["cameras"][0]))
    check("the single unnamed camera block is gone",
          "camera" not in meta, str(sorted(meta)))
    # The clip renders at the camera's own output size, so the recorded K is the
    # stored calibration verbatim (effective_intrinsics is the identity here)
    effective = {key: apply_mod.effective_intrinsics(
        bpy.data.objects[avm_cameras.object_name("DRIVE_Cam_", key)].data.opencv_cam,
        96, 72) for key in recorded}
    check("each entry's K / D / mount equals the camera object it names",
          all(entry["K"] == [float(effective[entry["camera"]].fx),
                             float(effective[entry["camera"]].fy),
                             float(effective[entry["camera"]].cx),
                             float(effective[entry["camera"]].cy)]
              and entry["output"] == [96, 72]
              and entry["mount"]["location"] == [
                  float(v) for v in bpy.data.objects[entry["name"]].location]
              for entry in meta["cameras"]),
          str([entry["K"] for entry in meta["cameras"]]))
    check("the recorded K is the camera's calibration, at its own size",
          all(approx(entry["K"][0], 317.77563818112867, 1e-3)
              for entry in meta["cameras"]),
          str(meta["cameras"][0]["K"]))
    check("clip.json names the camera keys, in recording order",
          meta["camera_names"] == recorded, str(meta["camera_names"]))
    check("clip.json states the file-name contract for stills and videos",
          meta["frame_pattern"] == "frame_%04d_<camera>.png"
          and meta["video_pattern"] == "<camera>.mp4",
          f"{meta['frame_pattern']!r} {meta['video_pattern']!r}")
    check("a multi-camera clip names no single video",
          meta["video"] == "", repr(meta["video"]))
    check("clip.json carries the real render size",
          tuple(meta["render"]["resolution"]) == (96, 72),
          str(meta["render"]))
    check("clip.json records the quality, device and sample override",
          meta["render"]["quality"] == "draft"
          and meta["render"]["device"] == expected_device
          and meta["render"]["samples"] == 2,
          str(meta["render"]))

    # the vehicle block: the geometry a consumer needs to build an ego mask, with
    # its frame named and exactly one copy per clip
    body = meta["vehicle"]["body"]
    car = bpy.data.objects["DRIVE_Car"]
    check("clip.json carries one vehicle block, in the vehicle frame",
          meta["vehicle"]["frame"] == "vehicle"
          and meta["vehicle"]["ground_clearance"] == settings.car_clearance,
          str(meta["vehicle"]))
    check("the vehicle block is the shared minibus geometry, at a readable precision",
          body == {"length": 4.8, "width": 2.4, "height": 2.88},
          str(body))
    check("the vehicle block carries the axle anchors",
          meta["vehicle"]["axles"] == {"wheel_base": 3.2, "rear_track": 1.8,
                                       "rear_center_offset": 2.8},
          str(meta["vehicle"]["axles"]))
    check("the exported width is the AVM-era body width, from the one source",
          body["width"] == avm_falcon.STEERING["body_width"] == vehicle.BODY_WIDTH_M,
          f"{body['width']} / {avm_falcon.STEERING['body_width']} / {vehicle.BODY_WIDTH_M}")
    # ... and the mesh really was built at that size: measure the body slab out
    # of the mesh's own body-material faces rather than re-reading the setting
    # (the wheels and the lamps are added proud of it, so car.dimensions is not it)
    body_verts = {index for polygon in car.data.polygons
                  if polygon.material_index == builder.CAR_BODY
                  for index in polygon.vertices}
    slab_x = max(abs(car.data.vertices[i].co.x) for i in body_verts)
    slab_y = max(abs(car.data.vertices[i].co.y) for i in body_verts)
    check("the rendered car mesh's body slab is exactly the exported box",
          approx(slab_x, body["width"] / 2.0, 1e-5)
          and approx(slab_y, body["length"] / 2.0, 1e-5),
          f"mesh {slab_x:.4f} x {slab_y:.4f} vs "
          f"{body['width'] / 2:.4f} x {body['length'] / 2:.4f}")
    check("the wheels and lamps do protrude, so the object is larger than the body",
          car.dimensions.x > body["width"] and car.dimensions.y > body["length"],
          f"{tuple(round(v, 4) for v in car.dimensions)} vs body {body}")
    # ... and the exported centre -> rear-axle transform really is the mesh's
    # wheel centre: the tyre ring is symmetric about its axle, so (max + min)/2
    # of the tyre vertices' y is the axle, measured off the mesh rather than
    # re-reading AXLE_FRACTION.
    tyre_verts = {index for polygon in car.data.polygons
                  if polygon.material_index == builder.CAR_TIRE
                  for index in polygon.vertices}
    tyre_y = [car.data.vertices[i].co.y for i in tyre_verts]
    rear_y = [y for y in tyre_y if y < 0]
    front_y = [y for y in tyre_y if y > 0]
    rear_axle = (max(rear_y) + min(rear_y)) / 2.0
    front_axle = (max(front_y) + min(front_y)) / 2.0
    exported_axle = meta["vehicle"]["center_to_rear_axle"]
    check("the exported rear axle is the mesh's rear wheel centre",
          approx(rear_axle, exported_axle[1], 1e-4)
          and approx(front_axle, -exported_axle[1], 1e-4)
          and exported_axle[0] == 0.0 and exported_axle[2] == 0.0,
          f"mesh rear {rear_axle:.4f} / front {front_axle:.4f} vs export {exported_axle}")

    # changing the Drive car's width must change the exported block: proof that
    # the export reads the scene, not a hardcoded 2.4
    settings.car_width = 2.1
    bpy.ops.opencv_cam.drive_rebuild()
    wide_dir = tempfile.mkdtemp(prefix="opencv_cam_width_")
    recording.render_clip(bpy.context, settings, wide_dir, samples=1)
    wider = json.load(open(os.path.join(wide_dir, "clip.json"), encoding="utf-8"))
    check("the exported vehicle block follows the scene's car width",
          wider["vehicle"]["body"]["width"] == 2.1,
          str(wider["vehicle"]["body"]))
    settings.car_width = 2.4
    bpy.ops.opencv_cam.drive_rebuild()
    # a rebuild re-applies the drive's own timeline (the clip IS the timeline),
    # so the deliberately odd 3..5 the restore check below uses has to go back
    scene.frame_start, scene.frame_end = 3, 5

    check("a render-only clip has no video and so no encode recipe",
          meta["video"] == "" and meta["video_encode"] == {},
          f"{meta['video']!r} {meta['video_encode']!r}")

    # a render-only clip never drops its stills - with no video they *are* the
    # artifact, whatever Keep Frames says
    settings.clip_keep_frames = False
    keep_dir = tempfile.mkdtemp(prefix="opencv_cam_keep_")
    try:
        kept = recording.render_clip(bpy.context, settings, keep_dir, samples=2)
    finally:
        settings.clip_keep_frames = True
    check("a render-only clip keeps its stills even with Keep Frames off",
          kept["frames"] == len(plan.frames)
          and all(os.path.exists(name) for name in kept["files"]),
          str(kept["files"][:3]))
    restored = (scene.render.filepath, scene.camera, scene.cycles.samples,
                scene.render.image_settings.file_format,
                scene.render.resolution_x, scene.render.resolution_y,
                scene.render.resolution_percentage,
                scene.cycles.device, scene.cycles.use_denoising,
                scene.cycles.max_bounces, scene.cycles.diffuse_bounces,
                scene.cycles.glossy_bounces, scene.render.use_persistent_data,
                scene.cycles.use_adaptive_sampling, scene.cycles.adaptive_threshold,
                scene.cycles.caustics_reflective,
                scene.cycles.caustics_refractive,
                scene.render.threads_mode, scene.render.threads,
                scene.frame_start, scene.frame_end, scene.frame_step)
    # name the knobs that did not come back - a bare tuple comparison makes the
    # one that drifted indistinguishable from the eighteen that did not
    knob_names = ("filepath", "camera", "samples", "file_format",
                  "resolution_x", "resolution_y", "resolution_percentage", "device",
                  "denoising", "max_bounces", "diffuse_bounces", "glossy_bounces",
                  "persistent", "adaptive", "adaptive_threshold", "caustics_reflective",
                  "caustics_refractive", "threads_mode", "threads", "frame_start",
                  "frame_end", "frame_step")
    drifted = [f"{name}: {b!r} -> {a!r}" for name, b, a in zip(knob_names, before, restored)
               if b != a]
    check("recording restored the render settings", not drifted, "; ".join(drifted))

    # export: the same clip plus one mp4 per camera, packed into one zip; no
    # explicit samples here, so the operator must take the selected quality preset
    settings.clip_quality = "balanced"
    zip_path = os.path.join(tempfile.mkdtemp(prefix="opencv_cam_zip_"), "clip.zip")
    check("export clip operator",
          bpy.ops.opencv_cam.drive_export_zip(filepath=zip_path) == {"FINISHED"})
    check("the export status is reported", settings.clip_status.startswith("9 frames"),
          settings.clip_status)
    with zipfile.ZipFile(zip_path) as archive:
        names = set(archive.namelist())
    videos = [recording.video_name(key) for key in recorded]
    check("the zip holds one mp4 per recorded camera, named after it",
          set(videos) == {"front.mp4", "back.mp4", "left.mp4", "right.mp4"}
          and set(videos) <= names,
          str(sorted(n for n in names if n.endswith(".mp4"))))
    check("the zip does not carry a single clip.mp4",
          "clip.mp4" not in names, str(sorted(n for n in names if n.endswith(".mp4"))))
    check("the zip holds the pngs, csv and json",
          {"frames.csv", "clip.json"} <= names
          and all(recording.frame_name(frame.index, key) in names
                  for frame in plan.frames for key in recorded),
          str(sorted(names)[:4]))
    export_meta = json.loads(zipfile.ZipFile(zip_path).read("clip.json"))
    check("a multi-camera clip.json names no single video",
          export_meta["video"] == "" and export_meta["video_pattern"] == "<camera>.mp4",
          f"{export_meta['video']!r} {export_meta.get('video_pattern')!r}")
    check("the export used the selected quality preset",
          export_meta["render"]["quality"] == "balanced"
          and export_meta["render"]["samples"] == 24
          and export_meta["render"]["device"] == "CPU",
          str(export_meta["render"]))

    # the encode recipe: what lets a harness tell "the algorithm drifted" from
    # "the encoder drifted" without shipping the stills
    recipe = export_meta.get("video_encode") or {}
    check("clip.json records the mp4's encode recipe",
          recipe.get("container") == "MPEG4" and recipe.get("codec") == "H264"
          and recipe.get("constant_rate_factor") == "HIGH"
          and recipe.get("fps") == plan.fps and bool(recipe.get("blender")),
          str(recipe))
    check("clip.json records the view transform the mp4 was encoded with",
          recipe.get("view_transform") == "Standard",
          str(recipe))

    # frame integrity, per lane: what the container reports is what the CSV has.
    # Blender's own movie reader is the decoder here, which is the same thing a
    # consumer does with OpenCV - the point is that the count survives encoding.
    movie_dir = tempfile.mkdtemp(prefix="opencv_cam_movie_")
    with zipfile.ZipFile(zip_path) as archive:
        for key in recorded:
            with open(os.path.join(movie_dir, recording.video_name(key)), "wb") as handle:
                handle.write(archive.read(recording.video_name(key)))
    for key in recorded:
        path = os.path.join(movie_dir, recording.video_name(key))
        movie = bpy.data.movieclips.load(path)
        try:
            check(f"the {key} mp4 decodes to as many frames as frames.csv has rows",
                  movie.frame_duration == len(rows) - 1,
                  f"{movie.frame_duration} vs {len(rows) - 1}")
            # the container is the stills' size, not the encoder scene's default
            check(f"the {key} mp4 is the render's resolution, not 1920x1080",
                  tuple(movie.size[:2]) == (96, 72),
                  str(tuple(movie.size[:2])))
            check(f"the {key} mp4 carries the clip's fps",
                  approx(movie.fps, plan.fps, 1e-6), str(movie.fps))
        finally:
            bpy.data.movieclips.remove(movie)

    # the stills are the encoder's input, not an artifact: dropping them leaves
    # the same clip minus the bytes nobody in the video pipeline reads
    settings.clip_keep_frames = False
    lean_path = os.path.join(tempfile.mkdtemp(prefix="opencv_cam_lean_"), "lean.zip")
    try:
        result = bpy.ops.opencv_cam.drive_export_zip(filepath=lean_path, samples=2)
    finally:
        settings.clip_keep_frames = True
    check("export clip without the frames", result == {"FINISHED"}, str(result))
    with zipfile.ZipFile(lean_path) as archive:
        lean_names = set(archive.namelist())
    check("the lean zip holds every video, the csv and the json but no pngs",
          {"frames.csv", "clip.json"} | set(videos) <= lean_names
          and not any(name.endswith(".png") for name in lean_names),
          str(sorted(lean_names)))
    check("dropping the frames shrinks the zip",
          os.path.getsize(lean_path) < os.path.getsize(zip_path),
          f"{os.path.getsize(lean_path)} vs {os.path.getsize(zip_path)}")
    lean_meta = json.loads(zipfile.ZipFile(lean_path).read("clip.json"))
    check("the lean export still carries the recipe and the per-camera entries",
          lean_meta["video_encode"]["view_transform"] == "Standard"
          and len(lean_meta["cameras"]) == len(recorded),
          str(lean_meta.get("video_encode")))

    # the recorded set is a setting, not a constant: dropping one camera must
    # drop one video, one CSV column group and one clip.json entry - and must
    # leave the vehicle block alone (the vehicle does not depend on the cameras)
    settings.camera("back").enable = False
    three_dir = tempfile.mkdtemp(prefix="opencv_cam_three_")
    three = recording.render_clip(bpy.context, settings, three_dir, samples=1)
    try:
        check("disabling a camera leaves three lanes",
              three["cameras"] == ["front", "left", "right"], str(three["cameras"]))
        three_rows = open(os.path.join(three_dir, "frames.csv"),
                          encoding="utf-8").read().rstrip("\n").split("\n")
        check("the csv follows the reduced set: 7 + 6 x 3 columns",
              len(three_rows[0].split(",")) == 7 + 6 * 3
              and "cam_back_x_m" not in three_rows[0],
              str(len(three_rows[0].split(","))))
        three_meta = json.load(open(os.path.join(three_dir, "clip.json"),
                                    encoding="utf-8"))
        check("clip.json follows the reduced set too",
              [entry["camera"] for entry in three_meta["cameras"]]
              == ["front", "left", "right"],
              str([entry["camera"] for entry in three_meta["cameras"]]))
        check("the unrecorded camera really rendered nothing",
              not any(name.endswith("_back.png") for name in os.listdir(three_dir)),
              str(sorted(os.listdir(three_dir))[:4]))
        check("the vehicle block is still a single copy",
              "vehicle" in three_meta
              and three_meta["vehicle"]["body"]["width"] == 2.4,
              str(three_meta["vehicle"]["body"]))
    finally:
        settings.camera("back").enable = True
        bpy.ops.opencv_cam.drive_rebuild()

    # a clip with no camera enabled cannot be recorded, and says so
    for key in avm_cameras.CAMERAS:
        settings.camera(key).enable = False
    no_lane_dir = tempfile.mkdtemp(prefix="opencv_cam_nolane_")
    try:
        recording.render_clip(bpy.context, settings, no_lane_dir, samples=1)
        check("a clip with no camera enabled is refused", False, "no error raised")
    except RuntimeError as exc:
        check("a clip with no camera enabled is refused",
              "no camera is enabled" in str(exc), str(exc))
    finally:
        settings.reset_cameras()
        bpy.ops.opencv_cam.drive_rebuild()
        check("Reset Defaults puts every camera back on record",
              settings.recorded_cameras() == list(avm_cameras.CAMERAS),
              str(settings.recorded_cameras()))

    # cancel: a cancelled render must be reported and must leave no zip behind
    original_step = recording.ClipJob.step

    def _cancelled_step(self):
        raise recording.RenderCancelled("test cancel")

    recording.ClipJob.step = _cancelled_step
    cancel_path = os.path.join(tempfile.mkdtemp(prefix="opencv_cam_cancel_"),
                               "cancel.zip")
    try:
        cancelled = bpy.ops.opencv_cam.drive_export_zip(filepath=cancel_path)
    finally:
        recording.ClipJob.step = original_step
    check("a cancelled export is reported and leaves no zip",
          cancelled == {"CANCELLED"} and not os.path.exists(cancel_path)
          and settings.clip_status == "cancelled",
          f"{cancelled} {settings.clip_status!r}")

    check("remove scene", bpy.ops.opencv_cam.drive_remove_scene() == {"FINISHED"})
    check("the panels hide after the remove",
          scenes_mod.has_scene(bpy.context, definition) is False)
    check("every camera object is gone too",
          not [obj for obj in bpy.data.objects if obj.name.startswith("DRIVE_Cam_")],
          str([obj.name for obj in bpy.data.objects
               if obj.name.startswith("DRIVE_Cam_")]))


def test_drive_matches_avm_defaults():
    """The Drive Scene must reproduce the AVM Scene's car and all four cameras.

    The two scenes are the same vehicle from the same calibration; this pins that
    the Drive Scene does not drift from the AVM defaults (car size and colour,
    and every camera's K / D / output size and mount pose), which is what makes
    the footage comparable and what lets the AVM renderer consume a clip.

    It is also the evidence for "adjust the AVM Scene and the Drive Scene
    follows": both read one source - the bundled calibration preset - so what is
    compared here is two readers of that source, not two copies of a number.
    """
    from opencv_camera.core.scenes import avm_cameras, avm_layout, vehicle

    def rgb_close(a, b, tol=1e-6):
        return all(abs(x - y) <= tol for x, y in zip(a[:3], b[:3]))

    clear_scene()
    scene = setup_scene(resolution=128, samples=4)

    check("build the AVM scene for comparison",
          bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"})
    avm = {key: bpy.data.objects[avm_cameras.object_name("AVM_Cam_", key)]
           for key in avm_cameras.CAMERAS}
    avm_body = tuple(bpy.data.materials["AVM_Car_Mat"].diffuse_color)

    check("build the Drive scene for comparison",
          bpy.ops.opencv_cam.drive_add_scene() == {"FINISHED"})
    drive = {key: bpy.data.objects[avm_cameras.object_name("DRIVE_Cam_", key)]
             for key in avm_cameras.CAMERAS}
    drive_body = tuple(bpy.data.materials["DRIVE_Car_Mat"].diffuse_color)

    palette = avm_layout.MINIBUS_MATERIALS["body"][0]
    check("the AVM and Drive ego cars share the body colour",
          rgb_close(avm_body, palette) and rgb_close(drive_body, palette)
          and rgb_close(avm_body, drive_body),
          f"avm={avm_body} drive={drive_body} palette={palette}")

    for attribute in ("car_length", "car_width", "car_height", "car_clearance"):
        a = getattr(scene.avm_scene, attribute)
        d = getattr(scene.drive_scene, attribute)
        check(f"the car {attribute} matches the AVM default", approx(a, d, 1e-9),
              f"{a} vs {d}")
    check("both cars are built from the shared geometry module",
          approx(scene.drive_scene.car_length, vehicle.BODY_LENGTH_M, 1e-6)
          and approx(scene.drive_scene.car_width, vehicle.BODY_WIDTH_M, 1e-6)
          and approx(scene.drive_scene.car_height, vehicle.BODY_HEIGHT_M, 1e-6),
          str((scene.drive_scene.car_length, scene.drive_scene.car_width,
               scene.drive_scene.car_height)))

    for key in avm_cameras.CAMERAS:
        a, d = avm[key].data.opencv_cam, drive[key].data.opencv_cam
        for attribute in ("fx", "fy", "cx", "cy"):
            check(f"the {key} camera {attribute} matches the AVM default",
                  approx(getattr(a.intrinsics, attribute),
                         getattr(d.intrinsics, attribute), 1e-9),
                  f"{getattr(a.intrinsics, attribute)} vs "
                  f"{getattr(d.intrinsics, attribute)}")
        check(f"the {key} camera calibration size matches the AVM default",
              (a.intrinsics.image_width, a.intrinsics.image_height)
              == (d.intrinsics.image_width, d.intrinsics.image_height),
              f"{a.intrinsics.image_width}x{a.intrinsics.image_height} vs "
              f"{d.intrinsics.image_width}x{d.intrinsics.image_height}")
        check(f"the {key} camera distortion matches the AVM default",
              (a.distortion.model, a.distortion.k1, a.distortion.k2,
               a.distortion.k3, a.distortion.k4)
              == (d.distortion.model, d.distortion.k1, d.distortion.k2,
                  d.distortion.k3, d.distortion.k4),
              f"{a.distortion.model} vs {d.distortion.model}")
        # AVM camera world pose == Drive camera vehicle-frame (local) pose
        worst = max(abs(avm[key].matrix_world[i][j] - drive[key].matrix_basis[i][j])
                    for i in range(4) for j in range(4))
        check(f"the {key} camera mount pose matches the AVM default", worst < 1e-6,
              f"max|delta|={worst:.2e}")

    # the AVM preset is the source both read; this is the proof that the Drive
    # Scene did not carry a second copy of any of the four poses
    records = avm_layout.cameras_from_preset(avm_layout.load_preset())
    check("the four Drive cameras carry the preset's four mount poses",
          all(approx(min(abs(drive[record["name"]].location[i]
                             - record["location"][i]) for i in (0, 1, 2)), 0.0, 1e-6)
              for record in records),
          str({record["name"]: tuple(round(v, 3) for v in record["location"])
               for record in records}))

    bpy.ops.opencv_cam.avm_remove_scene()
    bpy.ops.opencv_cam.drive_remove_scene()


def test_drive_sync_from_avm():
    """[Sync Cameras from AVM Scene] copies the live AVM cameras, one way.

    The reproducible source stays the bundled preset (the test above proves the
    Drive Scene reads it); this operator exists for the case where someone has
    adjusted the AVM Scene by hand and wants to see the same thing from the
    driving car without exporting and re-importing a preset first.
    """
    from opencv_camera.core.scenes import avm_cameras

    clear_scene()
    scene = setup_scene(resolution=64, samples=1)
    check("build both scenes",
          bpy.ops.opencv_cam.avm_add_scene() == {"FINISHED"}
          and bpy.ops.opencv_cam.drive_add_scene() == {"FINISHED"})

    nudge = {key: (0.11 * (index + 1), 0.0, 2.4 + 0.05 * index)
             for index, key in enumerate(avm_cameras.CAMERAS)}
    for key, location in nudge.items():
        camera = bpy.data.objects[avm_cameras.object_name("AVM_Cam_", key)]
        camera.location = location
        camera.data.opencv_cam.intrinsics.fx = 300.0 + len(key)

    check("the Drive cameras start on the preset, not on the nudged AVM poses",
          not any(abs(bpy.data.objects[avm_cameras.object_name("DRIVE_Cam_", key)]
                      .location.x - nudge[key][0]) < 1e-6
                  for key in avm_cameras.CAMERAS))

    check("sync from the AVM scene", bpy.ops.opencv_cam.drive_sync_cameras() == {"FINISHED"})
    for key in avm_cameras.CAMERAS:
        a = bpy.data.objects[avm_cameras.object_name("AVM_Cam_", key)]
        d = bpy.data.objects[avm_cameras.object_name("DRIVE_Cam_", key)]
        check(f"the {key} Drive camera took the AVM mount pose",
              approx(d.location.x, a.location.x, 1e-6)
              and approx(d.location.z, a.location.z, 1e-6),
              f"{tuple(round(v, 4) for v in d.location)} vs "
              f"{tuple(round(v, 4) for v in a.location)}")
        check(f"the {key} Drive camera took the AVM intrinsics",
              approx(d.data.opencv_cam.intrinsics.fx,
                     a.data.opencv_cam.intrinsics.fx, 1e-6),
              f"{d.data.opencv_cam.intrinsics.fx} vs {a.data.opencv_cam.intrinsics.fx}")
        check(f"the {key} Drive camera stays parented to the vehicle empty",
              d.parent is not None and d.parent.name == "DRIVE_Vehicle",
              str(d.parent))

    check("a rebuild keeps the synced poses (they are the objects' own)",
          bpy.ops.opencv_cam.drive_rebuild() == {"FINISHED"}
          and approx(bpy.data.objects["DRIVE_Cam_Front"].location.x,
                     nudge["front"][0], 1e-6),
          str(tuple(round(v, 4) for v in
                    bpy.data.objects["DRIVE_Cam_Front"].location)))

    bpy.ops.opencv_cam.avm_remove_scene()
    check("sync without an AVM scene is refused",
          bpy.ops.opencv_cam.drive_sync_cameras() == {"CANCELLED"})
    bpy.ops.opencv_cam.drive_remove_scene()


def test_road_scene():
    """The Road Scene: closed track, props, four cameras, 3D keyframes and a clip."""
    from opencv_camera.bl import scenes as scenes_mod
    from opencv_camera.bl.scenes import vehicle_mesh
    from opencv_camera.bl.scenes.drive_scene import builder as drive_builder
    from opencv_camera.bl.scenes.road_scene import builder, recording
    from opencv_camera.core.scenes import avm_cameras, avm_layout, road_path

    scene = setup_scene(resolution=64, samples=1)
    clear_scene()
    scene = setup_scene(resolution=64, samples=1)

    check("road add operator registered", "road_add_scene" in dir(bpy.ops.opencv_cam))
    check("add Road scene", bpy.ops.opencv_cam.road_add_scene() == {"FINISHED"})
    settings = scene.road_scene
    definition = scenes_mod.definition("road_scene")
    check("the registry knows the road scene",
          definition is not None and definition.label == "Road Scene", str(definition))
    check("root pointer set",
          settings.root is not None and settings.root.name == "ROAD_Root",
          str(settings.root))
    check("panels would show", scenes_mod.has_scene(bpy.context, definition))
    check("the export default name is stable, not the .blend's",
          recording.default_filename() == "road_scene.zip", recording.default_filename())
    check("the clip contract is version 3", recording.VERSION == 3, str(recording.VERSION))

    target = bpy.data.collections.get("Road Scene")
    names = sorted(obj.name for obj in target.objects)
    check("collection has the core objects",
          {"ROAD_Car", "ROAD_Ground", "ROAD_Root", "ROAD_Vehicle",
           "ROAD_Cam_Front", "ROAD_Cam_Back", "ROAD_Cam_Left",
           "ROAD_Cam_Right"} <= set(names), str(names))
    check("every object is namespaced",
          all(name.startswith("ROAD_") for name in names), str(names))

    cameras = {key: bpy.data.objects[avm_cameras.object_name("ROAD_Cam_", key)]
               for key in avm_cameras.CAMERAS}
    for key, camera in cameras.items():
        check(f"the {key} camera is a compiled custom fisheye",
              camera.data.type == "CUSTOM" and len(camera.data.custom_bytecode) > 0
              and camera.data.opencv_cam.distortion.model == "fisheye",
              f"{camera.data.type}, {len(camera.data.custom_bytecode)} bytes")
        check(f"the {key} camera hangs off ROAD_Vehicle",
              camera.parent is not None and camera.parent.name == "ROAD_Vehicle",
              str(camera.parent))
    check("all four cameras carry the same minibus calibration",
          len({round(c.data.opencv_cam.intrinsics.fx, 6) for c in cameras.values()}) == 1)
    check("every camera sits somewhere different (no copied pose)",
          len({tuple(round(v, 4) for v in c.location) for c in cameras.values()}) == 4)
    check("the default view frames the car and all four cameras",
          len(builder.view_targets(scene)) == 1 + len(avm_cameras.CAMERAS))

    # the Road Scene is the Drive Scene's twin: same calibration / mounts / mesh
    preset = {record["name"]: record
              for record in avm_layout.cameras_from_preset(avm_layout.load_preset())}
    same_calibration = True
    for key in avm_cameras.CAMERAS:
        record = preset[key]
        mount = avm_cameras.mount_of(record)
        camera = cameras[key]
        same_calibration = same_calibration and all(
            approx(a, b, 1e-5) for a, b in zip(camera.location, mount.location))
        same_calibration = same_calibration and all(
            approx(a, math.radians(b), 1e-5)
            for a, b in zip(camera.rotation_euler, mount.rotation_deg))
        same_calibration = same_calibration and approx(
            camera.data.opencv_cam.intrinsics.fx, record["K"][0], 1e-3)
    check("the road cameras use the AVM / Drive calibration and mount poses",
          same_calibration)
    check("the ego car is the shared Drive Scene mesh, not a look-alike",
          builder._car_mesh is vehicle_mesh.car_mesh
          and drive_builder._car_mesh is vehicle_mesh.car_mesh)

    # the track ------------------------------------------------------------
    track = settings.track()
    check("the built track closes and carries every road type",
          track.closed
          and {"straight", "curve", "slope_up", "slope_down"}
          == {segment.road_type for segment in track.segments},
          str(sorted({segment.road_type for segment in track.segments})))
    check("the default loop is the compact preset",
          settings.track_preset == "compact"
          and 118.0 < track.length < 130.0, f"{track.length:.2f} m")
    plan = settings.plan()
    arc = settings.parking_arc(track)
    expected_distance = track.length + (2.0 * arc.length if arc is not None else 0.0)
    check("the drive covers the loop and the parking manoeuvre",
          approx(plan.distance, expected_distance, 1e-6)
          and len(plan.frames) > 10
          and all(b.time >= a.time for a, b in zip(plan.frames, plan.frames[1:])),
          road_path.summary(plan))
    check("the parking manoeuvre starts and ends in the bay",
          arc is not None
          and approx(plan.frames[0].x, arc.bay.x, 1e-4)
          and approx(plan.frames[-1].x, arc.bay.x, 1e-4)
          and plan.frames[-1].direction == "reverse",
          f"({plan.frames[0].x:.2f},{plan.frames[0].y:.2f}) .. "
          f"({plan.frames[-1].x:.2f},{plan.frames[-1].y:.2f})")
    check("the parking plan carries the AVM vehicle signals",
          plan.frames[0].gear == "P"
          and any(frame.gear == "R" for frame in plan.frames)
          and any(abs(frame.steering_deg) > 1.0 for frame in plan.frames),
          f"gears={sorted({f.gear for f in plan.frames})}")

    # the keyframed vehicle carries the full 3D pose
    vehicle_obj = bpy.data.objects["ROAD_Vehicle"]
    sample = plan.frames[len(plan.frames) // 2]
    scene.frame_set(sample.index)
    check("the keyframed vehicle matches the pure plan",
          approx(vehicle_obj.location.x, sample.x, 1e-4)
          and approx(vehicle_obj.location.y, sample.y, 1e-4)
          and approx(vehicle_obj.location.z, sample.z, 1e-4)
          and approx(math.degrees(vehicle_obj.rotation_euler[0]), sample.pitch, 1e-3)
          and approx(math.degrees(vehicle_obj.rotation_euler[2]), sample.yaw, 1e-3),
          f"{tuple(round(v, 3) for v in vehicle_obj.location)}")
    check("the car hangs off the vehicle empty",
          bpy.data.objects["ROAD_Car"].parent.name == "ROAD_Vehicle")

    # the track mesh -------------------------------------------------------
    ground = bpy.data.objects["ROAD_Ground"]
    slots = [material.name for material in ground.data.materials]
    check("the track carries road / shoulder / grass / paint slots",
          len(slots) == 5 and slots[3] == "ROAD_Paint_White_Mat"
          and slots[4] == "ROAD_Paint_Yellow_Mat", str(slots))
    check("the asphalt surface has two procedural noise scales",
          [node.type for node in ground.data.materials[0].node_tree.nodes].count("TEX_NOISE") == 2,
          str([node.type for node in ground.data.materials[0].node_tree.nodes]))
    check("every track material slot is used",
          {polygon.material_index for polygon in ground.data.polygons} == {0, 1, 2, 3, 4},
          str(sorted({polygon.material_index for polygon in ground.data.polygons})))

    # the props ------------------------------------------------------------
    bay_cars = [name for name in names if name.startswith("ROAD_BayCar_")]
    pedestrians = [name for name in names if name.startswith("ROAD_Ped_")]
    trees = [name for name in names if name.startswith("ROAD_Tree_")]
    lamps = [name for name in names if name.startswith("ROAD_Lamp_")]
    signs = [name for name in names if name.startswith("ROAD_Sign_")]
    check("the parking bays carry one car each apart from the ego's",
          len(bay_cars) == int(settings.parking_bays) - 1, str(bay_cars))
    check("pedestrians number the setting",
          len(pedestrians) == int(settings.pedestrians), str(pedestrians))
    check("trees number the setting", len(trees) == int(settings.trees), str(trees))
    check("lamps number the setting", len(lamps) == int(settings.lamps), str(lamps))
    check("signs number the setting", len(signs) == int(settings.signs), str(signs))
    check("the props are parented to the root",
          all(bpy.data.objects[name].parent.name == "ROAD_Root"
              for name in bay_cars + pedestrians + trees + lamps + signs))
    check("the scene is lit by one even sun with a dim world (Drive's look)",
          [name for name in names if name.startswith("ROAD_Light_")]
          == ["ROAD_Light_Sun"]
          and approx(bpy.data.objects["ROAD_Light_Sun"].data.energy,
                     settings.light_energy, 1e-6),
          str([name for name in names if name.startswith("ROAD_Light_")]))

    # the zebra crossing: painted just before the up ramp, people waiting on it
    crossing = builder.crosswalk_distance(track)
    up_ramp = next(segment for segment in track.segments if segment.name == "ramp_up")
    check("the zebra crossing sits just before the up ramp",
          crossing is not None and up_ramp.start.s - 6.0 < crossing < up_ramp.start.s,
          f"{crossing} vs the ramp at {up_ramp.start.s:.2f} m")
    white_on = len([polygon for polygon in bpy.data.objects["ROAD_Ground"].data.polygons
                    if polygon.material_index == 3])
    settings.show_crosswalk = False
    bpy.ops.opencv_cam.road_rebuild()
    white_off = len([polygon for polygon in bpy.data.objects["ROAD_Ground"].data.polygons
                     if polygon.material_index == 3])
    settings.show_crosswalk = True
    bpy.ops.opencv_cam.road_rebuild()
    check("the zebra crossing paints extra white bars",
          white_on > white_off + 4, f"{white_on} vs {white_off} white faces")
    waiting = sorted(float(bpy.data.objects[name]["road_ped_distance"])
                     for name in pedestrians)
    check("two pedestrians wait at the crossing",
          sum(1 for distance in waiting if abs(distance - crossing) < 0.5) == 2,
          str([round(value, 2) for value in waiting]))

    # a per-segment drive: exactly one segment, labelled by road type -------
    settings.drive_segment = "ramp_up"
    settings.drive_direction = "forward"
    bpy.ops.opencv_cam.road_rebuild()
    segment_plan = settings.plan()
    check("a segment drive is labelled by its road type",
          all(frame.road_type == "slope_up" for frame in segment_plan.frames[:-1]),
          str({frame.road_type for frame in segment_plan.frames}))
    check("a segment drive is short and starts on its segment",
          segment_plan.frames[0].segment == "ramp_up"
          and segment_plan.distance <= 2.0 * settings.ramp_length + 1e-6,
          f"{segment_plan.distance:.2f} m")

    # reverse faces backwards and noses down on the climb ------------------
    settings.drive_direction = "reverse"
    bpy.ops.opencv_cam.road_rebuild()
    reverse_plan = settings.plan()
    check("reverse is labelled and noses down on the up ramp",
          all(frame.direction == "reverse" for frame in reverse_plan.frames)
          and min(frame.pitch for frame in reverse_plan.frames) < 0.0,
          f"min pitch {min(frame.pitch for frame in reverse_plan.frames):.3f}")

    # back to a short whole-loop clip we can actually render ----------------
    # (parking off here: the full manoeuvre clip is ~40 s and this test only
    # needs the export contract; the parking plan itself is asserted above)
    settings.drive_segment = "loop"
    settings.drive_direction = "forward"
    settings.drive_loops = 0.02
    settings.parking = False
    bpy.ops.opencv_cam.road_rebuild()
    check("walking pedestrians default off",
          settings.animate_pedestrians is False)
    check("a static scene uses persistent data",
          recording.PROFILE.persistent_data(settings) is True)

    # shrink the calibration to keep the render fast ------------------------
    for key in avm_cameras.CAMERAS:
        intrinsics = cameras[key].data.opencv_cam.intrinsics
        intrinsics.image_width, intrinsics.image_height = 64, 48

    directory = tempfile.mkdtemp(dir=TMPDL)
    report = recording.render_clip(bpy.context, settings, directory, samples=1)
    recorded = settings.recorded_cameras()
    check("the clip rendered every frame for every camera",
          report["frames"] == len(settings.plan().frames)
          and len([name for name in os.listdir(directory) if name.endswith(".png")])
          == report["frames"] * len(recorded),
          f"{report['frames']} frames")

    with open(os.path.join(directory, "frames.csv"), encoding="utf-8") as handle:
        rows = handle.read().rstrip("\n").split("\n")
    check("frames.csv header is the road contract",
          rows[0] == ",".join(road_path.csv_header(recorded)), rows[0])
    check("frames.csv keeps the drive columns then adds slope columns and labels",
          rows[0].startswith(",".join(road_path.CSV_VEHICLE_COLUMNS)),
          rows[0])
    check("frames.csv carries the steering / gear signal columns",
          ",steering_deg,gear," in rows[0], rows[0][:120])
    check("frames.csv has one row per frame",
          len(rows) == report["frames"] + 1, f"{len(rows)} rows")
    check("frames.csv rows carry the segment label",
          rows[1].split(",")[10] == settings.plan().frames[0].segment, rows[1])

    with open(os.path.join(directory, "clip.json"), encoding="utf-8") as handle:
        meta = json.load(handle)
    check("clip.json is the road contract v3",
          meta["format"] == "road_clip" and meta["version"] == 3, str(meta["version"]))
    check("clip.json describes the loop and its segments",
          len(meta["segments"]) == len(track.segments)
          and {record["road_type"] for record in meta["segments"]}
          == {"straight", "curve", "slope_up", "slope_down"},
          str([record["road_type"] for record in meta["segments"]]))
    check("clip.json carries the motion and road blocks",
          meta["motion"]["direction"] == "forward"
          and meta["motion"]["profile"] == "scenario"
          and meta["motion"]["parking"] is False
          and meta["road"]["surface"] == settings.ground_texture
          and meta["road"]["preset"] == "compact",
          str(meta["motion"]))
    check("clip.json describes the per-frame vehicle signals",
          meta["signals"]["columns"] == ["speed_mps", "steering_deg", "gear"]
          and meta["signals"]["gear_values"] == ["P", "R", "D"],
          str(meta.get("signals")))
    check("clip.json cameras match the recorded set",
          [entry["camera"] for entry in meta["cameras"]] == recorded, str(recorded))
    check("clip.json has one vehicle block",
          meta["vehicle"]["frame"] == "vehicle"
          and "body" in meta["vehicle"] and "center_to_rear_axle" in meta["vehicle"])

    # walking pedestrians key the scene and turn persistence off ------------
    settings.animate_pedestrians = True
    bpy.ops.opencv_cam.road_rebuild()
    walker = bpy.data.objects["ROAD_Ped_00"]
    check("a walking pedestrian is keyframed",
          walker.animation_data is not None and walker.animation_data.action is not None)
    check("an animated scene tells the renderer not to cache it",
          recording.PROFILE.persistent_data(settings) is False)
    settings.animate_pedestrians = False
    bpy.ops.opencv_cam.road_rebuild()
    check("a rebuilt static pedestrian drops its keys",
          bpy.data.objects["ROAD_Ped_00"].animation_data is None)

    # reset / remove --------------------------------------------------------
    settings.drive_speed = 11.0
    settings.drive_segment = "curve_0"
    bpy.ops.opencv_cam.road_reset_defaults()
    check("reset restores the template and drive defaults",
          approx(settings.drive_speed, 7.0, 1e-6)
          and approx(settings.drive_accel, 2.5, 1e-6)
          and approx(settings.drive_decel, 2.5, 1e-6)
          and approx(settings.slow_speed, 3.5, 1e-6)
          and settings.track_preset == "compact"
          and settings.parking is True
          and settings.drive_segment == "loop",
          f"{settings.drive_speed:.3f} / {settings.track_preset}")
    result = bpy.ops.opencv_cam.road_remove_scene()
    check("remove deletes the scene and its cameras",
          result == {"FINISHED"} and bpy.data.objects.get("ROAD_Root") is None
          and scene.road_scene.root is None
          and all(bpy.data.objects.get(avm_cameras.object_name("ROAD_Cam_", key)) is None
                  for key in avm_cameras.CAMERAS))


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
    check("the registry has all four scenes",
          ids == {"camera_scene", "avm_scene", "drive_scene", "road_scene"}, str(ids))
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


def test_render_resolution_drives_intrinsics():
    """The scene render resolution (Blender's Render tab) drives the intrinsics."""
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

    # same aspect (16:9) -> FOV preserving rescale to the scene resolution
    scene.render.resolution_x, scene.render.resolution_y = 640, 360
    apply_mod.apply_settings(cam_data, settings, scene)
    check("scene resolution drives the intrinsics",
          approx(cam_data.cycles_custom["fx"], 1500.0 * 640 / 1920, 1e-3),
          f"fx={cam_data.cycles_custom['fx']:.4f}")
    check("preview aspect follows the scene resolution",
          preview.preview_resolution(settings, "256") == (256, 144),
          str(preview.preview_resolution(settings, "256")))

    # different aspect -> centre crop, pixel pitch kept
    scene.render.resolution_x = scene.render.resolution_y = 640
    apply_mod.apply_settings(cam_data, settings, scene)
    check("aspect mismatch keeps the pixel pitch (crop)",
          approx(cam_data.cycles_custom["fx"], 1500.0, 1e-3),
          f"fx={cam_data.cycles_custom['fx']:.4f}")


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
        test_extrinsics_is_object_transform,
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
        test_avm_io_covers_every_setting,
        test_avm_coverage,
        test_avm_corner_detection,
        test_avm_corner_single_and_cache,
        test_avm_export_falcon,
        test_avm_visibility_and_logo,
        test_avm_ground_model,
        test_avm_export_bowl,
        test_drive_scene,
        test_drive_matches_avm_defaults,
        test_drive_sync_from_avm,
        test_road_scene,
        test_scene_default_view,
        test_shader_force_compile,
        test_presets,
        test_shader_text_upgrade_recompiles,
        test_live_apply,
        test_render_resolution_drives_intrinsics,
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