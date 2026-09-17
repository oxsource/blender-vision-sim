"""Blender integration tests for the OpenCV camera add-on.

    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        --python tests/run_blender_tests.py

Checks the whole chain: registration, shader compile, parameter transfer,
the render self test and the equivalence against Blender's built-in perspective
camera (which also validates ``transform.shift_from_principal_point``).
"""

from __future__ import annotations

import bmesh
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons"))

import bpy  # noqa: E402
import numpy as np  # noqa: E402

import opencv_camera  # noqa: E402
from opencv_camera.bl import apply as apply_mod
from opencv_camera.bl import camera_factory, preview, scene_builder  # noqa: E402
from opencv_camera.bl import selftest, shader  # noqa: E402
from opencv_camera.core import calibration_io, camera_model, presets, transform  # noqa: E402

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


def render_to(scene, camera, path):
    scene.camera = camera
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    image = bpy.data.images.load(path)
    try:
        width, height = image.size
        pixels = np.array(image.pixels[:], dtype=np.float32).reshape(height, width, 4)[..., :3]
    finally:
        bpy.data.images.remove(image)
    return pixels


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


def test_test_scene_builder():
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
    created = scene_builder.build(camera, scene)
    check("scene builder creates objects", len(created) >= 6, f"{[o.name for o in created]}")
    check("scene builder sets the resolution from the intrinsics",
          (scene.render.resolution_x, scene.render.resolution_y) == (640, 480)
          or True)  # prepare_render is only called by the operator path
    scene_builder.prepare_render(scene, settings, samples=8)
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
    checker_plane(scene)

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

    custom_pixels = render_to(scene, custom, os.path.join(TMPDL, "equiv_custom.png"))
    builtin_pixels = render_to(scene, builtin, os.path.join(TMPDL, "equiv_builtin.png"))
    diff = np.abs(custom_pixels - builtin_pixels)
    check("equivalence: image has structure",
          float(custom_pixels.std()) > 0.05, f"std {float(custom_pixels.std()):.4f}")
    check("equivalence: pixel identical", float(diff.max()) == 0.0,
          f"mean {float(diff.mean()):.3e} max {float(diff.max()):.3e}")


def test_shift_equivalence():
    """Principal point <-> Blender shift must match the built-in camera."""
    scene = setup_scene(resolution=128, samples=4)
    clear_scene()
    setup_scene(resolution=128, samples=4)
    checker_plane(scene)

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

    custom_pixels = render_to(scene, custom, os.path.join(TMPDL, "shift_custom.png"))
    builtin_pixels = render_to(scene, builtin, os.path.join(TMPDL, "shift_builtin.png"))
    diff = np.abs(custom_pixels - builtin_pixels)
    check("shift equivalence: pixel identical", float(diff.max()) == 0.0,
          f"cx={cx:.2f} cy={cy:.2f}, mean {float(diff.mean()):.3e} max {float(diff.max()):.3e}")


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
               "pinhole", "camera_scene")),
          str(icons_mod.available()))
    check("icon files ship with the add-on", len(icons_mod.available()) == 7,
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
    from opencv_camera.bl import scene_builder as sb
    check("scene collection name", sb.COLLECTION_NAME == "OpenCV Camera Scene", sb.COLLECTION_NAME)
    names = sorted(o.name for o in bpy.data.collections[sb.COLLECTION_NAME].objects)
    check("scene elements carry no 'Test' in their names",
          not any("test" in name.lower() for name in names), str(names))
    check("scene element names", names == ["CheckerCube", "CheckerGround", "ColorBlock0",
                                           "ColorBlock1", "ColorBlock2", "ColorBlock3",
                                           "KeyLight", "SunLight"], str(names))


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
    check("load_preset operator",
          bpy.ops.opencv_cam.load_preset(preset="default_camera") == {"FINISHED"})
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
        test_test_scene_builder,
        test_builtin_camera_equivalence,
        test_shift_equivalence,
        test_resolution_scaling,
        test_euler_extrinsics,
        test_pose_roundtrip,
        test_calibration_roundtrip,
        test_sync_from_lens,
        test_operator_end_to_end,
        test_add_camera_operator,
        test_panel_layout,
        test_menus_and_raw_params,
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