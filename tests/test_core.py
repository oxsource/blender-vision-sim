#!/usr/bin/env python3
"""Unit tests for the add-on core (no ``bpy``, runs with any python3).

    python3 tests/test_core.py
"""

from __future__ import annotations

import json
import math
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons", "opencv_camera"))

from core import calibration_io, camera_model, transform  # noqa: E402
from core.scenes import (avm_cameras, avm_coverage, avm_falcon, avm_layout,  # noqa: E402
                         drive_lot, drive_path, vehicle)

FAILURES = []


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name}" + (f" - {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_intrinsics():
    intr = camera_model.Intrinsics.auto(800.0, 810.0, 1920, 1080)
    resolved = intr.resolved()
    check("auto centre", approx(resolved.cx, 960.0) and approx(resolved.cy, 540.0))
    scaled = resolved.scaled(960, 540)
    check("scale halves focal length", approx(scaled.fx, 400.0) and approx(scaled.fy, 405.0))
    check("scale keeps fov", approx(intr.hfov_deg(), scaled.hfov_deg(), 1e-6))
    check("auto stays auto through scaling",
          camera_model.Intrinsics.auto(800.0, 800.0, 1920, 1080).scaled(640, 480).auto_center)


def test_distortion_roundtrip():
    dist = camera_model.Distortion.from_coefficients([-0.28, 0.073, 1.2e-4, -3.4e-5, 0.0])
    check("model detected", dist.model == camera_model.MODEL_BROWN_CONRADY)
    worst = 0.0
    for x, y in ((0.0, 0.0), (0.3, -0.2), (0.6, 0.4), (-0.5, 0.5)):
        xd, yd = camera_model.distort(x, y, dist)
        xu, yu, converged = camera_model.undistort_converged(xd, yd, dist, 20)
        worst = max(worst, abs(xu - x), abs(yu - y))
        if not converged:
            check("undistort converged", False, f"{x},{y}")
    check("undistort round trip", worst < 1e-7, f"max error {worst:.3e}")

    rational = camera_model.Distortion.from_coefficients(
        [-0.28, 0.073, 0.0, 0.0, 0.0, 0.01, -0.002, 0.0001]
    )
    check("rational detected", rational.model == camera_model.MODEL_RATIONAL)
    xd, yd = camera_model.distort(0.4, 0.3, rational)
    xu, yu, _ = camera_model.undistort_converged(xd, yd, rational, 30)
    check("rational round trip", abs(xu - 0.4) < 1e-6 and abs(yu - 0.3) < 1e-6)


def test_fisheye():
    dist = camera_model.Distortion.from_coefficients(
        [0.08476733270570755, 0.043184113434448945, -0.037989564107367736, 0.009428162434166068],
        model=camera_model.MODEL_FISHEYE,
    )
    check("fisheye model kept", dist.model == camera_model.MODEL_FISHEYE
          and dist.coefficients(4)[0] == dist.k1)
    worst = 0.0
    for r in (0.05, 0.2, 0.5, 0.9, 1.3, 1.6):
        x, y = r, 0.3 * r
        n = math.hypot(x, y)
        x, y = x / n * r, y / n * r
        xd, yd = camera_model.distort(x, y, dist)
        xu, yu, converged = camera_model.undistort_converged(xd, yd, dist, 30)
        worst = max(worst, abs(xu - x), abs(yu - y))
        if not converged:
            check("fisheye undistort converged", False, f"r={r}")
    check("fisheye round trip", worst < 1e-6, f"max error {worst:.3e}")

    # theta beyond 90 degrees must stay a valid ray (image corner of the AVM camera)
    intr = camera_model.Intrinsics(fx=317.77563818112867, fy=318.0250964604786,
                                   cx=636.2327868307656, cy=477.8201435641188,
                                   width=1280, height=960)
    ray = camera_model.ray_from_pixel(0.0, 0.0, intr, dist)
    theta = math.degrees(math.acos(max(-1.0, min(1.0, ray[2]))))
    check("fisheye corner ray valid", math.isfinite(theta) and theta < 140.0,
          f"theta at the top-left corner = {theta:.2f} deg")
    check("fisheye ray is unit length", approx(math.sqrt(sum(c * c for c in ray)), 1.0, 1e-9))

    # inside the valid domain (theta < 90 deg) pixel -> ray -> pixel must be exact
    radius = camera_model.fisheye_valid_radius_px(intr, dist)
    check("fisheye valid radius", 600.0 < radius < 650.0, f"{radius:.1f} px")
    samples = ((640.0, 480.0), (200.0, 700.0), (1000.0, 300.0), (640.0 + radius - 5.0, 480.0))
    worst = 0.0
    for u, v in samples:
        x, y, z = camera_model.ray_from_pixel(u, v, intr, dist)
        u2, v2, _ = camera_model.project_point(x * 5.0, y * 5.0, z * 5.0, intr, dist)
        worst = max(worst, abs(u2 - u), abs(v2 - v))
    check("fisheye pixel round trip (valid domain)", worst < 1e-6, f"max error {worst:.3e}")

    disabled = camera_model.Distortion.from_coefficients(
        list(dist.coefficients(4)), model=camera_model.MODEL_FISHEYE, enabled=False)
    xd, yd = camera_model.distort(0.5, 0.3, disabled)
    check("fisheye disabled == pinhole", (xd, yd) == (0.5, 0.3))


def test_projection():
    intr = camera_model.Intrinsics.auto(800.0, 800.0, 1280, 720)
    dist = camera_model.Distortion.from_coefficients([-0.1, 0.01, 0.0, 0.0, 0.0])
    u, v, depth = camera_model.project_point(0.4, -0.3, 2.0, intr, dist)
    x, y, z = camera_model.ray_from_pixel(u, v, intr, dist)
    check("project/ray z", approx(z, 1.0, 1e-12))
    check("project/ray round trip", abs(x - 0.2) < 1e-6 and abs(y + 0.15) < 1e-6,
          f"ray ({x:.6f}, {y:.6f}) vs (0.2, -0.15)")
    centre_ray = camera_model.ray_from_pixel(640.0, 360.0, intr, dist)
    check("pixel centre maps to optical axis",
          approx(centre_ray[0], 0.0, 1e-12) and approx(centre_ray[1], 0.0, 1e-12))


def test_transform():
    R_cv = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    t_cv = (0.1, 0.2, 3.0)
    R_b, t_b = transform.world_to_camera_to_blender(R_cv, t_cv)
    check("flip applied", R_b[4] == -R_cv[4] and t_b == (0.1, -0.2, -3.0))
    centre = transform.camera_center(R_b, t_b)
    # origin of the world frame must project to zero: R_b @ centre + t_b == 0
    projected = transform.mat3_vec(R_b, centre)
    check("camera centre consistent",
          all(approx(projected[i] + t_b[i], 0.0, 1e-12) for i in range(3)))
    matrix = transform.object_matrix_from_opencv(R_cv, t_cv)
    rotation, translation = transform.rt_from_mat4(matrix)
    check("object matrix round trip", rotation == tuple(transform.mat3_transpose(R_b))
          and translation == centre)
    check("flip is involutive",
          transform.blender_to_world_to_camera(R_b, t_b)[0] == tuple(R_cv))


def test_lens_conversion():
    fx = transform.fx_from_lens_mm(50.0, 36.0, 128)
    check("50mm/36mm/128px -> 177.78px", approx(fx, 50.0 / 36.0 * 128.0, 1e-9))
    check("lens round trip", approx(transform.lens_mm_from_fx(fx, 36.0, 128), 50.0, 1e-12))
    cx, cy = transform.principal_point_from_shift(0.1, 0.15, 128, 128)
    check("principal point from shift (measured convention)",
          approx(cx, 51.2, 1e-12) and approx(cy, 83.2, 1e-12))
    back = transform.shift_from_principal_point(cx, cy, 128, 128)
    check("shift round trip", approx(back[0], 0.1, 1e-12) and approx(back[1], 0.15, 1e-12))


def _write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return path


def test_bundled_preset():
    """The AVM preset shipped with the add-on must load as a fisheye camera."""
    preset = os.path.join(ROOT, "addons", "opencv_camera", "presets", "default_camera.yaml")
    calib = calibration_io.load_calibration(preset)
    check("preset intrinsics", approx(calib.intrinsics.fx, 317.77563818112867)
          and approx(calib.intrinsics.cy, 477.8201435641188)
          and (calib.width, calib.height) == (1280, 960))
    check("preset model is fisheye", calib.distortion.model == camera_model.MODEL_FISHEYE)
    check("preset coefficients", approx(calib.distortion.k1, 0.08476733270570755)
          and approx(calib.distortion.k4, 0.009428162434166068)
          and calib.distortion.coefficients(4) == [calib.distortion.k1, calib.distortion.k2,
                                                   calib.distortion.k3, calib.distortion.k4])


def test_filament_avm_config():
    """A multi-camera app config (K/D/input_size, nested K) must be importable."""
    path = _avm_config_path()
    if not os.path.exists(path):
        print("[SKIP] filament_avm config not present")
        return
    calib = calibration_io.load_calibration(path, camera_name="front")
    check("avm config intrinsics", approx(calib.intrinsics.fx, 317.77563818112867)
          and approx(calib.intrinsics.cy, 477.8201435641188))
    check("avm config size from input_size", (calib.width, calib.height) == (1280, 960))
    # the file has no distortion_model, so the 4 values are read with the default
    # (Brown-Conrady) meaning; the UI/model selector is where fisheye is chosen
    check("avm config coefficients parsed",
          approx(calib.distortion.k1, 0.08476733270570755)
          and approx(calib.distortion.p1, -0.037989564107367736)
          and calib.model_name == "")
    other = calibration_io.load_calibration(path, camera_name="left")
    check("avm config camera selection", other.source.endswith("vehicle_avm_minibus.json"))


def _avm_config_path() -> str:
    return os.path.normpath(os.path.join(
        ROOT, "..", "..", "..", "codes", "filament_avm", "configs",
        "vehicle_avm_minibus.json"))


def test_avm_layout():
    """Field equation, block geometry, the points(camera) contract and Store IO."""
    field = avm_layout.FieldSpec()
    check("field equation matches the HTML tool",
          (field.scene_w, field.scene_h) == (480.0, 840.0),
          f"{field.scene_w}x{field.scene_h}")

    preset = avm_layout.load_preset()
    field = avm_layout.field_from_preset(preset)
    cameras = avm_layout.cameras_from_preset(preset)
    check("preset field", (field.core_w, field.core_h, field.corner,
                           field.inner_w, field.inner_h) == (240.0, 480.0, 100.0, 20.0, 80.0))
    check("preset has the four cameras",
          [camera["name"] for camera in cameras] == ["front", "back", "left", "right"])

    geo = avm_layout.geometry(field)
    check("geometry half extents",
          approx(geo.core_hx, 1.2) and approx(geo.core_hy, 2.4)
          and approx(geo.c_in_x, 1.4) and approx(geo.c_out_x, 2.4)
          and approx(geo.c_in_y, 3.2) and approx(geo.c_out_y, 4.2)
          and approx(geo.half_x, 2.4) and approx(geo.half_y, 4.2))

    blocks = avm_layout.block_rects(field)
    check("four identical blocks",
          all(abs((x1 - x0) - 1.0) < 1e-9 and abs((y1 - y0) - 1.0) < 1e-9
              for x0, y0, x1, y1 in blocks.values()))
    check("block positions",
          blocks[avm_layout.BLOCK_FRONT_LEFT] == (-2.4, 3.2, -1.4, 4.2)
          and blocks[avm_layout.BLOCK_BACK_RIGHT] == (1.4, -4.2, 2.4, -3.2))

    # border only changes the field extent, never the blocks
    wider = field.replaced(border_w=50.0, border_h=50.0)
    check("border does not move the blocks",
          avm_layout.block_rects(wider) == blocks
          and avm_layout.geometry(wider).half_x == geo.half_x + 0.5)

    # the points(camera) contract: byte-for-byte the config's points_3d
    path = _avm_config_path()
    if os.path.exists(path):
        with open(path, encoding="utf-8") as handle:
            config = json.load(handle)
        worst = 0.0
        for camera in config["cameras"]:
            expected = camera["points_3d"]
            got = avm_layout.points(camera["name"], field)
            check(f"points({camera['name']}) count", len(got) == 8)
            worst = max(worst, max(abs(a - b) for g, e in zip(got, expected)
                                   for a, b in zip(g, e)))
        check("points(camera) matches the minibus config", worst < 1e-9, f"max err {worst:.1e}")
    else:
        print("[SKIP] filament_avm config not present: points(camera) not compared")

    # Store JSON (HTML / App interchange)
    store = avm_layout.to_store(field)
    check("Store JSON layout",
          store == {"border": "0x0", "corner": 100, "inner": "20x80", "car": "240x480"},
          str(store))
    check("Store round trip", avm_layout.from_store(store) == field)
    check("size_from is tolerant",
          avm_layout.size_from("10x20") == (10.0, 20.0)
          and avm_layout.size_from(50) == (50.0, 50.0)
          and avm_layout.size_from([1, 2]) == (1.0, 2.0)
          and avm_layout.size_from({"width": 3, "height": 4}) == (3.0, 4.0)
          and avm_layout.size_from("nonsense") is None)
    check("Store import keeps defaults for missing keys",
          avm_layout.from_store({"corner": 50}).corner == 50.0)


def test_avm_coverage():
    """Footprints, point coverage and the block visibility matrix."""
    preset = avm_layout.load_preset()
    field = avm_layout.field_from_preset(preset)
    cameras = avm_layout.cameras_from_preset(preset)
    models = {camera["name"]: avm_coverage._camera_models(camera) for camera in cameras}

    # a forward-facing camera sees in front of the car, not behind it
    intr, dist, matrix = models["front"]
    check("front camera covers the front blocks",
          avm_coverage.covers_ground_point((0.0, 3.7), intr, dist, matrix))
    check("front camera does not cover behind the car",
          not avm_coverage.covers_ground_point((0.0, -3.7), intr, dist, matrix))
    uv = avm_coverage.project_ground_point((0.0, 3.7), intr, dist, matrix)
    back = (avm_coverage.ground_point(uv[0], uv[1], intr, dist, matrix)
            if uv is not None else None)
    check("ground point projection round trip",
          uv is not None and back is not None
          and abs(back[0]) < 1e-6 and abs(back[1] - 3.7) < 1e-6,
          f"{uv} -> {back}")
    check("camera centre round trip",
          all(abs(a - b) < 1e-9 for a, b in
              zip(avm_coverage.camera_centre(matrix), cameras[0]["location"])))

    fp = avm_coverage.footprint(cameras[0], samples=64)
    check("footprint has ground points", len(fp.points) > 16, str(len(fp.points)))
    check("footprint points are finite",
          all(abs(x) < 1e4 and abs(y) < 1e4 for x, y in fp.points))
    check("footprint closed flag is a bool", isinstance(fp.closed, bool))

    report = avm_coverage.coverage_report(cameras, field, samples=64, step=0.05)
    check("coverage report is consistent",
          report.union_area <= report.field_area + 1e-9
          and report.overlap_area <= report.union_area + 1e-9
          and 0.0 <= report.coverage_ratio <= 1.0,
          f"union {report.union_area:.2f} overlap {report.overlap_area:.2f} "
          f"ratio {report.coverage_ratio:.3f}")

    visibility = report.visibility
    check("front_left seen by front and left",
          visibility["front_left"]["front"] and visibility["front_left"]["left"]
          and not visibility["front_left"]["back"])
    check("back_right seen by back and right",
          visibility["back_right"]["back"] and visibility["back_right"]["right"]
          and not visibility["back_right"]["front"])
    check("every block is seen by two cameras",
          all(sum(seen.values()) == 2 for seen in visibility.values()),
          str(visibility))
    check("field_ok is true for the minibus field", report.field_ok)


def test_calibration_io():
    tmp = tempfile.mkdtemp(prefix="opencv_cam_test_")
    opencv_yaml = """
%YAML:1.0
---
image_width: 1920
image_height: 1080
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1.3159e+03, 0., 9.55e+02, 0., 1.3150e+03, 5.4e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -2.831e-01, 7.34e-02, 1.2e-04, -3.4e-05, 0. ]
"""
    calib = calibration_io.load_calibration(_write(tmp, "opencv.yaml", opencv_yaml))
    check("opencv yaml intrinsics", approx(calib.intrinsics.fx, 1315.9) and approx(calib.intrinsics.cy, 540.0))
    check("opencv yaml size", (calib.width, calib.height) == (1920, 1080))
    check("opencv yaml distortion", approx(calib.distortion.k1, -0.2831) and approx(calib.distortion.p2, -3.4e-5))

    ros_yaml = """
image_width: 640
image_height: 480
camera_name: front
camera_matrix:
  rows: 3
  cols: 3
  data: [400.0, 0.0, 320.0, 0.0, 401.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.1, 0.01, 0.0, 0.0, 0.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [399.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
    calib = calibration_io.load_calibration(_write(tmp, "camera_info.yaml", ros_yaml))
    check("ros intrinsics", (calib.intrinsics.fx, calib.intrinsics.fy) == (400.0, 401.0))
    check("ros rectify", calib.rectify_intrinsics is not None
          and approx(calib.rectify_intrinsics.fx, 399.0))

    kalibr = """
cam0:
  camera_model: pinhole
  intrinsics: [461.629, 460.152, 362.680, 246.049]
  distortion_model: radtan
  distortion_coeffs: [-0.278874, 0.075067, 0.000276, 0.000016]
  resolution: [752, 480]
"""
    calib = calibration_io.load_calibration(_write(tmp, "camchain.yaml", kalibr))
    check("kalibr intrinsics", approx(calib.intrinsics.fx, 461.629) and approx(calib.intrinsics.cy, 246.049))
    check("kalibr resolution", (calib.width, calib.height) == (752, 480))

    intr = camera_model.Intrinsics(fx=500.0, fy=500.0, cx=319.5, cy=239.5, width=640, height=480)
    dist = camera_model.Distortion.from_coefficients([-0.2, 0.03, 1e-3, -2e-3, 0.0])
    out_json = calibration_io.save_calibration(os.path.join(tmp, "out.json"), calibration_io.Calibration(intr, dist))
    reloaded = calibration_io.load_calibration(out_json)
    check("json round trip", approx(reloaded.intrinsics.fx, 500.0)
          and approx(reloaded.distortion.p1, 1e-3) and approx(reloaded.distortion.k1, -0.2))
    payload = json.load(open(out_json))
    check("json layout", payload["camera_matrix"][0] == 500.0 and len(payload["distortion_coefficients"]) == 5)

    out_yaml = calibration_io.save_calibration(os.path.join(tmp, "out.yaml"), calibration_io.Calibration(intr, dist))
    reloaded = calibration_io.load_calibration(out_yaml)
    check("yaml round trip", approx(reloaded.intrinsics.fy, 500.0) and approx(reloaded.distortion.k2, 0.03))


def test_avm_falcon_config():
    """The Falcon config structure mirrors mediapipe_avm_calib JsonGenerator."""
    preset = avm_layout.load_preset()
    field = avm_layout.field_from_preset(preset)
    cameras = avm_layout.cameras_from_preset(preset)
    points_2d = {camera["name"]: [[100.0 + index, 200.0 + index]
                                  for index in range(8)] for camera in cameras}
    input_sizes = {camera["name"]: (1280, 960) for camera in cameras}
    config = avm_falcon.build_config(field, cameras, points_2d, input_sizes,
                                     "2026-01-01 00:00:00")

    check("falcon top-level keys",
          list(config) == ["comment", "name", "glb_file", "ibl_file",
                           "filamat_path", "vehicle_model", "mask_overlay",
                           "cameras", "bev_coord", "orbit_pitchs", "orbit_yaw",
                           "orbit_radius", "orbit_mouse_sens", "orbit_fov_y",
                           "orbit_near", "orbit_far", "steering_line"],
          str(list(config)))
    check("falcon assets", config["name"] == "filament_avm"
          and config["glb_file"].endswith(".glb")
          and config["vehicle_model"] == "vehicle")
    check("falcon mask overlay",
          config["mask_overlay"] == {
              "refer": [1.4, 3.2, 15.0], "padding": 0.2, "scale": 20},
          str(config["mask_overlay"]))
    check("falcon bev coord uses the scene width",
          config["bev_coord"] == [-4.8, 4.8, -4.8, 4.8], str(config["bev_coord"]))

    streams = config["cameras"]
    check("falcon has four cameras",
          [stream["name"] for stream in streams] == ["front", "back", "left", "right"])
    front = streams[0]
    check("falcon stream keys",
          list(front) == ["name", "points_2d", "points_3d", "K", "D", "input_size",
                          "model_scene", "bev_bound", "orbit", "enable", "ba_opt"],
          str(list(front)))
    check("falcon stream payload",
          len(front["points_2d"]) == 8 and len(front["points_3d"]) == 8
          and front["input_size"] == [1280, 960] and front["enable"] is True
          and front["model_scene"] == "NurbsPath.001"
          and front["bev_bound"] == [-15.0, 15.0, 0.0, 15.0]
          and front["ba_opt"] is False)
    check("falcon points_3d match the contract",
          front["points_3d"] == avm_layout.points("front", field))
    fx, fy, cx, cy = cameras[0]["K"]
    check("falcon K is the 3x3 matrix",
          front["K"] == [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    check("falcon side cameras ba_opt",
          streams[2]["ba_opt"] is True and streams[3]["ba_opt"] is True)
    check("falcon camera without points is disabled",
          avm_falcon.build_config(field, cameras, {}, input_sizes,
                                  "x")["cameras"][0]["enable"] is False)
    check("falcon steering line",
          config["steering_line"]["wheel_base"] == 3.2
          and config["steering_line"]["segments"] == 64
          and config["steering_line"]["back_prewarp"] is True)

    text = avm_falcon.dumps(config)
    check("falcon json parses back", json.loads(text) == config)
    check("falcon json keeps scalar arrays on one line",
          not any(re.fullmatch(r"\s*-?\d+(?:\.\d+)?,?", line)
                  for line in text.splitlines()),
          str([line.strip() for line in text.splitlines()
               if re.fullmatch(r"\s*-?\d+(?:\.\d+)?,?", line)][:3]))
    check("falcon json wraps the outer points_2d",
          '"points_2d": [' in text and "[\n" in text)


def test_drive_path():
    """The Drive Scene motion plan: speed profiles, sampling and the CSV."""
    check("forward is +Y at yaw 0", drive_path.forward(0.0) == (0.0, 1.0))
    check("yaw 90 turns the nose to -X",
          approx(drive_path.forward(90.0)[0], -1.0, 1e-12)
          and approx(drive_path.forward(90.0)[1], 0.0, 1e-12))

    # constant speed: one leg, the drive lasts distance / speed
    legs = drive_path.phases(20.0, 5.0, 1.0, profile=drive_path.CONSTANT)
    check("constant speed is a single leg",
          len(legs) == 1 and approx(legs[0].duration, 4.0, 1e-12), str(legs))
    check("sample at 0 and at the end",
          drive_path.sample(legs, 0.0) == (0.0, 5.0)
          and approx(drive_path.sample(legs, 4.0)[0], 20.0, 1e-9),
          str(drive_path.sample(legs, 4.0)))
    check("sample clamps past the end",
          approx(drive_path.sample(legs, 9.0)[0], 20.0, 1e-9))

    # trapezoid: ramp up, cruise, ramp down - starting and ending at rest
    legs = drive_path.phases(20.0, 5.0, 2.5, profile=drive_path.TRAPEZOID)
    check("trapezoid ramps in, cruises and brakes",
          [approx(leg.accel, a, 1e-12) for leg, a in zip(legs, (2.5, 0.0, -2.5))] == [True] * 3,
          str(legs))
    ramp = 5.0 / 2.5
    check("ramps take speed / accel seconds",
          approx(legs[0].duration, ramp, 1e-12)
          and approx(legs[-1].duration, ramp, 1e-12), str(legs))
    total = sum(leg.distance for leg in legs)
    check("the legs cover exactly the requested distance", approx(total, 20.0, 1e-9),
          f"{total:.6f}")
    check("no leg overshoots the cruise speed",
          max(leg.end_speed for leg in legs) <= 5.0 + 1e-12)

    # a drive too short for both ramps: the peak drops, the car still stops on it
    legs = drive_path.phases(2.0, 10.0, 2.0, profile=drive_path.TRAPEZOID)
    check("a short drive becomes a triangular profile",
          len(legs) == 2 and approx(max(leg.end_speed for leg in legs),
                                    math.sqrt(2.0 * 2.0), 1e-9),
          str(legs))
    check("the triangular profile covers the distance too",
          approx(sum(leg.distance for leg in legs), 2.0, 1e-9),
          f"{sum(leg.distance for leg in legs):.6f}")

    # the sampled plan: frames, pose, and speed that really is ds/dt
    drive = drive_path.plan(20.0, 5.0, accel=2.5, fps=10.0, heading=90.0,
                            start=(1.0, -2.0))
    check("frame count covers the duration", len(drive.frames) == int(round(drive.duration * 10.0)) + 1,
          f"{len(drive.frames)} frames, {drive.duration:.3f} s")
    check("the first frame is the start pose",
          drive.frames[0].distance == 0.0 and drive.frames[0].speed == 0.0
          and (drive.frames[0].x, drive.frames[0].y) == (1.0, -2.0), str(drive.frames[0]))
    check("the last frame stands exactly on the distance",
          approx(drive.frames[-1].distance, 20.0, 1e-9), str(drive.frames[-1]))
    check("yaw 90 drives towards -X",
          approx(drive.frames[-1].x, 1.0 - 20.0, 1e-6)
          and approx(drive.frames[-1].y, -2.0, 1e-9),
          f"({drive.frames[-1].x:.4f}, {drive.frames[-1].y:.4f})")
    check("distance is monotonic",
          all(b.distance >= a.distance
              for a, b in zip(drive.frames, drive.frames[1:])))
    check("time is monotonic",
          all(b.time >= a.time for a, b in zip(drive.frames, drive.frames[1:])))
    step = 1.0 / drive.fps
    worst = max(abs((b.distance - a.distance) / step - (a.speed + b.speed) * 0.5)
                for a, b in zip(drive.frames, drive.frames[1:]))
    check("the sampled speed is ds/dt (mean over the step)", worst < 2e-3, f"{worst:.2e}")

    # a duration that is not a whole number of frames ends on the exact duration
    odd = drive_path.plan(10.0, 3.0, profile=drive_path.CONSTANT, fps=4.0)
    check("the clip ends on the exact duration",
          approx(odd.frames[-1].time, odd.duration, 1e-12)
          and approx(odd.duration, 10.0 / 3.0, 1e-12),
          f"{odd.frames[-1].time:.6f} vs {odd.duration:.6f}")

    # the CSV is the contract with the algorithm side: one group of six columns
    # per recorded camera, named after it, in the order the caller lists them
    mount = drive_path.Mount(location=(-0.03, 2.47, 2.69),
                             rotation_deg=(20.0, -1.0, 2.0))
    cameras = ["front", "back", "left", "right"]
    text = drive_path.csv_text(drive, {name: mount for name in cameras})
    rows = text.rstrip("\n").split("\n")
    check("csv header", rows[0] == ",".join(drive_path.csv_header(cameras)), rows[0])
    check("csv has one row per frame", len(rows) == len(drive.frames) + 1,
          f"{len(rows)} rows for {len(drive.frames)} frames")
    check("every csv row carries one group of six per camera",
          all(len(row.split(",")) == 7 + 6 * len(cameras) for row in rows[1:]),
          f"{len(rows[1].split(','))} columns")
    third = rows[3].split(",")
    check("csv rows carry the frame's truth",
          int(third[0]) == drive.frames[2].index
          and approx(float(third[1]), drive.frames[2].time, 1e-6)
          and approx(float(third[3]), drive.frames[2].speed, 1e-6)
          and approx(float(third[6]), drive.frames[2].yaw, 1e-4), rows[3])
    for index, camera in enumerate(cameras):
        start = 7 + 6 * index
        expected = drive_path.camera_world_pose(drive.frames[2], mount)
        check(f"csv rows carry the {camera} camera world pose",
              all(approx(float(third[start + i]), expected[i], 1e-4) for i in range(6)),
              rows[3])

    # distinct mounts must land in their own column group - a shared pose would
    # hide a "the groups all came from one camera" mistake
    mounts = {name: drive_path.Mount(location=(0.1 * step, 0.2 * step, 2.0 + step))
              for step, name in enumerate(cameras)}
    mixed = drive_path.csv_text(drive, mounts).rstrip("\n").split("\n")
    for index, camera in enumerate(cameras):
        start = 7 + 6 * index
        expected = drive_path.camera_world_pose(drive.frames[2], mounts[camera])
        check(f"the {camera} group follows its own mount",
              all(approx(float(mixed[3].split(",")[start + i]), expected[i], 1e-4)
                  for i in range(6)),
              mixed[3].split(",")[start:start + 6])
    check("the column count follows the recorded set, not a fixed four",
          len(drive_path.csv_header(["front", "back"])) == 7 + 6 * 2
          and len(drive_path.csv_header(["left"])) == 7 + 6 * 1)

    check("a zero-length drive is a single frame",
          len(drive_path.plan(0.0, 5.0, fps=10.0).frames) == 1)
    check("summary mentions the frame count and fps",
          f"{len(drive.frames)} frames" in drive_path.summary(drive),
          drive_path.summary(drive))


def test_avm_cameras():
    """The four rig roles: one definition, and the clip contract's spelling."""
    check("the four roles are front/back/left/right",
          avm_cameras.CAMERAS == ("front", "back", "left", "right"),
          str(avm_cameras.CAMERAS))
    check("avm_layout re-exports the same tuple",
          avm_layout.CAMERAS is avm_cameras.CAMERAS,
          f"{avm_layout.CAMERAS} vs {avm_cameras.CAMERAS}")
    check("avm_layout's role constants come from the same source",
          (avm_layout.FRONT, avm_layout.BACK, avm_layout.LEFT, avm_layout.RIGHT)
          == avm_cameras.CAMERAS)
    check("object_name builds the AVM and Drive spellings",
          avm_cameras.object_name("AVM_Cam_", "front") == "AVM_Cam_Front"
          and avm_cameras.object_name("DRIVE_Cam_", "right") == "DRIVE_Cam_Right",
          avm_cameras.object_name("DRIVE_Cam_", "right"))
    check("enum_items covers every role in order",
          [item[0] for item in avm_cameras.enum_items()] == list(avm_cameras.CAMERAS),
          str(avm_cameras.enum_items()))
    check("every enum item has a label and a description",
          all(len(item) == 3 and item[1] and item[2]
              for item in avm_cameras.enum_items()),
          str(avm_cameras.enum_items()))

    # the key is what a clip's file names use; the renderer normalises a camera
    # name to it with FrameSource::NormalizeCameraKey() on the consuming side
    check("key_of strips the rig prefix and lowercases",
          avm_cameras.key_of("DRIVE_Cam_Front") == "front"
          and avm_cameras.key_of("AVM_Cam_Left") == "left"
          and avm_cameras.key_of("Back") == "back",
          avm_cameras.key_of("AVM_Cam_Left"))
    check("key_of is idempotent on a bare key",
          all(avm_cameras.key_of(key) == key for key in avm_cameras.CAMERAS))

    preset = avm_layout.load_preset()
    records = avm_layout.cameras_from_preset(preset)
    mounts = avm_cameras.mounts_of(records)
    check("a preset maps to one mount per role, in preset order",
          list(mounts) == list(avm_cameras.CAMERAS) and len(mounts) == 4,
          str(list(mounts)))
    front = next(r for r in records if r["name"] == "front")
    check("mount_of reads location and rotation_deg off the record",
          mounts["front"] == drive_path.Mount(
              location=tuple(front["location"]),
              rotation_deg=tuple(front["rotation"])),
          str(mounts["front"]))
    check("mounts_of skips a name that is not a rig role",
          avm_cameras.mounts_of([{"name": "top", "location": [1, 2, 3],
                                  "rotation": [0, 0, 0]}]) == {})


def test_vehicle():
    """The ego vehicle's geometry: one source, and the export's shape."""
    check("the body box is the minibus the presets calibrate",
          (vehicle.BODY_LENGTH_M, vehicle.BODY_WIDTH_M, vehicle.BODY_HEIGHT_M)
          == (4.8, 2.4, 2.88),
          str((vehicle.BODY_LENGTH_M, vehicle.BODY_WIDTH_M, vehicle.BODY_HEIGHT_M)))
    check("the axles are the app's steering geometry",
          (vehicle.WHEEL_BASE_M, vehicle.REAR_TRACK_M, vehicle.REAR_CENTER_OFFSET_M)
          == (3.2, 1.8, 2.8),
          str((vehicle.WHEEL_BASE_M, vehicle.REAR_TRACK_M, vehicle.REAR_CENTER_OFFSET_M)))
    check("the rear axle is the mesh wheel position, not the steering anchor",
          vehicle.AXLE_FRACTION == 0.31
          and vehicle.center_to_rear_axle() == [0.0, -1.488, 0.0]
          and vehicle.center_to_rear_axle(5.5) == [0.0, -1.705, 0.0],
          str(vehicle.center_to_rear_axle()))
    check("the Falcon steering config reads the shared body width",
          avm_falcon.STEERING["body_width"] == vehicle.BODY_WIDTH_M
          and avm_falcon.STEERING["wheel_base"] == vehicle.WHEEL_BASE_M
          and avm_falcon.STEERING["rear_track"] == vehicle.REAR_TRACK_M
          and avm_falcon.STEERING["rear_center_offset"] == vehicle.REAR_CENTER_OFFSET_M,
          str({k: avm_falcon.STEERING[k] for k in
               ("body_width", "wheel_base", "rear_track", "rear_center_offset")}))

    defaults = vehicle.block()
    check("the clip block names its frame",
          defaults["frame"] == "vehicle" and defaults["frame"] == vehicle.FRAME,
          str(defaults["frame"]))
    check("the clip block carries the body box in metres",
          defaults["body"] == {"length_m": 4.8, "width_m": 2.4, "height_m": 2.88},
          str(defaults["body"]))
    check("the clip block carries the centre -> rear-axle transform",
          defaults["center_to_rear_axle_m"] == [0.0, -1.488, 0.0],
          str(defaults.get("center_to_rear_axle_m")))
    check("the clip block carries the clearance and the axles",
          defaults["ground_clearance_m"] == 0.0
          and defaults["axles"] == {"wheel_base_m": 3.2, "rear_track_m": 1.8,
                                    "rear_center_offset_m": 2.8},
          str(defaults["axles"]))
    overridden = vehicle.block(length=5.5, width=2.1, height=3.0, clearance=0.2)
    check("an override flows into the block",
          overridden["body"] == {"length_m": 5.5, "width_m": 2.1, "height_m": 3.0}
          and overridden["ground_clearance_m"] == 0.2
          and overridden["center_to_rear_axle_m"] == [0.0, -1.705, 0.0],
          str(overridden["body"]))
    check("the block has no per-camera dimension",
          set(defaults) == {"frame", "body", "ground_clearance_m",
                            "center_to_rear_axle_m", "axles"},
          str(sorted(defaults)))


def test_drive_lot():
    """The car-park layout: bays, dividers, columns, parked cars and labels."""
    length, aisle, depth, bay_width = 48.0, 6.0, 5.2, 2.5
    count = drive_lot.bay_count(length, bay_width)
    check("bay count follows the pitch", count == 19, str(count))

    dividers = drive_lot.divider_positions(length, bay_width)
    check("one divider more than bays", len(dividers) == count + 1, str(len(dividers)))
    check("the dividers span the whole slab",
          approx(dividers[0], -length / 2, 1e-9)
          and approx(dividers[-1], length / 2, 1e-9), f"{dividers[0]}..{dividers[-1]}")
    gaps = [b - a for a, b in zip(dividers, dividers[1:])]
    check("the dividers are evenly spaced", max(gaps) - min(gaps) < 1e-9, str(gaps[:3]))

    rows = drive_lot.bays(length, aisle, depth, bay_width)
    check("both rows are laid out", len(rows) == 2 * count, str(len(rows)))
    left = [bay for bay in rows if bay.side < 0]
    right = [bay for bay in rows if bay.side > 0]
    check("the rows are lettered A / B",
          left[0].label == "A01" and right[0].label == "B01"
          and left[-1].label == f"A{count:02d}" and right[-1].label == f"B{count:02d}",
          f"{left[0].label}..{left[-1].label} / {right[0].label}..{right[-1].label}")
    check("every bay sits in its row",
          all(approx(abs(bay.x), aisle / 2 + depth / 2, 1e-9) for bay in rows)
          and approx(left[0].y, -length / 2 + left[0].width / 2, 1e-9)
          and all(-length / 2 <= bay.y <= length / 2 for bay in rows),
          f"x={left[0].x}, y={left[0].y}")
    check("a bay's number hangs in the aisle in front of it",
          all(abs(x) < aisle / 2 and approx(y, bay.y, 1e-9)
              for bay in rows for x, y in [drive_lot.label_position(bay, aisle)]))

    columns = drive_lot.pillar_positions(length, aisle, 2)
    check("two columns per side", len(columns) == 4, str(columns))
    check("the columns stand in the bay rows, off the aisle",
          all(aisle / 2 < abs(x) < aisle / 2 + depth for x, y in columns), str(columns))

    parked = drive_lot.parked_cars(rows, columns, 4)
    check("four parked cars per row", len(parked) == 8, str(len(parked)))
    check("no car is parked into a column",
          all(all(abs(car.bay.y - y) >= drive_lot.PILLAR_CLEARANCE
                  for x, y in columns if (x > 0) == (car.bay.side > 0))
              for car in parked),
          str([(car.bay.label, car.bay.y) for car in parked]))
    check("every parked car fits its bay",
          all(car.length <= car.bay.depth and car.width <= car.bay.width for car in parked))
    check("parked cars stand nose-in, facing the wall",
          all(approx(car.yaw, -90.0 * car.bay.side, 1e-9) for car in parked))
    check("each parked car has its own bay",
          len({car.bay.label for car in parked}) == len(parked))
    check("the forms and paints vary",
          len({car.paint for car in parked}) > 1
          and len({car.length for car in parked}) > 1)
    check("the layout is deterministic",
          [(car.bay.label, car.length, car.paint) for car in parked]
          == [(car.bay.label, car.length, car.paint)
              for car in drive_lot.parked_cars(rows, columns, 4)])
    check("without columns every bay is free",
          len(drive_lot.parked_cars(rows, [], 4)) == 8)
    check("no parked cars when none are asked for",
          drive_lot.parked_cars(rows, columns, 0) == [])


def main():
    for test in (test_intrinsics, test_distortion_roundtrip, test_fisheye, test_projection,
                 test_transform, test_lens_conversion, test_bundled_preset,
                 test_filament_avm_config, test_avm_layout, test_avm_coverage,
                 test_avm_falcon_config, test_drive_path, test_drive_lot,
                 test_avm_cameras, test_vehicle, test_calibration_io):
        test()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} failure(s): {', '.join(FAILURES)}")
        return 1
    print("all core tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())