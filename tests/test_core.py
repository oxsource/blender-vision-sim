#!/usr/bin/env python3
"""Unit tests for the add-on core (no ``bpy``, runs with any python3).

    python3 tests/test_core.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons", "opencv_camera"))

from core import calibration_io, camera_model, transform  # noqa: E402

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
    path = os.path.join(ROOT, "..", "..", "..", "codes", "filament_avm", "configs",
                        "vehicle_avm_minibus.json")
    path = os.path.normpath(path)
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


def main():
    for test in (test_intrinsics, test_distortion_roundtrip, test_fisheye, test_projection,
                 test_transform, test_lens_conversion, test_bundled_preset,
                 test_filament_avm_config, test_calibration_io):
        test()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} failure(s): {', '.join(FAILURES)}")
        return 1
    print("all core tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())