#!/usr/bin/env python3
"""Derive the AVM Scene default preset from a ``filament_avm`` vehicle config.

**Offline, one-off development tool** (needs ``numpy`` + ``cv2``): the AVM Scene
never solves poses at runtime - it only consumes the ``location`` / ``rotation``
/ ``K`` / ``D`` values this script bakes into ``presets/avm_scene/default.json``.

It reproduces the ``filament_avm`` production pipeline exactly
(``falcon/core/camera/camera_pose.cc`` + ``config.cc`` + ``ba_optimization.cc``):

1. undistort the 2D corner detections with the fisheye model (``P = K``);
2. per-camera ``cy`` refinement when ``ba_opt`` is set, via a single-parameter
   Levenberg-Marquardt loop (mirrors ``OptimizeCameraCy``);
3. zero-distortion ``cv2.solvePnP`` -> ``R, t`` with the world frame being the
   Blender vehicle frame (X right, Y forward, Z up);
4. convert to the Blender camera object matrix and take ``location`` + XYZ euler.

The **render** intrinsics stay the original ``K``: ``camera_pose.cc`` copies
``value.K`` *before* the optimisation mutates ``K`` in place, so the ``cy``
offset only ever affects the PnP.

Usage
-----
    python3 scripts/solve_avm_defaults.py --config <vehicle_avm_*.json>
    python3 scripts/solve_avm_defaults.py --config ... --core-w 240 --core-h 480
    python3 scripts/solve_avm_defaults.py --config ... --check    # diff vs bundled preset
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons", "opencv_camera"))

import numpy as np  # noqa: E402
import cv2  # noqa: E402

from core import paths, transform  # noqa: E402

#: the vehicle frame the config's points_3d live in is the Blender world frame
#: (X right, Y forward, Z up) - verified against camera_pose.cc: SolveOrbit
SCENE_ID = "avm_scene"
PRESET_NAME = "default"


# ---------------------------------------------------------------------------
# 1..3  the pose solve (a faithful port of the C++ path)
# ---------------------------------------------------------------------------
def undistort_points(points_2d: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """``CameraConfig::ImagePoints2D()``: fisheye undistort with ``P = K``."""
    pts = np.asarray(points_2d, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.fisheye.undistortPoints(pts, K, D, None, K).reshape(-1, 2)


def _residual(points_2d_und: np.ndarray, points_3d: np.ndarray,
              K: np.ndarray, cy: float) -> np.ndarray:
    K_cy = K.copy()
    K_cy[1, 2] += cy
    ok, rvec, tvec = cv2.solvePnP(points_3d, points_2d_und, K_cy, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    projected, _ = cv2.projectPoints(points_3d, rvec, tvec, K_cy, None)
    return (projected.reshape(-1, 2) - points_2d_und).ravel()


def optimize_cy(points_2d_und: np.ndarray, points_3d: np.ndarray,
                K: np.ndarray, max_iter: int = 100) -> Tuple[float, np.ndarray]:
    """Single-parameter LM over ``cy`` (mirrors ``ba_optimization.cc``)."""
    cy = 0.0
    lam, h = 1e-3, 1e-6
    residual = _residual(points_2d_und, points_3d, K, cy)
    cost = float(np.linalg.norm(residual))
    jacobian = (_residual(points_2d_und, points_3d, K, cy + h) - residual) / h
    jtj = float(jacobian @ jacobian)
    jtr = float(jacobian @ residual)
    for _ in range(max_iter):
        step = -jtr / (jtj + lam) if (jtj + lam) else 0.0
        trial = cy + step
        residual_new = _residual(points_2d_und, points_3d, K, trial)
        cost_new = float(np.linalg.norm(residual_new))
        if cost_new < cost:
            cy, residual, cost = trial, residual_new, cost_new
            lam *= 0.8
            if abs(step) < 1e-6:
                break
            jacobian = (_residual(points_2d_und, points_3d, K, cy + h) - residual) / h
            jtj = float(jacobian @ jacobian)
            jtr = float(jacobian @ residual)
        else:
            lam *= 10.0
            if lam > 1e9:
                break
    return cy, K


def euler_xyz_degrees(matrix: Sequence[float]) -> Tuple[float, float, float]:
    """XYZ euler (Blender convention, ``R = Rz @ Ry @ Rx``) of a row-major 4x4."""
    r00, r10, r20 = matrix[0], matrix[4], matrix[8]
    r21, r22 = matrix[9], matrix[10]
    sy = math.hypot(r00, r10)
    if sy > 1e-9:
        x = math.atan2(r21, r22)
        y = math.atan2(-r20, sy)
        z = math.atan2(r10, r00)
    else:  # gimbal lock
        x = math.atan2(-matrix[6], matrix[5])
        y = math.atan2(-r20, sy)
        z = 0.0
    return tuple(math.degrees(v) for v in (x, y, z))


def solve_camera(camera: Dict) -> Dict:
    """One camera config -> ``{name, K, D, location, rotation, ba_opt, cy}``."""
    K = np.asarray(camera["K"], dtype=np.float64)
    D = np.asarray(camera["D"], dtype=np.float64)
    points_3d = np.asarray(camera["points_3d"], dtype=np.float64)
    points_2d_und = undistort_points(camera["points_2d"], K, D)

    ba_opt = bool(camera.get("ba_opt", False))
    cy = 0.0
    if ba_opt:
        cy, _ = optimize_cy(points_2d_und, points_3d, K)

    K_pnp = K.copy()
    K_pnp[1, 2] += cy
    ok, rvec, tvec = cv2.solvePnP(points_3d, points_2d_und, K_pnp, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError(f"{camera['name']}: solvePnP failed")
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    translation = tvec.reshape(3)

    object_matrix = transform.object_matrix_from_opencv(
        rotation_matrix.reshape(9), tuple(translation)
    )
    location = (object_matrix[3], object_matrix[7], object_matrix[11])
    euler = euler_xyz_degrees(object_matrix)

    input_size = camera.get("input_size") or [1280, 960]
    return {
        "name": camera["name"],
        "enable": bool(camera.get("enable", True)),
        "location": [round(v, 6) for v in location],
        "rotation": [round(v, 4) for v in euler],
        "K": [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],
        "D": [float(v) for v in D],
        "output": [int(input_size[0]), int(input_size[1])],
        "ba_opt": ba_opt,
        "cy_offset": round(float(cy), 6),
        "camera_center": [round(v, 6) for v in (-rotation_matrix.T @ translation)],
    }


# ---------------------------------------------------------------------------
# 4  field dimensions from the 3D corner points
# ---------------------------------------------------------------------------
def field_from_points(points_3d: List[List[float]], core_w: float, core_h: float,
                      border_w: float = 0.0, border_h: float = 0.0) -> Dict:
    """Invert ``PlaneScenePerfs.Model.points()`` for the field parameters.

    ``points_3d`` only fix ``corner`` and ``core/2 + inner`` (``cIn``), so the
    ``core`` split has to come from outside (the vehicle size).  All values are
    returned in **centimetres** (the panel / Store unit); ``core_*`` and
    ``border_*`` are taken in cm as well.
    """
    xs = sorted({round(abs(p[0]), 6) for p in points_3d})
    ys = sorted({round(abs(p[1]), 6) for p in points_3d})
    if len(xs) != 2 or len(ys) != 2:
        raise ValueError(f"expected 2 distinct |x| and |y| block edges, got {xs} / {ys}")
    c_in_x, c_out_x = xs
    c_in_y, c_out_y = ys
    corner_m = 0.5 * ((c_out_x - c_in_x) + (c_out_y - c_in_y))
    inner_w_m = c_in_x - core_w / 200.0
    inner_h_m = c_in_y - core_h / 200.0
    if inner_w_m < -1e-9 or inner_h_m < -1e-9:
        raise ValueError("core is larger than the block inner edge (negative inner)")
    return {
        "core_w": float(core_w),
        "core_h": float(core_h),
        "corner": round(corner_m * 100.0, 6),
        "inner_w": round(max(0.0, inner_w_m) * 100.0, 6),
        "inner_h": round(max(0.0, inner_h_m) * 100.0, 6),
        "border_w": float(border_w),
        "border_h": float(border_h),
    }


# ---------------------------------------------------------------------------
# preset assembly
# ---------------------------------------------------------------------------
def build_preset(config_path: str, core_w: float, core_h: float,
                 border_w: float = 0.0, border_h: float = 0.0,
                 name: str = "minibus") -> Dict:
    with open(config_path, "r", encoding="utf-8") as handle:
        config = json.load(handle)
    cameras = [solve_camera(camera) for camera in config["cameras"]]
    all_points = [p for camera in config["cameras"] for p in camera["points_3d"]]
    field = field_from_points(all_points, core_w, core_h, border_w, border_h)
    return {
        "format": "avm_scene_preset",
        "version": 1,
        "name": name,
        "field": field,
        "cameras": cameras,
        "meta": {
            "source": os.path.basename(config_path),
            "solver": "scripts/solve_avm_defaults.py",
            "units": {"length": "m", "angle": "deg", "field": "cm"},
        },
    }


def _preset_path() -> str:
    return paths.scene_preset_file(SCENE_ID, PRESET_NAME)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, help="vehicle_avm_*.json")
    parser.add_argument("--core-w", type=float, default=240.0, help="car width [cm]")
    parser.add_argument("--core-h", type=float, default=480.0, help="car length [cm]")
    parser.add_argument("--border-w", type=float, default=0.0, help="field border [cm]")
    parser.add_argument("--border-h", type=float, default=0.0, help="field border [cm]")
    parser.add_argument("--name", default="minibus")
    parser.add_argument("--out", default="", help="output path (default: bundled preset)")
    parser.add_argument("--check", action="store_true",
                        help="compare with the bundled preset instead of writing")
    args = parser.parse_args()

    preset = build_preset(args.config, args.core_w, args.core_h,
                          args.border_w, args.border_h, args.name)
    text = json.dumps(preset, indent=2) + "\n"

    if args.check:
        with open(_preset_path(), "r", encoding="utf-8") as handle:
            bundled = handle.read()
        if bundled.strip() != text.strip():
            print(f"preset differs from {_preset_path()}", file=sys.stderr)
            return 1
        print(f"preset matches {_preset_path()}")
        return 0

    out = args.out or _preset_path()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as handle:
        handle.write(text)
    print(f"written {out}")
    for camera in preset["cameras"]:
        center = ", ".join(f"{v:+.4f}" for v in camera["camera_center"])
        rotation = ", ".join(f"{v:+.4f}" for v in camera["rotation"])
        print(f"  {camera['name']:5s} C=[{center}] rot=[{rotation}] "
              f"cy_offset={camera['cy_offset']:+.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
