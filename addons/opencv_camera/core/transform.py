"""Conversions between OpenCV conventions and Blender conventions.

Pure Python (no ``bpy``), matrices are plain row-major sequences of floats.

* OpenCV camera frame: ``+X`` right, ``+Y`` down, ``+Z`` forward.
* Blender camera local frame: ``+X`` right, ``+Y`` up, ``+Z`` backward
  (a Blender camera looks down its local ``-Z``).

The two frames are related by ``M = diag(1, -1, -1)``, so a world-to-camera
transform ``(R_cv, t_cv)`` from ``cv2.solvePnP`` becomes::

    R_b = M @ R_cv
    t_b = M @ t_cv

The camera object matrix (camera-to-world) is then ``[R_bᵀ | -R_bᵀ t_b]``.
"""

from __future__ import annotations

from typing import Sequence, Tuple

__all__ = [
    "ROT_FLIP",
    "mat3_mul",
    "mat3_transpose",
    "mat3_vec",
    "mat4_from_rt",
    "rt_from_mat4",
    "world_to_camera_to_blender",
    "blender_to_world_to_camera",
    "camera_center",
    "shift_from_principal_point",
    "principal_point_from_shift",
    "lens_mm_from_fx",
    "fx_from_lens_mm",
]

#: ``diag(1, -1, -1)``, row-major.
ROT_FLIP = (1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)


def mat3_mul(a: Sequence[float], b: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 3x3 multiplication."""
    return tuple(
        a[3 * r + 0] * b[0 + c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c]
        for r in range(3)
        for c in range(3)
    )


def mat3_transpose(a: Sequence[float]) -> Tuple[float, ...]:
    return tuple(a[3 * c + r] for r in range(3) for c in range(3))


def mat3_vec(a: Sequence[float], v: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(a[3 * r + 0] * v[0] + a[3 * r + 1] * v[1] + a[3 * r + 2] * v[2]
                 for r in range(3))


def mat4_from_rt(rotation: Sequence[float], translation: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 4x4 from a row-major 3x3 rotation and a 3-vector translation."""
    r = tuple(rotation)
    t = tuple(translation)
    return (
        r[0], r[1], r[2], t[0],
        r[3], r[4], r[5], t[1],
        r[6], r[7], r[8], t[2],
        0.0, 0.0, 0.0, 1.0,
    )


def rt_from_mat4(matrix: Sequence[float]) -> Tuple[Tuple[float, ...], Tuple[float, float, float]]:
    """Row-major 4x4 -> (row-major 3x3 rotation, 3-vector translation)."""
    m = tuple(float(v) for v in matrix)
    if len(m) != 16:
        raise ValueError(f"expected a 16 element matrix, got {len(m)}")
    rotation = (m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10])
    translation = (m[3], m[7], m[11])
    return rotation, translation


def world_to_camera_to_blender(
    R_cv: Sequence[float],
    t_cv: Sequence[float],
) -> Tuple[Tuple[float, ...], Tuple[float, float, float]]:
    """OpenCV world-to-camera ``(R, t)`` -> Blender world-to-camera ``(R, t)``."""
    R_b = mat3_mul(ROT_FLIP, R_cv)
    t_b = mat3_vec(ROT_FLIP, t_cv)
    return R_b, t_b


def blender_to_world_to_camera(
    R_b: Sequence[float],
    t_b: Sequence[float],
) -> Tuple[Tuple[float, ...], Tuple[float, float, float]]:
    """Inverse of :func:`world_to_camera_to_blender` (``M`` is its own inverse)."""
    return world_to_camera_to_blender(R_b, t_b)


def camera_center(R_b: Sequence[float], t_b: Sequence[float]) -> Tuple[float, float, float]:
    """Camera position in world space: ``-R_bᵀ t_b``."""
    return mat3_vec(mat3_transpose(R_b), tuple(-v for v in t_b))


def object_matrix_from_opencv(
    R_cv: Sequence[float],
    t_cv: Sequence[float],
    world_matrix: Sequence[float] | None = None,
) -> Tuple[float, ...]:
    """OpenCV world-to-camera pose -> Blender camera object (camera-to-world) 4x4.

    ``world_matrix`` (row-major 4x4) optionally maps the calibration world frame
    to the Blender world frame, e.g. when the calibration uses a ROS (Z-up,
    X-forward) frame instead of Blender's Z-up, Y-forward world.
    """
    R_b, t_b = world_to_camera_to_blender(R_cv, t_cv)
    centre = camera_center(R_b, t_b)
    camera_to_world = mat4_from_rt(mat3_transpose(R_b), centre)
    if world_matrix is None:
        return camera_to_world
    return _mat4_mul(world_matrix, camera_to_world)


def _mat4_mul(a: Sequence[float], b: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 4x4 multiplication."""
    out = []
    for r in range(4):
        for c in range(4):
            out.append(sum(a[4 * r + k] * b[4 * k + c] for k in range(4)))
    return tuple(out)


def shift_from_principal_point(
    cx: float,
    cy: float,
    width: int,
    height: int,
) -> Tuple[float, float]:
    """OpenCV principal point -> Blender ``shift_x`` / ``shift_y``.

    Note the sign asymmetry: Blender's image coordinates are bottom-up, so a
    positive ``shift_y`` moves the principal point *down* in OpenCV v terms.

    Verified against renders of a target on the optical axis (128x128,
    lens 50mm, sensor 36mm): shift (0.1, 0.15) measured a principal point of
    (51.107, 83.230) px, i.e. cx = (0.5 - 0.1) * 128 and cy = (0.5 + 0.15) * 128.
    """
    return 0.5 - cx / float(width), cy / float(height) - 0.5


def principal_point_from_shift(
    shift_x: float,
    shift_y: float,
    width: int,
    height: int,
) -> Tuple[float, float]:
    """Blender ``shift_x`` / ``shift_y`` -> OpenCV principal point."""
    return (0.5 - shift_x) * float(width), (0.5 + shift_y) * float(height)


def lens_mm_from_fx(fx: float, sensor_width_mm: float, width_px: int) -> float:
    """Pixel focal length -> Blender lens (mm) for a horizontal sensor fit."""
    return fx * sensor_width_mm / float(width_px)


def fx_from_lens_mm(lens_mm: float, sensor_width_mm: float, width_px: int) -> float:
    """Blender lens (mm) -> pixel focal length for a horizontal sensor fit."""
    return lens_mm * float(width_px) / sensor_width_mm
