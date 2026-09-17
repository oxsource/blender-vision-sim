"""OpenCV camera model: intrinsics, distortion, forward and inverse projection.

Pure Python on purpose (no ``bpy`` import) so it can be unit tested outside
Blender and reused by exporters or validators.

Conventions
-----------
* Camera frame follows OpenCV: ``+X`` right, ``+Y`` down, ``+Z`` forward.
* Pixel coordinates: origin at the top-left corner, ``u`` right, ``v`` down,
  pixel centres at ``(i + 0.5, j + 0.5)``.
* Pinhole intrinsics ``K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``.
* Distortion is OpenCV ``plumb_bob`` (Brown-Conrady)::

      num = 1 + k1 r^2 + k2 r^4 + k3 r^6
      den = 1 + k4 r^2 + k5 r^4 + k6 r^6
      x_d = x num/den + 2 p1 x y + p2 (r^2 + 2 x^2)
      y_d = y num/den + p1 (r^2 + 2 y^2) + 2 p2 x y

  ``k4..k6`` equal to zero selects the classic 5-parameter model, non-zero
  values select the rational model. Coefficients always use OpenCV order
  ``(k1, k2, p1, p2, k3, k4, k5, k6)``.
* ``cx``/``cy`` may be negative, which means "image centre" (resolution
  independent); call :meth:`Intrinsics.resolved` before doing maths.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence, Tuple

__all__ = [
    "Intrinsics",
    "Distortion",
    "MODEL_BROWN_CONRADY",
    "MODEL_RATIONAL",
    "MODEL_FISHEYE",
    "distort",
    "undistort",
    "undistort_converged",
    "normalized_from_pixel",
    "pixel_from_normalized",
    "ray_from_pixel",
    "project_point",
]

MODEL_BROWN_CONRADY = "brown_conrady"
MODEL_RATIONAL = "rational"
MODEL_FISHEYE = "fisheye"


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics in pixels, plus the resolution they belong to."""

    fx: float
    fy: float
    cx: float = -1.0
    cy: float = -1.0
    width: int = 0
    height: int = 0

    # -- construction -----------------------------------------------------
    @classmethod
    def auto(cls, fx: float, fy: float, width: int, height: int) -> "Intrinsics":
        """Intrinsics with the principal point at the image centre."""
        return cls(fx=fx, fy=fy, cx=-1.0, cy=-1.0, width=int(width), height=int(height))

    # -- derived values ---------------------------------------------------
    @property
    def auto_center(self) -> bool:
        return self.cx < 0.0 or self.cy < 0.0

    def resolved(self) -> "Intrinsics":
        """Return a copy with ``cx``/``cy`` replaced by the image centre."""
        if not self.auto_center:
            return self
        return replace(
            self,
            cx=0.5 * self.width if self.cx < 0.0 else self.cx,
            cy=0.5 * self.height if self.cy < 0.0 else self.cy,
        )

    def scaled(self, width: int, height: int) -> "Intrinsics":
        """Rescale intrinsics to another resolution (same FOV, same pixels).

        An automatic principal point (``cx``/``cy`` < 0) stays automatic.
        """
        if self.width <= 0 or self.height <= 0:
            raise ValueError("cannot rescale intrinsics without a source resolution")
        res = self.resolved()
        sx = float(width) / float(self.width)
        sy = float(height) / float(self.height)
        return Intrinsics(
            fx=res.fx * sx,
            fy=res.fy * sy,
            cx=-1.0 if self.cx < 0.0 else res.cx * sx,
            cy=-1.0 if self.cy < 0.0 else res.cy * sy,
            width=int(width),
            height=int(height),
        )

    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(0.5 * self.width / self.fx))

    def vfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(0.5 * self.height / self.fy))


@dataclass(frozen=True)
class Distortion:
    """OpenCV distortion coefficients (order: k1, k2, p1, p2, k3, k4, k5, k6)."""

    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0
    k5: float = 0.0
    k6: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    model: str = MODEL_BROWN_CONRADY
    enabled: bool = True

    # -- construction -----------------------------------------------------
    @classmethod
    def from_coefficients(
        cls,
        coefficients: Sequence[float],
        model: str = MODEL_BROWN_CONRADY,
        enabled: bool = True,
    ) -> "Distortion":
        """Build from OpenCV-ordered coefficients (4, 5, 8 or 12 values)."""
        values = [float(c) for c in coefficients]
        if len(values) not in (4, 5, 8, 12):
            raise ValueError(
                "expected 4 (k1,k2,p1,p2), 5, 8 or 12 distortion coefficients, "
                f"got {len(values)}"
            )
        if model == MODEL_FISHEYE:
            # fisheye uses (k1, k2, k3, k4) of the theta polynomial
            values = values + [0.0] * (4 - len(values))
            return cls(k1=values[0], k2=values[1], k3=values[2], k4=values[3],
                       model=model, enabled=enabled)
        padded = values + [0.0] * (8 - len(values))
        k1, k2, p1, p2, k3, k4, k5, k6 = padded[:8]
        if model == MODEL_BROWN_CONRADY and (k4 or k5 or k6):
            model = MODEL_RATIONAL
        return cls(k1=k1, k2=k2, k3=k3, k4=k4, k5=k5, k6=k6,
                   p1=p1, p2=p2, model=model, enabled=enabled)

    # -- derived values ---------------------------------------------------
    @property
    def has_rational(self) -> bool:
        return bool(self.k4 or self.k5 or self.k6)

    @property
    def is_identity(self) -> bool:
        return not any((self.k1, self.k2, self.k3, self.k4, self.k5, self.k6,
                        self.p1, self.p2))

    def coefficients(self, count: int = 5) -> list:
        """OpenCV-ordered coefficients (``count`` = 4, 5 or 8)."""
        if self.model == MODEL_FISHEYE:
            return [self.k1, self.k2, self.k3, self.k4][:count if count <= 4 else 4]
        order = [self.k1, self.k2, self.p1, self.p2, self.k3, self.k4, self.k5, self.k6]
        if count == 4:
            return [self.k1, self.k2, self.p1, self.p2]
        return order[:count]


def _check_model(dist: Distortion) -> None:
    if dist.model == MODEL_FISHEYE:
        raise NotImplementedError(
            "the OpenCV fisheye (equidistant) model is not implemented yet; "
            "see docs/roadmap.md (M4)"
        )


def _radial_terms(r2: float, dist: Distortion) -> Tuple[float, float]:
    r4 = r2 * r2
    r6 = r4 * r2
    num = 1.0 + dist.k1 * r2 + dist.k2 * r4 + dist.k3 * r6
    den = 1.0 + dist.k4 * r2 + dist.k5 * r4 + dist.k6 * r6
    return num, den


def distort(x: float, y: float, dist: Distortion) -> Tuple[float, float]:
    """Undistorted normalised coords -> distorted normalised coords."""
    if not dist.enabled or dist.is_identity:
        return x, y
    _check_model(dist)
    r2 = x * x + y * y
    num, den = _radial_terms(r2, dist)
    radial = num / den if den else 0.0
    xd = x * radial + 2.0 * dist.p1 * x * y + dist.p2 * (r2 + 2.0 * x * x)
    yd = y * radial + dist.p1 * (r2 + 2.0 * y * y) + 2.0 * dist.p2 * x * y
    return xd, yd


def undistort(
    xd: float,
    yd: float,
    dist: Distortion,
    iterations: int = 10,
) -> Tuple[float, float]:
    """Distorted normalised coords -> undistorted coords (fixed point iteration).

    Same iteration as ``cv2.undistortPoints``. Returns the best estimate even
    when it does not converge; use :func:`undistort_converged` to know.
    """
    x, y = undistort_converged(xd, yd, dist, iterations)[:2]
    return x, y


def undistort_converged(
    xd: float,
    yd: float,
    dist: Distortion,
    iterations: int = 10,
) -> Tuple[float, float, bool]:
    """Like :func:`undistort` but also reports whether the iteration converged."""
    if not dist.enabled or dist.is_identity:
        return xd, yd, True
    _check_model(dist)
    x, y = xd, yd
    converged = False
    for _ in range(max(1, int(iterations))):
        r2 = x * x + y * y
        num, den = _radial_terms(r2, dist)
        if abs(den) < 1e-12:
            break
        icdist = den / num if num else 0.0
        dx = 2.0 * dist.p1 * x * y + dist.p2 * (r2 + 2.0 * x * x)
        dy = dist.p1 * (r2 + 2.0 * y * y) + 2.0 * dist.p2 * x * y
        nx = (xd - dx) * icdist
        ny = (yd - dy) * icdist
        converged = abs(nx - x) < 1e-9 and abs(ny - y) < 1e-9
        x, y = nx, ny
        if abs(x) > 1e3 or abs(y) > 1e3:  # diverging
            return x, y, False
        if converged:
            return x, y, True
    return x, y, converged


def normalized_from_pixel(u: float, v: float, intr: Intrinsics) -> Tuple[float, float]:
    """Pixel -> distorted normalised image coordinates."""
    res = intr.resolved()
    return (u - res.cx) / res.fx, (v - res.cy) / res.fy


def pixel_from_normalized(xd: float, yd: float, intr: Intrinsics) -> Tuple[float, float]:
    """Distorted normalised image coordinates -> pixel."""
    res = intr.resolved()
    return res.fx * xd + res.cx, res.fy * yd + res.cy


def ray_from_pixel(
    u: float,
    v: float,
    intr: Intrinsics,
    dist: Distortion = Distortion(),
    iterations: int = 10,
) -> Tuple[float, float, float]:
    """Pixel -> ray direction in the OpenCV camera frame (``+Z`` forward).

    This is the inverse mapping the OSL camera shader implements.
    """
    xd, yd = normalized_from_pixel(u, v, intr)
    x, y = undistort(xd, yd, dist, iterations)
    return x, y, 1.0


def project_point(
    X: float,
    Y: float,
    Z: float,
    intr: Intrinsics,
    dist: Distortion = Distortion(),
) -> Tuple[float, float, float]:
    """Point in the OpenCV camera frame -> pixel (``u``, ``v``, depth)."""
    if Z == 0.0:
        raise ValueError("point projects to infinity (Z == 0)")
    x, y = X / Z, Y / Z
    xd, yd = distort(x, y, dist)
    u, v = pixel_from_normalized(xd, yd, intr)
    return u, v, Z