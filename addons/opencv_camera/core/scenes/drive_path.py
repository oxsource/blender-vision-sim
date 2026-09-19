"""Drive Scene motion plan: drive a straight line with a speed profile.

Pure Python (no ``bpy``): it turns "distance + speed profile + fps" into the
per-frame truth - time, travelled distance, speed and world pose - that the
Blender side keyframes the vehicle with *and* writes into ``frames.csv``.  Both
sides read the same plan, so a rendered frame and its CSV row cannot disagree.

Conventions follow the rest of the add-on: metres, seconds, Z up, and the
vehicle's nose is ``+Y`` at ``yaw = 0`` - so its forward is
``(-sin(yaw), cos(yaw))``, exactly what Blender's Z Euler does to ``+Y``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

#: speed profiles
CONSTANT = "constant"    #: hold one speed for the whole drive
TRAPEZOID = "trapezoid"  #: accelerate to the cruise speed, cruise, brake to a stop
PROFILES = (CONSTANT, TRAPEZOID)

#: the per-frame truth, one row per rendered frame.  The ``cam_*`` columns are the
#: camera's **world** pose at that frame (vehicle pose composed with the fixed
#: mount pose), so the algorithm side does not have to compose it itself.
CSV_HEADER = ("frame", "time_s", "distance_m", "speed_mps", "x_m", "y_m", "yaw_deg",
              "cam_x_m", "cam_y_m", "cam_z_m", "cam_roll_deg", "cam_pitch_deg",
              "cam_yaw_deg")

#: the speed profile can never need a smaller acceleration than this
MIN_ACCEL = 0.05


@dataclass(frozen=True)
class Phase:
    """One constant-acceleration leg of the speed profile."""

    duration: float
    speed: float   #: speed at the start of the leg [m/s]
    accel: float   #: [m/s^2]

    @property
    def end_speed(self) -> float:
        return self.speed + self.accel * self.duration

    @property
    def distance(self) -> float:
        return self.speed * self.duration + 0.5 * self.accel * self.duration ** 2


@dataclass(frozen=True)
class Frame:
    """One frame's truth."""

    index: int
    time: float
    distance: float
    speed: float
    x: float
    y: float
    yaw: float  #: degrees, the vehicle empty's Z Euler


@dataclass(frozen=True)
class Mount:
    """A camera's fixed pose in the vehicle frame (metres, XYZ euler degrees)."""

    location: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def camera_world_pose(frame: Frame, mount: Mount) -> Tuple[float, ...]:
    """The camera's world pose at ``frame``: ``(x, y, z, roll, pitch, yaw)``.

    Metres and degrees, same convention as the vehicle columns.  The vehicle only
    yaws about Z and Blender's XYZ euler composes on the left as ``Rz``, so the
    composed rotation is exactly ``(rx, ry, rz + yaw)`` and the position is the
    vehicle position plus the Z-rotated mount offset.
    """
    angle = math.radians(frame.yaw)
    cos, sin = math.cos(angle), math.sin(angle)
    mx, my, mz = mount.location
    rx, ry, rz = mount.rotation_deg
    return (frame.x + cos * mx - sin * my,
            frame.y + sin * mx + cos * my,
            mz,
            rx, ry, rz + frame.yaw)


@dataclass(frozen=True)
class Plan:
    """A sampled drive plus the parameters that produced it."""

    frames: List[Frame]
    profile: str
    fps: float
    duration: float
    distance: float
    cruise_speed: float
    accel: float
    heading: float
    start: Tuple[float, float]

    @property
    def peak_speed(self) -> float:
        return max((frame.speed for frame in self.frames), default=0.0)


def forward(yaw_deg: float) -> Tuple[float, float]:
    """The unit forward vector of a vehicle whose nose is ``+Y`` at yaw 0."""
    angle = math.radians(yaw_deg)
    return (-math.sin(angle), math.cos(angle))


def phases(distance: float, cruise: float, accel: float,
           profile: str = TRAPEZOID,
           start_speed: float = 0.0, end_speed: float = 0.0) -> List[Phase]:
    """The constant-acceleration legs whose integral covers ``distance``.

    ``trapezoid`` ramps from ``start_speed`` up to the cruise speed, holds it and
    brakes to ``end_speed``; when the drive is too short for both ramps the peak
    drops to whatever fits (a triangular profile), so the car still stops exactly
    on the requested distance instead of overshooting it.
    """
    distance = max(0.0, float(distance))
    if distance <= 0.0:
        return []
    if profile == CONSTANT:
        if float(cruise) <= 0.0:
            raise ValueError("the cruise speed must be positive")
        return [Phase(distance / float(cruise), float(cruise), 0.0)]

    v0, v1 = max(0.0, float(start_speed)), max(0.0, float(end_speed))
    a = max(MIN_ACCEL, float(accel))
    peak = min(float(cruise), math.sqrt(max(0.0, 2.0 * a * distance + v0 ** 2 + v1 ** 2) / 2.0))
    if peak < max(v0, v1):  # cannot be reached without reversing; hold the ends
        peak = max(v0, v1)
    ramp_in = (peak ** 2 - v0 ** 2) / (2.0 * a) if peak > v0 else 0.0
    ramp_out = (peak ** 2 - v1 ** 2) / (2.0 * a) if peak > v1 else 0.0
    cruise_distance = max(0.0, distance - ramp_in - ramp_out)

    legs: List[Phase] = []
    if ramp_in > 1e-12:
        legs.append(Phase((peak - v0) / a, v0, a))
    if cruise_distance > 1e-12 and peak > 0.0:
        legs.append(Phase(cruise_distance / peak, peak, 0.0))
    if ramp_out > 1e-12:
        legs.append(Phase((peak - v1) / a, peak, -a))
    return legs


def sample(legs: Sequence[Phase], time: float) -> Tuple[float, float]:
    """``(travelled, speed)`` at ``time``; clamped to the profile's ends."""
    remaining = max(0.0, float(time))
    travelled = 0.0
    for leg in legs:
        step = min(remaining, leg.duration)
        if step > 0.0:
            travelled += leg.speed * step + 0.5 * leg.accel * step * step
            remaining -= step
        if remaining <= 0.0:
            return travelled, leg.speed + leg.accel * step
    if not legs:
        return 0.0, 0.0
    return travelled, legs[-1].end_speed


def plan(distance: float, speed: float, accel: float = 1.0,
         profile: str = TRAPEZOID, fps: float = 10.0,
         heading: float = 0.0, start: Tuple[float, float] = (0.0, 0.0)) -> Plan:
    """Sample the drive at ``fps``.

    Frames sit at ``t = index / fps`` and the clip ends on the exact duration, so
    the last step can be shorter than the others when ``duration * fps`` is not a
    whole number (the CSV carries the real time of every row).
    """
    fps = max(1.0, float(fps))
    legs = phases(distance, speed, accel, profile)
    duration = sum(leg.duration for leg in legs)
    count = int(math.ceil(duration * fps - 1e-9)) + 1
    direction = forward(heading)
    frames: List[Frame] = []
    for index in range(count):
        time = min(index / fps, duration)
        travelled, speed_now = sample(legs, time)
        frames.append(Frame(
            index=index, time=time, distance=travelled, speed=speed_now,
            x=start[0] + direction[0] * travelled,
            y=start[1] + direction[1] * travelled,
            yaw=float(heading),
        ))
    return Plan(frames=frames, profile=profile, fps=fps, duration=duration,
                distance=max(0.0, float(distance)), cruise_speed=float(speed),
                accel=float(accel), heading=float(heading),
                start=(float(start[0]), float(start[1])))


def csv_text(plan_: Plan, mount: Mount = Mount()) -> str:
    """The per-frame truth as CSV: header plus one row per frame.

    ``mount`` is the camera's vehicle-frame pose; each row carries the camera's
    world pose it implies (see :func:`camera_world_pose`).
    """
    lines = [",".join(CSV_HEADER)]
    for frame in plan_.frames:
        camera = camera_world_pose(frame, mount)
        lines.append(
            f"{frame.index},{frame.time:.6f},{frame.distance:.6f},{frame.speed:.6f},"
            f"{frame.x:.6f},{frame.y:.6f},{frame.yaw:.4f},"
            f"{camera[0]:.6f},{camera[1]:.6f},{camera[2]:.6f},"
            f"{camera[3]:.4f},{camera[4]:.4f},{camera[5]:.4f}")
    return "\n".join(lines) + "\n"


def summary(plan_: Plan) -> str:
    """A one-line description for the panel / the operator report."""
    return (f"{len(plan_.frames)} frames @ {plan_.fps:g} fps, "
            f"{plan_.duration:.2f} s, {plan_.distance:.2f} m, "
            f"peak {plan_.peak_speed:.2f} m/s")
