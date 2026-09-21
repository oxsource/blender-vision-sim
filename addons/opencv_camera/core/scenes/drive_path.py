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
from typing import List, Mapping, Sequence, Tuple

#: speed profiles
CONSTANT = "constant"    #: hold one speed for the whole drive
TRAPEZOID = "trapezoid"  #: accelerate to the cruise speed, cruise, brake to a stop
PROFILES = (CONSTANT, TRAPEZOID)

#: the per-frame truth, one row per rendered frame.  The leading columns describe
#: the vehicle; after them comes **one group of six columns per recorded camera**,
#: in the order the caller lists them (``core.scenes.avm_cameras.CAMERAS``), each
#: naming the camera it belongs to.  See :func:`csv_header`.
CSV_VEHICLE_COLUMNS = ("frame", "time_s", "distance_m", "speed_mps",
                       "x_m", "y_m", "yaw_deg")

#: the six columns of one camera's group, suffixed onto ``cam_<camera>_``.  The
#: last three are **Blender XYZ Euler angles in degrees**, not a camera
#: pitch/yaw pair: the renderer's pose is an object transform, and pretending
#: otherwise is how a downstream consumer ends up applying them to the wrong
#: axes (``docs/drive-scene-multicam.md`` section 5.1).
CSV_CAMERA_COLUMNS = ("x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg")

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


def csv_header(cameras: Sequence[str]) -> Tuple[str, ...]:
    """The CSV's columns: the vehicle's, then one group of six per camera.

    ``7 + 6 * N`` columns.  The camera name is **in** the column name, so "which
    column belongs to which camera" is answerable by reading it - the previous
    single ``cam_*`` group only described one camera and never said which
    (``docs/drive-scene-multicam.md`` section 5.1).
    """
    columns = list(CSV_VEHICLE_COLUMNS)
    for camera in cameras:
        columns.extend(f"cam_{camera}_{name}" for name in CSV_CAMERA_COLUMNS)
    return tuple(columns)


def csv_text(plan_: Plan, mounts: Mapping[str, Mount]) -> str:
    """The per-frame truth as CSV: header plus one row per frame.

    ``mounts`` is ``{camera: vehicle-frame mount pose}``; the camera groups appear
    in the mapping's iteration order and every row carries each camera's **world**
    pose (vehicle pose composed with that fixed mount pose - see
    :func:`camera_world_pose`).
    """
    cameras = list(mounts)
    lines = [",".join(csv_header(cameras))]
    for frame in plan_.frames:
        cells = [str(frame.index), f"{frame.time:.6f}", f"{frame.distance:.6f}",
                 f"{frame.speed:.6f}", f"{frame.x:.6f}", f"{frame.y:.6f}",
                 f"{frame.yaw:.4f}"]
        for camera in cameras:
            x, y, z, roll, pitch, yaw = camera_world_pose(frame, mounts[camera])
            cells.extend((f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                          f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}"))
        lines.append(",".join(cells))
    return "\n".join(lines) + "\n"


def summary(plan_: Plan) -> str:
    """A one-line description for the panel / the operator report."""
    return (f"{len(plan_.frames)} frames @ {plan_.fps:g} fps, "
            f"{plan_.duration:.2f} s, {plan_.distance:.2f} m, "
            f"peak {plan_.peak_speed:.2f} m/s")
