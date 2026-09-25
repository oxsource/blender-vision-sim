"""Road Scene motion plan: drive a closed track forward or in reverse.

Pure Python (no ``bpy``).  Where :mod:`core.scenes.drive_path` samples a straight
line, this module samples the closed *track* of :mod:`core.scenes.road_track`,
so the same clip can be driven at any point on the loop, at any speed and in
either direction - the "road type x speed x direction" matrix of ``filament_avm``
M5 (CH-018).

The per-frame truth adds what a slope needs to the Drive Scene's seven columns:
``z_m`` (the vehicle centre's height), ``pitch_deg`` (road grade, nose up
positive) and ``roll_deg``, plus the two labels that make a matrix row
self-describing - ``segment`` (the named piece of the loop) and ``road_type``
(``straight`` / ``curve`` / ``slope_up`` / ``slope_down``) and ``direction``.

A reversing vehicle is expressed honestly: it faces **opposite** the direction of
travel, so its ``yaw`` is the tangent's heading plus 180 deg and its ``pitch``
flips sign.  The exported pose stays a plain Blender XYZ Euler triple, exactly
like the Drive Scene, so a consumer needs no new convention.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from . import drive_path, road_track, vehicle

__all__ = [
    "FORWARD", "REVERSE", "DIRECTIONS", "SCENARIO", "PROFILES", "Frame", "Plan",
    "CSV_VEHICLE_COLUMNS", "CSV_CAMERA_COLUMNS",
    "plan", "speed_field", "parking_plan", "csv_header", "csv_text", "summary",
    "camera_world_pose", "steering_deg", "gear_for",
    "GEAR_PARKED", "GEAR_REVERSE", "GEAR_DRIVE", "GEARS",
    "rotation_xyz", "euler_xyz",
]

FORWARD = "forward"
REVERSE = "reverse"
DIRECTIONS: Tuple[str, ...] = (FORWARD, REVERSE)

#: The geometry-aware profile: start at rest, cruise, slow for the zebra
#: crossing, and brake to a stop exactly at the end of the drive (the product's
#: low-speed envelope, ``docs/road-scene.md`` section 4).
SCENARIO = "scenario"
PROFILES: Tuple[str, ...] = (drive_path.CONSTANT, drive_path.TRAPEZOID, SCENARIO)

#: the distance grid ``scenario`` is solved on [m] - fine enough that the
#: acceleration-limited passes are smooth, coarse enough to stay cheap
SPEED_FIELD_RESOLUTION_M = 0.25

#: the per-frame truth.  The first seven columns are **exactly** the Drive
#: Scene's (a consumer that reads them by name keeps working); the next three
#: carry the vertical pose, then the matrix labels M5 adds, and finally the two
#: vehicle signals an AVM algorithm consumes: the **front-wheel steering angle**
#: (``steering_deg``, left positive) and the **gear** (``P`` / ``R`` / ``D``).
CSV_VEHICLE_COLUMNS: Tuple[str, ...] = (
    "frame", "time_s", "distance_m", "speed_mps",
    "x_m", "y_m", "yaw_deg",
    "z_m", "pitch_deg", "roll_deg",
    "segment", "road_type", "direction",
    "steering_deg", "gear",
)

#: the gear a consumer should see at a frame: parked when stopped, else the
#: direction of travel
GEAR_PARKED, GEAR_REVERSE, GEAR_DRIVE = "P", "R", "D"
GEARS: Tuple[str, ...] = (GEAR_PARKED, GEAR_REVERSE, GEAR_DRIVE)

#: below this speed [m/s] the car counts as stopped (gear ``P``)
GEAR_STOPPED_MPS = 1e-3

#: one camera's group: its world pose as a Blender XYZ Euler triple (degrees),
#: identical to the Drive Scene so the same consumer reads both.
CSV_CAMERA_COLUMNS: Tuple[str, ...] = drive_path.CSV_CAMERA_COLUMNS


@dataclass(frozen=True)
class Frame:
    """One frame's truth on the track."""

    index: int
    time: float
    distance: float
    speed: float
    x: float
    y: float
    z: float
    yaw: float    #: the vehicle's heading (nose), degrees
    pitch: float  #: nose up positive, degrees
    roll: float
    segment: str
    road_type: str
    direction: str
    steering_deg: float = 0.0   #: front-wheel angle, left positive
    gear: str = GEAR_PARKED     #: ``P`` / ``R`` / ``D``


def steering_deg(curvature: float, direction: str = FORWARD,
                 wheel_base: float = vehicle.WHEEL_BASE_M) -> float:
    """The front-wheel steering angle [deg] for a path curvature.

    The bicycle model ``tan(delta) = wheel_base * curvature``; the sign flips
    when reversing, because the car's nose then points against the travel and
    the wheels turn the other way relative to the body.
    """
    sign = 1.0 if str(direction).lower() == FORWARD else -1.0
    return math.degrees(math.atan(float(wheel_base) * float(curvature) * sign))


def gear_for(speed: float, direction: str = FORWARD) -> str:
    """``P`` when stopped, else ``D`` forward / ``R`` reverse."""
    if abs(float(speed)) <= GEAR_STOPPED_MPS:
        return GEAR_PARKED
    return GEAR_REVERSE if str(direction).lower() == REVERSE else GEAR_DRIVE


@dataclass(frozen=True)
class Plan:
    """A sampled drive plus the parameters that produced it."""

    frames: List[Frame]
    track: road_track.RoadTrack
    profile: str
    fps: float
    duration: float
    distance: float
    cruise_speed: float
    accel: float
    direction: str
    start_distance: float
    loops: float

    @property
    def peak_speed(self) -> float:
        return max((frame.speed for frame in self.frames), default=0.0)

    @property
    def sign(self) -> float:
        return 1.0 if self.direction == FORWARD else -1.0


# ---------------------------------------------------------------------------
# 3D poses (pure maths - the flat Drive Scene formula is the pitch == 0 case)
# ---------------------------------------------------------------------------
def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rotation_xyz(rx_deg: float, ry_deg: float, rz_deg: float):
    """``Rz @ Ry @ Rx`` for Blender's XYZ Euler order (degrees)."""
    rx, ry, rz = (math.radians(value) for value in (rx_deg, ry_deg, rz_deg))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rx @ Ry @ Rz composition, written as the XYZ Euler matrix
    return [
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
        [-sy, cy * sx, cx * cy],
    ]


def euler_xyz(matrix) -> Tuple[float, float, float]:
    """The XYZ Euler angles (degrees) of an ``Rz @ Ry @ Rx`` matrix."""
    sy = max(-1.0, min(1.0, -matrix[2][0]))
    ry = math.asin(sy)
    if abs(sy) < 1.0 - 1e-9:
        rx = math.atan2(matrix[2][1], matrix[2][2])
        rz = math.atan2(matrix[1][0], matrix[0][0])
    else:  # gimbal lock: fold the free rotation into rz
        rx = math.atan2(-matrix[1][2], matrix[1][1])
        rz = 0.0
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def _matvec(matrix, vector):
    return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def camera_world_pose(frame: Frame, mount: drive_path.Mount
                      ) -> Tuple[float, float, float, float, float, float]:
    """The camera's world pose at ``frame``: ``(x, y, z, roll, pitch, yaw)``.

    ``frame`` carries the vehicle's full 3D pose, so unlike the flat
    ``drive_path.camera_world_pose`` this composes the vehicle rotation with the
    mount's, which is what a sloped or reversing frame needs.  With a flat,
    forward frame the two agree exactly (asserted in the tests).
    """
    vehicle = rotation_xyz(frame.pitch, frame.roll, frame.yaw)
    mount_matrix = rotation_xyz(*mount.rotation_deg)
    world = _matmul(vehicle, mount_matrix)
    offset = _matvec(vehicle, tuple(float(value) for value in mount.location))
    rx, ry, rz = euler_xyz(world)
    return (frame.x + offset[0], frame.y + offset[1], frame.z + offset[2],
            rx, ry, rz)


# ---------------------------------------------------------------------------
# the scenario speed field
# ---------------------------------------------------------------------------
def _state_at(times: Sequence[float], distances: Sequence[float],
              speeds: Sequence[float], time: float) -> Tuple[float, float]:
    """``(distance, speed)`` at ``time``, exactly integrating the profile.

    Each cell of the field is constant-acceleration (``v^2`` linear in ``s``),
    so ``s(t)`` inside a cell is a quadratic - using it instead of interpolating
    a coarse ``(t, s)`` table is what keeps ``ds/dt == speed`` to frame accuracy
    when the car creeps out of rest.
    """
    if time <= times[0]:
        return float(distances[0]), float(speeds[0])
    if time >= times[-1]:
        return float(distances[-1]), float(speeds[-1])
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= time:
            lo = mid
        else:
            hi = mid
    span = distances[hi] - distances[lo]
    v0, v1 = float(speeds[lo]), float(speeds[hi])
    accel = (v1 * v1 - v0 * v0) / (2.0 * span) if span > 1e-12 else 0.0
    tau = time - times[lo]
    if abs(accel) < 1e-9:
        return min(float(distances[hi]), distances[lo] + v0 * tau), v0
    distance = distances[lo] + v0 * tau + 0.5 * accel * tau * tau
    speed = max(0.0, v0 + accel * tau)
    return max(float(distances[lo]), min(float(distances[hi]), distance)), speed


def _within(s: float, start: float, end: float, length: float) -> bool:
    """Is arc length ``s`` inside ``[start, end]`` on a loop of ``length``?"""
    if length <= 0.0:
        return False
    value = math.fmod(float(s), length)
    if value < 0.0:
        value += length
    a = math.fmod(float(start), length)
    if a < 0.0:
        a += length
    b = math.fmod(float(end), length)
    if b < 0.0:
        b += length
    if a <= b:
        return a <= value <= b
    return value >= a or value <= b


def speed_field(track: road_track.RoadTrack, *, distance: float,
                start_distance: float = 0.0, direction: str = FORWARD,
                cruise: float = 5.0, slow_speed: float = 3.0,
                accel: float = 1.0, decel: float = 1.0,
                start_speed: float = 0.0, end_speed: float = 0.0,
                slow_zones: Sequence[Tuple[float, float]] = (),
                resolution: float = SPEED_FIELD_RESOLUTION_M
                ) -> Tuple[List[float], List[float], List[float]]:
    """``(distance, speed, time)`` for a geometry-aware single pass.

    The target speed is ``slow_speed`` inside any ``slow_zones`` arc-length range
    (the zebra crossing) and ``cruise`` elsewhere; a forward pass limits how fast
    the car may accelerate into it and a backward pass how hard it may brake out
    of it, so the car leaves the start at rest, reaches the cruise, slows for the
    crossing and stops exactly at the end - a real braking distance, not a step.
    Both ends are pinned to ``start_speed`` / ``end_speed``.
    """
    distance = max(0.0, float(distance))
    if distance <= 0.0:
        return [0.0], [max(0.0, float(start_speed))], [0.0]
    cruise = max(0.0, float(cruise))
    slow_speed = max(0.0, min(float(slow_speed), cruise)) if cruise > 0.0 else 0.0
    accel = max(float(accel), 1e-3)
    decel = max(float(decel), 1e-3)
    sign = 1.0 if str(direction).lower() == FORWARD else -1.0
    steps = max(1, int(math.ceil(distance / max(0.05, float(resolution)))))
    step = distance / steps

    def target(travelled: float) -> float:
        s = start_distance + sign * travelled
        for zone_start, zone_end in slow_zones:
            if _within(s, zone_start, zone_end, track.length):
                return slow_speed
        return cruise

    speeds = [target(index * step) for index in range(steps + 1)]
    speeds[0] = min(speeds[0], max(0.0, float(start_speed)))
    speeds[-1] = min(speeds[-1], max(0.0, float(end_speed)))
    for index in range(1, steps + 1):          # acceleration limit
        ceiling = math.sqrt(speeds[index - 1] ** 2 + 2.0 * accel * step)
        speeds[index] = min(speeds[index], ceiling)
    for index in range(steps - 1, -1, -1):     # braking limit
        ceiling = math.sqrt(speeds[index + 1] ** 2 + 2.0 * decel * step)
        speeds[index] = min(speeds[index], ceiling)
    speeds[0] = min(speeds[0], max(0.0, float(start_speed)))
    speeds[-1] = min(speeds[-1], max(0.0, float(end_speed)))

    distances = [index * step for index in range(steps + 1)]
    times = [0.0]
    for index in range(1, steps + 1):
        v0, v1 = speeds[index - 1], speeds[index]
        accel = (v1 * v1 - v0 * v0) / (2.0 * step)
        if abs(accel) < 1e-9:
            duration = step / v0 if v0 > 1e-9 else 0.0
        else:
            duration = (v1 - v0) / accel
        times.append(times[-1] + max(0.0, duration))
    return distances, speeds, times


# ---------------------------------------------------------------------------
# the plan
# ---------------------------------------------------------------------------
def plan(track: road_track.RoadTrack, speed: float = 5.0,
         direction: str = FORWARD, profile: str = drive_path.CONSTANT,
         accel: float = 1.0, fps: float = 10.0, loops: float = 1.0,
         start_distance: float = 0.0, start_speed: float = 0.0,
         end_speed: float = 0.0, slow_speed: float = 3.0,
         decel: Optional[float] = None,
         slow_zones: Sequence[Tuple[float, float]] = ()) -> Plan:
    """Sample a drive of ``loops`` times around ``track``.

    ``direction`` is :data:`FORWARD` or :data:`REVERSE`; ``start_distance`` is
    the arc length (m) along the loop the drive starts at, which is what lets a
    clip cover exactly one segment.  ``constant`` / ``trapezoid`` reuse the Drive
    Scene's speed profile; :data:`SCENARIO` solves the geometry-aware field of
    :func:`speed_field` (start/stop, slow zones).
    """
    direction = str(direction).lower()
    if direction not in DIRECTIONS:
        raise ValueError(f"direction must be one of {DIRECTIONS}, got {direction!r}")
    fps = max(1.0, float(fps))
    sign = 1.0 if direction == FORWARD else -1.0
    distance = max(0.0, abs(float(loops)) * track.length)

    def frame_at(index: int, time: float, travelled: float, speed_now: float) -> Frame:
        s = start_distance + sign * travelled
        pose = track.pose_at(s)
        segment = track.segment_at(s)
        if direction == REVERSE:
            yaw = road_track.wrap_deg(pose.yaw + 180.0)
            pitch = -pose.pitch
            roll = -pose.roll
        else:
            yaw, pitch, roll = pose.yaw, pose.pitch, pose.roll
        return Frame(
            index=index, time=time, distance=travelled, speed=speed_now,
            x=pose.x, y=pose.y, z=pose.z, yaw=yaw, pitch=pitch, roll=roll,
            segment=segment.name, road_type=segment.road_type,
            direction=direction,
            steering_deg=steering_deg(pose.curvature, direction),
            gear=gear_for(speed_now, direction))

    if str(profile).lower() == SCENARIO:
        braking = accel if decel is None else decel
        distances, speeds, times = speed_field(
            track, distance=distance, start_distance=start_distance,
            direction=direction, cruise=speed, slow_speed=slow_speed,
            accel=accel, decel=braking, start_speed=start_speed,
            end_speed=end_speed, slow_zones=slow_zones)
        duration = times[-1]
        count = int(math.ceil(duration * fps - 1e-9)) + 1
        frames = []
        for index in range(count):
            time = min(index / fps, duration)
            travelled, speed_now = _state_at(times, distances, speeds, time)
            frames.append(frame_at(index, time, travelled, speed_now))
        return Plan(frames=frames, track=track, profile=profile, fps=fps,
                    duration=duration, distance=distance, cruise_speed=float(speed),
                    accel=float(accel), direction=direction,
                    start_distance=float(start_distance), loops=float(loops))

    legs = drive_path.phases(distance, speed, accel, profile,
                             start_speed=start_speed, end_speed=end_speed)
    duration = sum(leg.duration for leg in legs)
    count = int(math.ceil(duration * fps - 1e-9)) + 1
    frames = []
    for index in range(count):
        time = min(index / fps, duration)
        travelled, speed_now = drive_path.sample(legs, time)
        frames.append(frame_at(index, time, travelled, speed_now))
    return Plan(frames=frames, track=track, profile=profile, fps=fps,
                duration=duration, distance=distance, cruise_speed=float(speed),
                accel=float(accel), direction=direction,
                start_distance=float(start_distance), loops=float(loops))


def parking_plan(track: road_track.RoadTrack, *, s_entry: float, radius: float,
                 cruise: float = 25.0 / 3.6, parking_speed: float = 1.5,
                 slow_speed: float = 3.0, accel: float = 1.5,
                 decel: float = 2.5, fps: float = 10.0,
                 slow_zones: Sequence[Tuple[float, float]] = ()) -> Plan:
    """A whole-lap scenario that starts and ends parked in a bay.

    The clip is three pieces stitched on one clock: **pull out** of the bay
    forwards onto the centreline, **one lap** with the scenario speed profile,
    then **reverse back into the bay**.  The car's nose stays along the arc's
    increasing-parameter tangent throughout, so it leaves and returns nose-out -
    and the final piece is a genuine reversing manoeuvre (``direction=reverse``),
    which is what M5's direction axis needs.
    """
    arc = road_track.parking_arc(track, s_entry, radius)
    depth = arc.length
    fps = max(1.0, float(fps))
    accel = max(float(accel), drive_path.MIN_ACCEL)
    decel = max(float(decel), drive_path.MIN_ACCEL)
    parking_speed = max(0.05, float(parking_speed))

    exit_legs = drive_path.phases(depth, parking_speed, accel,
                                  drive_path.TRAPEZOID, start_speed=0.0,
                                  end_speed=parking_speed)
    entry_legs = drive_path.phases(depth, parking_speed, decel,
                                   drive_path.TRAPEZOID, start_speed=parking_speed,
                                   end_speed=0.0)
    lap = plan(track, speed=cruise, direction=FORWARD, profile=SCENARIO,
               accel=accel, decel=decel, fps=fps, loops=1.0,
               start_distance=s_entry, start_speed=parking_speed,
               end_speed=parking_speed, slow_speed=slow_speed,
               slow_zones=slow_zones)

    frames: List[Frame] = []
    exit_duration = sum(leg.duration for leg in exit_legs)
    entry_duration = sum(leg.duration for leg in entry_legs)

    def add_arc(legs, duration, reverse: bool, time_offset: float,
                distance_offset: float, label: str) -> None:
        count = int(math.ceil(duration * fps - 1e-9)) + 1
        for index in range(count):
            time = min(index / fps, duration)
            travelled, speed_now = drive_path.sample(legs, time)
            u = max(0.0, min(depth, depth - travelled if reverse else travelled))
            pose = arc.arc.pose_at(u, wrap=False)
            frames.append(Frame(
                index=0, time=time_offset + time, distance=distance_offset + travelled,
                speed=speed_now, x=pose.x, y=pose.y, z=pose.z, yaw=pose.yaw,
                pitch=pose.pitch, roll=pose.roll, segment=label,
                road_type="parking",
                direction=REVERSE if reverse else FORWARD,
                steering_deg=steering_deg(
                    pose.curvature, REVERSE if reverse else FORWARD),
                gear=gear_for(speed_now, REVERSE if reverse else FORWARD)))

    add_arc(exit_legs, exit_duration, False, 0.0, 0.0, "park_exit")
    for frame in lap.frames[1:]:                    # frame 0 is the exit's end pose
        frames.append(replace(frame, index=0, time=exit_duration + frame.time,
                              distance=depth + frame.distance, direction=FORWARD))
    add_arc(entry_legs, entry_duration, True,
            exit_duration + lap.duration, depth + lap.distance, "park_entry")

    frames = [replace(frame, index=index) for index, frame in enumerate(frames)]
    duration = exit_duration + lap.duration + entry_duration
    return Plan(frames=frames, track=track, profile="parking", fps=fps,
                duration=duration, distance=depth + lap.distance + depth,
                cruise_speed=float(cruise), accel=float(accel),
                direction=FORWARD, start_distance=float(s_entry), loops=1.0)


def csv_header(cameras: Sequence[str]) -> Tuple[str, ...]:
    """The CSV's columns: the vehicle's, then one group of six per camera."""
    columns = list(CSV_VEHICLE_COLUMNS)
    for camera in cameras:
        columns.extend(f"cam_{camera}_{name}" for name in CSV_CAMERA_COLUMNS)
    return tuple(columns)


def csv_text(plan_: Plan, mounts: Mapping[str, drive_path.Mount]) -> str:
    """The per-frame truth as CSV (vehicle pose + each camera's world pose)."""
    cameras = list(mounts)
    lines = [",".join(csv_header(cameras))]
    for frame in plan_.frames:
        cells = [str(frame.index), f"{frame.time:.6f}", f"{frame.distance:.6f}",
                 f"{frame.speed:.6f}", f"{frame.x:.6f}", f"{frame.y:.6f}",
                 f"{frame.yaw:.4f}", f"{frame.z:.6f}", f"{frame.pitch:.4f}",
                 f"{frame.roll:.4f}", frame.segment, frame.road_type,
                 frame.direction, f"{frame.steering_deg:.4f}", frame.gear]
        for camera in cameras:
            x, y, z, roll, pitch, yaw = camera_world_pose(frame, mounts[camera])
            cells.extend((f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                          f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}"))
        lines.append(",".join(cells))
    return "\n".join(lines) + "\n"


def summary(plan_: Plan) -> str:
    """A one-line description for the panel / the operator report."""
    return (f"{len(plan_.frames)} frames @ {plan_.fps:g} fps, "
            f"{plan_.duration:.2f} s, {plan_.distance:.2f} m {plan_.direction}, "
            f"peak {plan_.peak_speed:.2f} m/s")
