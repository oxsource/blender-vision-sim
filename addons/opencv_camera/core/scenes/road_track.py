"""Road Scene test track: a closed loop of straights, curves and ramps.

Pure Python (no ``bpy``): the geometry a transparent-chassis validation needs -
a **closed** road whose centreline carries straights, constant-radius curves and
up/down slopes, so the same clip can be driven forwards, backwards and at
different speeds (``filament_avm`` M5 / CH-018).

The track is built from an ordered list of :class:`SegmentSpec`; each segment is
turned into a :class:`_BuiltSegment` whose ``evaluate(t)`` returns the pose at
``t`` metres into that segment.  A closed track is only accepted when the last
segment lands back on the first pose (position, heading and height), so a typo in
a template can never silently produce a path that does not connect.

Conventions match the rest of the add-on: metres, Z up, the vehicle's nose is
``+Y`` at yaw 0, so its forward is ``(-sin(yaw), cos(yaw))``.  Track elevation is
carried by the pose's ``z`` and ``pitch`` (degrees, positive = nose up); curves
are flat (``roll`` stays 0) because a fixed-plane ground model is exactly what
M5 has to measure against, and banking the road would hide that error.

Segment kinds
-------------
``straight``  a flat straight (no rise): the baseline.
``curve``     a circular arc; signed ``radius`` (positive = left) and
              ``angle_deg`` (signed, must agree with ``radius``), flat.
``ramp``      a vertical curve: a straight in plan whose grade follows a half
              sine, so it starts and ends flat and joins the straights without a
              pitch kink; ``rise`` is the total height change (signed).
``s_curve``   a lane change: yaw follows ``amplitude * sin(2*pi*n*s/length)``.
              With an integer ``cycles`` it ends with the heading it started
              with and no net lateral offset, so it is a straight's worth of
              curve in both directions.  (Its forward reach is slightly short of
              its nominal length, so the default loop leaves it out; it exists
              for templates and tests that want a cheap double-apex.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

__all__ = [
    "Pose", "SegmentSpec", "BuiltSegment", "RoadTrack", "TrackPreset",
    "STRAIGHT", "CURVE", "RAMP", "S_CURVE", "KINDS",
    "default_track_specs", "default_track", "build_track",
    "TRACK_PRESETS", "preset_specs", "preset_track", "build_from",
    "crosswalk_distance", "crosswalk_slow_zone", "ParkingArc", "parking_arc",
    "DEFAULT_STRAIGHT_M", "DEFAULT_RADIUS_M", "DEFAULT_RAMP_RISE_M",
    "DEFAULT_RAMP_LENGTH_M", "DEFAULT_ROAD_WIDTH_M", "DEFAULT_SHOULDER_M",
]

STRAIGHT, CURVE, RAMP, S_CURVE = "straight", "curve", "ramp", "s_curve"
KINDS: Tuple[str, ...] = (STRAIGHT, CURVE, RAMP, S_CURVE)

#: default template dimensions [m]
DEFAULT_STRAIGHT_M = 40.0
DEFAULT_RADIUS_M = 14.0
DEFAULT_RAMP_RISE_M = 1.6
DEFAULT_RAMP_LENGTH_M = 12.0
DEFAULT_ROAD_WIDTH_M = 7.0
DEFAULT_SHOULDER_M = 1.0

#: loop length of the four-straight template: ``4 * straight + 2 * pi * radius``
#: (the ramps only split the two side straights, so they do not change it)
def track_length_for(straight: float, radius: float) -> float:
    return 4.0 * float(straight) + 2.0 * math.pi * float(radius)


@dataclass(frozen=True)
class TrackPreset:
    """A named loop size: the template dimensions that ship with the scene.

    Two coexist so a clip can trade render time for scale: ``compact`` keeps a
    whole loop near 20-30 s at the product's low-speed envelope, ``full`` keeps
    the original proportions for long-straight work.  Both contain every M5 road
    type (straight / curve / up / down / crossing), so either one alone is a
    complete test track.
    """

    key: str
    label: str
    description: str
    straight: float
    radius: float
    ramp_rise: float
    ramp_length: float
    road_width: float = DEFAULT_ROAD_WIDTH_M
    shoulder: float = DEFAULT_SHOULDER_M

    @property
    def length(self) -> float:
        return track_length_for(self.straight, self.radius)


#: the shipped loop sizes, in menu order.  ``compact`` is the default (a whole
#: loop in ~25 s at 25 km/h); ``full`` is the original 248 m template.
TRACK_PRESETS: Dict[str, TrackPreset] = {
    preset.key: preset for preset in (
        TrackPreset(
            key="compact", label="Compact (~124 m)",
            description="Compact loop: 20 m straights, 7 m curves, 10 m ramps - "
                        "a whole scenario lap in about 25 s at 25 km/h",
            straight=20.0, radius=7.0, ramp_rise=1.0, ramp_length=10.0),
        TrackPreset(
            key="full", label="Full (~248 m)",
            description="Original loop: 40 m straights, 14 m curves, 12 m ramps - "
                        "longer straights and gentler curves for boundary work",
            straight=DEFAULT_STRAIGHT_M, radius=DEFAULT_RADIUS_M,
            ramp_rise=DEFAULT_RAMP_RISE_M, ramp_length=DEFAULT_RAMP_LENGTH_M),
    )}

#: how finely an s_curve is integrated [m]; its evaluation interpolates this
#: table, so the value only has to be well below one frame's travel
S_CURVE_STEP_M = 0.05

#: zebra crossing geometry [m]: bars run along travel, repeat across the lane,
#: and the crossing sits just before the up ramp (see :func:`crosswalk_distance`)
CROSSWALK_DEPTH = 3.0
CROSSWALK_BAR = 0.5
CROSSWALK_GAP = 0.5
CROSSWALK_MARGIN = 0.35
CROSSWALK_CLEAR = 1.0
#: the driving-speed profile slows for this many metres either side of the
#: crossing (the renderer paints the bars itself; this is the geometry of the
#: slow zone, so the model and the paint cannot disagree)
CROSSWALK_SLOW_HALF_M = 6.0


def wrap_deg(angle: float) -> float:
    """An angle in ``(-180, 180]`` degrees."""
    value = math.fmod(float(angle), 360.0)
    if value <= -180.0:
        value += 360.0
    elif value > 180.0:
        value -= 360.0
    return value


@dataclass(frozen=True)
class Pose:
    """A centreline pose.

    ``yaw`` is the tangent's heading, ``pitch`` the road grade (positive = rising
    in the direction of travel), ``curvature`` the signed horizontal curvature
    ``1/R`` (positive = left).  ``s`` is the distance from the track's start.
    """

    x: float
    y: float
    z: float
    yaw: float
    pitch: float = 0.0
    roll: float = 0.0
    curvature: float = 0.0
    s: float = 0.0


@dataclass(frozen=True)
class SegmentSpec:
    """One piece of the centreline, as the templates and tests write it."""

    name: str
    kind: str
    length: float
    radius: float = 0.0         #: curve only: signed turning radius [m]
    angle_deg: float = 0.0      #: curve only: signed total heading change [deg]
    rise: float = 0.0           #: ramp only: signed height change [m]
    amplitude_deg: float = 0.0  #: s_curve only: peak heading amplitude [deg]
    cycles: float = 1.0         #: s_curve only: full sine periods

    def road_type(self) -> str:
        """The M5 label: which kind of road the segment is."""
        if self.kind == RAMP:
            return "slope_up" if self.rise > 0 else ("slope_down" if self.rise < 0 else "flat")
        return self.kind


@dataclass(frozen=True)
class BuiltSegment:
    """A spec placed in the world, with its own evaluator."""

    spec: SegmentSpec
    start: Pose
    length: float
    end: Pose
    evaluate: Callable[[float], Pose]

    @property
    def road_type(self) -> str:
        return self.spec.road_type()

    @property
    def name(self) -> str:
        return self.spec.name


def _place(start: Pose, right: float, forward: float, dz: float,
           yaw: float, pitch: float, curvature: float, s: float) -> Pose:
    """Put a local ``(right, forward, z)`` offset into the world at ``start``.

    ``right`` is the vehicle's +X at ``start.yaw`` (a left turn therefore has a
    negative ``right``), ``forward`` its +Y, and ``yaw``/``pitch`` are the
    segment's absolute angles.
    """
    angle = math.radians(start.yaw)
    cos, sin = math.cos(angle), math.sin(angle)
    return Pose(
        x=start.x + right * cos - forward * sin,
        y=start.y + right * sin + forward * cos,
        z=start.z + dz,
        yaw=wrap_deg(yaw),
        pitch=pitch,
        roll=0.0,
        curvature=curvature,
        s=s,
    )


def _straight(spec: SegmentSpec, start: Pose) -> BuiltSegment:
    if abs(spec.rise) > 1e-9:
        raise ValueError(f"{spec.name}: a straight is flat; use a ramp for a slope")

    def evaluate(t: float) -> Pose:
        return _place(start, 0.0, t, 0.0, start.yaw, 0.0, 0.0, start.s + t)

    end = evaluate(spec.length)
    return BuiltSegment(spec, start, spec.length, end, evaluate)


def _curve(spec: SegmentSpec, start: Pose) -> BuiltSegment:
    radius = float(spec.radius)
    if abs(radius) < 1e-9:
        raise ValueError(f"{spec.name}: a curve needs a non-zero radius")
    if radius * spec.angle_deg < 0.0:
        raise ValueError(
            f"{spec.name}: radius {radius} and angle {spec.angle_deg} have "
            "opposite signs; a left turn is both positive")
    length = abs(radius) * abs(math.radians(spec.angle_deg))

    def evaluate(t: float) -> Pose:
        psi = t / radius                       # radians, signed
        right = radius * (math.cos(psi) - 1.0)
        forward = radius * math.sin(psi)
        curvature = 1.0 / radius
        return _place(start, right, forward, 0.0,
                      start.yaw + math.degrees(psi), 0.0, curvature, start.s + t)

    end = evaluate(length)
    return BuiltSegment(spec, start, length, end, evaluate)


def _ramp(spec: SegmentSpec, start: Pose) -> BuiltSegment:
    length = float(spec.length)
    if length <= 0.0:
        raise ValueError(f"{spec.name}: a ramp needs a positive length")
    # grade(s) = peak * sin(pi s / L) -> integral over [0, L] is peak * 2L / pi
    peak = spec.rise * math.pi / (2.0 * length)

    def evaluate(t: float) -> Pose:
        grade = peak * math.sin(math.pi * t / length)
        z = spec.rise * 0.5 * (1.0 - math.cos(math.pi * t / length))
        pitch = math.degrees(math.atan(grade))
        return _place(start, 0.0, t, z, start.yaw, pitch, 0.0, start.s + t)

    end = evaluate(length)
    return BuiltSegment(spec, start, length, end, evaluate)


def _s_curve(spec: SegmentSpec, start: Pose) -> BuiltSegment:
    length = float(spec.length)
    if length <= 0.0:
        raise ValueError(f"{spec.name}: an s_curve needs a positive length")
    amplitude = math.radians(spec.amplitude_deg)
    cycles = float(spec.cycles)
    steps = max(8, int(round(length / S_CURVE_STEP_M)))
    step = length / steps

    # integrate d(right)/ds = -sin(psi), d(forward)/ds = cos(psi) with the
    # midpoint rule; only the table is kept, evaluation interpolates it
    ss: List[float] = [0.0]
    rights: List[float] = [0.0]
    forwards: List[float] = [0.0]
    psis: List[float] = [0.0]
    for index in range(steps):
        s0 = index * step
        s1 = s0 + step
        p0 = amplitude * math.sin(2.0 * math.pi * cycles * s0 / length)
        p1 = amplitude * math.sin(2.0 * math.pi * cycles * s1 / length)
        middle = 0.5 * (p0 + p1)
        rights.append(rights[-1] - math.sin(middle) * step)
        forwards.append(forwards[-1] + math.cos(middle) * step)
        psis.append(p1)
        ss.append(s1)

    def sample(t: float):
        t = min(max(0.0, t), length)
        position = t / step
        index = min(steps - 1, int(position))
        frac = position - index
        right = rights[index] + (rights[index + 1] - rights[index]) * frac
        forward = forwards[index] + (forwards[index + 1] - forwards[index]) * frac
        psi = psis[index] + (psis[index + 1] - psis[index]) * frac
        curvature = (amplitude * 2.0 * math.pi * cycles / length
                     * math.cos(2.0 * math.pi * cycles * t / length))
        return right, forward, psi, curvature

    def evaluate(t: float) -> Pose:
        right, forward, psi, curvature = sample(t)
        return _place(start, right, forward, 0.0,
                      start.yaw + math.degrees(psi), 0.0, curvature, start.s + t)

    end = evaluate(length)
    return BuiltSegment(spec, start, length, end, evaluate)


_BUILDERS: Dict[str, Callable[[SegmentSpec, Pose], BuiltSegment]] = {
    STRAIGHT: _straight,
    CURVE: _curve,
    RAMP: _ramp,
    S_CURVE: _s_curve,
}


@dataclass(frozen=True)
class RoadTrack:
    """A built centreline: a list of segments plus the queries the rest needs."""

    segments: Tuple[BuiltSegment, ...]
    closed: bool

    @property
    def length(self) -> float:
        return sum(segment.length for segment in self.segments)

    def segment_at(self, s: float) -> BuiltSegment:
        """The segment containing ``s`` (wrapping a closed track)."""
        if not self.segments:
            raise ValueError("the track has no segments")
        position = self._normalise(s)
        for segment in self.segments:
            if position < segment.start.s + segment.length - 1e-9:
                return segment
        return self.segments[-1]

    def pose_at(self, s: float, wrap: bool = True) -> Pose:
        """The centreline pose at distance ``s`` from the start."""
        if not self.segments:
            raise ValueError("the track has no segments")
        if wrap and self.closed:
            position = self._normalise(s)
        else:
            position = max(0.0, min(float(s), self.length))
        segment = self.segment_at(position)
        local = position - segment.start.s
        return replace(segment.evaluate(local), s=position)

    def _normalise(self, s: float) -> float:
        if not self.closed:
            return max(0.0, min(float(s), self.length))
        value = math.fmod(float(s), self.length)
        return value + self.length if value < 0.0 else value

    def samples(self, step: float = 1.0) -> List[Pose]:
        """A dense list of poses around the loop (for meshes and bounds)."""
        count = max(1, int(round(self.length / max(0.05, step))))
        if not self.closed:
            count += 1
        return [self.pose_at(index * self.length / count, wrap=False)
                for index in range(count)]

    def bounds(self) -> Tuple[float, float, float, float, float, float]:
        """``(min_x, max_x, min_y, max_y, min_z, max_z)`` of the centreline."""
        poses = self.samples()
        return (min(p.x for p in poses), max(p.x for p in poses),
                min(p.y for p in poses), max(p.y for p in poses),
                min(p.z for p in poses), max(p.z for p in poses))

    def describe(self) -> List[Dict]:
        """One record per segment, for ``clip.json`` and the panel."""
        return [{
            "name": segment.name,
            "road_type": segment.road_type,
            "kind": segment.spec.kind,
            "start_m": round(segment.start.s, 6),
            "end_m": round(segment.start.s + segment.length, 6),
            "length_m": round(segment.length, 6),
            "start_z_m": round(segment.start.z, 6),
            "end_z_m": round(segment.end.z, 6),
            "radius_m": round(segment.spec.radius, 6),
            "rise_m": round(segment.spec.rise, 6),
        } for segment in self.segments]


def build_track(specs: Sequence[SegmentSpec], closed: bool = True,
                tolerance: float = 1e-3, center: bool = False) -> RoadTrack:
    """Turn specs into a track, refusing one that does not close.

    ``tolerance`` is millimetres; a closed track must return to its start in
    position, height and heading.  ``center`` shifts the whole loop so its
    bounding-box centre sits on the origin in x/y (the height stays 0-based).
    """
    if not specs:
        raise ValueError("a track needs at least one segment")
    track = _build(specs, closed)
    if closed:
        first, last = track.segments[0].start, track.segments[-1].end
        error = max(math.hypot(last.x - first.x, last.y - first.y),
                    abs(last.z - first.z),
                    abs(math.radians(wrap_deg(last.yaw - first.yaw))))
        if error > tolerance:
            raise ValueError(
                f"the track does not close: {error:.6g} m/rad off after "
                f"{track.length:.3f} m (segments: "
                f"{', '.join(segment.name for segment in track.segments)})")
    if center:
        track = _recenter(track, specs, closed)
    return track


def build_from(start: Pose, specs: Sequence[SegmentSpec],
               closed: bool = False) -> RoadTrack:
    """Build an **open** segment chain starting at ``start`` (parking arcs)."""
    return _build(specs, closed, start=start)


def _build(specs: Sequence[SegmentSpec], closed: bool,
           start: Optional[Pose] = None) -> RoadTrack:
    built: List[BuiltSegment] = []
    if start is None:
        x = y = z = 0.0
        yaw = 0.0
        s = 0.0
    else:
        x, y, z, yaw, s = start.x, start.y, start.z, start.yaw, 0.0
    for spec in specs:
        builder = _BUILDERS.get(spec.kind)
        if builder is None:
            raise ValueError(f"{spec.name}: unknown segment kind {spec.kind!r}")
        segment_start = Pose(x=x, y=y, z=z, yaw=yaw, s=s)
        segment = builder(spec, segment_start)
        built.append(segment)
        end = segment.end
        x, y, z, yaw, s = end.x, end.y, end.z, end.yaw, end.s
    return RoadTrack(segments=tuple(built), closed=bool(closed))


def _recenter(track: RoadTrack, specs: Sequence[SegmentSpec], closed: bool) -> RoadTrack:
    min_x, max_x, min_y, max_y, _, _ = track.bounds()
    offset_x = -0.5 * (min_x + max_x)
    offset_y = -0.5 * (min_y + max_y)
    if abs(offset_x) < 1e-12 and abs(offset_y) < 1e-12:
        return track
    shifted = tuple(
        SegmentSpec(name=spec.name, kind=spec.kind, length=spec.length,
                    radius=spec.radius, angle_deg=spec.angle_deg, rise=spec.rise,
                    amplitude_deg=spec.amplitude_deg, cycles=spec.cycles)
        for spec in specs)
    rebuilt = _build(shifted, closed)
    # shift every built start (and thus every evaluated pose) by the offset
    moved: List[BuiltSegment] = []
    for segment in rebuilt.segments:
        start = replace(segment.start, x=segment.start.x + offset_x,
                        y=segment.start.y + offset_y)
        moved.append(_rebuild_with_start(segment, start))
    return RoadTrack(segments=tuple(moved), closed=rebuilt.closed)


def _rebuild_with_start(segment: BuiltSegment, start: Pose) -> BuiltSegment:
    builder = _BUILDERS[segment.spec.kind]
    return builder(segment.spec, start)


# ---------------------------------------------------------------------------
# the default template
# ---------------------------------------------------------------------------
#: names the default loop uses, so the panel can offer a per-segment drive
DEFAULT_SEGMENT_NAMES: Tuple[str, ...] = (
    "straight_a", "curve_0", "approach_up", "ramp_up", "crest", "curve_1",
    "straight_b", "curve_2", "approach_down", "ramp_down", "valley", "curve_3",
)


def default_track_specs(straight: float = DEFAULT_STRAIGHT_M,
                        radius: float = DEFAULT_RADIUS_M,
                        ramp_rise: float = DEFAULT_RAMP_RISE_M,
                        ramp_length: float = DEFAULT_RAMP_LENGTH_M,
                        ) -> List[SegmentSpec]:
    """A closed rounded rectangle with one up and one down ramp.

    Two opposite straights of ``straight`` m, joined by four 90 deg curves of
    ``radius`` m.  The other two straights carry the ramps, so the loop climbs
    ``ramp_rise`` on one side and comes back down on the other; the straights
    between the curves are always the same total length, which is what makes the
    loop close for any radius and straight length.
    """
    straight = max(1.0, float(straight))
    radius = max(1.0, float(radius))
    ramp_rise = float(ramp_rise)
    ramp_length = max(0.5, float(ramp_length))
    flat = max(0.1, (straight - ramp_length) / 2.0)
    # the two straight runs between curves must match for the loop to close
    return [
        SegmentSpec("straight_a", STRAIGHT, straight),
        SegmentSpec("curve_0", CURVE, 0.0, radius=radius, angle_deg=90.0),
        SegmentSpec("approach_up", STRAIGHT, flat),
        SegmentSpec("ramp_up", RAMP, ramp_length, rise=abs(ramp_rise)),
        SegmentSpec("crest", STRAIGHT, flat),
        SegmentSpec("curve_1", CURVE, 0.0, radius=radius, angle_deg=90.0),
        SegmentSpec("straight_b", STRAIGHT, straight),
        SegmentSpec("curve_2", CURVE, 0.0, radius=radius, angle_deg=90.0),
        SegmentSpec("approach_down", STRAIGHT, flat),
        SegmentSpec("ramp_down", RAMP, ramp_length, rise=-abs(ramp_rise)),
        SegmentSpec("valley", STRAIGHT, flat),
        SegmentSpec("curve_3", CURVE, 0.0, radius=radius, angle_deg=90.0),
    ]


def default_track(straight: float = DEFAULT_STRAIGHT_M,
                  radius: float = DEFAULT_RADIUS_M,
                  ramp_rise: float = DEFAULT_RAMP_RISE_M,
                  ramp_length: float = DEFAULT_RAMP_LENGTH_M,
                  center: bool = True) -> RoadTrack:
    """The default closed loop, centred on the origin in x/y."""
    return build_track(
        default_track_specs(straight, radius, ramp_rise, ramp_length),
        closed=True, center=center)


def preset_specs(preset: TrackPreset) -> List[SegmentSpec]:
    """The segment list of a named preset."""
    return default_track_specs(preset.straight, preset.radius,
                               preset.ramp_rise, preset.ramp_length)


def preset_track(key: str = "compact", center: bool = True) -> RoadTrack:
    """The named preset loop (``compact`` / ``full``), centred in x/y."""
    preset = TRACK_PRESETS.get(str(key))
    if preset is None:
        raise ValueError(f"unknown track preset {key!r}; have {tuple(TRACK_PRESETS)}")
    return build_track(preset_specs(preset), closed=True, center=center)


# ---------------------------------------------------------------------------
# the zebra crossing (geometry - shared by the scene's paint and the speed field)
# ---------------------------------------------------------------------------
def crosswalk_distance(track: "RoadTrack") -> Optional[float]:
    """Arc length of the zebra crossing: just before the road starts to climb.

    ``None`` when the template has no up ramp, in which case there is nothing to
    put a crossing "before".  Pure geometry, so the scene that paints the bars
    and the model that slows the car for them read the same rule.
    """
    for segment in track.segments:
        if segment.spec.kind == RAMP and segment.spec.rise > 0.0:
            return max(CROSSWALK_DEPTH / 2.0,
                       segment.start.s - CROSSWALK_DEPTH / 2.0 - CROSSWALK_CLEAR)
    return None


def crosswalk_slow_zone(track: "RoadTrack",
                        half: float = CROSSWALK_SLOW_HALF_M) -> Optional[Tuple[float, float]]:
    """The ``(start, end)`` arc lengths a driver slows over around the crossing."""
    centre = crosswalk_distance(track)
    if centre is None:
        return None
    return (centre - float(half), centre + float(half))


# ---------------------------------------------------------------------------
# the parking bay / pull-out arc (geometry of the reverse-parking manoeuvre)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ParkingArc:
    """A perpendicular bay beside the road and the quarter-arc in/out of it.

    ``bay`` is where the car sits (nose-out, facing the road); ``arc`` is the
    quarter circle from the bay to ``entry`` on the centreline, driven forward
    when leaving and **backwards** when parking.  Both directions keep the nose
    along the arc's increasing-parameter tangent, so the car ends nose-out again.
    """

    bay: Pose
    arc: RoadTrack
    entry: Pose
    radius: float

    @property
    def length(self) -> float:
        return self.arc.length


def parking_arc(track: "RoadTrack", s_entry: float, radius: float) -> ParkingArc:
    """The bay + pull-out arc for a perpendicular bay right of the road."""
    radius = max(0.5, float(radius))
    entry = track.pose_at(s_entry, wrap=False)
    angle = math.radians(entry.yaw)
    right = (math.cos(angle), math.sin(angle))
    forward = (-math.sin(angle), math.cos(angle))
    bay = Pose(
        x=entry.x - radius * forward[0] + radius * right[0],
        y=entry.y - radius * forward[1] + radius * right[1],
        z=entry.z, yaw=wrap_deg(entry.yaw + 90.0), pitch=0.0)
    arc = build_from(bay, [SegmentSpec(
        "parking", CURVE, 0.0, radius=-radius, angle_deg=-90.0)])
    return ParkingArc(bay=bay, arc=arc, entry=entry, radius=radius)
