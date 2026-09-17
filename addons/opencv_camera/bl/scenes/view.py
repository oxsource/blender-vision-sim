"""Default 3D-view framing for a freshly built scene.

A scene operator should leave the viewport *showing what it just built*.
Blender has no call for "frame these objects from this angle" that fits here:
``view_selected`` needs the objects **selected** (the Camera Scene deliberately
keeps its camera selected so its panel operators stay valid) and ``view_all``
frames the whole scene rather than the scene we just built.  So this module
computes the bounding sphere itself and writes ``region_3d`` directly - no
selection change, no operator context, nothing written into the file.

The default orbit is the industry **3/4 view: 45 deg azimuth, 30 deg elevation**.
Blender's own *User Perspective* start view sits at ~45 / 26.6 and true
isometric at 45 / 35.26, so 45 azimuth with a 26-35 elevation is the convention
across CAD, DCC and engines: it shows two side faces *and* the top at once,
while 0 deg (flat elevation) and 90 deg (plan view) do not read as a solid.

Azimuth follows Blender's numpad: 0 = front (viewer on -Y), 90 = right (viewer
on +X), 180 = back.  The default :data:`DEFAULT_AZIMUTH` is 135, i.e. the
three-quarter view seen from the subject's **front-right-above** - 45 deg off
both the +X and the +Y axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import bpy
from bpy.props import StringProperty
from mathutils import Euler, Vector

__all__ = [
    "ViewSpec",
    "DEFAULT_AZIMUTH", "DEFAULT_ELEVATION", "DEFAULT_MARGIN",
    "orbit_offset", "corners", "bounds", "view_areas", "frame",
    "scene_objects", "targets",
]

#: azimuth of the standard 3/4 view (degrees, Blender numpad convention)
DEFAULT_AZIMUTH = 135.0
#: elevation of the standard 3/4 view (degrees above the horizon)
DEFAULT_ELEVATION = 30.0
#: fraction of empty space left around the subject
DEFAULT_MARGIN = 1.1


@dataclass(frozen=True)
class ViewSpec:
    """The orbit a scene operator frames after building."""

    azimuth: float = DEFAULT_AZIMUTH      #: degrees, 0 = front, 90 = right
    elevation: float = DEFAULT_ELEVATION  #: degrees above the horizon
    margin: float = DEFAULT_MARGIN        #: > 1 leaves breathing room


# ---------------------------------------------------------------------------
# maths (pure: no bpy, so the tests can check it without a viewport)
# ---------------------------------------------------------------------------
def orbit_offset(azimuth: float, elevation: float) -> Vector:
    """Unit vector pointing from the subject towards the viewer.

    The inverse of the view direction, so the tests can assert "the viewer is in
    the front-right octant" without reading a ``region_3d``.
    """
    az = math.radians(azimuth)
    el = math.radians(elevation)
    return Vector((math.cos(el) * math.sin(az),
                   -math.cos(el) * math.cos(az),
                   math.sin(el)))


def _view_rotation(azimuth: float, elevation: float):
    """The ``region_3d.view_rotation`` of an orbit view.

    Blender's own axis views are the reference: top is ``(0, 0, 0)``, front is
    ``(90, 0, 0)`` and right is ``(90, 0, 90)``, so an orbit at ``elevation``
    above the horizon and ``azimuth`` around the vertical is
    ``Euler(90 - elevation, 0, azimuth)``.  ``R @ (0, 0, 1)`` then equals
    :func:`orbit_offset`, which is exactly what ``view_location`` +
    ``view_distance`` need.
    """
    return Euler((math.radians(90.0 - elevation), 0.0,
                  math.radians(azimuth)), "XYZ").to_quaternion()


def _frustum(region_3d) -> Tuple[float, float, bool]:
    """``(k_h, k_v, perspective)``: ``NDC = (view-space x, y) * k / depth``.

    Read off the viewport's own projection instead of computed from
    ``space.lens`` and a "36 mm sensor": how Blender maps the sensor onto the
    region is version dependent (and the viewport has no ``sensor_width`` of its
    own).  The projection matrix rows are ``k * (unit view axis)``, so their
    lengths are exactly the two scales - and they depend only on the lens and
    the region size, never on the view transform, so a stale matrix is still
    correct here.

    A perspective viewport uses ``depth = view_distance - z``; an orthographic
    one folds ``1 / view_distance`` into the scales and uses ``depth =
    view_distance``, so those are multiplied back up.  Both the mode and the
    scales come from the matrix (its bottom row is affine for ortho), never from
    ``view_perspective`` / ``is_perspective``: those only catch up on the next
    redraw, which would pair a stale matrix with a fresh mode and fit wrongly.
    """
    matrix = region_3d.perspective_matrix
    k_h = Vector(matrix[0][0:3]).length
    k_v = Vector(matrix[1][0:3]).length
    perspective = max(abs(matrix[3][axis]) for axis in range(3)) > 1e-6
    if not perspective:
        k_h *= region_3d.view_distance
        k_v *= region_3d.view_distance
    return k_h, k_v, perspective


def _view_space(points, centre: Vector, rotation) -> List[Vector]:
    """The corner points in view space: +X right, +Y up, +Z towards the viewer."""
    inverse = rotation.conjugated()
    return [inverse @ (point - centre) for point in points]


def _fit_distance(points, centre: Vector, rotation, frustum, perspective: bool,
                  margin: float) -> float:
    """``view_distance`` at which every corner is inside the frustum.

    For a corner at view-space ``(x, y, z)`` the frustum gives ``|x| * k_h <=
    depth``, so ``distance >= z + |x| * k_h`` (perspective) or ``distance >=
    |x| * k_h`` (orthographic, which has no depth term).  The maximum over the
    corners is exact - tighter (and no slower) than fitting a bounding sphere.
    """
    k_h, k_v = frustum
    distance = 0.0
    for q in _view_space(points, centre, rotation):
        x = abs(q.x) * k_h
        y = abs(q.y) * k_v
        if perspective:
            distance = max(distance, q.z + x, q.z + y)
        else:
            distance = max(distance, x, y)
    return max(distance, 1e-3) * margin


# ---------------------------------------------------------------------------
# scene contents
# ---------------------------------------------------------------------------
def corners(objects: Iterable[bpy.types.Object]) -> List[Vector]:
    """World-space corners of every object's local bounding box.

    ``bound_box`` is local, so every corner goes through ``matrix_world``.
    Empties, lights and cameras have a single-point box, which is fine: they are
    part of what should be visible.
    """
    points: List[Vector] = []
    for obj in objects:
        if obj is None:
            continue
        matrix = obj.matrix_world
        points.extend(matrix @ Vector(corner) for corner in obj.bound_box)
    return points


def bounds(objects: Iterable[bpy.types.Object]) -> Optional[Tuple[Vector, float]]:
    """World-space bounding sphere ``(centre, radius)`` of ``objects``.

    ``None`` when there is nothing to measure.  :func:`frame` fits the corners
    exactly; the sphere is the coarse answer (and what the tests assert on).
    """
    points = corners(objects)
    if not points:
        return None
    lo = Vector((min(p[axis] for p in points) for axis in range(3)))
    hi = Vector((max(p[axis] for p in points) for axis in range(3)))
    centre = (lo + hi) * 0.5
    return centre, max((point - centre).length for point in points)


def scene_objects(definition) -> List[bpy.types.Object]:
    """Every object of a registered scene's collection (empty when not built)."""
    name = getattr(definition, "collection_name", "")
    found = bpy.data.collections.get(name) if name else None
    return list(found.objects) if found is not None else []


def targets(definition, scene=None) -> List[bpy.types.Object]:
    """The objects a scene's default view should frame.

    A scene can narrow this down through ``SceneDefinition.view_targets``: the
    AVM scene frames the calibration field, not the 30 m ground or the props
    scattered 1.7 m outside it, because those would shrink the field to a few
    percent of the viewport.  Scenes without a hook get their whole collection.
    """
    hook = getattr(definition, "view_targets", None)
    scene = scene or getattr(bpy.context, "scene", None)
    if hook is not None and scene is not None:
        return list(hook(scene))
    return scene_objects(definition)


# ---------------------------------------------------------------------------
# the viewport write
# ---------------------------------------------------------------------------
def view_areas():
    """Yield ``(area, region, space, region_3d)`` for every usable 3D viewport."""
    window_manager = getattr(bpy.context, "window_manager", None)
    for window in getattr(window_manager, "windows", ()) or ():
        screen = window.screen
        if screen is None:
            continue
        for area in screen.areas:
            if area.type != "VIEW_3D":
                continue
            space = area.spaces.active
            region_3d = getattr(space, "region_3d", None)
            region = next((r for r in area.regions if r.type == "WINDOW"), None)
            if region_3d is None or region is None:
                continue
            yield area, region, space, region_3d


def frame(objects: Iterable[bpy.types.Object], spec: Optional[ViewSpec] = None) -> int:
    """Point every 3D viewport at ``objects`` from ``spec``'s orbit.

    Returns how many viewports were moved.  **0 is not an error**: an operator
    run from the background or from a context without a 3D view simply has
    nothing to move, and callers must not treat that as a failure.  A viewport
    whose rotation the user locked (``region_3d.lock_rotation``) is left alone.
    """
    spec = spec or ViewSpec()
    points = corners(objects)
    if not points:
        return 0
    lo = Vector((min(p[axis] for p in points) for axis in range(3)))
    hi = Vector((max(p[axis] for p in points) for axis in range(3)))
    centre = (lo + hi) * 0.5
    rotation = _view_rotation(spec.azimuth, spec.elevation)

    moved = 0
    for area, region, space, region_3d in view_areas():
        if region_3d.lock_rotation or region_3d.view_perspective == "CAMERA":
            # CAMERA view is driven by the render camera, not by us
            continue
        # read the frustum first: perspective_matrix goes stale once we write
        k_h, k_v, perspective = _frustum(region_3d)
        region_3d.view_location = centre
        region_3d.view_rotation = rotation
        # the same knob fits both projections: an ortho viewport zooms by
        # view_distance too, it just has no depth term (see _frustum)
        region_3d.view_distance = _fit_distance(points, centre, rotation,
                                                (k_h, k_v), perspective, spec.margin)
        area.tag_redraw()
        moved += 1
    return moved


# ---------------------------------------------------------------------------
# operator: back to the default view after the user has orbited away
# ---------------------------------------------------------------------------
class OPENCV_CAM_OT_frame_view(bpy.types.Operator):
    """Frame the scene from its default 3/4 angle (45 deg azimuth, 30 deg elevation)"""

    bl_idname = "opencv_cam.frame_view"
    bl_label = "Frame View"
    bl_description = (
        "Point the 3D viewport at the scene from its default three-quarter "
        "angle (45 deg azimuth, 30 deg elevation), fitted to the objects"
    )
    bl_options = {"REGISTER", "UNDO"}

    scene_id: StringProperty(
        name="Scene", default="", options={"HIDDEN"},
        description="Registered scene to frame; empty frames every built scene")

    @classmethod
    def poll(cls, context):
        return getattr(context, "scene", None) is not None

    def execute(self, context):
        # imported here: ``bl.scenes`` imports this module for ViewSpec
        from . import definition as scene_definition, definitions as scene_definitions

        wanted = ([scene_definition(self.scene_id)] if self.scene_id
                  else list(scene_definitions()))
        framed = 0
        for definition in wanted:
            if definition is None:
                continue
            objects = targets(definition)
            if objects:
                framed += frame(objects, definition.view)
        if not framed:
            self.report({"WARNING"}, "nothing to frame (nothing built, or no 3D viewport)")
            return {"CANCELLED"}
        return {"FINISHED"}


_CLASSES = (OPENCV_CAM_OT_frame_view,)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
