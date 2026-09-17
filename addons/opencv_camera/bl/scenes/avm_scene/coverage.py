"""AVM Scene coverage evaluation and raw-material export.

Coverage answers the question the scene exists for: *is the field good enough?*
Four ground footprints, the union / overlap / blind areas, and a block-by-camera
visibility matrix (``docs/avm-scene.md`` §16).  The maths lives in
:mod:`core.scenes.avm_coverage`; here we cache the numbers for the panel and draw
the footprint curves in the viewport.

``export_materials`` writes the "no real vehicle" material set: the four camera
images plus the scene parameters and a coverage report.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

import bpy

from ....core.scenes import avm_coverage, avm_layout
from . import builder, io as io_mod

CURVE_PREFIX = "AVM_Coverage_"
CURVE_LIFT = 0.002  #: metres above the ground

#: one colour per camera
CAMERA_COLORS = {
    "front": (0.20, 0.55, 0.95, 1.0),
    "back": (0.95, 0.45, 0.20, 1.0),
    "left": (0.30, 0.80, 0.40, 1.0),
    "right": (0.85, 0.75, 0.20, 1.0),
}


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------
def analyze(settings, step: float = 0.05, samples: int = 256) -> avm_coverage.CoverageReport:
    """Compute the coverage report and cache a summary on the settings."""
    cameras = io_mod.camera_records(settings)
    report = avm_coverage.coverage_report(
        cameras, settings.field_spec(), samples=samples, step=step)
    settings.coverage_status = (
        f"coverage {report.coverage_ratio * 100:.1f}%  |  "
        f"blind {report.blind_area:.2f} m2  |  overlap {report.overlap_area:.2f} m2  |  "
        + ("field OK" if report.field_ok else "field NOT sufficient"))
    settings.coverage_matrix = _matrix_text(report)
    return report


def _matrix_text(report: avm_coverage.CoverageReport) -> str:
    parts = []
    for block, seen in report.visibility.items():
        cameras = ", ".join(name for name, ok in seen.items() if ok) or "-"
        parts.append(f"{block}: {cameras}")
    return " | ".join(parts)


def ensure_curves(scene: bpy.types.Scene, settings,
                  report: Optional[avm_coverage.CoverageReport] = None):
    """Create/update one ground curve per camera footprint."""
    if report is None:
        report = analyze(settings)
    target = bpy.data.collections.get(builder.COLLECTION_NAME)
    root = settings.root
    curves = []
    for footprint in report.footprints:
        name = f"{CURVE_PREFIX}{builder.CAMERA_SUFFIX.get(footprint.camera, footprint.camera)}"
        obj = bpy.data.objects.get(name)
        if obj is None:
            obj = bpy.data.objects.new(name, bpy.data.curves.new(name, type="CURVE"))
            obj.data.dimensions = "3D"
            obj.data.bevel_depth = 0.01
            if target is not None:
                target.objects.link(obj)
        obj.data.materials.clear()
        obj.data.materials.append(_line_material(footprint.camera))
        _fill_spline(obj.data, footprint)
        if root is not None and obj.parent is not root:
            obj.parent = root
        curves.append(obj)
    apply_visibility(settings)
    return curves


def _line_material(camera: str) -> bpy.types.Material:
    name = f"AVM_Coverage_{camera}_Mat"
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
    material.use_nodes = True
    color = CAMERA_COLORS.get(camera, (1.0, 1.0, 1.0, 1.0))
    material.diffuse_color = color  # viewport solid shading
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    emission = nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = color
    emission.inputs["Strength"].default_value = 1.5
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


def _fill_spline(curve: bpy.types.Curve, footprint: avm_coverage.Footprint) -> None:
    curve.splines.clear()
    if len(footprint.points) < 2:
        return
    spline = curve.splines.new("POLY")
    spline.points.add(len(footprint.points) - 1)
    for index, (x, y) in enumerate(footprint.points):
        spline.points[index].co = (x, y, CURVE_LIFT, 1.0)
    spline.use_cyclic_u = bool(footprint.closed)


def apply_visibility(settings) -> None:
    """Show/hide the coverage curves with the layer toggle."""
    for name in avm_layout.CAMERAS:
        obj = bpy.data.objects.get(f"{CURVE_PREFIX}{builder.CAMERA_SUFFIX[name]}")
        if obj is None:
            continue
        hidden = not settings.show_coverage
        obj.hide_render = hidden
        obj.hide_set(hidden)


# ---------------------------------------------------------------------------
# material export
# ---------------------------------------------------------------------------
def export_materials(context, settings, directory: str, samples: int = 64,
                     name: str = "avm_scene") -> List[str]:
    """Render the four cameras and write the parameters + coverage report."""
    os.makedirs(directory, exist_ok=True)
    written: List[str] = []

    written += io_mod.render_cameras(context, settings, directory, samples=samples)

    plane_path = os.path.join(directory, "plane_scene.json")
    written.append(io_mod.write(plane_path, io_mod.to_plane(settings)))

    scene_path = os.path.join(directory, "avm_scene.json")
    written.append(io_mod.write(scene_path, io_mod.to_full(settings)))

    report = analyze(settings)
    coverage_path = os.path.join(directory, "coverage.json")
    written.append(_write_json(coverage_path, _report_dict(report)))

    filament_path = os.path.join(directory, f"vehicle_{name}.json")
    written.append(_write_json(filament_path, _filament_skeleton(settings, name)))

    spec_path = os.path.join(directory, "scene_spec.md")
    written.append(_write_text(spec_path, _spec_text(settings, report)))

    settings.coverage_status = f"exported {len(written)} files to {directory}"
    return written


def _report_dict(report: avm_coverage.CoverageReport) -> Dict:
    data = report.as_dict()
    data["footprints"] = {
        footprint.camera: {
            "closed": footprint.closed,
            "points": [[round(x, 4), round(y, 4)] for x, y in footprint.points],
        }
        for footprint in report.footprints
    }
    return data


def _filament_skeleton(settings, name: str) -> Dict:
    """A ``vehicle_avm_*.json`` skeleton: ``points_3d`` filled, ``points_2d`` left
    empty for the external corner detector (decision #13)."""
    field = settings.field_spec()
    geo = avm_layout.geometry(field)
    cameras = []
    for record in io_mod.camera_records(settings):
        fx, fy, cx, cy = record["K"]
        cameras.append({
            "name": record["name"],
            "points_2d": [],
            "points_3d": avm_layout.points(record["name"], field),
            "K": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            "D": record["D"],
            "input_size": _output_size(settings, record["name"]),
            "enable": record["enable"],
            "ba_opt": False,
        })
    return {
        "comment": "# AUTO GENERATED BY VISION SIM AVM SCENE (points_2d to be filled "
                   "by the external corner detector)",
        "name": name,
        "cameras": cameras,
        "bev_coord": [-geo.half_x, geo.half_x, -geo.half_y, geo.half_y],
    }


def _output_size(settings, camera_name: str):
    camera = bpy.data.objects.get(
        f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(camera_name, '')}")
    if camera is None:
        return [1280, 960]
    intrinsics = camera.data.opencv_cam.intrinsics
    return [int(intrinsics.image_width), int(intrinsics.image_height)]


def _spec_text(settings, report: avm_coverage.CoverageReport) -> str:
    field = settings.field_spec()
    width, height = avm_layout.scene_size(field)
    lines = [
        "# AVM Scene specification",
        "",
        "## Field (cm)",
        "",
        f"- border: {field.border_w:g} x {field.border_h:g}",
        f"- corner: {field.corner:g}",
        f"- inner: {field.inner_w:g} x {field.inner_h:g}",
        f"- core (car): {field.core_w:g} x {field.core_h:g}",
        f"- scene: {width:g} x {height:g} cm ({width / 100:.2f} x {height / 100:.2f} m)",
        "",
        "## Cameras",
        "",
        "| camera | location [m] | rotation [deg] |",
        "| --- | --- | --- |",
    ]
    for record in io_mod.camera_records(settings):
        location = ", ".join(f"{value:+.3f}" for value in record["location"])
        rotation = ", ".join(f"{value:+.2f}" for value in record["rotation"])
        lines.append(f"| {record['name']} | {location} | {rotation} |")
    lines += [
        "",
        "## Coverage",
        "",
        f"- coverage: {report.coverage_ratio * 100:.1f}% of the field",
        f"- blind: {report.blind_area:.2f} m2",
        f"- overlap (>=2 cameras): {report.overlap_area:.2f} m2",
        f"- field sufficient: {'yes' if report.field_ok else 'no'}",
        f"- open footprints (view passes the horizon): "
        f"{', '.join(report.open_polygons) or 'none'}",
        "",
        "### Block visibility",
        "",
        "| block | cameras |",
        "| --- | --- |",
    ]
    for block, seen in report.visibility.items():
        cameras = ", ".join(camera for camera, ok in seen.items() if ok) or "-"
        lines.append(f"| {block} | {cameras} |")
    lines.append("")
    return "\n".join(lines)


def _write_json(path: str, data: Dict) -> str:
    return _write_text(path, json.dumps(data, indent=2) + "\n")


def _write_text(path: str, text: str) -> str:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return path
