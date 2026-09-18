"""AVM Scene coverage evaluation.

Coverage answers the question the scene exists for: *is the field good enough?*
Four ground footprints, the union / overlap / blind areas, and a block-by-camera
visibility matrix (``docs/avm-scene.md`` §16).  The maths lives in
:mod:`core.scenes.avm_coverage`; here we cache the numbers for the panel and draw
the footprint curves in the viewport.
"""

from __future__ import annotations

from typing import Optional

import bpy

from ....core.scenes import avm_coverage
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
    builder.apply_visibility(settings)
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

