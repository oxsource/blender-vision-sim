#!/usr/bin/env python3
"""Validate a vehicle GLB against its ``vehicle_model.json`` (M2 contract).

    python3 tests/check_vehicle_glb.py vehicle.glb vehicle_model.json

Pure Python: it reads the GLB JSON chunk and the manifest, so it runs in CI and
in the core repo without Blender.  The checks mirror ``model_spec.md`` §5.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
from typing import Dict, List, Optional, Tuple

#: the frozen material slots plus the fixed glass material
SLOT_NAMES = {
    "paint_body", "paint_door", "trim_door_inner", "paint_mirror",
    "tire", "rim", "lamp_lens_front", "lamp_lens_rear", "glass",
}

#: tolerance for pivot translations [m]
PIVOT_TOL = 1e-3
#: triangle budget for the skeleton (M2 §5)
TRI_BUDGET = 2000


def read_glb(path: str) -> Dict:
    """The JSON chunk of a GLB file."""
    blob = open(path, "rb").read()
    magic, version, size = struct.unpack_from("<III", blob, 0)
    if magic != 0x46546C67:
        raise ValueError(f"{path}: not a GLB (bad magic)")
    offset, document = 12, None
    while offset < size:
        chunk_len, = struct.unpack_from("<I", blob, offset)
        tag = blob[offset + 4:offset + 8]
        if tag == b"JSON":
            document = json.loads(blob[offset + 8:offset + 8 + chunk_len].decode("utf-8"))
        offset += 8 + chunk_len
    if document is None:
        raise ValueError(f"{path}: no JSON chunk")
    return document


def _nodes(document: Dict) -> Dict[str, Dict]:
    return {node.get("name", ""): node for node in document.get("nodes", [])}


def _parents(document: Dict) -> Dict[str, str]:
    parents: Dict[str, str] = {}
    nodes = document.get("nodes", [])
    for node in nodes:
        for child in node.get("children", []):
            parents[nodes[child].get("name", "")] = node.get("name", "")
    return parents


def _translation(node: Dict) -> List[float]:
    value = node.get("translation")
    return [float(v) for v in value] if value else [0.0, 0.0, 0.0]


def _rotation_ok(node: Dict) -> bool:
    value = node.get("rotation")
    if not value:
        return True
    x, y, z, w = (float(v) for v in value)
    return abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6 and abs(abs(w) - 1.0) < 1e-6


def _scale_ok(node: Dict) -> bool:
    value = node.get("scale")
    return not value or all(abs(float(v) - 1.0) < 1e-6 for v in value)



def _material_name(document: Dict, primitive: Dict) -> str:
    index = primitive.get("material")
    if index is None:
        return ""
    return document.get("materials", [])[index].get("name", "")


def _world_translation(name: str, nodes: Dict[str, Dict],
                       parents: Dict[str, str]) -> List[float]:
    total = [0.0, 0.0, 0.0]
    current: Optional[str] = name
    while current:
        offset = _translation(nodes[current])
        total = [a + b for a, b in zip(total, offset)]
        current = parents.get(current)
    return total


def _bbox(document: Dict, nodes: Dict[str, Dict], parents: Dict[str, str],
          part_nodes: List[str]) -> Optional[Tuple[List[float], List[float]]]:
    low = [math.inf] * 3
    high = [-math.inf] * 3
    found = False
    for name in part_nodes:
        node = nodes.get(name, {})
        mesh_index = node.get("mesh")
        if mesh_index is None:
            continue
        base = _world_translation(name, nodes, parents)
        for primitive in document["meshes"][mesh_index].get("primitives", []):
            accessor = document["accessors"][primitive["attributes"]["POSITION"]]
            for axis in range(3):
                low[axis] = min(low[axis], base[axis] + accessor["min"][axis])
                high[axis] = max(high[axis], base[axis] + accessor["max"][axis])
            found = True
    return (low, high) if found else None


def check(glb_path: str, manifest_path: str) -> List[str]:
    """Return the list of contract violations (empty = pass)."""
    errors: List[str] = []
    document = read_glb(glb_path)
    with open(manifest_path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)

    for key in ("schema_version", "model", "parts", "appearance"):
        if key not in manifest:
            errors.append(f"manifest: missing {key!r}")
    if errors:
        return errors
    model = manifest["model"]
    for key in ("style_id", "frame", "root", "dimensions", "anchors",
                "door_slots", "absent_ok"):
        if key not in model:
            errors.append(f"manifest.model: missing {key!r}")
    if len(model.get("door_slots", [])) != 8:
        errors.append("manifest.model.door_slots: expected 8 entries")

    parts = manifest["parts"]
    ids = [part.get("id") for part in parts]
    if len(ids) != len(set(ids)):
        errors.append("manifest.parts: duplicate ids")
    expected_parts = {part["node"] for part in parts}
    expected_pivots = {part["pivot"] for part in parts if part.get("pivot")}
    if len(expected_parts) != len(parts):
        errors.append("manifest.parts: duplicate node names")

    nodes = _nodes(document)
    parents = _parents(document)
    root = model.get("root", "vehicle")
    if root not in nodes:
        errors.append(f"glb: missing root {root!r}")
    if root in parents:
        errors.append(f"glb: {root!r} must be a scene root")

    for name in expected_parts | expected_pivots:
        if name not in nodes:
            errors.append(f"glb: missing node {name!r}")
    for name in nodes:
        if (name.startswith("part_") or name.startswith("pivot_")) \
                and name not in expected_parts | expected_pivots:
            errors.append(f"glb: unexpected node {name!r}")

    # materials: the nine names must exist; only the ones the vehicle parts
    # actually use are required to be unlit (a combined bowl+vehicle GLB also
    # carries the bowl's PBR material).
    material_names = {material.get("name", "")
                      for material in document.get("materials", [])}
    missing = SLOT_NAMES - material_names
    if missing:
        errors.append(f"glb: missing materials {sorted(missing)}")

    part_names = []
    part_material_names = set()
    triangles = 0
    for part in parts:
        name = part["node"]
        if name not in nodes:
            continue
        node = nodes[name]
        if "mesh" not in node:
            errors.append(f"{name}: no mesh")
            continue
        part_names.append(name)
        if parents.get(name) != part.get("parent"):
            errors.append(f"{name}: parent {parents.get(name)!r} != {part.get('parent')!r}")
        if not _rotation_ok(node) or not _scale_ok(node):
            errors.append(f"{name}: node transform is not identity/scale 1")
        allowed = set(part.get("slots", [])) | {"glass"}
        for primitive in document["meshes"][node["mesh"]].get("primitives", []):
            material = _material_name(document, primitive)
            if material:
                part_material_names.add(material)
            if material not in allowed:
                errors.append(f"{name}: material {material!r} not in {sorted(allowed)}")
            index = primitive.get("indices")
            if index is None:
                count = document["accessors"][primitive["attributes"]["POSITION"]]["count"]
                triangles += count // 3
            else:
                triangles += document["accessors"][index]["count"] // 3
            if "TEXCOORD_0" not in primitive.get("attributes", {}):
                errors.append(f"{name}: missing TEXCOORD_0")
            else:
                uv = document["accessors"][primitive["attributes"]["TEXCOORD_0"]]
                if "min" in uv and "max" in uv:
                    if min(uv["min"]) < -1e-6 or max(uv["max"]) > 1.0 + 1e-6:
                        errors.append(f"{name}: UV outside 0..1")
        pivot_name = part.get("pivot")
        if pivot_name:
            pivot = nodes.get(pivot_name)
            if pivot is None:
                errors.append(f"{name}: missing pivot {pivot_name!r}")
            else:
                if "mesh" in pivot:
                    errors.append(f"{pivot_name}: pivot must be an empty")
                if parents.get(pivot_name) != root:
                    errors.append(f"{pivot_name}: parent {parents.get(pivot_name)!r} != {root!r}")
                expected = part.get("motion", {}).get("pivot_pos")
                actual = _translation(pivot)
                if expected is None or any(
                        abs(a - b) > PIVOT_TOL for a, b in zip(actual, expected)):
                    errors.append(f"{pivot_name}: translation {actual} != {expected}")
                if not _rotation_ok(pivot) or not _scale_ok(pivot):
                    errors.append(f"{pivot_name}: pivot transform is not identity/scale 1")

    for material in document.get("materials", []):
        if material.get("name", "") not in part_material_names:
            continue
        name = material.get("name", "")
        if "KHR_materials_unlit" not in material.get("extensions", {}):
            errors.append(f"material {name!r}: not KHR_materials_unlit")
        alpha = material.get("pbrMetallicRoughness", {}).get(
            "baseColorFactor", [1.0, 1.0, 1.0, 1.0])[3]
        if abs(float(alpha) - 1.0) > 1e-6:
            errors.append(f"material {name!r}: baseColor alpha is not 1")

    if document.get("animations"):
        errors.append("glb: animations must be empty (Q8 code-driven)")
    if document.get("skins"):
        errors.append("glb: skins must be empty")
    if triangles > TRI_BUDGET:
        errors.append(f"glb: {triangles} triangles > budget {TRI_BUDGET}")

    bbox = _bbox(document, nodes, parents, part_names)
    if bbox is None:
        errors.append("glb: no part geometry")
    else:
        low, high = bbox
        dimensions = model["dimensions"]
        width, height, length = dimensions["width"], dimensions["height"], dimensions["length"]
        if not (width - 0.02 <= high[0] - low[0] <= width + 0.40):
            errors.append(f"glb: width {high[0] - low[0]:.3f} vs {width}")
        if not (height - 0.02 <= high[1] - low[1] <= height + 0.02):
            errors.append(f"glb: height {high[1] - low[1]:.3f} vs {height}")
        if not (length - 0.02 <= high[2] - low[2] <= length + 0.10):
            errors.append(f"glb: length {high[2] - low[2]:.3f} vs {length}")

    return errors


def default_manifest(glb_path: str) -> str:
    return os.path.join(os.path.dirname(os.path.abspath(glb_path)), "vehicle_model.json")


def main(argv: List[str]) -> int:
    if len(argv) not in (2, 3):
        print(__doc__)
        return 2
    glb_path = argv[1]
    manifest_path = argv[2] if len(argv) == 3 else default_manifest(glb_path)
    try:
        errors = check(glb_path, manifest_path)
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: {type(exc).__name__}: {exc}")
        return 1
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: vehicle GLB matches the M2 contract")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
