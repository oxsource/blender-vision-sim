#!/usr/bin/env python3
"""The version-adaptation policy (no Blender, runs in CI).

Every Blender API difference between the supported versions must live in
``bl/compat.py`` - the global mechanism described in ``docs/architecture.md``
§2.3.  This test walks the add-on sources and fails when any *other* module
reaches for an API that changed across releases, so the rule is enforced instead
of merely documented.

    python3 tests/test_version_policy.py
"""

from __future__ import annotations

import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADDONS = os.path.join(ROOT, "addons")
COMPAT_REL = os.path.join("opencv_camera", "bl", "compat.py")

#: attribute names that changed across releases -> the shim to use instead
FORBIDDEN_ATTRS = {
    "fcurves": "compat.action_fcurves (4.4 turned actions into layers/strips/channelbags)",
    "blend_method": "compat.set_material_blend (renamed to surface_render_method in 4.2)",
    "surface_render_method": "compat.set_material_blend (4.2 rename of blend_method)",
}

#: ``bpy.ops.<module>.<operator>`` that moved -> the shim to use instead
FORBIDDEN_OPERATORS = {
    ("import_scene", "gltf"): "compat.import_model",
    ("import_scene", "fbx"): "compat.import_model",
    ("import_scene", "obj"): "compat.import_model (moved to wm.obj_import in 4.0)",
    ("wm", "obj_import"): "compat.import_model",
    ("export_scene", "gltf"): "compat.export_gltf",
}

#: string literals that are enum identifiers known to have changed
FORBIDDEN_STRINGS = {
    "BLENDER_EEVEE": "compat.eevee_engine",
    "BLENDER_EEVEE_NEXT": "compat.eevee_engine",
}

#: shims every compat.py must define, and the source module that owns them
REQUIRED_SHIMS = (
    "action_fcurves", "eevee_engine", "import_model", "export_gltf",
    "set_material_blend", "node_of_type", "sequence_strips", "enable_movie_output",
)

FAILURES = []


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name}" + (f" - {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def _dotted(node) -> str:
    """``a.b.c`` for an attribute chain rooted at a name, else ``""``."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return ".".join(reversed(parts))
    return ""


def scan_source(text: str, where: str) -> list:
    """Every policy violation in ``text``, as human-readable strings."""
    problems = []
    try:
        tree = ast.parse(text)
    except SyntaxError as exc:
        return [f"{where}: cannot parse: {exc}"]

    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr in FORBIDDEN_ATTRS:
            problems.append(f"{where}:{node.lineno}: .{node.attr} -> {FORBIDDEN_ATTRS[node.attr]}")

        dotted = _dotted(node)
        if dotted == "bpy.app.version":
            problems.append(f"{where}:{node.lineno}: bpy.app.version -> use bl/compat.py")
        parts = dotted.split(".")
        if len(parts) == 4 and parts[:2] == ["bpy", "ops"]:
            key = (parts[2], parts[3])
            if key in FORBIDDEN_OPERATORS:
                problems.append(
                    f"{where}:{node.lineno}: bpy.ops.{parts[2]}.{parts[3]} -> "
                    f"{FORBIDDEN_OPERATORS[key]}")

        if isinstance(node, ast.Subscript):
            base = node.value
            if isinstance(base, ast.Attribute) and base.attr == "nodes":
                if isinstance(node.slice, ast.Constant) and isinstance(node.slice.value, str):
                    problems.append(
                        f"{where}:{node.lineno}: nodes[{node.slice.value!r}] -> "
                        "compat.node_of_type (names are localized)")

        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            func = node.func
            if (func.attr == "get" and isinstance(func.value, ast.Attribute)
                    and func.value.attr == "nodes" and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and isinstance(node.args[0].value, str)):
                problems.append(
                    f"{where}:{node.lineno}: nodes.get({node.args[0].value!r}) -> "
                    "compat.node_of_type (names are localized)")

        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value in FORBIDDEN_STRINGS:
                problems.append(
                    f"{where}:{node.lineno}: {node.value!r} -> {FORBIDDEN_STRINGS[node.value]}")
    return problems


def _python_files():
    for base, dirs, files in os.walk(ADDONS):
        dirs[:] = [d for d in dirs if d != "__pycache__"]
        for name in sorted(files):
            if name.endswith(".py"):
                yield os.path.join(base, name)


def test_addon_sources_obey_the_policy():
    problems = []
    for path in _python_files():
        relative = os.path.relpath(path, ADDONS)
        if relative == COMPAT_REL:
            continue
        with open(path, encoding="utf-8") as handle:
            problems.extend(scan_source(handle.read(), relative))
    check("no version-sensitive API outside bl/compat.py", not problems,
          "; ".join(problems[:4]))


def test_scanner_catches_violations():
    """The policy test is only meaningful if it actually fails on a violation."""
    sample = (
        "import bpy\n"
        "def bad(action, material):\n"
        "    for curve in action.fcurves:\n"
        "        pass\n"
        "    material.blend_method = 'BLEND'\n"
        "    bpy.ops.import_scene.obj(filepath='x.obj')\n"
        "    node = material.node_tree.nodes['Principled BSDF']\n"
        "    node = material.node_tree.nodes.get('Background')\n"
        "    engine = 'BLENDER_EEVEE'\n"
        "    print(bpy.app.version)\n"
    )
    found = scan_source(sample, "<sample>")
    for expected in (".fcurves", ".blend_method", "bpy.ops.import_scene.obj",
                     "nodes[", "nodes.get(", "'BLENDER_EEVEE'", "bpy.app.version"):
        check(f"scanner flags {expected}",
              any(expected in problem for problem in found), f"{len(found)} violations found")


def test_compat_owns_every_shim():
    path = os.path.join(ADDONS, COMPAT_REL)
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read())
    defined = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    missing = [name for name in REQUIRED_SHIMS if name not in defined]
    check("bl/compat.py defines every shim", not missing, str(missing))
    check("bl/compat.py exposes a report()", "report" in defined)


def main() -> int:
    for test in (test_addon_sources_obey_the_policy, test_scanner_catches_violations,
                 test_compat_owns_every_shim):
        test()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} failure(s): {', '.join(FAILURES)}")
        return 1
    print("all version policy tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
