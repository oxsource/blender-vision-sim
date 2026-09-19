"""The add-on's single version-adaptation layer (global mechanism).

The add-on supports Blender 4.5 LTS and 5.x from one code base.  **Every**
Blender API difference between those releases lives here, and only here: no
other module may branch on ``bpy.app.version`` or reference an identifier that
has changed across releases.  ``tests/test_version_policy.py`` enforces that.

Rules:

* prefer **feature detection** (read the RNA / try the new shape and fall back)
  over version-number checks, so a future release that keeps the new shape keeps
  working and only a real change needs a new shim;
* one function per difference, named for the intent (``action_fcurves``) rather
  than the API (``legacy_fcurves``);
* each shim's docstring states which versions it bridges;
* a shim without a test is not done - core tests cover the pure-Python parts,
  ``tests/run_blender_tests.py`` exercises the live ones on the running Blender.
"""

from __future__ import annotations

import os
from typing import Dict, List, Set, Tuple

import bpy

#: the oldest Blender the add-on supports (mirrors ``blender_version_min``)
MIN_VERSION: Tuple[int, int] = (4, 5)
#: the versions the suite is run against (see docs/architecture.md §2.3)
TESTED_VERSIONS: Tuple[Tuple[int, int], ...] = ((4, 5), (5, 2))

#: EEVEE engine identifiers, newest spelling first.  4.2-4.4 call it
#: ``BLENDER_EEVEE_NEXT``, 5.0 renamed it back to ``BLENDER_EEVEE``.
_EEVEE_IDS = ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE")

#: model importers per file extension, newest operator name first.  The OBJ
#: importer moved from ``import_scene.obj`` to ``wm.obj_import`` in 4.0; the
#: others kept their names but are resolved the same way so a rename only has
#: to be reflected here.
_IMPORTERS = {
    ".glb": (("import_scene", "gltf"),),
    ".gltf": (("import_scene", "gltf"),),
    ".fbx": (("import_scene", "fbx"),),
    ".obj": (("wm", "obj_import"), ("import_scene", "obj")),
}

#: registry of the differences this module owns: ``(shim, what changed)``.
#: ``report()`` exposes it so a bug report can say which Blender sees which shim.
SHIMS: Tuple[Tuple[str, str], ...] = (
    ("action_fcurves", "4.4 slotted actions: action.fcurves -> layers/strips/channelbags"),
    ("eevee_engine", "5.0 renamed BLENDER_EEVEE_NEXT back to BLENDER_EEVEE"),
    ("import_model", "4.0 moved the OBJ importer to wm.obj_import"),
    ("export_gltf", "the glTF exporter's options vary between releases"),
    ("set_material_blend", "4.2 replaced Material.blend_method with surface_render_method"),
    ("node_of_type", "shader node names are localized; look nodes up by type"),
    ("sequence_strips", "4.4 renamed SequenceEditor.sequences to .strips"),
    ("enable_movie_output", "4.5 gates FFmpeg output behind ImageFormatSettings.media_type"),
)

__all__ = [
    "MIN_VERSION",
    "TESTED_VERSIONS",
    "SHIMS",
    "blender_version",
    "report",
    "action_fcurves",
    "eevee_engine",
    "import_model",
    "export_gltf",
    "set_material_blend",
    "node_of_type",
    "sequence_strips",
    "enable_movie_output",
]


def blender_version() -> Tuple[int, int, int]:
    """The running Blender as ``(major, minor, patch)``."""
    return tuple(bpy.app.version)


def report() -> List[Dict[str, str]]:
    """Every shim this module owns, plus the running version (for bug reports)."""
    entries = [{"shim": "blender", "bridges": bpy.app.version_string}]
    entries.extend({"shim": name, "bridges": bridges} for name, bridges in SHIMS)
    return entries


def action_fcurves(action) -> List:
    """Every F-Curve of an action, legacy (<= 4.3) or layered (4.4+ slotted).

    Blender 4.4 turned actions into layers of strips of per-slot channelbags and
    removed the old flat ``action.fcurves`` (kept as a compatibility shim through
    4.5, gone in 5.0).  This reads whichever shape the running Blender exposes.
    """
    legacy = getattr(action, "fcurves", None)
    if legacy is not None:
        return list(legacy)
    curves: List = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for channelbag in getattr(strip, "channelbags", ()):
                curves.extend(channelbag.fcurves)
    return curves


def eevee_engine() -> str:
    """The EEVEE engine identifier this Blender accepts.

    Read from RNA so an enum rename cannot break callers; ``BLENDER_EEVEE`` is
    the fallback for builds whose engine enum RNA under-reports.
    """
    valid = {item.identifier
             for item in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items}
    for identifier in _EEVEE_IDS:
        if identifier in valid:
            return identifier
    return _EEVEE_IDS[-1]


def _operator(module_name: str, operator_name: str):
    module = getattr(bpy.ops, module_name, None)
    if module is None:
        return None
    return getattr(module, operator_name, None)


def _operator_options(operator) -> Set[str]:
    """The keyword options an operator accepts, read from its RNA."""
    try:
        return {prop.identifier for prop in operator.get_rna_type().properties}
    except Exception:
        return set()


def import_model(path: str):
    """Import a model by extension, using whichever importer this Blender has.

    Raises ``ValueError`` for an unsupported extension and ``RuntimeError`` when
    the matching importer is missing (its add-on disabled / removed).
    """
    suffix = os.path.splitext(path)[1].lower()
    candidates = _IMPORTERS.get(suffix)
    if not candidates:
        raise ValueError(f"unsupported ground model format {suffix!r}")
    for module_name, operator_name in candidates:
        operator = _operator(module_name, operator_name)
        if operator is not None:
            return operator(filepath=path)
    raise RuntimeError(f"no {suffix} importer available in this Blender")


def export_gltf(filepath: str, selection: bool = True, apply_modifiers: bool = True):
    """Export a binary glTF, passing only the options this Blender's exporter has.

    The exporter gained and renamed options across releases; the accepted set is
    read from the operator's RNA instead of assumed.
    """
    operator = _operator("export_scene", "gltf")
    if operator is None:
        raise RuntimeError("the glTF exporter is not available in this Blender")
    supported = _operator_options(operator)
    options = {"filepath": filepath}
    if "export_format" in supported:
        options["export_format"] = "GLB"
    if selection and "use_selection" in supported:
        options["use_selection"] = True
    if apply_modifiers and "export_apply" in supported:
        options["export_apply"] = True
    return operator(**options)


def set_material_blend(material, blended: bool = True) -> None:
    """Make a material transparent, on EEVEE Legacy or EEVEE Next / 5.x.

    ``Material.blend_method`` was replaced by ``surface_render_method`` in 4.2;
    set whichever the running Blender exposes (setting both is harmless).
    """
    settings = (("blend_method", "BLEND" if blended else "OPAQUE"),
                ("surface_render_method", "BLENDED" if blended else "DITHERED"))
    for attribute, value in settings:
        if hasattr(material, attribute):
            try:
                setattr(material, attribute, value)
            except Exception:
                pass


def node_of_type(node_tree, node_type: str):
    """First node of an RNA ``type`` (e.g. ``"BSDF_PRINCIPLED"``), or ``None``.

    Node **names** are localized - a Chinese UI renames "Principled BSDF" and
    "World Output" - so nodes are found by their version-stable ``type``, never
    by name.
    """
    for node in node_tree.nodes:
        if node.type == node_type:
            return node
    return None


def sequence_strips(scene):
    """The sequence editor's strip collection, creating the editor if needed.

    Blender 4.4 renamed ``SequenceEditor.sequences`` to ``.strips``; resolve
    whichever this Blender exposes so callers just add strips.
    """
    editor = scene.sequence_editor or scene.sequence_editor_create()
    strips = getattr(editor, "strips", None)
    if strips is not None:   # an empty collection is falsy, so test for None
        return strips
    return editor.sequences


def enable_movie_output(image_settings) -> None:
    """Select FFmpeg movie output on a render's image settings.

    Blender 4.5 added ``ImageFormatSettings.media_type``; ``FFMPEG`` is only an
    assignable ``file_format`` once the media type is ``VIDEO``.
    """
    if hasattr(image_settings, "media_type"):
        image_settings.media_type = "VIDEO"
    image_settings.file_format = "FFMPEG"
