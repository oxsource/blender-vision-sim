"""The four surround cameras: which ones exist, in what order, under what names.

Pure Python (no ``bpy``).  "There are four cameras, they are called this, they
are recorded in that order" used to be spread over three places - the field
layout's ``CAMERAS``, the AVM builder's ``CAMERA_SUFFIX`` and the AVM panel's
``CAMERA_ITEMS``.  Collecting it here is what lets the AVM Scene and the Drive
Scene share **one** definition instead of two that only a test keeps aligned
(``docs/drive-scene-multicam.md`` section 4).

Scope, deliberately narrow (section 4.1): names, order, object naming, and how a
calibration record becomes a mount pose.  Nothing about rendering, collections,
directories or export formats - so roadmap M6's ``camera_rig`` can wrap this as
its data source rather than replace it.

The camera *key* is the rig role (``front``), the *object name* is
``<prefix><Suffix>`` (``AVM_Cam_Front`` / ``DRIVE_Cam_Front``).  The clip
contract speaks both: object names in ``clip.json``, keys in the file names -
see :func:`key_of`.
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence, Tuple

from . import drive_path

__all__ = [
    "FRONT", "BACK", "LEFT", "RIGHT", "CAMERAS", "SUFFIX", "LABEL", "DESCRIPTION",
    "object_name", "enum_items", "key_of", "mount_of", "mounts_of",
]

FRONT, BACK, LEFT, RIGHT = "front", "back", "left", "right"

#: The four rig roles.  **This tuple is the order**: it is the order the AVM
#: calibration points are generated in, the order the panels list them, and the
#: order a clip records them in, so a file's column group can be found by name
#: without guessing.
CAMERAS: Tuple[str, ...] = (FRONT, BACK, LEFT, RIGHT)

#: key -> object name suffix (``front`` -> ``Front``, i.e. ``AVM_Cam_Front``)
SUFFIX: Dict[str, str] = {
    FRONT: "Front",
    BACK: "Back",
    LEFT: "Left",
    RIGHT: "Right",
}

#: key -> the label the panels / enums show
LABEL: Dict[str, str] = {
    FRONT: "Front",
    BACK: "Back",
    LEFT: "Left",
    RIGHT: "Right",
}

#: key -> the enum tooltip
DESCRIPTION: Dict[str, str] = {
    FRONT: "Front camera",
    BACK: "Rear camera",
    LEFT: "Left camera",
    RIGHT: "Right camera",
}


def object_name(prefix: str, key: str) -> str:
    """The object name of ``key`` under ``prefix``.

    ``("AVM_Cam_", "front") -> "AVM_Cam_Front"``.  Both scenes build their
    cameras through this, so ``AVM_Cam_Front`` and ``DRIVE_Cam_Front`` cannot end
    up with different spellings (the strings are read back by ``blend`` files,
    by the clip contract and by the scripts that consume them).
    """
    return f"{prefix}{SUFFIX.get(key, key)}"


def enum_items() -> List[Tuple[str, str, str]]:
    """The ``EnumProperty`` items, in :data:`CAMERAS` order.

    A fresh list every call: Blender keeps its own copy and re-evaluates the
    annotations, so a module level list is what it actually wants.
    """
    return [(key, LABEL.get(key, key), DESCRIPTION.get(key, key)) for key in CAMERAS]


def key_of(name: str) -> str:
    """A camera object name back to its key: ``DRIVE_Cam_Front`` -> ``front``.

    This is the sim side of the naming contract the renderer's
    ``FrameSource::NormalizeCameraKey()`` implements on the consuming side
    (``filament_avm/falcon/core/frame/frame_source.cc``) - the two are written to
    agree, because the clip's file names carry the key while ``clip.json``
    carries the object name.
    """
    text = str(name)
    for prefix in ("DRIVE_Cam_", "AVM_Cam_"):
        if text.startswith(prefix):
            text = text[len(prefix):]
            break
    return text.lower()


def mount_of(record: Dict[str, Any]) -> drive_path.Mount:
    """A calibration record's fixed pose in the vehicle frame.

    ``record`` is an :func:`core.scenes.avm_layout.cameras_from_preset` entry:
    ``location`` in metres and ``rotation`` as XYZ Euler **degrees**, which is
    what :class:`drive_path.Mount` stores.
    """
    return drive_path.Mount(
        location=tuple(float(value) for value in record.get("location", (0.0, 0.0, 0.0))),
        rotation_deg=tuple(float(value) for value in record.get("rotation", (0.0, 0.0, 0.0))),
    )


def mounts_of(records: Sequence[Dict[str, Any]]) -> Dict[str, drive_path.Mount]:
    """``{key: Mount}`` for a preset's records, keeping the preset's order.

    Unknown names are skipped rather than guessed at: a record that is not one of
    :data:`CAMERAS` has no key to file its pose under.
    """
    mounts: Dict[str, drive_path.Mount] = {}
    for record in records:
        key = str(record.get("name", ""))
        if key in CAMERAS:
            mounts[key] = mount_of(record)
    return mounts
