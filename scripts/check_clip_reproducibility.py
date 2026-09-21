#!/usr/bin/env python3
"""Is a drive clip reproducible - and if not, by how much?

A validation run should be able to consume the exported **video** and throw the
stills away, because the stills are ~100x the bytes.  That is only safe if the
stills are *regenerable*: `clip.json` records the scenario and the render recipe,
so a harness that has to separate "the algorithm drifted" from "the encoder
drifted" can re-render the PNG sequence and re-encode it.

This script measures how far that holds.  It renders the same short clip three
times - twice back to back, once more after a full scene rebuild - and reports,
for every frame:

* whether the bytes are identical, and
* the **magnitude** of the difference when they are not (max / mean |Δ| in LSB).

The distinction is the point: "byte-identical" is what a bit-exact regression
needs, "pixel-wise negligible" is what a *re-generate the reference* workflow
needs.  They are different bars.

Usage:
    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        --python scripts/check_clip_reproducibility.py

Context: filament_avm docs/transparent_chassis_540_design.md 4.6.6 (decision D12).
"""
import hashlib
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons"))

import bpy  # noqa: E402
import numpy as np  # noqa: E402

import opencv_camera  # noqa: E402
from opencv_camera.bl.scenes.drive_scene import recording  # noqa: E402
from opencv_camera.core.scenes import avm_cameras  # noqa: E402

FAILURES = []


def check(name, condition, detail=""):
    print(f"[{'PASS' if condition else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def digest(path):
    with open(path, "rb") as handle:
        return hashlib.md5(handle.read()).hexdigest()


def chunks(path):
    """The PNG chunk list as ``(type, keyword, payload_md5, length)``.

    Comparing whole files answers the wrong question: a PNG can carry a timestamp
    or a different (equally valid) zlib split and still decode to the same pixels.
    ``keyword`` is the tEXt key when there is one, so a difference can be named
    ("Date", "cycles.ViewLayer.render_time") instead of pointed at.
    """
    with open(path, "rb") as handle:
        data = handle.read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n", f"{path} is not a PNG"
    out, offset = [], 8
    while offset + 12 <= len(data):
        length = int.from_bytes(data[offset:offset + 4], "big")
        kind = data[offset + 4:offset + 8].decode("latin1")
        payload = data[offset + 8:offset + 8 + length]
        keyword = ""
        if kind == "tEXt" and b"\x00" in payload:
            keyword = payload.split(b"\x00", 1)[0].decode("latin1")
        out.append((kind, keyword, hashlib.md5(payload).hexdigest(), length))
        offset += 12 + length
    return out


def chunk_diff(dir_a, dir_b, name):
    """Which chunks differ between two renders of the same frame, by name."""
    left, right = chunks(os.path.join(dir_a, name)), chunks(os.path.join(dir_b, name))
    differing = []
    for a, b in zip(left, right):
        if a == b:
            continue
        kind, keyword = a[0], a[1]
        differing.append(f"{kind}:{keyword}" if keyword else kind)
    if len(left) != len(right):
        differing.append(f"chunk count {len(left)} vs {len(right)}")
    return differing


def pixels(path):
    """The raw stored samples of a PNG as float 0..255, shape (h, w, 4)."""
    image = bpy.data.images.load(path, check_existing=False)
    try:
        image.colorspace_settings.name = "Non-Color"   # no transform, raw samples
        flat = np.array(image.pixels[:], dtype=np.float64)
        return flat.reshape(image.size[1], image.size[0], 4) * 255.0
    finally:
        bpy.data.images.remove(image)


def render_once(directory):
    settings = bpy.context.scene.drive_scene
    report = recording.render_clip(bpy.context, settings, directory, samples=4)
    return report, settings.plan()


def compare(label, dir_a, dir_b, names):
    same, worst, means, worst_at, chunk_kinds = 0, 0.0, [], None, set()
    for name in names:
        path_a, path_b = os.path.join(dir_a, name), os.path.join(dir_b, name)
        if digest(path_a) == digest(path_b):
            same += 1
            continue
        chunk_kinds.update(chunk_diff(dir_a, dir_b, name))
        delta = np.abs(pixels(path_a) - pixels(path_b))
        means.append(float(delta.mean()))
        if delta.max() > worst:
            worst, worst_at = float(delta.max()), (name, float(delta.mean()))
    mean_all = float(np.mean(means)) if means else 0.0
    print(f"  {label:<30} files identical {same}/{len(names)}   "
          f"max |Δ| {worst:.0f} LSB   mean |Δ| {mean_all:.4f} LSB", end="")
    print(f"   (worst {worst_at[0]}, its mean {worst_at[1]:.4f} LSB)" if worst_at
          else "   (pixels identical)")
    if chunk_kinds:
        print(f"  {'':<30} bytes differ in: {', '.join(sorted(chunk_kinds))}")
    return same, worst, mean_all, sorted(chunk_kinds)


def main():
    # the test suite registers the add-on directly rather than through
    # addon_utils, because `addons/` is on sys.path, not Blender's add-on path
    opencv_camera.register()
    scene = bpy.context.scene
    if scene.render.engine != "CYCLES":
        scene.render.engine = "CYCLES"
    settings = scene.drive_scene

    # a short, tiny clip - this is about determinism, not about pixels
    settings.drive_distance = 6.0
    settings.drive_speed = 3.0
    settings.drive_fps = 4
    scene.render.resolution_x, scene.render.resolution_y = 192, 144
    check("drive scene built", bpy.ops.opencv_cam.drive_add_scene() == {"FINISHED"})
    check("the clip is 9 frames", len(settings.plan().frames) == 9)
    # one camera keeps this probe about determinism rather than about render cost;
    # the four-lane path is covered by tests/run_blender_tests.py
    for key in avm_cameras.CAMERAS:
        if key != avm_cameras.FRONT:
            settings.camera(key).enable = False
    check("the probe records one camera", settings.recorded_cameras() == ["front"],
          str(settings.recorded_cameras()))
    bpy.ops.opencv_cam.drive_rebuild()

    dirs = [tempfile.mkdtemp(prefix=f"clip_repro_{tag}_") for tag in "abc"]
    report, plan = render_once(dirs[0])
    check("first render wrote every frame",
          report["frames"] == len(plan.frames), str(report["frames"]))
    render_once(dirs[1])                     # same session state, back to back
    check("rebuild", bpy.ops.opencv_cam.drive_rebuild() == {"FINISHED"})
    render_once(dirs[2])                     # after a full scene rebuild

    names = sorted(name for name in os.listdir(dirs[0]) if name.endswith(".png"))
    check("all three runs produced the same file set",
          all(sorted(os.listdir(d)) == sorted(os.listdir(dirs[0])) for d in dirs))

    print()
    print("=" * 78)
    print("two renders, back to back (same session state)")
    print("=" * 78)
    same_ab, max_ab, mean_ab, kinds_ab = compare("run A vs run B", dirs[0], dirs[1], names)

    print()
    print("=" * 78)
    print("run A vs run C (a full scene rebuild in between)")
    print("=" * 78)
    same_ac, max_ac, mean_ac, kinds_ac = compare("run A vs run C", dirs[0], dirs[2], names)

    print()
    print("=" * 78)
    print("the truth file")
    print("=" * 78)
    check("frames.csv is byte-identical across all three runs",
          len({digest(os.path.join(d, "frames.csv")) for d in dirs}) == 1)
    metas = [json.load(open(os.path.join(d, "clip.json"), encoding="utf-8")) for d in dirs]
    for meta in metas:
        meta.pop("created", None)
    check("clip.json is identical apart from its timestamp",
          metas[0] == metas[1] == metas[2])

    print()
    print("=" * 78)
    print("verdict")
    print("=" * 78)
    kinds = sorted(set(kinds_ab) | set(kinds_ac))
    pixel_exact = (max_ab == 0.0 and max_ac == 0.0)
    if same_ab == len(names) and same_ac == len(names):
        print("  Byte-identical: the stills are a pure function of the settings, so")
        print("  clip.json + a re-render IS the lossless reference and a lossless")
        print("  regression set does not have to be stored (design doc D12).")
    elif pixel_exact:
        print("  PIXEL-identical but not byte-identical.  Every frame decodes to exactly")
        print("  the same samples (Δ = 0 LSB over all 9 frames); only the PNG container")
        print(f"  bytes differ{': ' + ', '.join(kinds) if kinds else ''}.")
        print("  -> Hash the DECODED pixels, not the files, when checking a re-render;")
        print("     the lossless reference IS regenerable (design doc D12 = (a)).")
    else:
        print("  NOT pixel-identical - a bit-exact baseline cannot be regenerated, it")
        print("  has to be stored.  Pixel noise between two runs of the SAME clip:")
        print(f"    back to back : max {max_ab:.0f} LSB, mean {mean_ab:.4f} LSB, "
              f"{same_ab}/{len(names)} frames bit-identical")
        print(f"    after rebuild: max {max_ac:.0f} LSB, mean {mean_ac:.4f} LSB, "
              f"{same_ac}/{len(names)} frames bit-identical")
        print("  frames.csv (the truth) and every recorded setting are stable, so a")
        print("  re-render is a valid reference - just not a bit-exact one (D12 = (b)).")
    print(f"  blender {bpy.app.version_string}, Cycles 4 samples, 192x144, 9 frames")
    print(f"  runs kept in: {', '.join(dirs)}")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
