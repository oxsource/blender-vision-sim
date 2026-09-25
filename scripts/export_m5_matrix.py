#!/usr/bin/env python3
"""Render the isolated non-lighting M5 road / speed / direction matrix.

Run with Blender's Python:
    blender -b --factory-startup --python scripts/export_m5_matrix.py -- --output /tmp/m5-01b
"""

from __future__ import annotations

import argparse
import os
import sys

import bpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons"))

import opencv_camera
from opencv_camera.bl.scenes.road_scene import builder, recording


def matrix_cases():
    base = [
        (segment, speed, direction, 0.0)
        for segment in ("straight_a", "curve_0", "ramp_up", "ramp_down")
        for speed in (3.5, 7.0)
        for direction in ("forward", "reverse")
    ]
    banked = [
        ("curve_0", speed, direction, 6.0)
        for speed in (3.5, 7.0)
        for direction in ("forward", "reverse")
    ]
    return base + banked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--limit", type=int, default=0,
                        help="export only the first N cases as a smoke test")
    parser.add_argument("--case", action="append", default=[],
                        help="export only this slug (may be repeated)")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:]
                             if "--" in sys.argv else [])
    output = os.path.abspath(args.output)
    os.makedirs(output, exist_ok=True)

    opencv_camera.register()
    try:
        result = bpy.ops.opencv_cam.road_add_scene()
        if "FINISHED" not in result:
            raise RuntimeError(f"Road Scene creation failed: {result}")
        scene = bpy.context.scene
        settings = scene.road_scene
        settings.track_preset = "compact"
        settings.parking = False
        settings.show_crosswalk = False
        settings.pedestrians = 0
        settings.trees = 0
        settings.lamps = 0
        settings.signs = 0
        settings.animate_pedestrians = False
        settings.drive_profile = "constant"
        settings.drive_fps = max(1, int(args.fps))
        settings.clip_quality = "draft"
        settings.clip_device = "cpu"
        settings.clip_keep_frames = False
        settings.ensure_cameras()
        for camera in settings.cameras:
            camera.enable = True

        cases = matrix_cases()
        if args.case:
            wanted = set(args.case)
            cases = [case for case in cases
                     if f"{case[0]}_v{case[1]:g}_{case[2]}_bank{case[3]:g}" in wanted]
            found = {f"{case[0]}_v{case[1]:g}_{case[2]}_bank{case[3]:g}"
                     for case in cases}
            missing = wanted - found
            if missing:
                raise ValueError(f"unknown matrix case(s): {sorted(missing)}")
        if args.limit > 0:
            cases = cases[:args.limit]
        for segment, speed, direction, bank in cases:
            settings.drive_segment = segment
            settings.drive_speed = speed
            settings.drive_direction = direction
            settings.curve_bank_deg = bank
            builder.rebuild(scene, settings)
            slug = f"{segment}_v{speed:g}_{direction}_bank{bank:g}"
            filepath = os.path.join(output, f"{slug}.zip")
            result = recording.export_zip(bpy.context, settings, filepath,
                                          samples=max(1, int(args.samples)))
            if result.get("cancelled") or not os.path.isfile(filepath):
                raise RuntimeError(f"clip export failed for {slug}: {result}")
            print(f"M5_CASE {slug} frames={result['frames']} zip={filepath}", flush=True)
    finally:
        opencv_camera.unregister()


if __name__ == "__main__":
    main()
