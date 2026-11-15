#!/usr/bin/env python3
"""Export the ego car GLB + manifest and a combined bowl+vehicle GLB.

    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        --python scripts/export_ego_model.py -- --out /path/to/out

The bowl layout follows the ground model.  The bundled model is the 009 K7
de-duplicated quadrant layout, which the current Filament app cannot consume yet
(it maps one camera per node; a quadrant carries two).  For the app, pass the
legacy half-face model and leave --quadrants off:

    git show 5b35e12:addons/opencv_camera/models/unlit_round_bowls.glb \
        > /tmp/unlit_round_bowls_legacy.glb
    ... --python scripts/export_ego_model.py -- --out <dir> \
        --ground-model /tmp/unlit_round_bowls_legacy.glb
"""
from __future__ import annotations

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons"))

import bpy  # noqa: E402

import opencv_camera  # noqa: E402
from opencv_camera.core.scenes import vehicle  # noqa: E402


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--style", default=vehicle.STYLE_2ROW,
                        choices=[vehicle.STYLE_2ROW, vehicle.STYLE_3ROW])
    parser.add_argument("--quadrants", action="store_true")
    parser.add_argument("--checker", action="store_true",
                        help="put a checker baseColorTexture on paint_body/paint_door (V3)")
    parser.add_argument("--ground-model", default="",
                        help="ground .glb to import instead of the bundled bowl")
    # Body box of the displayed car.  The defaults are the reference vehicle
    # (Metro Elf); pass the recorded clip's own box to render a car that matches
    # the AVM mask and the camera mounts - a longer body puts the front/rear
    # mounts inside the chassis occluder and blinds those routes.
    parser.add_argument("--length", type=float, default=vehicle.BODY_LENGTH_M,
                        help="body length [m] (default %(default)s)")
    parser.add_argument("--width", type=float, default=vehicle.BODY_WIDTH_M,
                        help="body width [m] (default %(default)s)")
    parser.add_argument("--height", type=float, default=vehicle.BODY_HEIGHT_M,
                        help="body height [m] (default %(default)s)")
    return parser.parse_args(argv)


def add_checker_texture():
    size = 64
    image = bpy.data.images.new("checker_paint", width=size, height=size, alpha=False)
    pixels = []
    for y in range(size):
        for x in range(size):
            on = (((x * 8) // size) + ((y * 8) // size)) % 2 == 0
            value = 1.0 if on else 0.05
            pixels.extend((value, value, value, 1.0))
    image.pixels = pixels
    image.pack()
    for name in ("paint_body", "paint_door"):
        material = bpy.data.materials.get(name)
        if material is None or not material.use_nodes:
            continue
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        background = next((n for n in nodes if n.type == "BACKGROUND"), None)
        if background is None:
            continue
        texture = nodes.new("ShaderNodeTexImage")
        texture.image = image
        texture.interpolation = "Closest"
        links.new(texture.outputs["Color"], background.inputs["Color"])


def main():
    args = parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)

    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    opencv_camera.register()
    try:
        bpy.ops.opencv_cam.avm_add_scene()
        settings = bpy.context.scene.avm_scene
        settings.car_style = args.style
        settings.ground_radius = 15.0
        settings.ground_rim_height = 5.0
        settings.bowl_quadrants = bool(args.quadrants)
        settings.bowl_include_vehicle = True
        if args.ground_model:
            settings.ground_model = os.path.abspath(args.ground_model)
        # Display vehicle vs recorded rig: the car is the model itself, sized by
        # --length/--width/--height (default: the reference vehicle), independent
        # of the AVM mask.
        settings.car_follow_core = False
        settings.car_length = args.length
        settings.car_width = args.width
        settings.car_height = args.height
        bpy.ops.opencv_cam.avm_rebuild()
        if args.checker:
            add_checker_texture()

        vehicle_glb = os.path.join(out, "vehicle.glb")
        print(bpy.ops.opencv_cam.avm_export_vehicle(filepath=vehicle_glb))

        bowl_glb = os.path.join(out, "unlit_round_bowls.glb")
        print(bpy.ops.opencv_cam.avm_export_bowl(filepath=bowl_glb))
        print("wrote", vehicle_glb, bowl_glb)
    finally:
        opencv_camera.unregister()


main()
