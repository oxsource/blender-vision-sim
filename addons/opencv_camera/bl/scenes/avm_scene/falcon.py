"""Export the Falcon calibration config and the four original camera images.

This is the sim's ``JsonGenerate``: it produces the four "original" photos
(``front/back/left/right.png``) plus the full Falcon ``vehicle_avm.json``
(``core/scenes/avm_falcon.py``), packed into a zip.

The camera images come from the per-camera corner-detection cache whenever it is
still valid, so a scene where every camera was already detected exports without
re-rendering anything.
"""

from __future__ import annotations

import datetime
import os
import shutil
import tempfile
import zipfile
from typing import Dict, List, Tuple

import bpy

from ....core.scenes import avm_layout, avm_falcon
from ... import apply as apply_mod
from . import builder, corners, io as io_mod

#: the app's ``FileNames.CONFIG_JSON``
DEFAULT_NAME = "vehicle_avm"


def _input_size(name: str) -> Tuple[int, int]:
    camera = bpy.data.objects.get(
        f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(name, '')}")
    if camera is None:
        return 1280, 960
    return apply_mod.output_resolution(camera.data.opencv_cam, bpy.context.scene)


def build(settings) -> Dict:
    """The Falcon config object for the current settings / detected points.

    ``K`` is the effective intrinsics at the rendered image size and
    ``points_2d`` are the pixels detected in that same image, so the config is
    self-consistent whatever output resolution the cameras use."""
    records = io_mod.camera_records(settings)
    points_2d: Dict[str, List[List[float]]] = {}
    input_sizes: Dict[str, Tuple[int, int]] = {}
    for record in records:
        name = record["name"]
        entry = settings.camera(name)
        if entry is not None and entry.points_2d_ok:
            values = list(entry.points_2d)
            points_2d[name] = [[values[2 * index], values[2 * index + 1]]
                               for index in range(8)]
        width, height = _input_size(name)
        input_sizes[name] = (width, height)
        camera = bpy.data.objects.get(
            f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(name, '')}")
        if camera is not None:
            intrinsics = apply_mod.effective_intrinsics(
                camera.data.opencv_cam, width, height)
            record["K"] = [intrinsics.fx, intrinsics.fy,
                           intrinsics.cx, intrinsics.cy]
    date_text = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return avm_falcon.build_config(settings.field_spec(), records,
                                   points_2d, input_sizes, date_text)


def _camera_image(context, settings, name: str, directory: str,
                  samples: int) -> str:
    """The camera PNG in ``directory``: copy the cache or render it once."""
    destination = os.path.join(directory, f"{name}.png")
    if corners.is_cached(settings, name):
        shutil.copyfile(corners.raw_image_path(name), destination)
        return destination
    io_mod.render_cameras(context, settings, directory, samples=samples,
                          names=[name])
    if os.path.exists(destination):
        corners.detect_file(settings, name, destination, samples=samples)
    return destination


def export(context, settings, directory: str, samples: int = 64,
           name: str = DEFAULT_NAME) -> List[str]:
    """Write the 4 camera images, their annotations and ``<name>.json``."""
    os.makedirs(directory, exist_ok=True)
    written: List[str] = []
    for camera in avm_layout.CAMERAS:
        entry = settings.camera(camera)
        if entry is None or not entry.enable:
            continue
        written.append(_camera_image(context, settings, camera, directory, samples))
        annotated = corners.corner_image_path(camera)
        if os.path.exists(annotated):
            destination = os.path.join(directory, f"{camera}_annotated.png")
            shutil.copyfile(annotated, destination)
            written.append(destination)
    path = os.path.join(directory, f"{name}.json")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(avm_falcon.dumps(build(settings)))
    written.append(path)
    return written


def export_zip(context, settings, filepath: str, samples: int = 64,
               name: str = DEFAULT_NAME) -> List[str]:
    """Render (or reuse) the 4 images + config and pack them into a zip."""
    directory = tempfile.mkdtemp(prefix="avm_falcon_")
    try:
        files = export(context, settings, directory, samples=samples, name=name)
        parent = os.path.dirname(os.path.abspath(filepath))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with zipfile.ZipFile(filepath, "w", zipfile.ZIP_DEFLATED) as archive:
            for path in files:
                archive.write(path, arcname=os.path.basename(path))
        return [os.path.basename(path) for path in files]
    finally:
        shutil.rmtree(directory, ignore_errors=True)
