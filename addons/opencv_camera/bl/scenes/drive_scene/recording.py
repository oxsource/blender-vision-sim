"""Record a drive clip: one still per frame per camera, plus the per-frame truth.

The output of this module is the contract with the algorithm side:

* ``frame_%04d_<camera>.png`` - one still per recorded camera per frame;
* ``<camera>.mp4`` - the same frames as one H.264 video per camera
  (``export_zip`` only);
* ``frames.csv`` - one row per frame: time, distance, speed, the vehicle's world
  pose and **each** camera's world pose (vehicle pose composed with that camera's
  fixed mount pose);
* ``clip.json`` - everything needed to reproduce the clip (drive parameters, one
  ``cameras`` entry per recorded camera with its K / D / mount pose, the vehicle
  geometry, the render settings, and - with a video - the mp4's encode recipe).

A validation run consumes the **videos**, not the stills: they are ~1/100 of the
size, they are the shape a head unit actually receives (a compressed stream), and
the transparent-chassis reconstruction is indistinguishable between the two.
``clip_keep_frames`` therefore decides whether the stills travel with the export
- they stay the ground truth and the encoder's input either way, and the scenario
in ``clip.json`` can always re-render them.

The contract is **v2**: one camera group per recorded camera, named, instead of a
single unnamed ``cam_*`` group (``docs/drive-scene-multicam.md`` section 5).  The
order of ``frames.csv``'s column groups and of ``clip.json``'s ``cameras`` array
is ``avm_cameras.CAMERAS`` filtered by what the user enabled.

The rows come from :mod:`core.scenes.drive_path`, the very plan the vehicle was
keyframed with, so a PNG and its CSV row can never describe different poses.

:class:`ClipJob` is the stepwise engine: the modal export operator renders one
frame per UI tick (so it can show progress / an ETA and be cancelled), while
:func:`render_clip` / :func:`export_zip` drive the very same job synchronously
for tests and headless use.  One UI tick renders *every* recorded camera for that
frame, so the progress unit stays "frame" and the ETA stays comparable
(``docs/drive-scene-multicam.md`` section 9, C4).
"""

from __future__ import annotations

import datetime
import json
import math
import os
import shutil
import tempfile
import time
import zipfile
from typing import Dict, List, Optional, Sequence

import bpy

from ....core.scenes import drive_path, vehicle
from ... import apply as apply_mod
from ... import compat
from . import builder
from . import properties as drive_properties

FORMAT = "drive_clip"
#: 1 = a single unnamed ``camera`` (front only); 2 = a ``cameras`` array, one per
#: recorded camera, with named ``frames.csv`` column groups.  The version is the
#: only field a consumer needs to tell the two apart.
VERSION = 2
#: how a clip's stills are named; ``<camera>`` is the camera *key* (``front``),
#: which is the spelling the renderer's ``FrameSource`` looks for
FRAME_PATTERN = "frame_%04d_<camera>.png"
#: how a clip's videos are named - one container per camera, no single ``video``
VIDEO_PATTERN = "<camera>.mp4"
#: the default zip name of [Export Clip…] - a stable name, not the .blend's
DEFAULT_NAME = "drive_scene"

#: The view transform / look the stills are written with
#: (``builder._ensure_render_setup``).  ``encode_video`` mirrors the *render*
#: scene's pair instead of hardcoding these, so an encoder scene can never colour
#: finished pixels a second time; the constants are the fallback for a caller
#: that has no scene at hand.
OUTPUT_VIEW_TRANSFORM = "Standard"
OUTPUT_LOOK = "None"
#: What ``encode_video`` asks Blender's FFmpeg for; recorded in ``clip.json``
#: so a regression can be attributed to the codec rather than to the algorithm.
VIDEO_FORMAT = "MPEG4"
VIDEO_CODEC = "H264"
VIDEO_CRF = "HIGH"

#: Cycles compute backends that can evaluate the OSL camera shader.  OSL is
#: supported on CPU and NVIDIA OptiX only - Metal / CUDA / HIP / oneAPI fall
#: back to a wrong camera, so the export refuses to use them.
_OSL_CAMERA_DEVICES = ("OPTIX",)


class RenderCancelled(Exception):
    """The user aborted the render (ESC / the render window was closed)."""


def default_filename() -> str:
    """The default ``[Export Clip…]`` file name (``drive_scene.zip``)."""
    return f"{DEFAULT_NAME}.zip"


def frame_name(index: int, camera: str) -> str:
    """One still's file name: ``frame_0000_front.png``.

    Flat rather than one directory per camera: "frame N" stays the file name's
    primary key, which is what makes sorting, diffing and slicing a four-lane
    clip cheap (``docs/drive-scene-multicam.md`` section 5).
    """
    return f"frame_{index:04d}_{camera}.png"


def video_name(camera: str) -> str:
    """One camera's container name: ``front.mp4``."""
    return f"{camera}.mp4"


def compute_device_type() -> str:
    """The Cycles preference's active compute device type (``"NONE"`` if unset).

    Read through the Cycles add-on preferences; every access is guarded so a
    missing / disabled Cycles add-on cannot break the export.
    """
    try:
        addon = bpy.context.preferences.addons.get("cycles")
        preferences = getattr(addon, "preferences", None) if addon is not None else None
        return str(getattr(preferences, "compute_device_type", "NONE") or "NONE")
    except Exception:
        return "NONE"


def gpu_can_render_osl_camera(device_type: str) -> bool:
    """True when the GPU backend can evaluate the OSL camera (OptiX only)."""
    return str(device_type).upper() in _OSL_CAMERA_DEVICES


def enabled_osl_gpu_available() -> bool:
    """True when an OptiX GPU device is enabled in the Cycles preferences."""
    try:
        addon = bpy.context.preferences.addons.get("cycles")
        preferences = getattr(addon, "preferences", None) if addon is not None else None
        if preferences is None:
            return False
        preferences.get_devices()
        for device in preferences.devices:
            if (getattr(device, "use", False)
                    and gpu_can_render_osl_camera(getattr(device, "type", ""))):
                return True
    except Exception:
        pass
    return False


def resolve_device(requested: str, messages: List[str]) -> str:
    """The Cycles device to render with: ``"CPU"`` or ``"GPU"``.

    CPU is always correct.  GPU is only usable when the active compute device is
    OptiX (the one backend that runs OSL camera shaders) and an OptiX device is
    enabled; any other case is reported and falls back to CPU so the clip's
    camera stays correct.
    """
    if str(requested).lower() != "gpu":
        return "CPU"
    device_type = compute_device_type()
    if gpu_can_render_osl_camera(device_type) and enabled_osl_gpu_available():
        return "GPU"
    if gpu_can_render_osl_camera(device_type):
        reason = "no OptiX GPU device is enabled in the Cycles preferences"
    else:
        reason = f"{device_type} cannot run the OSL camera"
    messages.append(
        f"the OpenCV camera is an OSL shader, which Cycles evaluates on CPU or "
        f"OptiX only; {reason} - rendering on CPU instead")
    return "CPU"


def format_duration(seconds: Optional[float]) -> str:
    """``m:ss`` (or ``h:mm:ss``) for an ETA; ``--:--`` when unknown."""
    if seconds is None:
        return "--:--"
    try:
        value = float(seconds)
    except (TypeError, ValueError):
        return "--:--"
    if math.isnan(value) or math.isinf(value) or value < 0:
        return "--:--"
    total = int(round(value))
    hours, rest = divmod(total, 3600)
    minutes, secs = divmod(rest, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes}:{secs:02d}"


def progress_text(done: int, total: int, eta: Optional[float]) -> str:
    """``frame k/N · xx% · ETA m:ss`` for the panel / status line."""
    if total <= 0:
        return "export: nothing to render"
    percent = int(round(100.0 * done / total))
    return f"frame {done}/{total} · {percent}% · ETA {format_duration(eta)}"


def camera_mounts(cameras: Sequence[str]) -> Dict[str, drive_path.Mount]:
    """Each camera's fixed pose in the vehicle frame (metres, degrees).

    Read from the camera **objects**, not from the preset: the object transform
    is the single source of a mount pose, so a pose the user adjusted in
    ``CV Extrinsics`` is what the clip records (``docs/drive-scene.md``).
    """
    mounts: Dict[str, drive_path.Mount] = {}
    for key in cameras:
        camera = bpy.data.objects.get(builder.camera_name(key))
        if camera is None:
            mounts[key] = drive_path.Mount()
            continue
        mounts[key] = drive_path.Mount(
            location=tuple(float(value) for value in camera.location),
            rotation_deg=tuple(math.degrees(float(value)) for value in camera.rotation_euler),
        )
    return mounts


def camera_entry(key: str, width: int, height: int) -> Dict:
    """One ``clip.json`` ``cameras`` entry: what this camera is and where it sits.

    ``K`` / ``D`` are the **effective** intrinsics for the render's resolution
    (the calibration is stored at the camera's own output size), so a consumer
    can project with them without repeating the scaling rule.
    """
    name = builder.camera_name(key)
    entry: Dict = {
        "name": name,
        "camera": key,
        "model": "fisheye",
        "output": [int(width), int(height)],
    }
    camera = bpy.data.objects.get(name)
    if camera is not None:
        settings = camera.data.opencv_cam
        intrinsics = apply_mod.effective_intrinsics(settings, width, height)
        distortion = settings.distortion
        entry["K"] = [float(intrinsics.fx), float(intrinsics.fy),
                      float(intrinsics.cx), float(intrinsics.cy)]
        entry["D"] = [float(distortion.k1), float(distortion.k2),
                      float(distortion.k3), float(distortion.k4)]
        entry["mount"] = {
            "frame": "vehicle",
            "location": [float(value) for value in camera.location],
            "rotation_deg": [math.degrees(float(value))
                             for value in camera.rotation_euler],
        }
    return entry


def video_field(cameras: Sequence[str]) -> str:
    """``clip.json``'s ``video``: the container's name for a **single**-camera clip.

    With more than one camera there is no single container - the contract is one
    ``<camera>.mp4`` per camera, which is what :data:`VIDEO_PATTERN` states and
    what a consumer looks for - so this is empty rather than a misleading name.
    """
    return video_name(cameras[0]) if len(cameras) == 1 else ""


def clip_meta(scene: bpy.types.Scene, settings, plan, samples: int,
              cameras: Sequence[str], video: str = "", quality: str = "",
              device: str = "", video_encode: Optional[Dict] = None) -> Dict:
    """The clip's self-description (what was rendered / driven / recorded)."""
    width, height = apply_mod.render_resolution(scene)
    return {
        "format": FORMAT,
        "version": VERSION,
        "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "fps": plan.fps,
        "frames": len(plan.frames),
        "duration_s": round(plan.duration, 6),
        "drive": {
            "distance_m": plan.distance,
            "profile": plan.profile,
            "cruise_speed_mps": plan.cruise_speed,
            "accel_mps2": plan.accel,
            "heading_deg": plan.heading,
            "start_xy": list(plan.start),
            "vehicle_frame": "world: X/Y/Z euler, Z up, nose +Y at yaw 0",
        },
        "render": {
            "engine": "CYCLES",
            "device": device,
            "samples": int(samples),
            "quality": quality,
            "resolution": [int(width), int(height)],
        },
        "lot": {
            "texture": settings.ground_texture,
            "markings": bool(settings.show_bays),
            "aisle_width_m": float(settings.aisle_width),
            "bay_depth_m": float(settings.bay_depth),
            "bay_width_m": float(settings.bay_width),
        },
        # one entry per recorded camera, in recording order; the key in ``camera``
        # is the one the file names use, the ``name`` is the object it came from
        "cameras": [camera_entry(key, width, height) for key in cameras],
        "camera_names": [str(key) for key in cameras],
        "frame_pattern": FRAME_PATTERN,
        "video_pattern": VIDEO_PATTERN,
        # the vehicle is the same for every camera, so its geometry appears once
        "vehicle": vehicle.block(
            length=float(settings.car_length), width=float(settings.car_width),
            height=float(settings.car_height),
            clearance=float(settings.car_clearance)),
        "frames_csv": "frames.csv",
        "video": video,
        "video_encode": dict(video_encode) if video_encode else {},
        "time_base": "simulated: t = frame / fps, there is no absolute clock",
    }


class ClipJob:
    """A stepwise clip render shared by the modal operator and the sync API.

    ``start()`` applies the render settings (quality preset, device, every
    recorded camera's shader), ``step()`` renders exactly one frame - one still
    per recorded camera - and ``finish()`` writes the truth, the videos and the
    zip before restoring everything.  ``cancel()`` restores without writing
    anything.  The caller chooses the cadence: a modal timer in the UI, a plain
    loop in tests / headless runs.
    """

    def __init__(self, context, settings, filepath: str = "", directory: str = "",
                 samples: int = 0, encode: bool = True, pack: bool = True,
                 keep_frames: Optional[bool] = None) -> None:
        self.context = context
        self.scene = context.scene
        self.settings = settings
        self.filepath = filepath
        self.directory = directory
        self.samples = int(samples)
        self.encode = bool(encode)
        self.pack = bool(pack)
        # ``None`` = take the scene's setting, so every caller (the operator, the
        # script API, tests) agrees without passing it along by hand
        self.keep_frames = (bool(getattr(settings, "clip_keep_frames", True))
                            if keep_frames is None else bool(keep_frames))
        self.messages: List[str] = []
        self.plan = None
        self.frames: List = []
        #: the camera keys this job records, in ``avm_cameras.CAMERAS`` order
        self.cameras: List[str] = []
        #: their vehicle-frame mount poses, captured once in ``start()``
        self.mounts: Dict[str, drive_path.Mount] = {}
        self.total = 0
        self.done = 0
        self.written: List[str] = []
        self.quality = ""
        self.device = "CPU"
        self.sample_count = 0
        #: ``clip.json``'s ``video`` - empty unless there is exactly one camera
        self.video = ""
        self._camera_objects: Dict[str, bpy.types.Object] = {}
        self._saved: Dict = {}
        self._other_lights: List = []
        self._started = False
        self._finished = False
        self._owns_directory = False
        self._t0 = 0.0
        #: mirrored from the render scene in ``start()`` - the encoder must not
        #: colour the stills a second time (see ``encode_video``)
        self.view_transform = OUTPUT_VIEW_TRANSFORM
        self.look = OUTPUT_LOOK

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> "ClipJob":
        if self._started:
            return self
        self.cameras = list(self.settings.recorded_cameras())
        if not self.cameras:
            raise RuntimeError(
                "no camera is enabled for recording (tick at least one in Cameras)")
        missing = [key for key in self.cameras
                   if bpy.data.objects.get(builder.camera_name(key)) is None]
        if missing:
            raise RuntimeError(
                "the Drive Scene has no " + ", ".join(missing)
                + " camera (build it first)")
        self._camera_objects = {key: bpy.data.objects[builder.camera_name(key)]
                                for key in self.cameras}
        self.mounts = camera_mounts(self.cameras)
        self.video = video_field(self.cameras) if self.encode else ""
        self.plan = self.settings.plan()
        self.frames = list(self.plan.frames)
        self.total = len(self.frames)
        if not self.directory:
            self.directory = tempfile.mkdtemp(prefix="drive_clip_")
            self._owns_directory = True
        os.makedirs(self.directory, exist_ok=True)
        # the stills are written with this pair, so the encoder scene has to use
        # the same one (read before _apply touches anything; it never touches the
        # view settings, but reading early keeps that an implementation detail)
        self.view_transform = self.scene.view_settings.view_transform
        self.look = self.scene.view_settings.look
        try:
            self._apply()
        except Exception:
            self._restore()
            raise
        self.done = 0
        self._t0 = time.time()
        self._started = True
        return self

    def _apply(self) -> None:
        scene = self.scene
        render = scene.render
        cycles = scene.cycles
        quality = str(getattr(self.settings, "clip_quality", "draft"))
        preset = drive_properties.clip_quality_preset(quality)
        if self.samples > 0:
            preset["samples"] = self.samples
        self.quality = quality
        self.sample_count = int(preset["samples"])
        self.device = resolve_device(
            str(getattr(self.settings, "clip_device", "cpu")), self.messages)
        # the file around the scene usually carries lights of its own (a fresh
        # Blender scene has a point light); they would change the clip's lighting
        # from one recording to the next, so they are muted for the duration
        self._other_lights = [
            (light, light.hide_render) for light in scene.objects
            if light.type == "LIGHT"
            and not light.name.startswith(builder.LIGHT_PREFIX)]
        self._saved = {
            "filepath": render.filepath,
            "file_format": render.image_settings.file_format,
            "engine": render.engine,
            "device": cycles.device,
            "samples": cycles.samples,
            "denoising": cycles.use_denoising,
            "adaptive": cycles.use_adaptive_sampling,
            "adaptive_threshold": cycles.adaptive_threshold,
            "max_bounces": cycles.max_bounces,
            "diffuse_bounces": cycles.diffuse_bounces,
            "glossy_bounces": cycles.glossy_bounces,
            "caustics_reflective": cycles.caustics_reflective,
            "caustics_refractive": cycles.caustics_refractive,
            "persistent": render.use_persistent_data,
            "threads_mode": render.threads_mode,
            "threads": render.threads,
            "frame_start": scene.frame_start,
            "frame_end": scene.frame_end,
            "frame_step": scene.frame_step,
            "camera": scene.camera,
            "frame": scene.frame_current,
        }
        if render.engine != "CYCLES":
            render.engine = "CYCLES"
        cycles.device = self.device
        cycles.samples = self.sample_count
        cycles.use_denoising = bool(preset.get("denoise", self._saved["denoising"]))
        cycles.max_bounces = int(preset.get("max_bounces", self._saved["max_bounces"]))
        cycles.diffuse_bounces = int(preset.get("diffuse_bounces",
                                                self._saved["diffuse_bounces"]))
        cycles.glossy_bounces = int(preset.get("glossy_bounces",
                                               self._saved["glossy_bounces"]))
        if "adaptive_threshold" in preset:
            cycles.use_adaptive_sampling = True
            cycles.adaptive_threshold = float(preset["adaptive_threshold"])
        if "caustics" in preset:
            cycles.caustics_reflective = bool(preset["caustics"])
            cycles.caustics_refractive = bool(preset["caustics"])
        # the scene is static apart from the vehicle, so caching it across the
        # per-frame stills is free quality-wise and saves the sync every frame
        render.use_persistent_data = True
        render.threads_mode = "AUTO"
        for light, _ in self._other_lights:
            light.hide_render = True
        render.image_settings.file_format = "PNG"
        # every recorded camera's shader and Cycles parameters, once: switching
        # ``scene.camera`` per frame is then the only per-frame camera work
        for key in self.cameras:
            camera = self._camera_objects[key]
            ok, messages = apply_mod.apply_settings(
                camera.data, camera.data.opencv_cam, scene)
            if not ok:
                raise RuntimeError(f"{key} camera: " + "; ".join(messages))
        scene.camera = self._camera_objects[self.cameras[0]]
        scene.frame_start = self.frames[0].index
        scene.frame_end = self.frames[-1].index
        scene.frame_step = 1

    def _restore(self) -> None:
        if not self._saved:
            return
        scene = self.scene
        render = scene.render
        cycles = scene.cycles
        saved = self._saved
        self._saved = {}
        for light, hidden in self._other_lights:
            light.hide_render = hidden
        render.filepath = saved["filepath"]
        render.image_settings.file_format = saved["file_format"]
        render.engine = saved["engine"]
        cycles.device = saved["device"]
        cycles.samples = saved["samples"]
        cycles.use_denoising = saved["denoising"]
        cycles.use_adaptive_sampling = saved["adaptive"]
        cycles.adaptive_threshold = saved["adaptive_threshold"]
        cycles.max_bounces = saved["max_bounces"]
        cycles.diffuse_bounces = saved["diffuse_bounces"]
        cycles.glossy_bounces = saved["glossy_bounces"]
        cycles.caustics_reflective = saved["caustics_reflective"]
        cycles.caustics_refractive = saved["caustics_refractive"]
        render.use_persistent_data = saved["persistent"]
        render.threads_mode = saved["threads_mode"]
        render.threads = saved["threads"]
        scene.frame_step = saved["frame_step"]
        scene.frame_start = saved["frame_start"]
        scene.frame_end = saved["frame_end"]
        scene.camera = saved["camera"]
        scene.frame_set(saved["frame"])

    # -- stepping -----------------------------------------------------------
    def step(self) -> None:
        """Render the next frame: exactly one still per recorded camera.

        The cameras loop *inside* the frame, so ``done`` / ``total`` stay a frame
        count and the ETA a caller shows does not have to divide by the number of
        cameras to stay meaningful.
        """
        if self.done >= self.total:
            return
        frame = self.frames[self.done]
        render = self.scene.render
        self.scene.frame_set(frame.index)
        for key in self.cameras:
            self.scene.camera = self._camera_objects[key]
            render.filepath = os.path.join(self.directory, frame_name(frame.index, key))
            result = bpy.ops.render.render(write_still=True)
            if "CANCELLED" in result:
                raise RenderCancelled("the clip render was cancelled")
            self.written.append(render.filepath)
        self.done += 1

    # -- progress -----------------------------------------------------------
    def elapsed(self) -> float:
        return max(0.0, time.time() - self._t0) if self._t0 else 0.0

    def eta_seconds(self) -> Optional[float]:
        if self.done <= 0 or self.done >= self.total:
            return None
        return self.elapsed() / self.done * (self.total - self.done)

    def status_text(self) -> str:
        return progress_text(self.done, self.total, self.eta_seconds())

    # -- finish / cancel ----------------------------------------------------
    def _write_truth(self) -> Dict:
        csv_path = os.path.join(self.directory, "frames.csv")
        with open(csv_path, "w", encoding="utf-8") as handle:
            handle.write(drive_path.csv_text(self.plan, self.mounts))
        meta = clip_meta(self.scene, self.settings, self.plan, self.sample_count,
                         self.cameras, video=self.video, quality=self.quality,
                         device=self.device, video_encode=self._video_recipe())
        meta_path = os.path.join(self.directory, "clip.json")
        with open(meta_path, "w", encoding="utf-8") as handle:
            handle.write(json.dumps(meta, indent=2) + "\n")
        self.written.extend([csv_path, meta_path])
        return meta

    def _video_recipe(self) -> Dict:
        """The mp4's encode recipe, or ``{}`` when there is no video."""
        if not self.encode:
            return {}
        return video_encode_meta(self.plan.fps, self.view_transform, self.look)

    def _drop_frames(self) -> None:
        """Delete the stills once they are encoded (``clip_keep_frames`` off).

        The video is the artifact a validation run consumes; the stills are only
        the encoder's input and they are ~100x the size (measured on the default
        clip: 138 MB of PNG against 1.0 MB of H.264), so a clip that only feeds
        the video pipeline has no reason to carry them.  Never runs without a
        video - without one the stills *are* the artifact.
        """
        if self.keep_frames or not self.encode:
            return
        for path in list(self.written):
            if path.endswith(".png"):
                try:
                    os.remove(path)
                except OSError:
                    pass
        self.written = [path for path in self.written if os.path.exists(path)]

    def _pack(self) -> None:
        parent = os.path.dirname(os.path.abspath(self.filepath))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with zipfile.ZipFile(self.filepath, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in sorted(os.listdir(self.directory)):
                archive.write(os.path.join(self.directory, name), arcname=name)

    def finish(self) -> Dict:
        if self._finished:
            return self.report()
        try:
            if self.encode:
                for key in self.cameras:
                    encode_video(self.directory, self.plan, key,
                                 os.path.join(self.directory, video_name(key)),
                                 view_transform=self.view_transform, look=self.look)
            self._drop_frames()
            meta = self._write_truth()
            if self.pack:
                self._pack()
        finally:
            self._restore()
            if self._owns_directory:
                shutil.rmtree(self.directory, ignore_errors=True)
        self._finished = True
        return {"directory": self.directory, "frames": self.total,
                "cameras": list(self.cameras),
                "files": list(self.written), "plan": self.plan, "meta": meta,
                "filepath": self.filepath, "quality": self.quality,
                "device": self.device, "messages": list(self.messages)}

    def cancel(self) -> None:
        self._restore()
        if self._owns_directory:
            shutil.rmtree(self.directory, ignore_errors=True)
        self._finished = True

    def report(self) -> Dict:
        return {"directory": self.directory, "frames": self.total,
                "cameras": list(self.cameras),
                "files": list(self.written), "plan": self.plan,
                "filepath": self.filepath, "quality": self.quality,
                "device": self.device, "messages": list(self.messages)}


def video_encode_meta(fps: float, view_transform: str = OUTPUT_VIEW_TRANSFORM,
                      look: str = OUTPUT_LOOK) -> Dict:
    """The video's encode recipe - what a regression needs to reproduce the mp4.

    The stills are the ground truth and the scenario that produced them is in the
    same file, so a harness that has to separate "the algorithm drifted" from
    "the encoder drifted" can re-render the PNG sequence and re-encode it with
    exactly these settings.

    The video is *lossy* on purpose: it is what a validation run consumes, not an
    archival format.  Measured on the default clip (H.264 CRF HIGH at 1920x1080):
    1.0 MB / 46.7 dB whereas the stills are 138 MB, and the transparent-chassis
    reconstruction is indistinguishable between the two.
    """
    return {
        "container": VIDEO_FORMAT,
        "codec": VIDEO_CODEC,
        "constant_rate_factor": VIDEO_CRF,
        "view_transform": view_transform,
        "look": look,
        "fps": float(fps),
        # the Blender version pins the bundled FFmpeg build - it is the only
        # handle on the encoder that actually exists (4.5 dropped
        # bpy.app.ffmpeg_version, so there is no separate version string to read)
        "blender": bpy.app.version_string,
    }


def still_size(path: str) -> tuple:
    """The pixel size of a rendered still, read from the file.

    Measured rather than taken from the scene: the encoder has to lay the stills
    out on a canvas of *their* size, and the file is the only thing that knows it
    for sure (see :func:`encode_video`).
    """
    image = bpy.data.images.load(path)
    try:
        width, height = int(image.size[0]), int(image.size[1])
    finally:
        bpy.data.images.remove(image)
    return width, height


def encode_video(directory: str, plan, camera: str, output_path: str,
                 view_transform: str = OUTPUT_VIEW_TRANSFORM,
                 look: str = OUTPUT_LOOK) -> str:
    """Encode one camera's PNG sequence into an H.264 mp4 with Blender's FFmpeg.

    A throwaway sequencer scene turns the stills into a movie **without
    re-rendering** the 3D scene; it is removed again whatever happens.  One
    camera per call - the sequences cannot be told apart by Blender's sequencer
    once they are strips, so one scene per container is the honest mapping.

    The canvas is set to the **stills' own** pixel size.  A fresh scene is
    1920x1080, and a sequencer strip is drawn at its native size inside that
    canvas rather than scaled to fill it, so leaving the default turns a 96x72
    clip into a 1920x1080 container with a 96x72 picture floating in the middle
    of a black frame - measured, and invisible at 1920x1080 where the default
    happens to be right.  The consumer reads K / D for the render resolution, so
    a container of another size would put the geometry and the pixels at
    different scales.

    ``view_transform`` / ``look`` must be the pair the stills were *written* with
    (``ClipJob`` mirrors its render scene).  A fresh scene defaults to **AgX**, so
    without this the finished, already display-referred pixels would be tone-mapped
    a second time: measured -14 dB PSNR and up to -33 LSB in the highlights, and
    the file came out 14% *larger* as well.
    """
    first = frame_name(plan.frames[0].index, camera)
    width, height = still_size(os.path.join(directory, first))
    if width <= 0 or height <= 0:
        raise RuntimeError(f"cannot read the still size of {first}")
    scene = bpy.data.scenes.new("__drive_encode__")
    try:
        scene.render.resolution_x = width
        scene.render.resolution_y = height
        scene.render.resolution_percentage = 100
        scene.render.fps = max(1, int(round(plan.fps)))
        scene.render.fps_base = 1.0
        scene.view_settings.view_transform = view_transform
        scene.view_settings.look = look
        compat.enable_movie_output(scene.render.image_settings)
        scene.render.ffmpeg.format = VIDEO_FORMAT
        scene.render.ffmpeg.codec = VIDEO_CODEC
        scene.render.ffmpeg.constant_rate_factor = VIDEO_CRF
        scene.render.filepath = output_path
        scene.render.use_sequencer = True
        scene.frame_start = plan.frames[0].index
        scene.frame_end = plan.frames[-1].index
        strips = compat.sequence_strips(scene)
        strip = strips.new_image("clip", os.path.join(directory, first),
                                 1, plan.frames[0].index)
        for frame in plan.frames[1:]:
            strip.elements.append(frame_name(frame.index, camera))
        result = bpy.ops.render.render(animation=True, scene=scene.name)
        if "CANCELLED" in result:
            raise RenderCancelled("the clip encoding was cancelled")
    finally:
        bpy.data.scenes.remove(scene)
    return output_path


def render_clip(context, settings, directory: str, samples: int = 0) -> Dict:
    """Render every frame of the plan, for every recorded camera, into ``directory``.

    No video and no zip - the PNG sequence is the artifact, which is what
    ``scripts/check_clip_reproducibility.py`` and the test suite compare.  The
    render cost comes from ``settings.clip_quality``; a positive ``samples``
    overrides its sample count (tests / scripts).  Only the render cost changes -
    the output size and the cameras (K / D) are the same at every quality.  Every
    render setting that is touched is saved and restored, so recording never
    changes the user's setup.
    """
    job = ClipJob(context, settings, directory=directory, samples=samples,
                  encode=False, pack=False)
    job.start()
    try:
        while job.done < job.total:
            job.step()
        return job.finish()
    except RenderCancelled:
        job.cancel()
        raise


def export_zip(context, settings, filepath: str, samples: int = 0) -> Dict:
    """Render the clip, encode one video per camera and pack it all into one zip.

    The zip holds ``frames.csv`` (per-frame speed, vehicle pose and each camera's
    world pose), ``clip.json`` (K / D, mount poses, vehicle geometry, drive and
    render parameters, and the mp4s' encode recipe), one ``<camera>.mp4`` per
    recorded camera and - unless ``settings.clip_keep_frames`` is off - the
    ``frame_%04d_<camera>.png`` sequence.  The PNGs are written to a temporary
    directory and removed once the zip is closed.
    """
    job = ClipJob(context, settings, filepath=filepath, samples=samples,
                  encode=True, pack=True)
    job.start()
    try:
        while job.done < job.total:
            job.step()
        return job.finish()
    except RenderCancelled:
        job.cancel()
        return {"filepath": filepath, "frames": 0, "plan": settings.plan(),
                "cancelled": True}
