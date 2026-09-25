"""The clip recorder, shared by every scene that writes a drive-style clip.

The Drive Scene and the Road Scene render the same artifact - one still per
camera per frame, one H.264 video per camera, ``frames.csv`` with the per-frame
truth and its **camera world poses**, and ``clip.json`` describing the whole
run - so the engine lives here once and each scene supplies a
:class:`ClipProfile` naming its own pieces (object prefixes, plan module,
``clip.json`` blocks, default file name).

Nothing in this module knows a car park from a road; the scene-specific fields
are the profile's callables.  That is what keeps the two exports on one contract
instead of two that only a test keeps aligned.
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
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import bpy

from .. import apply as apply_mod
from .. import compat

__all__ = [
    "ClipProfile", "ClipJob", "RenderCancelled",
    "clip_quality_preset", "CLIP_QUALITY_PRESETS", "CLIP_QUALITIES",
    "frame_name", "video_name", "default_filename",
    "compute_device_type", "gpu_can_render_osl_camera", "enabled_osl_gpu_available",
    "resolve_device", "format_duration", "progress_text",
    "camera_mounts", "camera_size", "camera_entry", "video_field", "clip_meta",
    "video_encode_meta", "still_size", "encode_video", "render_clip", "export_zip",
    "FRAME_PATTERN", "VIDEO_PATTERN", "OUTPUT_VIEW_TRANSFORM", "OUTPUT_LOOK",
    "VIDEO_FORMAT", "VIDEO_CODEC", "VIDEO_CRF",
]

#: how a clip's stills / videos are named; ``<camera>`` is the camera *key*
FRAME_PATTERN = "frame_%04d_<camera>.png"
VIDEO_PATTERN = "<camera>.mp4"

#: The view transform / look the stills are written with.  ``encode_video``
#: mirrors the *render* scene's pair instead of hardcoding these, so an encoder
#: scene can never colour finished pixels a second time.
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

#: Clip export quality presets.  Only the render *cost* changes - the output
#: size and the camera (K / D) are identical at every setting.
CLIP_QUALITIES = [
    ("draft", "Draft", "Fastest: 8 samples, denoised, 2 light bounces"),
    ("balanced", "Balanced", "24 samples, denoised, 4 light bounces"),
    ("high", "High", "64 samples, the scene's own denoise / bounce settings"),
]

CLIP_QUALITY_PRESETS = {
    "draft": {
        "samples": 8,
        "denoise": True,
        "max_bounces": 2,
        "diffuse_bounces": 1,
        "glossy_bounces": 1,
        "adaptive_threshold": 0.1,
        "caustics": False,
    },
    "balanced": {
        "samples": 24,
        "denoise": True,
        "max_bounces": 4,
        "diffuse_bounces": 2,
        "glossy_bounces": 2,
        "adaptive_threshold": 0.05,
        "caustics": False,
    },
    "high": {
        "samples": 64,
    },
}


def clip_quality_preset(key: str) -> dict:
    """The Cycles settings for a quality key (unknown keys fall back to balanced)."""
    return dict(CLIP_QUALITY_PRESETS.get(str(key), CLIP_QUALITY_PRESETS["balanced"]))


class RenderCancelled(Exception):
    """The user aborted the render (ESC / the render window was closed)."""


@dataclass(frozen=True)
class ClipProfile:
    """What makes one scene's clip different from another's.

    The recorder is otherwise identical, so a new scene is "one profile", not a
    second copy of this module.
    """

    format: str                     #: ``clip.json``'s ``format`` field
    version: int                    #: ``clip.json``'s ``version`` field
    default_name: str               #: default ``[Export Clip]`` zip stem
    light_prefix: str               #: the scene's own light object prefix
    camera_name: Callable[[str], str]   #: key -> camera object name
    csv_text: Callable[[object, Dict], str]   #: frames.csv writer
    motion_meta: Callable[[object, object], Dict]   #: the drive / motion block
    scene_meta: Callable[[object, object], Dict]    #: the layout block
    vehicle_block: Callable[[object], Dict]         #: clip.json's vehicle block
    camera_model: str = "fisheye"
    #: whether the baked scene is static apart from the vehicle.  Cycles may
    #: then cache it across the per-frame stills; a scene with animated props
    #: must say no or the props freeze.
    persistent_data: Callable[[object], bool] = lambda settings: True


def default_filename(profile: ClipProfile) -> str:
    """The default ``[Export Clip…]`` file name (``<default_name>.zip``)."""
    return f"{profile.default_name}.zip"


def frame_name(index: int, camera: str) -> str:
    """One still's file name: ``frame_0000_front.png``."""
    return f"frame_{index:04d}_{camera}.png"


def video_name(camera: str) -> str:
    """One camera's container name: ``front.mp4``."""
    return f"{camera}.mp4"


def compute_device_type() -> str:
    """The Cycles preference's active compute device type (``"NONE"`` if unset)."""
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


def camera_mounts(profile: ClipProfile, cameras: Sequence[str]) -> Dict:
    """Each camera's fixed pose in the vehicle frame (metres, degrees).

    Read from the camera **objects**, not from the preset: the object transform
    is the single source of a mount pose, so a pose the user adjusted in
    ``CV Extrinsics`` is what the clip records.
    """
    from ...core.scenes import drive_path
    mounts: Dict[str, drive_path.Mount] = {}
    for key in cameras:
        camera = bpy.data.objects.get(profile.camera_name(key))
        if camera is None:
            mounts[key] = drive_path.Mount()
            continue
        mounts[key] = drive_path.Mount(
            location=tuple(float(value) for value in camera.location),
            rotation_deg=tuple(math.degrees(float(value)) for value in camera.rotation_euler),
        )
    return mounts


def camera_size(profile: ClipProfile, key: str) -> Tuple[int, int]:
    """A recorded camera's configured image size, in pixels."""
    camera = bpy.data.objects.get(profile.camera_name(key))
    if camera is None:
        return 0, 0
    intrinsics = camera.data.opencv_cam.intrinsics
    return max(1, int(intrinsics.image_width)), max(1, int(intrinsics.image_height))


def camera_entry(profile: ClipProfile, key: str, width: int, height: int) -> Dict:
    """One ``clip.json`` ``cameras`` entry: what this camera is and where it sits."""
    name = profile.camera_name(key)
    entry: Dict = {
        "name": name,
        "camera": key,
        "model": profile.camera_model,
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
    """``clip.json``'s ``video``: a container name only for a single-camera clip."""
    return video_name(cameras[0]) if len(cameras) == 1 else ""


def clip_meta(profile: ClipProfile, scene: bpy.types.Scene, settings, plan,
              samples: int, cameras: Sequence[str], video: str = "",
              quality: str = "", device: str = "",
              video_encode: Optional[Dict] = None) -> Dict:
    """The clip's self-description (what was rendered / driven / recorded)."""
    sizes = {key: camera_size(profile, key) for key in cameras}
    width, height = sizes[cameras[0]] if cameras else (0, 0)
    meta: Dict = {
        "format": profile.format,
        "version": profile.version,
        "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "fps": plan.fps,
        "frames": len(plan.frames),
        "duration_s": round(plan.duration, 6),
    }
    meta.update(profile.motion_meta(settings, plan))
    meta["render"] = {
        "engine": "CYCLES",
        "device": device,
        "samples": int(samples),
        "quality": quality,
        "resolution": [int(width), int(height)],
    }
    meta.update(profile.scene_meta(settings, plan))
    # one entry per recorded camera, in recording order
    meta["cameras"] = [camera_entry(profile, key, *sizes[key]) for key in cameras]
    meta["camera_names"] = [str(key) for key in cameras]
    meta["frame_pattern"] = FRAME_PATTERN
    meta["video_pattern"] = VIDEO_PATTERN
    # the vehicle is the same for every camera, so its geometry appears once
    meta["vehicle"] = profile.vehicle_block(settings)
    meta["frames_csv"] = "frames.csv"
    meta["video"] = video
    meta["video_encode"] = dict(video_encode) if video_encode else {}
    meta["time_base"] = "simulated: t = frame / fps, there is no absolute clock"
    return meta


class ClipJob:
    """A stepwise clip render shared by the modal operator and the sync API.

    ``start()`` applies the render settings, ``step()`` renders exactly one frame
    - one still per recorded camera - and ``finish()`` writes the truth, the
    videos and the zip before restoring everything.  ``cancel()`` restores
    without writing anything.  The caller chooses the cadence: a modal timer in
    the UI, a plain loop in tests / headless runs.
    """

    def __init__(self, profile: ClipProfile, context, settings,
                 filepath: str = "", directory: str = "", samples: int = 0,
                 encode: bool = True, pack: bool = True,
                 keep_frames: Optional[bool] = None) -> None:
        self.profile = profile
        self.context = context
        self.scene = context.scene
        self.settings = settings
        self.filepath = filepath
        self.directory = directory
        self.samples = int(samples)
        self.encode = bool(encode)
        self.pack = bool(pack)
        self.keep_frames = (bool(getattr(settings, "clip_keep_frames", True))
                            if keep_frames is None else bool(keep_frames))
        self.messages: List[str] = []
        self.plan = None
        self.frames: List = []
        self.cameras: List[str] = []
        self.mounts: Dict = {}
        self.total = 0
        self.done = 0
        self.written: List[str] = []
        self.quality = ""
        self.device = "CPU"
        self.sample_count = 0
        self.video = ""
        self._camera_objects: Dict[str, bpy.types.Object] = {}
        self._saved: Dict = {}
        self._other_lights: List = []
        self._started = False
        self._finished = False
        self._owns_directory = False
        self._t0 = 0.0
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
                   if bpy.data.objects.get(self.profile.camera_name(key)) is None]
        if missing:
            raise RuntimeError(
                "the scene has no " + ", ".join(missing) + " camera (build it first)")
        self._camera_objects = {
            key: bpy.data.objects[self.profile.camera_name(key)]
            for key in self.cameras}
        self.mounts = camera_mounts(self.profile, self.cameras)
        self.video = video_field(self.cameras) if self.encode else ""
        self.plan = self.settings.plan()
        self.frames = list(self.plan.frames)
        self.total = len(self.frames)
        if not self.directory:
            self.directory = tempfile.mkdtemp(prefix=f"{self.profile.default_name}_")
            self._owns_directory = True
        os.makedirs(self.directory, exist_ok=True)
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
        preset = clip_quality_preset(quality)
        if self.samples > 0:
            preset["samples"] = self.samples
        self.quality = quality
        self.sample_count = int(preset["samples"])
        self.device = resolve_device(
            str(getattr(self.settings, "clip_device", "cpu")), self.messages)
        self._other_lights = [
            (light, light.hide_render) for light in scene.objects
            if light.type == "LIGHT"
            and not light.name.startswith(self.profile.light_prefix)]
        self._saved = {
            "filepath": render.filepath,
            "file_format": render.image_settings.file_format,
            "resolution_x": render.resolution_x,
            "resolution_y": render.resolution_y,
            "resolution_percentage": render.resolution_percentage,
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
        # a baked scene may be cached across the per-frame stills; a scene with
        # animated props says no through its profile (else the props freeze)
        render.use_persistent_data = bool(self.profile.persistent_data(self.settings))
        render.threads_mode = "AUTO"
        for light, _ in self._other_lights:
            light.hide_render = True
        render.image_settings.file_format = "PNG"
        for key in self.cameras:
            camera = self._camera_objects[key]
            ok, messages = apply_mod.apply_settings(
                camera.data, camera.data.opencv_cam, scene,
                resolution=camera_size(self.profile, key))
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
        render.resolution_x = saved["resolution_x"]
        render.resolution_y = saved["resolution_y"]
        render.resolution_percentage = saved["resolution_percentage"]
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
        """Render the next frame: exactly one still per recorded camera."""
        if self.done >= self.total:
            return
        frame = self.frames[self.done]
        render = self.scene.render
        self.scene.frame_set(frame.index)
        for key in self.cameras:
            camera = self._camera_objects[key]
            width, height = camera_size(self.profile, key)
            render.resolution_x = width
            render.resolution_y = height
            render.resolution_percentage = 100
            self.scene.camera = camera
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
            handle.write(self.profile.csv_text(self.plan, self.mounts))
        meta = clip_meta(self.profile, self.scene, self.settings, self.plan,
                         self.sample_count, self.cameras, video=self.video,
                         quality=self.quality, device=self.device,
                         video_encode=self._video_recipe())
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
        """Delete the stills once they are encoded (``clip_keep_frames`` off)."""
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
    """The video's encode recipe - what a regression needs to reproduce the mp4."""
    return {
        "container": VIDEO_FORMAT,
        "codec": VIDEO_CODEC,
        "constant_rate_factor": VIDEO_CRF,
        "view_transform": view_transform,
        "look": look,
        "fps": float(fps),
        "blender": bpy.app.version_string,
    }


def still_size(path: str) -> tuple:
    """The pixel size of a rendered still, read from the file."""
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

    The canvas is set to the **stills' own** pixel size: a fresh scene is
    1920x1080 and a sequencer strip is drawn at its native size inside that
    canvas rather than scaled to fill it, so leaving the default turns a 96x72
    clip into a 1920x1080 container with a small picture floating in the middle.

    ``view_transform`` / ``look`` must be the pair the stills were *written*
    with (``ClipJob`` mirrors its render scene): a fresh scene defaults to
    **AgX**, so without this the finished pixels would be tone-mapped twice.
    """
    first = frame_name(plan.frames[0].index, camera)
    width, height = still_size(os.path.join(directory, first))
    if width <= 0 or height <= 0:
        raise RuntimeError(f"cannot read the still size of {first}")
    scene = bpy.data.scenes.new(f"__{_slug(camera)}_encode__")
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


def _slug(text: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in str(text))


def render_clip(profile: ClipProfile, context, settings, directory: str,
                samples: int = 0) -> Dict:
    """Render every frame, for every recorded camera, into ``directory``.

    No video and no zip - the PNG sequence is the artifact, which is what the
    reproducibility probe and the test suite compare.  Every render setting that
    is touched is saved and restored.
    """
    job = ClipJob(profile, context, settings, directory=directory, samples=samples,
                  encode=False, pack=False)
    job.start()
    try:
        while job.done < job.total:
            job.step()
        return job.finish()
    except RenderCancelled:
        job.cancel()
        raise


def export_zip(profile: ClipProfile, context, settings, filepath: str,
               samples: int = 0) -> Dict:
    """Render the clip, encode one video per camera and pack it into one zip."""
    job = ClipJob(profile, context, settings, filepath=filepath, samples=samples,
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
