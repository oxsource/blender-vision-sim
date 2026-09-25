"""Road Scene operators: add / rebuild / reset / remove / export clip."""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper

from ....core.scenes import road_path
from ..base import has_scene
from .. import view as view_mod
from . import DEFINITION, builder, controller, recording


def _scene(context):
    return getattr(context, "scene", None)


def _settings(context):
    return controller.settings_of(context)


class _RoadOperator:
    @classmethod
    def poll(cls, context):
        return _scene(context) is not None


class _RoadSceneOperator(_RoadOperator):
    @classmethod
    def poll(cls, context):
        return _scene(context) is not None and has_scene(context, DEFINITION)


_RESET_KEYS = (
    "track_preset",
    "straight_length", "curve_radius", "ramp_rise", "ramp_length", "road_width",
    "shoulder_width", "ground_texture", "show_markings", "show_crosswalk",
    "parking", "parking_bays", "parking_speed",
    "pedestrians", "trees", "lamps", "signs",
    "animate_pedestrians", "pedestrian_speed",
    "light_energy",
    "car_length", "car_width", "car_height", "car_clearance",
    "drive_speed", "drive_profile", "drive_accel", "drive_decel", "slow_speed",
    "drive_fps", "drive_direction", "drive_loops", "drive_segment",
    "active_camera", "clip_quality", "clip_device", "clip_keep_frames",
)


class OPENCV_CAM_OT_road_add_scene(_RoadOperator, bpy.types.Operator):
    """Create the Road Scene (closed loop, props, minibus, four cameras, drive)"""

    bl_idname = "opencv_cam.road_add_scene"
    bl_label = "Road Scene"
    bl_description = (
        "Create the Road Scene: a closed test loop with straights, curves and "
        "up/down ramps, roadside props (parked cars, pedestrians, trees, lamps, "
        "signs), and a minibus carrying the four OpenCV cameras at their real "
        "mount poses; the drive is keyframed on the vehicle"
    )
    bl_options = {"REGISTER", "UNDO"}

    reset: BoolProperty(
        name="Reset", default=False,
        description="Rebuild from the defaults even if the scene already exists")

    def execute(self, context):
        scene = _scene(context)
        settings = _settings(context)
        if settings is None:
            self.report({"ERROR"}, "scene.road_scene is not registered")
            return {"CANCELLED"}
        if settings.root is not None and not self.reset:
            self.report({"WARNING"}, "a Road Scene already exists (use Rebuild or Reset)")
            return {"CANCELLED"}
        if settings.root is not None and self.reset:
            builder.remove(scene, settings)
        created = builder.build(scene, settings)
        view_mod.frame(view_mod.scene_objects(DEFINITION), DEFINITION.view)
        for message in created.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, f"Road Scene ready: {road_path.summary(created['plan'])}")
        return {"FINISHED"}


class OPENCV_CAM_OT_road_rebuild(_RoadSceneOperator, bpy.types.Operator):
    """Rebuild the track, the props and the drive from the current settings"""

    bl_idname = "opencv_cam.road_rebuild"
    bl_label = "Rebuild"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        created = controller.rebuild_now(context)
        if created is None:
            self.report({"WARNING"}, "no Road Scene to rebuild")
            return {"CANCELLED"}
        for message in created.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, f"Road Scene rebuilt: {road_path.summary(created['plan'])}")
        return {"FINISHED"}


class OPENCV_CAM_OT_road_reset_defaults(_RoadSceneOperator, bpy.types.Operator):
    """Put every Road Scene setting back to its default"""

    bl_idname = "opencv_cam.road_reset_defaults"
    bl_label = "Reset Defaults"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        for name in _RESET_KEYS:
            settings.property_unset(name)
        settings.reset_cameras()
        controller.rebuild_now(context)
        self.report({"INFO"}, "Road Scene defaults restored")
        return {"FINISHED"}


class OPENCV_CAM_OT_road_sync_cameras(_RoadSceneOperator, bpy.types.Operator):
    """Copy the live AVM Scene cameras onto the Road Scene cameras"""

    bl_idname = "opencv_cam.road_sync_cameras"
    bl_label = "Sync Cameras from AVM Scene"
    bl_description = (
        "Overwrite the four Road cameras' intrinsics and mount poses with those "
        "of the AVM_Cam_* objects in this file. The reproducible source stays the "
        "bundled calibration preset, so a fresh Road Scene reads that instead"
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        from ....core.scenes import avm_cameras
        scene = _scene(context)
        if not any(bpy.data.objects.get(avm_cameras.object_name("AVM_Cam_", key))
                   for key in avm_cameras.CAMERAS):
            self.report({"WARNING"}, "no AVM Scene in this file - nothing to sync from")
            return {"CANCELLED"}
        for message in builder.sync_cameras_from_avm(_settings(context), scene):
            self.report({"INFO"}, message)
        return {"FINISHED"}


class OPENCV_CAM_OT_road_remove_scene(_RoadSceneOperator, bpy.types.Operator):
    """Remove the Road Scene (deletes its objects; the panels disappear)"""

    bl_idname = "opencv_cam.road_remove_scene"
    bl_label = "Remove Road Scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        removed = builder.remove(_scene(context), _settings(context))
        self.report({"INFO"}, f"Road Scene removed ({removed} objects)")
        return {"FINISHED"}


class OPENCV_CAM_OT_road_export_zip(_RoadSceneOperator, bpy.types.Operator, ExportHelper):
    """Export the whole clip - PNG sequence, video, CSV and JSON - as one zip"""

    bl_idname = "opencv_cam.road_export_zip"
    bl_label = "Export Clip"
    bl_description = (
        "Render the whole drive and pack it into one zip: one mp4 per camera "
        "(H.264), frames.csv (per-frame speed, 3D vehicle pose and each camera's "
        "world pose, with the segment label), clip.json (K / D, mount pose, loop "
        "and segment description, render parameters) and - while Clip Keep "
        "Frames is on - the PNG sequence"
    )
    bl_options = {"REGISTER"}

    filename_ext = ".zip"
    filter_glob: StringProperty(default="*.zip", options={"HIDDEN"})
    samples: IntProperty(
        name="Samples", default=0, min=0, max=4096,
        description="Override the Clip Quality sample count; 0 uses the preset")

    def invoke(self, context, event):
        if not self.filepath:
            self.filepath = recording.default_filename()
        return super().invoke(context, event)

    def execute(self, context):
        settings = _settings(context)
        plan = settings.plan()
        if bpy.app.background or getattr(context, "window", None) is None:
            return self._run_sync(context, settings, plan)
        return self._start_modal(context, settings, plan)

    def _run_sync(self, context, settings, plan):
        try:
            report = recording.export_zip(context, settings, self.filepath,
                                          samples=self.samples)
        except Exception as exc:
            settings.clip_status = f"error: {exc}"
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        return self._report_result(settings, plan, report)

    def _start_modal(self, context, settings, plan):
        try:
            job = recording.ClipJob(recording.PROFILE, context, settings,
                                    filepath=self.filepath, samples=self.samples,
                                    encode=True, pack=True)
            job.start()
        except Exception as exc:
            settings.clip_status = f"error: {exc}"
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self._job = job
        self._settings = settings
        self._plan = plan
        self._timer = context.window_manager.event_timer_add(0.01, window=context.window)
        context.window_manager.modal_handler_add(self)
        context.window_manager.progress_begin(0, max(1, job.total))
        settings.clip_status = job.status_text()
        for message in job.messages:
            self.report({"WARNING"}, message)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        window_manager = context.window_manager
        if event.type == "ESC":
            return self._cancel(context, "cancelled by ESC")
        if event.type == "TIMER":
            job = self._job
            try:
                if job.done >= job.total:
                    report = job.finish()
                    self._teardown(context)
                    return self._report_result(self._settings, self._plan, report)
                job.step()
            except recording.RenderCancelled:
                return self._cancel(context, "render cancelled")
            except Exception as exc:
                self._job.cancel()
                self._teardown(context)
                self._settings.clip_status = f"error: {exc}"
                self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
                return {"CANCELLED"}
            window_manager.progress_update(job.done)
            self._settings.clip_status = job.status_text()
            _redraw_ui(context)
        return {"RUNNING_MODAL"}

    def _cancel(self, context, reason):
        self._job.cancel()
        self._teardown(context)
        self._settings.clip_status = "cancelled"
        self.report({"INFO"}, f"Export cancelled ({reason})")
        return {"CANCELLED"}

    def _teardown(self, context):
        window_manager = context.window_manager
        window_manager.progress_end()
        timer = getattr(self, "_timer", None)
        if timer is not None:
            window_manager.event_timer_remove(timer)
            self._timer = None

    def _report_result(self, settings, plan, report):
        if report.get("cancelled"):
            settings.clip_status = "cancelled"
            self.report({"INFO"}, "Export cancelled")
            return {"CANCELLED"}
        settings.clip_status = (f"{report['frames']} frames -> {report['filepath']} "
                                f"({road_path.summary(plan)})")
        for message in report.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, settings.clip_status)
        return {"FINISHED"}


def _redraw_ui(context) -> None:
    try:
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                area.tag_redraw()
    except Exception:
        pass


_CLASSES = (
    OPENCV_CAM_OT_road_add_scene,
    OPENCV_CAM_OT_road_rebuild,
    OPENCV_CAM_OT_road_reset_defaults,
    OPENCV_CAM_OT_road_remove_scene,
    OPENCV_CAM_OT_road_sync_cameras,
    OPENCV_CAM_OT_road_export_zip,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
