"""Drive Scene operators: add / rebuild / reset / remove / export clip."""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper

from ....core.scenes import drive_path
from ..base import has_scene
from .. import view as view_mod
from . import DEFINITION, builder, controller, recording


def _scene(context):
    return getattr(context, "scene", None)


def _settings(context):
    return controller.settings_of(context)


class _DriveOperator:
    @classmethod
    def poll(cls, context):
        return _scene(context) is not None


class _DriveSceneOperator(_DriveOperator):
    """Base for operators that need an already-built Drive Scene."""

    @classmethod
    def poll(cls, context):
        return _scene(context) is not None and has_scene(context, DEFINITION)


#: every setting :class:`OPENCV_CAM_OT_drive_reset_defaults` puts back
_RESET_KEYS = (
    "aisle_length", "aisle_width", "bay_depth", "bay_width", "show_bays",
    "bay_numbers", "parked_cars", "pillar_count", "ground_texture",
    "light_energy", "shadows",
    "car_length", "car_width", "car_height", "car_clearance",
    "drive_distance", "drive_speed", "drive_accel", "drive_profile",
    "drive_fps", "drive_heading", "clip_quality", "clip_device",
)


class OPENCV_CAM_OT_drive_add_scene(_DriveOperator, bpy.types.Operator):
    """Create the Drive Scene (car park, minibus, front camera, keyframed drive)"""

    bl_idname = "opencv_cam.drive_add_scene"
    bl_label = "Drive Scene"
    bl_description = (
        "Create the Drive Scene: an indoor car park with lane markings, a "
        "minibus carrying the OpenCV front camera at its real mount pose, and "
        "the planned drive keyframed on the vehicle"
    )
    bl_options = {"REGISTER", "UNDO"}

    reset: BoolProperty(
        name="Reset", default=False,
        description="Rebuild from the defaults even if the scene already exists")

    def execute(self, context):
        scene = _scene(context)
        settings = _settings(context)
        if settings is None:
            self.report({"ERROR"}, "scene.drive_scene is not registered")
            return {"CANCELLED"}
        if settings.root is not None and not self.reset:
            self.report({"WARNING"}, "a Drive Scene already exists (use Rebuild or Reset)")
            return {"CANCELLED"}
        if settings.root is not None and self.reset:
            builder.remove(scene, settings)
        created = builder.build(scene, settings)
        view_mod.frame(view_mod.scene_objects(DEFINITION), DEFINITION.view)
        for message in created.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, f"Drive Scene ready: {drive_path.summary(created['plan'])}")
        return {"FINISHED"}


class OPENCV_CAM_OT_drive_rebuild(_DriveSceneOperator, bpy.types.Operator):
    """Rebuild the car park, the vehicle and the drive from the current settings"""

    bl_idname = "opencv_cam.drive_rebuild"
    bl_label = "Rebuild"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        created = controller.rebuild_now(context)
        if created is None:
            self.report({"WARNING"}, "no Drive Scene to rebuild")
            return {"CANCELLED"}
        for message in created.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, f"Drive Scene rebuilt: {drive_path.summary(created['plan'])}")
        return {"FINISHED"}


class OPENCV_CAM_OT_drive_reset_defaults(_DriveSceneOperator, bpy.types.Operator):
    """Put every Drive Scene setting back to its default"""

    bl_idname = "opencv_cam.drive_reset_defaults"
    bl_label = "Reset Defaults"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        for name in _RESET_KEYS:
            settings.property_unset(name)
        controller.rebuild_now(context)
        self.report({"INFO"}, "Drive Scene defaults restored")
        return {"FINISHED"}


class OPENCV_CAM_OT_drive_remove_scene(_DriveSceneOperator, bpy.types.Operator):
    """Remove the Drive Scene (deletes its objects; the panels disappear)"""

    bl_idname = "opencv_cam.drive_remove_scene"
    bl_label = "Remove Drive Scene"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        removed = builder.remove(_scene(context), _settings(context))
        self.report({"INFO"}, f"Drive Scene removed ({removed} objects)")
        return {"FINISHED"}


class OPENCV_CAM_OT_drive_export_zip(_DriveSceneOperator, bpy.types.Operator, ExportHelper):
    """Export the whole clip - PNG sequence, video, CSV and JSON - as one zip"""

    bl_idname = "opencv_cam.drive_export_zip"
    bl_label = "Export Clip"
    bl_description = (
        "Render the whole drive and pack it into one zip: frame_%04d.png, "
        "clip.mp4 (H.264), frames.csv (per-frame speed, vehicle and camera pose) "
        "and clip.json (K / D, mount pose, drive and render parameters). The "
        "render cost comes from the Clip Quality setting and the device from "
        "Clip Device; the render resolution is Blender's own Render ▸ Output and "
        "the scene's render settings are restored afterwards. In the UI the "
        "export renders frame by frame with a progress bar, an ETA and ESC to "
        "cancel; in background mode it runs synchronously"
    )
    bl_options = {"REGISTER"}

    filename_ext = ".zip"
    filter_glob: StringProperty(default="*.zip", options={"HIDDEN"})
    samples: IntProperty(
        name="Samples", default=0, min=0, max=4096,
        description="Override the Clip Quality sample count; 0 uses the quality "
                    "preset as-is")

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

    # -- background / script path (synchronous) -----------------------------
    def _run_sync(self, context, settings, plan):
        try:
            report = recording.export_zip(context, settings, self.filepath,
                                          samples=self.samples)
        except Exception as exc:
            settings.clip_status = f"error: {exc}"
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        return self._report_result(settings, plan, report)

    # -- UI path (modal, one frame per timer tick) --------------------------
    def _start_modal(self, context, settings, plan):
        try:
            job = recording.ClipJob(context, settings, filepath=self.filepath,
                                    samples=self.samples, encode=True, pack=True)
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
                                f"({drive_path.summary(plan)})")
        for message in report.get("messages") or []:
            self.report({"WARNING"}, message)
        self.report({"INFO"}, settings.clip_status)
        return {"FINISHED"}


def _redraw_ui(context) -> None:
    """Repaint every window so the panel's progress label updates during a modal run."""
    try:
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                area.tag_redraw()
    except Exception:
        pass


_CLASSES = (
    OPENCV_CAM_OT_drive_add_scene,
    OPENCV_CAM_OT_drive_rebuild,
    OPENCV_CAM_OT_drive_reset_defaults,
    OPENCV_CAM_OT_drive_remove_scene,
    OPENCV_CAM_OT_drive_export_zip,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
