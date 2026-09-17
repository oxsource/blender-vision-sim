"""Operators: the only place where user-visible actions are defined.

Operators stay thin: they resolve the context, call the ``bl`` helpers and turn
their return values into reports.
"""

from __future__ import annotations

import os

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper, ImportHelper

from ..core import calibration_io, presets
from . import apply as apply_mod
from . import camera_factory, preview, scene_builder, selftest, shader


def _camera_object(context):
    """Resolve the camera to work on.

    Prefers the active object when it is a camera, otherwise falls back to
    ``context.camera`` (the camera shown in Object Data Properties) so the panel
    operators keep working while some other object is selected.
    """
    obj = context.active_object
    if obj is not None and obj.type == "CAMERA":
        return obj
    for candidate in (getattr(context, "camera", None),
                      getattr(getattr(context, "scene", None), "camera", None)):
        if candidate is not None and candidate.type == "CAMERA":
            return candidate
    return None


class _CameraOperator:
    """Mixin providing shared context handling and reporting."""

    @classmethod
    def poll(cls, context):
        return _camera_object(context) is not None

    def camera(self, context):
        obj = _camera_object(context)
        if obj is None:
            raise RuntimeError("no camera in context")
        return obj, obj.data


#: enum items for the Add Camera operator.  A *list* (not a callable) so the
#: default can be a string identifier; with callable items bpy requires an index.
MODEL_ITEMS = [(key, value[0], "") for key, value in camera_factory.MODELS.items()]


def _preset_items(self, context):
    """Enum items: bundled presets plus "copy the active camera"."""
    items = [(presets.CURRENT, "Current Camera Settings",
              "Copy the intrinsics/distortion of the active camera")]
    for identifier, label in presets.list_presets():
        items.append((identifier, label, f"presets/{identifier}"))
    if len(items) == 1:
        items.append(("", "-- no preset files --", ""))
    return items


def _report_messages(operator, messages, level="INFO"):
    for message in messages:
        operator.report({level}, message)


# ---------------------------------------------------------------------------
class OPENCV_CAM_OT_apply(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.apply_settings"
    bl_label = "Apply"
    bl_description = (
        "Install/compile the OSL shader and copy the intrinsics and distortion "
        "into the camera's Cycles custom parameters"
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        settings.status = "ok" if ok else "error"
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"OpenCV camera applied (fx={settings.intrinsics.fx:.2f}, "
            f"distortion {'on' if settings.distortion.enabled else 'off'})",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_install_shader(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.install_shader"
    bl_label = "Install / Refresh Shader"
    bl_description = "Write the bundled OSL shader into a Text data-block and compile it"
    bl_options = {"REGISTER", "UNDO"}

    force: BoolProperty(name="Force", default=False, description="Overwrite local edits")

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        text, changed = shader.install_shader(settings, force=self.force)
        shader.attach(cam_data, settings)
        ok, messages = shader.ensure_compiled(cam_data)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, f"shader '{text.name}' {'updated' if changed else 'unchanged'} and compiled")
        return {"FINISHED"}


class OPENCV_CAM_OT_import_calibration(_CameraOperator, bpy.types.Operator, ImportHelper):
    bl_idname = "opencv_cam.import_calibration"
    bl_label = "Import Calibration"
    bl_description = "Load intrinsics and distortion from an OpenCV/ROS/Kalibr calibration file"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ""
    filter_glob: StringProperty(default="*.yaml;*.yml;*.json;*.xml", options={"HIDDEN"})
    camera_name: StringProperty(
        name="Camera",
        description="Camera to pick from a multi-camera config (empty = first enabled)",
        default="",
    )

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        try:
            calibration = calibration_io.load_calibration(
                self.filepath, camera_name=self.camera_name or None
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.calibration.filepath = self.filepath
        settings.calibration.last_import = os.path.basename(self.filepath)
        note = ""
        if not calibration.model_name:
            note = (" (no distortion_model in the file: coefficients were read as "
                    f"{calibration.distortion.model} - pick Fisheye in the panel if this "
                    "is a wide angle / AVM lens)")
        self.report(
            {"INFO"},
            f"{os.path.basename(self.filepath)}: {calibration.width}x{calibration.height}, "
            f"fx={calibration.intrinsics.fx:.2f}, model={calibration.distortion.model}{note}",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_export_calibration(_CameraOperator, bpy.types.Operator, ExportHelper):
    bl_idname = "opencv_cam.export_calibration"
    bl_label = "Export Calibration"
    bl_description = "Write the current intrinsics and distortion to a calibration file"
    bl_options = {"REGISTER"}

    filename_ext = ".yaml"
    filter_glob: StringProperty(default="*.yaml;*.yml;*.json", options={"HIDDEN"})

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        calibration = calibration_io.Calibration(
            intrinsics=apply_mod.effective_intrinsics(
                settings, settings.intrinsics.image_width, settings.intrinsics.image_height
            ),
            distortion=settings.core_distortion(),
        )
        try:
            path = calibration_io.save_calibration(self.filepath, calibration)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"written {path}")
        return {"FINISHED"}


class OPENCV_CAM_OT_sync_from_lens(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.sync_from_lens"
    bl_label = "From Blender Lens"
    bl_description = "Derive fx, fy and the principal point from lens/sensor/shift"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        messages = apply_mod.sync_intrinsics_from_lens(cam_data, settings, context.scene)
        _report_messages(self, messages)
        return {"FINISHED"}


class OPENCV_CAM_OT_apply_pose(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.apply_pose"
    bl_label = "Apply Pose to Object"
    bl_description = "Set the camera object transform from the OpenCV world-to-camera pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, cam_data = self.camera(context)
        apply_mod.apply_opencv_pose(obj, cam_data.opencv_cam)
        self.report({"INFO"}, f"{obj.name}: transform set from OpenCV R/t")
        return {"FINISHED"}


class OPENCV_CAM_OT_read_pose(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.read_pose"
    bl_label = "Read Pose from Object"
    bl_description = "Store the current camera transform as an OpenCV world-to-camera pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        rotation, translation = apply_mod.read_opencv_pose(obj, settings)
        settings.pose.rotation = rotation
        settings.pose.translation = translation
        self.report({"INFO"}, "pose read from the camera object")
        return {"FINISHED"}


class OPENCV_CAM_OT_add_camera(bpy.types.Operator):
    """Create a camera that already uses the OpenCV lens model"""

    bl_idname = "opencv_cam.add_camera"
    bl_label = "OpenCV Camera"
    bl_description = (
        "Add a camera configured as Cycles Custom with an OpenCV lens model "
        "(fisheye / Brown-Conrady / rational / pinhole)"
    )
    bl_options = {"REGISTER", "UNDO"}

    model: EnumProperty(name="Model", items=MODEL_ITEMS, default="fisheye")
    preset: EnumProperty(
        name="Preset",
        description="Start from a bundled calibration or copy the active camera",
        items=_preset_items,
    )
    use_rig: BoolProperty(
        name="Add Rig Empty",
        description="Parent the camera to an empty, handy for extrinsics/multi-camera setups",
        default=False,
    )
    at_cursor: BoolProperty(
        name="At 3D Cursor",
        description="Place the camera at the 3D cursor instead of the world origin",
        default=True,
    )

    def execute(self, context):
        scene = context.scene
        location = tuple(scene.cursor.location) if self.at_cursor else (0.0, 0.0, 0.0)
        preset = None if self.preset == presets.CURRENT else self.preset
        try:
            camera, messages = camera_factory.add_camera(
                scene, model=self.model, preset=preset, use_rig=self.use_rig, location=location
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        _report_messages(self, messages)
        self.report({"INFO"}, f"added {camera.name} ({camera_factory.MODELS[self.model][0]})")
        return {"FINISHED"}


class OPENCV_CAM_OT_load_preset(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.load_preset"
    bl_label = "Load Preset"
    bl_description = "Load a bundled preset calibration into this camera"
    bl_options = {"REGISTER", "UNDO"}

    preset: EnumProperty(name="Preset", items=_preset_items)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        if self.preset == presets.CURRENT:
            self.report({"WARNING"}, "pick a preset file")
            return {"CANCELLED"}
        try:
            calibration = presets.load_preset(self.preset)
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.calibration.last_import = self.preset
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, f"preset {self.preset} loaded ({calibration.intrinsics.fx:.2f} px)")
        return {"FINISHED"}


class OPENCV_CAM_OT_reset_defaults(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.reset_defaults"
    bl_label = "Reset Defaults"
    bl_description = "Restore the add-on default intrinsics and distortion coefficients"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        from . import properties as props
        intrinsics = props.AVM_FRONT_INTRINSICS
        coeffs = props.AVM_FRONT_DISTORTION
        settings.intrinsics.auto_center = False
        settings.intrinsics.fx = intrinsics["fx"]
        settings.intrinsics.fy = intrinsics["fy"]
        settings.intrinsics.cx = intrinsics["cx"]
        settings.intrinsics.cy = intrinsics["cy"]
        settings.intrinsics.image_width = intrinsics["image_width"]
        settings.intrinsics.image_height = intrinsics["image_height"]
        settings.intrinsics.scale_to_render = True
        settings.distortion.model = "fisheye"
        settings.distortion.enabled = True
        settings.distortion.k1, settings.distortion.k2, settings.distortion.k3, settings.distortion.k4 = coeffs
        settings.distortion.k5 = settings.distortion.k6 = 0.0
        settings.distortion.p1 = settings.distortion.p2 = 0.0
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report({"INFO"}, "defaults restored (reference AVM front camera)")
        return {"FINISHED"}


class OPENCV_CAM_OT_set_render_resolution(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.set_render_resolution"
    bl_label = "Set Render Resolution"
    bl_description = "Write the output size into Blender's render resolution now"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        changed, messages = apply_mod.apply_render_resolution(context.scene, settings)
        _report_messages(self, messages)
        width, height = apply_mod.output_resolution(settings, context.scene)
        if settings.auto_apply:
            ok, apply_messages = apply_mod.apply_values(cam_data, settings, context.scene)
            _report_messages(self, apply_messages, "INFO" if ok else "ERROR")
            if not ok:
                return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"render resolution set to {context.scene.render.resolution_x}"
            f"x{context.scene.render.resolution_y}"
            + ("" if changed else " (already correct)")
            + f"; output {width}x{height}",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_read_scene_resolution(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.read_scene_resolution"
    bl_label = "From Scene"
    bl_description = "Take Blender's current render resolution as the output size"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        # read first: assigning width/height drives the scene resolution, so
        # reading them between assignments would pick up the new value
        width = context.scene.render.resolution_x
        height = context.scene.render.resolution_y
        settings.output.mode = "custom"
        settings.output.width = width
        settings.output.height = height
        settings.output.preset = "custom"
        self.report(
            {"INFO"},
            f"output size taken from the scene: {settings.output.width}x{settings.output.height}",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_recompile(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.recompile"
    bl_label = "Recompile Shader"
    bl_description = "Refresh the bundled OSL shader text and recompile it"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        _, changed = shader.install_shader(settings, force=False)
        cam_data.custom_bytecode = ""
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"{shader.shader_filename(settings.distortion.model)} recompiled "
            f"({'text updated' if changed else 'text unchanged'}, "
            f"{len(cam_data.custom_bytecode)} chars)",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_preview(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.preview"
    bl_label = "Preview"
    bl_description = (
        "Render a quick preview with the current camera and show it in the Image Editor "
        "(render settings are restored afterwards)"
    )
    bl_options = {"REGISTER"}

    size: EnumProperty(
        name="Size",
        items=[("256", "256 px", ""), ("384", "384 px", ""), ("512", "512 px", ""), ("720", "720 px", "")],
        default="384",
    )
    samples: IntProperty(name="Samples", default=16, min=1, max=512)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        settings.preview.size = self.size
        settings.preview.samples = self.samples
        result = preview.render_preview(
            cam_data, settings, context.scene,
            size_key=self.size, samples=self.samples, denoise=settings.preview.denoise,
        )
        _report_messages(self, result["messages"], "INFO" if result["ok"] else "ERROR")
        if not result["ok"]:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"preview rendered at {result['resolution'][0]}x{result['resolution'][1]} "
            f"({result['samples']} samples)",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_save_preview(_CameraOperator, bpy.types.Operator, ExportHelper):
    bl_idname = "opencv_cam.save_preview"
    bl_label = "Save Preview Image"
    bl_description = "Render a preview and write it to a PNG file"
    bl_options = {"REGISTER"}

    filename_ext = ".png"
    filter_glob: StringProperty(default="*.png", options={"HIDDEN"})

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        try:
            path = preview.save_preview_image(
                cam_data, settings, context.scene,
                size_key=settings.preview.size, samples=max(settings.preview.samples, 16),
                filepath=self.filepath,
            )
        except Exception as exc:
            self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"written {path}")
        return {"FINISHED"}


class OPENCV_CAM_OT_add_test_scene(bpy.types.Operator):
    """Create a checker cube/ground/lights in front of the camera"""

    bl_idname = "opencv_cam.add_test_scene"
    bl_label = "Test Scene"
    bl_description = (
        "Create a checker cube, ground grid and lights in front of the camera, "
        "apply the current intrinsics and set up Cycles - then press F12 to look "
        "at the distortion. Adds a fisheye camera first when the scene has none"
    )
    bl_options = {"REGISTER", "UNDO"}

    distance: FloatProperty(name="Distance", default=4.0, min=0.5, max=100.0)
    samples: IntProperty(name="Samples", default=64, min=1, max=4096)

    @classmethod
    def poll(cls, context):
        return getattr(context, "scene", None) is not None

    def execute(self, context):
        obj = _camera_object(context)
        if obj is None:
            try:
                obj, messages = camera_factory.add_camera(context.scene, model="fisheye")
            except Exception as exc:
                self.report({"ERROR"}, f"{type(exc).__name__}: {exc}")
                return {"CANCELLED"}
            _report_messages(self, messages)
            self.report({"INFO"}, f"no camera in the scene: added {obj.name} first")
        ok, messages, created = scene_builder.apply_and_build(
            obj, context.scene, samples=self.samples
        )
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        self.report(
            {"INFO"},
            f"test scene ready ({len(created)} objects, "
            f"{context.scene.render.resolution_x}x{context.scene.render.resolution_y}) - press F12",
        )
        return {"FINISHED"}


class OPENCV_CAM_OT_add_rig_empty(bpy.types.Operator):
    """Add an empty to use as a camera rig and parent the active camera to it"""

    bl_idname = "opencv_cam.add_rig_empty"
    bl_label = "Camera Rig (Empty)"
    bl_description = (
        "Add an empty at the 3D cursor and parent the active camera to it, handy for "
        "extrinsics and multi-camera setups"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return getattr(context, "scene", None) is not None

    def execute(self, context):
        scene = context.scene
        obj = _camera_object(context)
        name = f"{obj.name}_rig" if obj is not None else "CameraRig"
        rig = bpy.data.objects.new(name, None)
        rig.empty_display_type = "PLAIN_AXES"
        rig.empty_display_size = 0.5
        rig.location = obj.location if obj is not None else tuple(scene.cursor.location)
        scene.collection.objects.link(rig)
        if obj is not None:
            obj.parent = rig
            obj.matrix_parent_inverse = rig.matrix_world.inverted()
        for candidate in scene.objects:
            candidate.select_set(candidate is rig)
        bpy.context.view_layer.objects.active = rig
        self.report({"INFO"}, f"added {rig.name}" + (f", {obj.name} parented to it" if obj else ""))
        return {"FINISHED"}


class OPENCV_CAM_OT_selftest(_CameraOperator, bpy.types.Operator):
    bl_idname = "opencv_cam.self_test"
    bl_label = "Run Self Test"
    bl_description = (
        "Render a test target and verify the image position against the OpenCV "
        "model (catches stale or broken shaders)"
    )
    bl_options = {"REGISTER"}

    resolution: IntProperty(name="Resolution", default=256, min=32, max=2048)
    samples: IntProperty(name="Samples", default=4, min=1, max=1024)
    tolerance_px: FloatProperty(name="Tolerance (px)", default=1.0, min=0.01, max=50.0)

    def execute(self, context):
        _, cam_data = self.camera(context)
        settings = cam_data.opencv_cam
        ok, messages = apply_mod.apply_settings(cam_data, settings, context.scene)
        _report_messages(self, messages, "INFO" if ok else "ERROR")
        if not ok:
            return {"CANCELLED"}
        try:
            result = selftest.run(
                cam_data,
                settings,
                context.scene,
                resolution=self.resolution,
                samples=self.samples,
                tolerance_px=self.tolerance_px,
            )
        except (selftest.SelfTestError, RuntimeError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        status = "PASSED" if result["passed"] else "FAILED"
        settings.status = f"selftest {status.lower()} ({result['error_px']:.3f} px)"
        self.report(
            {"INFO"} if result["passed"] else {"WARNING"},
            f"self test {status}: predicted {result['predicted'][0]:.2f},{result['predicted'][1]:.2f} "
            f"measured {result['measured'][0]:.3f},{result['measured'][1]:.3f} "
            f"error {result['error_px']:.3f} px (tolerance {result['tolerance_px']:.2f})",
        )
        return {"FINISHED"} if result["passed"] else {"CANCELLED"}


_CLASSES = (
    OPENCV_CAM_OT_apply,
    OPENCV_CAM_OT_add_camera,
    OPENCV_CAM_OT_add_rig_empty,
    OPENCV_CAM_OT_load_preset,
    OPENCV_CAM_OT_reset_defaults,
    OPENCV_CAM_OT_set_render_resolution,
    OPENCV_CAM_OT_read_scene_resolution,
    OPENCV_CAM_OT_recompile,
    OPENCV_CAM_OT_preview,
    OPENCV_CAM_OT_save_preview,
    OPENCV_CAM_OT_add_test_scene,
    OPENCV_CAM_OT_install_shader,
    OPENCV_CAM_OT_import_calibration,
    OPENCV_CAM_OT_export_calibration,
    OPENCV_CAM_OT_sync_from_lens,
    OPENCV_CAM_OT_apply_pose,
    OPENCV_CAM_OT_read_pose,
    OPENCV_CAM_OT_selftest,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)