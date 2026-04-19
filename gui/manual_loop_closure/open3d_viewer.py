from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from PyQt5 import QtCore, QtGui, QtWidgets

DEFAULT_TOP_DOWN_YAW_DEG = -90.0
DEFAULT_TOP_DOWN_PITCH_DEG = 89.0

INTERACTION_MODE_CAMERA = "camera"
INTERACTION_MODE_EDIT_SOURCE = "edit_source"

LOCK_MODE_XY_YAW = "xy_yaw"
LOCK_MODE_Z_ONLY = "z_only"
LOCK_MODE_XZ_PITCH = "xz_pitch"
LOCK_MODE_YZ_ROLL = "yz_roll"

EDIT_OPERATION_TRANSLATE = "translate"
EDIT_OPERATION_ROTATE = "rotate"

VIEW_PRESET_TOP = "top"
VIEW_PRESET_SIDE_X = "side_x"
VIEW_PRESET_SIDE_Y = "side_y"


@dataclass(frozen=True)
class PreviewScene:
    target_points: np.ndarray
    trajectory_points: Optional[np.ndarray] = None
    trajectory_values: Optional[np.ndarray] = None
    selected_target_trajectory_points: Optional[np.ndarray] = None
    editable_source_points_local: Optional[np.ndarray] = None
    editable_source_transform: Optional[np.ndarray] = None
    initial_source_points: Optional[np.ndarray] = None
    adjusted_source_points: Optional[np.ndarray] = None
    final_source_points: Optional[np.ndarray] = None
    transform_world_target: Optional[np.ndarray] = None
    transform_world_source_initial: Optional[np.ndarray] = None
    transform_world_source_adjusted: Optional[np.ndarray] = None
    transform_world_source_final: Optional[np.ndarray] = None


@dataclass(frozen=True)
class CameraState:
    center: np.ndarray
    distance: float
    yaw_deg: float
    pitch_deg: float


@dataclass(frozen=True)
class ManualAlignUpdate:
    transform_world_source: np.ndarray
    phase: str
    lock_mode: str


def _point_cloud(points: np.ndarray, colors: Optional[np.ndarray] = None) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    if points.size:
        cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    if colors is not None and colors.size:
        cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64, copy=False))
    return cloud


def _point_material(
    color: tuple[float, float, float, float],
    point_size: float,
) -> o3d.visualization.rendering.MaterialRecord:
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.base_color = color
    material.point_size = point_size
    return material


def _mesh_material() -> o3d.visualization.rendering.MaterialRecord:
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultLit"
    return material


def _axis_mesh(transform: np.ndarray, size: float) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    mesh.transform(transform)
    return mesh


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    rotated = points @ transform[:3, :3].T
    return rotated + transform[:3, 3]


def _normalize_values(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return values
    min_value = float(np.min(values))
    max_value = float(np.max(values))
    if max_value - min_value < 1e-9:
        return np.zeros_like(values, dtype=np.float64)
    return (values - min_value) / (max_value - min_value)


def _trajectory_colors(values: np.ndarray) -> np.ndarray:
    normalized = _normalize_values(values.astype(np.float64, copy=False))
    if normalized.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    anchors = np.asarray(
        [
            [0.08, 0.25, 0.78],
            [0.00, 0.65, 0.86],
            [0.16, 0.80, 0.44],
            [0.99, 0.85, 0.23],
            [0.93, 0.35, 0.20],
        ],
        dtype=np.float64,
    )
    scaled = normalized * (anchors.shape[0] - 1)
    lower = np.floor(scaled).astype(np.int32)
    upper = np.clip(lower + 1, 0, anchors.shape[0] - 1)
    weight = (scaled - lower).reshape(-1, 1)
    return anchors[lower] * (1.0 - weight) + anchors[upper] * weight


class EmbeddedOpen3DWidget(QtWidgets.QWidget):
    manual_align_changed = QtCore.pyqtSignal(object)
    manual_align_status = QtCore.pyqtSignal(str)
    camera_preset_changed = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(280, 220)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        self._renderer: Optional[o3d.visualization.rendering.OffscreenRenderer] = None
        self._rendered_image: Optional[QtGui.QImage] = None
        self._scene: Optional[PreviewScene] = None
        self._scene_center = np.zeros(3, dtype=np.float32)
        self._scene_extent = 1.0

        self._camera_center = np.zeros(3, dtype=np.float32)
        self._camera_distance = 5.0
        self._camera_yaw_deg = DEFAULT_TOP_DOWN_YAW_DEG
        self._camera_pitch_deg = DEFAULT_TOP_DOWN_PITCH_DEG
        self._camera_preset = VIEW_PRESET_TOP
        self._camera_ready = False

        self._display_mode = "preview"
        self._trajectory_point_size = 8.0
        self._target_point_size = 1.0
        self._source_point_size = 3.0
        self._show_world_axis = False
        self._interaction_mode = INTERACTION_MODE_CAMERA
        self._align_lock_mode = LOCK_MODE_XY_YAW
        self._snap_view_enabled = True
        self._translation_step_m = 0.05
        self._rotation_step_deg = 1.0
        self._edit_operation = EDIT_OPERATION_TRANSLATE
        self._drag_mode: Optional[str] = None
        self._last_mouse_pos = QtCore.QPoint()
        self._active_source_transform: Optional[np.ndarray] = None
        self._edit_drag_active = False
        self._edit_start_mouse_pos = QtCore.QPoint()
        self._edit_start_source_transform: Optional[np.ndarray] = None
        self._edit_current_source_transform: Optional[np.ndarray] = None

    def clear_scene(self) -> None:
        self._scene = None
        self._active_source_transform = None
        self._edit_drag_active = False
        self._edit_start_source_transform = None
        self._edit_current_source_transform = None
        if self._renderer is not None:
            self._renderer.scene.clear_geometry()
        self._rendered_image = None
        self.update()

    def shutdown(self) -> None:
        self._scene = None
        self._rendered_image = None
        if self._renderer is not None:
            self._renderer.scene.clear_geometry()
        self._renderer = None
        self._camera_ready = False
        self._active_source_transform = None
        self._edit_drag_active = False
        self._edit_start_source_transform = None
        self._edit_current_source_transform = None

    def capture_camera_state(self) -> Optional[CameraState]:
        if not self._camera_ready:
            return None
        return CameraState(
            center=self._camera_center.astype(np.float32, copy=True),
            distance=float(self._camera_distance),
            yaw_deg=float(self._camera_yaw_deg),
            pitch_deg=float(self._camera_pitch_deg),
        )

    def restore_camera_state(self, state: Optional[CameraState]) -> None:
        if state is None:
            return
        self._camera_center = state.center.astype(np.float32, copy=True)
        self._camera_distance = max(0.3, float(state.distance))
        self._camera_yaw_deg = float(state.yaw_deg)
        self._camera_pitch_deg = max(-89.0, min(89.0, float(state.pitch_deg)))
        self._set_camera_preset(self._preset_from_angles(self._camera_yaw_deg, self._camera_pitch_deg))
        self._camera_ready = True

    def _preset_from_angles(self, yaw_deg: float, pitch_deg: float) -> str:
        presets = {
            VIEW_PRESET_TOP: (-90.0, 89.0),
            VIEW_PRESET_SIDE_Y: (-90.0, 0.0),
            VIEW_PRESET_SIDE_X: (0.0, 0.0),
        }
        for preset, (preset_yaw, preset_pitch) in presets.items():
            if abs(yaw_deg - preset_yaw) < 1e-6 and abs(pitch_deg - preset_pitch) < 1e-6:
                return preset
        return ""

    def _set_camera_preset(self, preset: str) -> None:
        normalized = preset if preset in {VIEW_PRESET_TOP, VIEW_PRESET_SIDE_Y, VIEW_PRESET_SIDE_X} else ""
        if normalized == self._camera_preset:
            return
        self._camera_preset = normalized
        self.camera_preset_changed.emit(normalized)

    def set_interaction_mode(self, mode: str) -> None:
        normalized = mode.strip().lower()
        if normalized not in {INTERACTION_MODE_CAMERA, INTERACTION_MODE_EDIT_SOURCE}:
            normalized = INTERACTION_MODE_CAMERA
        previous_mode = self._interaction_mode
        self._interaction_mode = normalized
        if normalized == INTERACTION_MODE_CAMERA:
            self._edit_drag_active = False
            self.setCursor(QtCore.Qt.ArrowCursor)
        else:
            self.setCursor(QtCore.Qt.OpenHandCursor)
            if self._snap_view_enabled and previous_mode != normalized:
                self.apply_lock_view_preset()
        self._emit_align_status()

    def set_manual_align_options(
        self,
        *,
        lock_mode: str,
        edit_operation: str,
        translation_step_m: float,
        rotation_step_deg: float,
        snap_view: bool,
    ) -> None:
        normalized = lock_mode.strip().lower()
        if normalized not in {LOCK_MODE_XY_YAW, LOCK_MODE_Z_ONLY, LOCK_MODE_XZ_PITCH, LOCK_MODE_YZ_ROLL}:
            normalized = LOCK_MODE_XY_YAW
        previous_lock = self._align_lock_mode
        previous_snap = self._snap_view_enabled
        self._align_lock_mode = normalized
        if edit_operation not in {EDIT_OPERATION_TRANSLATE, EDIT_OPERATION_ROTATE}:
            edit_operation = EDIT_OPERATION_TRANSLATE
        if self._align_lock_mode == LOCK_MODE_Z_ONLY:
            self._edit_operation = EDIT_OPERATION_TRANSLATE
        else:
            self._edit_operation = edit_operation
        self._translation_step_m = max(0.001, float(translation_step_m))
        self._rotation_step_deg = max(0.1, float(rotation_step_deg))
        self._snap_view_enabled = bool(snap_view)
        if (
            self._interaction_mode == INTERACTION_MODE_EDIT_SOURCE
            and self._snap_view_enabled
            and (previous_lock != normalized or previous_snap != self._snap_view_enabled)
        ):
            self.apply_lock_view_preset()
        self._emit_align_status()

    def set_view_preset(self, preset: str) -> None:
        normalized = preset.strip().lower()
        if normalized == VIEW_PRESET_TOP:
            yaw_deg, pitch_deg = -90.0, 89.0
        elif normalized == VIEW_PRESET_SIDE_Y:
            yaw_deg, pitch_deg = -90.0, 0.0
        elif normalized == VIEW_PRESET_SIDE_X:
            yaw_deg, pitch_deg = 0.0, 0.0
        else:
            return

        self._camera_yaw_deg = yaw_deg
        self._camera_pitch_deg = pitch_deg
        self._set_camera_preset(normalized)
        self._camera_ready = True
        if self._scene is not None and self._renderer is not None:
            self._apply_camera()
            self._render_now()

    def apply_lock_view_preset(self) -> None:
        if self._align_lock_mode == LOCK_MODE_XY_YAW:
            self.set_view_preset(VIEW_PRESET_TOP)
        elif self._align_lock_mode == LOCK_MODE_Z_ONLY:
            self.set_view_preset(VIEW_PRESET_SIDE_Y)
        elif self._align_lock_mode == LOCK_MODE_XZ_PITCH:
            self.set_view_preset(VIEW_PRESET_SIDE_Y)
        elif self._align_lock_mode == LOCK_MODE_YZ_ROLL:
            self.set_view_preset(VIEW_PRESET_SIDE_X)

    def set_display_options(
        self,
        *,
        display_mode: str,
        trajectory_point_size: float,
        target_point_size: float,
        source_point_size: float,
        show_world_axis: bool,
        reset_camera: bool = False,
    ) -> None:
        display_mode = display_mode.lower().strip()
        if display_mode not in {"preview", "final", "compare"}:
            display_mode = "preview"
        current_camera = self.capture_camera_state()
        self._display_mode = display_mode
        self._trajectory_point_size = float(trajectory_point_size)
        self._target_point_size = max(0.5, float(target_point_size))
        self._source_point_size = max(0.5, float(source_point_size))
        self._show_world_axis = bool(show_world_axis)
        if self._scene is None:
            return
        self._ensure_renderer()
        self._upload_scene()
        if reset_camera or not self._camera_ready:
            self.reset_camera()
        else:
            self.restore_camera_state(current_camera)
            self._apply_camera()
            self._render_now()

    def _editable_source_transform(self, scene: PreviewScene) -> Optional[np.ndarray]:
        if self._edit_current_source_transform is not None and self._interaction_mode == INTERACTION_MODE_EDIT_SOURCE:
            return self._edit_current_source_transform
        if scene.editable_source_transform is not None:
            return scene.editable_source_transform
        if scene.transform_world_source_adjusted is not None:
            return scene.transform_world_source_adjusted
        return scene.transform_world_source_initial

    def update_scene(
        self,
        scene: PreviewScene,
        *,
        reset_camera: bool = False,
        camera_state: Optional[CameraState] = None,
    ) -> None:
        self._scene = scene
        if not self._edit_drag_active:
            self._edit_current_source_transform = None
        if camera_state is None:
            camera_state = self.capture_camera_state()
        self._ensure_renderer()
        self._upload_scene()
        if reset_camera or camera_state is None:
            self.reset_camera()
        else:
            self.restore_camera_state(camera_state)
            self._apply_camera()
            self._render_now()

    def reset_camera(self) -> None:
        center = self._scene_center.astype(np.float32, copy=True)
        extent = max(float(self._scene_extent), 1.0)
        self._camera_center = center
        self._camera_distance = max(2.5 * extent, 3.0)
        self._camera_yaw_deg = DEFAULT_TOP_DOWN_YAW_DEG
        self._camera_pitch_deg = DEFAULT_TOP_DOWN_PITCH_DEG
        self._set_camera_preset(VIEW_PRESET_TOP)
        self._camera_ready = True
        self._apply_camera()
        self._render_now()

    def _can_edit_source(self) -> bool:
        return (
            self._interaction_mode == INTERACTION_MODE_EDIT_SOURCE
            and self._scene is not None
            and self._active_source_transform is not None
        )

    def _modifier_scale(self, modifiers: QtCore.Qt.KeyboardModifiers) -> float:
        scale = 1.0
        if modifiers & QtCore.Qt.ControlModifier:
            scale *= 0.2
        if modifiers & QtCore.Qt.ShiftModifier:
            scale *= 5.0
        return scale

    def _lock_basis_vectors(self) -> tuple[np.ndarray, np.ndarray]:
        if self._align_lock_mode == LOCK_MODE_Z_ONLY:
            return (
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
            )
        if self._align_lock_mode == LOCK_MODE_XZ_PITCH:
            return (
                np.array([1.0, 0.0, 0.0], dtype=np.float64),
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
            )
        if self._align_lock_mode == LOCK_MODE_YZ_ROLL:
            return (
                np.array([0.0, 1.0, 0.0], dtype=np.float64),
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
            )
        return (
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
        )

    def _project_to_lock_plane(self, vector: np.ndarray) -> np.ndarray:
        projected = np.asarray(vector, dtype=np.float64).copy()
        if self._align_lock_mode == LOCK_MODE_Z_ONLY:
            projected[:] = (0.0, 0.0, 1.0)
        elif self._align_lock_mode == LOCK_MODE_XY_YAW:
            projected[2] = 0.0
        elif self._align_lock_mode == LOCK_MODE_XZ_PITCH:
            projected[1] = 0.0
        elif self._align_lock_mode == LOCK_MODE_YZ_ROLL:
            projected[0] = 0.0
        norm = np.linalg.norm(projected)
        if norm < 1e-6:
            return projected
        return projected / norm

    def _drag_translation(self, delta: QtCore.QPoint, modifiers: QtCore.Qt.KeyboardModifiers) -> np.ndarray:
        if self._align_lock_mode == LOCK_MODE_Z_ONLY:
            pixels_per_step = 10.0
            scale = (self._translation_step_m * self._modifier_scale(modifiers)) / pixels_per_step
            return np.array([0.0, 0.0, -float(delta.y()) * scale], dtype=np.float64)
        right_vec, up_vec = self._camera_basis()
        right_locked = self._project_to_lock_plane(right_vec)
        up_locked = self._project_to_lock_plane(up_vec)
        fallback_right, fallback_up = self._lock_basis_vectors()
        if np.linalg.norm(right_locked) < 1e-6:
            right_locked = fallback_right
        if np.linalg.norm(up_locked) < 1e-6:
            up_locked = fallback_up
        pixels_per_step = 10.0
        scale = (self._translation_step_m * self._modifier_scale(modifiers)) / pixels_per_step
        return (
            right_locked * (float(delta.x()) * scale)
            + up_locked * (-float(delta.y()) * scale)
        )

    def _rotation_axis_name(self) -> str:
        if self._align_lock_mode == LOCK_MODE_Z_ONLY:
            return "z"
        if self._align_lock_mode == LOCK_MODE_XZ_PITCH:
            return "y"
        if self._align_lock_mode == LOCK_MODE_YZ_ROLL:
            return "x"
        return "z"

    def _drag_rotation_deg(
        self,
        delta: QtCore.QPoint,
        modifiers: QtCore.Qt.KeyboardModifiers,
    ) -> float:
        pixels_per_step = 12.0
        return (
            float(delta.x()) / pixels_per_step
            * self._rotation_step_deg
            * self._modifier_scale(modifiers)
        )

    def _apply_manual_rotation(
        self,
        transform_world_source: np.ndarray,
        angle_deg: float,
    ) -> np.ndarray:
        rotated = transform_world_source.copy()
        rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, 0.0))
        axis_name = self._rotation_axis_name()
        if axis_name == "x":
            rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz((math.radians(angle_deg), 0.0, 0.0))
        elif axis_name == "y":
            rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz((0.0, math.radians(angle_deg), 0.0))
        else:
            rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, math.radians(angle_deg)))
        rotated[:3, :3] = rotation_delta @ rotated[:3, :3]
        return rotated

    def _emit_manual_align(self, transform_world_source: np.ndarray, phase: str) -> None:
        self.manual_align_changed.emit(
            ManualAlignUpdate(
                transform_world_source=transform_world_source.astype(np.float64, copy=True),
                phase=phase,
                lock_mode=self._align_lock_mode,
            )
        )
        self._emit_align_status()

    def _emit_align_status(self) -> None:
        if self._interaction_mode != INTERACTION_MODE_EDIT_SOURCE:
            self.manual_align_status.emit("View mode · wheel zoom")
            return
        lock_text = {
            LOCK_MODE_XY_YAW: "XY + Yaw",
            LOCK_MODE_Z_ONLY: "Z only",
            LOCK_MODE_XZ_PITCH: "XZ + Pitch",
            LOCK_MODE_YZ_ROLL: "YZ + Roll",
        }.get(self._align_lock_mode, "XY + Yaw")
        if self._align_lock_mode == LOCK_MODE_XY_YAW:
            axis_text = (
                "source only · drag yaw · wheel zoom"
                if self._edit_operation == EDIT_OPERATION_ROTATE
                else "source only · drag XY · wheel zoom"
            )
        elif self._align_lock_mode == LOCK_MODE_Z_ONLY:
            axis_text = "source only · drag Z · wheel zoom"
        elif self._align_lock_mode == LOCK_MODE_XZ_PITCH:
            axis_text = "source only · drag XZ · wheel zoom"
        else:
            axis_text = "source only · drag YZ · wheel zoom"
        self.manual_align_status.emit(
            f"Edit {lock_text} · {axis_text}"
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        if self._renderer is not None and self._scene is not None and self._camera_ready:
            self._apply_camera()
            self._render_now()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._can_edit_source():
            if event.button() == QtCore.Qt.LeftButton and not (event.modifiers() & QtCore.Qt.AltModifier):
                self._drag_mode = (
                    "edit_rotate"
                    if (
                        self._align_lock_mode == LOCK_MODE_XY_YAW
                        and self._edit_operation == EDIT_OPERATION_ROTATE
                    )
                    else "edit_translate"
                )
                self._edit_drag_active = True
                self._edit_start_mouse_pos = event.pos()
                base_transform = self._active_source_transform
                self._edit_start_source_transform = (
                    None if base_transform is None else base_transform.astype(np.float64, copy=True)
                )
                self._edit_current_source_transform = self._edit_start_source_transform
            if self._edit_start_source_transform is not None:
                self.setCursor(QtCore.Qt.ClosedHandCursor)
                self._emit_manual_align(self._edit_start_source_transform, "start")
            elif event.button() == QtCore.Qt.RightButton:
                self._drag_mode = "pan"
            elif event.button() == QtCore.Qt.MiddleButton or (
                event.button() == QtCore.Qt.LeftButton and (event.modifiers() & QtCore.Qt.AltModifier)
            ):
                self._drag_mode = "orbit"
            else:
                self._drag_mode = None
        elif event.button() == QtCore.Qt.LeftButton:
            self._drag_mode = "orbit"
        elif event.button() == QtCore.Qt.RightButton:
            self._drag_mode = "pan"
        else:
            self._drag_mode = None
        self._last_mouse_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._renderer is None or self._drag_mode is None or self._scene is None:
            super().mouseMoveEvent(event)
            return

        if self._drag_mode == "edit_translate":
            if self._edit_start_source_transform is None:
                super().mouseMoveEvent(event)
                return
            total_delta = event.pos() - self._edit_start_mouse_pos
            edited = self._edit_start_source_transform.copy()
            edited[:3, 3] += self._drag_translation(total_delta, event.modifiers())
            self._edit_current_source_transform = edited
            self._active_source_transform = edited
            self._emit_manual_align(edited, "update")
            self._upload_scene()
            self._apply_camera()
            self._render_now()
            super().mouseMoveEvent(event)
            return
        if self._drag_mode == "edit_rotate":
            if self._edit_start_source_transform is None:
                super().mouseMoveEvent(event)
                return
            total_delta = event.pos() - self._edit_start_mouse_pos
            edited = self._apply_manual_rotation(
                self._edit_start_source_transform,
                self._drag_rotation_deg(total_delta, event.modifiers()),
            )
            self._edit_current_source_transform = edited
            self._active_source_transform = edited
            self._emit_manual_align(edited, "update")
            self._upload_scene()
            self._apply_camera()
            self._render_now()
            super().mouseMoveEvent(event)
            return

        delta = event.pos() - self._last_mouse_pos
        self._last_mouse_pos = event.pos()
        if self._drag_mode == "orbit":
            self._camera_yaw_deg -= delta.x() * 0.35
            self._camera_pitch_deg += delta.y() * 0.35
            self._camera_pitch_deg = max(-89.0, min(89.0, self._camera_pitch_deg))
            self._set_camera_preset(self._preset_from_angles(self._camera_yaw_deg, self._camera_pitch_deg))
        elif self._drag_mode == "pan":
            right_vec, up_vec = self._camera_basis()
            pan_scale = max(self._camera_distance, 1.0) * 0.0015
            self._camera_center += (-delta.x() * pan_scale) * right_vec.astype(np.float32)
            self._camera_center += (delta.y() * pan_scale) * up_vec.astype(np.float32)

        self._apply_camera()
        self._render_now()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._drag_mode in {"edit_translate", "edit_rotate"} and self._edit_current_source_transform is not None:
            self._emit_manual_align(self._edit_current_source_transform, "end")
        if self._interaction_mode == INTERACTION_MODE_EDIT_SOURCE:
            self.setCursor(QtCore.Qt.OpenHandCursor)
        self._edit_drag_active = False
        self._edit_start_source_transform = None
        self._drag_mode = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:  # noqa: N802
        if self._renderer is None or self._scene is None:
            super().wheelEvent(event)
            return

        delta_steps = event.angleDelta().y() / 120.0
        if delta_steps != 0.0:
            self._camera_distance *= math.pow(0.9, delta_steps)
            self._camera_distance = max(0.3, self._camera_distance)
            self._apply_camera()
            self._render_now()
        super().wheelEvent(event)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(12, 12, 14))
        if self._rendered_image is not None:
            painter.drawImage(self.rect(), self._rendered_image)
        else:
            painter.setPen(QtGui.QColor(180, 180, 180))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "Point cloud preview")
        painter.end()
        super().paintEvent(event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    def _ensure_renderer(self) -> None:
        target_size = self.size()
        if target_size.width() <= 0 or target_size.height() <= 0:
            return
        if self._renderer is not None:
            return

        self._renderer = o3d.visualization.rendering.OffscreenRenderer(
            max(target_size.width(), 1),
            max(target_size.height(), 1),
        )
        self._renderer.scene.set_background([0.04, 0.04, 0.04, 1.0])

    def _visible_geometry(self, scene: PreviewScene) -> tuple[list[dict], Optional[np.ndarray]]:
        visible_points: list[dict] = []
        if scene.trajectory_points is not None and scene.trajectory_points.size:
            values = scene.trajectory_values
            if values is None or values.size != scene.trajectory_points.shape[0]:
                values = np.arange(scene.trajectory_points.shape[0], dtype=np.float64)
            visible_points.append(
                {
                    "name": "trajectory",
                    "points": scene.trajectory_points,
                    "colors": _trajectory_colors(values),
                    "point_size": max(1.0, self._trajectory_point_size),
                    "base_color": (1.0, 1.0, 1.0, 1.0),
                }
            )

        if (
            scene.selected_target_trajectory_points is not None
            and scene.selected_target_trajectory_points.size
        ):
            highlight_colors = np.tile(
                np.asarray([[1.0, 0.92, 0.18]], dtype=np.float64),
                (scene.selected_target_trajectory_points.shape[0], 1),
            )
            visible_points.append(
                {
                    "name": "target_trajectory_selected",
                    "points": scene.selected_target_trajectory_points,
                    "colors": highlight_colors,
                    "point_size": max(4.0, self._trajectory_point_size + 2.0),
                    "base_color": (1.0, 1.0, 1.0, 1.0),
                }
            )

        visible_points.append(
            {
                "name": "target",
                "points": scene.target_points,
                "colors": None,
                "point_size": self._target_point_size,
                "base_color": (0.65, 0.65, 0.65, 1.0),
            }
        )

        active_source_transform = None

        preview_points = scene.adjusted_source_points
        preview_transform = scene.transform_world_source_adjusted
        preview_color = (0.10, 0.70, 0.90, 1.0)
        if preview_points is None:
            preview_points = scene.initial_source_points
            preview_transform = scene.transform_world_source_initial
            preview_color = (1.00, 0.55, 0.05, 1.0)

        editable_transform = self._editable_source_transform(scene)
        if scene.editable_source_points_local is not None and editable_transform is not None:
            preview_points = _transform_points(scene.editable_source_points_local, editable_transform)
            preview_transform = editable_transform

        if self._display_mode == "preview":
            if preview_points is not None:
                visible_points.append(
                        {
                            "name": "source_preview",
                            "points": preview_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": preview_color,
                        }
                    )
                active_source_transform = preview_transform
        elif self._display_mode == "final":
            if scene.final_source_points is not None:
                visible_points.append(
                        {
                            "name": "source_final",
                            "points": scene.final_source_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": (0.20, 0.90, 0.30, 1.0),
                        }
                    )
                active_source_transform = scene.transform_world_source_final
            elif preview_points is not None:
                visible_points.append(
                        {
                            "name": "source_preview",
                            "points": preview_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": preview_color,
                        }
                    )
                active_source_transform = preview_transform
        else:
            if scene.final_source_points is not None:
                if preview_points is not None:
                    visible_points.append(
                        {
                            "name": "source_compare_preview",
                            "points": preview_points,
                            "colors": None,
                            "point_size": max(0.5, self._source_point_size - 1.0),
                            "base_color": (*preview_color[:3], 0.55),
                        }
                    )
                visible_points.append(
                        {
                            "name": "source_compare_final",
                            "points": scene.final_source_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": (0.20, 0.90, 0.30, 1.0),
                        }
                    )
                active_source_transform = scene.transform_world_source_final
            elif scene.initial_source_points is not None and scene.adjusted_source_points is not None:
                visible_points.append(
                        {
                            "name": "source_initial",
                            "points": scene.initial_source_points,
                            "colors": None,
                            "point_size": max(0.5, self._source_point_size - 1.0),
                            "base_color": (1.00, 0.55, 0.05, 0.55),
                        }
                    )
                visible_points.append(
                        {
                            "name": "source_adjusted",
                            "points": scene.adjusted_source_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": (0.10, 0.70, 0.90, 1.0),
                        }
                    )
                active_source_transform = scene.transform_world_source_adjusted
            elif preview_points is not None:
                visible_points.append(
                        {
                            "name": "source_preview",
                            "points": preview_points,
                            "colors": None,
                            "point_size": self._source_point_size,
                            "base_color": preview_color,
                        }
                    )
                active_source_transform = preview_transform

        return visible_points, active_source_transform

    def _upload_scene(self) -> None:
        if self._renderer is None:
            return
        self._renderer.scene.clear_geometry()
        self._active_source_transform = None
        if self._scene is None:
            self._scene_center = np.zeros(3, dtype=np.float32)
            self._scene_extent = 1.0
            return

        scene = self._scene
        visible_points, active_source_transform = self._visible_geometry(scene)
        self._active_source_transform = (
            None if active_source_transform is None else active_source_transform.astype(np.float64, copy=True)
        )
        bounds_chunks = []

        for geometry in visible_points:
            points = geometry["points"]
            if points.size == 0:
                continue
            self._renderer.scene.add_geometry(
                geometry["name"],
                _point_cloud(points, geometry["colors"]),
                _point_material(geometry["base_color"], geometry["point_size"]),
            )
            bounds_chunks.append(points)

        transforms = []
        if self._show_world_axis:
            transforms.append(np.eye(4, dtype=np.float64))
        if scene.transform_world_target is not None:
            transforms.append(scene.transform_world_target)
        if active_source_transform is not None:
            transforms.append(active_source_transform)
        for transform in transforms:
            bounds_chunks.append(transform[:3, 3].reshape(1, 3))

        if bounds_chunks:
            all_points = np.concatenate(bounds_chunks, axis=0)
            min_bound = np.min(all_points, axis=0)
            max_bound = np.max(all_points, axis=0)
            extent = np.maximum(max_bound - min_bound, 0.1)
            self._scene_center = ((min_bound + max_bound) * 0.5).astype(np.float32)
            self._scene_extent = float(np.max(extent))
        else:
            self._scene_center = np.zeros(3, dtype=np.float32)
            self._scene_extent = 1.0

        axis_size = max(0.35, min(2.0, 0.14 * self._scene_extent))
        if self._show_world_axis:
            self._add_axis("axis_world", np.eye(4, dtype=np.float64), axis_size * 0.8)
        if scene.transform_world_target is not None:
            self._add_axis("axis_target", scene.transform_world_target, axis_size)
        if active_source_transform is not None:
            self._add_axis("axis_source", active_source_transform, axis_size)

    def _add_axis(self, name: str, transform: np.ndarray, size: float) -> None:
        if self._renderer is None:
            return
        self._renderer.scene.add_geometry(
            name,
            _axis_mesh(transform, size),
            _mesh_material(),
        )

    def _camera_basis(self) -> tuple[np.ndarray, np.ndarray]:
        yaw = math.radians(self._camera_yaw_deg)
        pitch = math.radians(self._camera_pitch_deg)
        forward = np.array(
            [
                -math.cos(pitch) * math.cos(yaw),
                -math.cos(pitch) * math.sin(yaw),
                -math.sin(pitch),
            ],
            dtype=np.float64,
        )
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        right /= np.linalg.norm(right)
        camera_up = np.cross(right, forward)
        camera_up /= np.linalg.norm(camera_up)
        return right, camera_up

    def _camera_eye(self) -> np.ndarray:
        yaw = math.radians(self._camera_yaw_deg)
        pitch = math.radians(self._camera_pitch_deg)
        eye_offset = np.array(
            [
                math.cos(pitch) * math.cos(yaw),
                math.cos(pitch) * math.sin(yaw),
                math.sin(pitch),
            ],
            dtype=np.float32,
        )
        return self._camera_center + self._camera_distance * eye_offset

    def _apply_camera(self) -> None:
        if self._renderer is None:
            return

        width = max(self.width(), 1)
        height = max(self.height(), 1)
        aspect_ratio = width / float(height)
        near_plane = max(0.01, self._camera_distance * 0.01)
        far_plane = max(50.0, self._camera_distance * 20.0 + self._scene_extent * 10.0)
        self._renderer.scene.camera.set_projection(
            60.0,
            aspect_ratio,
            near_plane,
            far_plane,
            o3d.visualization.rendering.Camera.FovType.Vertical,
        )
        _, camera_up = self._camera_basis()
        self._renderer.scene.camera.look_at(
            self._camera_center.astype(np.float32),
            self._camera_eye().astype(np.float32),
            camera_up.astype(np.float32),
        )

    def _render_now(self) -> None:
        if self._renderer is None:
            return
        image = self._renderer.render_to_image()
        array = np.asarray(image)
        if array.size == 0:
            self._rendered_image = None
            self.update()
            return
        array = np.ascontiguousarray(array)
        qimage = QtGui.QImage(
            array.data,
            array.shape[1],
            array.shape[0],
            array.strides[0],
            QtGui.QImage.Format_RGB888,
        )
        self._rendered_image = qimage.copy()
        self.update()
